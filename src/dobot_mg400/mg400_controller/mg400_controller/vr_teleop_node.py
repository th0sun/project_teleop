#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🎬 MG400 VR Teleop Node - Main Entry Point
ประกอบทุกโมดูลเข้าด้วยกันและรัน ROS Node

การใช้งาน:
    ros2 run mg400_vr_controller teleop_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray, Int64, Int32, Bool
import threading
import numpy as np
import time
import json
import os
import re
from dataclasses import dataclass, field
from typing import Any, Optional

# Import configuration
from mg400_controller.common.config.robot_config import (
    ENABLE_GET_ERROR,
    JOINT_LIMITS,
    ELBOW_ANGLE_LIMIT
)
import mg400_controller.common.config.motion_config as motion_config

# Vendor protocol surface (do not hardcode MG400 ASCII strings here).
from mg400_protocol.dashboard import clear_error, get_tool

# Import core modules
from mg400_controller.common.core.robot_connection import RobotConnection
from mg400_controller.common.core.feedback_handler import FeedbackHandler
from mg400_controller.common.core.command_sender import CommandSender

# Import logic modules
from mg400_controller.common.logic.joint_validator import JointValidator
from mg400_controller.common.logic.motion_planner import MotionPlanner
from mg400_controller.common.logic.target_compensator import TargetLatencyCompensator

# Import utilities
from mg400_controller.common.utils.cli.interactive_cmd import InteractiveCommandHandler
from mg400_controller.common.utils.error.error_handler import ErrorHandler
from mg400_controller.common.utils.collision_haptic import CollisionHaptic
from mg400_controller.common.trajectory.trajectory_recorder import TrajectoryRecorder
from mg400_controller.common.trajectory.teach_job_handler import TeachJobHandler
from mg400_controller.common.logic.safety_monitor import SafetyMonitor
from mg400_controller.common.logic.scene_safety_guard import SceneSafetyGuard
from mg400_controller.common.logic.teleop_controller import TeleopController
from mg400_controller.common.utils.telemetry.latency_analyzer import LatencyAnalyzer
from mg400_controller.common.utils.clock_calibrator import ClockCalibrator
from mg400_controller.common.utils.cli.mode_selection import select_control_mode
from mg400_controller.common.utils.loggers.unified_triple_logger import (
    UnifiedTripleLogger,
    prompt_enable_triple_logging,
)
from mg400_controller.common.utils.pending_command_registry import (
    PendingCommand,
    PendingCommandRegistry,
)
from mg400_controller.common.teleop.tool_handlers import ToolCommandHandlers
from mg400_controller.common.teleop.sample_matcher import UnitySampleMatcher
from mg400_controller.common.teleop.unity_session import UnitySessionHandler
from mg400_controller.common.ros.teleop_interfaces import (
    create_publishers,
    create_subscriptions,
    attach_unity_subscription,
)
from mg400_controller.common.ros.topic_config import declare_topic_parameters
from mg400_controller.common.ros.unity_teleop_sample import (
    build_ros_joint_cmd_rx_sample,
)

# =========================
# ===== MAIN NODE ========
# =========================

@dataclass
class _ControlLoopContext:
    """Per-tick snapshot of every value the 50 Hz loop reuses across
    phases.

    The control loop used to be a single 493-line method that snapshotted
    a dozen locals at the top of the tick and then mutated / read them in
    13 different phases. Pulling those phases into helper methods would
    have meant either a 12-argument signature or (worse) helpers calling
    feedback.get_*() again, which can shift mid-tick and silently change
    correlation behaviour.

    This dataclass is the explicit handoff between phases. Each field is
    written exactly once (either at tick start or by the phase named in
    the comment), and downstream phases only read it.
    """

    # ── Tick-start snapshots (written by _build_loop_context) ──
    q_current: np.ndarray
    now: float                          # perf_counter — monotonic, for deltas
    now_wall: float                     # time.time — wall, for log alignment
    target_snapshot: np.ndarray
    unity_sample_snapshot: Any
    target_recv_time_snapshot: float
    target_recv_wall_snapshot: float
    unity_send_time_snapshot: float
    target_velocity_snapshot: Optional[np.ndarray]
    is_blocked: bool                    # playback / post-stop homing window
    operation_mode: str                 # "realtime" | "teach_repeat_playback"

    # ── Mid-tick outputs (written by tool-publish / mode-publish helpers) ──
    tool_act: Optional[np.ndarray] = None
    tool_tgt: Optional[np.ndarray] = None
    feedback_command_id: Optional[int] = None
    current_mode: int = 0
    velocity_mag: float = 0.0
    err_info: dict = field(default_factory=dict)
    session_logging_active: bool = False

class TeleopNode(Node):
    def __init__(self):
        super().__init__('mg400_vr_teleop')
        self.stop_event = threading.Event()
        self.connection = RobotConnection(self.get_logger())

        self._init_logic_modules()
        self._init_command_tracking_state()
        self._init_publishers_and_clock()
        self._declare_runtime_params()
        self._init_sample_worker()
        self._init_triple_logger()
        self._apply_mixed_primitives_param()
        self._register_subscriptions_with_qos()
        self._init_tool_state()

        if not self._connect_and_enable_robot():
            return

        self._init_robot_handlers()
        self._start_runtime()
        self._log_ready_banner()

    # ─────────────────────────────────────────────────────────────────────
    # __init__ phase helpers — see __init__ for the order they run in.
    # Each one is purely setup; the connect step is the one early-return
    # boundary and lives directly in __init__.
    # ─────────────────────────────────────────────────────────────────────

    def _init_logic_modules(self):
        """JointValidator, MotionPlanner, target compensator, optional
        adaptive telemetry, and the TeleopController that consumes them
        all.
        """
        import mg400_controller.common.config.robot_config as cfg
        self.validator = JointValidator(JOINT_LIMITS, ELBOW_ANGLE_LIMIT, self.get_logger())
        self.planner = MotionPlanner(cfg.CONTROL_MODE, self.get_logger())
        self.target_compensator = TargetLatencyCompensator(self.validator)

        # Adaptive realtime telemetry — kept separate from triple_logger
        # so post-hoc analysis of the new gate / per-cmd SpeedJ scheme
        # isn't mixed in with the production CSV.  Disabled cleanly if
        # the log dir can't be opened.
        self.adaptive_telemetry = None
        try:
            from mg400_controller.common.utils.telemetry.adaptive_telemetry import (
                AdaptiveTelemetry,
            )
            self.adaptive_telemetry = AdaptiveTelemetry()
            self.get_logger().info(
                f"📊 Adaptive telemetry: {self.adaptive_telemetry.path}"
            )
        except Exception as exc:
            self.get_logger().warn(f"Adaptive telemetry disabled: {exc}")

        # Teleop Controller (The Brain)
        self.controller = TeleopController(
            self.validator, self.planner, self.get_logger(),
            telemetry=self.adaptive_telemetry,
        )

    def _init_command_tracking_state(self):
        """All per-tick / per-command bookkeeping the loop reads from
        self.* — Unity sample maps, the pending-command registry, the
        active-command snapshot fields, and the ROS-side sequence
        counters.
        """
        self.latest_target = None
        self.sample_matcher = UnitySampleMatcher()
        self.unity_session = UnitySessionHandler(
            logger=self.get_logger(),
            stop_event=self.stop_event,
            sample_matcher=self.sample_matcher,
            get_triple_logger=lambda: self.triple_logger,
            on_session_changed=self._reset_pending_command_registry,
        )
        self.latest_target_unity_sample = None
        self.latest_target_recv_wall = 0.0
        self.active_command_unity_sample = None
        self.active_command_uid = None
        self.active_command_dobot_id = None
        self.active_command_response = None
        self.active_command_text = None
        self.active_feedback_command_id_before_send = None
        self.active_command_tracking_source = None
        self.active_command_tracking_confidence = None
        self.pending_command_registry = PendingCommandRegistry(max_size=100)
        self._last_unity_stale_warning = 0.0  # /unity/joint_cmd stale-drop rate-limit
        self.unity_joint_cmd_rx_session_id = f"ros_joint_cmd_rx_{int(time.time() * 1000)}"
        self._unity_joint_cmd_rx_seq = 0

    def _init_publishers_and_clock(self):
        """LatencyAnalyzer, ClockCalibrator, declared ROS topic names,
        the publisher bundle, and the 1 Hz RTT-heartbeat timer.
        """
        self.latency_analyzer = LatencyAnalyzer(motion_config)
        # Clock Synchronization (Triple-Lock) — drift estimated automatically.
        self.clock_calibrator = ClockCalibrator(window_size=50)

        self.topics = declare_topic_parameters(self)
        self.ros_publishers = create_publishers(self, topics=self.topics)
        # Level 3: RTT Heartbeat (ROS-side ping)
        self.create_timer(1.0, self._publish_heartbeat)

    def _declare_runtime_params(self):
        """Declare ROS params that influence behaviour at runtime.

        Today only ``use_unity_teleop_sample_for_control`` is exposed,
        as a tripwire: the parameter is still accepted so older launch
        files don't fail, but turning it on logs a loud warning. The
        live robot-control path is ``/unity/joint_cmd``;
        ``/unity/teleop_sample`` is log/sync only.
        """
        requested_sample_control = bool(
            self.declare_parameter("use_unity_teleop_sample_for_control", False).value
        )
        if requested_sample_control:
            self.get_logger().warn(
                "use_unity_teleop_sample_for_control is disabled in this "
                "build; ignoring. /unity/joint_cmd is the only live "
                "robot-control path."
            )

    def _init_sample_worker(self):
        """Per-loop timestamp contracts + counters, then start the
        Unity-session sample worker thread.

        Cross-layer timestamp contract:
          - T1/T2/T3/T4/T5 and CSV log timestamps are ROS-local wall
            seconds.
          - perf_counter is used only for control-loop intervals and
            stuck logic.
        """
        self.target_recv_time = 0.0  # T2: ROS receive wall time
        self.unity_send_time = 0.0   # T1: Unity send time calibrated into ROS wall time

        self._tool_query_counter = 0
        self._sample_counter = 0  # For session logger feedback decimation
        self._realtime_command_seq = 0
        self._realtime_speed_defaults_pending = False
        self._last_realtime_speed_defaults_attempt = 0.0

        # The sample queue + worker thread live on self.unity_session.
        self.unity_session.start_worker()

    def _reset_pending_command_registry(self):
        """Called by UnitySessionHandler when a new Unity session_id
        arrives. Reassigning is fine — older readers keep their
        reference until the next tick.
        """
        self.pending_command_registry = PendingCommandRegistry(max_size=100)

    def _init_triple_logger(self):
        """Ask the operator whether to enable the unified Unity→ROS2→Robot
        session log; create the logger if yes.
        """
        self.triple_logger = None
        if prompt_enable_triple_logging():
            self.triple_logger = UnifiedTripleLogger()
            self.get_logger().info(
                f"Teleop session logging enabled: {self.triple_logger.file_path}"
            )
        else:
            self.get_logger().info("Teleop session logging disabled")

    def _register_subscriptions_with_qos(self):
        """Declare the BEST_EFFORT QoS used for Unity inputs and wire up
        every subscription except the Unity firehose itself (that one is
        attached late, after handlers are ready, to avoid a race).

        Active inputs (full list of callback wiring lives in
        ``common/ros/teleop_interfaces.py:create_subscriptions``):
          unity_pong_callback        — Unity RTT heartbeat
          unity_teleop_sample        — Unity JSON trace protocol v1
          suction_callback           — Unity → vacuum gripper bool
          light_callback             — Unity → signal-light DO
          scene_safety_callback      — Unity → workspace-guard toggle
          dashboard_cmd_callback     — Unity → dashboard passthrough
          speed_factor_callback      — Unity → global SpeedFactor slider
          teach_job_request_callback — Unity → typed teach job request
        """
        self._unity_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.ros_subscriptions = self._register_ros_subscriptions()

    def _init_tool_state(self):
        """Last-known DO bitmask. Initialised before the robot connect
        step so the DO publish helper has something to diff against on
        the very first tick.

        The suction state machine itself now lives on ``self.tool_handlers``
        (see ``_init_robot_handlers``), which is created later because
        it needs the post-connect ``sender`` instance.
        """
        self.last_do_status = 0

    def _connect_and_enable_robot(self) -> bool:
        """Open the TCP connection and ENABLE the robot. Return True on
        success so __init__ can early-exit cleanly when there is no
        hardware reachable.
        """
        if not self.connection.connect():
            self.get_logger().error("Failed to connect to robot")
            return False
        self.connection.enable_robot()
        return True

    def _init_robot_handlers(self):
        """Everything that needs an open robot connection: feedback
        thread, command sender, optional GetError handler, haptic
        publisher, trajectory recorder, scene-safety guard, teach-job
        dispatcher, interactive CLI handler, and the 1 Hz safety
        monitor.
        """
        # PASS FEEDBACK HANDLER TO SENDER FOR SYNC
        self.feedback = FeedbackHandler(
            self.connection,
            self.ros_publishers.rviz,
            self.get_clock(),
            self.get_logger(),
            self.stop_event,
            feedback_callback=self._raw_feedback_packet_callback,
        )
        self.sender = CommandSender(self.connection, self.feedback, self.get_logger())

        # Unity tool / dashboard handlers — owns its own suction state
        # machine. Needs the sender, so it lives here (post-connect).
        self.tool_handlers = ToolCommandHandlers(
            connection=self.connection,
            sender=self.sender,
            logger=self.get_logger(),
            get_latest_target_fn=lambda: self.latest_target,
        )

        # ErrorHandler (GetError API) — optional, disabled for simulator.
        self.error_handler = None
        if ENABLE_GET_ERROR:
            self.error_handler = ErrorHandler(self.connection, self.get_logger())
            self.get_logger().info("✅ ErrorHandler enabled (GetError API)")
        else:
            self.get_logger().info(
                "⚠️  ErrorHandler disabled (set ENABLE_GET_ERROR=True for real robot)"
            )

        # Collision-based Haptic Feedback for Quest 3 VR.
        self.collision_haptic = CollisionHaptic(
            self.ros_publishers.haptic, self.get_logger()
        )

        # Trajectory Recorder (teach-and-repeat sequencer).
        self.trajectory_recorder = TrajectoryRecorder(
            command_send_fn=self.sender.send,
            logger=self.get_logger(),
            dashboard_send_fn=self.connection.send_dashboard_cmd,
            get_position_fn=self.feedback.get_current_position,
            get_robot_mode_fn=self.feedback.get_robot_mode,
            waypoint_callback=self._playback_waypoint_callback,
            target_callback=self._playback_target_callback,
        )
        self.trajectory_recorder.add_playback_event_callback(
            self._playback_lifecycle_callback
        )

        self.scene_safety_guard = SceneSafetyGuard(
            model_path=os.environ.get(
                "MG400_SCENE_SAFETY_MODEL", motion_config.SCENE_SAFETY_MODEL_PATH
            ),
            enabled=self._env_bool(
                "MG400_SCENE_SAFETY_ENABLED",
                motion_config.SCENE_SAFETY_ENABLED_DEFAULT,
            ),
            warn_distance_mm=motion_config.SCENE_SAFETY_WARN_DISTANCE_MM,
            logger=self.get_logger(),
        )
        self._last_scene_safety_block_log = 0.0

        # Job-request dispatcher for the production teach-repeat path.
        # Status / artifact replies go to /teach/job_status and
        # /teach/job_artifact.
        self.teach_job_handler = TeachJobHandler(
            recorder=self.trajectory_recorder,
            publish_status_fn=lambda payload: self.ros_publishers.teach_job_status.publish(
                String(data=payload)
            ),
            publish_artifact_fn=lambda payload: self.ros_publishers.teach_job_artifact.publish(
                String(data=payload)
            ),
            logger=self.get_logger(),
            allow_real_execute_fn=lambda: bool(self.connection.connected),
            scene_safety_guard=self.scene_safety_guard,
        )

        self.interactive = InteractiveCommandHandler(
            self.connection,
            self.get_logger(),
            self.stop_event,
        )

        self.safety_monitor = SafetyMonitor(
            self.ros_publishers.safety, self.get_logger(), self.error_handler
        )

    def _start_runtime(self):
        """Launch threads + timers that drive the live system: the
        feedback reader, the late Unity subscription (attached last on
        purpose to avoid racing with handler creation), the interactive
        CLI, the 50 Hz control-loop thread, and the 1 Hz / 20 Hz timers
        for safety and haptic publishing.
        """
        self.feedback.start()

        # Unity subscriber is attached last so the handlers above are
        # all ready when the firehose starts firing.
        self.ros_subscriptions.unity = attach_unity_subscription(
            self,
            self._unity_callback,
            self._unity_qos_profile,
            topics=self.topics,
        )

        self.get_logger().info("✅ Teleop Node fully initialized and listening.")
        self.interactive.start()

        # Control loop in its own high-precision thread (isolates from
        # ROS jitter / CPU load on the executor).
        self.control_loop_thread = threading.Thread(
            target=self._high_precision_control_loop, daemon=True
        )
        self.control_loop_thread.start()

        # Periodic timers
        self.create_timer(1.0, self.check_safety_status)        # safety monitor 1 Hz
        self.create_timer(0.05, self._publish_haptic_feedback)  # haptic 20 Hz

    def _log_ready_banner(self):
        """One-shot startup banner so the operator can confirm at a
        glance which gates, speed ranges, and stuck-time thresholds are
        live this session.
        """
        self.get_logger().info("✅ Teleop Node Ready")
        self.get_logger().info(f"🎓 Teach & Repeat: {self.topics.teach_job_request}")
        self.get_logger().info(
            "📊 Control Strategy: Adaptive Δ + Per-Cmd SpeedJ + Stuck Detection"
        )
        self.get_logger().info(
            f"📏 Adaptive gate: Δ "
            f"{np.degrees(motion_config.REALTIME_DELTA_MIN_RAD):.1f}°"
            f"-{np.degrees(motion_config.REALTIME_DELTA_MAX_RAD):.1f}°"
            f" | hand_vel "
            f"{np.degrees(motion_config.REALTIME_HAND_VEL_LOW_RAD_S):.0f}"
            f"-{np.degrees(motion_config.REALTIME_HAND_VEL_HIGH_RAD_S):.0f}°/s"
            f" | SpeedJ {motion_config.REALTIME_SPEEDJ_MIN}-{motion_config.REALTIME_SPEEDJ_MAX}%"
        )
        self.get_logger().info(
            f"⏱️  Stuck Time Threshold: {motion_config.STUCK_TIME_THRESHOLD:.1f} s"
        )

    @staticmethod
    def _env_bool(name, default=False):
        raw = os.environ.get(name)
        if raw is None:
            return bool(default)
        return raw.strip().lower() in ("1", "true", "yes", "on")

    def _scene_safety_callback(self, msg: Bool):
        enabled = bool(msg.data)
        self.scene_safety_guard.set_enabled(enabled)
        state = "enabled" if enabled else "disabled"
        loaded = "loaded" if self.scene_safety_guard.loaded else "no model"
        self.get_logger().warn(f"🧱 Scene safety {state} ({loaded})")

    def _publish_heartbeat(self):
        """Level 3: Send Ping to Unity to measure RTT"""
        msg = Int64()
        msg.data = int(self.get_clock().now().nanoseconds)
        self.ros_publishers.heartbeat.publish(msg)

    def _apply_mixed_primitives_param(self):
        """Read the ``use_mixed_primitives`` ROS param and apply it.

        Default value comes from ``motion_config.USE_MIXED_PRIMITIVES`` so
        a missing param keeps shipping behaviour.  The resolved value is
        written back to the module attribute because
        ``TrajectoryRecorder.compile_loaded_plan`` reads it via
        ``getattr(motion_config, "USE_MIXED_PRIMITIVES", False)``.

        Logged at INFO so the operator console makes the opt-in visible.
        """
        default = bool(getattr(motion_config, "USE_MIXED_PRIMITIVES", False))
        param = self.declare_parameter("use_mixed_primitives", default)
        value = bool(getattr(param, "value", default))
        motion_config.USE_MIXED_PRIMITIVES = value
        if value != default:
            self.get_logger().info(
                f"🔀 use_mixed_primitives overridden via ROS param: {value}"
            )
        else:
            self.get_logger().info(
                f"🔀 use_mixed_primitives = {value} (motion_config default)"
            )

    def _register_ros_subscriptions(self):
        """Group every ROS subscription into one factory call.

        Single source of truth for which Unity / monitor topics the node
        listens to.  Behaviour-identical wrapper around
        ``common.ros.teleop_interfaces.create_subscriptions``.  Reading
        this method tells you exactly which callbacks are wired without
        scrolling through the __init__ body.

        Production teach-repeat uses /teach/job_request only; old
        teach-status / raw-trajectory / JointTrajectory auto-play topics are
        intentionally not subscribed.
        """
        return create_subscriptions(
            self,
            unity_pong_callback=self.unity_session.pong_callback,
            unity_teleop_sample_callback=self.unity_session.sample_callback,
            # self.tool_handlers is created later (post-connect, inside
            # _init_robot_handlers because it needs the sender); use
            # lambdas to defer the attribute lookup until a message
            # actually arrives.
            suction_callback=lambda msg: self.tool_handlers.suction_callback(msg),
            light_callback=lambda msg: self.tool_handlers.light_callback(msg),
            scene_safety_callback=self._scene_safety_callback,
            dashboard_cmd_callback=lambda msg: self.tool_handlers.dashboard_cmd_callback(msg),
            speed_factor_callback=lambda msg: self.tool_handlers.speed_factor_callback(msg),
            teach_job_request_callback=self._teach_job_request_callback,
            topics=self.topics,
        )












    def _unity_callback(self, msg):
        """รับคำสั่งจาก Unity/VR - Store latest target only"""
        if not self.connection.connected or len(msg.position) < 4:
            return

        now_ros_sec = time.time()  # Use absolute time for sync and logging
        unity_send_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw_age_sec = now_ros_sec - unity_send_time_sec
        if (
            unity_send_time_sec > 0.0
            and 0.0 <= raw_age_sec <= 10.0
            and raw_age_sec > motion_config.UNITY_JOINT_CMD_STALE_DROP_SEC
        ):
            if now_ros_sec - self._last_unity_stale_warning > 0.5:
                self.get_logger().warn(
                    "Dropped stale /unity/joint_cmd before control "
                    f"age={raw_age_sec * 1000.0:.1f}ms"
                )
                self._last_unity_stale_warning = now_ros_sec
            return

        q_target = np.asarray(msg.position[:4], dtype=float)
        self._unity_joint_cmd_rx_seq += 1
        unity_sample = build_ros_joint_cmd_rx_sample(
            q_target.tolist(),
            session_id=self.unity_joint_cmd_rx_session_id,
            rx_seq_id=self._unity_joint_cmd_rx_seq,
            unity_send_ts=unity_send_time_sec,
            ros_recv_ts=now_ros_sec,
        )

        self._accept_joint_msg(msg, now_ros_sec, unity_sample=unity_sample, q_target=q_target)

    def _accept_joint_msg(self, msg, now_ros_sec, *, unity_sample=None, q_target=None):
        try:
            if q_target is None:
                q_target = np.array(msg.position)
            unity_send_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if unity_send_time_sec <= 0.0:
                unity_send_time_sec = now_ros_sec
            self._accept_unity_joint_target(
                q_target,
                unity_send_time_sec,
                now_ros_sec,
                unity_sample=unity_sample,
            )
        except Exception as e:
            self.get_logger().error(f"Error in _unity_callback: {e}")

    def _accept_unity_joint_target(
        self,
        q_target,
        unity_send_time_sec,
        now_ros_sec,
        *,
        unity_sample=None,
    ):
        # 1. Validate & Clamp Joints
        if np.any(np.isnan(q_target)):
            self.get_logger().warn("⚠️ Received NaN joints from Unity - ignoring command")
            return

        q_safe, was_clamped = self.validator.validate_and_clamp(q_target)

        if was_clamped:
            self.get_logger().warn("⚠️ Joint command exceeded limits - clamped to safe range", once=True)

        # --- 🕒 DYNAMIC CLOCK SYNCHRONIZATION (Triple-Lock) ---
        # Level 2 & 3: Filtered Min-Window + Drift Compensation
        corrected_unity_time = self.clock_calibrator.calibrate(unity_send_time_sec, now_ros_sec)

        # 3. Latency compensation based on measured Unity-to-ROS timing.
        q_compensated_safe = self.target_compensator.compensate(
            q_safe,
            corrected_unity_time,
            now_ros_sec,
        )

        # 📊 Publish Unity Input XYZ (FK of raw Unity joint angles, degrees)
        try:
            unity_xyz = self.feedback.kinematics.forward_kinematics(np.degrees(q_safe))
            xyz_msg = Float64MultiArray()
            xyz_msg.data = unity_xyz.tolist()
            self.ros_publishers.unity_xyz.publish(xyz_msg)
        except Exception:
            pass

        if self.triple_logger:
            self.triple_logger.log_unity_target(
                q_safe,
                q_compensated_safe,
                unity_send_time_sec,
                corrected_unity_time,
                now_ros_sec,
                unity_sample=unity_sample,
            )

        # 4. Update Latest Target (Do NOT send here - control_loop will decide when to send)
        if self._realtime_speed_defaults_pending:
            tr = self.trajectory_recorder
            playback_blocked = tr.is_playing or (time.perf_counter() < tr._block_until)
            if not playback_blocked:
                self._restore_realtime_speed_defaults("first_realtime_target")

        self.latest_target = q_compensated_safe      # Latency-compensated target
        self.latest_target_unity_sample = unity_sample
        self.latest_target_recv_wall = now_ros_sec
        self.target_recv_time = now_ros_sec          # T2: ROS receive time
        self.unity_send_time = corrected_unity_time  # T1: Calibrated Unity send time

    def _teach_job_request_callback(self, msg):
        """Dispatch a structured /teach/job_request payload via TeachJobHandler.

        The handler emits status updates on /teach/job_status and (for compile
        actions) the compiled artifact on /teach/job_artifact.  See
        ``teach_job_handler.py`` for the full schema.
        """
        try:
            payload = msg.data if isinstance(msg.data, str) else str(msg.data)
        except Exception:
            self.get_logger().error("teach_job_request: cannot read .data field")
            return

        payload_len = len(payload)
        action = None
        job_id = None
        try:
            envelope = json.loads(payload)
            if isinstance(envelope, dict):
                action = envelope.get("action")
                job_id = envelope.get("job_id")
        except Exception:
            pass

        if action:
            self.get_logger().info(
                f"teach_job_request received action={action!r} "
                f"job_id={job_id or ''!r} bytes={payload_len}"
            )
        else:
            snippet = payload[:160].replace("\n", " ")
            self.get_logger().warn(
                f"teach_job_request received unparsable envelope "
                f"bytes={payload_len} head={snippet!r}"
            )

        status = self.teach_job_handler.handle(payload)
        self.get_logger().info(
            f"teach_job_request handled stage={getattr(status, 'stage', '?')!r} "
            f"action={getattr(status, 'action', None)!r} "
            f"job_id={getattr(status, 'job_id', '')!r} "
            f"error={getattr(status, 'error_code', None)!r}"
        )

    def _reset_live_teleop_reference(self, reason: str):
        """Anchor live teleop at the robot's current pose after playback."""
        try:
            q_current = self.feedback.get_current_position()
        except Exception:
            q_current = None

        now_mono = time.perf_counter()
        now_wall = time.time()
        if q_current is None:
            self.latest_target = None
            self.controller.reset_reference(None, now=now_mono)
            try:
                self.target_compensator.reset()
            except AttributeError:
                pass
            self.get_logger().warn(
                f"⚠️  Live teleop reference cleared after {reason}; "
                "waiting for next Unity target"
            )
            return

        q_current = np.asarray(q_current[:4], dtype=float)
        self.latest_target = q_current.copy()
        self.controller.reset_reference(q_current, now=now_mono)
        try:
            self.target_compensator.reset(q_current, target_time=now_wall)
        except AttributeError:
            pass
        self.target_recv_time = now_wall
        self.unity_send_time = now_wall
        self.get_logger().info(
            f"🔁 Live teleop reference reset after {reason}: "
            f"{np.degrees(q_current).round(2).tolist()} deg"
        )

    def _playback_lifecycle_callback(self, event_name, payload):
        if event_name in {"playback_start", "playback_complete"}:
            self._reset_live_teleop_reference(event_name)
        if event_name == "playback_start":
            # Teach/repeat owns SpeedFactor while it is executing.  Do not let
            # a previously armed realtime reset overwrite a new teach run.
            self._realtime_speed_defaults_pending = False
        elif event_name == "playback_complete":
            # Keep the teach/repeat speed profile available for repeated runs.
            # Realtime speed is restored only when the next live Unity target
            # arrives, so the two speed domains stay explicit.
            self._realtime_speed_defaults_pending = True
            self.get_logger().info(
                "🏁 Teach/repeat playback complete; realtime SpeedFactor(100) "
                "is armed for the next /unity/joint_cmd target"
            )

    def _restore_realtime_speed_defaults(self, reason: str):
        """Make live teleop fast again after teach/repeat changed SpeedFactor."""
        if not self.connection.connected:
            self._realtime_speed_defaults_pending = True
            self.get_logger().warn(
                f"⚠️ Cannot restore realtime SpeedFactor after {reason}; robot disconnected"
            )
            return False
        ok = self.connection.set_realtime_speed_defaults()
        self._realtime_speed_defaults_pending = not ok
        if ok:
            self.get_logger().info(
                f"🏃 Realtime mode speed restored after {reason}: "
                "SpeedFactor/SpeedJ/AccJ = 100"
            )
        return ok

    def _playback_waypoint_callback(self, q_rad):
        """Called by TrajectoryRecorder when a waypoint is queued (sent to robot).
        Publishes ONLY to /teleop/sent_command.
        """
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.position = list(q_rad)
        self.ros_publishers.sent_command.publish(js)

    def _playback_target_callback(self, q_rad):
        """Called by TrajectoryRecorder at ~100Hz with the perfectly interpolated
        real-time target (equivalent to race.py's target line).
        Publishes to /teleop/playback_unity so the monitor draws the yellow target
        line accurately in real-time.
        """
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.position = list(q_rad)
        self.ros_publishers.playback_unity.publish(js)

        try:
            xyz = self.feedback.kinematics.forward_kinematics(np.degrees(q_rad))
            xyz_msg = Float64MultiArray()
            xyz_msg.data = xyz.tolist()
            self.ros_publishers.unity_xyz.publish(xyz_msg)
        except Exception:
            pass

    def _high_precision_control_loop(self):
        """
        Runs the control loop in a dedicated thread to avoid ROS executor jitter.
        Runs at 200Hz (5ms) so queue-aware decisions can react quickly while
        ROS callbacks stay responsive.
        """
        target_hz = 200.0
        period = 1.0 / target_hz
        next_time = time.perf_counter() + period

        while not self.stop_event.is_set():
            try:
                self._control_loop_step()
            except Exception as e:
                self.get_logger().error(f"Error in control loop: {e}")

            # Precision Sleep
            now = time.perf_counter()
            sleep_time = next_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)

            next_time += period
            # Prevent death spiral if severely lagging
            if time.perf_counter() > next_time + period:
                next_time = time.perf_counter() + period

    def _raw_feedback_packet_callback(self, sample: dict) -> None:
        """Log each accepted MG400 30004 feedback packet at packet cadence."""
        if self.triple_logger is None:
            return

        feedback_wall = float(sample["timestamp"])
        if not self.unity_session.session_logging_active(feedback_wall):
            return

        tr = getattr(self, "trajectory_recorder", None)
        is_blocked = bool(
            tr is not None and (tr.is_playing or (time.perf_counter() < tr._block_until))
        )
        operation_mode = "teach_repeat_playback" if is_blocked else "realtime"

        q_actual = np.asarray(sample["q_actual_rad"], dtype=float)
        q_target = np.asarray(sample["q_target_rad"], dtype=float)
        queue_backlog = float(np.max(np.abs(q_target[:4] - q_actual[:4])))

        self.triple_logger.log_robot_feedback(
            q_actual,
            ros_timestamp=feedback_wall,
            robot_mode=int(sample["robot_mode"]),
            error_status=int(sample["error_status"]),
            robot_tool_actual=sample["tool_vector_actual"],
            robot_tool_target=sample["tool_vector_target"],
            robot_feedback_command_id=int(sample["command_id"]),
            queue_backlog_rad=queue_backlog,
            run_queued_cmd=int(sample["run_queued_cmd"]),
            operation_mode=operation_mode,
            joints_are_degrees=False,
        )

    # ═════════════════════════════════════════════════════════════════════════
    # Control-loop phase helpers (extracted from _control_loop_step for
    # readability). Each one is a self-contained step inside the 50 Hz tick.
    # ═════════════════════════════════════════════════════════════════════════

    def _build_loop_context(self):
        """Capture every tick-stable value the loop's phases share.

        Returns None if the loop can't run this tick (no connection or
        no Unity target yet) so the caller can early-return.
        """
        if not (self.connection.connected and self.latest_target is not None):
            return None

        target_velocity_raw = self.target_compensator.target_velocity
        target_velocity_snapshot = (
            None
            if target_velocity_raw is None
            else np.asarray(target_velocity_raw, dtype=float).copy()
        )

        tr = self.trajectory_recorder
        is_blocked = tr.is_playing or (time.perf_counter() < tr._block_until)

        return _ControlLoopContext(
            q_current=self.feedback.get_current_position(),
            now=time.perf_counter(),
            now_wall=time.time(),
            target_snapshot=np.asarray(self.latest_target[:4], dtype=float).copy(),
            unity_sample_snapshot=self.latest_target_unity_sample,
            target_recv_time_snapshot=float(self.target_recv_time),
            target_recv_wall_snapshot=float(self.latest_target_recv_wall),
            unity_send_time_snapshot=float(self.unity_send_time),
            target_velocity_snapshot=target_velocity_snapshot,
            is_blocked=is_blocked,
            operation_mode="teach_repeat_playback" if is_blocked else "realtime",
        )

    def _snapshot_adaptive_telemetry(self, ctx):
        """Every 10th tick (~5 Hz at 50 Hz), enqueue a feedback snapshot
        so post-hoc analysis can correlate gate decisions with what the
        joints actually did. Enqueue is non-blocking; telemetry must
        never break the control loop.
        """
        if self.adaptive_telemetry is None or (self._sample_counter % 10) != 0:
            return
        try:
            qd = self.controller.robot_velocity
            q_target_fb = self.feedback.get_target_position()
            backlog = float(np.max(np.abs(np.asarray(q_target_fb) - ctx.q_current)))
            self.adaptive_telemetry.log_feedback(
                q_actual_rad=ctx.q_current,
                qd_rad_s=qd,
                backlog_rad=backlog,
                robot_mode=int(self.feedback.get_robot_mode()),
                note=ctx.operation_mode,
            )
        except Exception:
            pass

    def _trigger_smart_suction(self, ctx):
        """Delegate to the tool-handlers smart-suction check. Stays here
        as a thin wrapper so the control-loop body keeps reading as a
        sequence of node-level phases.
        """
        self.tool_handlers.maybe_fire_smart_suction(
            ctx.q_current, self.controller.stuck_start_time
        )

    def _publish_tool_vectors(self, ctx):
        """Publish actual/target tool poses + flange FK and stash the
        tool vectors on ``ctx`` so later phases reuse the same snapshot
        instead of re-reading the feedback handler (which can move
        between reads and break frame-by-frame correlation).
        """
        ctx.tool_act = self.feedback.get_tool_vector()
        ctx.tool_tgt = self.feedback.get_target_tool_vector()

        msg_act = Float64MultiArray()
        msg_act.data = ctx.tool_act.tolist()
        self.ros_publishers.tool_actual.publish(msg_act)

        msg_tgt = Float64MultiArray()
        msg_tgt.data = ctx.tool_tgt.tolist()
        self.ros_publishers.tool_target.publish(msg_tgt)

        # Flange actual = FK of actual joints (no tool offset).
        flange = self.feedback.get_flange_actual()
        msg_flange = Float64MultiArray()
        msg_flange.data = flange.tolist()
        self.ros_publishers.flange_actual.publish(msg_flange)

    def _publish_do_status(self):
        """Publish the robot's Digital Output bitmask and log transitions."""
        do_status = self.feedback.get_do_status()
        do_msg = Int64()
        do_msg.data = int(do_status)
        self.ros_publishers.do_status.publish(do_msg)

        if do_status != self.last_do_status:
            self.get_logger().info(
                f"📣 DO STATUS CHANGED: {bin(do_status)} (Hex: {hex(do_status)})"
            )
            self.last_do_status = do_status

    def _publish_robot_mode_error(self, ctx):
        """Publish robot_mode + error_status and stash on ctx everything
        downstream phases need: feedback_command_id, current_mode,
        err_info, velocity_mag, session_logging_active.
        """
        ctx.current_mode = int(self.feedback.get_robot_mode())
        ctx.feedback_command_id = self.feedback.get_command_id()
        mode_msg = Int32()
        mode_msg.data = ctx.current_mode
        self.ros_publishers.robot_mode.publish(mode_msg)

        ctx.err_info = self.feedback.get_error_status()
        err_msg = Int32()
        err_msg.data = int(ctx.err_info['error_status'])
        self.ros_publishers.error_status.publish(err_msg)
        ctx.velocity_mag = float(np.max(np.abs(self.controller.robot_velocity)))
        ctx.session_logging_active = self.unity_session.session_logging_active(ctx.now_wall)

    def _log_command_pass_samples(self, ctx):
        """Drain pending_command_registry pass-near samples into the
        triple-layer logger. Each sample represents a command whose
        target the robot has now physically passed (tool XYZ or joint
        norm within tolerance), letting us record settle metrics
        before the command formally completes.
        """
        if not (self.triple_logger and ctx.session_logging_active):
            return
        pass_samples = self.pending_command_registry.pass_samples(
            ctx.q_current,
            tool_actual=ctx.tool_act,
            now_wall=ctx.now_wall,
            norm_tolerance_rad=motion_config.COMMAND_PASS_NEAR_NORM_TOLERANCE_RAD,
            per_joint_tolerance_rad=motion_config.VALID_PER_JOINT_LIMIT,
            tool_xyz_tolerance_mm=motion_config.COMMAND_PASS_NEAR_TOOL_XYZ_TOLERANCE_MM,
            tool_r_tolerance_deg=motion_config.COMMAND_PASS_NEAR_TOOL_R_TOLERANCE_DEG,
        )
        for sample in pass_samples:
            command = sample.command
            feedback_before_send = command.feedback_command_id_before_send
            feedback_id_changed = (
                None
                if feedback_before_send is None
                else ctx.feedback_command_id != feedback_before_send
            )
            self.triple_logger.log_command_match(
                event_type=sample.event_type,
                control_command_seq=command.control_command_seq,
                ros_command_uid=command.ros_command_uid,
                dobot_command_id=command.dobot_command_id,
                robot_feedback_command_id=ctx.feedback_command_id,
                status=sample.status,
                dobot_command_text=command.command_text,
                command_tracking_source=sample.method,
                command_tracking_confidence=sample.confidence,
                feedback_command_id_before_send=feedback_before_send,
                feedback_command_id_at_result=ctx.feedback_command_id,
                feedback_command_id_changed=feedback_id_changed,
                settle_match_method=sample.method,
                settle_match_ambiguous=sample.candidate_count > 1,
                settle_candidate_count=sample.candidate_count,
                settle_match_error_rad=sample.joint_norm_error_rad,
                settle_match_age_ms=sample.match_age_ms,
                pending_command_count=sample.pending_count,
                ros_timestamp=ctx.now_wall,
                ros_cmd_joints=command.target_rad,
                robot_joints=ctx.q_current,
                ros_cmd_tool_target=command.target_tool,
                robot_tool_actual=ctx.tool_act,
                robot_tool_target=ctx.tool_tgt,
                final_error_rad=sample.joint_norm_error_rad,
                max_joint_error_rad=sample.joint_max_error_rad,
                velocity_mag_rad_s=ctx.velocity_mag,
                robot_mode=ctx.current_mode,
                error_status=ctx.err_info['error_status'],
                operation_mode=ctx.operation_mode,
                unity_sample=command.unity_sample,
            )

    def _periodic_tool_index_query(self):
        """Every ~5 s (every 250th 50 Hz tick) ask the robot for its
        currently-selected tool index and publish it. Cheap enough to
        skip silently on transport errors.
        """
        self._tool_query_counter += 1
        if self._tool_query_counter < 250:
            return
        self._tool_query_counter = 0
        try:
            resp = self.connection.send_and_wait(get_tool().render(), timeout=1.0)
            if resp:
                m = re.search(r'\{(\d+)\}', resp)
                if m:
                    ti_msg = Int32()
                    ti_msg.data = int(m.group(1))
                    self.ros_publishers.tool_index.publish(ti_msg)
        except Exception:
            pass

    def _auto_recover_from_error_mode(self, ctx):
        """If the robot has entered Mode 9 (error / limit hit), auto-clear
        the error at most every 3 seconds so the arm doesn't sit frozen.
        """
        if ctx.current_mode != 9:
            return
        if not hasattr(self, 'last_clear_error_time'):
            self.last_clear_error_time = 0.0
        if ctx.now - self.last_clear_error_time > 3.0:
            self.get_logger().error(
                "🛑 Robot is in ERROR STATE (Mode 9). Auto-clearing error..."
            )
            self.connection.send_and_wait(clear_error().render())
            self.last_clear_error_time = ctx.now

    def _make_controller_decision(self, ctx):
        """Run the queue-aware controller's should_send check and return
        ``(should_send, send_reason, queue_backlog_rad, run_queued_cmd)``.

        Pure delegation today — exists mainly so the control-loop body
        reads as a sequence of named phases instead of inlining the
        queue-aware logic where the eye skims past it.
        """
        queue_backlog_rad = self.feedback.get_queue_backlog()
        run_queued_cmd = self.feedback.get_run_queued_cmd()

        should_send, send_reason = self.controller.should_send_command(
            ctx.target_snapshot,
            ctx.q_current,
            now=ctx.now,
            queue_backlog_rad=queue_backlog_rad,
            run_queued_cmd=run_queued_cmd,
            target_velocity=ctx.target_velocity_snapshot,
        )
        return should_send, send_reason, queue_backlog_rad, run_queued_cmd

    def _track_motion_t4_t5(self, ctx):
        """Drive the latency analyzer's T4 (motion start) and T5
        (target reached) markers, and on T5 emit the latency-event +
        command-result rows to the triple logger.

        T5 is where most of the post-hoc analysis hangs off: it picks
        the best pending command by joint-norm match, falls back to
        the active command snapshot when no match was found, and
        decorates the analyzer's metrics dict with command identity,
        match metadata, and the operation mode the tick was in.
        """
        # T4: motion start
        self.latency_analyzer.update_tracking(ctx.velocity_mag)
        if ctx.velocity_mag > motion_config.MOTION_START_THRESHOLD:
            if self.latency_analyzer.mark_motion_start(ctx.now_wall):
                self.get_logger().debug(
                    f"Motion started: velocity={ctx.velocity_mag:.6f} rad/s"
                )

        # T5: target reached — only when the robot is both near the
        # last commanded target and effectively at rest.
        cmd_target = self.latency_analyzer.current_cmd_target
        dist = np.linalg.norm(ctx.q_current - cmd_target) if cmd_target is not None else 999
        is_stopped = ctx.velocity_mag < 0.005
        if not (dist < 0.01 and is_stopped):
            return
        if not self.latency_analyzer.mark_target_reached(ctx.now_wall):
            return

        metrics, report = self.latency_analyzer.analyze_arrival(
            ctx.q_current, ctx.velocity_mag
        )
        if not metrics:
            return

        self.get_logger().info(report)

        if not (self.triple_logger and ctx.session_logging_active):
            return

        match_result = self.pending_command_registry.match(
            ctx.q_current,
            feedback_command_id=ctx.feedback_command_id,
            now_wall=ctx.now_wall,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=motion_config.VALID_PER_JOINT_LIMIT,
        )
        matched_command = match_result.command
        if matched_command is not None:
            matched_seq = matched_command.control_command_seq
            matched_uid = matched_command.ros_command_uid
            matched_dobot_id = matched_command.dobot_command_id
            matched_command_text = matched_command.command_text
            matched_unity_sample = matched_command.unity_sample
            matched_target_rad = matched_command.target_rad
            matched_target_tool = matched_command.target_tool
            feedback_before_send = matched_command.feedback_command_id_before_send
        else:
            matched_seq = metrics.get("control_command_seq")
            matched_uid = self.active_command_uid
            matched_dobot_id = self.active_command_dobot_id
            matched_command_text = self.active_command_text
            matched_unity_sample = self.active_command_unity_sample
            matched_target_rad = metrics.get("target")
            matched_target_tool = metrics.get("ros_cmd_tool_target")
            feedback_before_send = self.active_feedback_command_id_before_send

        feedback_id_changed = (
            None
            if feedback_before_send is None
            else ctx.feedback_command_id != feedback_before_send
        )
        command_id_match = match_result.command_id_match
        command_result_status = match_result.status
        settle_match_method = match_result.method
        tracking_source = match_result.method
        tracking_confidence = match_result.confidence

        metrics["final_tool_actual"] = ctx.tool_act
        metrics["final_tool_target"] = ctx.tool_tgt
        metrics["operation_mode"] = ctx.operation_mode
        metrics["unity_sample"] = matched_unity_sample
        metrics["control_command_seq"] = matched_seq
        metrics["ros_command_uid"] = matched_uid
        metrics["dobot_command_id"] = matched_dobot_id
        metrics["robot_feedback_command_id"] = ctx.feedback_command_id
        metrics["command_id_match"] = command_id_match
        metrics["command_result_status"] = command_result_status
        metrics["dobot_command_text"] = matched_command_text
        metrics["command_tracking_source"] = tracking_source
        metrics["command_tracking_confidence"] = tracking_confidence
        metrics["feedback_command_id_before_send"] = feedback_before_send
        metrics["feedback_command_id_at_result"] = ctx.feedback_command_id
        metrics["feedback_command_id_changed"] = feedback_id_changed
        metrics["settle_match_method"] = settle_match_method
        metrics["settle_match_ambiguous"] = match_result.ambiguous
        metrics["settle_candidate_count"] = match_result.candidate_count
        metrics["settle_match_error_rad"] = match_result.match_error_rad
        metrics["settle_second_best_error_rad"] = match_result.second_best_error_rad
        metrics["settle_match_age_ms"] = match_result.match_age_ms
        metrics["pending_command_count"] = match_result.pending_count
        metrics["target"] = matched_target_rad
        metrics["ros_cmd_tool_target"] = matched_target_tool

        self.triple_logger.log_latency_event(metrics)
        self.triple_logger.log_command_result(
            control_command_seq=matched_seq,
            ros_command_uid=matched_uid,
            dobot_command_id=matched_dobot_id,
            robot_feedback_command_id=ctx.feedback_command_id,
            status=command_result_status,
            command_id_match=command_id_match,
            dobot_command_text=matched_command_text,
            command_tracking_source=tracking_source,
            command_tracking_confidence=tracking_confidence,
            feedback_command_id_before_send=feedback_before_send,
            feedback_command_id_at_result=ctx.feedback_command_id,
            feedback_command_id_changed=feedback_id_changed,
            settle_match_method=settle_match_method,
            settle_match_ambiguous=match_result.ambiguous,
            settle_candidate_count=match_result.candidate_count,
            settle_match_error_rad=match_result.match_error_rad,
            settle_second_best_error_rad=match_result.second_best_error_rad,
            settle_match_age_ms=match_result.match_age_ms,
            pending_command_count=match_result.pending_count,
            ros_timestamp=ctx.now_wall,
            ros_cmd_joints=matched_target_rad,
            robot_joints=ctx.q_current,
            ros_cmd_tool_target=matched_target_tool,
            robot_tool_actual=ctx.tool_act,
            robot_tool_target=ctx.tool_tgt,
            final_error_rad=metrics.get("final_error"),
            max_joint_error_rad=metrics.get("max_error"),
            robot_mode=ctx.current_mode,
            error_status=ctx.err_info['error_status'],
            operation_mode=ctx.operation_mode,
            unity_sample=matched_unity_sample,
        )
        self.pending_command_registry.mark_logged(match_result)

    def _control_loop_step(self):
        """
        Main Control Logic (50Hz) - Called by high-precision thread

        STRATEGY: "Proximity + Velocity-Based Stuck Detection"
        - Send when robot is CLOSE to last target (smooth real-time)
        - Send when robot is STUCK AND target changed significantly (safety)
        - NO TIMEOUT - Pure event-driven control
        """
        self._sample_counter += 1

        ctx = self._build_loop_context()
        if ctx is None:
            return

        # Teach & repeat recording is the one piece that mutates external
        # state from the loop prologue, so it stays inline rather than
        # hiding in a helper.
        tr = self.trajectory_recorder
        if tr.is_recording and not ctx.is_blocked:
            tr.record_tick(ctx.target_snapshot)

        self.controller.update_robot_state(ctx.q_current, ctx.now)

        self._snapshot_adaptive_telemetry(ctx)
        self._trigger_smart_suction(ctx)
        self._publish_tool_vectors(ctx)
        self._publish_do_status()
        self._publish_robot_mode_error(ctx)
        self._log_command_pass_samples(ctx)
        self._periodic_tool_index_query()
        self._auto_recover_from_error_mode(ctx)

        self._track_motion_t4_t5(ctx)

        # === SKIP TELEOP COMMANDS DURING PLAYBACK / POST-STOP HOMING ===
        if ctx.is_blocked:
            return  # monitoring data already published above; sequencer owns commands

        should_send, send_reason, queue_backlog_rad, run_queued_cmd = (
            self._make_controller_decision(ctx)
        )

        if should_send:
            self._build_and_send_command(
                ctx, send_reason, queue_backlog_rad, run_queued_cmd
            )

    def _build_and_send_command(self, ctx, send_reason, queue_backlog_rad, run_queued_cmd):
        """Final phase of the 50 Hz tick when the controller has decided
        a new command should go out.

        Sequence:
          1. Scene-safety guard (rate-limited error log on block, return).
          2. Format the wire command via the controller (force_send when
             we're recovering from a stuck condition so the queue-skip
             logic doesn't silently drop it).
          3. Send through the command_id-aware sender; on success,
             a) start latency tracking T1-T3,
             b) update active_command_* + pending_command_registry,
             c) write the ros_cmd row in the triple logger,
             d) emit CLI 'sent' report + JointState for the GUI graph,
             e) tell the controller this command went out so the next
                should_send call has the right baseline.
        """
        if motion_config.SCENE_SAFETY_BLOCK_REALTIME:
            safety_result = self.scene_safety_guard.check_joints_rad(ctx.target_snapshot)
            if safety_result.blocked:
                if ctx.now - self._last_scene_safety_block_log > 0.5:
                    self.get_logger().error(
                        "🧱 Scene safety blocked realtime target: "
                        f"{safety_result.status} {safety_result.detail} "
                        f"tcp={np.round(safety_result.point_xyzr[:3], 1).tolist()}"
                    )
                    self._last_scene_safety_block_log = ctx.now
                return

        # 1. Format Command
        # force_send=True when stuck: bypass should_skip_motion which silently drops commands
        is_stuck_recovery = send_reason.startswith("Stuck")
        cmd_str, q_safe = self.controller.format_command_string(
            ctx.target_snapshot,
            q_current=ctx.q_current,
            force_send=is_stuck_recovery,
        )
        if not cmd_str:
            return

        # 2. Timing Stats
        t3_cmd_send = time.time()
        # 3. Send to Robot
        send_result = self.sender.send_with_command_id(
            cmd_str,
            response_timeout=motion_config.REALTIME_COMMAND_ID_RESPONSE_TIMEOUT_SEC,
        )
        if not send_result.success:
            return

        self._realtime_command_seq += 1
        control_command_seq = self._realtime_command_seq
        ros_command_uid = f"ros_cmd_{control_command_seq:06d}"
        dobot_command_id = send_result.command_id
        feedback_command_id_before_send = ctx.feedback_command_id
        tracking_source = (
            "dobot_ack_id"
            if dobot_command_id is not None
            else "ros_sequence_pose_target"
        )
        tracking_confidence = "medium" if dobot_command_id is not None else "low"
        ros_cmd_tool_target = self.feedback.kinematics.forward_kinematics(
            np.degrees(q_safe)
        )
        # Start Tracking (T1-T3)
        self.latency_analyzer.start_tracking(
            ctx.unity_send_time_snapshot,
            ctx.target_recv_time_snapshot,
            t3_cmd_send,
            q_safe,
            current_q=ctx.q_current,
            command_seq=control_command_seq,
            target_tool=ros_cmd_tool_target,
        )
        self.active_command_unity_sample = ctx.unity_sample_snapshot
        self.active_command_uid = ros_command_uid
        self.active_command_dobot_id = dobot_command_id
        self.active_command_response = send_result.response
        self.active_command_text = cmd_str
        self.active_feedback_command_id_before_send = feedback_command_id_before_send
        self.active_command_tracking_source = tracking_source
        self.active_command_tracking_confidence = tracking_confidence
        self.pending_command_registry.register(
            PendingCommand(
                control_command_seq=control_command_seq,
                ros_command_uid=ros_command_uid,
                target_rad=q_safe.copy(),
                target_tool=np.asarray(ros_cmd_tool_target, dtype=float).copy(),
                dobot_command_id=dobot_command_id,
                command_text=cmd_str,
                unity_sample=ctx.unity_sample_snapshot,
                sent_wall_timestamp=t3_cmd_send,
                feedback_command_id_before_send=feedback_command_id_before_send,
            )
        )

        # File Log (CSV)
        sent_mono = time.perf_counter()
        time_since_last = sent_mono - self.controller.last_sent_time
        # Phase 13 uses a different velocity formula than phase 7 (no
        # abs around max) — preserve that behaviour by reading the raw
        # velocity again rather than reusing ctx.velocity_mag.
        velocity_mag = float(np.max(self.controller.robot_velocity))
        network_delay_ms = (
            (ctx.target_recv_time_snapshot - ctx.unity_send_time_snapshot) * 1000.0
            if ctx.target_recv_time_snapshot > 0 and ctx.unity_send_time_snapshot > 0
            else 0.0
        )
        decision_delay_ms = (
            (t3_cmd_send - ctx.target_recv_time_snapshot) * 1000.0
            if ctx.target_recv_time_snapshot > 0
            else 0.0
        )

        robot_status = self.feedback.get_error_status()

        if self.triple_logger:
            self.triple_logger.log_ros_cmd(
                q_safe,
                ctx.target_snapshot,
                t3_cmd_send,
                t1_unity_send_ros_wall=ctx.unity_send_time_snapshot,
                t2_ros_recv_wall=ctx.target_recv_time_snapshot,
                network_delay_ms=network_delay_ms,
                decision_delay_ms=decision_delay_ms,
                send_reason=send_reason,
                robot_mode=robot_status['robot_mode'],
                error_status=robot_status['error_status'],
                queue_backlog_rad=queue_backlog_rad,
                run_queued_cmd=run_queued_cmd,
                time_since_last_cmd_ms=time_since_last * 1000.0,
                velocity_mag_rad_s=velocity_mag,
                control_command_seq=control_command_seq,
                ros_command_uid=ros_command_uid,
                dobot_command_id=dobot_command_id,
                dobot_command_response=send_result.response,
                dobot_command_text=cmd_str,
                command_tracking_source=tracking_source,
                command_tracking_confidence=tracking_confidence,
                feedback_command_id_before_send=feedback_command_id_before_send,
                ros_cmd_tool_target=ros_cmd_tool_target,
                unity_sample=ctx.unity_sample_snapshot,
                joints_are_degrees=False,
            )

        # CLI Report
        msg = self.latency_analyzer.format_sent_report(
            True, send_reason, ctx.q_current, ctx.target_snapshot,
            self.controller.last_sent_target, self.controller.last_sent_time,
            self.controller.robot_velocity,
            robot_mode=robot_status['robot_mode'],
            error_status=robot_status['error_status'],
            time_since_last=time_since_last,
        )
        self.get_logger().info(msg)

        # 📊 Publish Sent Command for GUI graph
        sent_msg = JointState()
        sent_msg.header.stamp = self.get_clock().now().to_msg()
        sent_msg.position = q_safe.tolist()
        self.ros_publishers.sent_command.publish(sent_msg)

        # Update State in Controller
        self.controller.mark_command_sent(q_safe, sent_mono)

    def shutdown(self):
        """ปิดทุกอย่างอย่างเรียบร้อย"""
        self.get_logger().info("Shutting down...")
        self.stop_event.set()

        unity_session = getattr(self, "unity_session", None)
        if unity_session is not None and unity_session._worker_thread is not None:
            unity_session._worker_thread.join(timeout=5.0)
            self.get_logger().info(
                "Unity teleop sample worker stopped: "
                f"enqueued={unity_session.enqueued_count} "
                f"logged={unity_session.processed_count}"
            )

        if self.triple_logger:
            self.triple_logger.close()
            self.get_logger().info(f"Closed teleop session log: {self.triple_logger.file_path}")

        if self.adaptive_telemetry is not None:
            try:
                self.adaptive_telemetry.close()
                self.get_logger().info(
                    f"Closed adaptive telemetry: {self.adaptive_telemetry.path}"
                )
            except Exception as exc:
                self.get_logger().warn(f"Adaptive telemetry close failed: {exc}")

        self.feedback.stop()
        self.interactive.stop()
        self.connection.disconnect()

    def execute_motion_command(self, q_target):
        """
        ส่งคำสั่งเคลื่อนที่แบบ Synchronized (รอจนเสร็จ)
        เหมาะสำหรับ Mode 6 (Mouse Click) หรือ Step Move
        """
        # 1. Validate
        q_safe, is_clamped = self.validator.validate_and_clamp(q_target)

        # 2. Plan Command
        speed_percent = 50 # Default safe speed
        cmd_str = self.planner.format_command(q_safe, speed_percent)

        if not cmd_str:
            return

        # 3. Send & Sync (Blocking)
        self.get_logger().info(f"🔄 Executing Sync Motion to: {np.degrees(q_safe)}")

        # เรียกใช้ New Sync Method
        success = self.sender.send_command_with_sync(cmd_str)

        if success:
             self.get_logger().info("✅ Motion Complete (Synced)")
             self.controller.mark_command_sent(q_safe, time.perf_counter())
        else:
             self.get_logger().warn("⚠️ Motion Time-out or Failed")

    def _publish_haptic_feedback(self):
        """
        Publish collision-based haptic feedback for Quest 3 VR (20Hz)

        Reads collision state from feedback and sends haptic intensity to Unity/VR.
        """
        status = self.feedback.get_error_status()
        if not status:
            return

        collision_state = status.get('collision_state', 0)
        self.collision_haptic.update_and_publish(collision_state)


    def check_safety_status(self):
        """
        Safety Monitor Protocol (1Hz) - Delegated to SafetyMonitor class
        """
        self.safety_monitor.check_and_publish(self.feedback)

# =========================
# ===== ENTRY POINT ======
# =========================

def main():
    # เลือกโหมด
    select_control_mode()

    # เริ่ม ROS
    rclpy.init()
    node = TeleopNode()

    # Use MultiThreadedExecutor to prevent the 50Hz control loop
    # from blocking the Unity subscriber callbacks and vice versa.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
