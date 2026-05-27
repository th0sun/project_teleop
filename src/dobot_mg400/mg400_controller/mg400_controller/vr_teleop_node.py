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
import queue

# Import configuration
from mg400_controller.common.config.robot_config import (
    ENABLE_GET_ERROR,
    JOINT_LIMITS,
    ELBOW_ANGLE_LIMIT
)
from mg400_controller.common.config.motion_config import (
    SUCTION_ACTIVATION_THRESHOLD,
)
import mg400_controller.common.config.motion_config as motion_config

# Vendor protocol surface (do not hardcode MG400 ASCII strings here).
from mg400_protocol.dashboard import clear_error, get_tool, speed_factor

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
from mg400_controller.common.ros.teleop_interfaces import (
    create_publishers,
    create_subscriptions,
    attach_unity_subscription,
)
from mg400_controller.common.ros.topic_config import declare_topic_parameters
from mg400_controller.common.ros.unity_teleop_sample import (
    UnityTeleopSample,
    UnityTeleopSampleError,
    build_ros_joint_cmd_rx_sample,
    parse_unity_joint_frame_id,
    parse_unity_teleop_sample,
)

# =========================
# ===== MAIN NODE ========
# =========================

LEGACY_HOME_JOINT_TOL_RAD = 1e-4
LEGACY_HOME_COMMAND_MIN_INTERVAL_SEC = 1.0

class TeleopNode(Node):
    def __init__(self):
        super().__init__('mg400_vr_teleop')

        # 1. Initialize Modules
        self.stop_event = threading.Event()

        self.connection = RobotConnection(self.get_logger())
        import mg400_controller.common.config.robot_config as cfg

        # 1. Initialize logic modules
        self.validator = JointValidator(JOINT_LIMITS, ELBOW_ANGLE_LIMIT, self.get_logger())
        self.planner = MotionPlanner(cfg.CONTROL_MODE, self.get_logger())
        self.target_compensator = TargetLatencyCompensator(self.validator)

        # Adaptive realtime telemetry — kept separate from triple_logger so
        # post-hoc analysis of the new gate / per-cmd SpeedJ scheme isn't
        # mixed in with the production CSV.  Disabled cleanly if the
        # log dir can't be opened.
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

        self.latest_target = None
        self.latest_unity_sample = None
        self.latest_unity_sample_recv_time = 0.0
        self.unity_samples_by_identity = {}
        self.pending_joint_msgs_by_identity = {}
        self.latest_target_unity_sample = None
        self.latest_target_recv_wall = 0.0
        self.current_unity_session_id = None
        self.active_command_unity_sample = None
        self.active_command_uid = None
        self.active_command_dobot_id = None
        self.active_command_response = None
        self.active_command_text = None
        self.active_feedback_command_id_before_send = None
        self.active_command_tracking_source = None
        self.active_command_tracking_confidence = None
        self.pending_command_registry = PendingCommandRegistry(max_size=100)
        self._last_unity_sample_warning = 0.0
        self._last_unity_stale_warning = 0.0
        self._last_legacy_home_command_wall = 0.0
        self.unity_joint_cmd_rx_session_id = f"ros_joint_cmd_rx_{int(time.time() * 1000)}"
        self._unity_joint_cmd_rx_seq = 0

        # 1. Initialize logic modules
        # Import MotionConfig for thresholds and Analyzer
        self.latency_analyzer = LatencyAnalyzer(motion_config)

        # --- Clock Synchronization (Triple-Lock) ---
        self.clock_calibrator = ClockCalibrator(window_size=50) # Now estimates drift automatically

        # Level 3: RTT Heartbeat (ROS-side ping)
        self.topics = declare_topic_parameters(self)
        self.ros_publishers = create_publishers(self, topics=self.topics)
        self.create_timer(1.0, self._publish_heartbeat) # 1Hz Ping

        requested_sample_control = bool(
            self.declare_parameter("use_unity_teleop_sample_for_control", False).value
        )
        if requested_sample_control:
            self.get_logger().warn(
                "use_unity_teleop_sample_for_control is disabled: "
                "/unity/joint_cmd is the only live robot-control path; "
                "/unity/teleop_sample is log/sync-only."
            )
        self.use_unity_teleop_sample_for_control = False

        # Cross-layer timestamp contract:
        # - T1/T2/T3/T4/T5 and CSV log timestamps are ROS-local wall seconds.
        # - perf_counter is used only for control-loop intervals and stuck logic.
        self.target_recv_time = 0.0  # T2: ROS receive wall time
        self.unity_send_time = 0.0   # T1: Unity send time calibrated into ROS wall time

        self._tool_query_counter = 0
        self._sample_counter = 0  # For session logger feedback decimation
        self._realtime_command_seq = 0
        self._realtime_speed_defaults_pending = False
        self._last_realtime_speed_defaults_attempt = 0.0
        self._unity_sample_queue = queue.SimpleQueue()
        self._unity_control_latest = None
        self._unity_control_latest_lock = threading.Lock()
        self._unity_control_latest_event = threading.Event()
        self._unity_sample_enqueued_count = 0
        self._unity_sample_processed_count = 0
        self._unity_control_processed_count = 0
        self._unity_sample_worker_thread = threading.Thread(
            target=self._unity_teleop_sample_worker_loop,
            daemon=True,
        )
        self._unity_control_worker_thread = None
        self._unity_sample_worker_thread.start()

        # --- One-file Teleop Session Logger (Unity → ROS2 → Robot) ---
        self.triple_logger = None
        if prompt_enable_triple_logging():
            self.triple_logger = UnifiedTripleLogger()
            self.get_logger().info(f"Teleop session logging enabled: {self.triple_logger.file_path}")
        else:
            self.get_logger().info("Teleop session logging disabled")

        # Mixed-primitive opt-in via ROS param.  Default stays False
        # (matches motion_config) so the production teach-and-repeat
        # path keeps using JointMovJ-only.  Operators flip it per
        # launch via `--ros-args -p use_mixed_primitives:=true`.  See
        # AGENTS.md §4.4 — the Cartesian compile path is verified on
        # real hardware but kept opt-in until more demo paths are swept
        # through the classifier.
        self._apply_mixed_primitives_param()

        # 2. Setup ROS Interfaces
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions (Delayed UNITY to avoid race condition).
        # The factory groups all Unity / monitor / teach inputs in one
        # place; the per-callback wiring below is read top-to-bottom by
        # `create_subscriptions` in `common/ros/teleop_interfaces.py`.
        #
        # Active inputs:
        #   unity_pong_callback        — Unity RTT heartbeat
        #   unity_teleop_sample        — Unity JSON trace protocol v1
        #   suction_callback           — Unity → vacuum gripper bool
        #   light_callback             — Unity → signal-light DO
        #   scene_safety_callback      — Unity → workspace-guard toggle
        #   dashboard_cmd_callback     — Unity → dashboard passthrough
        #   speed_factor_callback      — Unity → global SpeedFactor slider
        #   teach_job_request_callback — Unity → typed teach job request
        #
        self.ros_subscriptions = self._register_ros_subscriptions()

        # Suction Cup Control (Smart Trigger)
        self.suction_state = False
        self.suction_pending = False
        self.suction_requested_state = False
        self.suction_target_q = None
        self.last_do_status = 0 # For debugging changes
        # 3. Connect to Robot
        if not self.connection.connect():
            self.get_logger().error("Failed to connect to robot")
            return

        self.connection.enable_robot()

        # 4. Initialize Handlers
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

        # ErrorHandler (GetError API) - optional, disabled for simulator
        self.error_handler = None
        if ENABLE_GET_ERROR:
            self.error_handler = ErrorHandler(self.connection, self.get_logger())
            self.get_logger().info("✅ ErrorHandler enabled (GetError API)")
        else:
            self.get_logger().info("⚠️  ErrorHandler disabled (set ENABLE_GET_ERROR=True for real robot)")

        # Collision-based Haptic Feedback for Quest 3 VR
        self.collision_haptic = CollisionHaptic(self.ros_publishers.haptic, self.get_logger())

        # Trajectory Recorder (teach-and-repeat sequencer)
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
            model_path=os.environ.get("MG400_SCENE_SAFETY_MODEL", motion_config.SCENE_SAFETY_MODEL_PATH),
            enabled=self._env_bool(
                "MG400_SCENE_SAFETY_ENABLED",
                motion_config.SCENE_SAFETY_ENABLED_DEFAULT,
            ),
            warn_distance_mm=motion_config.SCENE_SAFETY_WARN_DISTANCE_MM,
            logger=self.get_logger(),
        )
        self._last_scene_safety_block_log = 0.0

        # Job-request dispatcher for the production teach-repeat path. Status
        # / artifact replies go to /teach/job_status and /teach/job_artifact.
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
            self.stop_event
        )


        # 5. Initialize Helpers
        self.safety_monitor = SafetyMonitor(self.ros_publishers.safety, self.get_logger(), self.error_handler)

        # 6. Start Threads
        self.feedback.start()

        # 5. Start Unity Subscriber (End of init to prevent race condition)
        self.ros_subscriptions.unity = attach_unity_subscription(
            self,
            self._unity_callback,
            qos_profile,
            topics=self.topics,
        )

        self.get_logger().info("✅ Teleop Node fully initialized and listening.")
        self.interactive.start()

        # 6. Start Control Loop in a Dedicated High-Precision Thread (Isolates from ROS jitter/CPU load)
        self.control_loop_thread = threading.Thread(target=self._high_precision_control_loop, daemon=True)
        self.control_loop_thread.start()

        # 7. Start Safety Monitor (1Hz)
        self.create_timer(1.0, self.check_safety_status)

        # 8. Start Collision Haptic Publisher (20Hz) for Quest 3 VR
        self.create_timer(0.05, self._publish_haptic_feedback)

        self.get_logger().info("✅ Teleop Node Ready")
        self.get_logger().info(f"🎓 Teach & Repeat: {self.topics.teach_job_request}")
        self.get_logger().info("📊 Control Strategy: Adaptive Δ + Per-Cmd SpeedJ + Stuck Detection")
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
            unity_pong_callback=self._unity_pong_callback,
            unity_teleop_sample_callback=self._unity_teleop_sample_callback,
            suction_callback=self._suction_callback,
            light_callback=self._light_callback,
            scene_safety_callback=self._scene_safety_callback,
            dashboard_cmd_callback=self._dashboard_cmd_callback,
            speed_factor_callback=self._speed_factor_callback,
            teach_job_request_callback=self._teach_job_request_callback,
            topics=self.topics,
        )

    def _unity_pong_callback(self, msg):
        """Level 3 pong handler — currently a no-op.

        Format: "ros_ping_ns,unity_timestamp_sec"

        Earlier we estimated unity_offset = unity_ts - (ros_ping + rtt/2),
        but ClockCalibrator's min-window method proved more robust against
        jitter and is the active offset source. This subscription is kept
        so Unity's pong publishes don't error out, and so future work can
        plug an estimator back in here.
        """
        return


    def _unity_teleop_sample_callback(self, msg):
        """Fast ROS callback: capture raw Unity JSON and return immediately."""
        recv_time = time.time()
        payload = msg.data
        self._unity_sample_enqueued_count += 1
        self._unity_sample_queue.put((recv_time, payload))

    def _unity_teleop_sample_worker_loop(self):
        """Log every Unity JSON sample off the ROS executor thread."""
        while not self.stop_event.is_set() or not self._unity_sample_queue.empty():
            try:
                recv_time, payload = self._unity_sample_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self._process_unity_teleop_sample_payload(
                    payload,
                    recv_time,
                    drive_control=False,
                    log_sample=True,
                    remember_sample=True,
                    handle_session=True,
                    accept_legacy=True,
                )
                self._unity_sample_processed_count += 1
            except Exception as exc:
                self.get_logger().error(f"Unity teleop sample worker error: {exc}")

    def _unity_teleop_control_worker_loop(self):
        """/unity/teleop_sample direct control is intentionally disabled."""
        return

    def _process_unity_teleop_sample_payload(
        self,
        payload: str,
        now_ros_sec: float,
        *,
        drive_control: bool = True,
        log_sample: bool = True,
        remember_sample: bool = True,
        handle_session: bool = True,
        accept_legacy: bool = True,
    ):
        """Receive Unity JSON trace protocol samples.

        This never drives the robot directly.  `/unity/joint_cmd` is the live
        robot-control topic; this JSON sample is only for logging and matching
        that joint command to Unity-side controller/IK context.
        """
        try:
            sample = parse_unity_teleop_sample(payload)
        except UnityTeleopSampleError as exc:
            if now_ros_sec - self._last_unity_sample_warning > 1.0:
                self.get_logger().warn(f"Invalid /unity/teleop_sample ignored: {exc}")
                self._last_unity_sample_warning = now_ros_sec
            return

        if handle_session:
            self._handle_unity_session_transition(sample, now_ros_sec)
        if remember_sample:
            self._remember_unity_sample(sample, now_ros_sec)

        if log_sample and self.triple_logger:
            self.triple_logger.log_unity_sample(sample, now_ros_sec)

        if accept_legacy and not self.use_unity_teleop_sample_for_control:
            self._accept_pending_joint_for_sample(sample, now_ros_sec)

        if drive_control and now_ros_sec - self._last_unity_sample_warning > 1.0:
            self.get_logger().warn(
                "/unity/teleop_sample direct control is disabled; "
                "publish live joint targets on /unity/joint_cmd."
            )
            self._last_unity_sample_warning = now_ros_sec

    def _handle_unity_session_transition(self, sample, now_ros_sec):
        session_id = sample.session_id
        if self.current_unity_session_id is None:
            self.current_unity_session_id = session_id
            return
        if session_id == self.current_unity_session_id:
            return

        previous_session_id = self.current_unity_session_id
        self.current_unity_session_id = session_id
        self.unity_samples_by_identity.clear()
        self.pending_joint_msgs_by_identity.clear()
        self.pending_command_registry = PendingCommandRegistry(max_size=100)
        if not self.use_unity_teleop_sample_for_control:
            return

        self._clear_direct_unity_target("unity_session_changed")
        try:
            self.target_compensator.reset()
        except AttributeError:
            pass
        self.controller.reset_reference(None, now=time.perf_counter())
        self.get_logger().info(
            "🔁 Unity teleop session changed; cleared direct-control target "
            f"{previous_session_id} -> {session_id}"
        )

    def _clear_direct_unity_target(self, reason: str) -> None:
        if not self.use_unity_teleop_sample_for_control:
            return
        self.latest_target = None
        self.latest_target_unity_sample = None
        self.latest_target_recv_wall = 0.0
        self.target_recv_time = 0.0
        self.unity_send_time = 0.0
        self.controller.stuck_start_time = 0.0
        self.controller.is_stuck = False

    def _direct_unity_target_is_stale(self, now_wall: float, target_recv_wall: float, unity_sample) -> bool:
        if not self.use_unity_teleop_sample_for_control:
            return False
        if target_recv_wall <= 0.0:
            return True

        age_sec = now_wall - target_recv_wall
        if age_sec <= motion_config.UNITY_TELEOP_SAMPLE_STALE_TIMEOUT_SEC:
            return False

        self.controller.stuck_start_time = 0.0
        self.controller.is_stuck = False
        if now_wall - self._last_unity_stale_warning > 1.0:
            session_id = getattr(unity_sample, "session_id", "")
            unity_seq_id = getattr(unity_sample, "unity_seq_id", "")
            self.get_logger().warn(
                "Unity teleop sample stale; suppressing robot command "
                f"age={age_sec * 1000.0:.1f}ms "
                f"session_id={session_id} unity_seq_id={unity_seq_id}"
            )
            self._last_unity_stale_warning = now_wall
        return True

    def _teleop_session_logging_active(self, now_wall: float) -> bool:
        if not self.use_unity_teleop_sample_for_control:
            return True
        if self.latest_unity_sample_recv_time <= 0.0:
            return False
        return (
            now_wall - self.latest_unity_sample_recv_time
            <= motion_config.UNITY_TELEOP_SAMPLE_STALE_TIMEOUT_SEC
        )

    def _reset_direct_unity_after_gap(self, now_ros_sec: float) -> None:
        if not self.use_unity_teleop_sample_for_control:
            return
        if self.latest_target_recv_wall <= 0.0:
            return

        gap_sec = now_ros_sec - self.latest_target_recv_wall
        if gap_sec <= motion_config.UNITY_TELEOP_SAMPLE_STALE_TIMEOUT_SEC:
            return

        try:
            self.target_compensator.reset()
        except AttributeError:
            pass
        self.pending_command_registry = PendingCommandRegistry(max_size=100)
        self.get_logger().info(
            "🔁 Unity teleop resumed after stale gap; reset latency compensator "
            f"gap={gap_sec * 1000.0:.1f}ms"
        )

    def _remember_unity_sample(self, sample, recv_time):
        self.latest_unity_sample = sample
        self.latest_unity_sample_recv_time = recv_time
        self.unity_samples_by_identity[(sample.session_id, sample.unity_seq_id)] = (
            sample,
            recv_time,
        )

        if len(self.unity_samples_by_identity) <= 256:
            return
        oldest_key = min(
            self.unity_samples_by_identity,
            key=lambda key: self.unity_samples_by_identity[key][1],
        )
        self.unity_samples_by_identity.pop(oldest_key, None)

    def _remember_pending_joint_msg(self, identity, msg, recv_time):
        self.pending_joint_msgs_by_identity[identity] = (msg, recv_time)

        if len(self.pending_joint_msgs_by_identity) <= 512:
            return
        oldest_key = min(
            self.pending_joint_msgs_by_identity,
            key=lambda key: self.pending_joint_msgs_by_identity[key][1],
        )
        self.pending_joint_msgs_by_identity.pop(oldest_key, None)
        if recv_time - self._last_unity_sample_warning > 1.0:
            session_id, unity_seq_id = oldest_key
            self.get_logger().warn(
                "Dropped pending /unity/joint_cmd because matching "
                "/unity/teleop_sample did not arrive before cache limit: "
                f"session_id={session_id} unity_seq_id={unity_seq_id}"
            )
            self._last_unity_sample_warning = recv_time

    def _accept_pending_joint_for_sample(self, sample, now_ros_sec):
        identity = (sample.session_id, sample.unity_seq_id)
        pending = self.pending_joint_msgs_by_identity.pop(identity, None)
        if pending is None:
            return

        joint_msg, joint_recv_time = pending
        self._accept_joint_msg(
            joint_msg,
            joint_recv_time,
            unity_sample=self._sample_with_match_context(
                sample,
                "exact_joint_header_identity_deferred",
                max(0.0, (now_ros_sec - joint_recv_time) * 1000.0),
            ),
        )

    def _recent_unity_sample(self, now_ros_sec, max_age_sec=0.25):
        if self.latest_unity_sample is None:
            return None
        age = now_ros_sec - self.latest_unity_sample_recv_time
        if 0.0 <= age <= max_age_sec:
            return self._sample_with_match_context(
                self.latest_unity_sample,
                "age_only_recent_sample",
                age * 1000.0,
            )
        return None

    def _sample_with_match_context(self, sample, method: str, age_ms: float):
        raw = dict(sample.raw)
        raw["unity_sample_match_method"] = method
        raw["unity_sample_age_ms"] = age_ms
        return UnityTeleopSample(
            raw=raw,
            protocol_version=sample.protocol_version,
            session_id=sample.session_id,
            unity_seq_id=sample.unity_seq_id,
        )

    def _joint_msg_identity(self, msg):
        return parse_unity_joint_frame_id(
            getattr(getattr(msg, "header", None), "frame_id", "")
        )

    def _sample_for_joint_msg(self, msg, now_ros_sec):
        identity = parse_unity_joint_frame_id(
            getattr(getattr(msg, "header", None), "frame_id", "")
        )
        if identity is None:
            return self._recent_unity_sample(now_ros_sec)

        sample_pair = self.unity_samples_by_identity.get(identity)
        if sample_pair is not None:
            sample, recv_time = sample_pair
            return self._sample_with_match_context(
                sample,
                "exact_joint_header_identity",
                max(0.0, (time.time() - recv_time) * 1000.0),
            )

        return None

    def _is_legacy_home_joint_cmd(self, msg) -> bool:
        try:
            q_target = np.asarray(msg.position[:4], dtype=float)
        except Exception:
            return False
        if q_target.shape[0] < 4 or not np.all(np.isfinite(q_target)):
            return False
        return bool(np.all(np.abs(q_target[:4]) <= LEGACY_HOME_JOINT_TOL_RAD))

    def _accept_legacy_home_joint_cmd(self, now_ros_sec: float) -> None:
        if now_ros_sec - self._last_legacy_home_command_wall < LEGACY_HOME_COMMAND_MIN_INTERVAL_SEC:
            return
        self._last_legacy_home_command_wall = now_ros_sec

        self._clear_direct_unity_target("legacy_home_joint_cmd")
        self.pending_command_registry = PendingCommandRegistry(max_size=100)
        try:
            self.target_compensator.reset()
        except AttributeError:
            pass
        self.controller.reset_reference(None, now=time.perf_counter())
        self.get_logger().info(
            "🏠 Legacy /unity/joint_cmd Home accepted during direct Unity sample control"
        )
        self.trajectory_recorder.stop_all(go_home=True)

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

        self._reset_direct_unity_after_gap(now_ros_sec)

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

    def _suction_callback(self, msg):
        """รับคำสั่งเปิด/ปิดหัวดูด/Gripper จาก Unity (Trigger Button)"""
        requested_state = msg.data

        # ตรวจสอบว่าสถานะที่ขอมาต่างกับสถานะปัจจุบันหรือไม่
        if requested_state != self.suction_state:
            self.suction_requested_state = requested_state

            if requested_state and motion_config.SMART_SUCTION_ENABLED and self.latest_target is not None:
                self.suction_target_q = self.latest_target.copy()
                self.suction_pending = True
                self.get_logger().info("🔘 Smart Suction queued: ON (Waiting for robot to reach target)")
            else:
                # สั่งทันที (Immediate Mode) สำหรับการปิด/ปล่อย หรือเมื่อไม่ได้เปิด Smart Suction
                self._handle_suction_cmd(requested_state)

    def _handle_suction_cmd(self, state):
        """จัดการการเปิด/ปิดหัวดูดแบบมีลำดับ (Sequence Control)"""
        if not self.connection.connected:
            self.get_logger().warn("⚠️ Cannot toggle suction; Robot disconnected.")
            return

        if state:
            # 🟢 เปิดการดูด (Suck)
            self.sender.set_digital_output(motion_config.VACUUM_DO_PORT, True)
            self.sender.set_digital_output(motion_config.BLOW_DO_PORT, False)
            self.suction_state = True
            self.get_logger().info("吸 [SUCK] Vacuum ON, Blow OFF")
        else:
            # 🔴 เริ่มขั้นตอนการปล่อยลูก (Release Sequence: Vacuum OFF -> Blow ON -> Auto-Off)
            self.sender.set_digital_output(motion_config.VACUUM_DO_PORT, False)
            self.sender.set_digital_output(motion_config.BLOW_DO_PORT, True)
            self.get_logger().info(f"💨 [RELEASE] Vacuum OFF, Blow ON (for {motion_config.BLOW_DURATION}s)")

            # ตั้งเวลาปิดพอร์ตเป่าลมอัตโนมัติ (Safety Timer)
            def turn_off_blow():
                try:
                    self.sender.set_digital_output(motion_config.BLOW_DO_PORT, False)
                    self.get_logger().info("🛑 [IDLE] Blow OFF, All suction ports closed")
                    self.suction_state = False
                except Exception as e:
                    self.get_logger().error(f"Error in turn_off_blow timer: {e}")

            threading.Timer(motion_config.BLOW_DURATION, turn_off_blow).start()

    def _light_callback(self, msg):
        """Callback for external light control (e.g. from GUI)"""
        if len(msg.data) >= 2:
            port = msg.data[0]
            state = bool(msg.data[1])
            if self.connection.connected:
                self.sender.set_digital_output(port, state)
            else:
                self.get_logger().warn(f"⚠️ Cannot set light port {port}; Robot disconnected.")

    def _dashboard_cmd_callback(self, msg):
        """Callback for arbitrary Dashboard Commands sent from GUI over ROS."""
        cmd = msg.data.strip()
        if not cmd:
            return

        if self.connection.connected:
            self.get_logger().info(f"📨 Dashboard Command received from GUI: {cmd}")
            # Add newline if missing as required by Dobot protocol
            if not cmd.endswith('\n'):
                cmd += '\n'
            # Send via connection (non-blocking for basic commands)
            self.connection.send_dashboard_cmd(cmd)
        else:
            self.get_logger().warn(f"⚠️ Cannot send dashboard cmd '{cmd}'; Robot disconnected.")

    def _speed_factor_callback(self, msg):
        """Apply Dobot global SpeedFactor from GUI/Unity."""
        try:
            value = int(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warn(f"⚠️ Invalid SpeedFactor payload: {msg.data!r}")
            return
        value = max(1, min(100, value))
        if self.connection.connected:
            cmd = speed_factor(value).render()
            self.get_logger().info(f"🏃 SpeedFactor update from GUI: {value}%")
            self.connection.send_dashboard_cmd(cmd + "\n")
        else:
            self.get_logger().warn(f"⚠️ Cannot set SpeedFactor({value}); Robot disconnected.")

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
        if not self._teleop_session_logging_active(feedback_wall):
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

    def _control_loop_step(self):
        """
        Main Control Logic (50Hz) - Called by high-precision thread

        STRATEGY: "Proximity + Velocity-Based Stuck Detection"
        - Send when robot is CLOSE to last target (smooth real-time)
        - Send when robot is STUCK AND target changed significantly (safety)
        - NO TIMEOUT - Pure event-driven control
        """
        # Increment sample counter for triple-layer logger
        self._sample_counter += 1

        if self.connection.connected and self.latest_target is not None:
            q_current = self.feedback.get_current_position()
            # Use perf_counter for ultra-precise delta-time calculation in logic
            now = time.perf_counter()
            now_wall = time.time()
            target_snapshot = np.asarray(self.latest_target[:4], dtype=float).copy()
            unity_sample_snapshot = self.latest_target_unity_sample
            target_recv_time_snapshot = float(self.target_recv_time)
            target_recv_wall_snapshot = float(self.latest_target_recv_wall)
            unity_send_time_snapshot = float(self.unity_send_time)
            target_velocity_raw = self.target_compensator.target_velocity
            target_velocity_snapshot = (
                None
                if target_velocity_raw is None
                else np.asarray(target_velocity_raw, dtype=float).copy()
            )

            # === TEACH & REPEAT GATING ===
            # is_blocked: during playback OR during post-stop homing (5 s window)
            tr = self.trajectory_recorder
            is_blocked = tr.is_playing or (time.perf_counter() < tr._block_until)
            operation_mode = "teach_repeat_playback" if is_blocked else "realtime"
            if tr.is_recording and not is_blocked:
                # We record the *TARGET* from VR/Simulator, not the actual robot pos
                tr.record_tick(target_snapshot)

            # === UPDATE VELOCITY ===
            # Delegate velocity tracking to controller
            self.controller.update_robot_state(q_current, now)

            # === ADAPTIVE TELEMETRY: PERIODIC FEEDBACK SNAPSHOT ===
            # Capture robot-side state at ~5 Hz (every 10th 50 Hz tick) so
            # post-hoc analysis can correlate gate decisions with what the
            # joints actually did.  Cheap: enqueue is non-blocking.
            if self.adaptive_telemetry is not None and (self._sample_counter % 10) == 0:
                try:
                    qd = self.controller.robot_velocity
                    q_target_fb = self.feedback.get_target_position()
                    backlog = float(np.max(np.abs(np.asarray(q_target_fb) - q_current)))
                    self.adaptive_telemetry.log_feedback(
                        q_actual_rad=q_current,
                        qd_rad_s=qd,
                        backlog_rad=backlog,
                        robot_mode=int(self.feedback.get_robot_mode()),
                        note=operation_mode,
                    )
                except Exception:
                    # Never let telemetry break the control loop.
                    pass

            # === SMART SUCTION TRIGGER ===
            if self.suction_pending and self.suction_target_q is not None:
                dist = np.max(np.abs(q_current - self.suction_target_q))

                # ถ้าระยะห่างน้อยกว่า Threshold ที่ตั้งไว้ (ถึงเป้าหมายแล้ว)
                # หรือถ้าหุ่นยนต์หยุดนิ่งสนิทแล้ว (Stuck/Reached) ก็ให้ยิงคำสั่งได้เลยเหมือนกันป้องกันการค้าง
                if dist < SUCTION_ACTIVATION_THRESHOLD or self.controller.stuck_start_time > 0:
                    self._handle_suction_cmd(self.suction_requested_state)
                    self.suction_pending = False

            # === PUBLISH TOOL VECTORS (XYZ) ===
            tool_act = self.feedback.get_tool_vector()
            tool_tgt = self.feedback.get_target_tool_vector()

            msg_act = Float64MultiArray()
            msg_act.data = tool_act.tolist()
            self.ros_publishers.tool_actual.publish(msg_act)

            msg_tgt = Float64MultiArray()
            msg_tgt.data = tool_tgt.tolist()
            self.ros_publishers.tool_target.publish(msg_tgt)

            # Flange actual = FK of actual joints (no tool offset)
            flange = self.feedback.get_flange_actual()
            msg_flange = Float64MultiArray()
            msg_flange.data = flange.tolist()
            self.ros_publishers.flange_actual.publish(msg_flange)

            # === PUBLISH DO STATUS (Bitmask) ===
            do_status = self.feedback.get_do_status()
            do_msg = Int64()
            do_msg.data = int(do_status)
            self.ros_publishers.do_status.publish(do_msg)

            if do_status != self.last_do_status:
                self.get_logger().info(f"📣 DO STATUS CHANGED: {bin(do_status)} (Hex: {hex(do_status)})")
                self.last_do_status = do_status

            # === PUBLISH ROBOT MODE & ERROR ===
            current_mode = int(self.feedback.get_robot_mode())
            feedback_command_id = self.feedback.get_command_id()
            mode_msg = Int32()
            mode_msg.data = current_mode
            self.ros_publishers.robot_mode.publish(mode_msg)

            err_info = self.feedback.get_error_status()
            err_msg = Int32()
            err_msg.data = int(err_info['error_status'])
            self.ros_publishers.error_status.publish(err_msg)
            velocity_mag = np.max(np.abs(self.controller.robot_velocity))
            session_logging_active = self._teleop_session_logging_active(now_wall)

            if self.triple_logger and session_logging_active:
                pass_samples = self.pending_command_registry.pass_samples(
                    q_current,
                    tool_actual=tool_act,
                    now_wall=now_wall,
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
                        else feedback_command_id != feedback_before_send
                    )
                    self.triple_logger.log_command_match(
                        event_type=sample.event_type,
                        control_command_seq=command.control_command_seq,
                        ros_command_uid=command.ros_command_uid,
                        dobot_command_id=command.dobot_command_id,
                        robot_feedback_command_id=feedback_command_id,
                        status=sample.status,
                        dobot_command_text=command.command_text,
                        command_tracking_source=sample.method,
                        command_tracking_confidence=sample.confidence,
                        feedback_command_id_before_send=feedback_before_send,
                        feedback_command_id_at_result=feedback_command_id,
                        feedback_command_id_changed=feedback_id_changed,
                        settle_match_method=sample.method,
                        settle_match_ambiguous=sample.candidate_count > 1,
                        settle_candidate_count=sample.candidate_count,
                        settle_match_error_rad=sample.joint_norm_error_rad,
                        settle_match_age_ms=sample.match_age_ms,
                        pending_command_count=sample.pending_count,
                        ros_timestamp=now_wall,
                        ros_cmd_joints=command.target_rad,
                        robot_joints=q_current,
                        ros_cmd_tool_target=command.target_tool,
                        robot_tool_actual=tool_act,
                        robot_tool_target=tool_tgt,
                        final_error_rad=sample.joint_norm_error_rad,
                        max_joint_error_rad=sample.joint_max_error_rad,
                        velocity_mag_rad_s=velocity_mag,
                        robot_mode=current_mode,
                        error_status=err_info['error_status'],
                        operation_mode=operation_mode,
                        unity_sample=command.unity_sample,
                    )

            # === PERIODIC TOOL INDEX QUERY (every ~5s at 50Hz = 250 cycles) ===
            self._tool_query_counter += 1
            if self._tool_query_counter >= 250:
                self._tool_query_counter = 0
                try:
                    resp = self.connection.send_and_wait(get_tool().render(), timeout=1.0)
                    if resp:
                        import re
                        m = re.search(r'\{(\d+)\}', resp)
                        if m:
                            tidx = int(m.group(1))
                            ti_msg = Int32()
                            ti_msg.data = tidx
                            self.ros_publishers.tool_index.publish(ti_msg)
                except Exception:
                    pass

            # === AUTO-RECOVERY (Clear Error) ===
            # If the robot actually hits a hardware limit or another error, it enters Mode 9.
            # We auto-clear it so it doesn't stay permanently frozen.
            if current_mode == 9:
                if not hasattr(self, 'last_clear_error_time'):
                    self.last_clear_error_time = 0.0
                if now - self.last_clear_error_time > 3.0:
                    self.get_logger().error("🛑 Robot is in ERROR STATE (Mode 9). Auto-clearing error...")
                    self.connection.send_and_wait(clear_error().render())
                    self.last_clear_error_time = now

            # === MOTION TRACKING (Latency Analyzer) ===
            # T4: Motion Start
            # Update Analyzer Stats
            self.latency_analyzer.update_tracking(velocity_mag)

            if velocity_mag > motion_config.MOTION_START_THRESHOLD:
                if self.latency_analyzer.mark_motion_start(now_wall):
                     self.get_logger().debug(f"Motion started: velocity={velocity_mag:.6f} rad/s")

            # T5: Target Reached
            # Using basic check here to trigger detailed analysis
            dist = np.linalg.norm(q_current - self.latency_analyzer.current_cmd_target) if self.latency_analyzer.current_cmd_target is not None else 999
            is_stopped = velocity_mag < 0.005

            if dist < 0.01 and is_stopped:
                if self.latency_analyzer.mark_target_reached(now_wall):
                    # Get Full Report
                    metrics, report = self.latency_analyzer.analyze_arrival(q_current, velocity_mag)
                    if metrics:
                        # CLI Log
                        self.get_logger().info(report)

                        if self.triple_logger and session_logging_active:
                            match_result = self.pending_command_registry.match(
                                q_current,
                                feedback_command_id=feedback_command_id,
                                now_wall=now_wall,
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
                                else feedback_command_id != feedback_before_send
                            )
                            command_id_match = match_result.command_id_match
                            command_result_status = match_result.status
                            settle_match_method = match_result.method
                            tracking_source = match_result.method
                            tracking_confidence = match_result.confidence
                            metrics["final_tool_actual"] = tool_act
                            metrics["final_tool_target"] = tool_tgt
                            metrics["operation_mode"] = operation_mode
                            metrics["unity_sample"] = matched_unity_sample
                            metrics["control_command_seq"] = matched_seq
                            metrics["ros_command_uid"] = matched_uid
                            metrics["dobot_command_id"] = matched_dobot_id
                            metrics["robot_feedback_command_id"] = feedback_command_id
                            metrics["command_id_match"] = command_id_match
                            metrics["command_result_status"] = command_result_status
                            metrics["dobot_command_text"] = matched_command_text
                            metrics["command_tracking_source"] = tracking_source
                            metrics["command_tracking_confidence"] = tracking_confidence
                            metrics["feedback_command_id_before_send"] = feedback_before_send
                            metrics["feedback_command_id_at_result"] = feedback_command_id
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
                                robot_feedback_command_id=feedback_command_id,
                                status=command_result_status,
                                command_id_match=command_id_match,
                                dobot_command_text=matched_command_text,
                                command_tracking_source=tracking_source,
                                command_tracking_confidence=tracking_confidence,
                                feedback_command_id_before_send=feedback_before_send,
                                feedback_command_id_at_result=feedback_command_id,
                                feedback_command_id_changed=feedback_id_changed,
                                settle_match_method=settle_match_method,
                                settle_match_ambiguous=match_result.ambiguous,
                                settle_candidate_count=match_result.candidate_count,
                                settle_match_error_rad=match_result.match_error_rad,
                                settle_second_best_error_rad=match_result.second_best_error_rad,
                                settle_match_age_ms=match_result.match_age_ms,
                                pending_command_count=match_result.pending_count,
                                ros_timestamp=now_wall,
                                ros_cmd_joints=matched_target_rad,
                                robot_joints=q_current,
                                ros_cmd_tool_target=matched_target_tool,
                                robot_tool_actual=tool_act,
                                robot_tool_target=tool_tgt,
                                final_error_rad=metrics.get("final_error"),
                                max_joint_error_rad=metrics.get("max_error"),
                                robot_mode=current_mode,
                                error_status=err_info['error_status'],
                                operation_mode=operation_mode,
                                unity_sample=matched_unity_sample,
                            )
                            self.pending_command_registry.mark_logged(match_result)

            # === SKIP TELEOP COMMANDS DURING PLAYBACK / POST-STOP HOMING ===
            if is_blocked:
                return  # monitoring data already published above; sequencer owns commands

            if self._direct_unity_target_is_stale(
                now_wall,
                target_recv_wall_snapshot,
                unity_sample_snapshot,
            ):
                return

            # ---------------------------------------------------------
            # 🧠 TELEOP CONTROLLER DECISION
            # ---------------------------------------------------------

            # ──────────────────────────────────────────────────────
            # DEFAULT QUEUE-AWARE PRODUCTION LOGIC
            # ──────────────────────────────────────────────────────
            queue_backlog_rad = self.feedback.get_queue_backlog()
            run_queued_cmd = self.feedback.get_run_queued_cmd()

            should_send, send_reason = self.controller.should_send_command(
                target_snapshot,
                q_current,
                now=now,
                queue_backlog_rad=queue_backlog_rad,
                run_queued_cmd=run_queued_cmd,
                target_velocity=target_velocity_snapshot,
            )

            if should_send:
                if motion_config.SCENE_SAFETY_BLOCK_REALTIME:
                    safety_result = self.scene_safety_guard.check_joints_rad(target_snapshot)
                    if safety_result.blocked:
                        if now - self._last_scene_safety_block_log > 0.5:
                            self.get_logger().error(
                                "🧱 Scene safety blocked realtime target: "
                                f"{safety_result.status} {safety_result.detail} "
                                f"tcp={np.round(safety_result.point_xyzr[:3], 1).tolist()}"
                            )
                            self._last_scene_safety_block_log = now
                        return

                # 1. Format Command
                # force_send=True when stuck: bypass should_skip_motion which silently drops commands
                is_stuck_recovery = send_reason.startswith("Stuck")
                cmd_str, q_safe = self.controller.format_command_string(
                    target_snapshot,
                    q_current=q_current,
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
                if send_result.success:
                    self._realtime_command_seq += 1
                    control_command_seq = self._realtime_command_seq
                    ros_command_uid = f"ros_cmd_{control_command_seq:06d}"
                    dobot_command_id = send_result.command_id
                    feedback_command_id_before_send = feedback_command_id
                    tracking_source = (
                        "dobot_ack_id"
                        if dobot_command_id is not None
                        else "ros_sequence_pose_target"
                    )
                    tracking_confidence = (
                        "medium"
                        if dobot_command_id is not None
                        else "low"
                    )
                    ros_cmd_tool_target = self.feedback.kinematics.forward_kinematics(
                        np.degrees(q_safe)
                    )
                    # Start Tracking (T1-T3)
                    self.latency_analyzer.start_tracking(
                        unity_send_time_snapshot,
                        target_recv_time_snapshot,
                        t3_cmd_send,
                        q_safe,
                        current_q=q_current,
                        command_seq=control_command_seq,
                        target_tool=ros_cmd_tool_target,
                    )
                    self.active_command_unity_sample = unity_sample_snapshot
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
                            unity_sample=unity_sample_snapshot,
                            sent_wall_timestamp=t3_cmd_send,
                            feedback_command_id_before_send=feedback_command_id_before_send,
                        )
                    )

                    # File Log (CSV)
                    sent_mono = time.perf_counter()
                    time_since_last = sent_mono - self.controller.last_sent_time
                    velocity_mag = np.max(self.controller.robot_velocity)
                    network_delay_ms = (
                        (target_recv_time_snapshot - unity_send_time_snapshot) * 1000.0
                        if target_recv_time_snapshot > 0 and unity_send_time_snapshot > 0
                        else 0.0
                    )
                    decision_delay_ms = (
                        (t3_cmd_send - target_recv_time_snapshot) * 1000.0
                        if target_recv_time_snapshot > 0
                        else 0.0
                    )

                    robot_status = self.feedback.get_error_status()

                    if self.triple_logger:
                        self.triple_logger.log_ros_cmd(
                            q_safe,
                            target_snapshot,
                            t3_cmd_send,
                            t1_unity_send_ros_wall=unity_send_time_snapshot,
                            t2_ros_recv_wall=target_recv_time_snapshot,
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
                            unity_sample=unity_sample_snapshot,
                            joints_are_degrees=False,
                        )

                    # CLI Report
                    msg = self.latency_analyzer.format_sent_report(
                        should_send, send_reason, q_current, target_snapshot,
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

        unity_worker = getattr(self, "_unity_sample_worker_thread", None)
        if unity_worker is not None:
            unity_worker.join(timeout=5.0)
        unity_control_worker = getattr(self, "_unity_control_worker_thread", None)
        if unity_control_worker is not None:
            self._unity_control_latest_event.set()
            unity_control_worker.join(timeout=5.0)

        if unity_worker is not None or unity_control_worker is not None:
            pending = 0
            unity_queue = getattr(self, "_unity_sample_queue", None)
            if unity_queue is not None:
                try:
                    pending = unity_queue.qsize()
                except NotImplementedError:
                    pending = -1
            self.get_logger().info(
                "Unity teleop workers stopped: "
                f"enqueued={getattr(self, '_unity_sample_enqueued_count', 0)} "
                f"logged={getattr(self, '_unity_sample_processed_count', 0)} "
                f"controlled={getattr(self, '_unity_control_processed_count', 0)} "
                f"pending={pending}"
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
