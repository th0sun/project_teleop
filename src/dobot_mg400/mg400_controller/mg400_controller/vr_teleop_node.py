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

# Import configuration
from mg400_controller.common.config.robot_config import (
    ENABLE_GET_ERROR,
    JOINT_LIMITS,
    ELBOW_ANGLE_LIMIT
)
from mg400_controller.common.config.motion_config import (
    VACUUM_DO_PORT,
    BLOW_DO_PORT,
    SUCTION_ACTIVATION_THRESHOLD,
    SMART_SUCTION_ENABLED,
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
from mg400_controller.common.utils.interactive_cmd import InteractiveCommandHandler
from mg400_controller.common.utils.error_handler import ErrorHandler
from mg400_controller.common.utils.collision_haptic import CollisionHaptic
from mg400_controller.common.trajectory.trajectory_recorder import TrajectoryRecorder
from mg400_controller.common.trajectory.trajectory_recorder import frames_from_joint_trajectory_msg
from mg400_controller.common.trajectory.teach_job_handler import TeachJobHandler
from mg400_controller.common.logic.safety_monitor import SafetyMonitor
from mg400_controller.common.logic.scene_safety_guard import SceneSafetyGuard
from mg400_controller.common.logic.teleop_controller import TeleopController
from mg400_controller.common.utils.latency_analyzer import LatencyAnalyzer
from mg400_controller.common.utils.clock_calibrator import ClockCalibrator
from mg400_controller.common.utils.mode_selection import select_control_mode
from mg400_controller.common.utils.unified_triple_logger import (
    UnifiedTripleLogger,
    prompt_enable_triple_logging,
)
from mg400_controller.common.ros.teleop_interfaces import (
    create_publishers,
    create_subscriptions,
    attach_unity_subscription,
)
from mg400_controller.common.ros.topic_config import declare_topic_parameters

# =========================
# ===== MAIN NODE ========
# =========================

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

        # Teleop Controller (The Brain)
        self.controller = TeleopController(self.validator, self.planner, self.get_logger())

        self.latest_target = None

        # 1. Initialize logic modules
        # Import MotionConfig for thresholds and Analyzer
        self.latency_analyzer = LatencyAnalyzer(motion_config)

        # --- Clock Synchronization (Triple-Lock) ---
        self.clock_calibrator = ClockCalibrator(window_size=50) # Now estimates drift automatically

        # Level 3: RTT Heartbeat (ROS-side ping)
        self.topics = declare_topic_parameters(self)
        self.ros_publishers = create_publishers(self, topics=self.topics)
        self.create_timer(1.0, self._publish_heartbeat) # 1Hz Ping

        # Cross-layer timestamp contract:
        # - T1/T2/T3/T4/T5 and CSV log timestamps are ROS-local wall seconds.
        # - perf_counter is used only for control-loop intervals and stuck logic.
        self.target_recv_time = 0.0  # T2: ROS receive wall time
        self.unity_send_time = 0.0   # T1: Unity send time calibrated into ROS wall time

        self._tool_query_counter = 0
        self._sample_counter = 0  # For session logger feedback decimation
        self._realtime_speed_defaults_pending = False
        self._last_realtime_speed_defaults_attempt = 0.0

        # --- One-file Teleop Session Logger (Unity → ROS2 → Robot) ---
        self.triple_logger = None
        if prompt_enable_triple_logging():
            self.triple_logger = UnifiedTripleLogger()
            self.get_logger().info(f"Teleop session logging enabled: {self.triple_logger.file_path}")
        else:
            self.get_logger().info("Teleop session logging disabled")

        # 2. Setup ROS Interfaces
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions (Delayed UNITY to avoid race condition)
        self.ros_subscriptions = create_subscriptions(
            self,
            unity_pong_callback=self._unity_pong_callback,
            suction_callback=self._suction_callback,
            light_callback=self._light_callback,
            scene_safety_callback=self._scene_safety_callback,
            dashboard_cmd_callback=self._dashboard_cmd_callback,
            teach_status_callback=self._teach_status_callback,
            traj_data_callback=self._traj_data_callback,
            joint_trajectory_callback=self._joint_trajectory_callback,
            speed_factor_callback=self._speed_factor_callback,
            teach_job_request_callback=self._teach_job_request_callback,
            topics=self.topics,
        )

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
            self.stop_event
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

        # Job-request dispatcher for /teach/job_request (compile/preview_sim/
        # execute/export/stop/record_*).  Replaces the implicit "publish
        # JointTrajectory == execute now" behaviour.  Status / artifact replies
        # go to /teach/job_status and /teach/job_artifact.
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

        self.get_logger().info(f"✅ Teleop Node Ready")
        self.get_logger().info(
            f"🎓 Teach & Repeat: {self.topics.teach_status} + "
            f"{self.topics.trajectory_data} + {self.topics.unity_trajectory}"
        )
        self.get_logger().info(f"📊 Control Strategy: Proximity + Velocity-Based Stuck Detection")
        self.get_logger().info(f"📏 Dyn Proximity Base: {motion_config.DYNAMIC_PROXIMITY_BASE_RAD:.3f} rad ({np.degrees(motion_config.DYNAMIC_PROXIMITY_BASE_RAD):.1f} deg)")
        self.get_logger().info(f"🎯 Target Change Threshold: {motion_config.TARGET_CHANGE_THRESHOLD:.3f} rad ({np.degrees(motion_config.TARGET_CHANGE_THRESHOLD):.1f} deg)")
        self.get_logger().info(f"⏱️  Stuck Time Threshold: {motion_config.STUCK_TIME_THRESHOLD:.1f} s")
        self.get_logger().info(f"🚫 No Timeout - Pure Real-Time Control")

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

    def _unity_pong_callback(self, msg):
        """
        Level 3: Receive Pong from Unity
        msg.data format: "ros_ping_ns,unity_timestamp_sec"
        """
        try:
            parts = msg.data.split(',')
            if len(parts) < 2: return

            ros_ping_ns = int(parts[0])
            unity_ts = float(parts[1])
            now_ns = self.get_clock().now().nanoseconds

            # Calculate RTT
            rtt_sec = (now_ns - ros_ping_ns) * 1e-9

            # Level 3 Estimation: Unity_Time = ROS_Time + Offset
            # So Offset = Unity_Time - (ROS_Time_at_Unity)
            # ROS_Time_at_Unity approx = ros_ping_ns + RTT/2
            ros_at_unity = (ros_ping_ns * 1e-9) + (rtt_sec / 2.0)
            true_offset = unity_ts - ros_at_unity

            # We can use this to 'nudged' the calibrator or just log it
            # For now, ClockCalibrator's min-window is more robust against jitter
            pass
        except Exception:
            pass


    def _unity_callback(self, msg):
        """รับคำสั่งจาก Unity/VR - Store latest target only"""
        if not self.connection.connected or len(msg.position) < 4:
            return

        try:
            # 1. Validate & Clamp Joints
            q_target = np.array(msg.position)

            # 🛡️ Anti-NaN Protection
            if np.any(np.isnan(q_target)):
                self.get_logger().warn("⚠️ Received NaN joints from Unity - ignoring command")
                return

            q_safe, was_clamped = self.validator.validate_and_clamp(q_target)

            if was_clamped:
                self.get_logger().warn("⚠️ Joint command exceeded limits - clamped to safe range", once=True)

            # 2. Extract Unity timestamp (T1) and ROS timestamp (T2)
            unity_send_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now_ros_sec = time.time()  # Use absolute time for sync and logging

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
                )

            # 4. Update Latest Target (Do NOT send here - control_loop will decide when to send)
            if self._realtime_speed_defaults_pending:
                tr = self.trajectory_recorder
                playback_blocked = tr.is_playing or (time.perf_counter() < tr._block_until)
                if not playback_blocked:
                    self._restore_realtime_speed_defaults("first_realtime_target")

            self.latest_target = q_compensated_safe          # Latency-compensated target
            self.target_recv_time = now_ros_sec          # T2: ROS receive time
            self.unity_send_time = corrected_unity_time  # T1: Calibrated Unity send time

        except Exception as e:
            self.get_logger().error(f"Error in _unity_callback: {e}")

    def _realtime_latest_target(self, now_wall: float):
        """Return the latest Unity target only.

        Realtime teleop must not enqueue a batch of historical Unity samples:
        MG400's motion port is FIFO, so any batch sent while the hand is still
        moving becomes stale tail work.  CP is kept alive by feedback-driven
        top-up decisions in TeleopController, but each top-up sends only the
        current latest target.
        """
        if self.latest_target is None:
            return None

        max_age = float(getattr(motion_config, "REALTIME_TARGET_MAX_AGE_SEC", 0.35))
        latest_q = np.asarray(self.latest_target[:4], dtype=float)
        target_recv_time = float(self.target_recv_time)
        unity_send_time = float(self.unity_send_time)

        target_age = now_wall - target_recv_time if target_recv_time > 0 else 0.0
        if target_age > max_age:
            return None
        return latest_q, target_recv_time, unity_send_time

    def _suction_callback(self, msg):
        """รับคำสั่งเปิด/ปิดหัวดูด/Gripper จาก Unity (Trigger Button)"""
        requested_state = msg.data

        # ตรวจสอบว่าสถานะที่ขอมาต่างกับสถานะปัจจุบันหรือไม่
        if requested_state != self.suction_state:
            self.suction_requested_state = requested_state

            if requested_state and motion_config.SMART_SUCTION_ENABLED and self.latest_target is not None:
                self.suction_target_q = self.latest_target.copy()
                self.suction_pending = True
                self.get_logger().info(f"🔘 Smart Suction queued: ON (Waiting for robot to reach target)")
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

    # ── Teach & Repeat callbacks ──────────────────────────────────────────────
    def _teach_status_callback(self, msg):
        """Handle /unity/teach_status: Record | Stop | Save | Load:<name> | Preview.

        DEPRECATED in favour of /teach/job_request.  Retained because the
        ``Save`` semantics overlap with the host-side recorder pipeline.
        """
        self.get_logger().warn(
            f"⚠️  Legacy /unity/teach_status used. Prefer /teach/job_request "
            f"(topic: {self.topics.teach_job_request})",
            once=True,
        )
        status = msg.data.strip()
        tr = self.trajectory_recorder
        self.get_logger().info(f"🎓 Teach status: {status}")

        if status == "Record":
            tr.start_recording()
        elif status == "Stop":
            tr.stop_all(go_home=True)
        elif status == "Save":
            tr.save_temp()
        elif status.startswith("Save:"):
            name = status.split(":", 1)[1].strip()
            path = tr.save_as(name)
            self.get_logger().info(f"💾 Trajectory saved as → {path}")
        elif status.startswith("Load:"):
            name = status.split(":", 1)[1].strip()
            tr.load(name)
        elif status == "Preview":
            # Publish full trajectory data for monitor background dots (race.py style)
            if tr.loaded_frames:
                t0 = tr.loaded_frames[0]["timeStamp"]
                preview = {
                    "t": [f["timeStamp"] - t0 for f in tr.loaded_frames],
                    "q": [[f["j1"], f["j2"], f["j3"], f["j4"]] for f in tr.loaded_frames]
                }
                pmsg = String(); pmsg.data = json.dumps(preview)
                self.ros_publishers.traj_preview.publish(pmsg)
            tr.start_preview()
        else:
            self.get_logger().warn(f"⚠️ Unknown teach status: {status}")

    def _traj_data_callback(self, msg):
        """Handle /unity/trajectory_data: raw JSON from Unity Save button."""
        json_str = msg.data.strip()
        if json_str:
            path = self.trajectory_recorder.save_from_unity_json(json_str)
            if path:
                self.get_logger().info(f"🎓 Unity trajectory saved → {path}")

    def _joint_trajectory_callback(self, msg):
        """Handle Unity's saved teach-repeat JointTrajectory and play it.

        DEPRECATED auto-play path: publishing JointTrajectory implicitly meant
        "execute now".  Use /teach/job_request with action='execute' instead.
        Kept for backwards compatibility while Unity migrates.
        """
        self.get_logger().warn(
            "⚠️  Legacy JointTrajectory auto-play path used. Migrate Unity to "
            f"/teach/job_request (topic: {self.topics.teach_job_request})",
            once=True,
        )
        frames = frames_from_joint_trajectory_msg(msg)
        if not frames:
            self.get_logger().warn("⚠️ Unity JointTrajectory had no valid 4-joint points")
            return

        tr = self.trajectory_recorder
        if tr.load_frames(frames, name="unity_joint_trajectory"):
            t0 = frames[0]["timeStamp"]
            preview = {
                "t": [f["timeStamp"] - t0 for f in frames],
                "q": [[f["j1"], f["j2"], f["j3"], f["j4"]] for f in frames]
            }
            pmsg = String(); pmsg.data = json.dumps(preview)
            self.ros_publishers.traj_preview.publish(pmsg)
            self.get_logger().info(
                f"🎓 Unity JointTrajectory received: {len(frames)} points, "
                f"{frames[-1]['timeStamp'] - frames[0]['timeStamp']:.2f}s; starting playback"
            )
            tr.start_preview()

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
        self.teach_job_handler.handle(payload)

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

            # === TEACH & REPEAT GATING ===
            # is_blocked: during playback OR during post-stop homing (5 s window)
            tr = self.trajectory_recorder
            is_blocked = tr.is_playing or (time.perf_counter() < tr._block_until)
            operation_mode = "teach_repeat_playback" if is_blocked else "realtime"
            if tr.is_recording and not is_blocked:
                # We record the *TARGET* from VR/Simulator, not the actual robot pos
                tr.record_tick(self.latest_target)

            # === UPDATE VELOCITY ===
            # Delegate velocity tracking to controller
            self.controller.update_robot_state(q_current, now)

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
            mode_msg = Int32()
            mode_msg.data = current_mode
            self.ros_publishers.robot_mode.publish(mode_msg)

            err_info = self.feedback.get_error_status()
            err_msg = Int32()
            err_msg.data = int(err_info['error_status'])
            self.ros_publishers.error_status.publish(err_msg)

            if self.triple_logger and self._sample_counter % 5 == 0:
                self.triple_logger.log_robot_feedback(
                    q_current,
                    ros_timestamp=now_wall,
                    robot_mode=current_mode,
                    error_status=err_info['error_status'],
                    robot_tool_actual=tool_act,
                    robot_tool_target=tool_tgt,
                    operation_mode=operation_mode,
                    joints_are_degrees=False,
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
            velocity_mag = np.max(np.abs(self.controller.robot_velocity))

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

                        if self.triple_logger:
                            metrics["final_tool_actual"] = tool_act
                            metrics["final_tool_target"] = tool_tgt
                            metrics["operation_mode"] = operation_mode
                            self.triple_logger.log_latency_event(metrics)

            # === SKIP TELEOP COMMANDS DURING PLAYBACK / POST-STOP HOMING ===
            if is_blocked:
                return  # monitoring data already published above; sequencer owns commands

            # ---------------------------------------------------------
            # 🧠 TELEOP CONTROLLER DECISION
            # ---------------------------------------------------------

            # ──────────────────────────────────────────────────────
            # DEFAULT QUEUE-AWARE PRODUCTION LOGIC
            # ──────────────────────────────────────────────────────
            queue_backlog_rad = self.feedback.get_queue_backlog()
            run_queued_cmd = self.feedback.get_run_queued_cmd()

            should_send, send_reason = self.controller.should_send_command(
                self.latest_target,
                q_current,
                now=now,
                queue_backlog_rad=queue_backlog_rad,
                run_queued_cmd=run_queued_cmd,
                target_velocity=self.target_compensator.target_velocity,
            )

            if should_send:
                latest_item = self._realtime_latest_target(now_wall)
                if latest_item is None:
                    return
                target_q, target_recv_time, unity_send_time = latest_item

                if motion_config.SCENE_SAFETY_BLOCK_REALTIME:
                    safety_result = self.scene_safety_guard.check_joints_rad(target_q)
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
                    target_q, q_current=q_current, force_send=is_stuck_recovery)

                if not cmd_str:
                    return

                # 2. Timing Stats
                t3_cmd_send = time.time()
                # 3. Send to Robot
                if self.sender.send(cmd_str):

                    # Start Tracking (T1-T3)
                    self.latency_analyzer.start_tracking(
                        unity_send_time,
                        target_recv_time,
                        t3_cmd_send,
                        q_safe,
                        current_q=q_current
                    )

                    # File Log (CSV)
                    sent_mono = time.perf_counter()
                    time_since_last = sent_mono - self.controller.last_sent_time
                    velocity_mag = np.max(self.controller.robot_velocity)
                    network_delay_ms = (
                        (target_recv_time - unity_send_time) * 1000.0
                        if target_recv_time > 0 and unity_send_time > 0
                        else 0.0
                    )
                    decision_delay_ms = (
                        (t3_cmd_send - target_recv_time) * 1000.0
                        if target_recv_time > 0
                        else 0.0
                    )

                    robot_status = self.feedback.get_error_status()

                    if self.triple_logger:
                        self.triple_logger.log_ros_cmd(
                            q_safe,
                            target_q,
                            t3_cmd_send,
                            t1_unity_send_ros_wall=unity_send_time,
                            t2_ros_recv_wall=target_recv_time,
                            network_delay_ms=network_delay_ms,
                            decision_delay_ms=decision_delay_ms,
                            send_reason=send_reason,
                            robot_mode=robot_status['robot_mode'],
                            error_status=robot_status['error_status'],
                            queue_backlog_rad=queue_backlog_rad,
                            run_queued_cmd=run_queued_cmd,
                            time_since_last_cmd_ms=time_since_last * 1000.0,
                            velocity_mag_rad_s=velocity_mag,
                            joints_are_degrees=False,
                        )

                    # CLI Report
                    msg = self.latency_analyzer.format_sent_report(
                        should_send, send_reason, q_current, target_q,
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

        if self.triple_logger:
            self.triple_logger.close()
            self.get_logger().info(f"Closed teleop session log: {self.triple_logger.file_path}")

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
