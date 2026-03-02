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
from std_msgs.msg import String, Float64MultiArray, Bool, Int32MultiArray, Int64, Int32
import threading
import numpy as np
import time
import datetime
import csv
import queue

# Import configuration
from mg400_controller.common.config.robot_config import (
    CONTROL_MODE, 
    ENABLE_GET_ERROR,
    JOINT_LIMITS,
    ELBOW_ANGLE_LIMIT,
    LOGIC_MODE
)
from mg400_controller.common.config.motion_config import (
    UNITY_TOPIC, RVIZ_TOPIC, DEBUG_TOPIC, SAFETY_TOPIC, HAPTIC_TOPIC, 
    SUCTION_TOPIC, LIGHT_TOPIC, VACUUM_DO_PORT, BLOW_DO_PORT, 
    SUCTION_ACTIVATION_THRESHOLD, SMART_SUCTION_ENABLED
)
import mg400_controller.common.config.motion_config as motion_config

# Import core modules
from mg400_controller.common.core.robot_connection import RobotConnection
from mg400_controller.common.core.feedback_handler import FeedbackHandler
from mg400_controller.common.core.command_sender import CommandSender

# Import logic modules
from mg400_controller.common.logic.joint_validator import JointValidator
from mg400_controller.common.logic.motion_planner import MotionPlanner

# Import utilities
from mg400_controller.common.utils.interactive_cmd import InteractiveCommandHandler
from mg400_controller.common.utils.error_handler import ErrorHandler
from mg400_controller.common.utils.collision_haptic import CollisionHaptic
from mg400_controller.common.trajectory.trajectory_recorder import TrajectoryRecorder
from mg400_controller.common.utils.teleop_logger import TeleopLogger
from mg400_controller.common.logic.safety_monitor import SafetyMonitor
from mg400_controller.common.logic.teleop_controller import TeleopController
from mg400_controller.common.logic.target_predictor import TargetPredictor
from mg400_controller.common.utils.latency_analyzer import LatencyAnalyzer
from mg400_controller.common.utils.clock_calibrator import ClockCalibrator

# =========================
# === MODE SELECTION =====
# =========================

def select_control_mode():
    """ให้ผู้ใช้เลือกโหมดควบคุม"""
    print("\n" + "="*50)
    print("🤖 MG400 VR Teleop Controller")
    print("="*50)
    print("\nSelect control mode:")
    print("1 = JointMovJ (recommended - fast & accurate)")
    print("2 = MovJ (joint space with Cartesian planning)")
    print("3 = MovL (linear Cartesian motion)")
    
    mode_in = input("> ").strip()
    
    modes = {
        "1": "jointmovj",
        "2": "movj",
        "3": "movl"
    }
    
    selected = modes.get(mode_in, "jointmovj")
    print(f"\n[INFO] Control Mode = {selected.upper()}")
    
    # Update config
    import mg400_controller.common.config.robot_config as cfg
    cfg.CONTROL_MODE = selected
    
    # ── 🧪 Experimental Logic Mode Selection (TEMPORARY) ──
    print("\n" + "-"*50)
    print("🧪 Select Teleop Logic Mode:")
    print("0 = Default (current production logic)")
    print("1 = M11_Stable  (เสถียรสุด ทนทานทุกแพตเทิร์น)")
    print("2 = M14_Smooth  (ลื่นไหว ดีที่สุดสำหรับ circle/sine)")
    print("3 = M15_Sharp   (คมกริบ ดีที่สุดสำหรับ square/zigzag)")
    print("-"*50)
    
    logic_in = input("Logic> ").strip()
    
    logic_modes = {
        "0": "default",
        "1": "m11",
        "2": "m14",
        "3": "m15"
    }
    
    selected_logic = logic_modes.get(logic_in, "default")
    cfg.LOGIC_MODE = selected_logic
    
    logic_names = {"default": "Default (Production)", "m11": "M11_Stable",
                   "m14": "M14_Smooth", "m15": "M15_Sharp"}
    print(f"[INFO] Logic Mode = {logic_names.get(selected_logic, selected_logic)}")
    
    return selected

# =========================
# ===== MAIN NODE ========
# =========================

class TeleopNode(Node):
    def __init__(self):
        super().__init__('mg400_vr_teleop')
        
        # 1. Initialize Modules
        self.stop_event = threading.Event()
        
        self.connection = RobotConnection(self.get_logger())
        # 1. Initialize logic modules
        self.validator = JointValidator(JOINT_LIMITS, ELBOW_ANGLE_LIMIT, self.get_logger())
        self.planner = MotionPlanner(CONTROL_MODE, self.get_logger())
        
        # Teleop Controller (The Brain)
        self.controller = TeleopController(self.validator, self.planner, self.get_logger())
        self.predictor = TargetPredictor(default_dt=0.02, prediction_horizon_sec=0.08, logger=self.get_logger())
        
        # 🧪 Experimental Logic (TEMPORARY — if LOGIC_MODE != "default")
        self._experimental_strategy = None
        if LOGIC_MODE != "default":
            from mg400_controller.common.logic.experimental_logic import ExperimentalStrategy
            self._experimental_strategy = ExperimentalStrategy(
                LOGIC_MODE, self.planner, self.get_logger()
            )
            self.get_logger().info(f"🧪 EXPERIMENTAL MODE: {self._experimental_strategy.mode_name}")
        
        self.latest_target = np.zeros(4)
        self.current_cmd_target = np.zeros(4)
        
        # 1. Initialize logic modules
        # Import MotionConfig for thresholds and Analyzer
        self.latency_analyzer = LatencyAnalyzer(motion_config)
        
        # --- Clock Synchronization (Triple-Lock) ---
        self.clock_calibrator = ClockCalibrator(window_size=50) # Now estimates drift automatically
        
        # Level 3: RTT Heartbeat (ROS-side ping)
        self.pub_heartbeat = self.create_publisher(Int64, "/teleop/ros_ping", 10)
        self.sub_pong = self.create_subscription(String, "/teleop/unity_pong", self._unity_pong_callback, 10)
        self.create_timer(1.0, self._publish_heartbeat) # 1Hz Ping
        
        # --- Analytics Logging (Async) ---
        self.log_queue = queue.Queue()
        self.csv_filename = f"teleop_analytics_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Timestamp_ROS', 'Timestamp_Unity', 
            'Raw_J1', 'Raw_J2', 'Raw_J3', 'Raw_J4',
            'Pred_J1', 'Pred_J2', 'Pred_J3', 'Pred_J4'
        ])
        
        # Start Log Worker Thread
        self.log_worker = threading.Thread(target=self._log_worker_loop, daemon=True)
        self.log_worker.start()
        
        self.get_logger().info(f"📊 Logging analytics to: {self.csv_filename} (Async Thread Started)")
        
        # State Tracking
        self.target_recv_time = 0.0  # T2
        self.unity_send_time = 0.0   # T1
        
        self._tool_query_counter = 0

        # File Logger
        self.teleop_logger = TeleopLogger("~/project_teleop_ws/logs")
        self.get_logger().info(f" Logging to: {self.teleop_logger.log_dir}")
        
        # 2. Setup ROS Interfaces
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions (Delayed UNITY to avoid race condition)
        self.sub_unity = None
        
        self.pub_rviz = self.create_publisher(JointState, RVIZ_TOPIC, 10)
        self.pub_debug = self.create_publisher(String, DEBUG_TOPIC, 10)
        self.pub_safety = self.create_publisher(String, SAFETY_TOPIC, 10)
        self.pub_do_status = self.create_publisher(Int64, motion_config.DO_STATUS_TOPIC, 10)
        self.pub_robot_mode = self.create_publisher(Int32, motion_config.ROBOT_MODE_TOPIC, 10)
        self.pub_error_status = self.create_publisher(Int32, motion_config.ERROR_STATUS_TOPIC, 10)
        
        # Dashboard Response (VR confirmation feedback)
        self.pub_dashboard_resp = self.create_publisher(String, "/mg400/dashboard_response", 10)

        # Tool Vector Publishers (XYZ Reading)
        self.pub_tool_actual = self.create_publisher(Float64MultiArray, "/mg400/tool_vector_actual", 10)
        self.pub_tool_target = self.create_publisher(Float64MultiArray, "/mg400/tool_vector_target", 10)
        self.pub_flange_actual = self.create_publisher(Float64MultiArray, "/robot/flange_actual", 10)
        self.pub_tool_index = self.create_publisher(Int32, "/robot/tool_index", 10)
        
        # 📊 Graph Data Publishers (for monitor_gui.py visualization)
        self.pub_predicted_target = self.create_publisher(JointState, "/teleop/predicted_target", 10)
        self.pub_sent_command = self.create_publisher(JointState, "/teleop/sent_command", 10)
        self.pub_unity_xyz = self.create_publisher(Float64MultiArray, "/teleop/unity_xyz", 10)
        
        # Suction Cup Control (Smart Trigger)
        self.suction_state = False
        self.suction_pending = False
        self.suction_requested_state = False
        self.suction_target_q = None
        self.last_do_status = 0 # For debugging changes
        self.sub_suction = self.create_subscription(
            Bool, SUCTION_TOPIC, self._suction_callback, 10
        )
        
        # Light Control
        self.sub_lights = self.create_subscription(
            Int32MultiArray, LIGHT_TOPIC, self._light_callback, 10
        )
        
        # === VR Dashboard Control (Unity → ROS) ===
        # Allows Unity/VR headset to trigger all dashboard commands the way
        # the keyboard interactive handler does. Command strings are the same.
        self.sub_dashboard_cmd = self.create_subscription(
            String, "/vr/dashboard_cmd", self._dashboard_cmd_callback, 10
        )
        self.sub_set_speed = self.create_subscription(
            Int32, "/vr/set_speed", self._set_speed_callback, 10
        )

        # 3. Connect to Robot
        if not self.connection.connect():
            self.get_logger().error("Failed to connect to robot")
            return
        
        self.connection.enable_robot()
        
        # 4. Initialize Handlers
        # PASS FEEDBACK HANDLER TO SENDER FOR SYNC
        self.feedback = FeedbackHandler(
            self.connection,
            self.pub_rviz,
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
        self.pub_haptic = self.create_publisher(String, HAPTIC_TOPIC, 10)
        self.collision_haptic = CollisionHaptic(self.pub_haptic, self.get_logger())
        
        # Trajectory Recorder (teach-and-repeat mode)
        self.trajectory_recorder = TrajectoryRecorder(self.feedback, self.sender, self.get_logger())
        
        self.interactive = InteractiveCommandHandler(
            self.connection,
            self.get_logger(),
            self.stop_event
        )
        
        
        # 5. Initialize Helpers
        self.safety_monitor = SafetyMonitor(self.pub_safety, self.get_logger(), self.error_handler)
        
        # 6. Start Threads
        self.feedback.start()
        
        # 5. Start Unity Subscriber (End of init to prevent race condition)
        self.sub_unity = self.create_subscription(
            JointState, UNITY_TOPIC, self._unity_callback, qos_profile
        )
        
        self.get_logger().info("✅ Teleop Node fully initialized and listening.")
        self.interactive.start()
        
        # 6. Start Control Loop (50Hz) - Sends latest target when robot is close enough
        self.create_timer(0.02, self._control_loop)
        
        # 7. Start Safety Monitor (1Hz)
        self.create_timer(1.0, self.check_safety_status)
        
        # 8. Start Collision Haptic Publisher (20Hz) for Quest 3 VR
        self.create_timer(0.05, self._publish_haptic_feedback)
        
        self.get_logger().info(f"✅ Teleop Node Ready")
        self.get_logger().info(f"📊 Control Strategy: Proximity + Velocity-Based Stuck Detection")
        self.get_logger().info(f"📏 Dyn Proximity Base: {motion_config.DYNAMIC_PROXIMITY_BASE_RAD:.3f} rad ({np.degrees(motion_config.DYNAMIC_PROXIMITY_BASE_RAD):.1f} deg)")
        self.get_logger().info(f"🎯 Target Change Threshold: {motion_config.TARGET_CHANGE_THRESHOLD:.3f} rad ({np.degrees(motion_config.TARGET_CHANGE_THRESHOLD):.1f} deg)")
        self.get_logger().info(f"⏱️  Stuck Time Threshold: {motion_config.STUCK_TIME_THRESHOLD:.1f} s")
        self.get_logger().info(f"🚫 No Timeout - Pure Real-Time Control")

    def _log_worker_loop(self):
        """Background thread to handle disk I/O and non-critical logging"""
        import queue
        while not self.stop_event.is_set() or not self.log_queue.empty():
            try:
                # Get log item from queue (blocking with timeout)
                try:
                    log_data = self.log_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if log_data[0] == 'CSV':
                    self.csv_writer.writerow(log_data[1])
                    # Flush occasionally
                    if self.log_queue.qsize() == 0:
                        self.csv_file.flush()
                elif log_data[0] == 'TELEOP_LATENCY':
                    self.teleop_logger.log_latency_breakdown(*log_data[1])
                elif log_data[0] == 'TELEOP_PERF':
                    self.teleop_logger.log_performance_metrics(*log_data[1])
                
                self.log_queue.task_done()
            except Exception as e:
                # We don't use logger here as it might be dead or recursively called
                print(f"Error in Log Worker: {e}")
    
    def _publish_heartbeat(self):
        """Level 3: Send Ping to Unity to measure RTT"""
        msg = Int64()
        msg.data = int(self.get_clock().now().nanoseconds)
        self.pub_heartbeat.publish(msg)

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
            now_ros_sec = self.get_clock().now().nanoseconds * 1e-9
            
            # --- 🕒 DYNAMIC CLOCK SYNCHRONIZATION (Triple-Lock) ---
            # Level 2 & 3: Filtered Min-Window + Drift Compensation
            corrected_unity_time = self.clock_calibrator.calibrate(unity_send_time_sec, now_ros_sec)
            
            # 3. Kalman Filter Prediction (with Anti-Overshoot)
            # Overcome physical robot inertia by predicting targets +80ms into the future
            # Use calibrated send time to ensure accurate dt calculation even over Tailscale
            q_actual = self.feedback.get_current_position()
            
            # --- Anti-Overshoot: Dynamic Prediction Damping ---
            dist_to_target = np.linalg.norm(q_safe - q_actual)
            velocity_mag = self.controller.robot_velocity
            
            # Reduce horizon if close to target or moving slowly
            # Base = 80ms, drops to 10ms when dist < 0.1 rad
            base_horizon = 0.08
            dist_scale = np.clip(dist_to_target / 0.1, 0.1, 1.0)
            vel_scale = np.clip(velocity_mag / 0.05, 0.0, 1.0)
            dynamic_horizon = base_horizon * dist_scale * vel_scale
            
            self.predictor.prediction_horizon_sec = dynamic_horizon
            predicted_q = self.predictor.update_and_predict(q_safe, corrected_unity_time, q_actual=q_actual)
            
            # 📊 Publish Predicted Target for GUI graph
            pred_msg = JointState()
            pred_msg.header.stamp = self.get_clock().now().to_msg()
            pred_msg.position = predicted_q.tolist()
            self.pub_predicted_target.publish(pred_msg)
            
            # 📊 Publish Unity Input XYZ (FK of raw Unity joint angles, degrees)
            try:
                unity_xyz = self.feedback.kinematics.forward_kinematics(np.degrees(q_safe))
                xyz_msg = Float64MultiArray()
                xyz_msg.data = unity_xyz.tolist()
                self.pub_unity_xyz.publish(xyz_msg)
            except Exception:
                pass
            
            # --- Log to CSV (Async) ---
            self.log_queue.put(('CSV', [
                now_ros_sec, corrected_unity_time,
                q_safe[0], q_safe[1], q_safe[2], q_safe[3],
                predicted_q[0], predicted_q[1], predicted_q[2], predicted_q[3]
            ]))
                
            # 4. Update Latest Target (Do NOT send here - control_loop will decide when to send)
            self.latest_target = predicted_q
            self.target_recv_time = now_ros_sec          # T2: ROS receive time
            self.unity_send_time = corrected_unity_time  # T1: Calibrated Unity send time
            
        except Exception as e:
            self.get_logger().error(f"Error in _unity_callback: {e}")

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
    
    # ────────────────────────────────────────────────────
    # 🎮 VR DASHBOARD CONTROL CALLBACKS
    # ────────────────────────────────────────────────────
    
    # Maps Unity string commands to robot TCP command strings
    _DASHBOARD_CMD_MAP = {
        "enable":       "EnableRobot()",
        "disable":      "DisableRobot()",
        "clear_error":  "ClearError()",
        "reset":        "ResetRobot()",
        "estop":        "EmergencyStop()",
        "pause":        "Pause()",
        "continue":     "Continue()",
    }

    def _dashboard_cmd_callback(self, msg):
        """
        Unity/VR Dashboard Command Handler.
        Accepts: enable | disable | clear_error | reset | estop | pause | continue
        Topic: /vr/dashboard_cmd  (std_msgs/String)
        Feedback: /mg400/dashboard_response  (std_msgs/String)
        """
        cmd_key = msg.data.strip().lower()
        
        robot_cmd = self._DASHBOARD_CMD_MAP.get(cmd_key)
        if robot_cmd is None:
            err = f"Unknown dashboard command from VR: '{cmd_key}'. Valid: {list(self._DASHBOARD_CMD_MAP.keys())}"
            self.get_logger().warn(err)
            resp = String()
            resp.data = f"ERROR:{err}"
            self.pub_dashboard_resp.publish(resp)
            return

        if not self.connection.connected:
            err = f"Robot disconnected — cannot execute: {cmd_key}"
            self.get_logger().warn(f"⚠️ {err}")
            resp = String()
            resp.data = f"ERROR:{err}"
            self.pub_dashboard_resp.publish(resp)
            return

        # Emergency stop is a special case — warn loudly
        if cmd_key == "estop":
            self.get_logger().warn(f"🚨 EMERGENCY STOP triggered from VR!")
        
        success = self.connection.send_dashboard_cmd(robot_cmd)
        
        resp = String()
        if success:
            self.get_logger().info(f"✅ [VR] {cmd_key} → {robot_cmd} OK")
            resp.data = f"OK:{cmd_key}"
        else:
            self.get_logger().error(f"❌ [VR] {cmd_key} → {robot_cmd} FAILED")
            resp.data = f"ERROR:{cmd_key} failed"
        self.pub_dashboard_resp.publish(resp)

    def _set_speed_callback(self, msg):
        """
        Unity/VR Speed Factor Control.
        Sets SpeedFactor globally (0–100). All subsequent JointMovJ commands use this.
        Topic: /vr/set_speed  (std_msgs/Int32)
        Feedback: /mg400/dashboard_response  (std_msgs/String)
        """
        speed = int(msg.data)
        if not (0 <= speed <= 100):
            err = f"Speed out of range: {speed} (must be 0-100)"
            self.get_logger().warn(f"⚠️ {err}")
            resp = String()
            resp.data = f"ERROR:{err}"
            self.pub_dashboard_resp.publish(resp)
            return

        if not self.connection.connected:
            self.get_logger().warn("⚠️ Robot disconnected — cannot set speed.")
            return

        robot_cmd = f"SpeedFactor({speed})"
        success = self.connection.send_dashboard_cmd(robot_cmd)
        
        resp = String()
        if success:
            self.get_logger().info(f"🚀 [VR] SpeedFactor set to {speed}%")
            resp.data = f"OK:speed={speed}"
        else:
            self.get_logger().error(f"❌ [VR] Failed to set SpeedFactor({speed})")
            resp.data = f"ERROR:speed_set failed"
        self.pub_dashboard_resp.publish(resp)

    
    def _control_loop(self):
        """
        Main Control Loop (50Hz)
        
        STRATEGY: "Proximity + Velocity-Based Stuck Detection"
        - Send when robot is CLOSE to last target (smooth real-time)
        - Send when robot is STUCK AND target changed significantly (safety)
        - NO TIMEOUT - Pure event-driven control
        
        LOGIC:
        1. Update velocity tracking
        2. Check Proximity: Is robot close to last_sent_target?
        3. Check Stuck: Is robot not moving + target changed significantly?
        4. Send if either condition is true
        """
        if self.connection.connected and self.latest_target is not None:
            q_current = self.feedback.get_current_position()
            # ✅ ใช้ ROS clock เท่านั้น
            ros_now = self.get_clock().now()
            now = ros_now.nanoseconds * 1e-9
            
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
            self.pub_tool_actual.publish(msg_act)
            
            msg_tgt = Float64MultiArray()
            msg_tgt.data = tool_tgt.tolist()
            self.pub_tool_target.publish(msg_tgt)
            
            # Flange actual = FK of actual joints (no tool offset)
            flange = self.feedback.get_flange_actual()
            msg_flange = Float64MultiArray()
            msg_flange.data = flange.tolist()
            self.pub_flange_actual.publish(msg_flange)
            
            # === PUBLISH DO STATUS (Bitmask) ===
            do_status = self.feedback.get_do_status()
            do_msg = Int64()
            do_msg.data = int(do_status)
            self.pub_do_status.publish(do_msg)
            
            if do_status != self.last_do_status:
                self.get_logger().info(f"📣 DO STATUS CHANGED: {bin(do_status)} (Hex: {hex(do_status)})")
                self.last_do_status = do_status
            
            # === PUBLISH ROBOT MODE & ERROR ===
            current_mode = int(self.feedback.get_robot_mode())
            mode_msg = Int32()
            mode_msg.data = current_mode
            self.pub_robot_mode.publish(mode_msg)
            
            err_info = self.feedback.get_error_status()
            err_msg = Int32()
            err_msg.data = int(err_info['error_status'])
            self.pub_error_status.publish(err_msg)

            # === PERIODIC TOOL INDEX QUERY (every ~5s at 50Hz = 250 cycles) ===
            self._tool_query_counter += 1
            if self._tool_query_counter >= 250:
                self._tool_query_counter = 0
                try:
                    resp = self.connection.send_and_wait("GetTool()", timeout=1.0)
                    if resp:
                        import re
                        m = re.search(r'\{(\d+)\}', resp)
                        if m:
                            tidx = int(m.group(1))
                            ti_msg = Int32()
                            ti_msg.data = tidx
                            self.pub_tool_index.publish(ti_msg)
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
                    self.connection.send_and_wait("ClearError()")
                    self.last_clear_error_time = now
            
            # === MOTION TRACKING (Latency Analyzer) ===
            # T4: Motion Start
            velocity_mag = np.max(np.abs(self.controller.robot_velocity))
            
            # Update Analyzer Stats
            self.latency_analyzer.update_tracking(velocity_mag)
            
            if velocity_mag > motion_config.MOTION_START_THRESHOLD:
                if self.latency_analyzer.mark_motion_start(now):
                     self.get_logger().debug(f"Motion started: velocity={velocity_mag:.6f} rad/s")
            
            # T5: Target Reached
            # Using basic check here to trigger detailed analysis
            dist = np.linalg.norm(q_current - self.latency_analyzer.current_cmd_target) if self.latency_analyzer.current_cmd_target is not None else 999
            is_stopped = velocity_mag < 0.005
            
            if dist < 0.01 and is_stopped:
                if self.latency_analyzer.mark_target_reached(now):
                    # Get Full Report
                    metrics, report = self.latency_analyzer.analyze_arrival(q_current, velocity_mag)
                    if metrics:
                        # CLI Log
                        self.get_logger().info(report)
                        
                        # CSV Log (Async)
                        self.log_queue.put(('TELEOP_LATENCY', [
                            now, metrics['t1'], metrics['t2'], metrics['t3'], metrics['t4'], metrics['t5'],
                            metrics['network_ms'], metrics['decision_ms'], metrics['command_ms'],
                            metrics['response_ms'], metrics['motion_time_ms'], metrics['execution_ms'],
                            metrics['e2e_ms'],
                            metrics['target'], metrics['final_q'],
                            metrics['final_error'], metrics['max_error'], metrics['velocity'], metrics['is_valid']
                        ]))

            
            # ---------------------------------------------------------
            # 🧠 TELEOP CONTROLLER DECISION
            # ---------------------------------------------------------
            
            # 🧪 EXPERIMENTAL PATH (only active if user selected m11/m14/m15)
            if self._experimental_strategy is not None:
                should_send, cmd_str, q_safe, exp_reason = self._experimental_strategy.process(
                    self.latest_target, q_current, now
                )
                if should_send and cmd_str:
                    t3_cmd_send = time.time()
                    if self.sender.send(cmd_str):
                        self.latency_analyzer.start_tracking(
                            self.unity_send_time, self.target_recv_time,
                            t3_cmd_send, q_safe, current_q=q_current
                        )
                        self.get_logger().info(f"🧪 {exp_reason}")
                        
                        sent_msg = JointState()
                        sent_msg.header.stamp = self.get_clock().now().to_msg()
                        sent_msg.position = q_safe.tolist()
                        self.pub_sent_command.publish(sent_msg)
                        
                        # Keep default controller state updated for monitors
                        self.controller.last_sent_target = q_safe
                        self.controller.last_sent_time = t3_cmd_send
                return  # ← Skip default logic entirely when in experimental mode
            
            # ──────────────────────────────────────────────────────
            # DEFAULT PRODUCTION LOGIC (unchanged)
            # ──────────────────────────────────────────────────────
            
            should_send, send_reason = self.controller.should_send_command(
                self.latest_target, 
                q_current
            )
            
            if should_send:
                # 1. Format Command
                # force_send=True when stuck: bypass should_skip_motion which silently drops commands
                is_stuck_recovery = send_reason.startswith("Stuck")
                cmd_str, q_safe = self.controller.format_command_string(
                    self.latest_target, q_current=q_current, force_send=is_stuck_recovery)
                
                if not cmd_str:
                    return

                # 2. Timing Stats
                t3_cmd_send = time.time()
                decision_delay_ms = (t3_cmd_send - self.target_recv_time) * 1000
                
                # 3. Send to Robot
                if self.sender.send(cmd_str):
                    # Start Tracking (T1-T3)
                    self.latency_analyzer.start_tracking(
                        self.unity_send_time,
                        self.target_recv_time,
                        t3_cmd_send,
                        q_safe,
                        current_q=q_current
                    )
                                    
                    # File Log (CSV)
                    dist_to_last = np.max(np.abs(q_current - q_safe))
                    time_since_last = t3_cmd_send - self.controller.last_sent_time
                    velocity_mag = np.max(self.controller.robot_velocity)
                    
                    robot_status = self.feedback.get_error_status()
                    
                    # CLI Report
                    msg = self.latency_analyzer.format_sent_report(
                        should_send, send_reason, q_current, self.latest_target, 
                        self.controller.last_sent_target, self.controller.last_sent_time,
                        self.controller.robot_velocity,
                        robot_mode=robot_status['robot_mode'],
                        error_status=robot_status['error_status']
                    )
                    self.get_logger().info(msg)
                    
                    # 📊 Publish Sent Command for GUI graph
                    sent_msg = JointState()
                    sent_msg.header.stamp = self.get_clock().now().to_msg()
                    sent_msg.position = q_safe.tolist()
                    self.pub_sent_command.publish(sent_msg)
                    
                    # Update State in Controller
                    self.controller.last_sent_target = q_safe
                    self.controller.last_sent_time = t3_cmd_send
                    
                    self.log_queue.put(('TELEOP_PERF', [
                        now, self.unity_send_time, self.target_recv_time, t3_cmd_send,
                        0.0, 0.0, # Network delay calculated in analyzer report
                        q_current, self.latest_target,
                        dist_to_last, send_reason,
                        time_since_last, velocity_mag, self.controller.robot_velocity
                    ]))
    
    def shutdown(self):
        """ปิดทุกอย่างอย่างเรียบร้อย"""
        self.get_logger().info("Shutting down...")
        self.stop_event.set()
        
        # Wait for log queue to drain
        if hasattr(self, 'log_worker'):
            self.log_worker.join(timeout=1.0)
        
        self.feedback.stop()
        self.interactive.stop()
        self.connection.disconnect()
        
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"📊 Closed analytics log: {self.csv_filename}")

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
        else:
             self.get_logger().warn("⚠️ Motion Time-out or Failed")
            
        # Update State
        self.controller.last_sent_target = q_safe

    
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
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()