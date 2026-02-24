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
from std_msgs.msg import String, Float64MultiArray, Bool
import threading
import numpy as np
import time

# Import configuration
from mg400_controller.common.config.robot_config import (
    CONTROL_MODE, 
    ENABLE_GET_ERROR,
    JOINT_LIMITS,
    ELBOW_ANGLE_LIMIT
)
from mg400_controller.common.config.motion_config import UNITY_TOPIC, RVIZ_TOPIC, DEBUG_TOPIC, SAFETY_TOPIC, HAPTIC_TOPIC, SUCTION_TOPIC, SUCTION_DO_PORT, SUCTION_ACTIVATION_THRESHOLD, SMART_SUCTION_ENABLED
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
        
        self.latest_target = np.zeros(4)
        self.current_cmd_target = np.zeros(4)
        
        # 1. Initialize logic modules
        # Import MotionConfig for thresholds and Analyzer
        self.latency_analyzer = LatencyAnalyzer(motion_config)
        
        # --- Clock Synchronization State ---
        self.clock_offset = 0.0
        self.min_time_diff = float('inf')
        self.is_time_calibrated = False
        
        # --- Analytics Logging ---
        import csv
        import datetime
        self.csv_filename = f"teleop_analytics_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Timestamp_ROS', 'Timestamp_Unity', 
            'Raw_J1', 'Raw_J2', 'Raw_J3', 'Raw_J4',
            'Pred_J1', 'Pred_J2', 'Pred_J3', 'Pred_J4'
        ])
        self.get_logger().info(f"📊 Logging analytics to: {self.csv_filename}")
        self.min_time_diff = float('inf')
        self.is_time_calibrated = False
        
        # State Tracking
        self.target_recv_time = 0.0  # T2
        self.unity_send_time = 0.0   # T1
        
        # File Logger
        self.teleop_logger = TeleopLogger("~/project_teleop_ws/logs")
        self.get_logger().info(f" Logging to: {self.teleop_logger.log_dir}")
        
        # 2. Setup ROS Interfaces
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub_unity = self.create_subscription(
            JointState, UNITY_TOPIC, self._unity_callback, qos_profile
        )
        
        self.pub_rviz = self.create_publisher(JointState, RVIZ_TOPIC, 10)
        self.pub_debug = self.create_publisher(String, DEBUG_TOPIC, 10)
        self.pub_safety = self.create_publisher(String, SAFETY_TOPIC, 10)
        
        # Tool Vector Publishers (XYZ Reading)
        self.pub_tool_actual = self.create_publisher(Float64MultiArray, "/mg400/tool_vector_actual", 10)
        self.pub_tool_target = self.create_publisher(Float64MultiArray, "/mg400/tool_vector_target", 10)
        
        # Suction Cup Control (Smart Trigger)
        self.suction_state = False
        self.suction_pending = False
        self.suction_requested_state = False
        self.suction_target_q = None
        self.sub_suction = self.create_subscription(
            Bool, SUCTION_TOPIC, self._suction_callback, 10
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
        self.interactive.start()
        
        # 6. Start Control Loop (50Hz) - Sends latest target when robot is close enough
        self.create_timer(0.02, self._control_loop)
        
        # 7. Start Safety Monitor (1Hz)
        self.create_timer(1.0, self.check_safety_status)
        
        # 8. Start Collision Haptic Publisher (20Hz) for Quest 3 VR
        self.create_timer(0.05, self._publish_haptic_feedback)
        
        self.get_logger().info(f"✅ Teleop Node Ready")
        self.get_logger().info(f"📊 Control Strategy: Proximity + Velocity-Based Stuck Detection")
        self.get_logger().info(f"📏 Proximity Threshold: {motion_config.PROXIMITY_THRESHOLD:.3f} rad ({np.degrees(motion_config.PROXIMITY_THRESHOLD):.1f} deg)")
        self.get_logger().info(f"🎯 Target Change Threshold: {motion_config.TARGET_CHANGE_THRESHOLD:.3f} rad ({np.degrees(motion_config.TARGET_CHANGE_THRESHOLD):.1f} deg)")
        self.get_logger().info(f"⏱️  Stuck Time Threshold: {motion_config.STUCK_TIME_THRESHOLD:.1f} s")
        self.get_logger().info(f"🚫 No Timeout - Pure Real-Time Control")
    
    def _unity_callback(self, msg):
        """รับคำสั่งจาก Unity/VR - Store latest target only"""
        if not self.connection.connected:
            return
        
        # 1. Validate & Clamp Joints
        q_target = np.array(msg.position)
        q_safe, was_clamped = self.validator.validate_and_clamp(q_target)
        
        if was_clamped:
            self.get_logger().warn("⚠️ Joint command exceeded limits - clamped to safe range", once=True)
        
        # 2. Extract Unity timestamp (T1) and ROS timestamp (T2)
        unity_send_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_ros_sec = self.get_clock().now().nanoseconds * 1e-9
        
        # --- 🕒 DYNAMIC CLOCK SYNCHRONIZATION ---
        # Find the absolute minimum difference (fastest packet over network)
        # This isolated Network Delay from Absolute Clock Offset (e.g. 92 years diff)
        time_diff = now_ros_sec - unity_send_time_sec
        if time_diff < self.min_time_diff:
            self.min_time_diff = time_diff
            # We assume the absolute fastest ping achievable over LAN/Localhost is 1ms
            self.clock_offset = self.min_time_diff - 0.001
            
            if not self.is_time_calibrated:
                self.get_logger().info(f"⏰ Initial Clock Sync! Unity is {self.clock_offset:.3f}s off from ROS")
                self.is_time_calibrated = True

        # Correct Unity time to match Ubuntu time perfectly
        corrected_unity_time = unity_send_time_sec + self.clock_offset
        
        # 3. Kalman Filter Prediction
        # Overcome physical robot inertia by predicting targets +80ms into the future
        # Use calibrated send time to ensure accurate dt calculation even over Tailscale
        predicted_q = self.predictor.update_and_predict(q_safe, corrected_unity_time)
        
        # --- Log to CSV ---
        try:
            self.csv_writer.writerow([
                now_ros_sec, corrected_unity_time,
                q_safe[0], q_safe[1], q_safe[2], q_safe[3],
                predicted_q[0], predicted_q[1], predicted_q[2], predicted_q[3]
            ])
        except Exception as e:
            pass # Ignore write errors to not block the control loop
            
        # 4. Update Latest Target (Do NOT send here - control_loop will decide when to send)
        self.latest_target = predicted_q
        self.target_recv_time = now_ros_sec          # T2: ROS receive time
        self.unity_send_time = corrected_unity_time  # T1: Calibrated Unity send time

    def _suction_callback(self, msg):
        """รับคำสั่งเปิด/ปิดหัวดูด/Gripper จาก Unity (Trigger Button)"""
        requested_state = msg.data
        
        # ตรวจสอบว่าสถานะที่ขอมาต่างกับสถานะปัจจุบัน หรือต่างกับคำสั่งที่รอดำเนินการอยู่หรือไม่
        if requested_state != self.suction_state and requested_state != self.suction_requested_state:
            self.suction_requested_state = requested_state
            
            if SMART_SUCTION_ENABLED and self.latest_target is not None:
                self.suction_target_q = self.latest_target.copy()
                self.suction_pending = True
                self.get_logger().info(f"🔘 Smart Suction queued: {'ON' if requested_state else 'OFF'} (Waiting for robot to reach target)")
            else:
                # สั่งทันที (Immediate Mode) หรือ Fallback กรณีไม่ได้เปิด Smart Suction
                if self.connection.connected:
                    success = self.sender.set_digital_output(SUCTION_DO_PORT, requested_state)
                    if success:
                        self.suction_state = requested_state
                        if not SMART_SUCTION_ENABLED:
                            self.get_logger().info(f"🔘 Immediate Suction Activated: {'ON' if requested_state else 'OFF'}")
                else:
                    self.get_logger().warn("⚠️ Cannot toggle suction; Robot disconnected.")
    
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
                    if self.connection.connected:
                        success = self.sender.set_digital_output(SUCTION_DO_PORT, self.suction_requested_state)
                        if success:
                            self.get_logger().info(
                                f"🎯 Smart Suction Activated: {'ON' if self.suction_requested_state else 'OFF'} "
                                f"(Error: {dist:.4f} rad" + (" - Triggered by Stillness" if dist >= SUCTION_ACTIVATION_THRESHOLD else "") + ")"
                            )
                            self.suction_state = self.suction_requested_state
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
                        
                        # CSV Log
                        self.teleop_logger.log_latency_breakdown(
                            now, metrics['t1'], metrics['t2'], metrics['t3'], metrics['t4'], metrics['t5'],
                            metrics['network_ms'], metrics['decision_ms'], metrics['command_ms'],
                            metrics['response_ms'], metrics['motion_time_ms'], metrics['execution_ms'],
                            metrics['e2e_ms'],
                            metrics['target'], metrics['final_q'],
                            metrics['final_error'], metrics['max_error'], metrics['velocity'], metrics['is_valid']
                        )
            
            # ---------------------------------------------------------
            # 🧠 TELEOP CONTROLLER DECISION
            # ---------------------------------------------------------
            
            should_send, send_reason = self.controller.should_send_command(
                self.latest_target, 
                q_current
            )
            
            if should_send:
                # 1. Format Command
                cmd_str, q_safe = self.controller.format_command_string(self.latest_target)
                
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
                    
                    # CLI Report
                    msg = self.latency_analyzer.format_sent_report(
                        should_send, send_reason, q_current, self.latest_target, 
                        self.controller.last_sent_target, self.controller.last_sent_time,
                        self.controller.robot_velocity
                    )
                    self.get_logger().info(msg)
                    
                    # Update State in Controller
                    self.controller.last_sent_target = q_safe
                    self.controller.last_sent_time = t3_cmd_send
                    
                    self.teleop_logger.log_performance_metrics(
                        now, self.unity_send_time, self.target_recv_time, t3_cmd_send,
                        0.0, 0.0, # Network delay calculated in analyzer report
                        q_current, self.latest_target,
                        dist_to_last, send_reason,
                        time_since_last, velocity_mag, self.controller.robot_velocity
                    )
    
    def shutdown(self):
        """ปิดทุกอย่างอย่างเรียบร้อย"""
        self.get_logger().info("Shutting down...")
        self.stop_event.set()
        
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