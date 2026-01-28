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
from std_msgs.msg import String
import threading
import numpy as np
import time

# Import configuration
from mg400_controller.common.config.robot_config import CONTROL_MODE
from mg400_controller.common.config.motion_config import UNITY_TOPIC, RVIZ_TOPIC, DEBUG_TOPIC

# Import core modules
from mg400_controller.common.core.robot_connection import RobotConnection
from mg400_controller.common.core.feedback_handler import FeedbackHandler
from mg400_controller.common.core.command_sender import CommandSender

# Import logic modules
from mg400_controller.common.logic.joint_validator import JointValidator
from mg400_controller.common.logic.motion_planner import MotionPlanner

# Import utilities
from mg400_controller.common.utils.interactive_cmd import InteractiveCommandHandler

# =========================
# === LOGGING SETUP ======
# =========================

import logging
import os
from datetime import datetime

def setup_file_logger():
    """Setup file logger for detailed CSV logging"""
    # Create logs directory
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    
    # Create timestamped log file (human-readable format)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file = os.path.join(log_dir, f"teleop_data_{timestamp}.log")
    
    # Setup file handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter('%(message)s'))
    
    # Create logger
    file_logger = logging.getLogger('teleop_file')
    file_logger.setLevel(logging.INFO)
    file_logger.addHandler(file_handler)
    
    # Write CSV header
    file_logger.info("Timestamp,Unity_Send_Time,ROS_Recv_Time,Cmd_Send_Time,Network_Delay_ms,Decision_Delay_ms,Q_Curr_J1,Q_Curr_J2,Q_Curr_J3,Q_Curr_J4,Q_Target_J1,Q_Target_J2,Q_Target_J3,Q_Target_J4,Dist_to_Last_Sent,Send_Reason,Time_Since_Last_Cmd,Velocity_Mag,Vel_J1,Vel_J2,Vel_J3,Vel_J4")
    
    # Create separate latency breakdown logger
    latency_log_file = os.path.join(log_dir, f"latency_breakdown_{timestamp}.log")
    latency_handler = logging.FileHandler(latency_log_file)
    latency_handler.setLevel(logging.INFO)
    latency_handler.setFormatter(logging.Formatter('%(message)s'))
    
    latency_logger = logging.getLogger('latency_breakdown')
    latency_logger.setLevel(logging.INFO)
    latency_logger.addHandler(latency_handler)
    
    # Write latency CSV header - ✅ แยก Command Latency vs Motion Execution
    latency_logger.info("Timestamp,T1_Unity_Send,T2_ROS_Recv,T3_Cmd_Send,T4_Motion_Start,T5_Target_Reached,Network_Delay_ms,Decision_Delay_ms,Command_Latency_ms,Robot_Response_ms,Motion_Time_ms,Motion_Execution_ms,True_End_to_End_ms,Q_Target_J1,Q_Target_J2,Q_Target_J3,Q_Target_J4,Q_Final_J1,Q_Final_J2,Q_Final_J3,Q_Final_J4,Final_Error_rad,Max_Joint_Error_rad,Velocity_at_Arrival_rad_s,Is_Valid_Arrival")
    
    return file_logger, latency_logger

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
        self.validator = JointValidator(self.get_logger())
        self.planner = MotionPlanner(self.get_logger())
        
        # State Tracking
        self.latest_target = None
        self.target_recv_time = 0.0  # T2: When ROS received latest target
        self.unity_send_time = 0.0   # T1: When Unity sent the command
        self.last_sent_target = None
        self.last_cmd_time = 0.0
        
        # Motion Tracking (for true latency measurement)
        self.tracking_motion = False
        self.motion_start_time = 0.0     # T4: When robot started moving
        self.target_reached_time = 0.0   # T5: When robot reached target
        self.current_cmd_target = None   # Target being tracked
        self.last_velocity_mag = 0.0
        
        # Velocity Tracking
        self.last_robot_position = None
        self.last_position_time = 0.0
        self.robot_velocity = np.zeros(4)  # rad/s
        
        # Import Control Parameters from motion_config
        from mg400_controller.common.config.motion_config import (
            PROXIMITY_THRESHOLD,
            STUCK_VELOCITY_THRESHOLD,
            STUCK_TIME_THRESHOLD,
            TARGET_CHANGE_THRESHOLD,
            MOTION_START_THRESHOLD,
            VALID_FINAL_ERROR,
            VALID_MAX_JOINT_ERROR,
            VALID_VELOCITY,
            VALID_PER_JOINT_LIMIT
        )
        self.PROXIMITY_THRESHOLD = PROXIMITY_THRESHOLD
        self.STUCK_VELOCITY_THRESHOLD = STUCK_VELOCITY_THRESHOLD
        self.STUCK_TIME_THRESHOLD = STUCK_TIME_THRESHOLD
        self.TARGET_CHANGE_THRESHOLD = TARGET_CHANGE_THRESHOLD
        self.MOTION_START_THRESHOLD = MOTION_START_THRESHOLD
        self.VALID_FINAL_ERROR = VALID_FINAL_ERROR
        self.VALID_MAX_JOINT_ERROR = VALID_MAX_JOINT_ERROR
        self.VALID_VELOCITY = VALID_VELOCITY
        self.VALID_PER_JOINT_LIMIT = VALID_PER_JOINT_LIMIT
        self.last_stuck_check_time = 0.0
        
        # File Logger
        self.file_logger, self.latency_logger = setup_file_logger()
        self.get_logger().info(f" Logging to: logs/teleop_data_*.log")
        
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
        
        # 3. Connect to Robot
        if not self.connection.connect():
            self.get_logger().error("Failed to connect to robot")
            return
        
        self.connection.enable_robot()
        
        # 4. Initialize Handlers
        self.sender = CommandSender(self.connection, self.get_logger())
        
        self.feedback = FeedbackHandler(
            self.connection,
            self.pub_rviz,
            self.get_clock(),
            self.get_logger(),
            self.stop_event
        )
        
        self.interactive = InteractiveCommandHandler(
            self.connection,
            self.get_logger(),
            self.stop_event
        )
        
        # 5. Start Threads
        self.feedback.start()
        self.interactive.start()
        
        # 6. Start Control Loop (50Hz) - Sends latest target when robot is close enough
        self.create_timer(0.02, self._control_loop)
        
        self.get_logger().info(f"✅ Teleop Node Ready")
        self.get_logger().info(f"📊 Control Strategy: Proximity + Velocity-Based Stuck Detection")
        self.get_logger().info(f"📏 Proximity Threshold: {self.PROXIMITY_THRESHOLD:.3f} rad ({np.degrees(self.PROXIMITY_THRESHOLD):.1f} deg)")
        self.get_logger().info(f"🎯 Target Change Threshold: {self.TARGET_CHANGE_THRESHOLD:.3f} rad ({np.degrees(self.TARGET_CHANGE_THRESHOLD):.1f} deg)")
        self.get_logger().info(f"⏱️  Stuck Time Threshold: {self.STUCK_TIME_THRESHOLD:.1f} s")
        self.get_logger().info(f"🚫 No Timeout - Pure Real-Time Control")
    
    def _unity_callback(self, msg):
        """รับคำสั่งจาก Unity/VR - Store latest target only"""
        if not self.connection.connected:
            return
        
        # 1. Validate & Clamp Joints
        q_target = np.array(msg.position)
        q_safe, was_clamped = self.validator.validate_and_clamp(q_target)
        
        if was_clamped:
            self.get_logger().warn("⚠️ Joint command exceeded limits - clamped to safe range")
        
        # 2. Extract Unity timestamp (T1) - ✅ ใช้ ROS clock
        unity_send_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 3. Update Latest Target (Do NOT send here - control_loop will decide when to send)
        self.latest_target = q_safe
        # ✅ ใช้ ROS clock แทน time.time()
        now_ros_time = self.get_clock().now()
        self.target_recv_time = now_ros_time.nanoseconds * 1e-9  # T2: ROS receive time
        self.unity_send_time = unity_send_time_sec  # T1: Unity send time
    
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
            if self.last_robot_position is not None:
                dt = now - self.last_position_time
                if dt > 0.001:  # Avoid division by zero
                    self.robot_velocity = (q_current - self.last_robot_position) / dt
                else:
                    self.robot_velocity = np.zeros(4)
            
            self.last_robot_position = q_current.copy()
            self.last_position_time = now
            
            # === MOTION TRACKING (for latency measurement) ===
            if self.tracking_motion and self.current_cmd_target is not None:
                velocity_mag = np.linalg.norm(self.robot_velocity)
                
                # Detect motion start (T4): ✅ ลด threshold based on data analysis
                # Log analysis: 69% detection @ 0.003 rad/s
                # Target: 95%+ detection @ 0.002 rad/s
                if self.motion_start_time == 0.0 and velocity_mag > self.MOTION_START_THRESHOLD:
                    self.motion_start_time = now
                    self.get_logger().debug(f"Motion started: velocity={velocity_mag:.6f} rad/s")
                
                # Detect target reached (T5): ✅ เข้มงวดขึ้นตาม data analysis
                dist_to_target = np.linalg.norm(q_current - self.current_cmd_target)
                error_per_joint = np.abs(q_current - self.current_cmd_target)
                max_joint_error = np.max(error_per_joint)
                
                # Basic proximity check (loose, for T5 timestamp)
                is_near_target = dist_to_target < self.PROXIMITY_THRESHOLD  # 0.01 rad
                is_stopped = velocity_mag < self.STUCK_VELOCITY_THRESHOLD   # 0.005 rad/s
                
                if is_near_target and is_stopped and self.target_reached_time == 0.0:
                    self.target_reached_time = now
                    
                    # ✅ Log complete latency breakdown using SAVED timestamps
                    t1_unity_send = self.tracked_t1_unity
                    t2_ros_recv = self.tracked_t2_ros_recv
                    t3_cmd_send = self.tracked_t3_cmd_send
                    t4_motion_start = self.motion_start_time
                    t5_target_reached = self.target_reached_time
                    
                    # ✅ แยกชัดเจน: Command Latency vs Motion Execution
                    # 1. Command Latency (ควบคุมได้) = Unity → ROS ส่งคำสั่งเสร็จ
                    network_delay_ms = (t2_ros_recv - t1_unity_send) * 1000 if t1_unity_send > 0 else 0.0
                    decision_delay_ms = (t3_cmd_send - t2_ros_recv) * 1000
                    command_latency_ms = (t3_cmd_send - t1_unity_send) * 1000 if t1_unity_send > 0 else decision_delay_ms
                    
                    # 2. Motion Execution (ขึ้นกับหุ่น) = ส่งคำสั่ง → ถึงเป้าหมาย
                    robot_response_ms = (t4_motion_start - t3_cmd_send) * 1000 if t4_motion_start > 0 else 0.0
                    motion_time_ms = (t5_target_reached - t4_motion_start) * 1000 if t4_motion_start > 0 else (t5_target_reached - t3_cmd_send) * 1000
                    motion_execution_ms = (t5_target_reached - t3_cmd_send) * 1000
                    
                    # 3. Total (ความจริง) = Unity ส่ง → หุ่นถึงเป้าหมาย
                    true_end_to_end_ms = (t5_target_reached - t1_unity_send) * 1000 if t1_unity_send > 0 else (t5_target_reached - t2_ros_recv) * 1000
                    
                    # 4. Strict Validation (data-driven criteria)
                    # Log analysis shows false positives with:
                    #   - Final_Error up to 3.0° (0.052 rad)
                    #   - Max_Joint_Error up to 7.8° (0.136 rad)
                    #   - Velocity up to 0.01 rad/s
                    # New strict criteria from cfg.VALID_* constants
                    final_error = dist_to_target
                    is_valid_arrival = (
                        final_error < self.VALID_FINAL_ERROR and
                        max_joint_error < self.VALID_MAX_JOINT_ERROR and
                        velocity_mag < self.VALID_VELOCITY and
                        all(abs(err) < self.VALID_PER_JOINT_LIMIT for err in error_per_joint)
                    )
                    
                    # ✅ CLI Log: แยกชัดเจน controllable vs uncontrollable
                    latency_log = (
                        f"\n📊 LATENCY BREAKDOWN (Target Reached {'✅' if is_valid_arrival else '⚠️'}):"
                        f"\n   ━━━ Command Latency (Controllable) ━━━"
                        f"\n   🌐 Network:          {network_delay_ms:6.1f} ms  (Unity→ROS)"
                        f"\n   🧠 Decision:         {decision_delay_ms:6.1f} ms  (ROS processing)"
                        f"\n   📤 Command Total:    {command_latency_ms:6.1f} ms  (Unity→Cmd Sent)"
                        f"\n"
                        f"\n   ━━━ Motion Execution (Robot-Dependent) ━━━"
                        f"\n   🤖 Robot Response:   {robot_response_ms:6.1f} ms  (Cmd→Motion Start)"
                        f"\n   🏃 Motion Time:      {motion_time_ms:6.1f} ms  (Moving)"
                        f"\n   ⚙️  Execution Total:  {motion_execution_ms:6.1f} ms  (Cmd→Arrived)"
                        f"\n"
                        f"\n   ━━━ End-to-End ━━━"
                        f"\n   ⚡ TRUE LATENCY:     {true_end_to_end_ms:6.1f} ms  (Unity→Arrived)"
                        f"\n   📏 Final Error:      {final_error*57.3:.2f}° ({final_error:.4f} rad)"
                        f"\n   🎯 Max Joint Error:  {max_joint_error*57.3:.2f}° ({max_joint_error:.4f} rad)"
                        f"\n   🛑 Velocity:         {velocity_mag:.6f} rad/s"
                    )
                    self.get_logger().info(latency_log)
                    
                    
                    # ✅ Log to CSV with new structure
                    latency_csv = (
                        f"{now:.4f},"
                        f"{t1_unity_send:.4f},{t2_ros_recv:.4f},{t3_cmd_send:.4f},{t4_motion_start:.4f},{t5_target_reached:.4f},"
                        f"{network_delay_ms:.2f},{decision_delay_ms:.2f},{command_latency_ms:.2f},"
                        f"{robot_response_ms:.2f},{motion_time_ms:.2f},{motion_execution_ms:.2f},"
                        f"{true_end_to_end_ms:.2f},"
                        f"{self.current_cmd_target[0]:.6f},{self.current_cmd_target[1]:.6f},{self.current_cmd_target[2]:.6f},{self.current_cmd_target[3]:.6f},"
                        f"{q_current[0]:.6f},{q_current[1]:.6f},{q_current[2]:.6f},{q_current[3]:.6f},"
                        f"{final_error:.6f},{max_joint_error:.6f},{velocity_mag:.6f},{int(is_valid_arrival)}"
                    )
                    self.latency_logger.info(latency_csv)
                    
                    # Debug log
                    self.get_logger().debug(
                        f"Target reached: dist={dist_to_target:.6f}, "
                        f"max_joint_err={max_joint_error:.6f}, vel={velocity_mag:.6f}"
                    )
                    
                    # Stop tracking this motion
                    self.tracking_motion = False
            
            should_send = False
            send_reason = ""
            
            # === DECISION LOGIC ===
            if self.last_sent_target is None:
                # First command ever
                should_send = True
                send_reason = "Init"
            else:
                # Calculate distance: Current Robot Position vs Last Sent Target
                dist_to_last_sent = np.linalg.norm(q_current - self.last_sent_target)
                
                # Calculate how much target has changed
                target_delta = np.linalg.norm(self.latest_target - self.last_sent_target)
                
                # Condition 1: Proximity Check (Robot reached previous target)
                is_close = dist_to_last_sent < self.PROXIMITY_THRESHOLD
                
                # Condition 2: Stuck Detection (Robot not moving)
                velocity_mag = np.linalg.norm(self.robot_velocity)
                is_stuck = velocity_mag < self.STUCK_VELOCITY_THRESHOLD
                
                # For stuck, we need to be stuck for a minimum time to avoid false positives
                time_since_cmd = now - self.last_cmd_time
                stuck_long_enough = time_since_cmd > self.STUCK_TIME_THRESHOLD
                
                # Only send stuck recovery if target changed significantly
                # This prevents spam when Unity sends same target repeatedly
                target_changed_significantly = target_delta > self.TARGET_CHANGE_THRESHOLD
                
                if is_close and target_delta > 0.001:
                    # Robot is close AND target has changed (even slightly)
                    should_send = True
                    send_reason = f"Proximity_Dist{dist_to_last_sent:.3f}"
                elif is_stuck and stuck_long_enough and target_changed_significantly:
                    # Robot is stuck AND target changed significantly
                    should_send = True
                    send_reason = f"Stuck_Vel{velocity_mag:.4f}_Delta{target_delta:.3f}"
            
            # === SEND COMMAND ===
            if should_send:
                # ✅ ใช้ ROS clock
                ros_now = self.get_clock().now()
                now = ros_now.nanoseconds * 1e-9
                
                # Plan motion from CURRENT position to LATEST target
                command, speed, distance = self.planner.plan_motion(self.latest_target, q_current)
                
                if command is not None:
                    if self.sender.send_motion(command, speed, distance):
                        # Update state
                        prev_last_sent = self.last_sent_target.copy() if self.last_sent_target is not None else None
                        self.last_sent_target = self.latest_target.copy()
                        prev_cmd_time = self.last_cmd_time
                        self.last_cmd_time = now  # T3: Command send time
                        
                        # Start tracking this motion - SAVE timestamps now to prevent overwrite
                        self.tracking_motion = True
                        self.motion_start_time = 0.0
                        self.target_reached_time = 0.0
                        self.current_cmd_target = self.latest_target.copy()
                        
                        # Save timestamps for this specific command (won't be overwritten)
                        self.tracked_t1_unity = self.unity_send_time
                        self.tracked_t2_ros_recv = self.target_recv_time
                        self.tracked_t3_cmd_send = now
                        
                        # === DETAILED LOGGING ===
                        
                        # Calculate metrics
                        t1_unity_send = self.unity_send_time
                        t2_ros_recv = self.target_recv_time
                        t3_cmd_send = now
                        
                        network_delay_ms = (t2_ros_recv - t1_unity_send) * 1000 if t1_unity_send > 0 else 0.0
                        decision_delay_ms = (t3_cmd_send - t2_ros_recv) * 1000
                        
                        err_rad = self.latest_target - q_current
                        err_deg = np.degrees(err_rad)
                        dist_to_last = np.linalg.norm(q_current - prev_last_sent) if prev_last_sent is not None else 0.0
                        time_since_last = (now - prev_cmd_time) if prev_cmd_time > 0 else 0.0
                        velocity_mag = np.linalg.norm(self.robot_velocity)
                        velocity_deg = np.degrees(self.robot_velocity)
                        
                        # CLI Log (Human Readable)
                        log_msg = (
                            f"\n🚀 SENT [{send_reason}]"
                            f"\n   Network Delay: {network_delay_ms:.1f} ms (Unity→ROS)"
                            f"\n   Decision Delay: {decision_delay_ms:.1f} ms (ROS processing)"
                            f"\n   Time Since Last Cmd: {time_since_last*1000:.1f} ms"
                            f"\n   Distance to Last Target: {dist_to_last:.3f} rad ({np.degrees(dist_to_last):.1f} deg)"
                            f"\n   Robot Velocity: {velocity_mag:.4f} rad/s"
                            f"\n     J1={velocity_deg[0]:+7.2f}°/s, J2={velocity_deg[1]:+7.2f}°/s, J3={velocity_deg[2]:+7.2f}°/s, J4={velocity_deg[3]:+7.2f}°/s"
                            f"\n   Current Error (Target - Robot):"
                            f"\n     J1={err_deg[0]:+7.2f}°, J2={err_deg[1]:+7.2f}°, J3={err_deg[2]:+7.2f}°, J4={err_deg[3]:+7.2f}°"
                            f"\n   Robot Position:"
                            f"\n     J1={np.degrees(q_current[0]):+7.2f}°, J2={np.degrees(q_current[1]):+7.2f}°, J3={np.degrees(q_current[2]):+7.2f}°, J4={np.degrees(q_current[3]):+7.2f}°"
                            f"\n   Target Position:"
                            f"\n     J1={np.degrees(self.latest_target[0]):+7.2f}°, J2={np.degrees(self.latest_target[1]):+7.2f}°, J3={np.degrees(self.latest_target[2]):+7.2f}°, J4={np.degrees(self.latest_target[3]):+7.2f}°"
                        )
                        self.get_logger().info(log_msg)
                        
                        # File Log (CSV for analysis) - Enhanced format
                        file_log_msg = (
                            f"{now:.4f},"
                            f"{t1_unity_send:.4f},{t2_ros_recv:.4f},{t3_cmd_send:.4f},"
                            f"{network_delay_ms:.2f},{decision_delay_ms:.2f},"
                            f"{q_current[0]:.6f},{q_current[1]:.6f},{q_current[2]:.6f},{q_current[3]:.6f},"
                            f"{self.latest_target[0]:.6f},{self.latest_target[1]:.6f},{self.latest_target[2]:.6f},{self.latest_target[3]:.6f},"
                            f"{dist_to_last:.6f},"
                            f"{send_reason},"
                            f"{time_since_last*1000:.2f},"
                            f"{velocity_mag:.6f},"
                            f"{self.robot_velocity[0]:.6f},{self.robot_velocity[1]:.6f},{self.robot_velocity[2]:.6f},{self.robot_velocity[3]:.6f}"
                        )
                        self.file_logger.info(file_log_msg)
    
    def shutdown(self):
        """ปิดทุกอย่างอย่างเรียบร้อย"""
        self.get_logger().info("Shutting down...")
        self.stop_event.set()
        
        self.feedback.stop()
        self.interactive.stop()
        self.connection.disconnect()

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