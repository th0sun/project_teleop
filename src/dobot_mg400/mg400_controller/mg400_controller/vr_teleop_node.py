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
from mg400_controller.common.logic.queue_manager import QueueManager

# Import utilities
from mg400_controller.common.utils.interactive_cmd import InteractiveCommandHandler

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
        self.queue_mgr = QueueManager(self.get_logger())
        
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
        self.sender = CommandSender(self.connection, self.queue_mgr, self.get_logger())
        
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
        
        # 6. Start Queue Leak Timer (ลดคิวทุกๆ cycle)
        self.create_timer(0.01, self._queue_leak_callback)  # 100 Hz
        
        self.get_logger().info("✅ Teleop Node Ready")
    
    def _unity_callback(self, msg):
        """รับคำสั่งจาก Unity/VR"""
        if not self.connection.connected:
            return
        
        # 1. Validate & Clamp Joints
        q_target = np.array(msg.position)
        q_safe, was_clamped = self.validator.validate_and_clamp(q_target)
        
        # 2. Get Current Position
        q_current = self.feedback.get_current_position()
        
        # 3. Plan Motion
        command, speed, distance = self.planner.plan_motion(q_safe, q_current)
        
        if command is None:
            # ข้ามเพราะเคลื่อนที่น้อยเกินไป
            return
        
        # 4. Send Command
        self.sender.send_motion(command, speed, distance)
    
    def _queue_leak_callback(self):
        """Callback สำหรับลดคิว (leaky bucket)"""
        self.queue_mgr.leak_queue()
    
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