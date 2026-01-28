#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📡 Feedback Handler
รับและประมวลผลข้อมูล Real-time จากหุ่นยนต์

ใช้งาน:
    handler = FeedbackHandler(robot_connection, publisher, logger, stop_event)
    handler.start()
"""

import threading
import struct
import time
import numpy as np
from sensor_msgs.msg import JointState
from mg400_controller.common.utils.kinematics import KinematicsCalculator

class FeedbackHandler:
    def __init__(self, robot_connection, joint_publisher, clock, logger, stop_event):
        self.connection = robot_connection
        self.publisher = joint_publisher
        self.clock = clock
        self.logger = logger
        self.stop_event = stop_event
        
        self.kinematics = KinematicsCalculator()
        self.current_position = np.zeros(4)  # Active joints only
        self.last_valid_joints = np.zeros(4)
        
        self.thread = None
    
    def start(self):
        """เริ่ม thread รับ feedback"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.logger.info("📡 Feedback thread started")
    
    def get_current_position(self):
        """ดึงตำแหน่งปัจจุบัน (thread-safe)"""
        return self.current_position.copy()
    
    def _run(self):
        """Main loop รับข้อมูล feedback"""
        PACKET_SIZE = 1440
        buffer = b''
        
        # ตั้งค่า socket เป็น blocking mode
        self.connection.fb_sock.settimeout(None)
        
        self.logger.info("🎧 Listening for binary feedback (1440 bytes/packet)...")
        
        while not self.stop_event.is_set():
            try:
                # รับข้อมูล
                chunk = self.connection.fb_sock.recv(4096)
                if not chunk:
                    self.logger.warn("Connection closed by robot")
                    break
                
                buffer += chunk
                
                # ประมวลผล packet ที่สมบูรณ์
                while len(buffer) >= PACKET_SIZE:
                    data = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]
                    
                    self._process_packet(data)
            
            except BlockingIOError:
                time.sleep(0.005)
            except Exception as e:
                self.logger.error(f"Feedback error: {e}")
                time.sleep(1.0)
    
    def _process_packet(self, data):
        """ประมวลผล binary packet"""
        # Offset สำหรับ Joint Actual Position
        OFFSET_JOINT_ACTUAL = 432
        
        # อ่านค่า 6 joints (MG400 ใช้แค่ 4 ตัวแรก)
        q_all = struct.unpack_from('<6d', data, OFFSET_JOINT_ACTUAL)
        j1, j2, j3, j4 = q_all[0:4]
        
        # แปลงเป็น radians
        q_rad = np.radians([j1, j2, j3, j4])
        
        # --- Sanity Check ---
        if not self.kinematics.validate_sanity(self.last_valid_joints, q_rad):
            # ข้ามข้อมูลที่ผิดปกติ
            return
        
        self.last_valid_joints = q_rad
        self.current_position = q_rad
        
        # --- Parse Robot Mode ---
        # Offset 24 is Robot Mode (uint64)
        OFFSET_ROBOT_MODE = 24
        self.robot_mode = struct.unpack_from('<Q', data, OFFSET_ROBOT_MODE)[0]
        
        # --- คำนวณ Passive Joints ---
        all_joints = self.kinematics.calculate_passive_joints(q_rad)
        
        # --- Publish JointState ---
        msg = JointState()
        msg.header.stamp = self.clock.now().to_msg()
        msg.name = all_joints['names']
        msg.position = all_joints['positions']
        
        self.publisher.publish(msg)
    
    def get_robot_mode(self):
        """Thread-safe access to robot mode"""
        return getattr(self, 'robot_mode', 0)
    
    def stop(self):
        """หยุด thread"""
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)