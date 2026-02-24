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
        """ประมวลผล binary packet Using Manual Offsets (Robust Method)"""
        try:
            # 1. Parse Joint Angles (Proven Offset 432)
            OFFSET_JOINT_ACTUAL = 432
            # อ่านค่า 6 joints (MG400 ใช้แค่ 4 ตัวแรก)
            q_all = struct.unpack_from('<6d', data, OFFSET_JOINT_ACTUAL)
            j1, j2, j3, j4 = q_all[0:4]
            
            # แปลงเป็น radians
            q_rad = np.radians([j1, j2, j3, j4])
            
            # --- Sanity Check ---
            if not self.kinematics.validate_sanity(self.last_valid_joints, q_rad):
                # self.logger.warn(f"⚠️ Sanity Check Failed: Jump detected")
                return
            
            self.last_valid_joints = q_rad
            self.current_position = q_rad
            
            # 2. Parse Robot Mode (Proven Offset 24)
            OFFSET_ROBOT_MODE = 24
            self.robot_mode = struct.unpack_from('<Q', data, OFFSET_ROBOT_MODE)[0]
            
            # 3. Parse V4 Extra Data (Manual Offsets)
            try:
                # Motor Temperatures (Offset 864)
                OFFSET_TEMPS = 864
                self.motor_temperatures = struct.unpack_from('<6d', data, OFFSET_TEMPS)
                
                # Collision State (Offset 1039)
                OFFSET_COLLISION = 1039
                self.collision_state = data[OFFSET_COLLISION]
                
                # Error Status (Offset 1030)
                OFFSET_ERROR = 1030
                self.error_status = data[OFFSET_ERROR]
                
                # Command ID (Offset 1112)
                OFFSET_CMD_ID = 1112
                self.command_id = struct.unpack_from('<Q', data, OFFSET_CMD_ID)[0]
                
                # Digital Input/Output Status (Offset 8/16, 64-bit mask for V4)
                OFFSET_DI_STATUS = 8
                OFFSET_DO_STATUS = 16
                self.di_status = struct.unpack_from('<Q', data, OFFSET_DI_STATUS)[0]
                self.do_status = struct.unpack_from('<Q', data, OFFSET_DO_STATUS)[0]
                
            except Exception as e:
                self.logger.warn(f"Extra data parse error: {e}")
            
            # 4. Parse Tool Vector Actual (Offset 624) & Target (Offset 768)
            try:
                OFFSET_TOOL_ACTUAL = 624
                # Parse [x, y, z, rx, ry, rz]
                tool_actual = struct.unpack_from('<6d', data, OFFSET_TOOL_ACTUAL)
                self.tool_vector_actual = np.array(tool_actual)

                OFFSET_TOOL_TARGET = 768
                tool_target = struct.unpack_from('<6d', data, OFFSET_TOOL_TARGET)
                self.tool_vector_target = np.array(tool_target)

            except Exception as e:
                self.logger.warn(f"Tool Vector parse error: {e}")
            
            # 5. คำนวณ Passive Joints & Publish
            all_joints = self.kinematics.calculate_passive_joints(q_rad)
            
            # --- Publish JointState ---
            msg = JointState()
            msg.header.stamp = self.clock.now().to_msg()
            msg.name = all_joints['names']
            msg.position = all_joints['positions']
            
            self.publisher.publish(msg)
            
        except Exception as e:
            self.logger.error(f"Packet processing error: {e}")
    
    def get_robot_mode(self):
        """Thread-safe access to robot mode"""
        return getattr(self, 'robot_mode', 0)
        
    def get_command_id(self):
        """ดึง ID คำสั่งล่าสุดที่หุ่นยนต์ทำเสร็จแล้ว (ใช้สำหรับ Sync)"""
        return getattr(self, 'command_id', 0)

    def get_tool_vector(self):
        """ดึงค่า Tool Vector ล่าสุด (Actual) [x, y, z, rx, ry, rz]"""
        return getattr(self, 'tool_vector_actual', np.zeros(6))

    def get_target_tool_vector(self):
        """ดึงค่า Tool Vector เป้าหมาย (Target) [x, y, z, rx, ry, rz]"""
        return getattr(self, 'tool_vector_target', np.zeros(6))

    def get_error_status(self):
        """ดึงสถานะ Error และ Collision"""
        return {
            'error_status': getattr(self, 'error_status', 0),
            'collision_state': getattr(self, 'collision_state', 0),
            'robot_mode': self.get_robot_mode()
        }

    def get_motor_temperatures(self):
        """ดึงอุณหภูมิมอเตอร์ทั้ง 6 แกน"""
        return np.array(getattr(self, 'motor_temperatures', []))

    def get_do_status(self):
        """ดึงสถานะ Digital Output ทั้งหมด (Bitmask)"""
        return getattr(self, 'do_status', 0)

    def stop(self):
        """หยุด thread"""
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)