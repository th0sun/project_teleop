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
import socket
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
                # 🔄 Flush buffer to get LATEST packet (As seen in dobot_api.py)
                self.connection.fb_sock.setblocking(False)
                latest_packet = None
                while True:
                    try:
                        chunk = self.connection.fb_sock.recv(4096)
                        if not chunk: 
                            # Socket closed by peer
                            raise socket.error("Feedback socket closed by peer")
                        buffer += chunk
                    except BlockingIOError:
                        break
                
                # Process only the LATEST complete packet from the buffer
                while len(buffer) >= PACKET_SIZE:
                    latest_packet = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]
                
                if latest_packet:
                    self._process_packet(latest_packet)
                
                # Small sleep to yield
                time.sleep(0.001)
                
            except (socket.error, OSError) as e:
                self.logger.error(f"📡 Feedback socket error: {e}. Reconnecting...")
                if self.connection._reconnect_port('fb'):
                    # Success reconnected
                    self.connection.fb_sock.settimeout(None)
                    buffer = b'' # Clear stale buffer
                else:
                    time.sleep(1.0) # Wait before retry
            except Exception as e:
                self.logger.error(f"Feedback loop error: {e}")
                time.sleep(0.5)

    
    def _process_packet(self, data):
        """ประมวลผล binary packet ตามโครงสร้าง MyType ใน dobot_api.py"""
        try:
            # 0. Packet Verification (Offset 48: TestValue)
            # Expecting 0x0123456789ABCDEF (Little Endian constant from Dobot)
            TEST_VALUE_OFFSET = 48
            test_val = struct.unpack_from('<Q', data, TEST_VALUE_OFFSET)[0]
            
            # 🛑 ZERO-LOCK FIX:
            # Real robots send 0x0123456789ABCDEF. Mocks send 0.
            # Accept both so we don't break the Mock Simulator.
            VALID_TEST_VALUES = (0x0123456789ABCDEF, 0)
            
            if test_val not in VALID_TEST_VALUES:
                self.logger.warn(f"Packet Rejected: TestValue mismatch. Got: {hex(test_val)}", throttle_duration_sec=1.0)
                return

            # 1. Parse Joint Angles (Offset 432)
            OFFSET_JOINT_ACTUAL = 432
            q_all = struct.unpack_from('<6d', data, OFFSET_JOINT_ACTUAL)
            j1, j2, j3, j4 = q_all[0:4]
            
            # 🛑 ZERO-LOCK FIX (Continued):
            # If the robot reconnects and sends a completely empty buffer, 
            # test_val is 0 AND the joints are exactly 0.0, 0.0, 0.0, 0.0.
            # We must reject this so it doesn't lock the Robot's Position at 0.
            if test_val == 0 and sum(abs(x) for x in q_all[0:4]) < 0.000001:
                self.logger.warn("Packet Rejected: Blank zero-buffer detected (Zero-Lock Prevention)", throttle_duration_sec=1.0)
                return
                
            q_rad = np.radians([j1, j2, j3, j4])
            
            # --- Sanity Check ---
            if not self.kinematics.validate_sanity(self.last_valid_joints, q_rad):
                self.logger.warn(f"Packet Rejected: Sanity check failed. Jump from {np.degrees(self.last_valid_joints)} to {np.degrees(q_rad)}", throttle_duration_sec=1.0)
                return
            
            self.last_valid_joints = q_rad
            self.current_position = q_rad
            
            # 2. Parse Robot Mode (Offset 24)
            OFFSET_ROBOT_MODE = 24
            self.robot_mode = struct.unpack_from('<Q', data, OFFSET_ROBOT_MODE)[0]
            
            # 3. Parse Digital I/O (Offset 8/16)
            OFFSET_DI_STATUS = 8
            OFFSET_DO_STATUS = 16
            self.di_status = struct.unpack_from('<Q', data, OFFSET_DI_STATUS)[0]
            self.do_status = struct.unpack_from('<Q', data, OFFSET_DO_STATUS)[0]
            
            # 🌟 3.5 Parse Speed Scaling (SpeedFactor) -> Offset 64 (float64)
            OFFSET_SPEED_SCALING = 64
            self.speed_scaling = struct.unpack_from('<d', data, OFFSET_SPEED_SCALING)[0]
            self.logger.info(f"🚀 SpeedFactor Confirm: {self.speed_scaling}", throttle_duration_sec=3.0)
            
            # 4. Parse Error/Collision status
            OFFSET_ERROR = 1029
            OFFSET_COLLISION = 1038
            self.error_status = data[OFFSET_ERROR]
            self.collision_state = data[OFFSET_COLLISION]

            # 5. Parse Command ID (Offset 1112)
            OFFSET_CMD_ID = 1112
            self.command_id = struct.unpack_from('<Q', data, OFFSET_CMD_ID)[0]
            
            # 6. Parse Tool Vector Actual (Offset 624) & Target (Offset 768)
            OFFSET_TOOL_ACTUAL = 624
            tool_actual = struct.unpack_from('<6d', data, OFFSET_TOOL_ACTUAL)
            self.tool_vector_actual = np.array(tool_actual)

            OFFSET_TOOL_TARGET = 768
            tool_target = struct.unpack_from('<6d', data, OFFSET_TOOL_TARGET)
            self.tool_vector_target = np.array(tool_target)
            
            # 7. Joint State Calculation & Publishing
            all_joints = self.kinematics.calculate_passive_joints(q_rad)
            msg = JointState()
            msg.header.stamp = self.clock.now().to_msg()
            msg.name = all_joints['names']
            msg.position = all_joints['positions']
            self.publisher.publish(msg)
            
            # 8. Flange XYZ — FK of actual joints (no tool offset)
            # Used by GUI to show tool offset = TCP_actual - flange_actual
            self.flange_actual = self.kinematics.forward_kinematics(
                [j1, j2, j3, j4]  # already in degrees from feedback packet
            )
            
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

    def get_flange_actual(self):
        """ดึงค่า Flange XYZ (FK ของ joint จริง ไม่รวม tool offset) [x, y, z, rx, 0, 0]"""
        return getattr(self, 'flange_actual', np.zeros(6))

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