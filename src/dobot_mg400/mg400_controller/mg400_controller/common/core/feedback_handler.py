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
import time
from typing import Callable
import numpy as np
from mg400_protocol.feedback import FEEDBACK_PACKET_SIZE, parse_feedback_packet
from sensor_msgs.msg import JointState
from mg400_controller.common.utils.kinematics import KinematicsCalculator


SANITY_RESYNC_REJECT_COUNT = 3
SANITY_RESYNC_STABLE_THRESHOLD_RAD = 0.02


class FeedbackHandler:
    def __init__(
        self,
        robot_connection,
        joint_publisher,
        clock,
        logger,
        stop_event,
        feedback_callback: Callable[[dict], None] | None = None,
    ):
        self.connection = robot_connection
        self.publisher = joint_publisher
        self.clock = clock
        self.logger = logger
        self.stop_event = stop_event
        self.feedback_callback = feedback_callback
        
        self.kinematics = KinematicsCalculator()
        self.current_position = np.zeros(4)  # Active joints only
        self.target_position = np.zeros(4)
        self.last_valid_joints = np.zeros(4)
        self._last_rejected_joints = None
        self._sanity_reject_count = 0
        self.run_queued_cmd = 0
        self.command_id = 0
        
        self.thread = None
    
    def start(self):
        """เริ่ม thread รับ feedback"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.logger.info("📡 Feedback thread started")
    
    def get_current_position(self):
        """ดึงตำแหน่งปัจจุบัน (thread-safe)"""
        return self.current_position.copy()

    def get_target_position(self):
        """ดึงตำแหน่งเป้าหมายใน queue ของหุ่น (thread-safe)"""
        return self.target_position.copy()

    def get_queue_backlog(self):
        """ดึงระยะ backlog ระหว่าง QTarget และ QActual ของ 4 แกนหลัก"""
        return float(np.max(np.abs(self.target_position - self.current_position)))

    def get_run_queued_cmd(self):
        """ดึงสถานะว่าหุ่นกำลัง execute motion queue อยู่หรือไม่"""
        return int(getattr(self, 'run_queued_cmd', 0))
    
    def _run(self):
        """Main loop รับข้อมูล feedback"""
        buffer = b''
        
        # ตั้งค่า socket เป็น blocking mode
        self.connection.fb_sock.settimeout(None)
        
        self.logger.info("🎧 Listening for binary feedback (1440 bytes/packet)...")
        

        while not self.stop_event.is_set():
            try:
                # Read all currently available complete packets.  The control
                # state still ends on the newest packet, while the session CSV
                # can measure the real 30004 feedback cadence instead of a
                # decimated control-loop snapshot.
                self.connection.fb_sock.setblocking(False)
                packets = []
                while True:
                    try:
                        chunk = self.connection.fb_sock.recv(4096)
                        if not chunk: 
                            # Socket closed by peer
                            raise socket.error("Feedback socket closed by peer")
                        buffer += chunk
                    except BlockingIOError:
                        break
                
                # Process complete packets in wire order.  Keep any partial
                # packet in ``buffer`` for the next recv cycle.
                while len(buffer) >= FEEDBACK_PACKET_SIZE:
                    packets.append(buffer[:FEEDBACK_PACKET_SIZE])
                    buffer = buffer[FEEDBACK_PACKET_SIZE:]
                
                for packet in packets:
                    self._process_packet(packet)
                
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
        """ประมวลผล binary packet ผ่าน mg400_protocol feedback parser"""
        try:
            packet_wall_time = time.time()
            snapshot = parse_feedback_packet(data, allow_mock_zero_test_value=True)
            if snapshot is None:
                self.logger.warn(
                    "Packet Rejected: invalid MG400 feedback packet",
                    throttle_duration_sec=1.0,
                )
                return

            # 1. Parse Joint Angles (degrees in vendor packet -> radians internally)
            j1, j2, j3, j4 = snapshot.q_actual_deg
            q_rad = np.radians([j1, j2, j3, j4])
            
            # --- Sanity Check ---
            if not self.kinematics.validate_sanity(self.last_valid_joints, q_rad):
                stable_reject = (
                    self._last_rejected_joints is not None
                    and np.max(np.abs(q_rad - self._last_rejected_joints))
                    <= SANITY_RESYNC_STABLE_THRESHOLD_RAD
                )
                if stable_reject:
                    self._sanity_reject_count += 1
                else:
                    self._last_rejected_joints = q_rad.copy()
                    self._sanity_reject_count = 1

                if self._sanity_reject_count < SANITY_RESYNC_REJECT_COUNT:
                    self.logger.warn(f"Packet Rejected: Sanity check failed. Jump from {np.degrees(self.last_valid_joints)} to {np.degrees(q_rad)}", throttle_duration_sec=1.0)
                    return

                self.logger.warn(
                    "Feedback sanity resync after stable repeated jump. "
                    f"Old={np.degrees(self.last_valid_joints)} New={np.degrees(q_rad)}"
                )

            self._last_rejected_joints = None
            self._sanity_reject_count = 0
            
            self.last_valid_joints = q_rad
            self.current_position = q_rad

            # 1.5 Parse queue target and running-state feedback for backlog-aware gating.
            self.target_position = np.radians(snapshot.q_target_deg)
            self.run_queued_cmd = snapshot.run_queued_cmd
            
            # 2. Parse Robot Mode (Offset 24)
            self.robot_mode = snapshot.robot_mode
            
            # 3. Parse Digital I/O (Offset 8/16)
            self.di_status = snapshot.digital_inputs
            self.do_status = snapshot.digital_outputs
            
            # 🌟 3.5 Parse Speed Scaling (SpeedFactor) -> Offset 64 (float64)
            self.speed_scaling = snapshot.speed_scaling
            self.logger.info(f"🚀 SpeedFactor Confirm: {self.speed_scaling}", throttle_duration_sec=3.0)
            
            # 4. Parse Error/Collision status
            self.error_status = snapshot.error_status
            self.collision_state = snapshot.collision_state

            # 5. Parse Command ID (Offset 1112)
            self.command_id = snapshot.current_command_id
            
            # 6. Parse Tool Vector Actual (Offset 624) & Target (Offset 768)
            self.tool_vector_actual = np.array(snapshot.tool_vector_actual)
            self.tool_vector_target = np.array(snapshot.tool_vector_target)
            
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

            if self.feedback_callback is not None:
                try:
                    self.feedback_callback({
                        "timestamp": packet_wall_time,
                        "q_actual_rad": q_rad.copy(),
                        "q_target_rad": self.target_position.copy(),
                        "robot_mode": int(self.robot_mode),
                        "error_status": int(self.error_status),
                        "collision_state": int(self.collision_state),
                        "run_queued_cmd": int(self.run_queued_cmd),
                        "command_id": int(self.command_id),
                        "tool_vector_actual": self.tool_vector_actual.copy(),
                        "tool_vector_target": self.tool_vector_target.copy(),
                    })
                except Exception as exc:
                    self.logger.error(f"Feedback callback error: {exc}")
            
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
