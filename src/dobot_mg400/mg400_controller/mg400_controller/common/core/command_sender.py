#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📤 Command Sender
ส่งคำสั่งการเคลื่อนที่ไปยังหุ่นยนต์

ใช้งาน:
    sender = CommandSender(connection, queue_manager, logger)
    sender.send_motion(command_string, speed_percent, distance)
"""

import time
import re
import queue
import threading

class CommandSender:
    def __init__(self, robot_connection, feedback_handler, logger):
        self.connection = robot_connection
        self.feedback = feedback_handler
        self.logger = logger
        
        # Dashboard Command Queue (to avoid blocking the caller/ROS executor)
        self.dash_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self._queue_worker, daemon=True)
        self.worker_thread.start()
        self.logger.info("🧵 Dashboard command worker thread started")
    
    def _queue_worker(self):
        """Processes dashboard commands in a separate thread"""
        while True:
            try:
                # Command is a tuple: (command_string, callback_if_any)
                cmd_item = self.dash_queue.get()
                if cmd_item is None: break
                
                cmd_str, port, status = cmd_item
                
                # Execute using blocking send_and_wait (safe here in background thread)
                response = self.connection.send_and_wait(cmd_str)
                success = response is not None and "0," in response
                
                if success:
                    state_str = "ON" if status else "OFF"
                    self.logger.info(f"🔌 [Async] DO Port {port} set to {state_str}")
                else:
                    self.logger.error(f"❌ [Async] Failed to set DO Port {port}: {response}")
                
            except Exception as e:
                self.logger.error(f"Error in dashboard worker: {e}")
            finally:
                self.dash_queue.task_done()

    def send(self, command):
        """
        ส่งคำสั่งการเคลื่อนที่ (Non-blocking on Port 30003)
        """
        # ส่งคำสั่ง
        success = self.connection.send_motion_cmd(command)
        
        if success:
            return True
        else:
            self.logger.error("❌ Failed to send motion command")
            return False

    def set_digital_output(self, port: int, status: bool) -> bool:
        """
        สั่งเปิด/ปิด Digital Output (Non-blocking via Queue)
        """
        status_val = 1 if status else 0
        command = f"DOExecute({port}, {status_val})"
        
        # Push to background queue to avoid blocking ROS executor
        self.dash_queue.put((command, port, status))
        return True

    def send_command_with_sync(self, command, timeout=5.0):
        """
        ส่งคำสั่งและรอจนกว่าหุ่นยนต์จะเริ่มทำและทำเสร็จ (Blocking with Sync)
        
        Returns:
            bool: True ถ้าทำสำเร็จตามเวลา
        """
        # 1. ส่งคำสั่งและรับ response string (เช่น "0, {123},")
        response = self.connection.send_and_wait(command)
        if not response:
            self.logger.warn("❌ No response from robot")
            return False
            
        # 2. แกะ Command ID จาก response
        cmd_id = self._parse_command_id(response)
        if cmd_id == -1:
            self.logger.warn(f"❌ Could not parse ID from: {response}")
            return False
            
        # 3. รอให้หุ่นรับรู้ ID นี้ (Wait for Execution Start)
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_id = self.feedback.get_command_id()
            if current_id == cmd_id:
                break
            time.sleep(0.01)
        else:
            self.logger.warn(f"⚠️ Timeout waiting for start ID: {cmd_id}")
            return False
            
        # 4. รอให้หุ่นทำเสร็จ (Wait for Completion)
        # เงื่อนไข: ID ยังเท่าเดิม และ Robot Mode กลับมาเป็น Idle (5)
        while time.time() - start_time < timeout:
            current_id = self.feedback.get_command_id()
            robot_mode = self.feedback.get_robot_mode()
            
            # ถ้า ID เปลี่ยนไปแล้ว แสดงว่ามีคำสั่งใหม่มาแทรก -> ถือว่าจบคำสั่งนี้
            if current_id != cmd_id:
                return True
                
            # ถ้า ID เท่าเดิม และ Mode = 5 (Idle) -> เสร็จแล้ว
            if robot_mode == 5:
                return True
                
            # ถ้า Mode = 9 (Error) -> จบเห่
            if robot_mode == 9:
                self.logger.error("❌ Robot Error during motion")
                return False
                
            time.sleep(0.01)
            
        self.logger.warn(f"⚠️ Timeout waiting for completion: {cmd_id}")
        return False

    def _parse_command_id(self, response):
        """แกะ ID จาก response string format 'error_id, {command_id},'"""
        try:
            # หาตัวเลขในปีกกา {}
            match = re.search(r'\{(\d+)\}', response)
            if match:
                return int(match.group(1))
            return -1
        except Exception:
            return -1