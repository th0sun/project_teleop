#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📤 Command Sender
ส่งคำสั่งการเคลื่อนที่ไปยังหุ่นยนต์

ใช้งาน:
    sender = CommandSender(connection, queue_manager, logger)
    sender.send_motion(command_string, speed_percent, distance)
"""

import re
import queue
import threading
from dataclasses import dataclass
from mg400_protocol.commands import do_execute


@dataclass(frozen=True)
class MotionSendResult:
    success: bool
    response: str | None = None
    command_id: int | None = None


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

    def send_with_command_id(self, command, response_timeout=0.02):
        """
        ส่งคำสั่งการเคลื่อนที่และพยายามเก็บ command id จาก response ทันที.

        This stays realtime-safe: it waits only for the bounded port-30003
        acknowledgement window, not for motion completion.
        """
        if hasattr(self.connection, "send_motion_cmd_with_response"):
            success, response = self.connection.send_motion_cmd_with_response(
                command,
                response_timeout=response_timeout,
            )
        else:
            success = self.connection.send_motion_cmd(command)
            response = None

        if not success:
            self.logger.error("❌ Failed to send motion command")
            return MotionSendResult(False, response=response)

        command_id = None
        if response:
            parsed_id = self._parse_command_id(response)
            if parsed_id != -1:
                command_id = parsed_id

        return MotionSendResult(True, response=response, command_id=command_id)

    def set_digital_output(self, port: int, status: bool) -> bool:
        """
        สั่งเปิด/ปิด Digital Output (Non-blocking via Queue)
        """
        command = do_execute(port, status).render()
        
        # Push to background queue to avoid blocking ROS executor
        self.dash_queue.put((command, port, status))
        return True


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
