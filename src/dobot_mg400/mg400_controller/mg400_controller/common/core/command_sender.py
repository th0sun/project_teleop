#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# project_teleop_ws/src/dobot_mg400/mg400_controller/mg400_controller/common/core/command_sender.py

"""
📤 Command Sender
ส่งคำสั่งการเคลื่อนที่ไปยังหุ่นยนต์

ใช้งาน:
    sender = CommandSender(connection, queue_manager, logger)
    sender.send_motion(command_string, speed_percent, distance)
"""

class CommandSender:
    def __init__(self, robot_connection, logger):
        self.connection = robot_connection
        self.logger = logger
    
    def send_motion(self, command, speed_percent, distance):
        """
        ส่งคำสั่งการเคลื่อนที่
        
        Args:
            command: คำสั่งที่จะส่ง (string)
            speed_percent: ความเร็ว (%)
            distance: ระยะทาง (radians)
        
        Returns:
            True ถ้าส่งสำเร็จ, False ถ้าล้มเหลว
        """
        # ส่งคำสั่ง
        success = self.connection.send_motion_cmd(command)
        
        if success:
            # Log (throttle ทุก 0.3 วินาที)
            self.logger.info(
                f"📤 Sent | Speed: {speed_percent}% | Dist: {distance:.4f}",
                throttle_duration_sec=0.3
            )
            return True
        else:
            self.logger.error("❌ Failed to send motion command")
            return False