#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📤 Command Sender
ส่งคำสั่งการเคลื่อนที่ไปยังหุ่นยนต์

ใช้งาน:
    sender = CommandSender(connection, queue_manager, logger)
    sender.send_motion(command_string, speed_percent, distance)
"""

class CommandSender:
    def __init__(self, robot_connection, queue_manager, logger):
        self.connection = robot_connection
        self.queue = queue_manager
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
        # ตรวจสอบคิว
        if not self.queue.can_add_command():
            return False
        
        # ส่งคำสั่ง
        success = self.connection.send_motion_cmd(command)
        
        if success:
            self.queue.add_command()
            
            # Log (throttle ทุก 0.3 วินาที)
            self.logger.info(
                f"📤 Sent | Queue: {self.queue.get_depth()} | "
                f"Speed: {speed_percent}% | Dist: {distance:.4f}",
                throttle_duration_sec=0.3
            )
            return True
        else:
            self.logger.error("❌ Failed to send motion command")
            return False