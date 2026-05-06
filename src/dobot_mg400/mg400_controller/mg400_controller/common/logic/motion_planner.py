#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🎮 Motion Planner
วางแผนการเคลื่อนที่และคำนวณความเร็ว

ใช้งาน:
    planner = MotionPlanner()
    command = planner.plan_motion(target_joints, current_joints)
"""

import numpy as np
from mg400_protocol.commands import joint_mov_j, mov_j, mov_l
from mg400_controller.common.config.robot_config import SPATIAL_THRESHOLD
from mg400_controller.common.config.motion_config import *

class MotionPlanner:
    def __init__(self, control_mode, logger):
        self.control_mode = control_mode
        self.logger = logger
        self.last_command = None
    
    def should_skip_motion(self, q_target, q_current):
        """
        ตรวจสอบว่าควรข้ามคำสั่งนี้หรือไม่
        (เพราะเคลื่อนที่น้อยเกินไป)
        """
        if self.last_command is None:
            return False
        
        diff = np.abs(q_target - self.last_command)
        if np.max(diff) < SPATIAL_THRESHOLD:
            return True
        
        return False
    
    def calculate_speed(self, q_target, q_current):
        """
        คำนวณความเร็วที่เหมาะสมตามระยะทาง
        
        Returns:
            speed_percent (0-100)
        """
        distance = np.linalg.norm(q_target - q_current)
        
        if distance > SPEED_FAR_THRESHOLD:
            return SPEED_FAR
        elif distance > SPEED_MEDIUM_THRESHOLD:
            return SPEED_MEDIUM
        else:
            return SPEED_NEAR
    
    def format_command(self, q_rad, speed_percent):
        """
        สร้างคำสั่งตามรูปแบบที่กำหนด
        
        Returns:
            command string
        """
        q_deg = np.degrees(q_rad)
        
        if self.control_mode == "jointmovj":
            return joint_mov_j(
                q_deg[:4],
                speed_j=speed_percent,
                acc_j=ACC_VALUE,
                cp=CP_VALUE,
            ).render()
        
        elif self.control_mode == "movj":
            return mov_j(
                q_deg[:4],
                speed_j=speed_percent,
                acc_j=ACC_VALUE,
                cp=CP_VALUE,
            ).render()
        
        elif self.control_mode == "movl":
            return mov_l(
                q_deg[:4],
                speed_j=speed_percent,
                acc_j=ACC_VALUE,
                cp=CP_VALUE,
            ).render()
        
        return ""
    
    def plan_motion(self, q_target, q_current):
        """
        วางแผนการเคลื่อนที่
        
        Returns:
            (command_string, speed_percent, distance) หรือ (None, None, None) ถ้าควรข้าม
        """
        # ข้ามถ้าเคลื่อนที่น้อยเกินไป
        if self.should_skip_motion(q_target, q_current):
            return None, None, None
        
        # คำนวณความเร็ว
        distance = np.linalg.norm(q_target - q_current)
        speed = self.calculate_speed(q_target, q_current)
        
        # สร้างคำสั่ง
        command = self.format_command(q_target, speed)
        
        # บันทึกคำสั่งล่าสุด
        self.last_command = q_target.copy()
        
        return command, speed, distance

    def plan_batch_motion(self, q_target, q_current, num_steps=3, force_send=False):
        """
        วางแผนการเคลื่อนที่แบบชุด (Micro-interpolation)
        เพื่อความนิ่งสูงสุดในจังหวะเคลื่อนที่ละเอียด
        """
        # ข้ามถ้าเคลื่อนที่น้อยเกินไป เว้นแต่เป็น recovery/retrigger
        if not force_send and self.should_skip_motion(q_target, q_current):
            return None, None, None
            
        # 🐛 BUG FIX: Always interpolate from the robot's CURRENT position (q_current)
        # Using last_command can cause the robot to physically move backwards if 
        # last_command is stale and far behind the robot's actual position.
        q_start = q_current
        
        # คำนวณความเร็ว
        distance = np.linalg.norm(q_target - q_current)
        speed = self.calculate_speed(q_target, q_current)
        
        # สร้างชุดคำสั่งจุดย่อย
        commands = []
        steps = max(1, int(num_steps))
        for i in range(1, steps + 1):
            alpha = i / steps
            q_step = q_start + alpha * (q_target - q_start)
            cmd = self.format_command(q_step, speed)
            commands.append(cmd)
            
        # รวมคำสั่งด้วย semicolon
        batch_command = ";".join(commands)
        
        # บันทึกคำสั่งล่าสุด
        self.last_command = q_target.copy()
        
        return batch_command, speed, distance
