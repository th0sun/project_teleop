#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔒 Joint Limit Validator
ตรวจสอบและจำกัดค่า Joint ให้อยู่ในขอบเขตที่ปลอดภัย

ใช้งาน:
    validator = JointValidator(logger)
    safe_joints, was_clamped = validator.validate_and_clamp(joints_rad)
"""

import numpy as np
from mg400_controller.common.config.robot_config import JOINT_LIMITS

class JointValidator:
    def __init__(self, logger):
        self.logger = logger
        self.last_warning_time = 0
    
    def validate_and_clamp(self, q_rad):
        """
        ตรวจสอบและปรับค่า Joint ให้อยู่ในขอบเขต
        
        Args:
            q_rad: numpy array ของมุม Joint (radians)
        
        Returns:
            (clamped_joints, was_clamped)
        """
        q_deg = np.degrees(q_rad)
        clamped = False
        
        for i in range(len(q_deg)):
            if i not in JOINT_LIMITS:
                continue
            
            min_deg, max_deg = JOINT_LIMITS[i]
            
            if q_deg[i] < min_deg:
                q_deg[i] = min_deg
                clamped = True
            elif q_deg[i] > max_deg:
                q_deg[i] = max_deg
                clamped = True
        
        if clamped:
            import time
            now = time.time()
            # แสดง warning ทุก 2 วินาที
            if now - self.last_warning_time > 2.0:
                self.logger.warn("⚠️ Joint command exceeded limits - clamped to safe range")
                self.last_warning_time = now
        
        return np.radians(q_deg), clamped
    
    def is_within_limits(self, q_rad):
        """ตรวจสอบว่า Joint อยู่ในขอบเขตหรือไม่"""
        q_deg = np.degrees(q_rad)
        
        for i in range(len(q_deg)):
            if i not in JOINT_LIMITS:
                continue
            min_deg, max_deg = JOINT_LIMITS[i]
            if q_deg[i] < min_deg or q_deg[i] > max_deg:
                return False
        
        return True
    
    def get_limits(self, joint_index):
        """ดึงขอบเขตของ Joint แต่ละตัว"""
        return JOINT_LIMITS.get(joint_index, (-180, 180))