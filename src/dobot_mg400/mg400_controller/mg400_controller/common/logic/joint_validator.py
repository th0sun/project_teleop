#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
✅ Joint Position Validator

Validate และ Clamp ค่า Joint Position ก่อนส่งไปหุ่นยนต์
- ตรวจสอบขีดจำกัดของแต่ละ Joint
- ตรวจสอบ Elbow Angle Constraint (J3 - J2)
- Clamp ค่าที่เกินขอบเขตให้อยู่ใน Safe Zone
"""

import numpy as np
from ..config.robot_config import JOINT_LIMITS, ELBOW_ANGLE_LIMIT

class JointValidator:
    def __init__(self, logger):
        self.logger = logger
        self.last_warning_time = 0
    
    def validate_and_clamp(self, q_target: np.ndarray) -> tuple[np.ndarray, bool]:
        """
        Validate และ Clamp Joint Positions
        
        Args:
            q_target: Target joint positions [J1, J2, J3, J4] (radians)
            
        Returns:
            tuple: (clamped_positions, was_clamped)
                - clamped_positions: ค่าที่ clamp แล้ว
                - was_clamped: True ถ้ามีการ clamp
        """
        q_safe = q_target.copy()
        was_clamped = False
        
        # 1. Clamp with Safety Margin for 4-decimal formatting (0.0001 deg)
        # Reason: MotionPlanner formats string with {:.4f}, so nextafter's 1e-16 diff disappears.
        # We need margin >= 0.0001 to ensure rounding doesn't hit the limit.
        FORMATTING_MARGIN = 0.0001
        
        for joint_id, (min_val, max_val) in JOINT_LIMITS.items():
            min_safe_deg = min_val + FORMATTING_MARGIN
            max_safe_deg = max_val - FORMATTING_MARGIN
            
            min_rad = np.radians(min_safe_deg)
            max_rad = np.radians(max_safe_deg)
            
            if q_safe[joint_id] <= min_rad or q_safe[joint_id] >= max_rad:
                q_safe[joint_id] = np.clip(q_safe[joint_id], min_rad, max_rad)
                was_clamped = True
        
        # 2. Validate Elbow Angle Constraint (J3 - J2)
        j2_deg = np.degrees(q_safe[1])  # J2 (Shoulder)
        j3_deg = np.degrees(q_safe[2])  # J3 (Elbow)
        elbow_angle = j3_deg - j2_deg   # Relative angle
        
        # Same margin for Elbow Limit
        elbow_min, elbow_max = ELBOW_ANGLE_LIMIT
        safe_elbow_min = elbow_min + FORMATTING_MARGIN
        safe_elbow_max = elbow_max - FORMATTING_MARGIN
        
        if elbow_angle < safe_elbow_min or elbow_angle > safe_elbow_max:
            # Clamp J3 relative to J2
            elbow_clamped = np.clip(elbow_angle, safe_elbow_min, safe_elbow_max)
            j3_new_deg = j2_deg + elbow_clamped
            q_safe[2] = np.radians(j3_new_deg)
            was_clamped = True
        
        if was_clamped:
            import time
            now = time.time()
            # แสดง warning ทุก 2 วินาที
            if now - self.last_warning_time > 2.0:
                self.logger.warn("⚠️ Joint command exceeded limits - clamped to safe range")
                self.last_warning_time = now
        
        return q_safe, was_clamped

    
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