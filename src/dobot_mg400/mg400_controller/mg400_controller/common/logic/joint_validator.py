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
class JointValidator:
    def __init__(self, joint_limits, elbow_limit, logger):
        """
        Initialize Joint Validator
        
        Args:
            joint_limits: Dict identifying min/max degrees for each joint
            elbow_limit: Tuple (min, max) for J3-J2 relative angle
            logger: ROS logger
        """
        self.limits = joint_limits
        self.elbow_limit = elbow_limit
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
        clamp_reasons = []
        
        # 0. Check Input Length (Must have 4 joints for MG400)
        if len(q_safe) < 4:
            self.logger.error(f"❌ JointValidator: Expected 4 joints, got {len(q_safe)}. Validation aborted.")
            return q_safe, False
        
        # 1. Clamp with Safety Margin for 4-decimal formatting (0.0001 deg)
        # Reason: MotionPlanner formats string with {:.4f}, so nextafter's 1e-16 diff disappears.
        # We need margin >= 0.0001 to ensure rounding doesn't hit the limit.
        FORMATTING_MARGIN = 0.0001
        
        is_clamped = False
        clamp_reasons = []
        
        # Protective check bounds against NaNs
        if np.any(np.isnan(q_safe)):
            self.logger.error(f"❌ JointValidator: Detected NaN in target joint positions. Validation aborted.")
            return q_safe, True

        for joint_id, (min_val, max_val) in self.limits.items():
            min_safe_deg = min_val + FORMATTING_MARGIN
            max_safe_deg = max_val - FORMATTING_MARGIN
            
            min_rad = np.radians(min_safe_deg)
            max_rad = np.radians(max_safe_deg)
            
            if q_safe[joint_id] < min_rad or q_safe[joint_id] > max_rad:
                orig_deg = np.degrees(q_safe[joint_id])
                q_safe[joint_id] = np.clip(q_safe[joint_id], min_rad, max_rad)
                new_deg = np.degrees(q_safe[joint_id])
                clamp_reasons.append(f"J{joint_id+1}({orig_deg:.2f}°->{new_deg:.2f}°)")
                was_clamped = True
        
        # 2. Validate Elbow Angle Constraint (J3 - J2)
        j2_deg = np.degrees(q_safe[1])  # J2 (Shoulder)
        j3_deg = np.degrees(q_safe[2])  # J3 (Elbow)
        elbow_angle = j3_deg - j2_deg   # Relative angle
        
        # Same margin for Elbow Limit
        elbow_min, elbow_max = self.elbow_limit
        safe_elbow_min = elbow_min + FORMATTING_MARGIN
        safe_elbow_max = elbow_max - FORMATTING_MARGIN
        
        if elbow_angle < safe_elbow_min or elbow_angle > safe_elbow_max:
            # Clamp J3 relative to J2
            elbow_clamped = np.clip(elbow_angle, safe_elbow_min, safe_elbow_max)
            j3_new_deg = j2_deg + elbow_clamped
            q_safe[2] = np.radians(j3_new_deg)
            clamp_reasons.append(f"Elbow({elbow_angle:.2f}°->{elbow_clamped:.2f}°)")
            was_clamped = True
        
        if clamp_reasons:
            import time
            now = time.time()
            if not hasattr(self, '_last_warn_time') or (now - self._last_warn_time) > 2.0:
                self.logger.warn(f"⚠️ Limit Exceeded [{', '.join(clamp_reasons)}] - Clamped")
                self._last_warn_time = now
        
        return q_safe, was_clamped

    
    def is_within_limits(self, q_rad):
        """ตรวจสอบว่า Joint อยู่ในขอบเขตหรือไม่"""
        q_deg = np.degrees(q_rad)
        
        for i in range(len(q_deg)):
            if i not in self.limits:
                continue
            min_deg, max_deg = self.limits[i]
            if q_deg[i] < min_deg or q_deg[i] > max_deg:
                return False
        
        return True
    
    def get_limits(self, joint_index):
        """ดึงขอบเขตของ Joint แต่ละตัว"""
        return self.limits.get(joint_index, (-180, 180))