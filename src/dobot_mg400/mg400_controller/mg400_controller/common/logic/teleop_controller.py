#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🧠 Teleop Controller (The Brain)
Entrusted with the decision-making logic: 
"Should we send a command to the robot now?"

Responsibilities:
1. Velocity Tracking (monitoring robot speed)
2. Proximity Check (sending commands when close to previous target)
3. Stuck Detection (recovering when robot stops unexpectedly)
4. Command Validation & Formatting (clamping and formatting strings)

Extracted from vr_teleop_node.py for Clean Architecture.
"""

import time
import numpy as np
import math
from mg400_controller.common.config.robot_config import SPATIAL_THRESHOLD
from mg400_controller.common.config.motion_config import (
    STUCK_VELOCITY_THRESHOLD, STUCK_TIME_THRESHOLD,
    TARGET_CHANGE_THRESHOLD, MAX_SPEED_DEG, DYNAMIC_PROXIMITY_BASE_RAD, DYNAMIC_PROXIMITY_LOOKAHEAD_SEC
)

class TeleopController:
    def __init__(self, validator, planner, logger):
        """
        Initialize Teleop Controller
        
        Args:
            validator: JointValidator instance
            planner: MotionPlanner instance
            logger: ROS logger
        """
        self.validator = validator
        self.planner = planner
        self.logger = logger
        
        # State Tracking
        self.last_sent_target = None
        self.last_sent_time = 0.0
        
        # Velocity Tracking
        self.last_robot_q = np.zeros(4)
        self.last_robot_time = 0.0
        self.robot_velocity = np.zeros(4)
        
        # Stuck Detection
        self.stuck_start_time = 0.0
        self.is_stuck = False
        self.last_stuck_check_time = 0.0
        self.last_stuck_trigger_time = 0.0  # Cooldown: prevent firing more than once per 2s
        
    def update_robot_state(self, q_current, now):
        """
        Update robot velocity based on current position and time
        Should be called every loop iteration.
        """
        dt = now - self.last_robot_time
        
        if dt > 0.001:  # Avoid division by zero
            # Calculate velocity (rad/s)
            delta_q = np.abs(q_current - self.last_robot_q)
            self.robot_velocity = delta_q / dt
            
            # Simple Low-pass Filter (Exponential Moving Average)
            alpha = 0.3
            self.robot_velocity = alpha * (delta_q / dt) + (1 - alpha) * self.robot_velocity
            
            self.last_robot_q = q_current.copy()
            self.last_robot_time = now
            
        return self.robot_velocity

    def check_stuck_condition(self, velocity_mag, dist_to_target, now):
        """
        Internal method to check if robot is stuck
        """
        # If moving slow AND far from target -> Potential Stuck
        if velocity_mag < STUCK_VELOCITY_THRESHOLD and dist_to_target > DYNAMIC_PROXIMITY_BASE_RAD:
            if self.stuck_start_time == 0:
                self.stuck_start_time = now
            elif (now - self.stuck_start_time) > STUCK_TIME_THRESHOLD:
                return True # CONFIRMED STUCK
        else:
            self.stuck_start_time = 0
            
        return False

    def should_send_command(self, latest_target, q_current):
        """
        The Core Decision Logic: Should we send a command?
        
        Returns:
            (bool, str): (Should Send?, Reason)
        """
        now = time.time()
        
        # 0. First Run Check
        if self.last_sent_target is None:
            self.last_sent_target = latest_target
            self.last_sent_time = now
            return True, "Init"
            
        # 1. Update State
        self.update_robot_state(q_current, now)
        velocity_mag = np.max(self.robot_velocity)
        
        # Calculate Distances
        dist_to_last = np.max(np.abs(q_current - self.last_sent_target))
        change_in_target = np.max(np.abs(latest_target - self.last_sent_target))
        
        # ========================================================
        # 🚀 STRATEGY A: VELOCITY-BASED DYNAMIC PROXIMITY
        # ========================================================
        # ดูจุดที่หุ่นกำลังพุ่งไป ถ้าความเร็วสูงมาก ระยะส่งต่อก็จะกว้าง(ไกล)ตาม
        
        # นี่คือเส้นสีแดงที่ถ้าหุ่นวิ่งข้ามเมื่อไหร่ เราจะสโลว์ดาวน์เป้าใหม่ทันที
        trigger_distance = DYNAMIC_PROXIMITY_BASE_RAD + (velocity_mag * DYNAMIC_PROXIMITY_LOOKAHEAD_SEC)
        
        if dist_to_last < trigger_distance:
            if change_in_target > SPATIAL_THRESHOLD:
                self.last_sent_target = latest_target
                self.last_sent_time = now
                self.stuck_start_time = 0 # Reset stuck timer
                return True, f"DynProx_Dist{dist_to_last:.3f}_Thr{trigger_distance:.3f}"
        
        # 3. Strategy B: Velocity-Based Stuck Detection (Safety)
        # Robot stopped moving but hasn't reached target? Retrigger!
        
        # Throttle checks to 10Hz
        if (now - self.last_stuck_check_time) > 0.1:
            self.last_stuck_check_time = now
            
            error_to_last_target = np.max(np.abs(q_current - self.last_sent_target))
            
            if self.check_stuck_condition(velocity_mag, error_to_last_target, now):
                # Only trigger if user REALLY moved their hand OR if the robot is far from the current target
                if change_in_target > TARGET_CHANGE_THRESHOLD or error_to_last_target > DYNAMIC_PROXIMITY_BASE_RAD:
                    # 🛡️ Cooldown: prevent stuck from firing more than once per 2s
                    if (now - self.last_stuck_trigger_time) < 2.0:
                        return False, "StuckCooldown"
                    self.logger.warn(f"⚠️ Stuck Detected (Vel: {velocity_mag:.4f}) - Retriggering")
                    self.last_sent_target = latest_target
                    self.last_sent_time = now
                    self.stuck_start_time = 0
                    self.last_stuck_trigger_time = now  # Start cooldown
                    return True, f"Stuck_Vel{velocity_mag:.4f}_Delta{change_in_target:.3f}"

        return False, "Wait"

    def format_command_string(self, q_target, q_current=None, force_send=False):
        """
        Validate, Clamp, and Format Command String
        
        If moving slowly, use Batch Interpolation for higher stability.
        force_send=True bypasses skip logic (used for Stuck recovery).
        """
        # Validate & Clamp
        q_safe, is_clamped = self.validator.validate_and_clamp(q_target)
        if is_clamped:
            self.logger.warn("⚠️ Joint command exceeded limits - clamped to safe range")
            
        # Calculate speed
        speed_percent = 100
        
        # Determine if we should use BATCH mode (Precision Mode)
        # Use batching if velocity is low < 0.1 rad/s
        velocity_mag = np.max(self.robot_velocity)
        
        if force_send:
            # 🛡️ Force single-point command (bypasses should_skip_motion in batch planner)
            # Always update last_command so next batch won't skip
            self.planner.last_command = q_safe.copy()
            cmd_str = self.planner.format_command(q_safe, speed_percent)
            return cmd_str, q_safe
        elif q_current is not None and velocity_mag < 0.1:
            # ใช้ 3 จุดย่อยสำหรับจังหวะเล็งละเอียด
            cmd_str, _, _ = self.planner.plan_batch_motion(q_safe, q_current, num_steps=3)
            return cmd_str, q_safe
        else:
            # โหมดปกติ (Single Point)
            cmd_str = self.planner.format_command(q_safe, speed_percent)
            return cmd_str, q_safe