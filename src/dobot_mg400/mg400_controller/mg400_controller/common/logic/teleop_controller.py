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

import numpy as np
from mg400_controller.common.config.robot_config import SPATIAL_THRESHOLD
from mg400_controller.common.config.motion_config import ( PROXIMITY_THRESHOLD,
     STUCK_VELOCITY_THRESHOLD, STUCK_TIME_THRESHOLD,
    TARGET_CHANGE_THRESHOLD, DYNAMIC_PROXIMITY_BASE_RAD, DYNAMIC_PROXIMITY_LOOKAHEAD_SEC,
    QUEUE_BACKLOG_GATE_RAD, QUEUE_BUSY_ESCAPE_SEC,
    REALTIME_PIPELINE_TARGET, REALTIME_PIPELINE_MAX, REALTIME_TAIL_CHANGE_RAD,
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
        self.queue_busy_start_time = 0.0
        self.pipeline_depth = 0
        self.queue_tail_target = None
        
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
        if velocity_mag < STUCK_VELOCITY_THRESHOLD and dist_to_target > PROXIMITY_THRESHOLD:
            if self.stuck_start_time == 0:
                self.stuck_start_time = now
            elif (now - self.stuck_start_time) > STUCK_TIME_THRESHOLD:
                return True # CONFIRMED STUCK
        else:
            self.stuck_start_time = 0
            
        return False

    def should_send_command(self, latest_target, q_current, now=None, queue_backlog_rad=None, run_queued_cmd=None):
        """
        The Core Decision Logic: Should we send a command?

        Args:
            latest_target: newest target joint position in radians
            q_current: current robot joint position in radians
            now: monotonic timestamp from the control loop (perf_counter)
        
        Returns:
            (bool, str): (Should Send?, Reason)
        """
        if now is None:
            now = self.last_robot_time
        
        # 0. First Run Check
        if self.last_sent_target is None:
            return True, "Init"

        # 1. Read the velocity already computed by the control loop using the
        # same monotonic clock source.
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

        should_send, reason = self._short_pipeline_decision(
            latest_target=latest_target,
            q_current=q_current,
            velocity_mag=velocity_mag,
            now=now,
            dist_to_last=dist_to_last,
            change_in_target=change_in_target,
            trigger_distance=trigger_distance,
            queue_backlog_rad=queue_backlog_rad,
            run_queued_cmd=run_queued_cmd,
        )
        if should_send or reason != "Continue":
            return should_send, reason
        
        if dist_to_last < trigger_distance:
            if change_in_target > SPATIAL_THRESHOLD:
                return True, f"DynProx_Dist{dist_to_last:.3f}_Thr{trigger_distance:.3f}"
        
        # 3. Strategy B: Velocity-Based Stuck Detection (Safety)
        # Robot stopped moving but hasn't reached target? Retrigger!
        
        # Throttle checks to 10Hz
        if (now - self.last_stuck_check_time) > 0.1:
            self.last_stuck_check_time = now
            
            error_to_last_target = np.max(np.abs(q_current - self.last_sent_target))
            
            if self.check_stuck_condition(velocity_mag, error_to_last_target, now):
                # Only trigger if user REALLY moved their hand OR if the robot is far from the current target
                if change_in_target > TARGET_CHANGE_THRESHOLD or error_to_last_target > PROXIMITY_THRESHOLD:
                    self.logger.warn(f"⚠️ Stuck Detected (Vel: {velocity_mag:.4f}) - Retriggering")
                    return True, f"Stuck_Vel{velocity_mag:.4f}_Delta{change_in_target:.3f}"

        return False, "Wait"

    def _short_pipeline_decision(
        self,
        *,
        latest_target,
        q_current,
        velocity_mag,
        now,
        dist_to_last,
        change_in_target,
        trigger_distance,
        queue_backlog_rad,
        run_queued_cmd,
    ):
        """Coalesce Unity samples into a tiny MG400 command pipeline.

        The goal is not to send on a timer.  We keep just enough queued work for
        CP blending (normally current + next), while replacing the host-side
        tail target with the newest meaningful Unity target before it enters
        the robot FIFO queue.
        """
        if self.queue_tail_target is None:
            self.queue_tail_target = self.last_sent_target.copy()

        queue_busy = bool(
            run_queued_cmd is not None
            and queue_backlog_rad is not None
            and run_queued_cmd
            and queue_backlog_rad > QUEUE_BACKLOG_GATE_RAD
        )

        # If the robot reports no running queue, our host-side estimate is stale.
        if not run_queued_cmd:
            self.pipeline_depth = 0
            self.queue_busy_start_time = 0
        else:
            tail_error = np.max(np.abs(q_current - self.queue_tail_target))
            if tail_error < trigger_distance:
                # The robot is already entering the last queued target's blend
                # window, so the short pipeline can be refilled with a fresh tail.
                self.pipeline_depth = 0
                self.queue_busy_start_time = 0

        tail_change = np.max(np.abs(latest_target - self.queue_tail_target))
        target_depth = max(1, int(REALTIME_PIPELINE_TARGET))
        max_depth = max(target_depth, int(REALTIME_PIPELINE_MAX))

        if self.pipeline_depth >= max_depth:
            if queue_busy:
                if self.queue_busy_start_time == 0:
                    self.queue_busy_start_time = now
                queue_busy_duration = now - self.queue_busy_start_time
                if (
                    velocity_mag < STUCK_VELOCITY_THRESHOLD
                    and queue_busy_duration > QUEUE_BUSY_ESCAPE_SEC
                ):
                    return False, "Continue"
            return False, f"PipelineFull_Depth{self.pipeline_depth}"

        if tail_change < REALTIME_TAIL_CHANGE_RAD:
            return False, f"TailSimilar_Delta{tail_change:.3f}"

        if self.pipeline_depth < target_depth:
            return True, f"PipelineFill_Depth{self.pipeline_depth}_Delta{tail_change:.3f}"

        if queue_busy:
            if self.queue_busy_start_time == 0:
                self.queue_busy_start_time = now
            return False, f"PipelineHold_Backlog{queue_backlog_rad:.3f}"

        # Let the legacy dynamic-proximity/stuck logic below handle edge cases
        # once the pipeline is full enough and the robot queue is shallow.
        return False, "Continue"

    def mark_command_sent(self, q_target, sent_time):
        """Update controller state only after the motion socket accepts the command."""
        self.last_sent_target = q_target.copy()
        self.queue_tail_target = q_target.copy()
        self.last_sent_time = sent_time
        self.stuck_start_time = 0
        self.queue_busy_start_time = 0
        max_depth = max(1, int(REALTIME_PIPELINE_MAX))
        self.pipeline_depth = min(max_depth, self.pipeline_depth + 1)

    def format_command_string(self, q_target, q_current=None, force_send=False):
        """
        Validate, Clamp, and Format Command String
        """
        # Validate & Clamp
        q_safe, is_clamped = self.validator.validate_and_clamp(q_target)
        if is_clamped:
            self.logger.warn("⚠️ Joint command exceeded limits - clamped to safe range")
            
        # Calculate speed
        speed_percent = 100 
        
        # โหมดปกติ (Single Point) เพื่อความลื่นไหลที่สุด
        cmd_str = self.planner.format_command(q_safe, speed_percent)
        
        return cmd_str, q_safe
