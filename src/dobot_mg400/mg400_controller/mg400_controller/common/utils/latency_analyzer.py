#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📊 Latency Analyzer
Responsible for tracking motion timestamps (T1-T5), calculating latency breakdowns,
validating arrival quality, and generating human-readable reports.

Extracted from vr_teleop_node.py for Clean Architecture.
"""

import time
import numpy as np

class LatencyAnalyzer:
    def __init__(self, config):
        """
        Initialize Analyzer
        Args:
            config: Module containing VALID_* threshold constants
        """
        self.cfg = config
        self.reset_tracking()
        
    def reset_tracking(self):
        """Reset all tracking variables for a new motion"""
        self.t1_unity_send = 0.0
        self.t2_ros_recv = 0.0
        self.t3_cmd_send = 0.0
        self.t4_motion_start = 0.0
        self.t5_target_reached = 0.0
        self.current_cmd_target = None
        self.is_tracking = False
        self.peak_velocity = 0.0
        self.initial_q = None

    def start_tracking(self, t1, t2, t3, target, current_q=None):
        """Start tracking a new command"""
        self.t1_unity_send = t1
        self.t2_ros_recv = t2
        self.t3_cmd_send = t3
        self.current_cmd_target = target.copy()
        self.t4_motion_start = 0.0
        self.t5_target_reached = 0.0
        self.is_tracking = True
        self.peak_velocity = 0.0
        self.initial_q = current_q.copy() if current_q is not None else None

    def update_tracking(self, velocity_mag):
        """Update tracking stats (called every loop)"""
        if self.is_tracking and self.t4_motion_start > 0 and self.t5_target_reached == 0:
            if velocity_mag > self.peak_velocity:
                self.peak_velocity = velocity_mag

    def mark_motion_start(self, now):
        """Mark T4: Robot started moving"""
        if self.is_tracking and self.t4_motion_start == 0.0:
            self.t4_motion_start = now
            return True
        return False

    def mark_target_reached(self, now):
        """Mark T5: Robot reached target"""
        if self.is_tracking and self.t5_target_reached == 0.0:
            self.t5_target_reached = now
            return True
        return False

    def analyze_arrival(self, q_current, velocity_mag):
        """
        Analyze arrival quality and calculate all latency metrics
        Returns: (metrics_dict, report_string)
        """
        if not self.is_tracking or self.t5_target_reached == 0.0:
            return None, ""

        # 1. Calc Latency Breakdowns
        t1, t2, t3, t4, t5 = self.t1_unity_send, self.t2_ros_recv, self.t3_cmd_send, self.t4_motion_start, self.t5_target_reached
        
        # Command Latency (Controllable)
        # Command Latency (Controllable)
        network_delay_ms = (t2 - t1) * 1000 if (t1 > 0 and t2 > 0) else 0.0
        decision_delay_ms = (t3 - t2) * 1000 if t2 > 0 else 0.0
        
        if t1 > 0:
            command_latency_ms = (t3 - t1) * 1000
        elif t2 > 0:
            command_latency_ms = (t3 - t2) * 1000
        else:
            command_latency_ms = 0.0
        
        # Motion Execution (Robot-Dependent)
        robot_response_ms = (t4 - t3) * 1000 if (t4 > 0 and t3 > 0) else 0.0
        
        if t4 > 0:
            motion_time_ms = (t5 - t4) * 1000
        elif t3 > 0:
            motion_time_ms = (t5 - t3) * 1000
        else:
            motion_time_ms = 0.0
            
        motion_execution_ms = (t5 - t3) * 1000 if t3 > 0 else 0.0
        
        # End-to-End
        if t1 > 0:
            true_end_to_end_ms = (t5 - t1) * 1000
        elif t2 > 0:
            true_end_to_end_ms = (t5 - t2) * 1000
        elif t3 > 0:
            true_end_to_end_ms = (t5 - t3) * 1000
        else:
            true_end_to_end_ms = 0.0
        
        # 2. Validation
        dist_to_target = np.linalg.norm(q_current - self.current_cmd_target)
        error_per_joint = np.abs(q_current - self.current_cmd_target)
        max_joint_error = np.max(error_per_joint)
        
        # Calculate Average Speed (Displacement / Time)
        displacement = np.linalg.norm(q_current - self.initial_q) if hasattr(self, 'initial_q') and self.initial_q is not None else 0.0
        avg_speed = displacement / (motion_time_ms / 1000.0) if motion_time_ms > 0 else 0.0
        
        final_error = dist_to_target
        is_valid_arrival = (
            final_error < self.cfg.VALID_FINAL_ERROR and
            max_joint_error < self.cfg.VALID_MAX_JOINT_ERROR and
            velocity_mag < self.cfg.VALID_VELOCITY and
            all(abs(err) < self.cfg.VALID_PER_JOINT_LIMIT for err in error_per_joint)
        )
        
        # 3. Pack Metrics
        metrics = {
            'now': time.time(),
            't1': t1, 't2': t2, 't3': t3, 't4': t4, 't5': t5,
            'network_ms': network_delay_ms,
            'decision_ms': decision_delay_ms,
            'command_ms': command_latency_ms,
            'response_ms': robot_response_ms,
            'motion_time_ms': motion_time_ms,
            'execution_ms': motion_execution_ms,
            'e2e_ms': true_end_to_end_ms,
            'target': self.current_cmd_target,
            'final_q': q_current,
            'final_error': final_error,
            'max_error': max_joint_error,
            'velocity': velocity_mag,
            'peak_velocity': self.peak_velocity,
            'avg_speed': avg_speed,
            'is_valid': is_valid_arrival
        }
        
        # 4. Generate CLI Report
        report = (
            f"\n📊 LATENCY BREAKDOWN (Target Reached {'✅' if is_valid_arrival else '⚠️'}):"
            f"\n   ━━━ Command Latency (Controllable) ━━━"
            f"\n   🌐 Network:          {network_delay_ms:6.1f} ms  (Unity→ROS)"
            f"\n   🧠 Decision:         {decision_delay_ms:6.1f} ms  (ROS processing)"
            f"\n   📤 Command Total:    {command_latency_ms:6.1f} ms  (Unity→Cmd Sent)"
            f"\n"
            f"\n   ━━━ Motion Execution (Robot-Dependent) ━━━"
            f"\n   🤖 Robot Response:   {robot_response_ms:6.1f} ms  (Cmd→Motion Start)"
            f"\n   🏃 Motion Time:      {motion_time_ms:6.1f} ms  (Moving)"
            f"\n   ⚙️  Execution Total:  {motion_execution_ms:6.1f} ms  (Cmd→Arrived)"
            f"\n"
            f"\n   ━━━ End-to-End ━━━"
            f"\n   ⚡ TRUE LATENCY:     {true_end_to_end_ms:6.1f} ms  (Unity→Arrived)"
            f"\n   📏 Final Error:      {final_error*57.3:.2f}° ({final_error:.4f} rad)"
            f"\n   🎯 Max Joint Error:  {max_joint_error*57.3:.2f}° ({max_joint_error:.4f} rad)"
            f"\n   🚀 Velocity Stats:   Peak={self.peak_velocity:.4f} rad/s, Avg={avg_speed:.4f} rad/s"
            f"\n   🛑 Arrival Vel:      {velocity_mag:.6f} rad/s"
        )
        
        return metrics, report

    def format_sent_report(self, should_send, reason, q_current, target, prev_target, prev_time, velocity, robot_mode=None, error_status=None):
        """Generate CLI report for SEND event"""
        if not should_send: return ""
        
        t1, t2, t3 = self.t1_unity_send, self.t2_ros_recv, self.t3_cmd_send
        now = time.time()
        
        network_delay_ms = (t2 - t1) * 1000 if (t1 > 0 and t2 > 0) else 0.0
        decision_delay_ms = (t3 - t2) * 1000 if t2 > 0 else 0.0
        
        err_deg = np.degrees(target - q_current)
        dist_to_last = np.linalg.norm(q_current - prev_target) if prev_target is not None else 0.0
        time_since_last = (now - prev_time) if prev_time > 0 else 0.0
        velocity_mag = np.linalg.norm(velocity)
        velocity_deg = np.degrees(velocity)
        
        mode_str = f" | Robot Mode: {robot_mode}" if robot_mode is not None else ""
        error_str = f" | Error Status: {error_status}" if error_status is not None else ""
        
        log_msg = (
            f"\n🚀 SENT [{reason}]{mode_str}{error_str}"
            f"\n   Network Delay: {network_delay_ms:.1f} ms (Unity→ROS)"
            f"\n   Decision Delay: {decision_delay_ms:.1f} ms (ROS processing)"
            f"\n   Time Since Last Cmd: {time_since_last*1000:.1f} ms"
            f"\n   Distance to Last Target: {dist_to_last:.3f} rad ({np.degrees(dist_to_last):.1f} deg)"
            f"\n   Robot Velocity: {velocity_mag:.4f} rad/s"
            f"\n     J1={velocity_deg[0]:+7.2f}°/s, J2={velocity_deg[1]:+7.2f}°/s, J3={velocity_deg[2]:+7.2f}°/s, J4={velocity_deg[3]:+7.2f}°/s"
            f"\n   Current Error (Target - Robot):"
            f"\n     J1={err_deg[0]:+7.2f}°, J2={err_deg[1]:+7.2f}°, J3={err_deg[2]:+7.2f}°, J4={err_deg[3]:+7.2f}°"
            f"\n   Robot Position:"
            f"\n     J1={np.degrees(q_current[0]):+7.2f}°, J2={np.degrees(q_current[1]):+7.2f}°, J3={np.degrees(q_current[2]):+7.2f}°, J4={np.degrees(q_current[3]):+7.2f}°"
            f"\n   Target Position:"
            f"\n     J1={np.degrees(target[0]):+7.2f}°, J2={np.degrees(target[1]):+7.2f}°, J3={np.degrees(target[2]):+7.2f}°, J4={np.degrees(target[3]):+7.2f}°"
        )
        return log_msg
