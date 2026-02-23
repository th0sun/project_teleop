#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
📝 Teleop Logger
Handles CSV logging for performance metrics and analysis.
Extracted from vr_teleop_node.py for better separation of concerns.
"""

import os
import logging
from datetime import datetime

class TeleopLogger:
    def __init__(self, log_dir=None):
        """
        Initialize file logger for detailed CSV logging
        Creates two log files:
        1. Latency Breakdown (Arrivals analysis)
        2. Performance Metrics (Decision/Send events)
        
        Args:
            log_dir: Directory to save log files (default: ~/dobot_logs)
        """
        self.log_dir = os.path.expanduser(log_dir) if log_dir else os.path.expanduser("~/dobot_logs")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Setup Latency Logger
        self.latency_logger = self._setup_logger(
            "teleop_latency", 
            os.path.join(self.log_dir, f"teleop_latency_{timestamp}.csv"),
            (
                "Timestamp,T1_Unity_Send,T2_ROS_Recv,T3_Cmd_Send,T4_Motion_Start,T5_Target_Reached,"
                "Network_Delay_ms,Decision_Delay_ms,Command_Latency_ms,Robot_Response_ms,Motion_Time_ms,Motion_Execution_ms,True_End_to_End_ms,"
                "Q_Target_J1,Q_Target_J2,Q_Target_J3,Q_Target_J4,"
                "Q_Final_J1,Q_Final_J2,Q_Final_J3,Q_Final_J4,"
                "Final_Error_rad,Max_Joint_Error_rad,"
                "Velocity_at_Arrival_rad_s,Is_Valid_Arrival"
            )
        )
        
        # Setup Performance Logger
        self.perf_logger = self._setup_logger(
            "teleop_perf", 
            os.path.join(self.log_dir, f"teleop_struct_{timestamp}.csv"),
            (
                "Timestamp,T1_Unity_Send,T2_ROS_Recv,T3_Cmd_Send,"
                "Network_Delay_ms,Decision_Delay_ms,"
                "Q_Current_J1,Q_Current_J2,Q_Current_J3,Q_Current_J4,"
                "Q_Target_J1,Q_Target_J2,Q_Target_J3,Q_Target_J4,"
                "Dist_to_Last,Send_Reason,Time_Since_Last_ms,"
                "Velocity_Mag,Robot_Vel_J1,Robot_Vel_J2,Robot_Vel_J3,Robot_Vel_J4"
            )
        )
        
    def _setup_logger(self, name, file_path, header):
        """Internal method to setup a Python logger with file handler"""
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)
        logger.propagate = False # Prevent double logging if attached to root
        
        # Clear existing handlers if any (to avoid duplicates on restart)
        if logger.hasHandlers():
            logger.handlers.clear()
        
        # File Handler
        fh = logging.FileHandler(file_path)
        fh.setLevel(logging.INFO)
        
        # Simple formatter
        formatter = logging.Formatter('%(message)s')
        fh.setFormatter(formatter)
        
        logger.addHandler(fh)
        
        # Write Header
        logger.info(header)
        
        return logger

    def log_latency_breakdown(self, 
                              now, t1, t2, t3, t4, t5,
                              network_ms, decision_ms, command_ms,
                              response_ms, motion_time_ms, execution_ms,
                              e2e_ms,
                              target, final_q,
                              final_error, max_error, velocity, is_valid):
        """Log latency breakdown analysis (Called on Target Reached)"""
        log_msg = (
            f"{now:.4f},"
            f"{t1:.4f},{t2:.4f},{t3:.4f},{t4:.4f},{t5:.4f},"
            f"{network_ms:.2f},{decision_ms:.2f},{command_ms:.2f},"
            f"{response_ms:.2f},{motion_time_ms:.2f},{execution_ms:.2f},{e2e_ms:.2f},"
            f"{target[0]:.6f},{target[1]:.6f},{target[2]:.6f},{target[3]:.6f},"
            f"{final_q[0]:.6f},{final_q[1]:.6f},{final_q[2]:.6f},{final_q[3]:.6f},"
            f"{final_error:.6f},{max_error:.6f},"
            f"{velocity:.6f},{is_valid}"
        )
        self.latency_logger.info(log_msg)

    def log_performance_metrics(self, 
                                now, t1_unity_send, t2_ros_recv, t3_cmd_send,
                                network_delay_ms, decision_delay_ms,
                                q_current, latest_target,
                                dist_to_last, send_reason,
                                time_since_last, velocity_mag, robot_velocity):
        """Log performance/decision metrics (Called on Send)"""
        log_msg = (
            f"{now:.4f},"
            f"{t1_unity_send:.4f},{t2_ros_recv:.4f},{t3_cmd_send:.4f},"
            f"{network_delay_ms:.2f},{decision_delay_ms:.2f},"
            f"{q_current[0]:.6f},{q_current[1]:.6f},{q_current[2]:.6f},{q_current[3]:.6f},"
            f"{latest_target[0]:.6f},{latest_target[1]:.6f},{latest_target[2]:.6f},{latest_target[3]:.6f},"
            f"{dist_to_last:.6f},"
            f"{send_reason},"
            f"{time_since_last*1000:.2f},"
            f"{velocity_mag:.6f},"
            f"{robot_velocity[0]:.6f},{robot_velocity[1]:.6f},{robot_velocity[2]:.6f},{robot_velocity[3]:.6f}"
        )
        self.perf_logger.info(log_msg)
