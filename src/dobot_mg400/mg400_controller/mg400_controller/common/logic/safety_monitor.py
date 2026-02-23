#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🛡️ Safety Monitor
Handles periodic safety checks:
- Motor temperatures
- Collision Detection
- Error Status (from Feedback or GetError API)
Extracted from vr_teleop_node.py
"""

import time
import json
import csv
import os
import numpy as np
from std_msgs.msg import String

class SafetyMonitor:
    def __init__(self, publisher, logger, error_handler=None):
        """
        Initialize Safety Monitor
        
        Args:
            publisher: ROS publisher for safety status topic
            logger: ROS logger
            error_handler: Optional ErrorHandler instance (if enabled)
        """
        self.publisher = publisher
        self.logger = logger
        self.error_handler = error_handler
        
        # Diagnostic Log
        self.log_file = os.path.expanduser("~/project_teleop_ws/robot_diagnostics.csv")
        self._init_csv_log()
        
    def _init_csv_log(self):
        """Create/Reset diagnostic log"""
        try:
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp", "RobotMode", "ErrorStatus", "CollisionState", 
                               "Joint1", "Joint2", "Joint3", "Joint4", "MaxTemp", "HasError"])
            self.logger.info(f"📝 Diagnostic Log created at: {self.log_file}")
        except Exception as e:
            self.logger.error(f"Failed to create diag log: {e}")
        
    def check_and_publish(self, feedback_handler):
        """
        Perform safety check and publish status
        
        Args:
            feedback_handler: Instance of FeedbackHandler to read raw data
        """
        # 1. Check Motor Temps
        temps = feedback_handler.get_motor_temperatures()
        max_temp = np.max(temps) if len(temps) > 0 else 0
        
        if max_temp > 65.0:
            self.logger.error(f"🔥 CRITICAL TEMP: {max_temp:.1f}°C - STOPPING ROBOT")
            # In a real scenario, we might trigger E-Stop here
        elif max_temp > 55.0:
             self.logger.warn(f"🌡️ High Temp: {max_temp:.1f}°C")
             
        # 2. Get Basic Status
        status = feedback_handler.get_error_status()
        
        # 3. Collision Check
        if status.get('collision_state', 0) > 0:
            self.logger.error(f"💥 COLLISION DETECTED! State: {status['collision_state']}")
            
        # 4. Advanced Error Checking using GetError() API (if enabled)
        # 4. Error Checking Strategy
        # Priority 1: Check real-time feedback (Fastest)
        basic_error_id = int(status.get('error_status', 0))
        robot_mode = int(status.get('robot_mode', 0))
        
        # Priority 2: Get detailed info via API (Slower but detailed)
        advanced_errors = []
        if self.error_handler:
            advanced_errors = self.error_handler.check_errors()
            if advanced_errors:
                 self.error_handler.log_errors(advanced_errors)
        
        # Combine: If feedback shows error but no detailed info yet, print basic warning
        # Combine checks
        has_error = (basic_error_id != 0 or robot_mode == 9)
        
        if has_error:
            if advanced_errors:
                pass # Already logged by error_handler
            else:
                 # CASE: Error Detected (Mode 9) but API returns NO INFO (Silent Error)
                 self.logger.error(f"🔴 Robot Error Detected! (Mode: {robot_mode})")
                 self.logger.error(f"⚠️  No Error Code received from Robot. Possible Causes:")
                 self.logger.error(f"   1. Joint Limit Reached (Check J1-J4)")
                 self.logger.error(f"   2. Singularity Point (Arm too stretched/folded)")
                 self.logger.error(f"   3. Workspace Violation")
        
        # 5. Publish Status for Dashboard (enhanced with error details)
        msg = {
            'timestamp': time.time(),
            'temps': temps.tolist() if isinstance(temps, np.ndarray) else list(temps),
            'collision': int(status.get('collision_state', 0)),
            'error_status': int(status.get('error_status', 0)),
            'robot_mode': int(status.get('robot_mode', 0)),
            'max_temp': float(max_temp),
            # Add detailed error information (if error_handler enabled)
            'errors': [self.error_handler.format_error_message(e) for e in advanced_errors] if (self.error_handler and advanced_errors) else [],
            'error_severity': self.error_handler.get_highest_severity(advanced_errors) if (self.error_handler and advanced_errors) else 0
        }
        
        self.publisher.publish(String(data=json.dumps(msg)))
        
        # 6. Log to CSV (Diagnostic)
        try:
            q_deg = np.degrees(feedback_handler.get_current_position())
            has_error = (basic_error_id != 0 or robot_mode == 9)
            
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f"{time.time():.3f}",
                    robot_mode,
                    basic_error_id,
                    status.get('collision_state', 0),
                    f"{q_deg[0]:.2f}", f"{q_deg[1]:.2f}", f"{q_deg[2]:.2f}", f"{q_deg[3]:.2f}",
                    f"{max_temp:.1f}",
                    "YES" if has_error else "NO"
                ])
        except Exception:
            pass # Ignore log errors to prevent loop crash
