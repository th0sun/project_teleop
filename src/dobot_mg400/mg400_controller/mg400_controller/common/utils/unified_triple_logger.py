#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔬 Unified Triple-Layer Logger (Unity → ROS2 → Robot)

บันทึกข้อมูลจาก 3 เลเยอร์พร้อมกัน เพื่อ validate ความตรงกัน:
- Layer 1: Unity (target ที่ส่งมาจาก VR/Controller)
- Layer 2: ROS2 (command ที่ ROS ส่งให้หุ่น)
- Layer 3: Robot (actual position ที่หุ่นรายงานกลับ)

File format: unified_triple_log_YYYYMMDD_HHMMSS.csv
"""

import os
import csv
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple, List
import numpy as np


class UnifiedTripleLogger:
    """
    Synchronized 3-layer logger for cross-validation between
    Unity virtual model, ROS planning layer, and real robot state.
    """
    
    def __init__(self, log_dir: str = None):
        """
        Initialize triple-layer logger
        
        Args:
            log_dir: Directory to save log files (default: project_teleop/logs/triple_layer)
        """
        if log_dir is None:
            # Default: save to project_teleop/logs/triple_layer (relative to CWD)
            log_dir = os.path.abspath("./logs/triple_layer")
        else:
            log_dir = os.path.expanduser(log_dir)
        
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = self.log_dir / f"unified_triple_log_{timestamp}.csv"
        
        # CSV file handle
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        
        # Sample counter
        self._sample_count = 0
        self._start_time = time.time()
        
        # Initialize CSV
        self._init_csv()
        
    def _init_csv(self):
        """Initialize CSV file with headers"""
        self._file = open(self.file_path, mode='w', newline='')
        self._writer = csv.writer(self._file)
        
        # Header: timestamp + 3 layers × 4 joints
        header = [
            'sample_id',
            'elapsed_sec',
            'ros_timestamp',
            # Layer 1: Unity (Input from user/VR)
            'unity_j1_deg', 'unity_j2_deg', 'unity_j3_deg', 'unity_j4_deg',
            # Layer 2: ROS2 (Command sent to robot)
            'ros_cmd_j1_deg', 'ros_cmd_j2_deg', 'ros_cmd_j3_deg', 'ros_cmd_j4_deg',
            # Layer 3: Robot (Actual feedback)
            'robot_j1_deg', 'robot_j2_deg', 'robot_j3_deg', 'robot_j4_deg',
            # Deltas for validation
            'delta_unity_ros_j1', 'delta_unity_ros_j2', 'delta_unity_ros_j3', 'delta_unity_ros_j4',
            'delta_ros_robot_j1', 'delta_ros_robot_j2', 'delta_ros_robot_j3', 'delta_ros_robot_j4',
        ]
        self._writer.writerow(header)
        self._file.flush()
        
    def log_sample(
        self,
        unity_joints: Optional[Tuple[float, float, float, float]] = None,
        ros_cmd_joints: Optional[Tuple[float, float, float, float]] = None,
        robot_joints: Optional[Tuple[float, float, float, float]] = None,
        ros_timestamp: Optional[float] = None,
    ):
        """
        Log one synchronized sample from all 3 layers
        
        Args:
            unity_joints: (j1,j2,j3,j4) in degrees from Unity
            ros_cmd_joints: (j1,j2,j3,j4) in degrees that ROS sent
            robot_joints: (j1,j2,j3,j4) in degrees from robot feedback
            ros_timestamp: ROS time when sample was captured
        """
        with self._lock:
            self._sample_count += 1
            elapsed = time.time() - self._start_time
            ros_ts = ros_timestamp if ros_timestamp else time.time()
            
            # Convert to lists (handle None)
            unity = list(unity_joints) if unity_joints else [None, None, None, None]
            ros_cmd = list(ros_cmd_joints) if ros_cmd_joints else [None, None, None, None]
            robot = list(robot_joints) if robot_joints else [None, None, None, None]
            
            # Calculate deltas (only if both values exist)
            delta_unity_ros = []
            delta_ros_robot = []
            
            for i in range(4):
                # Unity vs ROS
                if unity[i] is not None and ros_cmd[i] is not None:
                    delta_unity_ros.append(unity[i] - ros_cmd[i])
                else:
                    delta_unity_ros.append(None)
                    
                # ROS vs Robot
                if ros_cmd[i] is not None and robot[i] is not None:
                    delta_ros_robot.append(ros_cmd[i] - robot[i])
                else:
                    delta_ros_robot.append(None)
            
            row = [
                self._sample_count,
                f"{elapsed:.6f}",
                f"{ros_ts:.6f}",
                # Unity
                *[f"{v:.4f}" if v is not None else "" for v in unity],
                # ROS Command
                *[f"{v:.4f}" if v is not None else "" for v in ros_cmd],
                # Robot
                *[f"{v:.4f}" if v is not None else "" for v in robot],
                # Deltas
                *[f"{v:.4f}" if v is not None else "" for v in delta_unity_ros],
                *[f"{v:.4f}" if v is not None else "" for v in delta_ros_robot],
            ]
            
            self._writer.writerow(row)
            
            # Flush every 100 samples
            if self._sample_count % 100 == 0:
                self._file.flush()
                
    def log_unity_only(self, unity_joints: Tuple[float, float, float, float], ros_timestamp: float):
        """Log when only Unity data is available (no command sent yet)"""
        self.log_sample(unity_joints=unity_joints, ros_timestamp=ros_timestamp)
        
    def log_ros_cmd(self, ros_cmd_joints: Tuple[float, float, float, float], unity_joints: Optional[Tuple] = None, ros_timestamp: float = None):
        """Log when ROS sends command to robot"""
        self.log_sample(unity_joints=unity_joints, ros_cmd_joints=ros_cmd_joints, ros_timestamp=ros_timestamp)
        
    def log_robot_feedback(self, robot_joints: Tuple[float, float, float, float], ros_cmd_joints: Optional[Tuple] = None, ros_timestamp: float = None):
        """Log when robot feedback arrives"""
        self.log_sample(ros_cmd_joints=ros_cmd_joints, robot_joints=robot_joints, ros_timestamp=ros_timestamp)
        
    def log_full_sync(
        self,
        unity_joints: Tuple[float, float, float, float],
        ros_cmd_joints: Tuple[float, float, float, float],
        robot_joints: Tuple[float, float, float, float],
        ros_timestamp: float,
    ):
        """Log complete synchronized sample (all 3 layers)"""
        self.log_sample(unity_joints, ros_cmd_joints, robot_joints, ros_timestamp)
        
    def close(self):
        """Close log file"""
        with self._lock:
            if self._file and not self._file.closed:
                self._file.flush()
                self._file.close()
                
    def get_summary(self) -> str:
        """Get summary of logged data"""
        elapsed = time.time() - self._start_time
        return (
            f"🔬 Triple-Layer Log: {self._sample_count} samples "
            f"({elapsed:.1f}s) → {self.file_path.name}"
        )
        
    def __del__(self):
        """Destructor - ensure file is closed"""
        self.close()


# ── Interactive Prompt ─────────────────────────────────────────────────

def prompt_enable_triple_logging() -> bool:
    """
    Interactive prompt to ask user if they want to enable 3-layer logging
    Returns: True if logging should be enabled
    """
    print("\n" + "=" * 60)
    print("🔬 Unified Triple-Layer Logger")
    print("=" * 60)
    print("\nบันทึกข้อมูลจาก 3 เลเยอร์พร้อมกัน:")
    print("  Layer 1: Unity (VR/Controller target)")
    print("  Layer 2: ROS2 (Command sent to robot)")
    print("  Layer 3: Robot (Actual feedback)")
    print("\nใช้สำหรับ:")
    print("  • Validate ความตรงกันระหว่างโมเดลกับหุ่นจริง")
    print("  • วิเคราะห์ latency และ tracking error")
    print("  • สร้างรีพอร์ตเปรียบเทียบ")
    print("\nไฟล์บันทึก: ~/project_teleop_ws/logs/triple_layer/unified_triple_log_YYYYMMDD_HHMMSS.csv")
    print("=" * 60)
    
    response = input("\nEnable 3-layer logging? [Y/n]: ").strip().lower()
    return response in ('', 'y', 'yes', 'yes')


# ── Analysis Tools ─────────────────────────────────────────────────────

def analyze_triple_log(file_path: str) -> dict:
    """
    Analyze a triple-layer log file and return statistics
    
    Returns dict with:
        - total_samples
        - avg_delta_unity_ros (per joint)
        - avg_delta_ros_robot (per joint)
        - max_delta_ros_robot (per joint)
    """
    import pandas as pd
    
    df = pd.read_csv(file_path)
    
    results = {
        'total_samples': len(df),
        'duration_sec': df['elapsed_sec'].max() if len(df) > 0 else 0,
    }
    
    # Calculate statistics for deltas
    for i in range(1, 5):
        unity_ros_col = f'delta_unity_ros_j{i}'
        ros_robot_col = f'delta_ros_robot_j{i}'
        
        if unity_ros_col in df.columns:
            valid_data = df[df[unity_ros_col].notna()][unity_ros_col]
            if len(valid_data) > 0:
                results[f'j{i}_unity_ros_mean'] = valid_data.mean()
                results[f'j{i}_unity_ros_max'] = valid_data.abs().max()
                
        if ros_robot_col in df.columns:
            valid_data = df[df[ros_robot_col].notna()][ros_robot_col]
            if len(valid_data) > 0:
                results[f'j{i}_ros_robot_mean'] = valid_data.mean()
                results[f'j{i}_ros_robot_max'] = valid_data.abs().max()
    
    return results


if __name__ == "__main__":
    # Test the logger
    if prompt_enable_triple_logging():
        logger = UnifiedTripleLogger()
        
        # Simulate some data
        for i in range(10):
            unity = (i * 0.5, i * 0.3, i * 0.2, i * 0.1)
            ros_cmd = (i * 0.5 + 0.01, i * 0.3 + 0.01, i * 0.2 + 0.01, i * 0.1 + 0.01)
            robot = (i * 0.5 + 0.02, i * 0.3 + 0.02, i * 0.2 + 0.02, i * 0.1 + 0.02)
            
            logger.log_full_sync(unity, ros_cmd, robot, time.time())
            time.sleep(0.1)
            
        print(logger.get_summary())
        logger.close()
        print(f"\n✅ Log saved to: {logger.file_path}")
