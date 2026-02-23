#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trajectory Recording and Playback Module

Provides teach-and-repeat functionality for MG400 robot:
- Record robot trajectory during VR teleoperation
- Save/load trajectories to JSON files
- Replay trajectories with adjustable speed
- Progress monitoring

NOTE: This operates SEPARATELY from real-time teleoperation control.
"""

import time
import json
import numpy as np
from typing import List, Dict, Optional


class TrajectoryRecorder:
    """
    Trajectory recording and playback for teach-and-repeat mode
    
    Use cases:
    - Record a VR teleoperation session → replay later
    - Teach by demonstration
    - Create reusable motion sequences
    - Demo/testing mode
    
    This is SEPARATE from real-time VR teleoperation.
    """
    
    def __init__(self, feedback_handler, command_sender, logger):
        """
        Initialize trajectory recorder
        
        Args:
            feedback_handler: FeedbackHandler instance to read robot position
            command_sender: CommandSender instance to send motion commands
            logger: ROS logger instance
        """
        self.feedback = feedback_handler
        self.sender = command_sender
        self.logger = logger
        
        self.is_recording = False
        self.trajectory = []  # List of waypoints
        self.record_start_time = 0.0
        
        # Recording parameters
        self.RECORD_RATE = 10.0  # Hz - waypoint sampling rate
        
    def start_recording(self) -> None:
        """
        Start recording robot trajectory
        
        Call record_waypoint() at regular intervals during recording.
        """
        self.is_recording = True
        self.trajectory = []
        self.record_start_time = time.time()
        self.logger.info("🔴 Trajectory recording started...")
    
    def record_waypoint(self) -> bool:
        """
        Record current robot position as a waypoint
        
        Should be called at ~10Hz during recording.
        
        Returns:
            True if waypoint recorded, False if not recording
        """
        if not self.is_recording:
            return False
        
        # Get current joint positions
        q_current = self.feedback.get_current_position()
        if q_current is None:
            self.logger.warn("Cannot record waypoint: no current position")
            return False
        
        # Calculate elapsed time since recording started
        timestamp = time.time() - self.record_start_time
        
        waypoint = {
            'time': timestamp,
            'joints': q_current.tolist(),
            'timestamp_abs': time.time()
        }
        self.trajectory.append(waypoint)
        
        return True
    
    def stop_recording(self) -> List[Dict]:
        """
        Stop recording and return trajectory
        
        Returns:
            List of recorded waypoints
        """
        self.is_recording = False
        duration = self.trajectory[-1]['time'] if self.trajectory else 0
        self.logger.info(
            f"⏹️  Trajectory recording stopped\n"
            f"   Waypoints: {len(self.trajectory)}\n"
            f"   Duration: {duration:.2f}s"
        )
        return self.trajectory
    
    def save_trajectory(self, filename: str) -> bool:
        """
        Save trajectory to JSON file
        
        Args:
            filename: Path to save file
            
        Returns:
            True if successful
        """
        try:
            trajectory_data = {
                'trajectory': self.trajectory,
                'metadata': {
                    'waypoints': len(self.trajectory),
                    'duration': self.trajectory[-1]['time'] if self.trajectory else 0,
                    'recorded_at': self.trajectory[0]['timestamp_abs'] if self.trajectory else 0,
                    'record_rate_hz': self.RECORD_RATE
                }
            }
            
            with open(filename, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            
            self.logger.info(f"💾 Trajectory saved: {filename}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to save trajectory: {e}")
            return False
    
    def load_trajectory(self, filename: str) -> bool:
        """
        Load trajectory from JSON file
        
        Args:
            filename: Path to load file
            
        Returns:
            True if successful
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.trajectory = data['trajectory']
            metadata = data.get('metadata', {})
            
            self.logger.info(
                f"📂 Trajectory loaded: {filename}\n"
                f"   Waypoints: {metadata.get('waypoints', len(self.trajectory))}\n"
                f"   Duration: {metadata.get('duration', 0):.2f}s"
            )
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load trajectory: {e}")
            return False
    
    def replay_trajectory(self, speed_scale: float = 1.0, loop: bool = False) -> bool:
        """
        Replay recorded trajectory
        
        Args:
            speed_scale: Speed multiplier (1.0 = original speed, 0.5 = half speed, 2.0 = double speed)
            loop: If True, repeat trajectory continuously (use Ctrl+C to stop)
            
        Returns:
            True if replay completed successfully
        """
        if not self.trajectory:
            self.logger.error("No trajectory to replay")
            return False
        
        self.logger.info(
            f"▶️  Replaying trajectory\n"
            f"   Waypoints: {len(self.trajectory)}\n"
            f"   Speed: {speed_scale}x\n"
            f"   Loop: {loop}"
        )
        
        try:
            iteration = 0
            while True:
                iteration += 1
                if loop:
                    self.logger.info(f"Loop iteration: {iteration}")
                
                start_time = time.time()
                last_logged_progress = 0
                
                for i, waypoint in enumerate(self.trajectory):
                    # Calculate when this waypoint should be reached
                    target_time = waypoint['time'] / speed_scale
                    
                    # Wait until it's time
                    elapsed = time.time() - start_time
                    if elapsed < target_time:
                        time.sleep(target_time - elapsed)
                    
                    # Send command
                    joints = waypoint['joints']
                    command = f"JointMovJ({joints[0]},{joints[1]},{joints[2]},{joints[3]})"
                    
                    # Send non-blocking motion command
                    self.sender.send_motion(command, speed_percent=100, distance=0.0)
                    
                    # Log progress every 10%
                    progress = int((i / len(self.trajectory)) * 100)
                    if progress >= last_logged_progress + 10:
                        self.logger.info(f"Progress: {progress}%")
                        last_logged_progress = progress
                
                self.logger.info("✅ Trajectory replay complete")
                
                if not loop:
                    break
                    
        except KeyboardInterrupt:
            self.logger.info("Trajectory replay interrupted by user")
            return False
        except Exception as e:
            self.logger.error(f"Trajectory replay failed: {e}")
            return False
        
        return True
    
    def get_trajectory_info(self) -> Dict:
        """
        Get information about current trajectory
        
        Returns:
            Dictionary with trajectory metadata
        """
        if not self.trajectory:
            return {
                'loaded': False,
                'waypoints': 0,
                'duration': 0,
                'avg_sample_rate': 0
            }
        
        duration = self.trajectory[-1]['time']
        avg_rate = len(self.trajectory) / duration if duration > 0 else 0
        
        return {
            'loaded': True,
            'waypoints': len(self.trajectory),
            'duration': duration,
            'avg_sample_rate': avg_rate,
            'start_position': self.trajectory[0]['joints'],
            'end_position': self.trajectory[-1]['joints']
        }
    
    def clear_trajectory(self) -> None:
        """Clear current trajectory"""
        self.trajectory = []
        self.logger.info("Trajectory cleared")
