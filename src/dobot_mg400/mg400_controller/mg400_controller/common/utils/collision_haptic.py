#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Collision-Based Haptic Feedback Module

Provides haptic feedback for Quest 3 VR controllers based on collision detection.
Uses built-in MG400 collision detection (sensorless via motor current).

Features:
- Monitor collision state from feedback packet
- Calculate haptic intensity for VR controllers
- Publish to ROS topic for Unity/Quest 3 integration
"""

import json
import time
from typing import Dict
from std_msgs.msg import String


class CollisionHaptic:
    """
    Collision-based haptic feedback for Quest 3 VR
    
    Uses MG400's built-in collision detection to trigger haptic feedback.
    When collision is detected, sends maximum haptic intensity to VR controllers.
    """
    
    def __init__(self, publisher, logger):
        """
        Initialize collision haptic handler
        
        Args:
            publisher: ROS publisher for haptic feedback data
            logger: ROS logger instance
        """
        self.publisher = publisher
        self.logger = logger
        
        # Haptic parameters
        self.COLLISION_HAPTIC_INTENSITY = 1.0  # Maximum haptic when collision detected
        self.COLLISION_HAPTIC_DURATION = 0.3   # Duration in seconds
        
        # State tracking
        self.last_collision_state = 0
        self.collision_start_time = 0.0
        
    def update_and_publish(self, collision_state: int) -> None:
        """
        Update collision state and publish haptic feedback
        
        Args:
            collision_state: Collision state from feedback packet (0 = no collision, >0 = collision)
        """
        # Detect collision transition
        if collision_state > 0 and self.last_collision_state == 0:
            # Collision just started
            self.collision_start_time = time.time()
            self.logger.warn(f"💥 Collision detected! Sending haptic feedback to VR")
        
        # Calculate haptic intensity
        haptic_intensity = self.get_haptic_intensity(collision_state)
        
        # Publish haptic message
        haptic_msg = {
            'collision_detected': bool(collision_state > 0),
            'collision_state': int(collision_state),
            'haptic_intensity': float(haptic_intensity),
            'timestamp': time.time()
        }
        
        # Create ROS String message
        msg = String()
        msg.data = json.dumps(haptic_msg)
        self.publisher.publish(msg)
        
        # Update last state
        self.last_collision_state = collision_state
    
    def get_haptic_intensity(self, collision_state: int) -> float:
        """
        Calculate haptic intensity based on collision state
        
        Args:
            collision_state: Collision state from robot (0 = none, >0 = collision)
            
        Returns:
            Haptic intensity (0.0-1.0) for Quest 3 controllers
        """
        if collision_state == 0:
            return 0.0
        
        # Collision detected - send maximum haptic
        # Optionally: decay over time after collision
        elapsed = time.time() - self.collision_start_time
        if elapsed > self.COLLISION_HAPTIC_DURATION:
            return 0.0  # Haptic expired
        
        # Linear decay over duration
        intensity = self.COLLISION_HAPTIC_INTENSITY * (1.0 - elapsed / self.COLLISION_HAPTIC_DURATION)
        return max(0.0, min(1.0, intensity))
    
    def get_haptic_message_format(self) -> Dict:
        """
        Get example haptic message format for documentation
        
        Returns:
            Example haptic message dictionary
        """
        return {
            'collision_detected': False,
            'collision_state': 0,
            'haptic_intensity': 0.0,
            'timestamp': 1738138200.0
        }
