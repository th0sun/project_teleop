#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from collections import deque

class ClockCalibrator:
    """
    Handles synchronization between Unity (Remote) and ROS (Local) clocks.
    
    Features:
    1. Sliding Min-Window: Estimates absolute LAN delay by finding the fastest packets.
    2. Dynamic Drift Estimation: Automatically calculates clock drift using linear regression 
       on the 'min-delay' floor, removing the need for hardcoded values.
    3. Jump Detection: Resets if a massive system time shift (NTP update) occurs.
    """
    
    def __init__(self, window_size=50, regression_buffer_size=100):
        """
        Args:
            window_size: Number of samples to look back for the 'min' LAN floor.
            regression_buffer_size: Number of 'min-floor' points to keep for drift estimation.
        """
        self.window_size = window_size
        self.delay_history = deque(maxlen=window_size)
        
        # Linear Regression Buffer (t_ros, min_normalized_diff)
        self.regression_points = deque(maxlen=regression_buffer_size)
        
        self.drift_rate = 0.0 # Starts at 0, estimated dynamically
        self.base_offset = 0.0
        self.t_start_ros = 0.0
        self.is_calibrated = False
        
        # Jump detection
        self.last_raw_diff = None
        self.jump_threshold = 0.5 # 500ms jump triggers reset
        
    def calibrate(self, t1_unity, t2_ros):
        """
        Update calibration state with a new pair of timestamps.
        
        Args:
            t1_unity: Timestamp sent by Unity (Remote clock)
            t2_ros: Timestamp received by ROS (Local clock)
            
        Returns:
            corrected_t1: The estimated ROS time when Unity sent the packet.
        """
        raw_diff = t2_ros - t1_unity
        
        # 1. Initialize or Reset on Jump
        if not self.is_calibrated or (self.last_raw_diff is not None and abs(raw_diff - self.last_raw_diff) > self.jump_threshold):
            self.base_offset = raw_diff - 0.004 # Assume 4ms LAN floor for the very first packet
            self.t_start_ros = t2_ros
            self.drift_rate = 0.0
            self.delay_history.clear()
            self.regression_points.clear()
            self.is_calibrated = True
            
        self.last_raw_diff = raw_diff
        
        # 2. Calculate Drift-Corrected Difference
        elapsed_ros = t2_ros - self.t_start_ros
        drift_term = elapsed_ros * self.drift_rate
        
        # normalized_diff should be constant if drift_rate is correct
        normalized_diff = raw_diff - drift_term
        self.delay_history.append(normalized_diff)
        
        # 3. Find the minimum diff in current window (the jitter-free floor)
        stable_offset = min(self.delay_history)
        
        # 4. Periodically update Drift Rate using Linear Regression on stable floors
        # We store (t_ros, stable_offset) as a point for the slope calculation
        if len(self.delay_history) >= self.window_size:
            self.regression_points.append((elapsed_ros, stable_offset))
            
            # Re-estimate drift every 50 points
            if len(self.regression_points) >= 50:
                self._estimate_drift()
        
        # 5. Final Corrected T1
        # t1_ros = t1_unity + offset + drift
        corrected_t1 = t1_unity + stable_offset + drift_term
        
        return corrected_t1

    def _estimate_drift(self):
        """Perform linear regression to find the drift slope"""
        pts = np.array(self.regression_points)
        x = pts[:, 0] # elapsed_ros
        y = pts[:, 1] # stable_offset
        
        # Simple linear fit: y = m*x + c
        # The 'm' here is the *additional* drift rate we've observed
        if len(x) < 2: return
        
        coeffs = np.polyfit(x, y, 1)
        residual_drift = coeffs[0]
        
        # Update the total drift rate (accumulation)
        self.drift_rate += residual_drift
        
        # 🛡️ Anti-Overflow: Clamp drift rate to realistic local clock drift limits (±5ms/sec)
        # Also clean NaNs if polyfit failed
        if np.isnan(self.drift_rate):
            self.drift_rate = 0.0
        self.drift_rate = np.clip(self.drift_rate, -0.005, 0.005)
        
        # Clear regression points so we don't mix old data (before drift change) with new data
        self.regression_points.clear()

    def get_current_offset(self):
        """Returns the total current offset including drift"""
        if not self.is_calibrated: return 0.0
        elapsed = time.time() - self.t_start_ros
        return min(self.delay_history) + (elapsed * self.drift_rate)

if __name__ == "__main__":
    # Test Logic
    # Simulate a real drift of -0.00055 s/s
    REAL_DRIFT = -0.00055
    cal = ClockCalibrator()
    
    print("Testing Dynamic Drift Estimation Logic...")
    t1_start = 1000.0
    t2_start = 1000.750 + 0.004 # 750ms initial offset
    
    for i in range(200):
        # ROS time (t2) increases by 0.1s
        elapsed = i * 0.1
        t2 = t2_start + elapsed + np.random.uniform(0, 0.002) # Small jitter
        
        # Unity time (t1) increases, but with drift
        # t1 = start + elapsed - (elapsed * REAL_DRIFT)
        # Unity clock is faster -> sends 'future' timestamps
        t1 = t1_start + elapsed - (elapsed * REAL_DRIFT)
        
        c_t1 = cal.calibrate(t1, t2)
        
        if i % 20 == 0:
            print(f"Sample {i:3d}: Drift Est: {cal.drift_rate*1000:6.3f}ms/s | Delay: {(t2-c_t1)*1000:5.1f}ms")
    
    print(f"Final Estimated Drift: {cal.drift_rate*1000:.3f} ms/s (Target: {REAL_DRIFT*1000:.3f})")
