import numpy as np
import math

class TargetPredictor:
    """
    Predictive Kalman Filter for VR Teleoperation targets.
    This class tracks position and velocity of the 4 robot joints.
    It projects the predicted target 'horizon' seconds into the future
    to compensate for the physical acceleration lag of the real robot motors.
    """
    def __init__(self, default_dt=0.02, prediction_horizon_sec=0.08, logger=None):
        self.dt = default_dt
        self.horizon = prediction_horizon_sec
        self.logger = logger
        self.last_timestamp = 0.0
        
        # State vector: [position, velocity]^T for 4 joints
        # Shape: (4 joints, 2 variables, 1 vector dimension) -> (4, 2, 1)
        self.x = np.zeros((4, 2, 1)) 
        
        # Covariance Matrix P
        self.P = np.stack([np.eye(2)] * 4) # (4, 2, 2)
        
        # State Transition Matrix F
        self.F = np.array([
            [1.0, self.dt],
            [0.0, 1.0]
        ])
        
        # Measurement Matrix H (We only measure Position from Unity, not raw Velocity)
        self.H = np.array([[1.0, 0.0]])
        
        # Process Noise Covariance Q (Trust in the mathematical model)
        # We expect velocity to change frequently (human hand movement)
        self.Q = np.array([
            [1e-4, 0.0],
            [0.0, 2e-2]
        ])
        
        # Measurement Noise Covariance R (Trust in Unity sensor data)
        # VR tracking is generally very accurate but has slight jitter
        self.R = np.array([[1e-3]])
        
        self.I = np.eye(2)
        self.is_initialized = False

    def update_and_predict(self, raw_target_q, timestamp_sec, q_actual=None):
        """
        Takes the raw [4,] joint target from Unity and the package timestamp.
        Optionally takes q_actual from robot to scale the prediction horizon.
        Returns the [4,] predicted future joint target.
        """
        # 0. Initialize on first frame
        if not self.is_initialized:
            self.x[:, 0, 0] = raw_target_q
            self.x[:, 1, 0] = 0.0
            self.last_timestamp = timestamp_sec
            self.is_initialized = True
            return raw_target_q
            
        # --- DYNAMIC DT CALCULATION (For Network Jitter / Tailscale) ---
        dt = timestamp_sec - self.last_timestamp
        self.last_timestamp = timestamp_sec
        
        # Guard against zero/negative dt (out of order packets) or massive lag spikes >0.5s
        if dt <= 0.001 or dt > 0.5:
            dt = self.dt # Fallback to default
            
        # Update State Transition Matrix F with real dt
        self.F[0, 1] = dt
            
        # 📝 Kalman Filter Loop for each joint
        for i in range(4):
            # --- 1. PREDICT STEP ---
            # Project state ahead
            self.x[i] = self.F @ self.x[i]
            # Project error covariance ahead
            self.P[i] = self.F @ self.P[i] @ self.F.T + self.Q
            
            # --- 2. UPDATE STEP ---
            z = np.array([[raw_target_q[i]]]) # Observation (1, 1)
            
            # Innovation (Difference between measurement and prediction)
            y = z - self.H @ self.x[i] 
            
            # Innovation covariance
            S = self.H @ self.P[i] @ self.H.T + self.R 
            
            # Kalman Gain
            K = self.P[i] @ self.H.T @ np.linalg.inv(S) 
            
            # Update state estimate
            self.x[i] = self.x[i] + K @ y
            
            # Update error covariance
            self.P[i] = (self.I - K @ self.H) @ self.P[i]

        # --- 3. SAFETY OVERRIDE (Anti-Overshoot) ---
        velocities = self.x[:, 1, 0]
        max_vel = np.max(np.abs(velocities))
        
        # Deadband: If highest joint velocity is < 1.0 deg/sec
        # We classify this as "The hand has stopped".
        # Force the output to the exact physical hand position to prevent drift.
        if max_vel < math.radians(1.0):
            return raw_target_q
            
        # --- 4. PREDICT FUTURE HORIZON ---
        # Anti-overshoot for large point-to-point jumps (e.g., Teach & Repeat)
        # If the target is very far away from the current state (a sudden jump),
        # do not predict into the future because that causes the command to overshoot.
        position_error = np.max(np.abs(raw_target_q - self.x[:, 0, 0]))
        if position_error > 0.3: # ~17 degrees
            return raw_target_q
            
        # --- 🌊 DAMPENED DYNAMIC HORIZON ---
        # Scale the horizon based on how far the robot is from the target.
        # If the robot is close (low tracking error), reduce prediction to gain precision.
        # If the robot is far (high tracking error), use full prediction to gain speed.
        final_horizon = self.horizon
        
        if q_actual is not None and np.any(q_actual != 0.0):
            # Tracking Error = Distance between current hand position and robot actual position
            tracking_error = np.max(np.abs(raw_target_q - q_actual))
            
            # Scaling Logic:
            # 0.05 rad (~3 deg) -> Start dampening
            # 0.01 rad (~0.5 deg) -> Minimum prediction
            if tracking_error < 0.05:
                # Calculate scale from 0.0 (error=0.01) to 1.0 (error=0.05)
                scale = np.clip((tracking_error - 0.01) / (0.05 - 0.01), 0.0, 1.0)
                final_horizon = self.horizon * scale
        
        # If moving, project the physical state forward by `final_horizon` seconds
        future_q = self.x[:, 0, 0] + (velocities * final_horizon)
        
        return future_q
