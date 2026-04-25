"""Clock calibration for remote teaching streams.

This is robot-neutral capture plumbing: it estimates how a remote
source clock (Unity/OpenXR/etc.) maps into local ROS/process time using
a sliding minimum-delay window plus conservative drift estimation.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np


class ClockCalibrator:
    """Synchronize remote teaching timestamps into local time."""

    def __init__(self, window_size: int = 50, regression_buffer_size: int = 100):
        self.window_size = window_size
        self.delay_history: Deque[float] = deque(maxlen=window_size)
        self.regression_points: Deque[Tuple[float, float]] = deque(
            maxlen=regression_buffer_size
        )

        self.drift_rate = 0.0
        self.base_offset = 0.0
        self.t_start_ros = 0.0
        self.is_calibrated = False

        self.last_raw_diff: Optional[float] = None
        self.jump_threshold = 0.5

    def calibrate(self, t1_remote: float, t2_local: float) -> float:
        """Return the estimated local time when the remote packet was sent."""
        raw_diff = t2_local - t1_remote

        jumped = (
            self.last_raw_diff is not None
            and abs(raw_diff - self.last_raw_diff) > self.jump_threshold
        )
        if not self.is_calibrated or jumped:
            self.base_offset = raw_diff - 0.004
            self.t_start_ros = t2_local
            self.drift_rate = 0.0
            self.delay_history.clear()
            self.regression_points.clear()
            self.is_calibrated = True

        self.last_raw_diff = raw_diff

        elapsed_local = t2_local - self.t_start_ros
        drift_term = elapsed_local * self.drift_rate
        normalized_diff = raw_diff - drift_term
        self.delay_history.append(normalized_diff)

        stable_offset = min(self.delay_history)

        if len(self.delay_history) >= self.window_size:
            self.regression_points.append((elapsed_local, stable_offset))
            if len(self.regression_points) >= 50:
                self._estimate_drift()

        return t1_remote + stable_offset + drift_term

    def _estimate_drift(self) -> None:
        """Update drift-rate estimate from the stable-offset floor."""
        pts = np.array(self.regression_points)
        x = pts[:, 0]
        y = pts[:, 1]
        if len(x) < 2:
            return

        residual_drift = np.polyfit(x, y, 1)[0]
        self.drift_rate += residual_drift

        if np.isnan(self.drift_rate):
            self.drift_rate = 0.0
        self.drift_rate = float(np.clip(self.drift_rate, -0.005, 0.005))
        self.regression_points.clear()

    def get_current_offset(self, now_fn=time.time) -> float:
        """Return the current remote-to-local offset including drift."""
        if not self.is_calibrated:
            return 0.0
        elapsed = now_fn() - self.t_start_ros
        return min(self.delay_history) + (elapsed * self.drift_rate)
