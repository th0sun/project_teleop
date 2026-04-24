#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Latency compensation for incoming Unity/VR joint targets.
"""

import numpy as np


class TargetLatencyCompensator:
    def __init__(
        self,
        validator,
        alpha=0.3,
        max_compensation_sec=0.08,
        min_sample_dt_sec=0.002,
        max_sample_dt_sec=0.5,
    ):
        self.validator = validator
        self.alpha = alpha
        self.max_compensation_sec = max_compensation_sec
        self.min_sample_dt_sec = min_sample_dt_sec
        self.max_sample_dt_sec = max_sample_dt_sec

        self._prev_target_q = None
        self._prev_target_t = None
        self._ema_target_vel = np.zeros(4)

    def compensate(self, q_safe, corrected_target_time, receive_time):
        """Return a joint target shifted forward by measured Unity-to-ROS latency."""
        q_safe = np.asarray(q_safe)
        network_latency_sec = max(0.0, receive_time - corrected_target_time)

        if self._prev_target_q is None:
            self._prev_target_q = q_safe.copy()
            self._prev_target_t = corrected_target_time

        dt_target = corrected_target_time - self._prev_target_t
        if self.min_sample_dt_sec < dt_target < self.max_sample_dt_sec:
            raw_vel = (q_safe[:4] - self._prev_target_q[:4]) / dt_target
            self._ema_target_vel = (
                self.alpha * raw_vel + (1.0 - self.alpha) * self._ema_target_vel
            )

        self._prev_target_q = q_safe.copy()
        self._prev_target_t = corrected_target_time

        latency_comp_sec = min(network_latency_sec, self.max_compensation_sec)
        q_compensated = q_safe[:4] + self._ema_target_vel * latency_comp_sec
        q_compensated_full = np.concatenate([q_compensated, q_safe[4:]])
        q_compensated_safe, _ = self.validator.validate_and_clamp(q_compensated_full)
        return q_compensated_safe
