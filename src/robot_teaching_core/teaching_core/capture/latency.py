"""Latency compensation for captured teaching targets."""

from __future__ import annotations

from typing import Protocol, Tuple

import numpy as np


class JointClampValidator(Protocol):
    def validate_and_clamp(self, q_target: np.ndarray) -> Tuple[np.ndarray, bool]:
        """Return a safe target and whether clamping occurred."""
        ...


class TargetLatencyCompensator:
    """Shift a target forward using measured capture-to-receive latency."""

    def __init__(
        self,
        validator: JointClampValidator,
        alpha: float = 0.3,
        max_compensation_sec: float = 0.08,
        min_sample_dt_sec: float = 0.002,
        max_sample_dt_sec: float = 0.5,
        joint_count: int = 4,
    ):
        self.validator = validator
        self.alpha = alpha
        self.max_compensation_sec = max_compensation_sec
        self.min_sample_dt_sec = min_sample_dt_sec
        self.max_sample_dt_sec = max_sample_dt_sec
        self.joint_count = joint_count

        self._prev_target_q = None
        self._prev_target_t = None
        self._ema_target_vel = np.zeros(joint_count)

    def compensate(
        self,
        q_safe,
        corrected_target_time: float,
        receive_time: float,
    ) -> np.ndarray:
        """Return a joint target shifted forward by measured latency."""
        q_safe = np.asarray(q_safe)
        if len(q_safe) < self.joint_count:
            q_validated, _ = self.validator.validate_and_clamp(q_safe)
            return q_validated

        network_latency_sec = max(0.0, receive_time - corrected_target_time)

        if self._prev_target_q is None:
            self._prev_target_q = q_safe.copy()
            self._prev_target_t = corrected_target_time

        dt_target = corrected_target_time - self._prev_target_t
        if self.min_sample_dt_sec < dt_target < self.max_sample_dt_sec:
            raw_vel = (
                q_safe[: self.joint_count] - self._prev_target_q[: self.joint_count]
            ) / dt_target
            self._ema_target_vel = (
                self.alpha * raw_vel + (1.0 - self.alpha) * self._ema_target_vel
            )

        self._prev_target_q = q_safe.copy()
        self._prev_target_t = corrected_target_time

        latency_comp_sec = min(network_latency_sec, self.max_compensation_sec)
        q_compensated = (
            q_safe[: self.joint_count] + self._ema_target_vel * latency_comp_sec
        )
        q_compensated_full = np.concatenate([q_compensated, q_safe[self.joint_count :]])
        q_compensated_safe, _ = self.validator.validate_and_clamp(q_compensated_full)
        return q_compensated_safe
