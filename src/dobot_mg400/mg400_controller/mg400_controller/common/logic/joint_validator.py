"""MG400 joint-limit adapter around robot-neutral clamp helpers."""

from __future__ import annotations

import time

import numpy as np

from teaching_core.program.validation import clamp_joint_target


class JointValidator:
    """Validate and clamp MG400 joint targets before sending commands."""

    FORMATTING_MARGIN_DEG = 0.0001

    def __init__(self, joint_limits, elbow_limit, logger):
        self.limits = joint_limits
        self.elbow_limit = elbow_limit
        self.logger = logger
        self._last_warn_time = 0.0

    def validate_and_clamp(self, q_target: np.ndarray) -> tuple[np.ndarray, bool]:
        """Return an MG400-safe joint target and whether it was clamped."""
        q_safe = np.asarray(q_target).copy()

        if len(q_safe) < 4:
            self.logger.error(
                f"JointValidator: Expected 4 joints, got {len(q_safe)}. "
                "Validation aborted."
            )
            return q_safe, False

        result = clamp_joint_target(
            q_safe,
            self.limits,
            relative_constraint=(1, 2, self.elbow_limit, "Elbow"),
            expected_min_len=4,
            formatting_margin_deg=self.FORMATTING_MARGIN_DEG,
        )

        if "NaN" in result.reasons:
            self.logger.error(
                "JointValidator: Detected NaN in target joint positions. "
                "Validation aborted."
            )
            return result.q_safe, True

        if result.reasons:
            now = time.time()
            if (now - self._last_warn_time) > 2.0:
                self.logger.warn(
                    f"Limit Exceeded [{', '.join(result.reasons)}] - Clamped"
                )
                self._last_warn_time = now

        return result.q_safe, result.was_clamped


