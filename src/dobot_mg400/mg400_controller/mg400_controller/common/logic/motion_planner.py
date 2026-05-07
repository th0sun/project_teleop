#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""🎮 Motion Planner — render MG400 motion command strings.

Post-2026-05 cleanup: this module used to host plan_motion(),
plan_batch_motion(), should_skip_motion(), and calculate_speed().  None of
them had any caller outside the file (verified by repo-wide grep) — the
adaptive teleop controller calls format_command() directly with its own
per-command speed/acc/cp.  The unused entrypoints + the SPEED_* /
SPATIAL_THRESHOLD bands they consumed have been removed to make the
planner's actual surface area obvious: it is a stateless string-builder
keyed on the configured control_mode.
"""

import numpy as np
from mg400_protocol.commands import joint_mov_j, mov_j, mov_l
from mg400_controller.common.config.motion_config import ACC_VALUE, CP_VALUE


class MotionPlanner:
    def __init__(self, control_mode, logger):
        self.control_mode = control_mode
        self.logger = logger

    def format_command(self, q_rad, speed_percent, acc_percent=None, cp=None):
        """Render the per-command MG400 string.

        Args:
            q_rad: 4-vector of target joint angles in radians.
            speed_percent: SpeedJ % (0-100) for this command.
            acc_percent: AccJ % override.  Defaults to ACC_VALUE so existing
                callers (batch interpolation, mock tests) keep prior behaviour
                while the realtime adaptive controller threads its own value.
            cp: Continuous Path % override.  Defaults to CP_VALUE.  The realtime
                controller passes REALTIME_CP (80) here per EXP4 results;
                callers that don't care fall back to the global default.
        """
        q_deg = np.degrees(q_rad)
        if acc_percent is None:
            acc_percent = ACC_VALUE
        if cp is None:
            cp = CP_VALUE

        if self.control_mode == "jointmovj":
            return joint_mov_j(
                q_deg[:4],
                speed_j=speed_percent,
                acc_j=acc_percent,
                cp=cp,
            ).render()

        elif self.control_mode == "movj":
            return mov_j(
                q_deg[:4],
                speed_j=speed_percent,
                acc_j=acc_percent,
                cp=cp,
            ).render()

        elif self.control_mode == "movl":
            return mov_l(
                q_deg[:4],
                speed_j=speed_percent,
                acc_j=acc_percent,
                cp=cp,
            ).render()

        return ""
