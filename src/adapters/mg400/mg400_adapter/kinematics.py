"""MG400 FK provider backed by the existing runtime utility."""

from __future__ import annotations

from math import cos, radians, sin
from typing import Optional, Sequence

import numpy as np

from mg400_adapter.profile import MG400_PROVIDER_ID
from mg400_controller.common.utils.kinematics import KinematicsCalculator
from teaching_core.kinematics.provider import JointVector, NotSupported, PoseTuple
from teaching_core.kinematics.registry import register_provider


class MG400FkProvider:
    """FK-only provider for the current MG400 model."""

    provider_id = MG400_PROVIDER_ID
    dof = 4
    base_frame = "mg400_base"
    tcp_frame = "mg400_tcp"

    def __init__(self):
        self._calculator = KinematicsCalculator()

    def fk(self, q_rad: Sequence[float]) -> PoseTuple:
        if len(q_rad) < self.dof:
            raise ValueError(f"MG400 FK expects 4 joints, got {len(q_rad)}")

        joints_deg = np.degrees(np.asarray(q_rad[: self.dof], dtype=float))
        pose_mm = self._calculator.forward_kinematics(joints_deg)
        yaw_rad = radians(float(pose_mm[3]))
        half_yaw = yaw_rad / 2.0
        return (
            (
                float(pose_mm[0]) / 1000.0,
                float(pose_mm[1]) / 1000.0,
                float(pose_mm[2]) / 1000.0,
            ),
            (0.0, 0.0, sin(half_yaw), cos(half_yaw)),
        )

    def solve_ik(
        self,
        target_pose: PoseTuple,
        seed_rad: Optional[Sequence[float]] = None,
    ) -> JointVector:
        raise NotSupported(
            "MG400 adapter v0 exposes FK only; canonical Cartesian IK "
            "lands in a later adapter milestone."
        )


def register_mg400_provider() -> MG400FkProvider:
    """Register and return the process-local MG400 FK provider."""
    provider = MG400FkProvider()
    register_provider(provider)
    return provider
