"""Null kinematics provider.

Used by tests and by adapters that declare ``kinematics.kind=NONE``
(offline-export-only). FK and IK both raise :class:`NotSupported` so
mistakes surface immediately.
"""

from __future__ import annotations

from typing import Optional, Sequence

from teaching_core.kinematics.provider import (
    JointVector,
    NotSupported,
    PoseTuple,
)


class NullProvider:
    """Explicit no-FK/no-IK provider."""

    def __init__(
        self,
        provider_id: str = "null",
        *,
        dof: int = 0,
        base_frame: str = "base",
        tcp_frame: str = "tcp",
    ) -> None:
        self.provider_id = provider_id
        self.dof = dof
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame

    def fk(self, q_rad: Sequence[float]) -> PoseTuple:
        raise NotSupported(
            f"NullProvider({self.provider_id!r}) cannot compute FK"
        )

    def solve_ik(
        self,
        target_pose: PoseTuple,
        seed_rad: Optional[Sequence[float]] = None,
    ) -> JointVector:
        raise NotSupported(
            f"NullProvider({self.provider_id!r}) cannot solve IK"
        )
