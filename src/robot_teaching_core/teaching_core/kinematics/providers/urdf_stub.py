"""URDF-backed kinematics provider — PR1 skeleton.

The Protocol shape is fixed; the library backend is **not** yet
chosen (open question per proposal §10: pinocchio vs KDL). This stub
loads and stores the URDF path so callers can wire the seam end-to-
end, but FK/IK raise :class:`NotSupported` with a clear pointer to
the M1 scoping decision.

When a backend is picked, this file becomes a thin adapter and the
constructor parses the URDF into the chosen library's structures.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence, Tuple

from teaching_core.kinematics.provider import (
    JointVector,
    NotSupported,
    PoseTuple,
)


class URDFProviderStub:
    """Skeleton URDF-backed provider.

    PR1 scope: validates that the URDF file exists and remembers
    metadata (joint count, frames). FK/IK are not yet implemented —
    the backend (pinocchio vs KDL) is an open question.
    """

    def __init__(
        self,
        provider_id: str,
        urdf_path: str,
        *,
        dof: int,
        base_frame: str,
        tcp_frame: str,
        joint_names: Optional[Sequence[str]] = None,
    ) -> None:
        path = Path(urdf_path)
        if not path.exists():
            raise FileNotFoundError(f"URDF not found at {urdf_path!r}")
        self.provider_id = provider_id
        self.urdf_path = str(path)
        self.dof = dof
        self.base_frame = base_frame
        self.tcp_frame = tcp_frame
        self.joint_names: Tuple[str, ...] = tuple(joint_names or ())

    # ----- Protocol surface ----------------------------------------

    def fk(self, q_rad: Sequence[float]) -> PoseTuple:
        raise NotSupported(
            f"URDFProviderStub({self.provider_id!r}): FK backend not yet "
            "implemented; pinocchio vs KDL is open per proposal §10."
        )

    def solve_ik(
        self,
        target_pose: PoseTuple,
        seed_rad: Optional[Sequence[float]] = None,
    ) -> JointVector:
        raise NotSupported(
            f"URDFProviderStub({self.provider_id!r}): IK backend not yet "
            "implemented; pinocchio vs KDL is open per proposal §10."
        )
