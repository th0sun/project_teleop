"""KinematicsProvider Protocol.

The Protocol is intentionally minimal. The lifter only needs FK
(joint state -> Cartesian pose); adapters that do per-step IK call
``solve_ik`` and may set ``fk_only=True`` on their KinematicsContract
to declare that IK is somebody else's problem (e.g., the vendor
controller does it).

Library choice (pinocchio vs KDL) is **not** locked by this Protocol.
Concrete providers can be backed by either; the registry returns a
provider instance, not a library handle.
"""

from __future__ import annotations

from typing import Optional, Protocol, Sequence, Tuple, runtime_checkable

# (x, y, z) position in metres + (x, y, z, w) quaternion.
PoseTuple = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]
JointVector = Tuple[float, ...]


class KinematicsError(RuntimeError):
    """Generic kinematics failure (singularity, joint-limit violation, ...)."""


class NotSupported(KinematicsError):
    """Raised when a provider does not implement the requested operation.

    Example: lifter-side FK-only providers raise this on ``solve_ik``.
    """


@runtime_checkable
class KinematicsProvider(Protocol):
    """Pluggable FK/IK seam.

    Implementations must be importable without ROS2 (so they can run
    inside unit tests in any CI environment).
    """

    provider_id: str
    dof: int
    base_frame: str
    tcp_frame: str

    def fk(self, q_rad: Sequence[float]) -> PoseTuple:
        """Joint state -> TCP pose in ``base_frame``."""
        ...

    def solve_ik(
        self,
        target_pose: PoseTuple,
        seed_rad: Optional[Sequence[float]] = None,
    ) -> JointVector:
        """TCP pose -> joint state. Raises :class:`NotSupported` if FK-only."""
        ...
