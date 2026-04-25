"""KinematicsContract + KinematicsKind (proposal §4.4).

Replaces the v1.1 ``urdf_path: str`` requirement with a richer contract
so that:

- ROS2-native arms (URDF + SRDF + ros2_control) can plug in cleanly;
- queued-TCP arms (MG400) can ship a NAMED in-tree provider until a
  vetted URDF lands;
- pluggable IK services (RelaxedIK, TracIK, vendor) are addressable;
- pure offline-export adapters (RAPID/KRL/TP) are not blocked from the
  profile layer at all.

Validation rules in ``validate_contract`` are the v0.1 contract that
``test_capability_contract.py`` enforces.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class KinematicsKind(str, Enum):
    URDF = "urdf"          # standard ROS2 description; preferred
    NAMED = "named"        # built-in provider (e.g. "mg400_4axis_fk")
    EXTERNAL = "external"  # pluggable IK service (RelaxedIK, TracIK, vendor)
    NONE = "none"          # OFFLINE_EXPORT-only adapter; no IK ever runs


class KinematicsContractError(ValueError):
    """Raised when a KinematicsContract is internally inconsistent."""


@dataclass(frozen=True)
class KinematicsContract:
    """What the adapter needs in order to run IK / FK.

    Field rules (enforced by :func:`validate_contract`):

    - ``kind == URDF``     → ``urdf_path`` required
    - ``kind == NAMED``    → ``provider_id`` required
    - ``kind == EXTERNAL`` → ``provider_id`` required (registry lookup)
    - ``kind == NONE``     → all other fields must be unset
    """

    kind: KinematicsKind
    urdf_path: Optional[str] = None
    srdf_path: Optional[str] = None
    planning_group: Optional[str] = None
    provider_id: Optional[str] = None
    fk_only: bool = False  # provider supplies FK only (lifter-side use)


def validate_contract(c: KinematicsContract) -> None:
    """Raise :class:`KinematicsContractError` on internal inconsistency."""
    k = c.kind
    if k == KinematicsKind.URDF:
        if not c.urdf_path:
            raise KinematicsContractError(
                "kind=URDF requires urdf_path"
            )
    elif k == KinematicsKind.NAMED:
        if not c.provider_id:
            raise KinematicsContractError(
                "kind=NAMED requires provider_id"
            )
        if c.urdf_path:
            raise KinematicsContractError(
                "kind=NAMED must not set urdf_path; use kind=URDF instead"
            )
    elif k == KinematicsKind.EXTERNAL:
        if not c.provider_id:
            raise KinematicsContractError(
                "kind=EXTERNAL requires provider_id (registry lookup)"
            )
    elif k == KinematicsKind.NONE:
        if any([c.urdf_path, c.srdf_path, c.planning_group, c.provider_id]):
            raise KinematicsContractError(
                "kind=NONE must not set any provider/URDF fields"
            )
        if c.fk_only:
            raise KinematicsContractError(
                "kind=NONE cannot declare fk_only"
            )
    else:  # pragma: no cover -- enum exhaustively covered
        raise KinematicsContractError(f"unknown kinematics kind: {k!r}")
