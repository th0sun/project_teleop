"""Typed builders for the Dobot MG400 4-axis TCP command subset.

The manual command format is ASCII:
``CommandName(param1,param2,key=value)``.  This module owns that
format so translators do not hardcode vendor strings directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class DobotCommand:
    """Rendered Dobot TCP command plus structured metadata."""

    name: str
    args: Tuple[object, ...] = ()
    kwargs: Tuple[Tuple[str, object], ...] = ()

    def render(self) -> str:
        parts = [_format_value(value) for value in self.args]
        parts.extend(
            f"{key}={_format_value(value)}"
            for key, value in self.kwargs
            if value is not None
        )
        return f"{self.name}(" + ",".join(parts) + ")"

    def __str__(self) -> str:
        return self.render()


def _format_value(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.4f}"
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, (list, tuple)):
        return "{" + ",".join(_format_value(item) for item in value) + "}"
    return str(value)


def _speed(value: float) -> int:
    """Clamp to vendor [1, 100] range used by SpeedJ / AccJ / SpeedFactor."""
    return max(1, min(100, int(round(value))))


def _blend_pct(value: float) -> int:
    """Clamp to vendor [0, 100] range used by ``CP`` (0 = no blending)."""
    return max(0, min(100, int(round(value))))


def joint_mov_j(
    joints_deg: Iterable[float],
    *,
    speed_j: float,
    acc_j: Optional[float] = None,
    cp: Optional[float] = None,
) -> DobotCommand:
    """Build ``JointMovJ(J1,J2,J3,J4,SpeedJ=R,AccJ=R,CP=R)``."""
    joints = tuple(float(value) for value in joints_deg)
    if len(joints) != 4:
        raise ValueError(f"JointMovJ expects 4 joints, got {len(joints)}")

    kwargs = [("SpeedJ", _speed(speed_j))]
    if acc_j is not None:
        kwargs.append(("AccJ", _speed(acc_j)))
    if cp is not None:
        kwargs.append(("CP", _blend_pct(cp)))
    return DobotCommand("JointMovJ", joints, tuple(kwargs))


def mov_j(
    pose_or_joint_values: Iterable[float],
    *,
    speed_j: float,
    acc_j: Optional[float] = None,
    cp: Optional[float] = None,
) -> DobotCommand:
    """Build the legacy 4-value ``MovJ`` form used by the current runtime."""
    values = tuple(float(value) for value in pose_or_joint_values)
    if len(values) != 4:
        raise ValueError(f"MovJ expects 4 values, got {len(values)}")

    kwargs = [("SpeedJ", _speed(speed_j))]
    if acc_j is not None:
        kwargs.append(("AccJ", _speed(acc_j)))
    if cp is not None:
        kwargs.append(("CP", _blend_pct(cp)))
    return DobotCommand("MovJ", values, tuple(kwargs))


def mov_l(
    pose_values: Iterable[float],
    *,
    speed_j: float,
    acc_j: Optional[float] = None,
    cp: Optional[float] = None,
) -> DobotCommand:
    """Build the legacy 4-value ``MovL`` form used by the current runtime."""
    values = tuple(float(value) for value in pose_values)
    if len(values) != 4:
        raise ValueError(f"MovL expects 4 values, got {len(values)}")

    kwargs = [("SpeedJ", _speed(speed_j))]
    if acc_j is not None:
        kwargs.append(("AccJ", _speed(acc_j)))
    if cp is not None:
        kwargs.append(("CP", _blend_pct(cp)))
    return DobotCommand("MovL", values, tuple(kwargs))


def do_execute(index: int, status: bool) -> DobotCommand:
    """Build ``DOExecute(index,status)``."""
    return DobotCommand("DOExecute", (int(index), bool(status)))
