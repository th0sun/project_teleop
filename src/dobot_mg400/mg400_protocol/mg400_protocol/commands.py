"""Typed builders for the Dobot MG400 4-axis TCP command subset.

The manual command format is ASCII:
``CommandName(param1,param2,key=value)``.  This module owns that
format so translators do not hardcode vendor strings directly.

Motion primitives supported:
- ``JointMovJ``  — joint-space point-to-point
- ``MovJ``       — Cartesian point-to-point (joint interpolation)
- ``MovL``       — Cartesian linear interpolation
- ``Arc``        — Cartesian arc (3-point: current → through → target)
- ``Circle``     — full circle (current + 2 defining points)
- ``MovLIO``     — Cartesian linear + inline digital-output triggers
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


# ── Cartesian motion primitives ──────────────────────────────────────────

def arc(
    through_xyzr: Iterable[float],
    target_xyzr: Iterable[float],
    *,
    speed_l: Optional[float] = None,
    acc_l: Optional[float] = None,
    cp: Optional[float] = None,
) -> DobotCommand:
    """Build ``Arc(X1,Y1,Z1,R1,X2,Y2,Z2,R2,SpeedL=R,AccL=R,CP=R)``.

    The robot moves from the current position through ``through_xyzr``
    to ``target_xyzr`` in arc-interpolated mode.  All three points
    (current, through, target) must not be collinear.
    """
    through = tuple(float(v) for v in through_xyzr)
    target = tuple(float(v) for v in target_xyzr)
    if len(through) != 4:
        raise ValueError(f"Arc through-point expects 4 values (X,Y,Z,R), got {len(through)}")
    if len(target) != 4:
        raise ValueError(f"Arc target-point expects 4 values (X,Y,Z,R), got {len(target)}")

    args = through + target
    kwargs: list[tuple[str, object]] = []
    if speed_l is not None:
        kwargs.append(("SpeedL", _speed(speed_l)))
    if acc_l is not None:
        kwargs.append(("AccL", _speed(acc_l)))
    if cp is not None:
        kwargs.append(("CP", _blend_pct(cp)))
    return DobotCommand("Arc", args, tuple(kwargs))


def circle(
    count: int,
    p1_xyzr: Iterable[float],
    p2_xyzr: Iterable[float],
) -> DobotCommand:
    """Build ``Circle(count,{X1,Y1,Z1,R1},{X2,Y2,Z2,R2})``.

    The robot traces *count* full circles defined by the current position,
    ``p1_xyzr``, and ``p2_xyzr``, returning to the start after each lap.
    """
    p1 = tuple(float(v) for v in p1_xyzr)
    p2 = tuple(float(v) for v in p2_xyzr)
    if len(p1) != 4:
        raise ValueError(f"Circle P1 expects 4 values, got {len(p1)}")
    if len(p2) != 4:
        raise ValueError(f"Circle P2 expects 4 values, got {len(p2)}")
    if int(count) < 1:
        raise ValueError(f"Circle count must be >= 1, got {count}")
    return DobotCommand("Circle", (int(count), list(p1), list(p2)))


def mov_l_io(
    target_xyzr: Iterable[float],
    io_triggers: Iterable[Tuple[int, int, int, int]],
    *,
    speed_l: Optional[float] = None,
    acc_l: Optional[float] = None,
    cp: Optional[float] = None,
) -> DobotCommand:
    """Build ``MovLIO(X,Y,Z,R,{Mode,Distance,Index,Status},...)``.

    Each *io_trigger* is ``(mode, distance, index, status)``:
    - mode 0 = distance percentage (0..100], mode 1 = distance value (mm)
    - positive distance = from start, negative = from target
    - index = DO port, status = 0 or 1
    """
    target = tuple(float(v) for v in target_xyzr)
    if len(target) != 4:
        raise ValueError(f"MovLIO expects 4 target values (X,Y,Z,R), got {len(target)}")
    triggers = [tuple(int(v) for v in t) for t in io_triggers]
    for i, t in enumerate(triggers):
        if len(t) != 4:
            raise ValueError(f"IO trigger {i} expects 4 values, got {len(t)}")

    args: list[object] = list(target) + [list(t) for t in triggers]
    kwargs: list[tuple[str, object]] = []
    if speed_l is not None:
        kwargs.append(("SpeedL", _speed(speed_l)))
    if acc_l is not None:
        kwargs.append(("AccL", _speed(acc_l)))
    if cp is not None:
        kwargs.append(("CP", _blend_pct(cp)))
    return DobotCommand("MovLIO", tuple(args), tuple(kwargs))


def mov_l_cartesian(
    target_xyzr: Iterable[float],
    *,
    speed_l: Optional[float] = None,
    acc_l: Optional[float] = None,
    cp: Optional[float] = None,
) -> DobotCommand:
    """Build ``MovL(X,Y,Z,R,SpeedL=R,AccL=R,CP=R)``.

    Unlike :func:`mov_l` (which uses ``SpeedJ``), this builder targets
    the Cartesian-linear motion command documented in the 4-axis TCP/IP
    guide and uses ``SpeedL`` / ``AccL`` keyword arguments.
    """
    values = tuple(float(v) for v in target_xyzr)
    if len(values) != 4:
        raise ValueError(f"MovL expects 4 values (X,Y,Z,R), got {len(values)}")

    kwargs: list[tuple[str, object]] = []
    if speed_l is not None:
        kwargs.append(("SpeedL", _speed(speed_l)))
    if acc_l is not None:
        kwargs.append(("AccL", _speed(acc_l)))
    if cp is not None:
        kwargs.append(("CP", _blend_pct(cp)))
    return DobotCommand("MovL", values, tuple(kwargs))
