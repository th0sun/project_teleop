"""Retiming helpers for timestamped joint demonstrations.

The user demonstration is the source of truth.  This module preserves the
recorded timestamps when they fit the robot limits, and only stretches the
segments that are physically too fast for the selected robot profile.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Sequence, Tuple


@dataclass(frozen=True)
class TimedJointPoint:
    """One joint waypoint at a timestamp.

    Units are caller-defined, but all points and limits must use the same unit
    system.  The MG400 playback path uses degrees and degrees/second.
    """

    time_s: float
    position: Tuple[float, ...]


@dataclass(frozen=True)
class JointTimingLimits:
    """Velocity limits used to validate or stretch demonstration timing."""

    max_velocity: Tuple[float, ...]
    min_segment_duration_s: float = 1e-3
    safety_scale: float = 1.0

    def effective_velocity(self) -> Tuple[float, ...]:
        if self.safety_scale <= 0.0:
            raise ValueError("safety_scale must be > 0")
        return tuple(float(v) * float(self.safety_scale) for v in self.max_velocity)


@dataclass(frozen=True)
class SegmentTiming:
    """Timing decision for one segment."""

    index: int
    original_dt_s: float
    retimed_dt_s: float
    required_velocity: Tuple[float, ...]
    limit_velocity: Tuple[float, ...]

    @property
    def stretch_factor(self) -> float:
        return self.retimed_dt_s / self.original_dt_s

    @property
    def is_original_feasible(self) -> bool:
        return self.retimed_dt_s <= self.original_dt_s + 1e-9

    @property
    def max_required_velocity(self) -> float:
        return max(abs(v) for v in self.required_velocity)


@dataclass(frozen=True)
class RetimedTrajectory:
    """Result of preserving or retiming a timestamped joint path."""

    points: Tuple[TimedJointPoint, ...]
    segments: Tuple[SegmentTiming, ...]
    original_duration_s: float
    retimed_duration_s: float

    @property
    def is_original_timing_feasible(self) -> bool:
        return all(segment.is_original_feasible for segment in self.segments)

    @property
    def time_scale(self) -> float:
        if self.original_duration_s <= 1e-9:
            return 1.0
        return self.retimed_duration_s / self.original_duration_s

    @property
    def max_segment_stretch(self) -> float:
        if not self.segments:
            return 1.0
        return max(segment.stretch_factor for segment in self.segments)


def retime_joint_path(
    points: Iterable[TimedJointPoint],
    limits: JointTimingLimits,
) -> RetimedTrajectory:
    """Preserve timestamps when possible; stretch impossible segments.

    The geometric path is unchanged: every output point has the same joint
    position as the input.  Only the timestamps may move later.
    """

    pts = tuple(points)
    if not pts:
        raise ValueError("retime_joint_path requires at least one point")

    dof = len(pts[0].position)
    if dof == 0:
        raise ValueError("joint points must contain at least one joint")

    effective_velocity = limits.effective_velocity()
    if len(effective_velocity) != dof:
        raise ValueError(
            f"limit DOF mismatch: got {len(effective_velocity)}, expected {dof}"
        )
    if any(v <= 0.0 or not isfinite(v) for v in effective_velocity):
        raise ValueError("all max_velocity values must be finite and > 0")

    min_dt = max(float(limits.min_segment_duration_s), 1e-9)
    first_t = float(pts[0].time_s)
    previous_original_t = first_t
    previous_retimed_t = 0.0
    out = [TimedJointPoint(time_s=0.0, position=_as_tuple(pts[0].position))]
    segments = []

    for index, point in enumerate(pts[1:], start=1):
        current_original_t = float(point.time_s)
        original_dt = current_original_t - previous_original_t
        if original_dt <= 0.0:
            raise ValueError("joint point timestamps must be strictly increasing")
        original_dt = max(original_dt, min_dt)

        prev_pos = out[-1].position
        curr_pos = _as_tuple(point.position)
        if len(curr_pos) != dof:
            raise ValueError(
                f"point {index} DOF mismatch: got {len(curr_pos)}, expected {dof}"
            )

        delta = tuple(abs(curr_pos[j] - prev_pos[j]) for j in range(dof))
        required_velocity = tuple(delta[j] / original_dt for j in range(dof))
        required_dt = max(
            (delta[j] / effective_velocity[j]) if delta[j] > 0.0 else 0.0
            for j in range(dof)
        )
        retimed_dt = max(original_dt, required_dt, min_dt)
        previous_retimed_t += retimed_dt

        segments.append(
            SegmentTiming(
                index=index,
                original_dt_s=original_dt,
                retimed_dt_s=retimed_dt,
                required_velocity=required_velocity,
                limit_velocity=effective_velocity,
            )
        )
        out.append(TimedJointPoint(time_s=previous_retimed_t, position=curr_pos))
        previous_original_t = current_original_t

    original_duration = float(pts[-1].time_s) - first_t
    return RetimedTrajectory(
        points=tuple(out),
        segments=tuple(segments),
        original_duration_s=max(original_duration, 0.0),
        retimed_duration_s=out[-1].time_s,
    )


def speed_percent_for_segment(
    previous_position: Sequence[float],
    target_position: Sequence[float],
    duration_s: float,
    speed_at_100: Sequence[float],
    *,
    min_percent: int = 1,
    max_percent: int = 100,
) -> int:
    """Map a timed joint segment to a vendor speed percentage.

    Returns a clamped integer percentage.  Call ``retime_joint_path`` first if
    the original duration may require more than 100%.
    """

    prev = _as_tuple(previous_position)
    target = _as_tuple(target_position)
    speed_limit = _as_tuple(speed_at_100)
    if len(prev) != len(target) or len(prev) != len(speed_limit):
        raise ValueError("position and speed_at_100 lengths must match")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be > 0")
    if any(v <= 0.0 for v in speed_limit):
        raise ValueError("speed_at_100 values must be > 0")

    required_pct = max(
        (abs(target[j] - prev[j]) / duration_s) / speed_limit[j] * 100.0
        for j in range(len(prev))
    )
    return max(min_percent, min(max_percent, int(round(required_pct))))


def _as_tuple(values: Sequence[float]) -> Tuple[float, ...]:
    return tuple(float(v) for v in values)
