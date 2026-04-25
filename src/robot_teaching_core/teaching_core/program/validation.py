"""Robot-neutral joint validation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class JointClampResult:
    q_safe: np.ndarray
    was_clamped: bool
    reasons: Tuple[str, ...] = ()


def clamp_joint_positions(
    q_target,
    joint_limits_deg: Mapping[int, Tuple[float, float]],
    *,
    formatting_margin_deg: float = 0.0001,
) -> JointClampResult:
    """Clamp joint positions in radians against per-joint degree limits."""
    q_safe = np.asarray(q_target).copy()
    if joint_limits_deg and len(q_safe) <= max(joint_limits_deg):
        return JointClampResult(q_safe=q_safe, was_clamped=False, reasons=("short",))
    if np.any(np.isnan(q_safe)):
        return JointClampResult(q_safe=q_safe, was_clamped=True, reasons=("NaN",))

    reasons = []
    for joint_id, (min_deg, max_deg) in joint_limits_deg.items():
        min_safe_deg = min_deg + formatting_margin_deg
        max_safe_deg = max_deg - formatting_margin_deg
        min_rad = np.radians(min_safe_deg)
        max_rad = np.radians(max_safe_deg)
        if q_safe[joint_id] < min_rad or q_safe[joint_id] > max_rad:
            orig_deg = np.degrees(q_safe[joint_id])
            q_safe[joint_id] = np.clip(q_safe[joint_id], min_rad, max_rad)
            new_deg = np.degrees(q_safe[joint_id])
            reasons.append(f"J{joint_id + 1}({orig_deg:.2f}deg->{new_deg:.2f}deg)")

    return JointClampResult(
        q_safe=q_safe, was_clamped=bool(reasons), reasons=tuple(reasons)
    )


def clamp_relative_joint_angle(
    q_target,
    *,
    parent_index: int,
    child_index: int,
    relative_limit_deg: Tuple[float, float],
    formatting_margin_deg: float = 0.0001,
    label: str = "Relative",
) -> JointClampResult:
    """Clamp a child joint to keep child-parent angle within limits."""
    q_safe = np.asarray(q_target).copy()
    if len(q_safe) <= max(parent_index, child_index):
        return JointClampResult(q_safe=q_safe, was_clamped=False, reasons=("short",))
    if np.any(np.isnan(q_safe)):
        return JointClampResult(q_safe=q_safe, was_clamped=True, reasons=("NaN",))

    parent_deg = np.degrees(q_safe[parent_index])
    child_deg = np.degrees(q_safe[child_index])
    relative_deg = child_deg - parent_deg

    min_deg, max_deg = relative_limit_deg
    safe_min = min_deg + formatting_margin_deg
    safe_max = max_deg - formatting_margin_deg

    if safe_min <= relative_deg <= safe_max:
        return JointClampResult(q_safe=q_safe, was_clamped=False)

    clamped_relative = np.clip(relative_deg, safe_min, safe_max)
    q_safe[child_index] = np.radians(parent_deg + clamped_relative)
    return JointClampResult(
        q_safe=q_safe,
        was_clamped=True,
        reasons=(f"{label}({relative_deg:.2f}deg->{clamped_relative:.2f}deg)",),
    )


def clamp_joint_target(
    q_target,
    joint_limits_deg: Mapping[int, Tuple[float, float]],
    *,
    relative_constraint: Optional[Tuple[int, int, Tuple[float, float], str]] = None,
    expected_min_len: Optional[int] = None,
    formatting_margin_deg: float = 0.0001,
) -> JointClampResult:
    """Clamp per-joint limits and an optional relative joint constraint."""
    q_safe = np.asarray(q_target).copy()
    if expected_min_len is not None and len(q_safe) < expected_min_len:
        return JointClampResult(q_safe=q_safe, was_clamped=False, reasons=("short",))

    joint_result = clamp_joint_positions(
        q_safe, joint_limits_deg, formatting_margin_deg=formatting_margin_deg
    )
    q_safe = joint_result.q_safe
    reasons = list(joint_result.reasons)
    was_clamped = joint_result.was_clamped

    if relative_constraint is not None and not np.any(np.isnan(q_safe)):
        parent, child, limits, label = relative_constraint
        relative_result = clamp_relative_joint_angle(
            q_safe,
            parent_index=parent,
            child_index=child,
            relative_limit_deg=limits,
            formatting_margin_deg=formatting_margin_deg,
            label=label,
        )
        q_safe = relative_result.q_safe
        reasons.extend(relative_result.reasons)
        was_clamped = was_clamped or relative_result.was_clamped

    return JointClampResult(
        q_safe=q_safe,
        was_clamped=was_clamped,
        reasons=tuple(reasons),
    )
