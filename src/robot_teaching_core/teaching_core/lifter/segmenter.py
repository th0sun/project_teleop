"""Lifter v0: dwell + velocity segmentation, Cartesian lift via provider.

Algorithm summary:

1. estimate joint velocity per frame as ``|q[i+1] - q[i]| / dt``;
2. emit a candidate ``move`` waypoint at every frame where the local
   joint velocity drops below ``dwell_velocity_rad_s`` for at least
   ``dwell_window_s`` seconds (== a settle / hold);
3. always emit the final frame as a waypoint;
4. each waypoint is lifted to a ``MoveStep`` whose ``pose`` is the
   provider FK of that frame's joints; ``joint_hint_rad`` records the
   captured joints so single-morphology replay can fall back to them;
5. ``orientation_intent`` is required on every step — the lifter's
   default rule is set per the kinematics provider's declared
   authority (passed via ``LifterConfig``).

Robot-neutral by construction: this file imports nothing from
``adapters/*`` or any robot-specific module. The
:class:`teaching_core.kinematics.KinematicsProvider` Protocol is the
only seam used to compute Cartesian poses.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

from teaching_core.calibration.types import CalibrationBinding
from teaching_core.kinematics.provider import (
    KinematicsProvider,
    NotSupported,
)
from teaching_core.lifter.session import SessionFrame, SessionStream
from teaching_core.program.types import (
    Annotations,
    Blend,
    CanonicalProgram,
    Defaults,
    Frames,
    Motion,
    MoveStep,
    OrientationIntent,
    Pose,
    Source,
)


class SegmentationError(ValueError):
    """Raised when the input stream cannot be segmented into a program."""


@dataclass(frozen=True)
class LifterConfig:
    """Tunable knobs for v0 segmentation.

    Defaults are robot-neutral; lift quality is data-driven, not
    MG400-specific.
    """

    program_id: str
    capture_id: str
    captured_at: str
    # Default to hand_authored so synthetic / unit-test sessions do not
    # spuriously trigger the VR-capture-needs-calibration schema rule.
    # Real VR pipelines pass demonstrator="hand_vr" + a calibration.
    demonstrator: str = "hand_authored"
    calibration: Optional[CalibrationBinding] = None

    # Below this max-joint velocity AND for at least dwell_window_s,
    # a frame counts as a settle (waypoint candidate).
    dwell_velocity_rad_s: float = 0.05
    dwell_window_s: float = 0.20

    # Default per-step orientation intent. Callers set this from the
    # provider's orientation_authority (e.g., YAW_ONLY for SCARA).
    default_orientation_intent: OrientationIntent = OrientationIntent.EXACT

    motion: Motion = Motion.JOINT
    pose_frame: str = "world"
    blend: Blend = Blend.SMOOTH
    task_label: Optional[str] = None


def _max_abs_diff(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def _frame_to_pose(provider: KinematicsProvider, frame: SessionFrame) -> Pose:
    try:
        position, orientation = provider.fk(frame.q_rad)
    except NotSupported as exc:
        raise SegmentationError(
            f"kinematics provider {provider.provider_id!r} cannot run FK; "
            "lifter v0 needs an FK-capable provider"
        ) from exc
    return Pose(
        position_m=tuple(float(v) for v in position),
        orientation_quat_xyzw=tuple(float(v) for v in orientation),
    )


def _segment_indices(
    frames: Sequence[SessionFrame], cfg: LifterConfig
) -> List[int]:
    """Return indices of frames that should become canonical waypoints.

    Always emits index 0 and the final index, plus any settle in
    between (joint velocity below threshold for at least dwell_window).
    """
    n = len(frames)
    if n == 0:
        return []
    if n == 1:
        return [0]

    # Per-frame joint velocity magnitude (max-abs across joints).
    velocity: List[float] = [0.0]
    for i in range(1, n):
        dt = frames[i].timestamp_s - frames[i - 1].timestamp_s
        if dt <= 0.0:
            raise SegmentationError(
                f"frame {i} has non-monotonic timestamp ({frames[i].timestamp_s} "
                f"<= {frames[i - 1].timestamp_s})"
            )
        velocity.append(_max_abs_diff(frames[i].q_rad, frames[i - 1].q_rad) / dt)

    waypoints: List[int] = [0]
    settle_started_at: Optional[float] = None
    settle_anchor: Optional[int] = None

    for i in range(1, n - 1):
        if velocity[i] <= cfg.dwell_velocity_rad_s:
            if settle_started_at is None:
                settle_started_at = frames[i].timestamp_s
                settle_anchor = i
            else:
                window = frames[i].timestamp_s - settle_started_at
                if (
                    window >= cfg.dwell_window_s
                    and settle_anchor is not None
                    and waypoints[-1] != settle_anchor
                ):
                    waypoints.append(settle_anchor)
        else:
            settle_started_at = None
            settle_anchor = None

    if waypoints[-1] != n - 1:
        waypoints.append(n - 1)
    return waypoints


def lift_session(
    stream: SessionStream,
    *,
    provider: KinematicsProvider,
    config: LifterConfig,
) -> CanonicalProgram:
    """Segment a session stream into a canonical program.

    The provider supplies forward kinematics. The lifter never imports
    a concrete provider; the caller injects one (R14 / proposal §6 M1).
    """
    frames = stream.frames
    if not frames:
        raise SegmentationError("empty session stream")

    expected_dof = provider.dof
    for i, frame in enumerate(frames):
        if expected_dof and len(frame.q_rad) != expected_dof:
            raise SegmentationError(
                f"frame {i}: q_rad length {len(frame.q_rad)} does not match "
                f"provider dof {expected_dof}"
            )

    indices = _segment_indices(frames, config)

    move_steps: List[MoveStep] = []
    for idx in indices:
        frame = frames[idx]
        pose = _frame_to_pose(provider, frame)
        move_steps.append(
            MoveStep(
                motion=config.motion,
                pose_frame=config.pose_frame,
                pose=pose,
                orientation_intent=config.default_orientation_intent,
                joint_hint_rad=tuple(frame.q_rad),
                blend=config.blend,
            )
        )

    return CanonicalProgram(
        program_id=config.program_id,
        source=Source(
            capture_id=config.capture_id,
            captured_at=config.captured_at,
            demonstrator=config.demonstrator,
        ),
        frames=Frames(world=provider.base_frame),
        defaults=Defaults(blend=config.blend),
        steps=tuple(move_steps),
        calibration=config.calibration,
        annotations=Annotations(task_label=config.task_label),
    )
