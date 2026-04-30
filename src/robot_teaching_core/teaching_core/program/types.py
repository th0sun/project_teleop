"""Canonical program IR dataclasses (v0.1).

This file mirrors the schema in ``schema.py`` and the robot-neutral teaching
architecture described by the report materials.

Design notes:
- Dataclasses are ``frozen=True`` so a loaded program is immutable in
  memory. Round-trip through ``program_to_dict`` / ``program_from_dict``
  in ``io.py`` is the only intended way to mutate.
- ``orientation_intent`` is REQUIRED on every ``move`` step. Default
  values for ``orientation_tolerance_rad`` come from the intent, not
  from a global default — see ``DEFAULT_TOLERANCE_BY_INTENT``.
- Pose orientation is currently quaternion (xyzw). Axis-angle
  representation is an open question (proposal §10) and can change
  without breaking the dataclass shape.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Literal, Optional, Tuple, Union

from teaching_core.calibration.types import CalibrationBinding

SCHEMA_VERSION = "0.1"


# ---- Enums -----------------------------------------------------------


class Motion(str, Enum):
    """Motion-type hint for ``move`` steps."""

    JOINT = "joint"
    LINEAR = "linear"
    CIRCULAR = "circular"
    SPLINE = "spline"


class OrientationIntent(str, Enum):
    """Per-step orientation intent.

    Required on every ``move`` step. Modeled after MoveIt's
    ``OrientationConstraint`` (per-axis absolute tolerance).
    """

    EXACT = "exact"
    YAW_ONLY = "yaw_only"
    TOOL_AXIS_ALIGN = "tool_axis_align"
    FREE = "free"


class Blend(str, Enum):
    NONE = "none"
    SMOOTH = "smooth"
    SHARP = "sharp"


# Default per-axis (rx, ry, rz) tolerances per intent, in radians.
# Roughly pi means "free axis"; small values mean "tight."
import math  # noqa: E402

_PI = math.pi
DEFAULT_TOLERANCE_BY_INTENT: Dict[OrientationIntent, Tuple[float, float, float]] = {
    OrientationIntent.EXACT: (0.05, 0.05, 0.05),
    OrientationIntent.YAW_ONLY: (_PI, _PI, 0.05),
    OrientationIntent.TOOL_AXIS_ALIGN: (0.05, 0.05, _PI),
    OrientationIntent.FREE: (_PI, _PI, _PI),
}


# ---- Pose & frames ---------------------------------------------------


@dataclass(frozen=True)
class Pose:
    position_m: Tuple[float, float, float]
    orientation_quat_xyzw: Tuple[float, float, float, float]


@dataclass(frozen=True)
class ObjectFrame:
    parent: str
    pose: Optional[Pose]  # None == "unresolved at capture time"


@dataclass(frozen=True)
class Frames:
    world: str = "robot_base"
    objects: Dict[str, ObjectFrame] = field(default_factory=dict)
    tool_offset_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)


# ---- Steps -----------------------------------------------------------


@dataclass(frozen=True)
class MoveStep:
    """A Cartesian-anchored move target.

    ``orientation_intent`` is required. ``orientation_tolerance_rad``
    is optional; if omitted, ``DEFAULT_TOLERANCE_BY_INTENT`` applies.
    """

    motion: Motion
    pose_frame: str
    pose: Pose
    orientation_intent: OrientationIntent
    orientation_tolerance_rad: Optional[Tuple[float, float, float]] = None
    joint_hint_rad: Optional[Tuple[float, ...]] = None
    speed_pct: Optional[float] = None
    accel_pct: Optional[float] = None
    blend: Optional[Blend] = None
    tolerance_m: Optional[float] = None

    kind: Literal["move"] = "move"

    def effective_tolerance_rad(self) -> Tuple[float, float, float]:
        """Resolved per-axis orientation tolerance (intent default if unset)."""
        if self.orientation_tolerance_rad is not None:
            return self.orientation_tolerance_rad
        return DEFAULT_TOLERANCE_BY_INTENT[self.orientation_intent]


@dataclass(frozen=True)
class ToolStep:
    tool: str
    action: str
    settle_ms: int = 0
    kind: Literal["tool"] = "tool"


@dataclass(frozen=True)
class WaitStep:
    duration_ms: int
    kind: Literal["wait"] = "wait"


@dataclass(frozen=True)
class SetFrameStep:
    """Bind an object frame's pose at runtime (e.g., probed fixture)."""

    frame_name: str
    pose: Pose
    kind: Literal["set_frame"] = "set_frame"


Step = Union[MoveStep, ToolStep, WaitStep, SetFrameStep]


# ---- Program top-level ------------------------------------------------


@dataclass(frozen=True)
class Source:
    capture_id: str
    captured_at: str  # ISO-8601 timestamp string
    demonstrator: str = "hand_vr"


@dataclass(frozen=True)
class Defaults:
    speed_pct: float = 50.0
    accel_pct: float = 50.0
    blend: Blend = Blend.SMOOTH


@dataclass(frozen=True)
class Annotations:
    skill_hint: Optional[str] = None
    task_label: Optional[str] = None


@dataclass(frozen=True)
class CanonicalProgram:
    program_id: str
    source: Source
    frames: Frames
    defaults: Defaults
    steps: Tuple[Step, ...]
    calibration: Optional[CalibrationBinding] = None
    annotations: Annotations = field(default_factory=Annotations)

    schema_version: str = SCHEMA_VERSION
    action_layout_compat: Optional[str] = "open_x_embodiment_7dof"
