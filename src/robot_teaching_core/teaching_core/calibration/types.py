"""Calibration contracts for grounding VR captures into task frames.

The core rule from proposal v1.3 is intentionally conservative:
OpenXR/Unity space is an operator-space source, not a robot workspace.
A VR-captured program needs an explicit calibration binding before it
can be treated as replayable.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class SourceSpace(str, Enum):
    OPENXR_STAGE = "openxr_stage"
    UNITY_WORLD = "unity_world"
    HAND_AUTHORED = "hand_authored"
    ROBOT_TASK = "robot_task"


class CalibrationMethod(str, Enum):
    THREE_POINT_TABLE_FIXTURE = "three_point_table_fixture"
    OBJECT_FRAME_BIND = "object_frame_bind"
    ROBOT_PROBE = "robot_probe"
    MANUAL_TRANSFORM = "manual_transform"


@dataclass(frozen=True)
class SpatialTransform:
    """Similarity transform from a source space into a task frame."""

    translation_m: Tuple[float, float, float]
    rotation_quat_xyzw: Tuple[float, float, float, float]
    uniform_scale: float = 1.0


@dataclass(frozen=True)
class CalibrationQuality:
    """Quality metadata for calibration acquisition."""

    rms_error_m: float


@dataclass(frozen=True)
class CalibrationBinding:
    """How a source capture frame was grounded into a task frame."""

    source_space: SourceSpace
    task_frame: str
    source_to_task_transform: SpatialTransform
    method: CalibrationMethod
    quality: CalibrationQuality
    workspace_feedback: str = "advisory_only"
