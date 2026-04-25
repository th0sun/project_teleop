"""JSON Schema for the canonical program (v0.1).

The schema is the contract between lifter, adapters, and any future
program editor. ``orientation_intent`` and ``pose_frame`` are required
on every ``move`` step. Programs that omit them MUST be rejected.

We deliberately keep ``validate_program_dict`` self-contained (no
``jsonschema`` dependency) so this package builds in a minimal
environment. The rules below are the v0.1 contract; richer JSON
Schema can be added once a real validator is on the path.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Tuple

from teaching_core.calibration.types import CalibrationMethod, SourceSpace
from teaching_core.program.types import (
    Blend,
    Motion,
    OrientationIntent,
    SCHEMA_VERSION,
)


class SchemaValidationError(ValueError):
    """Raised when a program dict does not conform to the v0.1 schema."""


# Human-readable schema (for docs + golden output). Kept inline
# rather than as a JSON file so single-file edits stay coherent.
SCHEMA: Dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "title": "robot_teaching_core canonical program",
    "type": "object",
    "required": [
        "schema_version",
        "program_id",
        "source",
        "frames",
        "defaults",
        "steps",
    ],
    "properties": {
        "schema_version": {"const": SCHEMA_VERSION},
        "program_id": {"type": "string", "minLength": 1},
        "action_layout_compat": {"type": ["string", "null"]},
        "source": {
            "type": "object",
            "required": ["capture_id", "captured_at"],
            "properties": {
                "capture_id": {"type": "string"},
                "captured_at": {"type": "string"},
                "demonstrator": {"type": "string"},
            },
        },
        "frames": {
            "type": "object",
            "required": ["world", "tool_offset_m"],
            "properties": {
                "world": {"type": "string"},
                "objects": {"type": "object"},
                "tool_offset_m": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                },
            },
        },
        "defaults": {"type": "object"},
        "annotations": {"type": "object"},
        "calibration": {"$ref": "#/$defs/calibration"},
        "steps": {
            "type": "array",
            "items": {"$ref": "#/$defs/step"},
        },
    },
    "$defs": {
        "pose": {
            "type": "object",
            "required": ["position_m", "orientation_quat_xyzw"],
            "properties": {
                "position_m": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                },
                "orientation_quat_xyzw": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 4,
                    "maxItems": 4,
                },
            },
        },
        "spatial_transform": {
            "type": "object",
            "required": ["translation_m", "rotation_quat_xyzw"],
            "properties": {
                "translation_m": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 3,
                    "maxItems": 3,
                },
                "rotation_quat_xyzw": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 4,
                    "maxItems": 4,
                },
                "uniform_scale": {"type": "number", "exclusiveMinimum": 0.0},
            },
        },
        "calibration": {
            "type": "object",
            "required": [
                "source_space",
                "task_frame",
                "source_to_task_transform",
                "method",
                "quality",
            ],
            "properties": {
                "source_space": {"enum": [s.value for s in SourceSpace]},
                "task_frame": {"type": "string"},
                "source_to_task_transform": {"$ref": "#/$defs/spatial_transform"},
                "method": {"enum": [m.value for m in CalibrationMethod]},
                "quality": {
                    "type": "object",
                    "required": ["rms_error_m"],
                    "properties": {
                        "rms_error_m": {"type": "number", "minimum": 0.0},
                    },
                },
                "workspace_feedback": {"type": "string"},
            },
        },
        "step": {
            "oneOf": [
                {"$ref": "#/$defs/move_step"},
                {"$ref": "#/$defs/tool_step"},
                {"$ref": "#/$defs/wait_step"},
                {"$ref": "#/$defs/set_frame_step"},
            ],
        },
        "move_step": {
            "type": "object",
            "required": [
                "kind",
                "motion",
                "pose_frame",
                "pose",
                "orientation_intent",
            ],
            "properties": {
                "kind": {"const": "move"},
                "motion": {"enum": [m.value for m in Motion]},
                "pose_frame": {"type": "string"},
                "pose": {"$ref": "#/$defs/pose"},
                "orientation_intent": {
                    "enum": [i.value for i in OrientationIntent],
                },
                "orientation_tolerance_rad": {
                    "type": "array",
                    "items": {"type": "number", "minimum": 0.0},
                    "minItems": 3,
                    "maxItems": 3,
                },
                "joint_hint_rad": {
                    "type": "array",
                    "items": {"type": "number"},
                },
                "speed_pct": {"type": "number"},
                "accel_pct": {"type": "number"},
                "blend": {"enum": [b.value for b in Blend]},
                "tolerance_m": {"type": "number", "minimum": 0.0},
            },
        },
        "tool_step": {
            "type": "object",
            "required": ["kind", "tool", "action"],
            "properties": {
                "kind": {"const": "tool"},
                "tool": {"type": "string"},
                "action": {"type": "string"},
                "settle_ms": {"type": "integer", "minimum": 0},
            },
        },
        "wait_step": {
            "type": "object",
            "required": ["kind", "duration_ms"],
            "properties": {
                "kind": {"const": "wait"},
                "duration_ms": {"type": "integer", "minimum": 0},
            },
        },
        "set_frame_step": {
            "type": "object",
            "required": ["kind", "frame_name", "pose"],
            "properties": {
                "kind": {"const": "set_frame"},
                "frame_name": {"type": "string"},
                "pose": {"$ref": "#/$defs/pose"},
            },
        },
    },
}

SCHEMA_JSON: str = json.dumps(SCHEMA, indent=2, sort_keys=True)


# ---- Hand-written validator (no external deps) -----------------------


_VALID_MOTIONS = {m.value for m in Motion}
_VALID_INTENTS = {i.value for i in OrientationIntent}
_VALID_BLENDS = {b.value for b in Blend}
_VALID_KINDS = {"move", "tool", "wait", "set_frame"}
_VALID_SOURCE_SPACES = {s.value for s in SourceSpace}
_VALID_CALIBRATION_METHODS = {m.value for m in CalibrationMethod}
_VR_DEMONSTRATORS = {"hand_vr", "unity_vr", "vr"}


def _check(cond: bool, msg: str) -> None:
    if not cond:
        raise SchemaValidationError(msg)


def _validate_pose(d: Any, where: str) -> None:
    _check(isinstance(d, dict), f"{where}: pose must be object")
    for key, n in (("position_m", 3), ("orientation_quat_xyzw", 4)):
        _check(key in d, f"{where}: pose missing '{key}'")
        v = d[key]
        _check(
            isinstance(v, list) and len(v) == n
            and all(isinstance(x, (int, float)) for x in v),
            f"{where}: pose.{key} must be {n} numbers",
        )


def _validate_number_array(d: Any, n: int, where: str) -> None:
    _check(
        isinstance(d, list) and len(d) == n
        and all(isinstance(x, (int, float)) for x in d),
        f"{where}: must be {n} numbers",
    )


def _validate_calibration(d: Any) -> None:
    where = "calibration"
    _check(isinstance(d, dict), f"{where}: must be object")
    for key in (
        "source_space",
        "task_frame",
        "source_to_task_transform",
        "method",
        "quality",
    ):
        _check(key in d, f"{where}: missing required '{key}'")
    _check(
        d["source_space"] in _VALID_SOURCE_SPACES,
        f"{where}: invalid source_space {d['source_space']!r}",
    )
    _check(isinstance(d["task_frame"], str) and d["task_frame"],
           f"{where}: task_frame must be non-empty string")
    _check(
        d["method"] in _VALID_CALIBRATION_METHODS,
        f"{where}: invalid method {d['method']!r}",
    )

    transform = d["source_to_task_transform"]
    _check(isinstance(transform, dict),
           f"{where}: source_to_task_transform must be object")
    for key, n in (("translation_m", 3), ("rotation_quat_xyzw", 4)):
        _check(key in transform,
               f"{where}: source_to_task_transform missing '{key}'")
        _validate_number_array(
            transform[key], n, f"{where}: source_to_task_transform.{key}"
        )
    if "uniform_scale" in transform:
        scale = transform["uniform_scale"]
        _check(
            isinstance(scale, (int, float)) and scale > 0.0,
            f"{where}: source_to_task_transform.uniform_scale must be > 0",
        )

    quality = d["quality"]
    _check(isinstance(quality, dict), f"{where}: quality must be object")
    _check("rms_error_m" in quality, f"{where}: quality missing 'rms_error_m'")
    rms = quality["rms_error_m"]
    _check(
        isinstance(rms, (int, float)) and rms >= 0.0,
        f"{where}: quality.rms_error_m must be non-negative number",
    )


def _validate_move_step(d: Dict[str, Any], i: int) -> None:
    where = f"steps[{i}] (move)"
    for key in ("motion", "pose_frame", "pose", "orientation_intent"):
        _check(key in d, f"{where}: missing required '{key}'")
    _check(d["motion"] in _VALID_MOTIONS, f"{where}: invalid motion {d['motion']!r}")
    _check(isinstance(d["pose_frame"], str), f"{where}: pose_frame must be string")
    _check(
        d["orientation_intent"] in _VALID_INTENTS,
        f"{where}: invalid orientation_intent {d['orientation_intent']!r}",
    )
    _validate_pose(d["pose"], where)
    if "orientation_tolerance_rad" in d:
        v = d["orientation_tolerance_rad"]
        _check(
            isinstance(v, list) and len(v) == 3
            and all(isinstance(x, (int, float)) and x >= 0.0 for x in v),
            f"{where}: orientation_tolerance_rad must be 3 non-negative numbers",
        )
    if "blend" in d:
        _check(d["blend"] in _VALID_BLENDS, f"{where}: invalid blend {d['blend']!r}")


def _validate_tool_step(d: Dict[str, Any], i: int) -> None:
    where = f"steps[{i}] (tool)"
    for key in ("tool", "action"):
        _check(key in d, f"{where}: missing required '{key}'")


def _validate_wait_step(d: Dict[str, Any], i: int) -> None:
    where = f"steps[{i}] (wait)"
    _check("duration_ms" in d, f"{where}: missing 'duration_ms'")
    _check(
        isinstance(d["duration_ms"], int) and d["duration_ms"] >= 0,
        f"{where}: duration_ms must be non-negative integer",
    )


def _validate_set_frame_step(d: Dict[str, Any], i: int) -> None:
    where = f"steps[{i}] (set_frame)"
    for key in ("frame_name", "pose"):
        _check(key in d, f"{where}: missing required '{key}'")
    _validate_pose(d["pose"], where)


_STEP_VALIDATORS = {
    "move": _validate_move_step,
    "tool": _validate_tool_step,
    "wait": _validate_wait_step,
    "set_frame": _validate_set_frame_step,
}


def validate_program_dict(d: Any) -> None:
    """Validate a canonical-program dict against the v0.1 schema.

    Raises ``SchemaValidationError`` on the first violation found.
    """
    _check(isinstance(d, dict), "program: top-level must be object")
    for key in ("schema_version", "program_id", "source", "frames",
                "defaults", "steps"):
        _check(key in d, f"program: missing required '{key}'")
    _check(
        d["schema_version"] == SCHEMA_VERSION,
        f"program: schema_version must be {SCHEMA_VERSION!r}",
    )
    _check(isinstance(d["program_id"], str) and d["program_id"],
           "program: program_id must be non-empty string")

    src = d["source"]
    _check(isinstance(src, dict), "program: source must be object")
    for key in ("capture_id", "captured_at"):
        _check(key in src, f"source: missing required '{key}'")
    demonstrator = src.get("demonstrator", "hand_vr")

    frames = d["frames"]
    _check(isinstance(frames, dict), "program: frames must be object")
    _check("world" in frames, "frames: missing 'world'")
    _check("tool_offset_m" in frames, "frames: missing 'tool_offset_m'")
    tom = frames["tool_offset_m"]
    _check(
        isinstance(tom, list) and len(tom) == 3
        and all(isinstance(x, (int, float)) for x in tom),
        "frames: tool_offset_m must be 3 numbers",
    )

    if "calibration" in d:
        _validate_calibration(d["calibration"])
    elif demonstrator in _VR_DEMONSTRATORS:
        raise SchemaValidationError(
            "program: VR-captured source requires 'calibration' block"
        )

    steps = d["steps"]
    _check(isinstance(steps, list), "program: steps must be array")
    for i, step in enumerate(steps):
        _check(isinstance(step, dict), f"steps[{i}]: must be object")
        kind = step.get("kind")
        _check(
            kind in _VALID_KINDS,
            f"steps[{i}]: unknown kind {kind!r}; v0.1 supports {_VALID_KINDS}",
        )
        _STEP_VALIDATORS[kind](step, i)
