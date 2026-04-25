"""Canonical program JSON IO + dataclass <-> dict round-trip.

``load_program`` and ``dump_program`` always run schema validation.
Programs that fail validation never reach adapter code.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

from teaching_core.program.schema import (
    SchemaValidationError,
    validate_program_dict,
)
from teaching_core.calibration.types import (
    CalibrationBinding,
    CalibrationMethod,
    CalibrationQuality,
    SourceSpace,
    SpatialTransform,
)
from teaching_core.program.types import (
    Annotations,
    Blend,
    CanonicalProgram,
    Defaults,
    Frames,
    Motion,
    MoveStep,
    ObjectFrame,
    OrientationIntent,
    Pose,
    SetFrameStep,
    Source,
    Step,
    ToolStep,
    WaitStep,
    SCHEMA_VERSION,
)


# ---- to-dict helpers --------------------------------------------------


def _pose_to_dict(p: Pose) -> Dict[str, Any]:
    return {
        "position_m": list(p.position_m),
        "orientation_quat_xyzw": list(p.orientation_quat_xyzw),
    }


def _calibration_to_dict(c: CalibrationBinding) -> Dict[str, Any]:
    t = c.source_to_task_transform
    return {
        "source_space": c.source_space.value,
        "task_frame": c.task_frame,
        "source_to_task_transform": {
            "translation_m": list(t.translation_m),
            "rotation_quat_xyzw": list(t.rotation_quat_xyzw),
            "uniform_scale": t.uniform_scale,
        },
        "method": c.method.value,
        "quality": {"rms_error_m": c.quality.rms_error_m},
        "workspace_feedback": c.workspace_feedback,
    }


def _step_to_dict(s: Step) -> Dict[str, Any]:
    if isinstance(s, MoveStep):
        out: Dict[str, Any] = {
            "kind": "move",
            "motion": s.motion.value,
            "pose_frame": s.pose_frame,
            "pose": _pose_to_dict(s.pose),
            "orientation_intent": s.orientation_intent.value,
        }
        if s.orientation_tolerance_rad is not None:
            out["orientation_tolerance_rad"] = list(s.orientation_tolerance_rad)
        if s.joint_hint_rad is not None:
            out["joint_hint_rad"] = list(s.joint_hint_rad)
        if s.speed_pct is not None:
            out["speed_pct"] = s.speed_pct
        if s.accel_pct is not None:
            out["accel_pct"] = s.accel_pct
        if s.blend is not None:
            out["blend"] = s.blend.value
        if s.tolerance_m is not None:
            out["tolerance_m"] = s.tolerance_m
        return out
    if isinstance(s, ToolStep):
        return {
            "kind": "tool",
            "tool": s.tool,
            "action": s.action,
            "settle_ms": s.settle_ms,
        }
    if isinstance(s, WaitStep):
        return {"kind": "wait", "duration_ms": s.duration_ms}
    if isinstance(s, SetFrameStep):
        return {
            "kind": "set_frame",
            "frame_name": s.frame_name,
            "pose": _pose_to_dict(s.pose),
        }
    raise TypeError(f"unknown step type: {type(s).__name__}")


def program_to_dict(p: CanonicalProgram) -> Dict[str, Any]:
    objects_out: Dict[str, Any] = {}
    for name, frame in p.frames.objects.items():
        objects_out[name] = {
            "parent": frame.parent,
            "pose": _pose_to_dict(frame.pose) if frame.pose is not None else None,
        }
    out: Dict[str, Any] = {
        "schema_version": p.schema_version,
        "program_id": p.program_id,
        "source": {
            "capture_id": p.source.capture_id,
            "captured_at": p.source.captured_at,
            "demonstrator": p.source.demonstrator,
        },
        "frames": {
            "world": p.frames.world,
            "objects": objects_out,
            "tool_offset_m": list(p.frames.tool_offset_m),
        },
        "defaults": {
            "speed_pct": p.defaults.speed_pct,
            "accel_pct": p.defaults.accel_pct,
            "blend": p.defaults.blend.value,
        },
        "steps": [_step_to_dict(s) for s in p.steps],
        "annotations": {
            "skill_hint": p.annotations.skill_hint,
            "task_label": p.annotations.task_label,
        },
    }
    if p.action_layout_compat is not None:
        out["action_layout_compat"] = p.action_layout_compat
    if p.calibration is not None:
        out["calibration"] = _calibration_to_dict(p.calibration)
    return out


# ---- from-dict helpers ------------------------------------------------


def _pose_from_dict(d: Dict[str, Any]) -> Pose:
    return Pose(
        position_m=tuple(d["position_m"]),  # type: ignore[arg-type]
        orientation_quat_xyzw=tuple(d["orientation_quat_xyzw"]),  # type: ignore[arg-type]
    )


def _calibration_from_dict(d: Dict[str, Any]) -> CalibrationBinding:
    transform_d = d["source_to_task_transform"]
    quality_d = d["quality"]
    return CalibrationBinding(
        source_space=SourceSpace(d["source_space"]),
        task_frame=d["task_frame"],
        source_to_task_transform=SpatialTransform(
            translation_m=tuple(transform_d["translation_m"]),  # type: ignore[arg-type]
            rotation_quat_xyzw=tuple(transform_d["rotation_quat_xyzw"]),  # type: ignore[arg-type]
            uniform_scale=transform_d.get("uniform_scale", 1.0),
        ),
        method=CalibrationMethod(d["method"]),
        quality=CalibrationQuality(rms_error_m=quality_d["rms_error_m"]),
        workspace_feedback=d.get("workspace_feedback", "advisory_only"),
    )


def _step_from_dict(d: Dict[str, Any]) -> Step:
    kind = d["kind"]
    if kind == "move":
        return MoveStep(
            motion=Motion(d["motion"]),
            pose_frame=d["pose_frame"],
            pose=_pose_from_dict(d["pose"]),
            orientation_intent=OrientationIntent(d["orientation_intent"]),
            orientation_tolerance_rad=(
                tuple(d["orientation_tolerance_rad"])  # type: ignore[arg-type]
                if "orientation_tolerance_rad" in d else None
            ),
            joint_hint_rad=(
                tuple(d["joint_hint_rad"])  # type: ignore[arg-type]
                if "joint_hint_rad" in d else None
            ),
            speed_pct=d.get("speed_pct"),
            accel_pct=d.get("accel_pct"),
            blend=Blend(d["blend"]) if "blend" in d else None,
            tolerance_m=d.get("tolerance_m"),
        )
    if kind == "tool":
        return ToolStep(
            tool=d["tool"],
            action=d["action"],
            settle_ms=d.get("settle_ms", 0),
        )
    if kind == "wait":
        return WaitStep(duration_ms=d["duration_ms"])
    if kind == "set_frame":
        return SetFrameStep(
            frame_name=d["frame_name"],
            pose=_pose_from_dict(d["pose"]),
        )
    raise SchemaValidationError(f"unknown step kind {kind!r}")


def program_from_dict(d: Dict[str, Any]) -> CanonicalProgram:
    """Build a ``CanonicalProgram`` from a dict (validates first)."""
    validate_program_dict(d)

    frames_d = d["frames"]
    objects_in = frames_d.get("objects", {}) or {}
    objects: Dict[str, ObjectFrame] = {}
    for name, frame_d in objects_in.items():
        pose_d = frame_d.get("pose")
        objects[name] = ObjectFrame(
            parent=frame_d["parent"],
            pose=_pose_from_dict(pose_d) if pose_d is not None else None,
        )

    defaults_d = d.get("defaults", {})
    annotations_d = d.get("annotations", {}) or {}
    src_d = d["source"]

    return CanonicalProgram(
        program_id=d["program_id"],
        schema_version=d["schema_version"],
        action_layout_compat=d.get("action_layout_compat"),
        source=Source(
            capture_id=src_d["capture_id"],
            captured_at=src_d["captured_at"],
            demonstrator=src_d.get("demonstrator", "hand_vr"),
        ),
        frames=Frames(
            world=frames_d["world"],
            objects=objects,
            tool_offset_m=tuple(frames_d["tool_offset_m"]),  # type: ignore[arg-type]
        ),
        calibration=(
            _calibration_from_dict(d["calibration"])
            if d.get("calibration") is not None else None
        ),
        defaults=Defaults(
            speed_pct=defaults_d.get("speed_pct", 50.0),
            accel_pct=defaults_d.get("accel_pct", 50.0),
            blend=Blend(defaults_d.get("blend", Blend.SMOOTH.value)),
        ),
        steps=tuple(_step_from_dict(s) for s in d["steps"]),
        annotations=Annotations(
            skill_hint=annotations_d.get("skill_hint"),
            task_label=annotations_d.get("task_label"),
        ),
    )


# ---- file-level helpers ----------------------------------------------


def load_program(path: Union[str, Path]) -> CanonicalProgram:
    """Load and validate a canonical program from a JSON file."""
    text = Path(path).read_text(encoding="utf-8")
    return program_from_dict(json.loads(text))


def dump_program(
    p: CanonicalProgram,
    path: Union[str, Path],
    *,
    indent: Optional[int] = 2,
) -> None:
    """Validate and write a canonical program to a JSON file."""
    d = program_to_dict(p)
    validate_program_dict(d)
    Path(path).write_text(
        json.dumps(d, indent=indent, sort_keys=False) + "\n",
        encoding="utf-8",
    )
