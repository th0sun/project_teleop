"""Shared test fixtures (in-memory canonical programs)."""

from __future__ import annotations

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
    ToolStep,
    WaitStep,
)


def make_minimal_program() -> CanonicalProgram:
    """Smallest program that exercises every v0.1 step kind."""
    move1 = MoveStep(
        motion=Motion.LINEAR,
        pose_frame="world",
        pose=Pose(
            position_m=(0.30, 0.0, 0.20),
            orientation_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
        orientation_intent=OrientationIntent.YAW_ONLY,
        joint_hint_rad=(0.0, 0.4, -0.6, 0.0),
        speed_pct=40.0,
        blend=Blend.SMOOTH,
        tolerance_m=0.002,
    )
    tool_on = ToolStep(tool="vacuum", action="on", settle_ms=150)
    move2 = MoveStep(
        motion=Motion.LINEAR,
        pose_frame="target_part",
        pose=Pose(
            position_m=(0.0, 0.0, 0.05),
            orientation_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
        orientation_intent=OrientationIntent.EXACT,
        orientation_tolerance_rad=(0.05, 0.05, 0.02),
    )
    wait = WaitStep(duration_ms=200)
    bind = SetFrameStep(
        frame_name="target_part",
        pose=Pose(
            position_m=(0.30, 0.0, 0.0),
            orientation_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ),
    )
    return CanonicalProgram(
        program_id="pick_demo_2026-04-25_01",
        source=Source(
            capture_id="unity_session_abc123",
            captured_at="2026-04-25T10:12:03Z",
            demonstrator="hand_vr",
        ),
        frames=Frames(
            world="robot_base",
            objects={"target_part": ObjectFrame(parent="world", pose=None)},
            tool_offset_m=(0.0, 0.0, 0.0),
        ),
        calibration=CalibrationBinding(
            source_space=SourceSpace.OPENXR_STAGE,
            task_frame="world",
            source_to_task_transform=SpatialTransform(
                translation_m=(0.10, -0.03, 0.72),
                rotation_quat_xyzw=(0.0, 0.0, 0.7071, 0.7071),
                uniform_scale=1.0,
            ),
            method=CalibrationMethod.THREE_POINT_TABLE_FIXTURE,
            quality=CalibrationQuality(rms_error_m=0.008),
        ),
        defaults=Defaults(speed_pct=50.0, accel_pct=50.0, blend=Blend.SMOOTH),
        steps=(move1, tool_on, bind, move2, wait),
        annotations=Annotations(skill_hint=None, task_label="pick_from_bin"),
    )
