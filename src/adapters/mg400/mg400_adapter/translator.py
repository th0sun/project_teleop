"""Translate canonical programs into current MG400 native commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Tuple

import numpy as np

from mg400_controller.common.config import motion_config
from mg400_controller.common.config.robot_config import CONTROL_MODE
from mg400_controller.common.logic.motion_planner import MotionPlanner
from teaching_core.adapter_api import UnsupportedStep
from teaching_core.program.types import (
    CanonicalProgram,
    MoveStep,
    Motion,
    SetFrameStep,
    ToolStep,
    WaitStep,
)


class _NullLogger:
    def info(self, message):  # pragma: no cover - compatibility shim
        pass

    def warn(self, message):  # pragma: no cover - compatibility shim
        pass

    def error(self, message):  # pragma: no cover - compatibility shim
        pass


@dataclass(frozen=True)
class MG400NativeCommand:
    kind: Literal["motion", "digital_output", "wait"]
    command: str = ""
    duration_ms: int = 0


@dataclass(frozen=True)
class MG400CommandPlan:
    commands: Tuple[MG400NativeCommand, ...]
    control_mode: str


def _speed_pct(value: float) -> int:
    return max(1, min(100, int(round(value))))


def _translate_move(step: MoveStep, planner: MotionPlanner, speed_pct: int):
    if step.motion != Motion.JOINT:
        raise UnsupportedStep(
            "MG400 adapter v0 only translates motion='joint' steps. "
            "Cartesian moves require the later MG400 IK/translator pass."
        )
    if step.joint_hint_rad is None or len(step.joint_hint_rad) < 4:
        raise UnsupportedStep(
            "MG400 joint move requires joint_hint_rad with at least 4 joints."
        )

    q_rad = np.asarray(step.joint_hint_rad[:4], dtype=float)
    return MG400NativeCommand(
        kind="motion",
        command=planner.format_command(q_rad, speed_pct),
    )


def _translate_tool(step: ToolStep):
    if step.tool != "vacuum" or step.action not in {"on", "off"}:
        raise UnsupportedStep(
            f"MG400 adapter v0 does not support tool step {step.tool}:{step.action}"
        )

    if step.action == "on":
        return (
            MG400NativeCommand(
                kind="digital_output",
                command=f"DOExecute({motion_config.VACUUM_DO_PORT}, 1)",
            ),
            MG400NativeCommand(
                kind="digital_output",
                command=f"DOExecute({motion_config.BLOW_DO_PORT}, 0)",
            ),
        )

    blow_ms = int(round(motion_config.BLOW_DURATION * 1000))
    return (
        MG400NativeCommand(
            kind="digital_output",
            command=f"DOExecute({motion_config.VACUUM_DO_PORT}, 0)",
        ),
        MG400NativeCommand(
            kind="digital_output",
            command=f"DOExecute({motion_config.BLOW_DO_PORT}, 1)",
        ),
        MG400NativeCommand(kind="wait", duration_ms=blow_ms),
        MG400NativeCommand(
            kind="digital_output",
            command=f"DOExecute({motion_config.BLOW_DO_PORT}, 0)",
        ),
    )


def translate_program(
    program: CanonicalProgram,
    *,
    control_mode: str = CONTROL_MODE,
    logger=None,
) -> MG400CommandPlan:
    """Translate the currently safe subset of the canonical IR to MG400 commands."""
    planner = MotionPlanner(control_mode, logger or _NullLogger())
    commands = []

    for step in program.steps:
        if isinstance(step, MoveStep):
            speed = _speed_pct(step.speed_pct or program.defaults.speed_pct)
            commands.append(_translate_move(step, planner, speed))
        elif isinstance(step, ToolStep):
            commands.extend(_translate_tool(step))
        elif isinstance(step, WaitStep):
            commands.append(
                MG400NativeCommand(kind="wait", duration_ms=step.duration_ms)
            )
        elif isinstance(step, SetFrameStep):
            raise UnsupportedStep(
                "MG400 adapter v0 requires object frames to be resolved before "
                "translation."
            )
        else:  # pragma: no cover - dataclass union should be exhaustive
            raise UnsupportedStep(f"unsupported canonical step: {step!r}")

    return MG400CommandPlan(commands=tuple(commands), control_mode=control_mode)
