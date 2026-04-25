"""Translate canonical programs into current MG400 native commands."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Literal, Tuple

import numpy as np

from mg400_controller.common.config import motion_config
from mg400_controller.common.config.robot_config import CONTROL_MODE
from mg400_protocol.commands import do_execute, joint_mov_j
from teaching_core.adapter_api import UnsupportedStep
from teaching_core.program.types import (
    CanonicalProgram,
    MoveStep,
    Motion,
    SetFrameStep,
    ToolStep,
    WaitStep,
)


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


def _translate_move(step: MoveStep, control_mode: str, speed_pct: int):
    if step.motion != Motion.JOINT:
        raise UnsupportedStep(
            "MG400 adapter v0 only translates motion='joint' steps. "
            "Cartesian moves require the later MG400 IK/translator pass."
        )
    if control_mode != "jointmovj":
        raise UnsupportedStep(
            "MG400 adapter v0 only exposes protocol-backed JointMovJ. "
            "MovJ/MovL support should be added through mg400_protocol."
        )
    if step.joint_hint_rad is None or len(step.joint_hint_rad) < 4:
        raise UnsupportedStep(
            "MG400 joint move requires joint_hint_rad with at least 4 joints."
        )

    q_deg = np.degrees(np.asarray(step.joint_hint_rad[:4], dtype=float))
    return MG400NativeCommand(
        kind="motion",
        command=joint_mov_j(
            q_deg,
            speed_j=speed_pct,
            acc_j=motion_config.ACC_VALUE,
            cp=motion_config.CP_VALUE,
        ).render(),
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
                command=do_execute(motion_config.VACUUM_DO_PORT, True).render(),
            ),
            MG400NativeCommand(
                kind="digital_output",
                command=do_execute(motion_config.BLOW_DO_PORT, False).render(),
            ),
        )

    blow_ms = int(round(motion_config.BLOW_DURATION * 1000))
    return (
        MG400NativeCommand(
            kind="digital_output",
            command=do_execute(motion_config.VACUUM_DO_PORT, False).render(),
        ),
        MG400NativeCommand(
            kind="digital_output",
            command=do_execute(motion_config.BLOW_DO_PORT, True).render(),
        ),
        MG400NativeCommand(kind="wait", duration_ms=blow_ms),
        MG400NativeCommand(
            kind="digital_output",
            command=do_execute(motion_config.BLOW_DO_PORT, False).render(),
        ),
    )


def translate_program(
    program: CanonicalProgram,
    *,
    control_mode: str = CONTROL_MODE,
    logger=None,
) -> MG400CommandPlan:
    """Translate the currently safe subset of the canonical IR to MG400 commands."""
    commands = []

    for step in program.steps:
        if isinstance(step, MoveStep):
            speed = _speed_pct(step.speed_pct or program.defaults.speed_pct)
            commands.append(_translate_move(step, control_mode, speed))
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


def plan_to_dict(plan: MG400CommandPlan) -> Dict[str, Any]:
    """Serialise an MG400CommandPlan to a JSON-safe dict (for export/debug)."""
    def _cmd(c: MG400NativeCommand) -> Dict[str, Any]:
        out: Dict[str, Any] = {"kind": c.kind}
        if c.command:
            out["command"] = c.command
        if c.duration_ms:
            out["duration_ms"] = c.duration_ms
        return out

    return {
        "robot_id": "dobot_mg400",
        "control_mode": plan.control_mode,
        "command_count": len(plan.commands),
        "commands": [_cmd(c) for c in plan.commands],
    }
