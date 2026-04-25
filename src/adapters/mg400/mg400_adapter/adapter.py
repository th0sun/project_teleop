"""RobotAdapter implementation for the current MG400 runtime."""

from __future__ import annotations

import re
import time
from typing import Optional

from mg400_adapter.profile import make_mg400_profile
from mg400_adapter.translator import MG400CommandPlan, translate_program
from teaching_core.adapter_api import (
    AdaptedPlan,
    CancelToken,
    ExecutionMode,
    ExecutionResult,
    ProgressSink,
    UnsupportedStep,
    profile_supports_execution_mode,
)
from teaching_core.program.types import CanonicalProgram


class MG400Adapter:
    """Adapter facade over the existing MG400 command sender."""

    def __init__(self, *, sender=None, logger=None, control_mode=None):
        self.profile = make_mg400_profile()
        self.sender = sender
        self.logger = logger
        self.control_mode = control_mode

    def plan(self, program: CanonicalProgram, mode: ExecutionMode) -> AdaptedPlan:
        if not profile_supports_execution_mode(self.profile, mode):
            raise UnsupportedStep(
                f"{self.profile.robot_id} does not support {mode.value}"
            )
        payload = translate_program(
            program,
            **({"control_mode": self.control_mode} if self.control_mode else {}),
            logger=self.logger,
        )
        return AdaptedPlan(
            robot_id=self.profile.robot_id,
            mode=mode,
            payload=payload,
            diagnostics=(
                "MG400 adapter v0 executes joint-hint programs; Cartesian IK is "
                "not implemented yet.",
            ),
        )

    def execute(
        self,
        plan: AdaptedPlan,
        *,
        cancel: CancelToken,
        progress: Optional[ProgressSink] = None,
    ) -> ExecutionResult:
        if cancel.cancelled:
            return ExecutionResult(success=False, message="cancelled")
        if not isinstance(plan.payload, MG400CommandPlan):
            return ExecutionResult(success=False, message="unexpected MG400 plan")

        if plan.mode == ExecutionMode.OFFLINE_EXPORT:
            if progress is not None:
                progress.on_progress("offline export ready", 1.0)
            return ExecutionResult(
                success=True,
                message=f"{len(plan.payload.commands)} MG400 commands planned",
            )

        if self.sender is None:
            return ExecutionResult(
                success=False,
                message="queued execution requires an MG400 CommandSender",
            )

        return self._execute_queued(plan.payload, cancel=cancel, progress=progress)

    def _execute_queued(
        self,
        payload: MG400CommandPlan,
        *,
        cancel: CancelToken,
        progress: Optional[ProgressSink],
    ) -> ExecutionResult:
        total = len(payload.commands)
        for index, command in enumerate(payload.commands, start=1):
            if cancel.cancelled:
                return ExecutionResult(success=False, message="cancelled")
            if command.kind == "wait":
                time.sleep(command.duration_ms / 1000.0)
            elif command.kind == "digital_output":
                self._send_digital_output(command.command)
            else:
                if not self.sender.send(command.command):
                    return ExecutionResult(
                        success=False,
                        message=f"failed to send command {index}/{total}",
                    )
            if progress is not None:
                progress.on_progress(f"sent {index}/{total}", index / total)

        return ExecutionResult(success=True, message=f"sent {total} commands")

    def _send_digital_output(self, command: str) -> None:
        match = re.fullmatch(r"DOExecute\((\d+),\s*([01])\)", command)
        if not match:
            raise ValueError(f"unsupported digital output command: {command}")
        port = int(match.group(1))
        status = bool(int(match.group(2)))
        self.sender.set_digital_output(port, status)
