"""RobotAdapter Protocol and execution-mode contracts.

This module is intentionally robot-neutral. Adapters keep their native
plan payloads opaque to the core while still sharing mode/support/error
semantics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional, Protocol, Tuple, runtime_checkable

from teaching_core.capability.profile import RobotCapabilityProfile
from teaching_core.program.types import CanonicalProgram


class ExecutionMode(str, Enum):
    OFFLINE_EXPORT = "offline_export"
    SUPERVISED_PLAYBACK = "supervised_playback"
    QUEUED = "queued"
    TRAJECTORY_ACTION = "trajectory_action"
    STREAMING = "streaming"


class AdapterError(RuntimeError):
    """Base class for robot-adapter failures."""


class UnsupportedStep(AdapterError):
    """Raised when a program step exceeds the robot profile's capability."""


@dataclass(frozen=True)
class AdaptedPlan:
    """Opaque adapter plan with minimal shared metadata."""

    robot_id: str
    mode: ExecutionMode
    payload: Any
    diagnostics: Tuple[str, ...] = ()


@dataclass(frozen=True)
class ExecutionResult:
    """Result returned by adapter execution."""

    success: bool
    message: str = ""
    diagnostics: Tuple[str, ...] = ()


@runtime_checkable
class CancelToken(Protocol):
    @property
    def cancelled(self) -> bool:
        """True when execution should stop as soon as safely possible."""
        ...


@runtime_checkable
class ProgressSink(Protocol):
    def on_progress(self, message: str, fraction: Optional[float] = None) -> None:
        """Receive adapter progress diagnostics."""
        ...


@runtime_checkable
class RobotAdapter(Protocol):
    profile: RobotCapabilityProfile

    def plan(self, program: CanonicalProgram, mode: ExecutionMode) -> AdaptedPlan:
        """Translate + validate. Raise UnsupportedStep on capability mismatch."""
        ...

    def execute(
        self,
        plan: AdaptedPlan,
        *,
        cancel: CancelToken,
        progress: Optional[ProgressSink] = None,
    ) -> ExecutionResult:
        """Run an adapted plan. Implementations must observe cancellation."""
        ...


def profile_supports_execution_mode(
    profile: RobotCapabilityProfile,
    mode: ExecutionMode,
) -> bool:
    """Return whether a profile declares support for an execution mode."""
    support = profile.execution
    return {
        ExecutionMode.OFFLINE_EXPORT: support.offline_program,
        ExecutionMode.SUPERVISED_PLAYBACK: support.supervised_playback,
        ExecutionMode.QUEUED: support.queued,
        ExecutionMode.TRAJECTORY_ACTION: support.trajectory_action,
        ExecutionMode.STREAMING: support.streaming,
    }[mode]
