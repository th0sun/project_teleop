"""RobotAdapter contract tests."""

from __future__ import annotations

import unittest
from dataclasses import dataclass
from typing import Optional

from teaching_core.adapter_api import (
    AdaptedPlan,
    CancelToken,
    ExecutionMode,
    ExecutionResult,
    ProgressSink,
    RobotAdapter,
    UnsupportedStep,
    profile_supports_execution_mode,
)
from teaching_core.calibration.feedback import WorkspaceKind, WorkspaceModel
from teaching_core.capability.kinematics import KinematicsContract, KinematicsKind
from teaching_core.capability.profile import (
    ExecutionSupport,
    MotionSupport,
    OrientationAuthority,
    RobotCapabilityProfile,
    ToolModel,
)

try:
    from ._fixtures import make_minimal_program
except ImportError:  # support direct unittest discovery without -t
    from _fixtures import make_minimal_program  # type: ignore[import-not-found]


@dataclass(frozen=True)
class _Cancel:
    cancelled: bool = False


class _Progress:
    def __init__(self) -> None:
        self.messages = []

    def on_progress(self, message: str, fraction: Optional[float] = None) -> None:
        self.messages.append((message, fraction))


class _TinyAdapter:
    profile = RobotCapabilityProfile(
        robot_id="tiny",
        dof=1,
        joint_names=("J1",),
        joint_limits_rad=((-1.0, 1.0),),
        kinematics=KinematicsContract(kind=KinematicsKind.NAMED, provider_id="tiny_fk"),
        ros2_control_command_interfaces=("position",),
        ros2_control_state_interfaces=("position",),
        base_frame="tiny_base",
        tcp_frame="tiny_tcp",
        workspace=WorkspaceModel(
            kind=WorkspaceKind.BOX,
            frame="tiny_base",
            margin_m=0.01,
            source="fixture",
            payload={"min_m": [-0.1, -0.1, 0.0], "max_m": [0.1, 0.1, 0.2]},
        ),
        motion=MotionSupport(
            joint=True,
            linear=False,
            circular=False,
            spline=False,
            servo_stream=False,
            blending="none",
        ),
        execution=ExecutionSupport(
            offline_program=True,
            queued=False,
            supervised_playback=False,
            trajectory_action=False,
            streaming=False,
        ),
        tool=ToolModel(kind="none"),
        orientation_authority=OrientationAuthority.FULL_6DOF,
        max_tcp_speed_m_s=0.1,
        max_joint_speed_rad_s=(0.5,),
        timing_contract="best_effort",
    )

    def plan(self, program, mode: ExecutionMode) -> AdaptedPlan:
        if not profile_supports_execution_mode(self.profile, mode):
            raise UnsupportedStep(f"{self.profile.robot_id} does not support {mode.value}")
        return AdaptedPlan(robot_id=self.profile.robot_id, mode=mode, payload=program)

    def execute(
        self,
        plan: AdaptedPlan,
        *,
        cancel: CancelToken,
        progress: Optional[ProgressSink] = None,
    ) -> ExecutionResult:
        if cancel.cancelled:
            return ExecutionResult(success=False, message="cancelled")
        if progress is not None:
            progress.on_progress("executed", 1.0)
        return ExecutionResult(success=True, message=f"{plan.mode.value} done")


class TestAdapterApi(unittest.TestCase):

    def test_runtime_protocol_accepts_structural_adapter(self) -> None:
        adapter = _TinyAdapter()
        self.assertIsInstance(adapter, RobotAdapter)

    def test_profile_supports_execution_modes(self) -> None:
        adapter = _TinyAdapter()
        self.assertTrue(
            profile_supports_execution_mode(
                adapter.profile, ExecutionMode.OFFLINE_EXPORT
            )
        )
        self.assertFalse(
            profile_supports_execution_mode(adapter.profile, ExecutionMode.QUEUED)
        )

    def test_adapter_rejects_unsupported_mode(self) -> None:
        adapter = _TinyAdapter()
        with self.assertRaises(UnsupportedStep):
            adapter.plan(make_minimal_program(), ExecutionMode.QUEUED)

    def test_execute_observes_cancel_and_progress_contracts(self) -> None:
        adapter = _TinyAdapter()
        plan = adapter.plan(make_minimal_program(), ExecutionMode.OFFLINE_EXPORT)
        progress = _Progress()
        ok = adapter.execute(plan, cancel=_Cancel(False), progress=progress)
        self.assertTrue(ok.success)
        self.assertEqual(progress.messages, [("executed", 1.0)])

        cancelled = adapter.execute(plan, cancel=_Cancel(True))
        self.assertFalse(cancelled.success)
        self.assertIn("cancelled", cancelled.message)


if __name__ == "__main__":
    unittest.main()
