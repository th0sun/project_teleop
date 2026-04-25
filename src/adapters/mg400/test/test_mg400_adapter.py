import unittest

import numpy as np

from mg400_adapter import MG400Adapter, register_mg400_provider
from mg400_adapter.profile import make_mg400_profile
from mg400_adapter.translator import translate_program
from teaching_core.adapter_api import ExecutionMode, UnsupportedStep
from teaching_core.capability.profile import validate_profile
from teaching_core.kinematics.provider import NotSupported
from teaching_core.kinematics.registry import (
    clear_registry,
    get_provider,
    list_providers,
)
from teaching_core.lifter.segmenter import LifterConfig, lift_session
from teaching_core.lifter.session import SessionStream
from teaching_core.program.types import OrientationIntent as OI
from teaching_core.program.types import (
    CanonicalProgram,
    Defaults,
    Frames,
    Motion,
    MoveStep,
    OrientationIntent,
    Pose,
    Source,
    ToolStep,
    WaitStep,
)


class _Cancel:
    cancelled = False


class _Progress:
    def __init__(self):
        self.events = []

    def on_progress(self, message, fraction=None):
        self.events.append((message, fraction))


def _pose():
    return Pose(
        position_m=(0.20, 0.0, 0.10),
        orientation_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _program(*steps):
    return CanonicalProgram(
        program_id="adapter-test",
        source=Source(capture_id="unit", captured_at="2026-04-25T00:00:00Z"),
        frames=Frames(),
        defaults=Defaults(speed_pct=50.0),
        steps=tuple(steps),
    )


class MG400AdapterTest(unittest.TestCase):
    def test_profile_validates_and_registers_fk_provider(self):
        clear_registry()
        provider = register_mg400_provider()
        profile = make_mg400_profile()

        validate_profile(profile)
        self.assertEqual(provider.provider_id, profile.kinematics.provider_id)
        self.assertIn("mg400_4axis_fk", list_providers())
        self.assertIs(get_provider("mg400_4axis_fk"), provider)

    def test_fk_provider_returns_pose_in_meters_and_rejects_ik(self):
        provider = register_mg400_provider()
        position_m, quat = provider.fk((0.0, 0.0, 0.0, 0.0))

        self.assertEqual(len(position_m), 3)
        self.assertEqual(len(quat), 4)
        self.assertTrue(np.isclose(quat[3], 1.0))
        self.assertGreater(position_m[0], 0.0)
        with self.assertRaises(NotSupported):
            provider.solve_ik((position_m, quat))

    def test_translates_joint_hint_move_to_existing_motion_planner_command(self):
        step = MoveStep(
            motion=Motion.JOINT,
            pose_frame="world",
            pose=_pose(),
            orientation_intent=OrientationIntent.YAW_ONLY,
            joint_hint_rad=(0.0, 0.1, -0.1, 0.0),
            speed_pct=40.0,
        )

        plan = translate_program(_program(step))

        self.assertEqual(len(plan.commands), 1)
        self.assertEqual(plan.commands[0].kind, "motion")
        self.assertIn("JointMovJ(0.0000,5.7296,-5.7296,0.0000", plan.commands[0].command)
        self.assertIn("SpeedJ=40", plan.commands[0].command)

    def test_rejects_cartesian_move_until_ik_translator_exists(self):
        step = MoveStep(
            motion=Motion.LINEAR,
            pose_frame="world",
            pose=_pose(),
            orientation_intent=OrientationIntent.YAW_ONLY,
            joint_hint_rad=(0.0, 0.1, -0.1, 0.0),
        )

        with self.assertRaises(UnsupportedStep):
            translate_program(_program(step))

    def test_translates_vacuum_tool_and_wait_steps(self):
        plan = translate_program(
            _program(ToolStep(tool="vacuum", action="off"), WaitStep(duration_ms=25))
        )

        self.assertEqual(
            [command.kind for command in plan.commands],
            ["digital_output", "digital_output", "wait", "digital_output", "wait"],
        )
        self.assertEqual(plan.commands[-1].duration_ms, 25)

    def test_adapter_offline_execute_reports_planned_command_count(self):
        adapter = MG400Adapter()
        progress = _Progress()
        step = MoveStep(
            motion=Motion.JOINT,
            pose_frame="world",
            pose=_pose(),
            orientation_intent=OrientationIntent.YAW_ONLY,
            joint_hint_rad=(0.0, 0.1, -0.1, 0.0),
        )

        plan = adapter.plan(_program(step), ExecutionMode.OFFLINE_EXPORT)
        result = adapter.execute(plan, cancel=_Cancel(), progress=progress)

        self.assertTrue(result.success)
        self.assertIn("1 MG400 commands", result.message)
        self.assertEqual(progress.events, [("offline export ready", 1.0)])


    def test_lifter_end_to_end_with_mg400_fk_provider(self):
        """Registry → provider injection → lift_session → translate.

        Validates the full M4 pipeline:
        session stream → canonical program → mg400 command plan.
        No adapter import inside teaching_core; provider injected here.
        """
        clear_registry()
        provider = register_mg400_provider()

        # Four-joint stream: swing J1 from 0→30 deg (in rad), rest static.
        import math
        pairs = [
            (i * 0.05, (math.radians(i * 6), 0.0, 0.0, 0.0))
            for i in range(6)
        ]
        stream = SessionStream.from_pairs(pairs)
        cfg = LifterConfig(
            program_id="mg400_e2e_test",
            capture_id="cap_e2e_001",
            captured_at="2026-04-25T19:00:00Z",
            default_orientation_intent=OI.YAW_ONLY,
        )

        program = lift_session(stream, provider=provider, config=cfg)

        # Every step must have YAW_ONLY intent and positions in meters.
        from teaching_core.program.types import MoveStep
        moves = [s for s in program.steps if isinstance(s, MoveStep)]
        self.assertGreaterEqual(len(moves), 2)
        for m in moves:
            self.assertEqual(m.orientation_intent, OI.YAW_ONLY)
            # FK output must be in meters: MG400 reach ≤ 0.45 m
            x, y, z = m.pose.position_m
            self.assertLess(abs(x), 0.50)
            self.assertLess(abs(y), 0.50)

        # Must round-trip through schema validation.
        from teaching_core.program.io import program_to_dict
        from teaching_core.program.schema import validate_program_dict
        validate_program_dict(program_to_dict(program))

        # Must translate to at least one MG400 command.
        plan = translate_program(program)
        self.assertGreaterEqual(len(plan.commands), 2)


if __name__ == "__main__":
    unittest.main()
