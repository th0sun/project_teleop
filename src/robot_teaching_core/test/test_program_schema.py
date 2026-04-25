"""Schema-validation tests for canonical program v0.1.

Most important assertion: ``orientation_intent`` is REQUIRED on every
``move`` step. Programs that omit it MUST be rejected. See proposal
§4.3 + R15.
"""

from __future__ import annotations

import copy
import unittest

from teaching_core.program.io import (
    program_from_dict,
    program_to_dict,
)
from teaching_core.program.schema import (
    SchemaValidationError,
    SCHEMA_VERSION,
    validate_program_dict,
)

try:
    from ._fixtures import make_minimal_program
except ImportError:  # support direct unittest discovery without -t
    from _fixtures import make_minimal_program  # type: ignore[import-not-found]


class TestProgramSchema(unittest.TestCase):

    def setUp(self) -> None:
        self.dict_form = program_to_dict(make_minimal_program())

    def test_minimal_program_validates(self) -> None:
        validate_program_dict(self.dict_form)  # must not raise

    def test_schema_version_locked(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        bad["schema_version"] = "9.9"
        with self.assertRaises(SchemaValidationError):
            validate_program_dict(bad)
        self.assertEqual(SCHEMA_VERSION, "0.1")

    def test_orientation_intent_required_on_move(self) -> None:
        """Removing orientation_intent from any move step is a hard error.

        This is the single most important contract from R15.
        """
        bad = copy.deepcopy(self.dict_form)
        # First move step in fixture is index 0.
        first_move_idx = next(
            i for i, s in enumerate(bad["steps"]) if s["kind"] == "move"
        )
        del bad["steps"][first_move_idx]["orientation_intent"]
        with self.assertRaises(SchemaValidationError) as ctx:
            validate_program_dict(bad)
        self.assertIn("orientation_intent", str(ctx.exception))

    def test_pose_frame_required_on_move(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        first_move_idx = next(
            i for i, s in enumerate(bad["steps"]) if s["kind"] == "move"
        )
        del bad["steps"][first_move_idx]["pose_frame"]
        with self.assertRaises(SchemaValidationError) as ctx:
            validate_program_dict(bad)
        self.assertIn("pose_frame", str(ctx.exception))

    def test_unknown_step_kind_rejected(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        bad["steps"].append({"kind": "fly_to_moon"})
        with self.assertRaises(SchemaValidationError) as ctx:
            validate_program_dict(bad)
        self.assertIn("fly_to_moon", str(ctx.exception))

    def test_invalid_orientation_intent_rejected(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        first_move_idx = next(
            i for i, s in enumerate(bad["steps"]) if s["kind"] == "move"
        )
        bad["steps"][first_move_idx]["orientation_intent"] = "kind_of_exact"
        with self.assertRaises(SchemaValidationError):
            validate_program_dict(bad)

    def test_vr_capture_requires_calibration(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        del bad["calibration"]
        bad["source"]["demonstrator"] = "hand_vr"
        with self.assertRaises(SchemaValidationError) as ctx:
            validate_program_dict(bad)
        self.assertIn("calibration", str(ctx.exception))

    def test_hand_authored_program_may_omit_calibration(self) -> None:
        manual = copy.deepcopy(self.dict_form)
        del manual["calibration"]
        manual["source"]["demonstrator"] = "hand_authored"
        validate_program_dict(manual)

    def test_calibration_transform_must_have_positive_scale(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        bad["calibration"]["source_to_task_transform"]["uniform_scale"] = 0.0
        with self.assertRaises(SchemaValidationError):
            validate_program_dict(bad)

    def test_orientation_tolerance_must_have_three_values(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        first_move_idx = next(
            i for i, s in enumerate(bad["steps"]) if s["kind"] == "move"
        )
        bad["steps"][first_move_idx]["orientation_tolerance_rad"] = [0.1, 0.1]
        with self.assertRaises(SchemaValidationError):
            validate_program_dict(bad)

    def test_pose_must_be_3_position_4_quat(self) -> None:
        bad = copy.deepcopy(self.dict_form)
        first_move_idx = next(
            i for i, s in enumerate(bad["steps"]) if s["kind"] == "move"
        )
        bad["steps"][first_move_idx]["pose"]["position_m"] = [0.0, 0.0]
        with self.assertRaises(SchemaValidationError):
            validate_program_dict(bad)


class TestProgramRoundTrip(unittest.TestCase):

    def test_dataclass_round_trip(self) -> None:
        program = make_minimal_program()
        d = program_to_dict(program)
        validate_program_dict(d)
        program2 = program_from_dict(d)
        self.assertEqual(program, program2)

    def test_round_trip_preserves_orientation_intent(self) -> None:
        program = make_minimal_program()
        d = program_to_dict(program)
        program2 = program_from_dict(d)
        for s1, s2 in zip(program.steps, program2.steps):
            if hasattr(s1, "orientation_intent"):
                self.assertEqual(
                    s1.orientation_intent, s2.orientation_intent  # type: ignore[attr-defined]
                )

    def test_default_tolerance_resolves_per_intent(self) -> None:
        program = make_minimal_program()
        for step in program.steps:
            if hasattr(step, "orientation_intent"):
                tol = step.effective_tolerance_rad()  # type: ignore[attr-defined]
                self.assertEqual(len(tol), 3)
                self.assertTrue(all(t >= 0.0 for t in tol))


class TestFileIO(unittest.TestCase):

    def test_load_dump_round_trip(self) -> None:
        from pathlib import Path
        import tempfile
        from teaching_core.program.io import dump_program, load_program

        program = make_minimal_program()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "demo.program.json"
            dump_program(program, path)
            self.assertTrue(path.exists())
            loaded = load_program(path)
            self.assertEqual(program, loaded)


if __name__ == "__main__":
    unittest.main()
