import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mg400_controller.common.logic.scene_safety_guard import SceneSafetyGuard


class FakeKinematics:
    def forward_kinematics(self, joints_deg):
        # Use the first three joint values as an xyz fixture-space point so
        # tests stay focused on scene-safety policy, not MG400 geometry.
        x, y, z = [float(v) for v in joints_deg[:3]]
        return np.array([x, y, z, 0.0, 0.0, 0.0])


class FakeCommand:
    def __init__(self, joints_deg):
        self.joints_deg = tuple(joints_deg)


class FakePlan:
    def __init__(self, commands):
        self.queued_commands = tuple(commands)


def _write_model(tmpdir):
    path = Path(tmpdir) / "scene.json"
    path.write_text(json.dumps({
        "schema": "mg400_scene_safety_model.v1",
        "primitives": [
            {
                "object_name": "camera_post",
                "role": "obstacle",
                "collision_policy": "avoid",
                "raw_box": {"min_xyz": [0, 0, 0], "max_xyz": [10, 10, 20]},
                "safety_box": {"min_xyz": [-2, -2, -2], "max_xyz": [12, 12, 22]},
            },
            {
                "object_name": "pick_fixture",
                "role": "support_surface",
                "collision_policy": "contact_allowed",
                "raw_box": {"min_xyz": [20, 20, 5], "max_xyz": [40, 40, 9]},
                "safety_box": {"min_xyz": [18, 18, -5], "max_xyz": [42, 42, 12]},
            },
        ],
    }), encoding="utf-8")
    return str(path)


class SceneSafetyGuardTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.model_path = _write_model(self.tmp.name)

    def _guard(self, enabled=True):
        return SceneSafetyGuard(
            model_path=self.model_path,
            enabled=enabled,
            kinematics=FakeKinematics(),
            warn_distance_mm=20.0,
            deep_contact_mm=3.0,
        )

    def test_disabled_guard_never_blocks(self):
        result = self._guard(enabled=False).check_tcp([5, 5, 5, 0])
        self.assertEqual(result.status, "DISABLED")
        self.assertFalse(result.blocked)

    def test_inside_avoid_box_blocks(self):
        result = self._guard().check_tcp([5, 5, 5, 0])
        self.assertEqual(result.status, "INSIDE")
        self.assertIn("camera_post", result.detail)
        self.assertTrue(result.blocked)

    def test_contact_allowed_surface_is_allowed_until_too_deep(self):
        guard = self._guard()
        contact = guard.check_tcp([30, 30, 4, 0])
        self.assertEqual(contact.status, "CONTACT")
        self.assertFalse(contact.blocked)

        too_deep = guard.check_tcp([30, 30, 1, 0])
        self.assertEqual(too_deep.status, "SURFACE_HIT")
        self.assertTrue(too_deep.blocked)

    def test_plan_reports_only_blocking_commands(self):
        guard = self._guard()
        plan = FakePlan([
            FakeCommand([50, 50, 50, 0]),
            FakeCommand([5, 5, 5, 0]),
            FakeCommand([30, 30, 4, 0]),
        ])
        violations = guard.check_plan(plan)
        self.assertEqual(len(violations), 1)
        self.assertEqual(violations[0].status, "INSIDE")


if __name__ == "__main__":
    unittest.main()
