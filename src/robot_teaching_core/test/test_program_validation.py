import unittest

import numpy as np

from teaching_core.program.validation import (
    clamp_joint_positions,
    clamp_joint_target,
    clamp_relative_joint_angle,
)


class JointValidationHelpersTest(unittest.TestCase):
    def test_clamps_joint_position_against_degree_limits(self):
        result = clamp_joint_positions(
            np.radians([100.0, 0.0]),
            {0: (-90.0, 90.0)},
        )

        self.assertTrue(result.was_clamped)
        self.assertAlmostEqual(np.degrees(result.q_safe[0]), 89.9999)
        self.assertEqual(result.reasons, ("J1(100.00deg->90.00deg)",))

    def test_clamps_relative_child_joint_angle(self):
        result = clamp_relative_joint_angle(
            np.radians([0.0, 10.0, 100.0]),
            parent_index=1,
            child_index=2,
            relative_limit_deg=(-20.0, 70.0),
            label="Elbow",
        )

        self.assertTrue(result.was_clamped)
        self.assertAlmostEqual(np.degrees(result.q_safe[2]), 79.9999)
        self.assertEqual(result.reasons, ("Elbow(90.00deg->70.00deg)",))

    def test_joint_target_handles_short_and_nan_inputs_without_crashing(self):
        short = clamp_joint_target(
            np.radians([0.0, 0.0]),
            {3: (-180.0, 180.0)},
            expected_min_len=4,
        )
        nan = clamp_joint_target(
            np.array([np.nan, 0.0, 0.0, 0.0]),
            {0: (-90.0, 90.0)},
        )

        self.assertFalse(short.was_clamped)
        self.assertEqual(short.reasons, ("short",))
        self.assertTrue(nan.was_clamped)
        self.assertEqual(nan.reasons, ("NaN",))

    def test_relative_constraint_handles_nan_without_crashing(self):
        result = clamp_relative_joint_angle(
            np.array([0.0, np.nan, 0.0]),
            parent_index=1,
            child_index=2,
            relative_limit_deg=(-20.0, 70.0),
        )

        self.assertTrue(result.was_clamped)
        self.assertEqual(result.reasons, ("NaN",))


if __name__ == "__main__":
    unittest.main()
