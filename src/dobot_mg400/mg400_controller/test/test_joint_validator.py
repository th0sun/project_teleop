import unittest

import numpy as np

from mg400_controller.common.logic.joint_validator import JointValidator


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, message):
        self.errors.append(message)

    def warn(self, message):
        self.warnings.append(message)


class JointValidatorTest(unittest.TestCase):
    def test_clamps_mg400_joint_and_elbow_limits_through_core_helper(self):
        logger = FakeLogger()
        validator = JointValidator(
            joint_limits={
                0: (-90.0, 90.0),
                1: (-45.0, 45.0),
                2: (-10.0, 120.0),
                3: (-180.0, 180.0),
            },
            elbow_limit=(-20.0, 70.0),
            logger=logger,
        )

        q_safe, was_clamped = validator.validate_and_clamp(
            np.radians([100.0, 10.0, 100.0, 0.0])
        )

        self.assertTrue(was_clamped)
        self.assertAlmostEqual(np.degrees(q_safe[0]), 89.9999)
        self.assertAlmostEqual(np.degrees(q_safe[2]), 79.9999)
        self.assertEqual(len(logger.errors), 0)
        self.assertEqual(len(logger.warnings), 1)

    def test_rejects_short_mg400_target_without_clamping(self):
        logger = FakeLogger()
        validator = JointValidator(
            joint_limits={0: (-90.0, 90.0)},
            elbow_limit=(-20.0, 70.0),
            logger=logger,
        )

        q_safe, was_clamped = validator.validate_and_clamp(np.radians([0.0, 0.0]))

        self.assertFalse(was_clamped)
        self.assertEqual(len(q_safe), 2)
        self.assertEqual(len(logger.errors), 1)


if __name__ == "__main__":
    unittest.main()
