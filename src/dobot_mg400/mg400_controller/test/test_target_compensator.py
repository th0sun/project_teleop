import unittest

import numpy as np

from mg400_controller.common.logic.target_compensator import TargetLatencyCompensator


class DummyValidator:
    def validate_and_clamp(self, q_target):
        return np.asarray(q_target), False


class TargetLatencyCompensatorTest(unittest.TestCase):
    def test_compensates_by_filtered_target_velocity_and_latency_cap(self):
        compensator = TargetLatencyCompensator(DummyValidator())

        q0 = np.array([0.0, 0.0, 0.0, 0.0])
        q1 = np.array([0.1, 0.0, 0.0, 0.0])

        first = compensator.compensate(q0, corrected_target_time=10.0, receive_time=10.1)
        second = compensator.compensate(q1, corrected_target_time=10.1, receive_time=10.3)

        self.assertTrue(np.allclose(first, q0))
        self.assertAlmostEqual(second[0], 0.124)

    def test_preserves_extra_joint_fields_after_compensation(self):
        compensator = TargetLatencyCompensator(DummyValidator())

        q0 = np.array([0.0, 0.0, 0.0, 0.0, 9.0])
        q1 = np.array([0.1, 0.0, 0.0, 0.0, 9.0])

        compensator.compensate(q0, corrected_target_time=1.0, receive_time=1.0)
        out = compensator.compensate(q1, corrected_target_time=1.1, receive_time=1.18)

        self.assertEqual(len(out), 5)
        self.assertEqual(out[4], 9.0)

    def test_reset_clears_target_velocity(self):
        compensator = TargetLatencyCompensator(DummyValidator())

        compensator.compensate(np.zeros(4), corrected_target_time=1.0, receive_time=1.0)
        compensator.compensate(
            np.array([0.1, 0.0, 0.0, 0.0]),
            corrected_target_time=1.1,
            receive_time=1.18,
        )
        self.assertGreater(compensator.target_speed, 0.0)

        anchor = np.array([0.2, 0.0, 0.0, 0.0])
        compensator.reset(anchor, target_time=2.0)

        self.assertEqual(compensator.target_speed, 0.0)
        out = compensator.compensate(anchor, corrected_target_time=2.1, receive_time=2.18)
        self.assertTrue(np.allclose(out, anchor))


if __name__ == "__main__":
    unittest.main()
