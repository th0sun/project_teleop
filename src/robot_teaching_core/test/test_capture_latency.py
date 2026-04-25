import unittest

import numpy as np

from teaching_core.capture.latency import TargetLatencyCompensator


class DummyValidator:
    def __init__(self):
        self.calls = []

    def validate_and_clamp(self, q_target):
        self.calls.append(np.asarray(q_target))
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

    def test_joint_count_is_configurable_for_non_mg400_adapters(self):
        compensator = TargetLatencyCompensator(DummyValidator(), joint_count=6)

        q0 = np.zeros(7)
        q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 99.0])

        compensator.compensate(q0, corrected_target_time=1.0, receive_time=1.0)
        out = compensator.compensate(q1, corrected_target_time=1.1, receive_time=1.18)

        self.assertAlmostEqual(out[5], 0.248)
        self.assertEqual(out[6], 99.0)

    def test_short_targets_are_delegated_to_validator_without_broadcast_error(self):
        validator = DummyValidator()
        compensator = TargetLatencyCompensator(validator, joint_count=6)

        out = compensator.compensate(
            np.array([0.0, 0.0]),
            corrected_target_time=1.0,
            receive_time=1.1,
        )

        self.assertEqual(len(out), 2)
        self.assertEqual(len(validator.calls), 1)


if __name__ == "__main__":
    unittest.main()
