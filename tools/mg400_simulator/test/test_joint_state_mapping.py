import math
import unittest

from core.unity_tcp_bridge import (
    active_joint_positions_deg,
    cdr_joint_state,
    parse_cdr_joint_state_deg,
)


class JointStateMappingTest(unittest.TestCase):
    def test_extracts_active_joints_from_full_mg400_urdf_joint_state(self):
        names = [
            "mg400_j1",
            "mg400_j2_1",
            "mg400_j2_2",
            "mg400_j3",
            "mg400_j3_1",
            "mg400_j3_2",
            "mg400_j4_1",
            "mg400_j4_2",
            "mg400_j5",
        ]
        positions_deg = [10.0, 20.0, 20.0, 30.0, -20.0, -20.0, -30.0, 30.0, 40.0]
        positions_rad = [math.radians(v) for v in positions_deg]

        self.assertSequenceAlmostEqual(
            active_joint_positions_deg(names, positions_rad),
            [10.0, 20.0, 30.0, 40.0],
        )

    def test_cdr_parser_does_not_take_first_four_mimic_joints(self):
        names = [
            "mg400_j1",
            "mg400_j2_1",
            "mg400_j2_2",
            "mg400_j3",
            "mg400_j3_1",
            "mg400_j3_2",
            "mg400_j4_1",
            "mg400_j4_2",
            "mg400_j5",
        ]
        positions_rad = [math.radians(v) for v in [1.0, 2.0, 2.0, 3.0, -2.0, -2.0, -3.0, 3.0, 4.0]]

        parsed = parse_cdr_joint_state_deg(cdr_joint_state(names, positions_rad))

        self.assertSequenceAlmostEqual(parsed, [1.0, 2.0, 3.0, 4.0])

    def test_unity_active_joint_contract_still_maps_first_four(self):
        names = ["joint1", "joint2", "joint3", "joint4"]
        positions_rad = [math.radians(v) for v in [5.0, -6.0, 7.0, -8.0]]

        self.assertSequenceAlmostEqual(
            active_joint_positions_deg(names, positions_rad),
            [5.0, -6.000000000000001, 7.0, -8.0],
        )

    def assertSequenceAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=9)


if __name__ == "__main__":
    unittest.main()
