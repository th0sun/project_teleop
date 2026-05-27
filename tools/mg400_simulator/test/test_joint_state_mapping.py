import math
import unittest

from core.unity_tcp_bridge import (
    UnityTcpBridge,
    active_joint_positions_deg,
    build_teleop_sample_payload,
    cdr_joint_state,
    encode_unity_joint_frame_id,
    parse_cdr_joint_state_deg,
    parse_cdr_joint_state_frame_id,
    parse_cdr_string,
    parse_unity_joint_frame_id,
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

    def test_builds_unity_teleop_sample_v1_with_identity_and_joints(self):
        payload = build_teleop_sample_payload(
            [0.1, 0.2, 0.3, 0.4],
            session_id="session-a",
            unity_seq_id=7,
            timestamp=123.456,
        )

        self.assertEqual(payload["protocol_version"], "teleop_sample_v1")
        self.assertEqual(payload["session_id"], "session-a")
        self.assertEqual(payload["unity_seq_id"], 7)
        self.assertEqual(payload["unity_send_ts"], 123.456)
        self.assertEqual(payload["j1_ik_rad"], 0.1)
        self.assertEqual(payload["joints_ik_rad"], [0.1, 0.2, 0.3, 0.4])
        self.assertTrue(payload["is_valid"])

    def test_tcp_bridge_publishes_teleop_sample_before_joint_cmd(self):
        sent = []

        class CapturingBridge(UnityTcpBridge):
            def _send_msg(self, dest, data_bytes):
                sent.append((dest, data_bytes))

        bridge = CapturingBridge()
        bridge.connected = True
        bridge.teleop_session_id = "session-b"
        bridge.publish_joint_cmd([10.0, 20.0, 30.0, 40.0])

        self.assertEqual(sent[0][0], "/unity/teleop_sample")
        self.assertEqual(sent[1][0], "/unity/joint_cmd")
        sample_json = parse_cdr_string(sent[0][1])
        self.assertIn('"session_id":"session-b"', sample_json)
        self.assertIn('"unity_seq_id":1', sample_json)
        self.assertIn('"protocol_version":"teleop_sample_v1"', sample_json)

        frame_id = parse_cdr_joint_state_frame_id(sent[1][1])
        self.assertEqual(
            parse_unity_joint_frame_id(frame_id),
            ("session-b", 1),
        )

    def test_joint_state_frame_id_carries_exact_teleop_identity(self):
        frame_id = encode_unity_joint_frame_id("session with/slash", 12)
        payload = cdr_joint_state(
            ["joint1", "joint2", "joint3", "joint4"],
            [0.1, 0.2, 0.3, 0.4],
            frame_id=frame_id,
            timestamp=123.456,
        )

        parsed_frame_id = parse_cdr_joint_state_frame_id(payload)
        self.assertEqual(parsed_frame_id, frame_id)
        self.assertEqual(
            parse_unity_joint_frame_id(parsed_frame_id),
            ("session with/slash", 12),
        )
        self.assertSequenceAlmostEqual(
            parse_cdr_joint_state_deg(payload),
            [math.degrees(0.1), math.degrees(0.2), math.degrees(0.3), math.degrees(0.4)],
        )

    def assertSequenceAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=9)


if __name__ == "__main__":
    unittest.main()
