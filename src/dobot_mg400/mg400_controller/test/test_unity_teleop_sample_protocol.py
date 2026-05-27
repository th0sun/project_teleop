import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from mg400_controller.common.ros.unity_teleop_sample import (
    ROS_JOINT_CMD_RX_SOURCE,
    UNITY_SAMPLE_PROTOCOL_FIELDS,
    UNITY_TELEOP_PROTOCOL_VERSION,
    UnityTeleopSampleError,
    build_ros_joint_cmd_rx_sample,
    encode_unity_joint_frame_id,
    parse_unity_joint_frame_id,
    parse_unity_teleop_sample,
)
from mg400_controller.common.utils.unified_triple_logger import UnifiedTripleLogger


class UnityTeleopSampleProtocolTest(unittest.TestCase):
    def test_parse_protocol_sample_and_log_fields(self):
        payload = {
            "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
            "session_id": "run_001",
            "unity_seq_id": 42,
            "source": "unity",
            "controller_capture_ts": 10.0,
            "unity_send_ts": 10.3,
            "controller_id": "right",
            "ctrl_pos_x_m": 0.1,
            "ctrl_pos_y_m": -0.2,
            "ctrl_pos_z_m": 0.3,
            "j1_ik_deg": 10.0,
            "j2_ik_deg": 20.0,
            "j3_ik_deg": 30.0,
            "j4_ik_deg": 40.0,
            "unity_filter_status": "passed",
        }

        sample = parse_unity_teleop_sample(json.dumps(payload))

        self.assertEqual(sample.protocol_version, UNITY_TELEOP_PROTOCOL_VERSION)
        self.assertEqual(sample.session_id, "run_001")
        self.assertEqual(sample.unity_seq_id, 42)
        self.assertEqual(sample.ik_joints_deg(), [10.0, 20.0, 30.0, 40.0])
        for actual, expected in zip(sample.ik_joints_rad(), [10.0, 20.0, 30.0, 40.0]):
            self.assertAlmostEqual(actual, math.radians(expected))
        self.assertEqual(sample.unity_send_ts(), 10.3)
        fields = sample.to_log_fields()
        self.assertEqual(fields["unity_seq_id"], 42)
        self.assertEqual(fields["source"], "unity")
        self.assertEqual(fields["controller_id"], "right")
        self.assertEqual(fields["unity_filter_status"], "passed")
        self.assertEqual(fields["j1_ik_deg"], 10.0)
        self.assertAlmostEqual(fields["j1_ik_rad"], math.radians(10.0))
        protocol_fields = sample.to_protocol_fields()
        self.assertEqual(protocol_fields["source"], "unity")
        self.assertEqual(protocol_fields["j1_ik_deg"], 10.0)
        self.assertNotIn("j1_ik_rad", protocol_fields)
        self.assertNotIn("unity_sample_match_method", protocol_fields)

    def test_accepts_seq_id_alias_for_friendlier_unity_payloads(self):
        sample = parse_unity_teleop_sample(json.dumps({
            "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
            "session_id": "run_002",
            "seq_id": 7,
        }))

        self.assertEqual(sample.unity_seq_id, 7)
        self.assertEqual(sample.to_log_fields()["unity_seq_id"], 7)

    def test_builds_ros_assigned_joint_cmd_receive_identity(self):
        sample = build_ros_joint_cmd_rx_sample(
            [math.radians(10.0), math.radians(20.0), math.radians(30.0), math.radians(40.0)],
            session_id="rx-session",
            rx_seq_id=12,
            unity_send_ts=100.0,
            ros_recv_ts=100.025,
        )
        fields = sample.to_log_fields()

        self.assertEqual(sample.protocol_version, UNITY_TELEOP_PROTOCOL_VERSION)
        self.assertEqual(sample.session_id, "rx-session")
        self.assertEqual(sample.unity_seq_id, 12)
        self.assertEqual(fields["source"], ROS_JOINT_CMD_RX_SOURCE)
        self.assertEqual(fields["unity_seq_id"], 12)
        self.assertEqual(fields["j1_ik_deg"], 10.0)
        self.assertAlmostEqual(fields["j4_ik_rad"], math.radians(40.0))
        self.assertEqual(fields["unity_sample_match_method"], "ros_assigned_joint_cmd_rx")
        self.assertAlmostEqual(fields["unity_sample_age_ms"], 25.0)

    def test_accepts_nested_unity_payload_shape(self):
        payload = {
            "meta": {
                "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
                "session_id": "run_20260525_123000",
                "unity_seq_id": 1234,
                "source": "unity",
            },
            "timestamps": {
                "controller_capture_ts": 20.0,
                "unity_target_ts": 20.1,
                "unity_ik_ts": 20.2,
                "unity_send_ts": 20.3,
            },
            "controller_raw": {
                "controller_id": "right",
                "pos": {"x": 0.11, "y": -0.22, "z": 0.33},
                "rot": {"x": 0.0, "y": 0.1, "z": 0.2, "w": 0.97},
                "trigger": 0.8,
                "grip": 0.6,
                "primary_button": True,
                "secondary_button": False,
            },
            "targets": {
                "raw": {"x": 101.0, "y": 202.0, "z": 303.0, "r_deg": 10.0},
                "filtered": {"x": 111.0, "y": 222.0, "z": 333.0, "r_deg": 11.0},
                "filter_status": "passed",
                "filter_detail": "ok",
            },
            "ik_result": {
                "joints_ik_deg": [10.0, 20.0, 30.0, 40.0],
            },
            "flags": {
                "control_mode": "joint",
                "is_valid": True,
                "invalid_reason": "",
            },
        }

        sample = parse_unity_teleop_sample(json.dumps(payload))
        fields = sample.to_log_fields()

        self.assertEqual(sample.session_id, "run_20260525_123000")
        self.assertEqual(sample.unity_seq_id, 1234)
        self.assertEqual(fields["source"], "unity")
        self.assertEqual(sample.unity_send_ts(), 20.3)
        self.assertEqual(sample.ik_joints_deg(), [10.0, 20.0, 30.0, 40.0])
        for actual, expected in zip(sample.ik_joints_rad(), [10.0, 20.0, 30.0, 40.0]):
            self.assertAlmostEqual(actual, math.radians(expected))
        self.assertEqual(fields["ctrl_pos_x_m"], 0.11)
        self.assertEqual(fields["ctrl_rot_w"], 0.97)
        self.assertEqual(fields["trigger_value"], 0.8)
        self.assertEqual(fields["target_raw_x_mm"], 101.0)
        self.assertEqual(fields["target_filt_r_deg"], 11.0)
        self.assertEqual(fields["unity_filter_status"], "passed")
        self.assertTrue(fields["is_valid"])

    def test_accepts_legacy_rad_payload_and_derives_latest_deg_fields(self):
        sample = parse_unity_teleop_sample(json.dumps({
            "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
            "session_id": "run_legacy_rad",
            "unity_seq_id": 8,
            "j1_ik_rad": 0.1,
            "j2_ik_rad": 0.2,
            "j3_ik_rad": 0.3,
            "j4_ik_rad": 0.4,
        }))

        for actual, expected in zip(sample.ik_joints_rad(), [0.1, 0.2, 0.3, 0.4]):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(sample.to_protocol_fields()["j1_ik_deg"], math.degrees(0.1))

    def test_rejects_payload_without_command_identity(self):
        with self.assertRaises(UnityTeleopSampleError):
            parse_unity_teleop_sample(json.dumps({
                "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
                "session_id": "run_003",
            }))

    def test_joint_frame_id_encodes_same_identity_as_sample_protocol(self):
        frame_id = encode_unity_joint_frame_id("run/004 with spaces", 123)

        self.assertTrue(frame_id.startswith(f"{UNITY_TELEOP_PROTOCOL_VERSION};"))
        self.assertEqual(
            parse_unity_joint_frame_id(frame_id),
            ("run/004 with spaces", 123),
        )
        self.assertIsNone(parse_unity_joint_frame_id("legacy_sim"))
        self.assertIsNone(parse_unity_joint_frame_id(""))

    def test_unified_logger_writes_protocol_identity_columns(self):
        sample = parse_unity_teleop_sample(json.dumps({
            "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
            "session_id": "run_004",
            "unity_seq_id": 99,
            "unity_send_ts": 12.5,
            "j1_ik_deg": 10.0,
            "j2_ik_deg": 20.0,
            "j3_ik_deg": 30.0,
            "j4_ik_deg": 40.0,
        }))
        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = UnifiedTripleLogger(log_dir=tmp_dir)
            logger.log_unity_sample(sample, ros_recv_timestamp=13.0)
            logger.log_ros_cmd(
                [math.radians(10.0), math.radians(20.0), math.radians(30.0), math.radians(40.0)],
                ros_timestamp=13.1,
                control_command_seq=5,
                ros_command_uid="ros_cmd_000005",
                dobot_command_id=123,
                dobot_command_response="0,{123},JointMovJ(...);",
                dobot_command_text="JointMovJ(1,2,3,4,SpeedJ=20)",
                command_tracking_source="dobot_ack_id",
                command_tracking_confidence="medium",
                feedback_command_id_before_send=122,
                unity_sample=sample,
                joints_are_degrees=False,
            )
            logger.log_command_result(
                control_command_seq=5,
                ros_command_uid="ros_cmd_000005",
                dobot_command_id=123,
                robot_feedback_command_id=123,
                status="reached",
                command_id_match=True,
                dobot_command_text="JointMovJ(1,2,3,4,SpeedJ=20)",
                command_tracking_source="dobot_ack_feedback_id_pose",
                command_tracking_confidence="high",
                feedback_command_id_before_send=122,
                feedback_command_id_at_result=123,
                feedback_command_id_changed=True,
                settle_match_method="dobot_id_and_pose_settle",
                settle_match_ambiguous=False,
                settle_candidate_count=1,
                settle_match_error_rad=0.0,
                settle_second_best_error_rad=None,
                settle_match_age_ms=100.0,
                pending_command_count=1,
                ros_timestamp=13.2,
                ros_cmd_joints=[math.radians(10.0), math.radians(20.0), math.radians(30.0), math.radians(40.0)],
                robot_joints=[math.radians(10.0), math.radians(20.0), math.radians(30.0), math.radians(40.0)],
                unity_sample=sample,
            )
            path = Path(logger.file_path)
            unity_sample_path = Path(logger.unity_sample_file_path)
            logger.close()

            with path.open(newline="") as fh:
                rows = list(csv.DictReader(fh))
            with unity_sample_path.open(newline="") as fh:
                unity_rows = list(csv.DictReader(fh))

        sample_row = rows[0]
        ros_row = rows[1]
        result_row = rows[2]
        self.assertEqual(sample_row["unity_seq_id"], "99")
        self.assertEqual(sample_row["session_id"], "run_004")
        self.assertEqual(sample_row["j1_ik_deg"], "10.000000")
        self.assertAlmostEqual(float(sample_row["j1_ik_rad"]), math.radians(10.0), places=6)
        self.assertEqual(ros_row["unity_seq_id"], "99")
        self.assertEqual(ros_row["ros_command_uid"], "ros_cmd_000005")
        self.assertEqual(ros_row["dobot_command_id"], "123")
        self.assertEqual(ros_row["dobot_command_response"], "0,{123},JointMovJ(...);")
        self.assertEqual(ros_row["dobot_command_text"], "JointMovJ(1,2,3,4,SpeedJ=20)")
        self.assertEqual(ros_row["command_tracking_source"], "dobot_ack_id")
        self.assertEqual(ros_row["command_tracking_confidence"], "medium")
        self.assertEqual(ros_row["feedback_command_id_before_send"], "122")
        self.assertEqual(result_row["event_type"], "command_result")
        self.assertEqual(result_row["command_result_status"], "reached")
        self.assertEqual(result_row["command_id_match"], "true")
        self.assertEqual(result_row["feedback_command_id_at_result"], "123")
        self.assertEqual(result_row["feedback_command_id_changed"], "true")
        self.assertEqual(result_row["settle_match_method"], "dobot_id_and_pose_settle")
        self.assertEqual(result_row["settle_match_ambiguous"], "false")
        self.assertEqual(result_row["settle_candidate_count"], "1")
        self.assertEqual(result_row["settle_match_error_rad"], "0.000000")
        self.assertEqual(result_row["settle_match_age_ms"], "100.000")
        self.assertEqual(result_row["pending_command_count"], "1")
        self.assertEqual(len(unity_rows), 1)
        self.assertEqual(list(unity_rows[0].keys()), list(UNITY_SAMPLE_PROTOCOL_FIELDS))
        self.assertEqual(unity_rows[0]["unity_seq_id"], "99")
        self.assertEqual(unity_rows[0]["session_id"], "run_004")
        self.assertEqual(unity_rows[0]["j1_ik_deg"], "10.000000")
        self.assertNotIn("event_type", unity_rows[0])
        self.assertNotIn("j1_ik_rad", unity_rows[0])
        self.assertNotIn("unity_sample_match_method", unity_rows[0])


if __name__ == "__main__":
    unittest.main()
