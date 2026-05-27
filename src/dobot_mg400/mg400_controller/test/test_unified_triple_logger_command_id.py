import csv
import tempfile
import unittest
from pathlib import Path

from mg400_controller.common.utils.loggers.unified_triple_logger import UnifiedTripleLogger


class UnifiedTripleLoggerCommandIdTest(unittest.TestCase):
    def test_dobot_command_id_is_not_carried_into_rows_without_current_id(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = UnifiedTripleLogger(log_dir=tmp_dir)
            logger.log_ros_cmd(
                [0.1, 0.2, 0.3, 0.4],
                ros_timestamp=1.0,
                control_command_seq=1,
                ros_command_uid="ros_cmd_000001",
                dobot_command_id=123,
                joints_are_degrees=False,
            )
            logger.log_ros_cmd(
                [0.2, 0.2, 0.3, 0.4],
                ros_timestamp=2.0,
                control_command_seq=2,
                ros_command_uid="ros_cmd_000002",
                dobot_command_id=None,
                joints_are_degrees=False,
            )
            logger.log_command_result(
                control_command_seq=2,
                ros_command_uid="ros_cmd_000002",
                dobot_command_id=None,
                status="reached_pose_only",
                ros_timestamp=2.2,
                ros_cmd_joints=[0.2, 0.2, 0.3, 0.4],
                robot_joints=[0.2, 0.2, 0.3, 0.4],
            )
            path = Path(logger.file_path)
            logger.close()

            with path.open(newline="") as fh:
                rows = list(csv.DictReader(fh))

        self.assertEqual(rows[0]["dobot_command_id"], "123")
        self.assertEqual(rows[1]["dobot_command_id"], "")
        self.assertEqual(rows[2]["dobot_command_id"], "")

    def test_logs_joint_and_tool_match_events_separately(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = UnifiedTripleLogger(log_dir=tmp_dir)
            logger.log_command_match(
                event_type="joint_match",
                control_command_seq=1,
                ros_command_uid="ros_cmd_000001",
                dobot_command_id=None,
                status="passed_near_4j",
                command_tracking_source="joint4_pass_near",
                command_tracking_confidence="medium",
                settle_match_method="joint4_pass_near",
                settle_match_ambiguous=False,
                settle_candidate_count=1,
                settle_match_error_rad=0.002,
                settle_match_age_ms=120.0,
                pending_command_count=4,
                ros_timestamp=2.0,
                ros_cmd_joints=[0.1, 0.2, 0.3, 0.4],
                robot_joints=[0.101, 0.199, 0.3, 0.4],
                final_error_rad=0.002,
                max_joint_error_rad=0.001,
            )
            logger.log_command_match(
                event_type="tool_match",
                control_command_seq=1,
                ros_command_uid="ros_cmd_000001",
                dobot_command_id=None,
                status="passed_near_xyz",
                command_tracking_source="tool_xyz_pass_near",
                command_tracking_confidence="medium",
                settle_match_method="tool_xyz_pass_near",
                settle_match_ambiguous=False,
                settle_candidate_count=1,
                settle_match_age_ms=125.0,
                pending_command_count=4,
                ros_timestamp=2.1,
                ros_cmd_tool_target=[250.0, -90.0, 100.0, -20.0, 0.0, 0.0],
                robot_tool_actual=[251.0, -91.0, 103.0, -20.2, 0.0, 0.0],
                robot_tool_target=[251.0, -91.0, 103.0, -20.2, 0.0, 0.0],
            )
            path = Path(logger.file_path)
            logger.close()

            with path.open(newline="") as fh:
                rows = list(csv.DictReader(fh))

        self.assertEqual(rows[0]["event_type"], "joint_match")
        self.assertEqual(rows[0]["command_result_status"], "passed_near_4j")
        self.assertEqual(rows[0]["settle_match_method"], "joint4_pass_near")
        self.assertEqual(rows[0]["final_error_rad"], "0.002000")
        self.assertEqual(rows[1]["event_type"], "tool_match")
        self.assertEqual(rows[1]["command_result_status"], "passed_near_xyz")
        self.assertEqual(rows[1]["settle_match_method"], "tool_xyz_pass_near")
        self.assertEqual(rows[1]["error_ros_cmd_tool_to_robot_actual_mm"], "3.316625")

    def test_logs_clock_offset_columns_for_network_delay_audit(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = UnifiedTripleLogger(log_dir=tmp_dir)
            logger.log_unity_target(
                [0.1, 0.2, 0.3, 0.4],
                [0.1, 0.2, 0.3, 0.4],
                unity_raw_timestamp=12.5,
                unity_ros_timestamp=100.0,
                ros_recv_timestamp=100.08,
            )
            path = Path(logger.file_path)
            logger.close()

            with path.open(newline="") as fh:
                rows = list(csv.DictReader(fh))

        self.assertEqual(rows[0]["network_delay_ms"], "80.000")
        self.assertEqual(rows[0]["unity_clock_offset_ms"], "87500.000")
        self.assertEqual(rows[0]["unity_to_ros_raw_offset_ms"], "87580.000")

    def test_active_command_age_is_blank_when_worker_timestamp_predates_latest_command(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = UnifiedTripleLogger(log_dir=tmp_dir)
            logger.log_ros_cmd(
                [0.1, 0.2, 0.3, 0.4],
                ros_timestamp=10.0,
                control_command_seq=1,
                ros_command_uid="ros_cmd_000001",
                joints_are_degrees=False,
            )
            logger.log_robot_feedback(
                [0.1, 0.2, 0.3, 0.4],
                ros_timestamp=9.95,
                joints_are_degrees=False,
            )
            path = Path(logger.file_path)
            logger.close()

            with path.open(newline="") as fh:
                rows = list(csv.DictReader(fh))

        self.assertEqual(rows[1]["active_ros_command_age_ms"], "")


if __name__ == "__main__":
    unittest.main()
