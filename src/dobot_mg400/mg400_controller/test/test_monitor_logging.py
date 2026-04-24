import csv
import tempfile
import unittest
from pathlib import Path

from mg400_controller.common.utils.monitor_logging import (
    ManualMonitorLogger,
    SessionLogger,
)


class MonitorLoggingTest(unittest.TestCase):
    def test_manual_monitor_logger_writes_target_and_actual_rows(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = ManualMonitorLogger(output_dir=tmp_dir)
            logger.start_session()
            logger.log_sample(
                target_xyz=[1.0, 2.0, 3.0],
                actual_xyz=[4.0, 5.0, 6.0],
                target_joints=[10.0, 20.0, 30.0, 40.0],
                actual_joints=[11.0, 21.0, 31.0, 41.0],
                total_diff=9.5,
            )
            target_path = Path(logger.target_path)
            actual_path = Path(logger.actual_path)
            logger.stop_session()

            with target_path.open(newline="") as target_file:
                target_rows = list(csv.reader(target_file))
            with actual_path.open(newline="") as actual_file:
                actual_rows = list(csv.reader(actual_file))

        self.assertEqual(len(target_rows), 2)
        self.assertEqual(len(actual_rows), 2)
        self.assertEqual(target_rows[1][1:4], ["1.000", "2.000", "3.000"])
        self.assertEqual(actual_rows[1][1:4], ["4.000", "5.000", "6.000"])

    def test_session_logger_logs_snapshot_without_thread_start(self):
        snapshot = {
            "raw_unity": [1.0, 2.0, 3.0, 4.0],
            "predicted": [1.5, 2.5, 3.5, 4.5],
            "sent": [2.0, 3.0, 4.0, 5.0],
            "actual": [2.5, 3.5, 4.5, 5.5],
            "tool_target": [10.0, 20.0, 30.0, 0.0, 0.0, 0.0],
            "tool_actual": [11.0, 21.0, 31.0, 0.0, 0.0, 0.0],
            "unity_xyz": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "robot_mode": 7,
            "error_status": 0,
        }

        with tempfile.TemporaryDirectory() as tmp_dir:
            logger = SessionLogger(
                snapshot_fn=lambda: snapshot,
                base_dir=tmp_dir,
                autostart=False,
            )
            logger.log_snapshot(snapshot)
            session_dir = Path(logger.session_dir)
            logger.close()

            with (session_dir / "joints_tracking.csv").open(newline="") as joints_file:
                joints_rows = list(csv.reader(joints_file))
            with (session_dir / "xyz_tracking.csv").open(newline="") as xyz_file:
                xyz_rows = list(csv.reader(xyz_file))

        self.assertEqual(len(joints_rows), 2)
        self.assertEqual(len(xyz_rows), 2)
        self.assertEqual(joints_rows[1][2:6], ["1.0", "2.0", "3.0", "4.0"])
        self.assertEqual(xyz_rows[1][2:5], ["10.0", "20.0", "30.0"])


if __name__ == "__main__":
    unittest.main()
