import unittest
from pathlib import Path

import numpy as np

from mg400_protocol.alarms import AlarmCatalog
from mg400_protocol.commands import do_execute, joint_mov_j, mov_j, mov_l
from mg400_protocol.feedback import (
    FEEDBACK_PACKET_SIZE,
    FEEDBACK_TEST_VALUE,
    MG400_FEEDBACK_DTYPE,
    parse_feedback_packet,
)

REPO_ROOT = Path(__file__).resolve().parents[4]


class MG400ProtocolTest(unittest.TestCase):
    def test_command_builders_render_vendor_ascii_format(self):
        motion = joint_mov_j(
            (0.0, 5.72957795, -5.72957795, 0.0),
            speed_j=40,
            acc_j=100,
            cp=100,
        )
        digital_output = do_execute(16, True)

        self.assertEqual(
            motion.render(),
            "JointMovJ(0.0000,5.7296,-5.7296,0.0000,SpeedJ=40,AccJ=100,CP=100)",
        )
        self.assertEqual(digital_output.render(), "DOExecute(16,1)")

    def test_legacy_motion_builders_match_existing_runtime_format(self):
        self.assertEqual(
            mov_j((1.0, 2.0, 3.0, 4.0), speed_j=50, acc_j=100, cp=100).render(),
            "MovJ(1.0000,2.0000,3.0000,4.0000,SpeedJ=50,AccJ=100,CP=100)",
        )
        self.assertEqual(
            mov_l((1.0, 2.0, 3.0, 4.0), speed_j=50, acc_j=100, cp=100).render(),
            "MovL(1.0000,2.0000,3.0000,4.0000,SpeedJ=50,AccJ=100,CP=100)",
        )

    def test_feedback_layout_matches_1440_byte_vendor_packet(self):
        self.assertEqual(MG400_FEEDBACK_DTYPE.itemsize, FEEDBACK_PACKET_SIZE)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["QTarget"][1], 192)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["QActual"][1], 432)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["ToolVectorActual"][1], 624)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["ToolVectorTarget"][1], 768)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["RunQueuedCmd"][1], 1014)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["ErrorStatus"][1], 1029)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["CurrentCommandId"][1], 1112)

    def test_parse_feedback_packet_returns_named_snapshot(self):
        record = np.zeros(1, dtype=MG400_FEEDBACK_DTYPE)
        record["TestValue"][0] = FEEDBACK_TEST_VALUE
        record["QActual"][0][:4] = [1.0, 2.0, 3.0, 4.0]
        record["QTarget"][0][:4] = [5.0, 6.0, 7.0, 8.0]
        record["RobotMode"][0] = 5
        record["RunQueuedCmd"][0] = 1
        record["CurrentCommandId"][0] = 123
        record["ToolVectorActual"][0] = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]

        snapshot = parse_feedback_packet(record.tobytes())

        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot.q_actual_deg, (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(snapshot.q_target_deg, (5.0, 6.0, 7.0, 8.0))
        self.assertEqual(snapshot.robot_mode, 5)
        self.assertEqual(snapshot.run_queued_cmd, 1)
        self.assertEqual(snapshot.current_command_id, 123)
        self.assertEqual(snapshot.tool_vector_actual[0], 10.0)

    def test_alarm_catalog_loads_vendor_alarm_json(self):
        catalog = AlarmCatalog.from_files(
            REPO_ROOT / "Dobot_TCP_IP_Python_V4/files/alarmController.json",
            REPO_ROOT / "Dobot_TCP_IP_Python_V4/files/alarmServo.json",
        )

        shoulder_singularity = catalog.lookup(16)
        servo_current = catalog.lookup(8752)

        self.assertGreater(len(catalog), 300)
        self.assertEqual(shoulder_singularity.source, "controller")
        self.assertIn("singularity", shoulder_singularity.description.lower())
        self.assertEqual(servo_current.source, "servo")
        self.assertIn("overcurrent", servo_current.description.lower())


if __name__ == "__main__":
    unittest.main()
