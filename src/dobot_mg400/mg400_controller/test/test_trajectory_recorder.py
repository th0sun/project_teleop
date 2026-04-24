import unittest
import tempfile

import numpy as np

from mg400_controller.common.trajectory.trajectory_recorder import (
    PREVIEW_START_TOLERANCE_DEG,
    TrajectoryRecorder,
)


class FakeLogger:
    def info(self, msg):
        self.last_info = msg

    def warn(self, msg):
        self.last_warn = msg

    def error(self, msg):
        self.last_error = msg


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def sleep(self, amount):
        self.value += amount


class PositionFeed:
    def __init__(self, positions_deg):
        self.positions_deg = list(positions_deg)
        self.index = 0

    def __call__(self):
        if self.index >= len(self.positions_deg):
            current = self.positions_deg[-1]
        else:
            current = self.positions_deg[self.index]
            self.index += 1
        return np.radians(current)


class TrajectoryRecorderTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_send_go_to_start_uses_start_motion_and_waits_until_arrival(self):
        sent_commands = []
        clock = FakeClock()
        feed = PositionFeed([
            [15.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_commands.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )

        arrived = recorder._send_go_to_start({
            "j1": 0.0,
            "j2": 0.0,
            "j3": 0.0,
            "j4": 0.0,
        })

        self.assertTrue(arrived)
        self.assertEqual(len(sent_commands), 1)
        self.assertIn("SpeedJ=20", sent_commands[0])
        self.assertIn("AccJ=50", sent_commands[0])
        self.assertIn("CP=0", sent_commands[0])

    def test_wait_until_near_target_times_out_when_position_never_arrives(self):
        clock = FakeClock()
        feed = PositionFeed([
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )

        arrived = recorder._wait_until_near_target(
            [0.0, 0.0, 0.0, 0.0],
            tolerance_deg=PREVIEW_START_TOLERANCE_DEG,
            timeout_sec=0.12,
        )

        self.assertFalse(arrived)

    def test_playback_complete_waits_for_robot_to_reach_final_target(self):
        clock = FakeClock()
        target_q = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
        ])
        feed = PositionFeed([
            [25.0, 10.0, 0.0, 0.0],
            [20.5, 10.2, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )

        self.assertFalse(recorder._playback_complete(2, 1.0, 1.0, target_q))
        self.assertTrue(recorder._playback_complete(2, 1.1, 1.0, target_q))

    def test_playback_complete_uses_extra_timeout_without_feedback(self):
        clock = FakeClock()
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=None,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        target_q = np.array([[0.0, 0.0, 0.0, 0.0]])

        self.assertFalse(recorder._playback_complete(1, 1.5, 1.0, target_q))
        self.assertTrue(recorder._playback_complete(1, 3.1, 1.0, target_q))


if __name__ == "__main__":
    unittest.main()
