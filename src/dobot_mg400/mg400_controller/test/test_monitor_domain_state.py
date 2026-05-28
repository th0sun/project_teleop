import unittest

import numpy as np

from mg400_controller.common.utils.monitor.execution_metrics import ExecutionMonitor
from mg400_controller.common.utils.monitor.joint_graph_buffer import JointGraphBuffer


class FakeTelemetry:
    def __init__(self):
        self.latest_target_joints = [1.0, 2.0, 3.0, 4.0]
        self.latest_predicted_joints = [1.5, 2.5, 3.5, 4.5]
        self.latest_sent_joints = [2.0, 3.0, 4.0, 5.0]
        self.latest_actual_joints = [0.5, 1.5, 2.5, 3.5]
        self.sent_fresh = [True, False, True, False]

    def consume_sent_fresh(self, index):
        was_fresh = self.sent_fresh[index]
        self.sent_fresh[index] = False
        return was_fresh


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, amount):
        self.value += amount


class MonitorDomainStateTest(unittest.TestCase):
    def test_execution_monitor_tracks_motion_lifecycle(self):
        clock = FakeClock()
        monitor = ExecutionMonitor(start_threshold=2.0, stop_threshold=0.5, time_fn=clock)

        self.assertEqual(monitor.update(0.0), "IDLE")
        self.assertEqual(monitor.update(3.0), "STARTED")
        clock.advance(0.25)
        self.assertEqual(monitor.update(0.2), "FINISHED")
        avg_t, min_t, max_t = monitor.get_stats()
        self.assertAlmostEqual(avg_t, 0.25)
        self.assertAlmostEqual(min_t, 0.25)
        self.assertAlmostEqual(max_t, 0.25)

    def test_joint_graph_buffer_appends_and_builds_limits(self):
        clock = FakeClock()
        buffer = JointGraphBuffer(window_sec=10.0, max_sample_hz=10, time_fn=clock)
        telemetry = FakeTelemetry()

        clock.advance(0.1)
        rel_t = buffer.append_telemetry(telemetry)
        t_arr, u, p, s, a = buffer.get_joint_arrays(0)

        self.assertAlmostEqual(rel_t, 0.1)
        self.assertTrue(np.allclose(t_arr, [0.1]))
        self.assertTrue(np.allclose(u, [1.0]))
        self.assertTrue(np.allclose(p, [1.5]))
        self.assertTrue(np.allclose(a, [0.5]))
        self.assertTrue(np.allclose(s, [2.0], equal_nan=True))
        x_limits = buffer.get_x_limits(rel_t)
        self.assertEqual(x_limits[0], 0)
        self.assertAlmostEqual(x_limits[1], 0.6)

        limits = buffer.get_y_limits(u, p, s, a)
        self.assertIsNotNone(limits)
        self.assertLess(limits[0], 0.5)
        self.assertGreater(limits[1], 2.0)


if __name__ == "__main__":
    unittest.main()
