import unittest

from teaching_core.trajectory import (
    JointTimingLimits,
    TimedJointPoint,
    retime_joint_path,
    speed_percent_for_segment,
)


class TrajectoryRetimingTest(unittest.TestCase):
    def test_preserves_original_timing_when_within_limits(self):
        result = retime_joint_path(
            [
                TimedJointPoint(10.0, (0.0, 0.0)),
                TimedJointPoint(10.5, (10.0, -5.0)),
                TimedJointPoint(11.0, (20.0, -10.0)),
            ],
            JointTimingLimits(max_velocity=(90.0, 90.0)),
        )

        self.assertTrue(result.is_original_timing_feasible)
        self.assertEqual([round(p.time_s, 3) for p in result.points], [0.0, 0.5, 1.0])
        self.assertEqual(result.points[-1].position, (20.0, -10.0))
        self.assertAlmostEqual(result.time_scale, 1.0)

    def test_stretches_only_segments_that_exceed_robot_velocity(self):
        result = retime_joint_path(
            [
                TimedJointPoint(0.0, (0.0, 0.0)),
                TimedJointPoint(0.05, (20.0, 0.0)),
                TimedJointPoint(0.55, (25.0, 0.0)),
            ],
            JointTimingLimits(max_velocity=(100.0, 100.0)),
        )

        self.assertFalse(result.is_original_timing_feasible)
        self.assertAlmostEqual(result.segments[0].original_dt_s, 0.05)
        self.assertAlmostEqual(result.segments[0].retimed_dt_s, 0.20)
        self.assertAlmostEqual(result.segments[1].retimed_dt_s, 0.50)
        self.assertEqual([round(p.time_s, 3) for p in result.points], [0.0, 0.2, 0.7])
        self.assertAlmostEqual(result.time_scale, 0.7 / 0.55)

    def test_speed_percent_uses_segment_duration_instead_of_fixed_cap(self):
        slow = speed_percent_for_segment(
            (0.0, 0.0),
            (10.0, 0.0),
            1.0,
            (100.0, 100.0),
            min_percent=1,
        )
        fast = speed_percent_for_segment(
            (0.0, 0.0),
            (10.0, 0.0),
            0.2,
            (100.0, 100.0),
            min_percent=1,
        )

        self.assertEqual(slow, 10)
        self.assertEqual(fast, 50)
        self.assertGreater(fast, slow)

    def test_rejects_non_monotonic_timestamps(self):
        with self.assertRaises(ValueError):
            retime_joint_path(
                [
                    TimedJointPoint(0.0, (0.0,)),
                    TimedJointPoint(0.0, (1.0,)),
                ],
                JointTimingLimits(max_velocity=(100.0,)),
            )


if __name__ == "__main__":
    unittest.main()
