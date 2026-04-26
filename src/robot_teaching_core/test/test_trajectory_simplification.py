"""Unit tests for joint-space RDP path simplification.

Tests cover the canonical motion shapes that the MG400 sees during
teach-and-repeat playback:

    * straight line in 1 joint        — collapses to endpoints
    * straight line in time-vs-joint  — keeps endpoints, drops collinear
    * sharp corner (L-shape)          — keeps the corner waypoint
    * smooth curve (sin wave)         — keeps proportional to curvature
    * already-sparse path             — no-op
    * zero / negative tolerance       — disabled, returns input unchanged
    * 1- and 2-point inputs           — return verbatim (no recursion)
    * multi-joint corner              — corner detected on the joint that moves

The metric is max-joint deviation from the time-parameterised lerp between
segment endpoints, which mirrors the MG400 controller's blend behaviour
between commanded waypoints.
"""

import math
import unittest

from teaching_core.trajectory import (
    TimedJointPoint,
    simplify_joint_path_rdp,
)


class _SimplificationTestBase(unittest.TestCase):
    @staticmethod
    def _line(start, end, n, j_axis=0):
        """N points evenly spaced along a straight joint-space line in time."""
        out = []
        for i in range(n):
            ratio = i / (n - 1) if n > 1 else 0.0
            t = start[0] + ratio * (end[0] - start[0])
            j = [0.0, 0.0, 0.0, 0.0]
            j[j_axis] = start[1] + ratio * (end[1] - start[1])
            out.append(TimedJointPoint(time_s=t, position=tuple(j)))
        return out

    @staticmethod
    def _times(points):
        return [round(p.time_s, 6) for p in points]


class StraightLineCollapseTest(_SimplificationTestBase):
    def test_dense_straight_line_collapses_to_endpoints(self):
        # 11 evenly spaced waypoints along a straight 0°→10° motion in j1
        points = self._line((0.0, 0.0), (1.0, 10.0), n=11)
        result = simplify_joint_path_rdp(points, tolerance=0.5)
        self.assertEqual(len(result), 2,
                         f"straight line should collapse to endpoints; got {self._times(result)}")
        self.assertEqual(result[0], points[0])
        self.assertEqual(result[-1], points[-1])

    def test_straight_line_with_offset_time_origin_still_collapses(self):
        # Time origin shifted to 100s — RDP must use relative time, not absolute
        points = self._line((100.0, 0.0), (101.0, 10.0), n=21)
        result = simplify_joint_path_rdp(points, tolerance=0.5)
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[0].time_s, 100.0)
        self.assertAlmostEqual(result[-1].time_s, 101.0)

    def test_straight_line_below_tolerance_collapses(self):
        # Tiny noise (0.05°) on the middle of an otherwise perfect line.
        # Default 0.5° tolerance must absorb it.
        points = [
            TimedJointPoint(0.0, (0.0, 0.0, 0.0, 0.0)),
            TimedJointPoint(0.5, (5.05, 0.0, 0.0, 0.0)),  # +0.05° noise
            TimedJointPoint(1.0, (10.0, 0.0, 0.0, 0.0)),
        ]
        result = simplify_joint_path_rdp(points, tolerance=0.5)
        self.assertEqual(len(result), 2)


class CornerPreservationTest(_SimplificationTestBase):
    def test_sharp_l_corner_keeps_endpoints_and_corner(self):
        # 5 points right (j1: 0→5), 1 corner, 5 points up (j2: 0→5)
        points = []
        for i in range(6):
            points.append(TimedJointPoint(
                time_s=i * 0.1,
                position=(float(i), 0.0, 0.0, 0.0),
            ))
        for i in range(1, 6):
            points.append(TimedJointPoint(
                time_s=0.5 + i * 0.1,
                position=(5.0, float(i), 0.0, 0.0),
            ))

        result = simplify_joint_path_rdp(points, tolerance=0.5)

        kept_times = self._times(result)
        # Endpoints + corner at t=0.5 (j1=5, j2=0) at minimum
        self.assertIn(0.0, kept_times)
        self.assertIn(0.5, kept_times)  # corner
        self.assertIn(1.0, kept_times)
        # Aggressive simplification: each leg is straight, so we expect
        # exactly 3 kept points
        self.assertEqual(len(result), 3,
                         f"L-corner should keep exactly 3 points; got {kept_times}")

    def test_corner_in_separate_joint_still_detected(self):
        # j3 stays still while j1 zigzags — corner detection must scan all axes
        points = [
            TimedJointPoint(0.0, (0.0, 0.0, 0.0, 0.0)),
            TimedJointPoint(0.5, (5.0, 0.0, 0.0, 0.0)),  # peak
            TimedJointPoint(1.0, (0.0, 0.0, 0.0, 0.0)),  # back to start in j1
        ]
        result = simplify_joint_path_rdp(points, tolerance=0.5)
        self.assertEqual(len(result), 3)


class CurvePreservationTest(_SimplificationTestBase):
    def test_sine_curve_keeps_points_proportional_to_curvature(self):
        # j1 = 10° * sin(πt) over t∈[0,1] — peak at t=0.5
        points = [
            TimedJointPoint(
                time_s=i / 20.0,
                position=(10.0 * math.sin(math.pi * i / 20.0), 0.0, 0.0, 0.0),
            )
            for i in range(21)
        ]

        coarse = simplify_joint_path_rdp(points, tolerance=2.0)
        fine = simplify_joint_path_rdp(points, tolerance=0.1)

        # Coarse keeps fewer than fine, both keep more than 2 (curve, not line)
        self.assertGreater(len(coarse), 2)
        self.assertGreater(len(fine), len(coarse),
                           "tighter tolerance must keep more points")
        self.assertLess(len(coarse), len(points),
                        "coarse tolerance must drop something")
        # Both keep endpoints
        self.assertEqual(coarse[0], points[0])
        self.assertEqual(coarse[-1], points[-1])

    def test_simplified_curve_stays_within_tolerance_under_lerp(self):
        """Regression: every dropped point must lie within tolerance of the
        time-lerp of its surrounding kept points.  This is the property the
        MG400 controller's linear blend will actually replay."""
        points = [
            TimedJointPoint(
                time_s=i / 50.0,
                position=(15.0 * math.sin(math.pi * i / 50.0),
                          7.0 * math.cos(math.pi * i / 50.0), 0.0, 0.0),
            )
            for i in range(51)
        ]
        tolerance = 0.5
        kept = simplify_joint_path_rdp(points, tolerance=tolerance)
        kept_keys = {round(p.time_s, 9) for p in kept}

        # For each dropped point, find its bracketing kept points and verify
        # the time-lerp deviation is ≤ tolerance.
        kept_sorted = list(kept)
        for p in points:
            if round(p.time_s, 9) in kept_keys:
                continue
            # Find the bracketing kept points (kept is time-sorted)
            lo = None
            hi = None
            for k in kept_sorted:
                if k.time_s <= p.time_s:
                    lo = k
                if k.time_s >= p.time_s and hi is None:
                    hi = k
            self.assertIsNotNone(lo)
            self.assertIsNotNone(hi)
            span = hi.time_s - lo.time_s
            ratio = (p.time_s - lo.time_s) / span if span > 1e-12 else 0.5
            for ax in range(len(p.position)):
                expected = lo.position[ax] + ratio * (hi.position[ax] - lo.position[ax])
                self.assertLessEqual(
                    abs(p.position[ax] - expected),
                    tolerance + 1e-9,
                    f"dropped point at t={p.time_s} axis {ax} exceeds tolerance",
                )


class EdgeCaseTest(_SimplificationTestBase):
    def test_two_point_input_returned_unchanged(self):
        points = [
            TimedJointPoint(0.0, (0.0, 0.0, 0.0, 0.0)),
            TimedJointPoint(1.0, (10.0, 0.0, 0.0, 0.0)),
        ]
        self.assertEqual(simplify_joint_path_rdp(points, tolerance=0.5), tuple(points))

    def test_single_point_input_returned_unchanged(self):
        points = [TimedJointPoint(0.0, (0.0, 0.0, 0.0, 0.0))]
        self.assertEqual(simplify_joint_path_rdp(points, tolerance=0.5), tuple(points))

    def test_zero_tolerance_disables_simplification(self):
        points = self._line((0.0, 0.0), (1.0, 10.0), n=10)
        result = simplify_joint_path_rdp(points, tolerance=0.0)
        self.assertEqual(len(result), 10)

    def test_negative_tolerance_treated_as_disabled(self):
        points = self._line((0.0, 0.0), (1.0, 10.0), n=10)
        result = simplify_joint_path_rdp(points, tolerance=-1.0)
        self.assertEqual(len(result), 10)

    def test_already_sparse_path_kept_when_curvature_high(self):
        # 3 points where the middle is far off the line between endpoints
        points = [
            TimedJointPoint(0.0, (0.0, 0.0, 0.0, 0.0)),
            TimedJointPoint(0.5, (10.0, 0.0, 0.0, 0.0)),  # detour
            TimedJointPoint(1.0, (0.0, 0.0, 0.0, 0.0)),
        ]
        result = simplify_joint_path_rdp(points, tolerance=0.5)
        self.assertEqual(len(result), 3)


if __name__ == "__main__":
    unittest.main()
