"""Segment classifier tests — ground-truth shapes for teach-and-repeat playback.

Each test pins behaviour for a recognisable demonstration shape and the
classifier output it should produce.  Read this file to understand which
trajectory geometry maps to which MG400 motion primitive (MovL, Arc,
JointMovJ chain) and why — the comments next to each shape explain the
geometric reason for the expected classification.

Tolerances mirror ``motion_config.py`` defaults:

    line_tol_mm = 2.0
    arc_tol_mm  = 3.0
    max_arc_radius_mm = 10000

These are the production knobs; tests use them verbatim so the test
output reflects what the live controller will actually emit.
"""

import math
import unittest
from typing import List, Tuple

import numpy as np

from teaching_core.trajectory.segment_classifier import (
    SegmentType,
    classify_segments,
    segment_summary,
)


# ── Synthetic FK matching MG400 4-DOF SCARA-like geometry ───────────────
# A full-precision FK lives in mg400_controller; this test-only stub
# captures the same kinematic *shape* (planar-arm forward kinematics with
# J1 rotation around Z, J2/J3 in a vertical plane, J4 = wrist yaw) so the
# classifier sees realistic Cartesian curvature without depending on the
# downstream package.  Link lengths roughly match an MG400.

_L1 = 175.0
_L2 = 175.0
_OFFSET = 109.0
_BASE_Z = 293.0


def _fk(j1: float, j2: float, j3: float, j4: float) -> Tuple[float, float, float, float]:
    """Approximate MG400 FK in degrees in / (mm, deg) out.

    The exact link lengths don't matter for these tests; what matters is
    that pure J1 rotation traces a circular arc in XY, J2/J3 motion is
    curved in XZ, and a coordinated combination produces curvature in
    both planes — all of which the classifier needs to recognise.
    """
    j1r, j2r, j3r = math.radians(j1), math.radians(j2), math.radians(j3)
    r = _L1 * math.cos(j2r) + _L2 * math.cos(j2r + j3r) + _OFFSET
    z = _L1 * math.sin(j2r) + _L2 * math.sin(j2r + j3r) + _BASE_Z
    x = r * math.cos(j1r)
    y = r * math.sin(j1r)
    return (x, y, z, j4)


def _shape_summary(name: str, segs) -> str:
    return f"{name}: {segment_summary(segs)}"


# ── Tests ────────────────────────────────────────────────────────────────

class CurveShapesAsArc(unittest.TestCase):
    """Smooth curves must collapse to a single Arc command."""

    def test_pure_j1_sweep_is_one_arc(self):
        # Pure base rotation 0→60° over 10 waypoints — TCP traces a
        # circular arc in XY at constant Z.  The whole sweep is one Arc.
        pts = [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 60, 10)]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(len(segs), 1, _shape_summary("pure J1 sweep", segs))
        self.assertEqual(segs[0].type, SegmentType.ARC)
        self.assertEqual(segs[0].point_count, 10)

    def test_pure_j1_sweep_with_r_wobble_still_one_arc(self):
        # Same J1 sweep but with random ±5° R/yaw wobble — wrist shake is
        # normal in VR demos.  The R gate allows small deviations from the
        # controller's endpoint interpolation, so the geometry-based ARC fit
        # must still win.
        rng = np.random.default_rng(0)
        pts = [
            (j, 0.0, -90.0, float(rng.uniform(-5, 5)))
            for j in np.linspace(0, 60, 10)
        ]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(len(segs), 1, _shape_summary("J1 sweep + R wobble", segs))
        self.assertEqual(segs[0].type, SegmentType.ARC)

    def test_non_linear_r_motion_splits_instead_of_single_arc(self):
        # XYZ traces a clean circular arc, but the wrist makes a deliberate
        # non-linear 60° move in the middle and returns.  Collapsing this into
        # one Arc would lose the demonstrated R timing; splitting into multiple
        # Arc/GENERAL segments is acceptable because each Arc has its own
        # through-point R value.
        pts = [
            (j, 0.0, -90.0, r)
            for j, r in zip(np.linspace(0, 60, 7), [0.0, 0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        ]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertGreater(len(segs), 1, _shape_summary("J1 sweep + intentional R move", segs))

    def test_curved_j2_descent_is_one_arc(self):
        # Coordinated J2/J3 descent — TCP curves in XZ (radius set by
        # link geometry).  Should be one Arc, not many MovL.
        pts = [(0.0, 30.0 * t, -90.0 + 30.0 * t, 0.0) for t in np.linspace(0, 1, 10)]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(len(segs), 1, _shape_summary("J2/J3 descent", segs))
        self.assertEqual(segs[0].type, SegmentType.ARC)


class StraightShapesAsLine(unittest.TestCase):
    """Genuinely straight Cartesian paths must use MovL, not Arc."""

    def test_short_chord_path_is_one_line(self):
        # Tiny J1 sweep — chord deviation under 2mm makes both LINE and
        # ARC fit; tie-break favours LINE (simpler primitive).
        pts = [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 5, 6)]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(len(segs), 1, _shape_summary("short J1", segs))
        self.assertEqual(segs[0].type, SegmentType.LINE)

    def test_two_point_path_is_general(self):
        # 2 waypoints can't form an Arc (needs ≥3) and a 2-point LINE is
        # degenerate — emit one JointMovJ chunk to preserve fidelity.
        pts = [(0.0, 0.0, -90.0, 0.0), (30.0, 10.0, -80.0, 5.0)]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(len(segs), 1, _shape_summary("two-point path", segs))
        self.assertEqual(segs[0].type, SegmentType.GENERAL)


class CompoundShapes(unittest.TestCase):
    """Multi-phase paths split at the curvature break, not within phases."""

    def test_pick_place_decomposes_to_few_arcs(self):
        # Three phases: J1 sweep out → J2/J3 descend → J1 sweep back.
        # Each phase is its own Arc; the classifier's greedy longest-first
        # naturally cuts at curvature breaks.  Total ≤ 3 commands instead
        # of ~13 JointMovJ.
        pts: List[Tuple[float, float, float, float]] = []
        pts += [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 30, 5)]
        pts += [(30.0, b, -90.0 + b, 0.0) for b in np.linspace(0, 20, 4)[1:]]
        pts += [(j, 20.0, -70.0, 0.0) for j in np.linspace(30, 60, 5)[1:]]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertLessEqual(len(segs), 3, _shape_summary("pick-place 3-phase", segs))

    def test_elbow_turn_breaks_into_two_arcs(self):
        # Elbow turn: pure J1 sweep (TCP arcs in XY plane) then pure J2/J3
        # descent (TCP arcs in vertical plane).  The two arcs lie in
        # different planes so a single circle cannot fit them — classifier
        # must break at the corner.  Expect 2 segments, both ARC.
        pts: List[Tuple[float, float, float, float]] = []
        pts += [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 30, 6)]
        pts += [(30.0, b, -90.0 + b, 0.0) for b in np.linspace(0, 20, 6)[1:]]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(len(segs), 2, _shape_summary("elbow turn", segs))
        for seg in segs:
            self.assertNotEqual(seg.type, SegmentType.GENERAL,
                                f"elbow piece fell back to GENERAL: {seg}")


class GuaranteesAndEdgeCases(unittest.TestCase):
    """Invariants that must hold regardless of input shape."""

    def test_segments_cover_input_end_to_end(self):
        pts = [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 45, 12)]
        segs = classify_segments(pts, fk_fn=_fk)
        self.assertEqual(segs[0].start_idx, 0)
        self.assertEqual(segs[-1].end_idx, len(pts) - 1)
        for prev, nxt in zip(segs, segs[1:]):
            self.assertEqual(prev.end_idx, nxt.start_idx,
                             "adjacent segments must share boundary waypoint")

    def test_disable_arc_falls_back_to_line_or_general(self):
        # Mock backends without Arc support — classifier must still
        # produce a valid covering using LINE/GENERAL only.
        pts = [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 60, 10)]
        segs = classify_segments(pts, fk_fn=_fk, enable_arc=False)
        types = {s.type for s in segs}
        self.assertNotIn(SegmentType.ARC, types)
        self.assertEqual(segs[-1].end_idx, len(pts) - 1)

    def test_arc_through_point_is_valid_index(self):
        pts = [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 60, 10)]
        segs = classify_segments(pts, fk_fn=_fk)
        for seg in segs:
            if seg.type == SegmentType.ARC:
                self.assertIsNotNone(seg.arc_through_idx)
                self.assertGreater(seg.arc_through_idx, seg.start_idx)
                self.assertLess(seg.arc_through_idx, seg.end_idx)
                self.assertIsNotNone(seg.through_xyzr)

    def test_max_arc_radius_pushes_near_straight_to_line(self):
        # Tiny-curvature path with a small radius cap — fitted radius
        # exceeds the cap, so Arc is rejected and the path is demoted to
        # LINE / GENERAL.
        pts = [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 1.5, 6)]
        segs = classify_segments(pts, fk_fn=_fk, max_arc_radius_mm=100.0)
        self.assertTrue(all(s.type != SegmentType.ARC for s in segs),
                        f"radius cap should reject Arc; got {segment_summary(segs)}")

    def test_imbalanced_vertical_arc_is_rejected(self):
        # Regression for the real MG400 alarm we saw on Pick_place_1:
        # start→through is tiny, through→target is much longer, and Z rises
        # sharply.  The endpoint IK can pass, but the controller's internal Arc
        # interpolation can spike joint velocity.  Do not emit a single Arc for
        # this geometry.
        def identity_fk(x, y, z, r):
            return (x, y, z, r)

        pts = [
            (0.0, 0.0, 0.0, 0.0),
            (11.0, -1.0, 0.0, 1.0),
            (70.0, -20.0, 58.0, 4.0),
        ]
        segs = classify_segments(pts, fk_fn=identity_fk, arc_tol_mm=3.0)

        self.assertTrue(all(seg.type != SegmentType.ARC for seg in segs), segment_summary(segs))

    def test_balanced_quarter_circle_is_arc(self):
        def identity_fk(x, y, z, r):
            return (x, y, z, r)

        pts = [
            (100.0 * math.cos(a), 100.0 * math.sin(a), 50.0, 0.0)
            for a in np.linspace(0.0, math.pi / 2.0, 9)
        ]
        segs = classify_segments(pts, fk_fn=identity_fk, arc_tol_mm=0.5)

        self.assertEqual(len(segs), 1, segment_summary(segs))
        self.assertEqual(segs[0].type, SegmentType.ARC)
        self.assertIsNotNone(segs[0].through_xyzr)

    def test_arc_with_straight_run_splits(self):
        # Regression for Pick_place_1 kept waypoint 22→28.  One shallow Arc can
        # fit the full span numerically, but visually and mechanically it is a
        # straight/near-straight run joined to a curve.  The classifier should
        # prefer smaller honest primitives instead of one long "magic" Arc.
        def identity_fk(x, y, z, r):
            return (x, y, z, r)

        pts = [
            (202.226279, 252.498089, -72.378286, 51.308685),
            (201.370558, 253.943285, -27.991391, 51.586479),
            (200.702934, 255.477875, 17.133648, 51.846889),
            (198.548609, 255.726845, 57.788775, 52.173916),
            (195.501903, 255.542246, 89.420805, 52.582302),
            (193.042748, 254.876674, 109.094472, 52.859833),
            (192.380033, 253.916040, 116.623028, 52.850533),
        ]
        segs = classify_segments(pts, fk_fn=identity_fk, line_tol_mm=2.0, arc_tol_mm=3.0)

        self.assertGreater(len(segs), 1, segment_summary(segs))
        self.assertFalse(
            len(segs) == 1 and segs[0].type == SegmentType.ARC,
            segment_summary(segs),
        )


class CommandReductionBenchmark(unittest.TestCase):
    """Snapshot expected command counts so regressions are immediately visible."""

    def test_documented_reductions(self):
        cases = {
            "pure J1 sweep 10pts": (
                [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 60, 10)], 1,
            ),
            "J2/J3 descent 10pts": (
                [(0.0, 30 * t, -90 + 30 * t, 0.0) for t in np.linspace(0, 1, 10)], 1,
            ),
            "elbow turn 11pts (J1 then J2/J3)": (
                [(j, 0.0, -90.0, 0.0) for j in np.linspace(0, 30, 6)]
                + [(30.0, b, -90.0 + b, 0.0) for b in np.linspace(0, 20, 6)[1:]],
                2,
            ),
        }
        for name, (pts, expected_cmds) in cases.items():
            segs = classify_segments(pts, fk_fn=_fk)
            actual_cmds = sum(
                1 if s.type in (SegmentType.LINE, SegmentType.ARC)
                else s.point_count - 1
                for s in segs
            )
            self.assertLessEqual(
                actual_cmds, expected_cmds,
                f"{name}: expected ≤ {expected_cmds} cmds, got {actual_cmds} "
                f"({segment_summary(segs)})",
            )


if __name__ == "__main__":
    unittest.main()
