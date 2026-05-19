#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Geometric / kinematic helpers used by the playback compile pipeline.

Pure functions pulled out of ``TrajectoryRecorder`` so the recorder
class stays focused on state + orchestration.  Behaviour is identical
to the pre-extraction code; the recorder's private methods
(``_point_to_polyline_distance``, ``_fk_for_classifier``, ``_frame_q_deg``,
``_frame_xyzr``, ``_tool_pose_reachable``, ``_line_primitive_reachable``,
``_arc_primitive_points``, ``_arc_primitive_reachable``,
``_primitive_path_xyz``, ``_raw_fit_error_mm``) now delegate to the
functions defined here.

These helpers do not depend on recorder instance state — they take
their inputs as arguments — so they are safe to use from other
analysis tools (log replay, test fixtures) without instantiating a
recorder.

The larger ``compile_loaded_plan`` / ``_compile_mixed_commands`` glue
remains on the recorder for now; see AGENTS.md §10 for the C3 plan
(playback_executor extraction).
"""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

import numpy as np

from mg400_controller.common.utils.kinematics import KinematicsCalculator
from teaching_core.trajectory.command_arc_sampling import sample_command_arc_xyzr
from teaching_core.trajectory.segment_classifier import SegmentType


# ── Joint-frame accessors ───────────────────────────────────────────────────
def frame_q_deg(frame: dict) -> Tuple[float, float, float, float]:
    """Extract the J1-J4 joint angles (degrees) from a recorder frame dict."""
    return (
        float(frame["j1"]),
        float(frame["j2"]),
        float(frame["j3"]),
        float(frame["j4"]),
    )


def fk_for_classifier(j1: float, j2: float, j3: float, j4: float
                      ) -> Tuple[float, float, float, float]:
    """FK adapter for segment classifier: (j1,j2,j3,j4) → (x,y,z,r)."""
    kin = KinematicsCalculator()
    result = kin.forward_kinematics([j1, j2, j3, j4])
    # result is [x, y, z, rx, ry, rz]
    return (
        float(result[0]),
        float(result[1]),
        float(result[2]),
        float(result[3]),
    )


def frame_xyzr(frame: dict) -> Tuple[float, float, float, float]:
    """Return MG400 tool pose for one frame in controller coordinates."""
    return tuple(float(v) for v in fk_for_classifier(*frame_q_deg(frame)))


# ── IK / reachability ──────────────────────────────────────────────────────
def tool_pose_reachable(xyzr: Sequence[float]) -> bool:
    """Return True if ``KinematicsCalculator`` can resolve IK for the pose."""
    return KinematicsCalculator().is_tool_pose_reachable(xyzr)


def line_primitive_reachable(seg, samples: int = 12) -> bool:
    """Sample a straight-line Cartesian segment and check every sample IKs."""
    start = np.array(seg.start_xyzr, dtype=float)
    end = np.array(seg.end_xyzr, dtype=float)
    for fraction in np.linspace(0.0, 1.0, max(2, samples)):
        pose = start + (end - start) * float(fraction)
        if not tool_pose_reachable(pose[:4]):
            return False
    return True


def arc_primitive_points(seg, samples: int = 16) -> List[Tuple[float, ...]]:
    """Discrete XYZR samples along the arc defined by start/through/end."""
    through_xyzr = seg.through_xyzr
    if through_xyzr is None:
        return []
    return sample_command_arc_xyzr(seg.start_xyzr, through_xyzr, seg.end_xyzr, samples)


def arc_primitive_reachable(seg, samples: int = 16) -> bool:
    """Sample an arc segment and check every sample IKs."""
    points = arc_primitive_points(seg, samples=samples)
    if not points:
        return False
    return all(tool_pose_reachable(point) for point in points)


# ── Geometry ───────────────────────────────────────────────────────────────
def point_to_polyline_distance(point_xyz, polyline_xyz) -> float:
    """Minimum perpendicular distance from a 3D point to a polyline."""
    point = np.asarray(point_xyz, dtype=float)
    polyline = np.asarray(polyline_xyz, dtype=float)
    if len(polyline) == 0:
        return 0.0
    if len(polyline) == 1:
        return float(np.linalg.norm(point - polyline[0]))

    best = float("inf")
    for start, end in zip(polyline[:-1], polyline[1:]):
        segment = end - start
        denom = float(np.dot(segment, segment))
        if denom <= 1e-12:
            dist = float(np.linalg.norm(point - start))
        else:
            t = float(np.dot(point - start, segment) / denom)
            t = max(0.0, min(1.0, t))
            closest = start + t * segment
            dist = float(np.linalg.norm(point - closest))
        best = min(best, dist)
    return best


def primitive_path_xyz(seg, primitive_type: SegmentType) -> np.ndarray:
    """Densified XYZ samples representing the primitive's commanded path."""
    if primitive_type == SegmentType.LINE:
        return np.asarray(
            [
                np.asarray(seg.start_xyzr[:3], dtype=float),
                np.asarray(seg.end_xyzr[:3], dtype=float),
            ],
            dtype=float,
        )
    if primitive_type == SegmentType.ARC and seg.through_xyzr is not None:
        samples = sample_command_arc_xyzr(
            seg.start_xyzr,
            seg.through_xyzr,
            seg.end_xyzr,
            samples=48,
        )
        return np.asarray(
            [np.asarray(p[:3], dtype=float) for p in samples], dtype=float
        )
    return np.asarray([], dtype=float)


def raw_fit_error_mm(
    seg,
    primitive_type: SegmentType,
    source_frames: Sequence[dict],
    raw_frames: Sequence[dict],
) -> float:
    """Max XYZ distance from raw Unity path to the emitted primitive path.

    Classification happens after RDP simplification.  This guard checks
    the command-shaped primitive against the original dense capture so
    a sparse set of kept waypoints cannot accidentally approve an Arc /
    MovL that cuts too far away from what the user taught.
    """
    if not source_frames or not raw_frames:
        return 0.0
    if primitive_type not in {SegmentType.LINE, SegmentType.ARC}:
        return 0.0

    primitive_path = primitive_path_xyz(seg, primitive_type)
    if len(primitive_path) < 2:
        return float("inf")

    start_t = float(source_frames[seg.start_idx]["timeStamp"])
    end_t = float(source_frames[seg.end_idx]["timeStamp"])
    if start_t > end_t:
        start_t, end_t = end_t, start_t

    eps = 1e-6
    raw_slice = [
        frame for frame in raw_frames
        if start_t - eps <= float(frame["timeStamp"]) <= end_t + eps
    ]
    if not raw_slice:
        raw_slice = [source_frames[seg.start_idx], source_frames[seg.end_idx]]

    max_error = 0.0
    for frame in raw_slice:
        raw_xyz = np.asarray(frame_xyzr(frame)[:3], dtype=float)
        error = point_to_polyline_distance(raw_xyz, primitive_path)
        max_error = max(max_error, error)
    return float(max_error)
