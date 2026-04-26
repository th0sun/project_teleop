"""Segment classification for mixed-primitive motion planning.

After RDP simplification reduces redundant waypoints, this module groups
the remaining waypoints into segments that can each be served by a single
MG400 motion command:

    LINE    → ``MovL``   — consecutive points collinear in Cartesian space
    ARC     → ``Arc``    — consecutive points fitting a circular arc
    GENERAL → ``JointMovJ`` chain (fallback)

The classifier works in Cartesian space (mm) so the output maps directly
to the MG400 TCP motion commands which all operate in Cartesian coordinates.
A *forward-kinematics* callback converts the stored joint-space waypoints
to (X, Y, Z, R) on the fly.

Algorithm
---------
Greedy front-to-back scan:

1.  Start a new segment at the current index.
2.  Try to extend it as a LINE: check if every point from start to
    candidate-end falls within ``line_tol_mm`` of the XYZ line through
    start and end.
3.  If the line breaks, try to extend as an ARC: fit a circle through
    start, midpoint, and candidate-end and check if every intermediate
    XYZ point falls within ``arc_tol_mm`` of the fitted circle.
4.  If neither succeeds, fall back to a 2-point GENERAL segment and
    advance.
5.  Repeat from step 1 at the end of the committed segment.

Each segment stores indices into the original (simplified) waypoint list
and the Cartesian coordinates needed to emit the corresponding command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np


class SegmentType(Enum):
    """Motion primitive type for a path segment."""
    LINE = "line"       # → MovL
    ARC = "arc"         # → Arc
    GENERAL = "general" # → JointMovJ chain


@dataclass(frozen=True)
class CartesianPoint:
    """Cartesian pose for the MG400 end-effector."""
    x: float   # mm
    y: float   # mm
    z: float   # mm
    r: float   # deg (yaw)

    def xyz(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def xyzr(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.r)


@dataclass(frozen=True)
class Segment:
    """A classified path segment mapping to a single motion primitive.

    Attributes
    ----------
    type : SegmentType
        LINE, ARC, or GENERAL.
    start_idx : int
        Index of the first waypoint in this segment (into the simplified list).
    end_idx : int
        Index of the last waypoint (inclusive).
    cartesian_points : tuple of CartesianPoint
        All Cartesian poses in [start_idx .. end_idx].
    arc_through_idx : int or None
        For ARC segments only — the index of the "through" point (mid-point
        of the arc).  The Arc command needs current-pos + through + target.
    """
    type: SegmentType
    start_idx: int
    end_idx: int
    cartesian_points: Tuple[CartesianPoint, ...] = field(default_factory=tuple)
    arc_through_idx: Optional[int] = None

    @property
    def point_count(self) -> int:
        return self.end_idx - self.start_idx + 1

    @property
    def start_xyzr(self) -> Tuple[float, float, float, float]:
        return self.cartesian_points[0].xyzr()

    @property
    def end_xyzr(self) -> Tuple[float, float, float, float]:
        return self.cartesian_points[-1].xyzr()

    @property
    def through_xyzr(self) -> Optional[Tuple[float, float, float, float]]:
        if self.arc_through_idx is None:
            return None
        local = self.arc_through_idx - self.start_idx
        return self.cartesian_points[local].xyzr()


# ── Forward kinematics callback type ────────────────────────────────────

# (j1_deg, j2_deg, j3_deg, j4_deg) → (x_mm, y_mm, z_mm, r_deg)
FKFunction = Callable[[float, float, float, float], Tuple[float, float, float, float]]


# ── Geometry helpers ────────────────────────────────────────────────────

def _point_to_line_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Perpendicular distance from *point* to the line through *a* → *b* (3D)."""
    ab = b - a
    ab_len = np.linalg.norm(ab)
    if ab_len < 1e-9:
        return float(np.linalg.norm(point - a))
    t = np.dot(point - a, ab) / (ab_len * ab_len)
    t = max(0.0, min(1.0, t))  # clamp to segment
    projection = a + t * ab
    return float(np.linalg.norm(point - projection))


def _fit_circle_3pts(
    p0: np.ndarray, p1: np.ndarray, p2: np.ndarray,
) -> Tuple[Optional[np.ndarray], Optional[float]]:
    """Fit a circle through 3 points in 3D.

    Returns (center, radius) or (None, None) if points are collinear.
    """
    # Vectors from p0
    v1 = p1 - p0
    v2 = p2 - p0
    cross = np.cross(v1, v2)
    cross_norm = np.linalg.norm(cross)
    if cross_norm < 1e-9:
        return None, None  # collinear

    # Circle in the plane defined by the 3 points
    # Using the circumcenter formula
    d1 = np.dot(v1, v1)
    d2 = np.dot(v2, v2)
    denom = 2.0 * cross_norm * cross_norm
    alpha = d2 * np.dot(v1, v1 - v2) / denom
    beta = d1 * np.dot(v2, v2 - v1) / denom
    center = p0 + alpha * v1 + beta * v2
    radius = float(np.linalg.norm(center - p0))
    return center, radius


def _point_to_circle_distance(
    point: np.ndarray, center: np.ndarray, radius: float,
    normal: np.ndarray,
) -> float:
    """Distance from a point to the circle defined by center/radius/plane.

    Two components: distance from the circle's plane + radial deviation.
    """
    # Project point onto the circle's plane
    v = point - center
    off_plane = abs(float(np.dot(v, normal)))
    # In-plane: radial deviation
    in_plane = v - np.dot(v, normal) * normal
    radial_dev = abs(float(np.linalg.norm(in_plane)) - radius)
    return math.sqrt(off_plane ** 2 + radial_dev ** 2)


def _angle_delta_deg(a: float, b: float) -> float:
    """Shortest absolute angular distance in degrees."""
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _r_lerp(start_r: float, end_r: float, fraction: float) -> float:
    """Interpolate yaw/R along the shortest angular path."""
    delta = (float(end_r) - float(start_r) + 180.0) % 360.0 - 180.0
    return float(start_r) + delta * float(fraction)


def _r_follows_linear_profile(
    cart_points: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_deg: float,
) -> bool:
    """Check whether R/yaw stays close to endpoint interpolation."""
    if end - start < 2:
        return True
    start_r = cart_points[start].r
    end_r = cart_points[end].r
    span = end - start
    for i in range(start + 1, end):
        expected = _r_lerp(start_r, end_r, (i - start) / span)
        if _angle_delta_deg(cart_points[i].r, expected) > tolerance_deg:
            return False
    return True


# ── Line detection ──────────────────────────────────────────────────────

def _is_line_segment(
    cart_points: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_mm: float,
    r_tolerance_deg: float,
) -> bool:
    """Check if points[start..end] are collinear in XYZ within tolerance."""
    if end - start < 2:
        return True  # ≤2 points are trivially a line
    a = cart_points[start].xyz()
    b = cart_points[end].xyz()
    for i in range(start + 1, end):
        d = _point_to_line_distance(cart_points[i].xyz(), a, b)
        if d > tolerance_mm:
            return False
    return _r_follows_linear_profile(cart_points, start, end, r_tolerance_deg)


# ── Arc detection ───────────────────────────────────────────────────────

def _is_arc_segment(
    cart_points: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_mm: float,
    r_tolerance_deg: float,
) -> Optional[int]:
    """Check if points[start..end] lie on a circular arc within tolerance.

    Returns the index of the best through-point (for the Arc command),
    or None if the segment is not a valid arc.
    """
    n = end - start + 1
    if n < 3:
        return None

    # Use the midpoint as the through-point for fitting
    mid = (start + end) // 2
    p0 = cart_points[start].xyz()
    p1 = cart_points[mid].xyz()
    p2 = cart_points[end].xyz()

    center, radius = _fit_circle_3pts(p0, p1, p2)
    if center is None or radius is None:
        return None
    if radius > 5000.0:
        return None  # Too large → effectively a line, not a useful arc

    # Normal of the circle's plane
    v1 = p1 - p0
    v2 = p2 - p0
    normal = np.cross(v1, v2)
    norm_len = np.linalg.norm(normal)
    if norm_len < 1e-9:
        return None
    normal = normal / norm_len

    # Check all intermediate points
    for i in range(start + 1, end):
        d = _point_to_circle_distance(cart_points[i].xyz(), center, radius, normal)
        if d > tolerance_mm:
            return None
    if not _r_follows_linear_profile(cart_points, start, end, r_tolerance_deg):
        return None

    return mid


# ── Main classifier ─────────────────────────────────────────────────────

def classify_segments(
    joint_points: Sequence[Tuple[float, float, float, float]],
    fk_fn: FKFunction,
    line_tol_mm: float = 1.0,
    arc_tol_mm: float = 2.0,
    r_tol_deg: float = 2.0,
    min_points_for_arc: int = 3,
    enable_arc: bool = True,
) -> List[Segment]:
    """Classify a simplified waypoint sequence into typed motion segments.

    Parameters
    ----------
    joint_points
        Sequence of (j1, j2, j3, j4) in degrees — the already-simplified
        waypoints from the RDP pass.
    fk_fn
        Forward kinematics: (j1,j2,j3,j4) → (x_mm, y_mm, z_mm, r_deg).
    line_tol_mm
        Maximum Cartesian deviation (mm) for a segment to be classified
        as LINE.
    arc_tol_mm
        Maximum Cartesian deviation (mm) from the fitted circle for an
        ARC segment.
    r_tol_deg
        Maximum wrist/yaw deviation (deg) from endpoint interpolation for
        LINE/ARC compression.  If the tool orientation changes with a
        different profile, the segment falls back to GENERAL.
    min_points_for_arc
        Minimum number of waypoints required to attempt arc fitting.
    enable_arc
        If False, skip ARC classification and use LINE or GENERAL only.  This
        is useful for test backends such as the current MG400 Mock image, which
        accepts ``MovL`` but does not implement ``Arc``.

    Returns
    -------
    List of Segment, covering the entire waypoint sequence end-to-end.
    Each segment's ``end_idx`` equals the next segment's ``start_idx``
    (shared boundary point) to ensure continuity.
    """
    n = len(joint_points)
    if n <= 1:
        if n == 1:
            cp = _joints_to_cart(joint_points[0], fk_fn)
            return [Segment(SegmentType.GENERAL, 0, 0,
                            cartesian_points=(cp,))]
        return []

    # Pre-compute all Cartesian points
    cart = [_joints_to_cart(jp, fk_fn) for jp in joint_points]

    segments: List[Segment] = []
    i = 0

    while i < n - 1:
        best_end = i + 1
        best_type = SegmentType.GENERAL
        best_through: Optional[int] = None

        # 1. Try to extend as LINE (longest first)
        for candidate_end in range(n - 1, i, -1):
            if _is_line_segment(cart, i, candidate_end, line_tol_mm, r_tol_deg):
                best_end = candidate_end
                best_type = SegmentType.LINE
                break

        # 2. Try ARC when enabled — greedily extend from n-1 downward.
        #    ARC wins only when it covers MORE waypoints than the best LINE
        #    (or when no LINE was found).  This fixes the case where LINE
        #    collects many short segments for a smooth curve (e.g. J1 rotation)
        #    that would be served by a single Arc command instead.
        if enable_arc:
            for candidate_end in range(n - 1, i + min_points_for_arc - 1, -1):
                if candidate_end <= best_end:
                    break  # ARC must cover strictly more waypoints than best LINE
                through_idx = _is_arc_segment(cart, i, candidate_end, arc_tol_mm, r_tol_deg)
                if through_idx is not None:
                    best_end = candidate_end
                    best_type = SegmentType.ARC
                    best_through = through_idx
                    break

        segments.append(Segment(
            type=best_type,
            start_idx=i,
            end_idx=best_end,
            cartesian_points=tuple(cart[i:best_end + 1]),
            arc_through_idx=best_through,
        ))
        i = best_end  # next segment starts at the end of this one

    return segments


def _joints_to_cart(
    joints: Tuple[float, float, float, float], fk_fn: FKFunction,
) -> CartesianPoint:
    x, y, z, r = fk_fn(joints[0], joints[1], joints[2], joints[3])
    return CartesianPoint(x=x, y=y, z=z, r=r)


# ── Summary helpers ─────────────────────────────────────────────────────

def segment_summary(segments: List[Segment]) -> str:
    """Human-readable summary of classified segments."""
    counts = {t: 0 for t in SegmentType}
    total_points = 0
    for s in segments:
        counts[s.type] += 1
        total_points = max(total_points, s.end_idx + 1)

    total_commands = sum(
        1 if s.type in (SegmentType.LINE, SegmentType.ARC)
        else s.point_count - 1
        for s in segments
    )
    parts = []
    if counts[SegmentType.LINE]:
        parts.append(f"{counts[SegmentType.LINE]} MovL")
    if counts[SegmentType.ARC]:
        parts.append(f"{counts[SegmentType.ARC]} Arc")
    if counts[SegmentType.GENERAL]:
        n_gen_cmds = sum(s.point_count - 1 for s in segments if s.type == SegmentType.GENERAL)
        parts.append(f"{n_gen_cmds} JointMovJ")

    return f"{total_points} pts → {total_commands} cmds ({', '.join(parts)})"
