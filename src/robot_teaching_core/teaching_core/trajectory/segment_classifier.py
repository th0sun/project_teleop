"""Segment classification for mixed-primitive motion planning.

After RDP simplification reduces the recorded waypoint count, this module
groups the remaining waypoints into segments that each map to a single
MG400 motion command:

    LINE    → ``MovL``   — consecutive points collinear in Cartesian space
    ARC     → ``Arc``    — consecutive points fitting a circular arc
    GENERAL → ``JointMovJ`` chain (fallback for genuinely irregular shape)

Algorithm
---------
At each anchor index ``i`` we run two independent longest-first searches:

1.  longest LINE span starting at ``i`` (XYZ-collinearity within
    ``line_tol_mm``)
2.  longest ARC span starting at ``i`` (least-squares circle fit, every
    intermediate point within ``arc_tol_mm`` of the fitted circle)

Whichever covers more waypoints wins.  On a tie LINE wins because a
straight line is the degenerate case of an arc with infinite radius —
emitting MovL is simpler and more predictable on the controller.  If
neither succeeds the segment falls back to a 2-point GENERAL chunk and
the scan advances by one waypoint.

R-axis check
------------
MG400 ``MovL`` and ``Arc`` interpolate R/yaw between endpoint poses, so the
classifier may collapse small hand wobble.  It must not collapse a path where
the demonstrated wrist/yaw motion is intentionally non-linear, because that
would silently lose task intent.  LINE/ARC candidates therefore need both XYZ
geometry and R/yaw to match endpoint interpolation within ``r_tol_deg``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np


class SegmentType(Enum):
    """Motion primitive type for a path segment."""
    LINE = "line"        # → MovL
    ARC = "arc"          # → Arc
    GENERAL = "general"  # → JointMovJ chain


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

    ``cartesian_points`` holds every Cartesian pose in
    ``[start_idx .. end_idx]`` so the renderer can emit MovL / Arc /
    JointMovJ commands without a second FK pass.  ``arc_through_idx`` is
    the absolute index (into the simplified waypoint list) of the arc's
    through-point — the Arc command needs current-pos + through + target.
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

@dataclass(frozen=True)
class _CommandArc:
    """Exact circular path implied by Dobot ``Arc(current, through, target)``."""
    center: np.ndarray
    radius: float
    normal: np.ndarray
    u: np.ndarray
    v: np.ndarray
    total_angle: float
    through_fraction: float
    start_to_through_mm: float
    through_to_end_mm: float
    chord_mm: float

    @property
    def sweep_rad(self) -> float:
        return abs(self.total_angle)

    @property
    def min_leg_mm(self) -> float:
        return min(self.start_to_through_mm, self.through_to_end_mm)

    @property
    def max_leg_mm(self) -> float:
        return max(self.start_to_through_mm, self.through_to_end_mm)

    @property
    def leg_ratio(self) -> float:
        if self.max_leg_mm < 1e-9:
            return 0.0
        return self.min_leg_mm / self.max_leg_mm

def _point_to_line_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Perpendicular distance from *point* to the line through *a* → *b* (3D).

    The projection parameter is clamped to [0, 1] so points outside the
    segment are measured to the nearest endpoint instead of an infinite
    line — this matches the geometric intent of "is this point ON the
    line segment between a and b".
    """
    ab = b - a
    ab_len = np.linalg.norm(ab)
    if ab_len < 1e-9:
        return float(np.linalg.norm(point - a))
    t = np.dot(point - a, ab) / (ab_len * ab_len)
    t = max(0.0, min(1.0, t))
    projection = a + t * ab
    return float(np.linalg.norm(point - projection))


def _fit_circle_least_squares(
    points: Sequence[np.ndarray],
) -> Tuple[Optional[np.ndarray], Optional[float], Optional[np.ndarray]]:
    """Best-fit circle through ``points`` (≥3 in 3D) via Kasa method.

    The plane is found by SVD on the centered points; the in-plane
    least-squares fit is the algebraic Kasa solution
    ``x² + y² + ax + by + c = 0``.

    Returns ``(center, radius, plane_normal)`` or ``(None, None, None)``
    if the input has < 3 points or the linear system is rank-deficient
    (degenerate / collinear / numerically singular).
    """
    if len(points) < 3:
        return None, None, None

    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 3:
        return None, None, None

    centroid = pts.mean(axis=0)
    centered = pts - centroid

    # SVD → smallest singular vector is the plane normal
    _, _, vh = np.linalg.svd(centered)
    normal = vh[2, :]
    normal_norm = np.linalg.norm(normal)
    if normal_norm < 1e-12:
        return None, None, None
    normal = normal / normal_norm

    # Build an orthonormal basis (u, v) for the plane
    seed = np.array([1.0, 0.0, 0.0]) if abs(normal[2]) > 0.9 else np.array([0.0, 0.0, 1.0])
    u = seed - np.dot(seed, normal) * normal
    u_norm = np.linalg.norm(u)
    if u_norm < 1e-12:
        return None, None, None
    u = u / u_norm
    v_vec = np.cross(normal, u)

    x_2d = centered @ u
    y_2d = centered @ v_vec
    z = x_2d ** 2 + y_2d ** 2
    A = np.column_stack([x_2d, y_2d, np.ones(len(x_2d))])

    try:
        coeffs, _, rank, _ = np.linalg.lstsq(A, -z, rcond=None)
    except np.linalg.LinAlgError:
        return None, None, None
    if rank < 3:
        return None, None, None

    a, b, c = coeffs
    cx_2d = -a / 2.0
    cy_2d = -b / 2.0
    r_squared = cx_2d ** 2 + cy_2d ** 2 - c
    if r_squared <= 0:
        return None, None, None

    radius = math.sqrt(r_squared)
    center = centroid + cx_2d * u + cy_2d * v_vec
    return center, radius, normal


def _point_to_circle_distance(
    point: np.ndarray, center: np.ndarray, radius: float, normal: np.ndarray,
) -> float:
    """3D distance from a point to the planar circle.

    Combines the off-plane component (point's height above the circle's
    plane) and the in-plane radial deviation (|‖p_in_plane − center‖ − r|)
    in quadrature.  This matches the geometric distance to the circle in
    its embedded 3D form, not the 2D in-plane distance alone.
    """
    v = point - center
    off_plane = abs(float(np.dot(v, normal)))
    in_plane = v - np.dot(v, normal) * normal
    radial_dev = abs(float(np.linalg.norm(in_plane)) - radius)
    return math.sqrt(off_plane ** 2 + radial_dev ** 2)


def _positive_angle(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _command_arc_from_points(
    start_xyz: np.ndarray,
    through_xyz: np.ndarray,
    end_xyz: np.ndarray,
) -> Optional[_CommandArc]:
    """Return the exact arc that the MG400 command would define.

    This is intentionally command-shaped, not least-squares-shaped: the real
    robot receives only current, through, and target, so classification must
    validate against that exact circle.
    """
    v1 = through_xyz - start_xyz
    v2 = end_xyz - start_xyz
    cross = np.cross(v1, v2)
    cross_norm = float(np.linalg.norm(cross))
    if cross_norm < 1e-9:
        return None

    d1 = float(np.dot(v1, v1))
    d2 = float(np.dot(v2, v2))
    denom = 2.0 * cross_norm * cross_norm
    center = start_xyz + (d2 * float(np.dot(v1, v1 - v2)) / denom) * v1
    center += (d1 * float(np.dot(v2, v2 - v1)) / denom) * v2
    radius = float(np.linalg.norm(start_xyz - center))
    if radius < 1e-9:
        return None

    normal = cross / cross_norm
    u = (start_xyz - center) / radius
    v = np.cross(normal, u)

    def raw_angle(point: np.ndarray) -> float:
        rel = point - center
        return math.atan2(float(np.dot(rel, v)), float(np.dot(rel, u)))

    raw_through = raw_angle(through_xyz)
    raw_end = raw_angle(end_xyz)
    ccw_through = _positive_angle(raw_through)
    ccw_end = _positive_angle(raw_end)
    if 0.0 <= ccw_through <= ccw_end:
        total_angle = ccw_end
        through_fraction = ccw_through / ccw_end if ccw_end > 1e-9 else 0.5
    else:
        cw_through = _positive_angle(-raw_through)
        cw_end = _positive_angle(-raw_end)
        if not (0.0 <= cw_through <= cw_end):
            return None
        total_angle = -cw_end
        through_fraction = cw_through / cw_end if cw_end > 1e-9 else 0.5

    return _CommandArc(
        center=center,
        radius=radius,
        normal=normal,
        u=u,
        v=v,
        total_angle=total_angle,
        through_fraction=float(through_fraction),
        start_to_through_mm=float(np.linalg.norm(through_xyz - start_xyz)),
        through_to_end_mm=float(np.linalg.norm(end_xyz - through_xyz)),
        chord_mm=float(np.linalg.norm(end_xyz - start_xyz)),
    )


def sample_command_arc_xyzr(
    start_xyzr: Sequence[float],
    through_xyzr: Sequence[float],
    end_xyzr: Sequence[float],
    samples: int,
) -> List[Tuple[float, float, float, float]]:
    """Sample the exact Dobot ``Arc`` geometry including piecewise R motion.

    The returned poses are for analysis/preflight only; they are not sent to
    the robot.  If the three points cannot form an arc, an empty list is
    returned.
    """
    start = np.asarray(start_xyzr[:3], dtype=float)
    through = np.asarray(through_xyzr[:3], dtype=float)
    end = np.asarray(end_xyzr[:3], dtype=float)
    arc = _command_arc_from_points(start, through, end)
    if arc is None:
        return []

    r0 = float(start_xyzr[3])
    r1 = float(through_xyzr[3])
    r2 = float(end_xyzr[3])
    out: List[Tuple[float, float, float, float]] = []
    for fraction in np.linspace(0.0, 1.0, max(3, samples)):
        theta = arc.total_angle * float(fraction)
        xyz = arc.center + arc.radius * (math.cos(theta) * arc.u + math.sin(theta) * arc.v)
        if arc.through_fraction <= 1e-9:
            r = r2
        elif fraction <= arc.through_fraction:
            r = _interp_angle_deg(r0, r1, fraction / arc.through_fraction)
        else:
            tail = (fraction - arc.through_fraction) / max(1.0 - arc.through_fraction, 1e-9)
            r = _interp_angle_deg(r1, r2, tail)
        out.append((float(xyz[0]), float(xyz[1]), float(xyz[2]), float(r)))
    return out


def _point_to_command_arc_distance(point: np.ndarray, arc: _CommandArc) -> Tuple[float, Optional[float]]:
    rel = point - arc.center
    raw_angle = math.atan2(float(np.dot(rel, arc.v)), float(np.dot(rel, arc.u)))
    if arc.total_angle >= 0.0:
        along = _positive_angle(raw_angle)
    else:
        along = _positive_angle(-raw_angle)
    total = abs(arc.total_angle)
    fraction: Optional[float] = along / total if total > 1e-9 else None
    if fraction is not None and fraction > 1.0 + 1e-6:
        fraction = None

    off_plane = abs(float(np.dot(rel, arc.normal)))
    in_plane = rel - np.dot(rel, arc.normal) * arc.normal
    radial_dev = abs(float(np.linalg.norm(in_plane)) - arc.radius)
    return math.sqrt(off_plane ** 2 + radial_dev ** 2), fraction


def _angle_delta_deg(a: float, b: float) -> float:
    """Shortest signed angular distance from ``b`` to ``a`` in degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


def _interp_angle_deg(a: float, b: float, t: float) -> float:
    """Interpolate yaw in degrees along the shortest angular path."""
    return a + _angle_delta_deg(b, a) * max(0.0, min(1.0, t))


def _r_matches_endpoint_interpolation(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_deg: float,
) -> bool:
    """Does R/yaw follow the controller's endpoint interpolation closely?

    Tiny VR wrist wobble is acceptable, but a deliberate non-linear wrist path
    should remain GENERAL so it can be represented by intermediate JointMovJ
    waypoints instead of being flattened into one MovL/Arc.
    """
    if end - start < 2:
        return True
    if tolerance_deg < 0:
        return True

    start_r = float(cart[start].r)
    end_r = float(cart[end].r)
    span = end - start
    for i in range(start + 1, end):
        t = (i - start) / span
        expected = _interp_angle_deg(start_r, end_r, t)
        if abs(_angle_delta_deg(float(cart[i].r), expected)) > tolerance_deg:
            return False
    return True


def _r_matches_arc_interpolation(
    cart: Sequence[CartesianPoint],
    start: int,
    through: int,
    end: int,
    through_fraction: float,
    tolerance_deg: float,
) -> bool:
    if tolerance_deg < 0:
        return True
    start_r = float(cart[start].r)
    through_r = float(cart[through].r)
    end_r = float(cart[end].r)
    span = end - start
    if span < 2:
        return True
    for i in range(start + 1, end):
        path_fraction = (i - start) / span
        if through_fraction <= 1e-9:
            expected = end_r
        elif path_fraction <= through_fraction:
            expected = _interp_angle_deg(start_r, through_r, path_fraction / through_fraction)
        else:
            tail = (path_fraction - through_fraction) / max(1.0 - through_fraction, 1e-9)
            expected = _interp_angle_deg(through_r, end_r, tail)
        if abs(_angle_delta_deg(float(cart[i].r), expected)) > tolerance_deg:
            return False
    return True


# ── LINE / ARC primitives ───────────────────────────────────────────────

def _is_line_segment(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_mm: float,
    r_tolerance_deg: float,
) -> bool:
    """Are points[start..end] collinear in XYZ within ``tolerance_mm``?

    <= 2 points are trivially collinear.  R/yaw must also match the
    controller's endpoint interpolation within ``r_tolerance_deg``.
    """
    if end - start < 2:
        return True
    if not _r_matches_endpoint_interpolation(cart, start, end, r_tolerance_deg):
        return False
    a = cart[start].xyz()
    b = cart[end].xyz()
    for i in range(start + 1, end):
        if _point_to_line_distance(cart[i].xyz(), a, b) > tolerance_mm:
            return False
    return True


_ARC_THROUGH_MIN_CHORD_MM = 15.0
_ARC_MAX_LEG_IMBALANCE = 4.0
_ARC_VERTICAL_IMBALANCE_ZSPAN_MM = 40.0
_ARC_MIN_THROUGH_FRACTION = 0.18
_ARC_MAX_THROUGH_FRACTION = 0.82
_ARC_SPLIT_IMPROVEMENT_RATIO = 0.75
_ARC_SPLIT_MIN_IMPROVEMENT_MM = 0.25


def _line_fit_error(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    r_tolerance_deg: float,
) -> float:
    if not _r_matches_endpoint_interpolation(cart, start, end, r_tolerance_deg):
        return math.inf
    a = cart[start].xyz()
    b = cart[end].xyz()
    return max(
        (_point_to_line_distance(cart[i].xyz(), a, b) for i in range(start + 1, end)),
        default=0.0,
    )


def _best_command_arc_fit_error(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_mm: float,
    max_radius_mm: float,
    r_tolerance_deg: float,
    *,
    min_chord_mm: float = _ARC_THROUGH_MIN_CHORD_MM,
) -> float:
    if end - start + 1 < 3:
        return math.inf

    start_xyz = cart[start].xyz()
    end_xyz = cart[end].xyz()
    z_values = [float(cart[i].z) for i in range(start, end + 1)]
    z_span = max(z_values) - min(z_values)
    best = math.inf
    for through in range(start + 1, end):
        through_xyz = cart[through].xyz()
        if float(np.linalg.norm(through_xyz - start_xyz)) < min_chord_mm:
            continue
        if float(np.linalg.norm(through_xyz - end_xyz)) < min_chord_mm:
            continue

        arc = _command_arc_from_points(start_xyz, through_xyz, end_xyz)
        if arc is None:
            continue
        if arc.radius > max_radius_mm:
            continue
        if arc.leg_ratio < (1.0 / _ARC_MAX_LEG_IMBALANCE) and z_span > _ARC_VERTICAL_IMBALANCE_ZSPAN_MM:
            continue
        if not (_ARC_MIN_THROUGH_FRACTION <= arc.through_fraction <= _ARC_MAX_THROUGH_FRACTION):
            continue
        if not _r_matches_arc_interpolation(cart, start, through, end, arc.through_fraction, r_tolerance_deg):
            continue

        max_error = 0.0
        fractions: List[float] = []
        valid = True
        for point_idx in range(start + 1, end):
            distance, fraction = _point_to_command_arc_distance(cart[point_idx].xyz(), arc)
            if fraction is None or distance > tolerance_mm:
                valid = False
                break
            max_error = max(max_error, distance)
            fractions.append(fraction)
        if not valid:
            continue
        if any(b < a - 0.05 for a, b in zip(fractions, fractions[1:])):
            continue
        best = min(best, max_error)
    return best


def _better_as_split(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    full_arc_error: float,
    tolerance_mm: float,
    max_radius_mm: float,
    r_tolerance_deg: float,
) -> bool:
    """Would this span be more honest as two primitives than one Arc?

    A single command-shaped arc can numerically fit a span that is visually a
    straight run connected to a curved run.  That is legal geometry, but it is
    not the motion intent we want to send to the MG400.  If a two-piece
    Line/Arc or Arc/Line split reduces the worst fit error clearly, reject the
    long Arc so the greedy scan can form the smaller primitives.
    """
    if end - start + 1 < 5 or not math.isfinite(full_arc_error):
        return False

    split_limit = max(
        0.0,
        full_arc_error * _ARC_SPLIT_IMPROVEMENT_RATIO,
        full_arc_error - _ARC_SPLIT_MIN_IMPROVEMENT_MM,
    )
    for split in range(start + 2, end - 1):
        left_line = _line_fit_error(cart, start, split, r_tolerance_deg)
        right_line = _line_fit_error(cart, split, end, r_tolerance_deg)
        left_arc = _best_command_arc_fit_error(
            cart,
            start,
            split,
            tolerance_mm,
            max_radius_mm,
            r_tolerance_deg,
        )
        right_arc = _best_command_arc_fit_error(
            cart,
            split,
            end,
            tolerance_mm,
            max_radius_mm,
            r_tolerance_deg,
        )

        split_error = min(
            max(left_line, right_arc),
            max(left_arc, right_line),
            max(left_line, right_line),
        )
        if split_error < split_limit:
            return True
    return False


def _select_arc_through(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_mm: float,
    max_radius_mm: float,
    r_tolerance_deg: float,
    min_chord_mm: float = _ARC_THROUGH_MIN_CHORD_MM,
) -> Optional[int]:
    """Pick the intermediate frame best suited as the Arc through-point.

    The MG400 controller defines the arc with three Cartesian poses:
    current (= start), ``through_xyzr``, and ``target_xyzr``.  When the
    through-point sits very close to either endpoint, the implied circle
    is ill-conditioned and small numerical errors produce large Z swings
    during execution.  Picking the frame at the geometric midpoint of the
    raw chord (rather than the midpoint of the index range) removes that
    failure mode for unevenly-sampled trajectories.

    "Best" = maximum perpendicular distance from the chord ``start → end``
    (the bulgiest point of the arc), with a minimum chord-length
    separation from both endpoints so the three points are non-degenerate.
    Returns ``None`` if no candidate satisfies the separation rule —
    callers should reject the arc and fall back to a simpler primitive.
    """
    z_values = [float(cart[i].z) for i in range(start, end + 1)]
    z_span = max(z_values) - min(z_values)
    best_idx: Optional[int] = None
    best_score = math.inf
    for i in range(start + 1, end):
        score = _score_arc_through_candidate(
            cart, start, i, end,
            tolerance_mm=tolerance_mm,
            max_radius_mm=max_radius_mm,
            r_tolerance_deg=r_tolerance_deg,
            min_chord_mm=min_chord_mm,
            z_span=z_span,
        )
        if score is None:
            continue
        if score < best_score:
            best_score = score
            best_idx = i
    return best_idx


def _score_arc_through_candidate(
    cart: Sequence[CartesianPoint],
    start: int,
    through: int,
    end: int,
    *,
    tolerance_mm: float,
    max_radius_mm: float,
    r_tolerance_deg: float,
    min_chord_mm: float,
    z_span: float,
) -> Optional[float]:
    """Score one through-point candidate, or return ``None`` to reject.

    Rejects in priority order — first failure wins so we skip the
    expensive arc-fit work when an early geometric test rules the
    candidate out:

      1. Chord separation from either endpoint < ``min_chord_mm``.
      2. Three points collinear / arc radius > ``max_radius_mm`` /
         leg imbalance with large Z span / through-fraction outside
         ``[_ARC_MIN_, _ARC_MAX_]``.
      3. R/yaw doesn't follow the arc interpolation within
         ``r_tolerance_deg``.
      4. Any intermediate point sits more than ``tolerance_mm`` from
         the fitted arc, or the per-point progress fractions go
         non-monotonic.
      5. Splitting into two sub-arcs would fit substantially better.

    The accepted score blends the worst per-point error with two
    soft penalties (leg-balance, sweep > π) so a slightly-worse but
    geometrically cleaner arc beats a borderline 180° one.
    """
    start_xyz = cart[start].xyz()
    end_xyz = cart[end].xyz()
    through_xyz = cart[through].xyz()
    if float(np.linalg.norm(through_xyz - start_xyz)) < min_chord_mm:
        return None
    if float(np.linalg.norm(through_xyz - end_xyz)) < min_chord_mm:
        return None

    arc = _command_arc_from_points(start_xyz, through_xyz, end_xyz)
    if arc is None or arc.radius > max_radius_mm:
        return None
    if arc.leg_ratio < (1.0 / _ARC_MAX_LEG_IMBALANCE) and z_span > _ARC_VERTICAL_IMBALANCE_ZSPAN_MM:
        return None
    if not (_ARC_MIN_THROUGH_FRACTION <= arc.through_fraction <= _ARC_MAX_THROUGH_FRACTION):
        return None
    if not _r_matches_arc_interpolation(cart, start, through, end, arc.through_fraction, r_tolerance_deg):
        return None

    max_error = 0.0
    fractions: List[float] = []
    for j in range(start + 1, end):
        distance, fraction = _point_to_command_arc_distance(cart[j].xyz(), arc)
        if fraction is None or distance > tolerance_mm:
            return None
        max_error = max(max_error, distance)
        fractions.append(fraction)
    if any(b < a - 0.05 for a, b in zip(fractions, fractions[1:])):
        return None
    if _better_as_split(
        cart, start, end,
        full_arc_error=max_error,
        tolerance_mm=tolerance_mm,
        max_radius_mm=max_radius_mm,
        r_tolerance_deg=r_tolerance_deg,
    ):
        return None

    balance_penalty = (1.0 - arc.leg_ratio) * 0.25
    sweep_penalty = max(0.0, arc.sweep_rad - math.pi) * 0.1
    return max_error + balance_penalty + sweep_penalty


def _is_arc_segment(
    cart: Sequence[CartesianPoint],
    start: int,
    end: int,
    tolerance_mm: float,
    max_radius_mm: float,
    r_tolerance_deg: float,
) -> Optional[int]:
    """Do points[start..end] lie on a circular arc within ``tolerance_mm``?

    Uses a least-squares fit on every point in the span (not just three
    samples) so a noisy or near-symmetric arc still gets a stable circle.

    Returns the absolute through-point index for the Arc command, or None
    if the span isn't a valid arc.  Reasons for rejection:

    * < 3 points (need three for a circle)
    * degenerate fit (collinear / rank-deficient)
    * fitted radius > ``max_radius_mm`` — at that radius the arc is
      effectively straight; LINE is the more honest classification
    * any intermediate point deviates more than ``tolerance_mm`` from
      the fitted circle in 3D
    * no intermediate frame satisfies the through-point chord-separation
      rule (would yield a degenerate 3-point arc command)

    The through-point is the frame with maximum perpendicular distance
    from the chord, not the midpoint of the index range — uneven Unity
    sampling otherwise produces near-collinear (start, through) pairs
    that ball-up the controller's arc planner.
    """
    n = end - start + 1
    if n < 3:
        return None
    points = [cart[i].xyz() for i in range(start, end + 1)]
    center, radius, normal = _fit_circle_least_squares(points)
    if center is None or radius is None or normal is None:
        return None
    if radius > max_radius_mm:
        return None

    for i in range(start + 1, end):
        if _point_to_circle_distance(cart[i].xyz(), center, radius, normal) > tolerance_mm:
            return None

    return _select_arc_through(
        cart,
        start,
        end,
        tolerance_mm=tolerance_mm,
        max_radius_mm=max_radius_mm,
        r_tolerance_deg=r_tolerance_deg,
    )


# ── Helpers ─────────────────────────────────────────────────────────────

def _longest_line(
    cart: Sequence[CartesianPoint],
    start: int,
    n: int,
    tol_mm: float,
    r_tol_deg: float,
) -> int:
    """Longest LINE span starting at ``start`` — returns end index (≥ start+1)."""
    for cand in range(n - 1, start, -1):
        if _is_line_segment(cart, start, cand, tol_mm, r_tol_deg):
            return cand
    return start + 1  # 2-point fallback (always trivially collinear)


def _longest_arc(
    cart: Sequence[CartesianPoint],
    start: int,
    n: int,
    tol_mm: float,
    max_radius_mm: float,
    r_tol_deg: float,
    min_points: int,
) -> Tuple[Optional[int], Optional[int]]:
    """Longest ARC span starting at ``start`` — returns (end_idx, through_idx)."""
    for cand in range(n - 1, start + min_points - 2, -1):
        through = _is_arc_segment(cart, start, cand, tol_mm, max_radius_mm, r_tol_deg)
        if through is not None:
            return cand, through
    return None, None


def _joints_to_cart(
    joints: Tuple[float, float, float, float], fk_fn: FKFunction,
) -> CartesianPoint:
    x, y, z, r = fk_fn(joints[0], joints[1], joints[2], joints[3])
    return CartesianPoint(x=x, y=y, z=z, r=r)


# ── Main classifier ─────────────────────────────────────────────────────

def classify_segments(
    joint_points: Sequence[Tuple[float, float, float, float]],
    fk_fn: FKFunction,
    line_tol_mm: float = 2.0,
    arc_tol_mm: float = 3.0,
    max_arc_radius_mm: float = 10000.0,
    r_tol_deg: float = 10.0,
    min_points_for_arc: int = 3,
    enable_arc: bool = True,
) -> List[Segment]:
    """Classify a simplified joint-space waypoint sequence into typed segments.

    Parameters
    ----------
    joint_points
        Sequence of (j1, j2, j3, j4) in degrees — already simplified by RDP.
    fk_fn
        Forward kinematics: (j1,j2,j3,j4) → (x_mm, y_mm, z_mm, r_deg).
    line_tol_mm
        Max XYZ deviation from the chord between segment endpoints for a
        span to qualify as LINE.
    arc_tol_mm
        Max 3D deviation from the least-squares-fitted circle for a span
        to qualify as ARC.
    max_arc_radius_mm
        Cap on the fitted arc radius.  Above this radius the arc is
        effectively straight and LINE is the better fit.  Defaults to
        10 000 mm — well outside the MG400's reach so any genuine arc on
        the working envelope passes, but a near-straight curve doesn't
        sneak in as a giant-radius Arc.
    r_tol_deg
        Max permitted R/yaw deviation from endpoint interpolation.  Small
        hand wobble can still collapse to MovL/Arc, while meaningful non-linear
        wrist motion falls back to GENERAL.
    min_points_for_arc
        Minimum waypoint count to attempt arc fitting (need ≥ 3).
    enable_arc
        Set False to disable Arc emission (e.g. for backends that don't
        implement it); the classifier falls back to LINE / GENERAL only.

    Returns
    -------
    List of Segment, end-to-end.  Adjacent segments share their boundary
    waypoint (segment N's ``end_idx`` == segment N+1's ``start_idx``) so
    the renderer can stream them without re-anchoring.

    Tie-break
    ---------
    When LINE and ARC cover the same number of waypoints LINE wins — a
    straight line is the limit of an arc with infinite radius, so when
    both fit equally well the simpler primitive is the honest choice.
    """
    n = len(joint_points)
    if n == 0:
        return []
    if n == 1:
        cp = _joints_to_cart(joint_points[0], fk_fn)
        return [Segment(SegmentType.GENERAL, 0, 0, cartesian_points=(cp,))]

    cart = [_joints_to_cart(jp, fk_fn) for jp in joint_points]
    segments: List[Segment] = []
    i = 0

    while i < n - 1:
        line_end = _longest_line(cart, i, n, line_tol_mm, r_tol_deg)

        arc_end: Optional[int] = None
        arc_through: Optional[int] = None
        if enable_arc and (n - i) >= min_points_for_arc:
            arc_end, arc_through = _longest_arc(
                cart, i, n, arc_tol_mm, max_arc_radius_mm, r_tol_deg, min_points_for_arc,
            )

        # Pick the longer span; ties go to LINE (simpler primitive).
        if arc_end is not None and arc_end > line_end:
            seg_type = SegmentType.ARC
            best_end = arc_end
            best_through = arc_through
        else:
            seg_type = SegmentType.LINE
            best_end = line_end
            best_through = None

        # 2-point LINE on truly irregular data is GENERAL — we couldn't
        # extend either primitive past one waypoint, so emit a JointMovJ
        # chunk to preserve fidelity instead of a degenerate MovL.
        if seg_type == SegmentType.LINE and best_end == i + 1:
            seg_type = SegmentType.GENERAL

        segments.append(Segment(
            type=seg_type,
            start_idx=i,
            end_idx=best_end,
            cartesian_points=tuple(cart[i:best_end + 1]),
            arc_through_idx=best_through,
        ))
        i = best_end

    return segments


# ── Summary ─────────────────────────────────────────────────────────────

def segment_summary(segments: List[Segment]) -> str:
    """Human-readable summary of classified segments."""
    counts = {t: 0 for t in SegmentType}
    total_points = 0
    for s in segments:
        counts[s.type] += 1
        total_points = max(total_points, s.end_idx + 1)

    total_commands = sum(
        1 if s.type in (SegmentType.LINE, SegmentType.ARC) else s.point_count - 1
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
