"""Path simplification for dense teach-and-repeat recordings.

The Unity recorder samples joint state at ~20 FPS while the user demonstrates
a motion, which produces hundreds of waypoints for even a short path.  The
MG400 controller blends linearly between commanded waypoints, so a recorded
straight line of fifty points generates fifty queued JointMovJ commands —
each one re-running the controller's blend planner — and the motion queue
backs up faster than playback can drain it.

This module implements a Ramer-Douglas-Peucker (RDP) simplification in the
robot's joint space, with the time-parameterised linear interpolation
matching the controller's actual blend behaviour.  Intermediate waypoints
that fall within ``tolerance`` of the lerp between surrounding kept
waypoints are dropped; original waypoints are returned verbatim with no
synthetic interpolation.

The result preserves the recorded path shape while collapsing dense runs
of near-collinear samples into the few critical points that actually
change the motion.  A pure straight line collapses to its endpoints; a
sharp corner keeps endpoints + corner; a smooth curve keeps a handful of
points proportional to its curvature.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from teaching_core.trajectory.retiming import TimedJointPoint


def simplify_joint_path_rdp(
    points: Sequence[TimedJointPoint],
    tolerance: float,
) -> Tuple[TimedJointPoint, ...]:
    """Ramer-Douglas-Peucker simplification for timed joint paths.

    Parameters
    ----------
    points
        Original recorded waypoints, sorted by ``time_s``.  All points must
        share the same joint dimension and unit system.
    tolerance
        Maximum permitted joint deviation between an intermediate point and
        the time-lerp of its surrounding kept points, in the same unit as
        ``position`` (degrees for the MG400 path).  ``tolerance <= 0``
        disables simplification and returns the input unchanged.

    Returns
    -------
    Tuple of kept points in original order.  The first and last point are
    always preserved.  Inputs of length ``<= 2`` are returned unchanged.

    Notes
    -----
    Distance metric is the max-joint absolute deviation from the
    time-parameterised linear interpolation between the segment endpoints,
    which mirrors the MG400 controller's blend between two commanded
    waypoints.  ``tolerance`` therefore translates directly into worst-case
    path deviation during playback — set it to the smallest joint motion
    the operator considers visually meaningful.
    """
    n = len(points)
    if tolerance <= 0 or n <= 2:
        return tuple(points)

    keep = [False] * n
    keep[0] = True
    keep[-1] = True
    _rdp_recurse(points, 0, n - 1, float(tolerance), keep)
    return tuple(p for p, k in zip(points, keep) if k)


def _rdp_recurse(
    points: Sequence[TimedJointPoint],
    lo: int,
    hi: int,
    tolerance: float,
    keep: List[bool],
) -> None:
    if hi - lo < 2:
        return

    a = points[lo]
    b = points[hi]
    span_t = b.time_s - a.time_s

    max_err = 0.0
    max_idx = -1
    a_pos = a.position
    b_pos = b.position
    n_axes = len(a_pos)

    for i in range(lo + 1, hi):
        p = points[i]
        if span_t > 1e-12:
            ratio = (p.time_s - a.time_s) / span_t
        else:
            ratio = 0.5

        err = 0.0
        for ax in range(n_axes):
            expected = a_pos[ax] + ratio * (b_pos[ax] - a_pos[ax])
            d = abs(p.position[ax] - expected)
            if d > err:
                err = d

        if err > max_err:
            max_err = err
            max_idx = i

    if max_err > tolerance and max_idx > 0:
        keep[max_idx] = True
        _rdp_recurse(points, lo, max_idx, tolerance, keep)
        _rdp_recurse(points, max_idx, hi, tolerance, keep)
