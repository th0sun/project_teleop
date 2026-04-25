#!/usr/bin/env python3
"""Demonstrate teach-and-repeat retiming for the MG400 demo path.

This is intentionally software-only.  It answers one question before touching
the real robot: can the recorded hand timing be preserved, and if not, how much
must it be stretched while keeping every taught waypoint?
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

_REPO = Path(__file__).resolve().parents[2]
for _pkg in (
    "src/robot_teaching_core",
    "src/dobot_mg400/mg400_protocol",
):
    _p = str(_REPO / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mg400_protocol.commands import joint_mov_j  # noqa: E402
from teaching_core.trajectory import (  # noqa: E402
    JointTimingLimits,
    TimedJointPoint,
    retime_joint_path,
    speed_percent_for_segment,
)

MG400_EFFECTIVE_SPEED_AT_100_DEG_S = 90.0
MIN_SPEEDJ = 15
STREAM_CP = 20
FINAL_CP = 0


def pick_place_demo() -> List[TimedJointPoint]:
    return _points([
        (0.00, (0.0, 0.0, 0.0, 0.0)),
        (0.50, (5.0, -3.0, 0.0, 0.0)),
        (1.00, (10.0, -7.0, 0.0, 0.0)),
        (1.50, (15.0, -11.0, 0.0, 0.0)),
        (2.00, (20.0, -15.0, 0.0, 0.0)),
        (2.50, (20.0, -8.0, 0.0, 0.0)),
        (3.00, (20.0, 0.0, 0.0, 0.0)),
    ])


def too_fast_hand_demo() -> List[TimedJointPoint]:
    return _points([
        (0.00, (0.0, 0.0, 0.0, 0.0)),
        (0.05, (20.0, -15.0, 0.0, 0.0)),
        (0.10, (20.0, 0.0, 0.0, 0.0)),
    ])


def _points(rows: Iterable[Tuple[float, Sequence[float]]]) -> List[TimedJointPoint]:
    return [TimedJointPoint(t, tuple(q)) for t, q in rows]


def _print_case(name: str, points: List[TimedJointPoint]) -> None:
    limits = JointTimingLimits(
        max_velocity=(MG400_EFFECTIVE_SPEED_AT_100_DEG_S,) * 4,
    )
    result = retime_joint_path(points, limits)
    print(f"\n=== {name} ===")
    print(f"waypoints in/out: {len(points)} / {len(result.points)}")
    print(f"original duration: {result.original_duration_s:.3f}s")
    print(f"retimed duration:  {result.retimed_duration_s:.3f}s")
    print(f"time scale:        x{result.time_scale:.3f}")
    print(f"original feasible: {result.is_original_timing_feasible}")
    print("task shell: vacuum_on -> motion path -> wait 300ms -> vacuum_off")
    print()
    print("idx  orig_dt  play_dt  max_req_deg_s  SpeedJ  CP  command")
    print("---  -------  -------  -------------  ------  --  -------")
    for segment in result.segments:
        prev = result.points[segment.index - 1]
        curr = result.points[segment.index]
        speed_j = speed_percent_for_segment(
            prev.position,
            curr.position,
            segment.retimed_dt_s,
            (MG400_EFFECTIVE_SPEED_AT_100_DEG_S,) * 4,
            min_percent=MIN_SPEEDJ,
        )
        cp = FINAL_CP if segment.index == len(result.points) - 1 else STREAM_CP
        command = joint_mov_j(curr.position, speed_j=speed_j, acc_j=100, cp=cp).render()
        print(
            f"{segment.index:>3}  "
            f"{segment.original_dt_s:>7.3f}  "
            f"{segment.retimed_dt_s:>7.3f}  "
            f"{segment.max_required_velocity:>13.1f}  "
            f"{speed_j:>6}  "
            f"{cp:>2}  "
            f"{command}"
        )


def main() -> None:
    print("MG400 teach-repeat retiming demo")
    print(f"assumed SpeedJ=100 effective joint speed: {MG400_EFFECTIVE_SPEED_AT_100_DEG_S:.1f} deg/s")
    _print_case("A. demo pick-and-place timing is preserved", pick_place_demo())
    _print_case("B. too-fast hand motion is stretched, not decimated", too_fast_hand_demo())


if __name__ == "__main__":
    main()
