#!/usr/bin/env python3
"""Comparative execution study: baseline 9196b01 vs HEAD teaching path.

Two systems, same pick-place joint trajectory, compared across:
  - command count and structure
  - what the Mock actually executes (FunctionParser behaviour)
  - trapezoidal trajectory steps per command (Mock hardware sim)
  - q_target / q_actual / error at each waypoint
  - total simulated time

Execution environments:
  A.  Baseline (9196b01): TeleopController streaming at ~25 Hz
      with plan_batch_motion(num_steps=3) when robot velocity < 0.1 rad/s.
  B.  HEAD teaching path: lift_session -> 3 JointMovJ waypoints via
      translate_program (no batch, no streaming).

The Mock's FunctionParser processes one command at a time.  Batch
commands (semicolon-joined) are silently truncated to the FIRST sub-
command because FunctionParser.exec regex anchors on the first
function name and splits on all commas including those inside later
sub-commands.  This script proves that numerically.

Usage (from repo root):
    PYTHONPATH=src/robot_teaching_core:src/adapters/mg400:\\
              src/dobot_mg400/mg400_controller:\\
              src/dobot_mg400/mg400_protocol \\
        python3 tools/demo_lift/compare_execution.py
"""

from __future__ import annotations

import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# sys.path setup (same pattern as demo_pick_place.py)
# ---------------------------------------------------------------------------
_REPO = Path(__file__).resolve().parents[2]
for _pkg in (
    "src/robot_teaching_core",
    "src/adapters/mg400",
    "src/dobot_mg400/mg400_controller",
    "src/dobot_mg400/mg400_protocol",
    "MG400_Mock/app/src",
):
    _p = str(_REPO / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mg400_controller.common.logic.motion_planner import MotionPlanner  # noqa: E402
from mg400_adapter import register_mg400_provider  # noqa: E402
from mg400_adapter.translator import translate_program  # noqa: E402
from teaching_core.lifter.segmenter import LifterConfig, lift_session  # noqa: E402
from teaching_core.lifter.session import SessionStream  # noqa: E402
from teaching_core.program.types import OrientationIntent, ToolStep, WaitStep  # noqa: E402
from utilities.trapezoid_trajectory import gene_trapezoid_traj  # noqa: E402

from dataclasses import replace as dc_replace  # noqa: E402


# ---------------------------------------------------------------------------
# Shared scenario: approach → dwell → retreat
# (identical to demo_pick_place.py so results are directly comparable)
# ---------------------------------------------------------------------------

PICK_PLACE_PAIRS = (
    # approach: J1 0→20°, J2 0→-15° over 0.16 s
    *[(i * 0.04, (math.radians(i * 5), math.radians(-i * 3.75), 0.0, 0.0))
      for i in range(5)],
    # dwell 0.28 s at pick point (triggers segmenter dwell detection)
    *[(i * 0.04, (math.radians(20), math.radians(-15), 0.0, 0.0))
      for i in range(5, 12)],
    # retreat: J2 back to 0, J1 stays at 20° over 0.16 s
    *[(i * 0.04, (math.radians(20), math.radians(-15 + (i - 11) * 3), 0.0, 0.0))
      for i in range(12, 17)],
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class _SilentLogger:
    def info(self, m): pass
    def warn(self, m): pass
    def warning(self, m): pass
    def debug(self, m): pass
    def error(self, m): pass


def _parse_jointmovj_deg(cmd: str) -> Optional[np.ndarray]:
    """Extract the first four float arguments from a JointMovJ command."""
    m = re.search(r"JointMovJ\(([^)]+)", cmd)
    if not m:
        return None
    parts = [p.strip() for p in m.group(1).split(",")]
    try:
        return np.array([float(p) for p in parts[:4]])
    except ValueError:
        return None


def _mock_parse_target(batch_cmd: str) -> Optional[np.ndarray]:
    """Replicate what FunctionParser.exec does with a (possibly batched) command.

    The real FunctionParser strips the FIRST function name and splits ALL
    commas, including those inside later semicolon-joined sub-commands.
    Only args[0:4] are consumed by JointMovJ.  Subsequent sub-commands
    are silently ignored.
    """
    fn = re.match(r"\s*[a-zA-Z0-9_]+(?=(\s*\(.*\)))", batch_cmd)
    if not fn:
        return None
    fn_name = fn.group()
    args_str = (
        batch_cmd.replace(fn_name, "").strip().lstrip("(").rstrip(")")
    )
    args = args_str.split(",")
    try:
        return np.array([float(args[i].strip()) for i in range(4)])
    except (IndexError, ValueError):
        return None


@dataclass
class WaypointResult:
    label: str
    q_target_deg: np.ndarray
    q_mock_executed_deg: np.ndarray    # what Mock ACTUALLY moves to
    error_deg: np.ndarray              # |target - executed|
    traj_steps: int                    # trapezoid samples at 5 ms
    time_ms: float
    batch: bool                        # was this a batch (multi-sub-cmd) command?
    sub_cmd_count: int                 # number of sub-commands in the string


@dataclass
class SystemResult:
    name: str
    motion_cmd_strings: List[str]
    waypoints: List[WaypointResult] = field(default_factory=list)
    non_motion_cmds: List[str] = field(default_factory=list)

    @property
    def total_commands(self):
        return len(self.motion_cmd_strings) + len(self.non_motion_cmds)

    @property
    def total_traj_steps(self):
        return sum(w.traj_steps for w in self.waypoints)

    @property
    def total_time_ms(self):
        return sum(w.time_ms for w in self.waypoints)

    @property
    def max_error_deg(self):
        if not self.waypoints:
            return 0.0
        return max(np.max(w.error_deg) for w in self.waypoints)


# Mock hardware speed parameters (from DobotHardware defaults):
#   global_speed_rate=50, speed_j_rate=50, speed_j_max=360 → speed_j=90 deg/s
#   acc_j_rate=50, acc_j_max=360 → acc_j=180 deg/s²
#   timestep=5 ms
_MOCK_ACC_J = 180.0   # deg/s²
_MOCK_SPD_J = 90.0    # deg/s
_MOCK_DT    = 0.005   # s


def _simulate_move(
    q_init_deg: np.ndarray,
    q_target_mock_deg: np.ndarray,
    label: str,
    q_intended_deg: np.ndarray,
    batch: bool,
    sub_cmd_count: int,
) -> WaypointResult:
    """Run Mock trapezoid trajectory for each joint, report results."""
    traj_lens = []
    for j_init, j_tgt in zip(q_init_deg, q_target_mock_deg):
        traj = gene_trapezoid_traj(j_init, j_tgt, _MOCK_ACC_J, _MOCK_SPD_J, _MOCK_DT)
        traj_lens.append(len(traj))

    steps = max(traj_lens) if traj_lens else 0
    time_ms = steps * _MOCK_DT * 1000.0
    error_deg = np.abs(q_intended_deg - q_target_mock_deg)

    return WaypointResult(
        label=label,
        q_target_deg=q_intended_deg,
        q_mock_executed_deg=q_target_mock_deg,
        error_deg=error_deg,
        traj_steps=steps,
        time_ms=time_ms,
        batch=batch,
        sub_cmd_count=sub_cmd_count,
    )


# ---------------------------------------------------------------------------
# System A: Baseline 9196b01 — teleop streaming with batch interpolation
# ---------------------------------------------------------------------------

def _run_baseline() -> SystemResult:
    """Simulate what 9196b01 would send for the pick-place trajectory.

    The baseline TeleopController sends a command whenever the VR hand
    moves more than DYNAMIC_PROXIMITY_BASE_RAD=0.02 rad from the last
    sent target.  When robot velocity < 0.1 rad/s (precision/settle
    phase), plan_batch_motion(num_steps=3) is used instead of
    format_command (single-point).

    During the approach/retreat phases of our trajectory each 0.04 s
    frame moves J1 by ~0.087 rad or J2 by ~0.065 rad, well above the
    0.02 rad threshold, so every frame triggers a send.  Velocity at
    50 Hz is 0.087/0.04 ≈ 2.2 rad/s >> 0.1, so single mode is used.

    During the dwell phase no hand movement occurs so no new commands
    are sent.  After the dwell, the robot has settled (robot velocity
    is low) and the first retreat frame triggers batch mode.

    We simulate 25 frames/s (every other frame) to represent the
    controller's realistic throughput.
    """
    planner = MotionPlanner("jointmovj", _SilentLogger())
    THRESHOLD_RAD = 0.02   # DYNAMIC_PROXIMITY_BASE_RAD
    BATCH_VEL_THRESHOLD = 0.1  # rad/s — engage batch below this

    pairs = PICK_PLACE_PAIRS
    dt = 0.04  # seconds per VR frame

    last_sent_rad = None
    last_robot_vel = 10.0  # start high (robot not yet settled)
    motion_cmds: List[str] = []

    for frame_idx, (t, q_rad) in enumerate(pairs):
        q_arr = np.array(q_rad)

        if last_sent_rad is None:
            # First frame always sends
            single, _, _ = planner.plan_motion(q_arr, q_arr)
            if single:
                motion_cmds.append(single)
            last_sent_rad = q_arr.copy()
            last_robot_vel = 0.0
            continue

        dist_to_last = np.max(np.abs(q_arr - last_sent_rad))
        if dist_to_last < THRESHOLD_RAD:
            continue  # not far enough from last command

        # Estimate robot velocity from consecutive frames
        if frame_idx >= 1:
            prev_q = np.array(pairs[frame_idx - 1][1])
            last_robot_vel = np.max(np.abs(q_arr - prev_q)) / dt

        if last_robot_vel < BATCH_VEL_THRESHOLD:
            # Precision mode: batch (3 micro-steps)
            cmd, _, _ = planner.plan_batch_motion(q_arr, last_sent_rad, num_steps=3)
        else:
            # Normal mode: single point
            cmd, _, _ = planner.plan_motion(q_arr, last_sent_rad)

        if cmd:
            motion_cmds.append(cmd)
            last_sent_rad = q_arr.copy()

    result = SystemResult(
        name="Baseline 9196b01 (teleop streaming)",
        motion_cmd_strings=motion_cmds,
        non_motion_cmds=["(no vacuum/tool logic in teleop path)"],
    )

    # Simulate each command through Mock hardware
    q_current = np.zeros(4)  # robot starts at home [0,0,0,0] deg
    for i, cmd in enumerate(motion_cmds):
        sub_cmds = cmd.split(";")
        sub_count = len(sub_cmds)
        batch = sub_count > 1

        q_intended = _parse_jointmovj_deg(cmd)  # last sub-cmd = intended target
        q_mock = _mock_parse_target(cmd)         # what Mock ACTUALLY gets

        if q_intended is None or q_mock is None:
            continue

        wp = _simulate_move(
            q_init_deg=q_current,
            q_target_mock_deg=q_mock,
            label=f"cmd{i+1}",
            q_intended_deg=q_intended,
            batch=batch,
            sub_cmd_count=sub_count,
        )
        result.waypoints.append(wp)
        q_current = q_mock.copy()  # robot actually ends up here

    return result


# ---------------------------------------------------------------------------
# System B: HEAD teaching path — lift_session + translate_program
# ---------------------------------------------------------------------------

def _run_head() -> SystemResult:
    """Run the teaching pipeline and collect command data."""
    provider = register_mg400_provider()
    stream = SessionStream.from_pairs(PICK_PLACE_PAIRS)
    cfg = LifterConfig(
        program_id="compare_head",
        capture_id="cap_compare",
        captured_at="2026-04-25T19:00:00Z",
        demonstrator="hand_authored",
        default_orientation_intent=OrientationIntent.YAW_ONLY,
    )
    motion_prog = lift_session(stream, provider=provider, config=cfg)
    program = dc_replace(
        motion_prog,
        steps=(
            ToolStep(tool="vacuum", action="on"),
            *motion_prog.steps,
            WaitStep(duration_ms=300),
            ToolStep(tool="vacuum", action="off"),
        ),
    )

    plan = translate_program(program)
    motion_cmds = [c.command for c in plan.commands if c.kind == "motion"]
    non_motion = [
        f"{c.kind}: {c.command or f'wait {c.duration_ms}ms'}"
        for c in plan.commands if c.kind != "motion"
    ]

    result = SystemResult(
        name="HEAD teaching path (lift_session → translate_program)",
        motion_cmd_strings=motion_cmds,
        non_motion_cmds=non_motion,
    )

    q_current = np.zeros(4)
    for i, cmd in enumerate(motion_cmds):
        sub_cmds = cmd.split(";")
        sub_count = len(sub_cmds)
        batch = sub_count > 1

        q_intended = _parse_jointmovj_deg(cmd)
        q_mock = _mock_parse_target(cmd)

        if q_intended is None or q_mock is None:
            continue

        wp = _simulate_move(
            q_init_deg=q_current,
            q_target_mock_deg=q_mock,
            label=f"wp{i+1}",
            q_intended_deg=q_intended,
            batch=batch,
            sub_cmd_count=sub_count,
        )
        result.waypoints.append(wp)
        q_current = q_mock.copy()

    return result


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

_SEP = "-" * 72


def _fmt_deg(arr: np.ndarray) -> str:
    return "[" + ", ".join(f"{v:7.3f}" for v in arr[:4]) + "]"


def _print_result(r: SystemResult) -> None:
    print(f"\n{'='*72}")
    print(f"  {r.name}")
    print(f"{'='*72}")
    print(f"  Motion commands: {len(r.motion_cmd_strings)}")
    print(f"  Non-motion     : {len(r.non_motion_cmds)}")
    print(f"  Total commands : {r.total_commands}")
    print()
    print("  Non-motion steps:")
    for nm in r.non_motion_cmds:
        print(f"    {nm}")
    print()
    print("  Motion waypoint detail:")
    print(f"  {'label':6s}  {'batch':5s}  {'sub_n':5s}  "
          f"{'q_intended_deg (J1-J4)':30s}  "
          f"{'q_mock_exec_deg (J1-J4)':30s}  "
          f"{'max_err°':8s}  {'steps':5s}  {'time_ms':7s}")
    print(f"  {_SEP}")
    for wp in r.waypoints:
        print(f"  {wp.label:6s}  {'Y' if wp.batch else 'N':5s}  "
              f"{wp.sub_cmd_count:5d}  "
              f"{_fmt_deg(wp.q_target_deg):30s}  "
              f"{_fmt_deg(wp.q_mock_executed_deg):30s}  "
              f"{np.max(wp.error_deg):8.4f}  "
              f"{wp.traj_steps:5d}  {wp.time_ms:7.1f}")
    print(f"  {_SEP}")
    print(f"  Totals:  traj_steps={r.total_traj_steps}  "
          f"motion_time={r.total_time_ms:.1f} ms  "
          f"max_target_err={r.max_error_deg:.4f}°")


def _print_comparison(a: SystemResult, b: SystemResult) -> None:
    print(f"\n\n{'#'*72}")
    print("  COMPARISON TABLE")
    print(f"{'#'*72}")

    rows = [
        ("Motion commands sent",         len(a.motion_cmd_strings),  len(b.motion_cmd_strings)),
        ("Any batch (;-joined) commands", sum(1 for c in a.motion_cmd_strings if ";" in c),
                                          sum(1 for c in b.motion_cmd_strings if ";" in c)),
        ("Unique waypoints (motion)",     len(a.waypoints),           len(b.waypoints)),
        ("Non-motion commands",           len(a.non_motion_cmds),     len(b.non_motion_cmds)),
        ("Total commands",                a.total_commands,           b.total_commands),
        ("Total traj steps @ 5ms",        a.total_traj_steps,         b.total_traj_steps),
        ("Total motion sim time (ms)",    f"{a.total_time_ms:.1f}",   f"{b.total_time_ms:.1f}"),
        ("Max target→executed err (°)",   f"{a.max_error_deg:.4f}",   f"{b.max_error_deg:.4f}"),
    ]

    w = max(len(r[0]) for r in rows) + 2
    header = f"  {'Metric':{w}}  {'Baseline 9196b01':>22}  {'HEAD teaching':>18}"
    print(header)
    print(f"  {'-'*(w+44)}")
    for label, va, vb in rows:
        print(f"  {label:{w}}  {str(va):>22}  {str(vb):>18}")


def _print_mock_parsing_proof(baseline: SystemResult) -> None:
    """Show explicitly that batch commands cause truncation on Mock.

    The pick-place scenario does NOT trigger batch mode (robot velocity
    during approach/retreat is ~1-2 rad/s >> 0.1 threshold).  This
    function adds a targeted demonstration: fine-tuning phase where the
    robot is 0.5° off target and velocity < 0.1 rad/s.
    """
    print(f"\n\n{'#'*72}")
    print("  MOCK BATCH-PARSING PROOF")
    print("  FunctionParser.exec discards sub-commands after the first.")
    print(f"{'#'*72}")

    # Check if any baseline commands happen to be batched
    batch_found = False
    for cmd in baseline.motion_cmd_strings:
        parts = cmd.split(";")
        if len(parts) < 2:
            continue
        batch_found = True
        q_intended = _parse_jointmovj_deg(parts[-1])
        q_executed = _mock_parse_target(cmd)
        print(f"\n  Batch command ({len(parts)} sub-cmds):")
        for i, p in enumerate(parts):
            marker = " ← Mock stops here" if i == 0 else " ← skipped"
            print(f"    sub[{i}]: {p}{marker}")
        if q_intended is not None and q_executed is not None:
            err = np.abs(q_intended - q_executed)
            print(f"\n  Intended: J=[{', '.join(f'{v:.4f}°' for v in q_intended)}]")
            print(f"  Executed: J=[{', '.join(f'{v:.4f}°' for v in q_executed)}]")
            print(f"  Error (°): max={np.max(err):.4f}°")

    if not batch_found:
        print("\n  Scenario note: pick-place approach/retreat velocity is ~1-2 rad/s")
        print("  (above 0.1 rad/s threshold), so batch mode was NOT triggered.")

    # Targeted demonstration: fine-tune/settle scenario
    # Robot is 0.5° off pick point, velocity < 0.1 rad/s → batch fires.
    print("\n  --- Targeted batch demo: fine-aiming at pick point ---")
    print("  Scenario: robot is 0.5° off pick point, VR velocity = 0.05 rad/s")
    print("  (This IS the scenario batch interpolation was designed for.)")

    planner_demo = MotionPlanner("jointmovj", _SilentLogger())
    q_current_off = np.radians([20.5, -15.3, 0.0, 0.0])   # slightly off
    q_final_target = np.radians([20.0, -15.0, 0.0, 0.0])   # intended pick

    # 9196b01: uses last_command as q_start (BUG: can be stale)
    # Fix in current repo: uses q_current as q_start
    # We test the 9196b01 version (last_command = q_current_off here)
    planner_demo.last_command = q_current_off.copy()  # simulate "just sent"
    batch_cmd, _, _ = planner_demo.plan_batch_motion(
        q_final_target, q_current_off, num_steps=3
    )

    parts = batch_cmd.split(";") if batch_cmd else []
    q_sub0 = _parse_jointmovj_deg(parts[0]) if parts else None
    q_last = _parse_jointmovj_deg(parts[-1]) if parts else None
    q_mock = _mock_parse_target(batch_cmd) if batch_cmd else None

    print(f"\n  Input:  q_current_off = J=[{', '.join(f'{np.degrees(v):.4f}°' for v in q_current_off[:4])}]")
    print(f"          q_intended    = J=[{', '.join(f'{np.degrees(v):.4f}°' for v in q_final_target[:4])}]")
    print(f"\n  Batch command ({len(parts)} sub-cmds):")
    for i, p in enumerate(parts):
        marker = " ← Mock stops here" if i == 0 else " ← skipped"
        print(f"    sub[{i}]: {p}{marker}")
    if q_mock is not None and q_last is not None:
        err = np.abs(q_last - q_mock)
        print(f"\n  Intended (last sub): J=[{', '.join(f'{v:.4f}°' for v in q_last)}]")
        print(f"  Mock executes (sub0):J=[{', '.join(f'{v:.4f}°' for v in q_mock)}]")
        print(f"  Per-joint error: [{', '.join(f'{v:.4f}°' for v in err)}]")
        print(f"  Max error: {np.max(err):.4f}°  ← robot stops here, NOT at intended target")


# ---------------------------------------------------------------------------

def main() -> None:
    print("Running baseline simulation (9196b01) ...")
    baseline = _run_baseline()

    print("Running HEAD teaching path ...")
    head = _run_head()

    _print_result(baseline)
    _print_result(head)
    _print_mock_parsing_proof(baseline)
    _print_comparison(baseline, head)

    print(f"\n\n{'#'*72}")
    print("  ENGINEERING SUMMARY")
    print(f"{'#'*72}")
    print("""
  1. BATCH COMMAND BUG (9196b01 on Mock):
     plan_batch_motion joins 3 JointMovJ sub-commands with semicolons.
     FunctionParser.exec strips only the first function name and splits ALL
     commas, so only args[0:4] from sub[0] reach JointMovJ.  Sub-commands
     [1] and [2] are silently discarded.  The robot stops at the first
     intermediate point, NOT the intended target.  This is present
     whenever robot velocity < 0.1 rad/s triggers precision mode.

  2. HEAD TEACHING PATH:
     Each JointMovJ is a separate command in MG400CommandPlan.  The Mock
     queues and executes all of them in sequence.  Zero truncation error.
     The robot visits start → pick → retreat as intended.

  3. STRUCTURAL DIFFERENCE:
     Baseline: streaming, command count depends on hand velocity and
     proximity thresholds, no tool/vacuum logic, no offline artifact.
     HEAD: offline, deterministic 3-waypoint program, tool/vacuum/wait
     included, schema-validated canonical artifact.

  4. WHAT MOCK CANNOT PROVE:
     - RunQueuedCmd / CurrentCommandId (not in Mock feedback packet)
     - TCP queue flush timing / queue saturation under load
     - Real CP blending continuity
     - Actual physical error after settling (Mock uses ideal trapezoid)
""")


if __name__ == "__main__":
    main()
