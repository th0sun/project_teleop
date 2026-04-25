#!/usr/bin/env python3
"""TCP comparative execution study: baseline 9196b01 vs HEAD teaching path.

Runs BOTH systems against the live MG400 Mock (Docker, 127.0.0.1).
Collects real TCP feedback: q_target, q_actual, robot_mode,
tool_vector_actual, timing at 8 ms intervals.

Prerequisites:
    docker compose -f MG400_Mock/docker-compose.yml -f - up -d <<EOF
    services:
      dobot:
        ports:
          - "29999:29999"
          - "30003:30003"
          - "30004:30004"
    EOF

Usage (from repo root):
    PYTHONPATH=src/robot_teaching_core:src/adapters/mg400:\\
              src/dobot_mg400/mg400_controller:\\
              src/dobot_mg400/mg400_protocol \\
        python3 tools/demo_lift/compare_tcp.py
"""

from __future__ import annotations

import math
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# sys.path
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

from mg400_adapter import register_mg400_provider  # noqa: E402
from mg400_adapter.translator import translate_program  # noqa: E402
from mg400_controller.common.logic.motion_planner import MotionPlanner  # noqa: E402
from tcp_interface.realtime_packet import RealtimePacketType  # noqa: E402
from teaching_core.lifter.segmenter import LifterConfig, lift_session  # noqa: E402
from teaching_core.lifter.session import SessionStream  # noqa: E402
from teaching_core.program.types import OrientationIntent, ToolStep, WaitStep  # noqa: E402

from dataclasses import replace as dc_replace  # noqa: E402

# ---------------------------------------------------------------------------
# Mock connection parameters
# ---------------------------------------------------------------------------
MOCK_IP = "127.0.0.1"
DASHBOARD_PORT = 29999
MOTION_PORT    = 30003
FEEDBACK_PORT  = 30004
PKT_SIZE       = np.dtype(RealtimePacketType).itemsize
FEEDBACK_HZ    = 125   # 8 ms period

MODE_ENABLE  = 5
MODE_RUNNING = 7

# ---------------------------------------------------------------------------
# Shared pick-place trajectory (identical to demo_pick_place / compare_execution)
# ---------------------------------------------------------------------------
PAIRS = (
    *[(i * 0.04, (math.radians(i * 5), math.radians(-i * 3.75), 0.0, 0.0))
      for i in range(5)],
    *[(i * 0.04, (math.radians(20), math.radians(-15), 0.0, 0.0))
      for i in range(5, 12)],
    *[(i * 0.04, (math.radians(20), math.radians(-15 + (i - 11) * 3), 0.0, 0.0))
      for i in range(12, 17)],
)


# ---------------------------------------------------------------------------
# Low-level Mock helpers
# ---------------------------------------------------------------------------

def _connect_dashboard() -> socket.socket:
    s = socket.create_connection((MOCK_IP, DASHBOARD_PORT), timeout=5)
    s.settimeout(5)
    return s


def _connect_motion() -> socket.socket:
    s = socket.create_connection((MOCK_IP, MOTION_PORT), timeout=5)
    s.settimeout(5)
    return s


def _connect_feedback() -> socket.socket:
    s = socket.create_connection((MOCK_IP, FEEDBACK_PORT), timeout=5)
    s.settimeout(5)
    return s


def _dashboard_cmd(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.05)
    try:
        return sock.recv(1024).decode().strip()
    except Exception:
        return ""


def _decode_digital_outputs(mask: int) -> List[int]:
    ports: List[int] = []
    bit = 0
    while mask:
        if mask & 1:
            ports.append(bit)
        mask >>= 1
        bit += 1
    return ports


def _read_packet(fb: socket.socket) -> dict:
    """Read one feedback packet and return named fields."""
    data = b""
    while len(data) < PKT_SIZE:
        chunk = fb.recv(PKT_SIZE - len(data))
        if not chunk:
            raise ConnectionError("feedback socket closed")
        data += chunk
    arr = np.frombuffer(data, dtype=RealtimePacketType)
    return {
        "robot_mode":         int(arr["robot_mode"][0]),
        "digital_outputs":    int(arr["digital_outputs"][0]),
        "q_actual_rad":       np.array(arr["q_actual"][0][:4]),
        "q_target_rad":       np.array(arr["q_target"][0][:4]),
        # MG400_Mock JointMovJ / feedback already operate in degrees.
        "q_actual_deg":       np.array(arr["q_actual"][0][:4]),
        "q_target_deg":       np.array(arr["q_target"][0][:4]),
        "tool_vector_actual": np.array(arr["tool_vector_actual"][0]),
        "qd_actual_rad":      np.array(arr["qd_actual"][0][:4]),
    }


def _drain_feedback(fb: socket.socket) -> None:
    """Drop buffered packets so the next read reflects the current command."""
    old_timeout = fb.gettimeout()
    try:
        fb.settimeout(0.001)
        while True:
            chunk = fb.recv(PKT_SIZE)
            if not chunk:
                break
    except TimeoutError:
        pass
    finally:
        fb.settimeout(old_timeout)


def _wait_for_motion_cycle(fb: socket.socket, timeout_s: float = 10.0) -> List[dict]:
    """Poll feedback through RUNNING -> ENABLE. Return all samples."""
    samples: List[dict] = []
    t0 = time.monotonic()
    saw_running = False
    while time.monotonic() - t0 < timeout_s:
        pkt = _read_packet(fb)
        pkt["elapsed_ms"] = (time.monotonic() - t0) * 1000.0
        samples.append(pkt)
        if pkt["robot_mode"] == MODE_RUNNING:
            saw_running = True
        elif saw_running and pkt["robot_mode"] == MODE_ENABLE:
            break
    return samples


def _reset_to_home(dash: socket.socket, motion: socket.socket, fb: socket.socket) -> None:
    """Send robot to [0,0,0,0] deg and wait until it arrives."""
    _drain_feedback(fb)
    motion.sendall(b"JointMovJ(0,0,0,0)\n")
    time.sleep(0.05)
    _wait_for_motion_cycle(fb, timeout_s=8.0)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class CommandRecord:
    label: str
    sent_cmd: str
    is_batch: bool
    sub_cmd_count: int
    q_target_intended_deg: np.ndarray  # what we WANTED to execute
    q_target_mock_deg: np.ndarray      # what Mock actually received (from feedback)
    q_actual_final_deg: np.ndarray     # q_actual when motion completed
    tool_vec_final: np.ndarray         # [x,y,z,rx,ry,rz] mm at completion
    error_deg: np.ndarray              # |q_target_mock - q_actual_final|
    samples: List[dict]                # all feedback packets during this move
    duration_ms: float


@dataclass
class RunResult:
    name: str
    records: List[CommandRecord] = field(default_factory=list)
    non_motion_log: List[str] = field(default_factory=list)
    non_motion_executed: int = 0
    non_motion_unsupported: int = 0

    @property
    def total_duration_ms(self):
        return sum(r.duration_ms for r in self.records)

    @property
    def max_error_deg(self):
        if not self.records:
            return 0.0
        return max(float(np.max(r.error_deg)) for r in self.records)

    @property
    def total_samples(self):
        return sum(len(r.samples) for r in self.records)


# ---------------------------------------------------------------------------
# Helpers: parse intended target from command string
# ---------------------------------------------------------------------------

def _parse_first_4(cmd: str):
    import re
    m = re.search(r"JointMovJ\(([^)]+)", cmd)
    if not m:
        return None
    parts = [p.strip() for p in m.group(1).split(",")]
    try:
        return np.array([float(p) for p in parts[:4]])
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# Run one command list against Mock TCP
# ---------------------------------------------------------------------------

def _execute_cmd_list(
    name: str,
    motion_cmds: List[str],
    non_motion_cmds: List[str],
    dash: socket.socket,
    motion: socket.socket,
    fb: socket.socket,
) -> RunResult:
    result = RunResult(name=name)

    for i, cmd in enumerate(motion_cmds):
        sub_cmds = cmd.split(";")
        is_batch = len(sub_cmds) > 1

        # Intended target = last sub-command (what caller actually wants)
        q_intended = _parse_first_4(sub_cmds[-1])
        if q_intended is None:
            continue

        # Send command to Mock
        _drain_feedback(fb)
        t_send = time.monotonic()
        motion.sendall((cmd + "\n").encode())
        time.sleep(0.02)   # let Mock queue the command

        # Poll through RUNNING -> ENABLE
        samples = _wait_for_motion_cycle(fb, timeout_s=15.0)
        duration_ms = (time.monotonic() - t_send) * 1000.0

        # Final state from last sample
        final = samples[-1]
        q_actual_final = final["q_actual_deg"].copy()
        tool_final = final["tool_vector_actual"].copy()

        # q_target seen in feedback DURING motion = what Mock actually moved to
        # Take the last sample where robot was still running (or first running)
        q_target_mock = final["q_target_deg"].copy()
        # Also check: first running sample often shows q_target
        running_samples = [s for s in samples if s["robot_mode"] == MODE_RUNNING]
        if running_samples:
            q_target_mock = running_samples[0]["q_target_deg"].copy()

        error_deg = np.abs(q_intended - q_actual_final)

        rec = CommandRecord(
            label=f"cmd{i+1}",
            sent_cmd=cmd[:80],   # truncate for display
            is_batch=is_batch,
            sub_cmd_count=len(sub_cmds),
            q_target_intended_deg=q_intended,
            q_target_mock_deg=q_target_mock,
            q_actual_final_deg=q_actual_final,
            tool_vec_final=tool_final,
            error_deg=error_deg,
            samples=samples,
            duration_ms=duration_ms,
        )
        result.records.append(rec)
        print(f"    {name[:20]:20s} {rec.label}: "
              f"done in {duration_ms:.0f}ms, "
              f"samples={len(samples)}, "
              f"max_err={float(np.max(error_deg)):.4f}°")

    for item in non_motion_cmds:
        if item.startswith("wait: wait "):
            duration_ms = int(item.removeprefix("wait: wait ").removesuffix("ms"))
            time.sleep(duration_ms / 1000.0)
            result.non_motion_executed += 1
            result.non_motion_log.append(f"{item} [slept]")
            continue

        if item.startswith("digital_output: "):
            cmd = item.removeprefix("digital_output: ").strip()
            _drain_feedback(fb)
            before = _read_packet(fb)["digital_outputs"]
            resp = _dashboard_cmd(dash, cmd)
            time.sleep(0.08)
            after = _read_packet(fb)["digital_outputs"]
            before_ports = _decode_digital_outputs(before)
            after_ports = _decode_digital_outputs(after)
            if before == after:
                result.non_motion_unsupported += 1
                result.non_motion_log.append(
                    f"{item} [resp={resp or 'NO_RESP'}; mock_digital_outputs_unchanged "
                    f"{before}->{after} ports {before_ports}->{after_ports}]"
                )
            else:
                result.non_motion_executed += 1
                result.non_motion_log.append(
                    f"{item} [resp={resp or 'NO_RESP'}; digital_outputs "
                    f"{before}->{after} ports {before_ports}->{after_ports}]"
                )
            continue

        result.non_motion_unsupported += 1
        result.non_motion_log.append(f"{item} [unhandled]")

    return result


# ---------------------------------------------------------------------------
# Build command lists
# ---------------------------------------------------------------------------

class _SilentLogger:
    def info(self, m): pass
    def warn(self, m): pass
    def warning(self, m): pass
    def debug(self, m): pass
    def error(self, m): pass


def _build_baseline_cmds() -> List[str]:
    planner = MotionPlanner("jointmovj", _SilentLogger())
    THRESHOLD_RAD = 0.02
    BATCH_VEL_THRESHOLD = 0.1
    pairs = PAIRS
    dt = 0.04

    last_sent_rad = None
    last_robot_vel = 10.0
    cmds: List[str] = []

    for idx, (t, q_rad) in enumerate(pairs):
        q_arr = np.array(q_rad)
        if last_sent_rad is None:
            single, _, _ = planner.plan_motion(q_arr, q_arr)
            if single:
                cmds.append(single)
            last_sent_rad = q_arr.copy()
            last_robot_vel = 0.0
            continue
        dist_to_last = np.max(np.abs(q_arr - last_sent_rad))
        if dist_to_last < THRESHOLD_RAD:
            continue
        if idx >= 1:
            prev_q = np.array(pairs[idx - 1][1])
            last_robot_vel = np.max(np.abs(q_arr - prev_q)) / dt
        if last_robot_vel < BATCH_VEL_THRESHOLD:
            cmd, _, _ = planner.plan_batch_motion(q_arr, last_sent_rad, num_steps=3)
        else:
            cmd, _, _ = planner.plan_motion(q_arr, last_sent_rad)
        if cmd:
            cmds.append(cmd)
            last_sent_rad = q_arr.copy()
    return cmds


def _build_head_cmds() -> Tuple[List[str], List[str]]:
    provider = register_mg400_provider()
    stream = SessionStream.from_pairs(PAIRS)
    cfg = LifterConfig(
        program_id="compare_tcp_head",
        capture_id="cap_tcp_001",
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
    return motion_cmds, non_motion


def _build_batch_demo_cmd() -> str:
    """Craft a batch command for the fine-aiming scenario."""
    planner = MotionPlanner("jointmovj", _SilentLogger())
    q_current_off = np.radians([20.5, -15.3, 0.0, 0.0])
    q_final_target = np.radians([20.0, -15.0, 0.0, 0.0])
    planner.last_command = q_current_off.copy()
    cmd, _, _ = planner.plan_batch_motion(q_final_target, q_current_off, num_steps=3)
    return cmd


# ---------------------------------------------------------------------------
# Report helpers
# ---------------------------------------------------------------------------

def _fmt4(arr: np.ndarray, precision: int = 3) -> str:
    return "[" + ", ".join(f"{v:+8.{precision}f}" for v in arr[:4]) + "]"


def _print_run(r: RunResult) -> None:
    print(f"\n{'='*72}")
    print(f"  {r.name}")
    print(f"{'='*72}")
    if r.non_motion_log:
        print("  Non-motion steps:")
        for nm in r.non_motion_log:
            print(f"    {nm}")
        print(f"  Executed non-motion: {r.non_motion_executed}")
        print(f"  Unsupported/non-verifiable on Mock: {r.non_motion_unsupported}")
        print()
    print(f"  Motion commands: {len(r.records)}")
    print(f"  Total duration:  {r.total_duration_ms:.0f} ms (TCP + motion)")
    print(f"  Total samples:   {r.total_samples} × 8ms")
    print()
    hdr = f"  {'label':6}  {'batch':5}  {'q_intended_deg':36}  {'q_actual_final_deg':36}  {'max_err°':8}  {'ms':7}  {'samples':7}"
    print(hdr)
    print(f"  {'-'*70}")
    for rec in r.records:
        print(f"  {rec.label:6}  {'Y' if rec.is_batch else 'N':5}  "
              f"{_fmt4(rec.q_target_intended_deg):36}  "
              f"{_fmt4(rec.q_actual_final_deg):36}  "
              f"{float(np.max(rec.error_deg)):8.4f}  "
              f"{rec.duration_ms:7.0f}  "
              f"{len(rec.samples):7}")
    print(f"  {'-'*70}")
    print(f"  Total: duration={r.total_duration_ms:.0f}ms  "
          f"max_target→actual={r.max_error_deg:.4f}°")

    # tool_vector_actual of last waypoint
    if r.records:
        tv = r.records[-1].tool_vec_final
        print(f"  TCP at end: [x={tv[0]:.1f}, y={tv[1]:.1f}, z={tv[2]:.1f}] mm  "
              f"rx={tv[3]:.1f}°")


def _print_batch_demo(rec: CommandRecord) -> None:
    print(f"\n{'#'*72}")
    print("  BATCH COMMAND DEMO (fine-aiming scenario, live TCP)")
    print(f"{'#'*72}")
    print(f"  Sent: {rec.sent_cmd}...")
    parts = (rec.sent_cmd + "...").split(";")
    print(f"  Sub-commands: {rec.sub_cmd_count}")
    print(f"  Intended target  (last sub): J4= {_fmt4(rec.q_target_intended_deg)}")
    print(f"  Mock q_target seen in fb:    J4= {_fmt4(rec.q_target_mock_deg)}")
    print(f"  q_actual at completion:      J4= {_fmt4(rec.q_actual_final_deg)}")
    print(f"  Per-joint error (°): [{', '.join(f'{v:.4f}' for v in rec.error_deg[:4])}]")
    print(f"  Max error: {float(np.max(rec.error_deg)):.4f}°")
    print(f"  Duration: {rec.duration_ms:.0f}ms  Samples: {len(rec.samples)}")


def _print_comparison(baseline: RunResult, head: RunResult) -> None:
    print(f"\n{'#'*72}")
    print("  COMPARISON TABLE")
    print(f"{'#'*72}")
    rows = [
        ("Motion commands",           len(baseline.records),       len(head.records)),
        ("Non-motion cmds",           len(baseline.non_motion_log), len(head.non_motion_log)),
        ("Executed non-motion",       baseline.non_motion_executed, head.non_motion_executed),
        ("Unsupported non-motion",    baseline.non_motion_unsupported, head.non_motion_unsupported),
        ("Total feedback samples",    baseline.total_samples,       head.total_samples),
        ("Total motion duration (ms)", f"{baseline.total_duration_ms:.0f}",
                                       f"{head.total_duration_ms:.0f}"),
        ("Max target→actual err (°)", f"{baseline.max_error_deg:.4f}",
                                       f"{head.max_error_deg:.4f}"),
        ("Tool path: TCP at end",
         f"[{baseline.records[-1].tool_vec_final[:3].round(1).tolist()}]" if baseline.records else "—",
         f"[{head.records[-1].tool_vec_final[:3].round(1).tolist()}]" if head.records else "—"),
    ]
    w = max(len(r[0]) for r in rows) + 2
    print(f"  {'Metric':{w}}  {'Baseline 9196b01':>22}  {'HEAD teaching':>18}")
    print(f"  {'-'*(w+44)}")
    for label, va, vb in rows:
        print(f"  {label:{w}}  {str(va):>22}  {str(vb):>18}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Connecting to MG400 Mock at 127.0.0.1 ...")
    dash = _connect_dashboard()
    motion = _connect_motion()
    fb = _connect_feedback()

    # Enable robot
    r = _dashboard_cmd(dash, "EnableRobot()")
    print(f"EnableRobot: {r}")
    time.sleep(0.5)

    # -------------------------------------------------------------------
    # PART 0: Batch command demo (fine-aiming, velocity < 0.1 rad/s)
    # -------------------------------------------------------------------
    print("\n--- [0/3] Batch command demo: robot at 20.5°,-15.3° → target 20°,-15° ---")
    # Move robot near pick point first
    motion.sendall(b"JointMovJ(20.5,-15.3,0,0)\n")
    time.sleep(0.1)
    _wait_for_motion_cycle(fb, timeout_s=8.0)
    time.sleep(0.1)

    batch_cmd = _build_batch_demo_cmd()
    batch_parts = batch_cmd.split(";")
    q_intended_last = _parse_first_4(batch_parts[-1])
    t0 = time.monotonic()
    _drain_feedback(fb)
    motion.sendall((batch_cmd + "\n").encode())
    time.sleep(0.02)
    batch_samples = _wait_for_motion_cycle(fb, timeout_s=8.0)
    dur_ms = (time.monotonic() - t0) * 1000.0

    final_b = batch_samples[-1]
    running_b = [s for s in batch_samples if s["robot_mode"] == MODE_RUNNING]
    q_target_mock = running_b[0]["q_target_deg"] if running_b else final_b["q_target_deg"]
    batch_rec = CommandRecord(
        label="batch_demo",
        sent_cmd=batch_cmd,
        is_batch=True,
        sub_cmd_count=len(batch_parts),
        q_target_intended_deg=q_intended_last,
        q_target_mock_deg=q_target_mock,
        q_actual_final_deg=final_b["q_actual_deg"],
        tool_vec_final=final_b["tool_vector_actual"],
        error_deg=np.abs(q_intended_last - final_b["q_actual_deg"]),
        samples=batch_samples,
        duration_ms=dur_ms,
    )
    _print_batch_demo(batch_rec)

    # Reset to home
    print("\n--- Resetting to home ---")
    motion.sendall(b"JointMovJ(0,0,0,0)\n")
    time.sleep(0.1)
    _wait_for_motion_cycle(fb, timeout_s=8.0)
    time.sleep(0.2)

    # -------------------------------------------------------------------
    # PART 1: Baseline 9196b01
    # -------------------------------------------------------------------
    print("\n--- [1/3] Baseline 9196b01 (teleop streaming, 10 commands) ---")
    baseline_cmds = _build_baseline_cmds()
    print(f"    {len(baseline_cmds)} motion commands to send")
    baseline = _execute_cmd_list(
        "Baseline 9196b01",
        baseline_cmds,
        [],
        dash, motion, fb,
    )
    baseline.non_motion_log = ["(no vacuum/tool logic in teleop path)"]
    baseline.non_motion_unsupported = 1

    # Reset to home
    print("--- Resetting to home ---")
    _reset_to_home(dash, motion, fb)
    time.sleep(0.2)

    # -------------------------------------------------------------------
    # PART 2: HEAD teaching path
    # -------------------------------------------------------------------
    print("\n--- [2/3] HEAD teaching path (3 JointMovJ + vacuum) ---")
    head_motion_cmds, head_non_motion = _build_head_cmds()
    print(f"    {len(head_motion_cmds)} motion commands, {len(head_non_motion)} non-motion")
    head = _execute_cmd_list(
        "HEAD teaching",
        head_motion_cmds,
        head_non_motion,
        dash, motion, fb,
    )

    # -------------------------------------------------------------------
    # Report
    # -------------------------------------------------------------------
    _print_run(baseline)
    _print_run(head)
    _print_comparison(baseline, head)

    print(f"\n{'#'*72}")
    print("  WHAT MOCK CANNOT PROVE")
    print(f"{'#'*72}")
    print("""
  - RunQueuedCmd / CurrentCommandId: not in Mock feedback packet
  - TCP queue saturation / backpressure under real load
  - CP blending continuity (Mock executes sequentially, no true blend)
  - Physical settling error (Mock: ideal trapezoid, no friction/backlash)
  - Real hardware response latency beyond TCP round-trip
""")

    dash.close()
    motion.close()
    fb.close()


if __name__ == "__main__":
    main()
