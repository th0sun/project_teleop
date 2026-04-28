#!/usr/bin/env python3
"""Measure teach-and-repeat replay timing against the live MG400 Mock.

This uses the real TrajectoryRecorder playback path, sends commands over
MG400_Mock TCP, samples realtime feedback, and compares q_actual(t) against
the taught trajectory q_target(t). It intentionally reports Mock limitations:
the Mock motion port does not acknowledge commands and does not implement the
real robot's queued-command feedback fields.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
import re
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
for _pkg in (
    "src/robot_teaching_core",
    "src/dobot_mg400/mg400_controller",
    "src/dobot_mg400/mg400_protocol",
    "MG400_Mock/app/src",
):
    _p = str(_REPO / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mg400_controller.common.trajectory.trajectory_recorder import (  # noqa: E402
    TrajectoryRecorder,
)
from mg400_controller.common.config import motion_config  # noqa: E402
from mg400_controller.common.utils.kinematics import KinematicsCalculator  # noqa: E402
from tcp_interface.realtime_packet import RealtimePacketType  # noqa: E402

MOCK_IP = os.environ.get("MG400_MOCK_HOST", "172.10.0.2")
DASHBOARD_PORT = int(os.environ.get("MG400_DASHBOARD_PORT", "29999"))
MOTION_PORT = int(os.environ.get("MG400_MOTION_PORT", "30003"))
FEEDBACK_PORT = int(os.environ.get("MG400_FEEDBACK_PORT", "30004"))
PKT_SIZE = np.dtype(RealtimePacketType).itemsize

MODE_ENABLE = 5
MODE_RUNNING = 7
KINEMATICS = KinematicsCalculator()
COMMAND_NAME_RE = re.compile(r"^\s*([A-Za-z0-9_]+)\s*\(")


class Logger:
    def info(self, msg):
        print(msg)

    def warn(self, msg):
        print(msg)

    def error(self, msg):
        print(msg)


@dataclass
class FeedbackSample:
    t: float
    robot_mode: int
    q_actual_deg: np.ndarray
    q_target_deg: np.ndarray
    tool_vector_actual: np.ndarray


class FeedbackMonitor:
    def __init__(self):
        self.samples: List[FeedbackSample] = []
        self._latest: Optional[FeedbackSample] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def latest_q_rad(self):
        with self._lock:
            latest = self._latest
        if latest is None:
            return None
        return np.radians(latest.q_actual_deg)

    def latest_robot_mode(self):
        with self._lock:
            latest = self._latest
        if latest is None:
            return 0
        return latest.robot_mode

    def snapshot(self) -> List[FeedbackSample]:
        with self._lock:
            return list(self.samples)

    def _run(self):
        sock = socket.create_connection((MOCK_IP, FEEDBACK_PORT), timeout=5)
        sock.settimeout(5)
        try:
            while not self._stop.is_set():
                data = b""
                while len(data) < PKT_SIZE and not self._stop.is_set():
                    data += sock.recv(PKT_SIZE - len(data))
                if len(data) < PKT_SIZE:
                    break
                arr = np.frombuffer(data, dtype=RealtimePacketType)
                sample = FeedbackSample(
                    t=time.time(),
                    robot_mode=int(arr["robot_mode"][0]),
                    q_actual_deg=np.array(arr["q_actual"][0][:4]),
                    q_target_deg=np.array(arr["q_target"][0][:4]),
                    tool_vector_actual=np.array(arr["tool_vector_actual"][0]),
                )
                with self._lock:
                    self.samples.append(sample)
                    self._latest = sample
        finally:
            sock.close()


@dataclass
class CommandEvent:
    t: float
    command: str


@dataclass
class ReplayEvents:
    playback_start_t: Optional[float] = None
    playback_complete_t: Optional[float] = None
    events: List[Dict] = field(default_factory=list)
    commands: List[CommandEvent] = field(default_factory=list)

    def callback(self, event_name: str, payload: Dict):
        now = time.time()
        item = {"t": now, "event": event_name, **payload}
        self.events.append(item)
        if event_name == "playback_start":
            self.playback_start_t = now
        elif event_name == "playback_complete":
            self.playback_complete_t = now


def _connect_dashboard() -> socket.socket:
    sock = socket.create_connection((MOCK_IP, DASHBOARD_PORT), timeout=5)
    sock.settimeout(5)
    return sock


def _connect_motion() -> socket.socket:
    sock = socket.create_connection((MOCK_IP, MOTION_PORT), timeout=5)
    sock.settimeout(5)
    return sock


def _dashboard_cmd(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.05)
    try:
        return sock.recv(1024).decode().strip()
    except Exception:
        return ""


def _wait_mode(monitor: FeedbackMonitor, mode: int, timeout_s: float = 8.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        samples = monitor.snapshot()
        if samples and samples[-1].robot_mode == mode:
            return True
        time.sleep(0.02)
    return False


def _wait_near_q(
    monitor: FeedbackMonitor,
    target_deg: np.ndarray,
    *,
    tolerance_deg: float,
    timeout_s: float,
) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        samples = monitor.snapshot()
        if samples:
            err = float(np.max(np.abs(samples[-1].q_actual_deg - target_deg)))
            if err <= tolerance_deg:
                return True
        time.sleep(0.02)
    return False


def _make_teach_frames() -> List[Dict]:
    """Compact synthetic teaching trajectory with explicit timestamps."""
    points = [
        (0.00, (0.0, 0.0, 0.0, 0.0)),
        (0.50, (5.0, -3.0, 0.0, 0.0)),
        (1.00, (10.0, -7.0, 0.0, 0.0)),
        (1.50, (15.0, -11.0, 0.0, 0.0)),
        (2.00, (20.0, -15.0, 0.0, 0.0)),
        (2.50, (20.0, -8.0, 0.0, 0.0)),
        (3.00, (20.0, 0.0, 0.0, 0.0)),
    ]
    return [
        {
            "timeStamp": t,
            "j1": q[0],
            "j2": q[1],
            "j3": q[2],
            "j4": q[3],
        }
        for t, q in points
    ]


def _load_trajectory_frames(path: Path) -> List[Dict]:
    data = json.loads(path.read_text(encoding="utf-8"))
    frames = data if isinstance(data, list) else data.get("frames", [])
    out = []
    for idx, frame in enumerate(frames):
        try:
            out.append({
                "timeStamp": float(frame["timeStamp"]),
                "j1": float(frame["j1"]),
                "j2": float(frame["j2"]),
                "j3": float(frame["j3"]),
                "j4": float(frame["j4"]),
            })
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid trajectory frame at index {idx}: {frame!r}") from exc
    if not out:
        raise ValueError(f"No frames found in {path}")
    return out


def _interp_target(frames: List[Dict], elapsed_s: float) -> np.ndarray:
    t = np.array([f["timeStamp"] - frames[0]["timeStamp"] for f in frames], dtype=float)
    q = np.array([[f["j1"], f["j2"], f["j3"], f["j4"]] for f in frames], dtype=float)
    elapsed_s = max(float(t[0]), min(float(t[-1]), elapsed_s))
    return np.array([np.interp(elapsed_s, t, q[:, axis]) for axis in range(4)])


def _frame_tool_vectors(frames: List[Dict]) -> np.ndarray:
    return np.array([
        KINEMATICS.forward_kinematics([frame["j1"], frame["j2"], frame["j3"], frame["j4"]])
        for frame in frames
    ], dtype=float)


def _interp_tool_target(frames: List[Dict], tool_vectors: np.ndarray, elapsed_s: float) -> np.ndarray:
    t = np.array([f["timeStamp"] - frames[0]["timeStamp"] for f in frames], dtype=float)
    elapsed_s = max(float(t[0]), min(float(t[-1]), elapsed_s))
    return np.array([np.interp(elapsed_s, t, tool_vectors[:, axis]) for axis in range(tool_vectors.shape[1])])


def _point_to_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    segment = end - start
    denom = float(np.dot(segment, segment))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - start))
    alpha = float(np.dot(point - start, segment) / denom)
    alpha = max(0.0, min(1.0, alpha))
    projection = start + alpha * segment
    return float(np.linalg.norm(point - projection))


def _nearest_path_xyz_error(point_xyz: np.ndarray, target_tool_vectors: np.ndarray) -> float:
    xyz_path = target_tool_vectors[:, :3]
    if len(xyz_path) == 0:
        return 0.0
    if len(xyz_path) == 1:
        return float(np.linalg.norm(point_xyz - xyz_path[0]))
    return min(
        _point_to_segment_distance(point_xyz, xyz_path[idx], xyz_path[idx + 1])
        for idx in range(len(xyz_path) - 1)
    )


def _command_name(command: str) -> str:
    match = COMMAND_NAME_RE.match(command or "")
    return match.group(1) if match else "unknown"


def _first_arrival_time(
    samples: List[FeedbackSample],
    start_t: float,
    target_deg: np.ndarray,
    *,
    tolerance_deg: float,
    earliest_elapsed_s: float = 0.0,
) -> Optional[float]:
    for sample in samples:
        if sample.t < start_t + earliest_elapsed_s:
            continue
        err = float(np.max(np.abs(sample.q_actual_deg - target_deg)))
        if err <= tolerance_deg:
            return sample.t - start_t
    return None


def _analyse(frames: List[Dict], samples: List[FeedbackSample], events: ReplayEvents) -> Dict:
    if events.playback_start_t is None:
        latest = samples[-1] if samples else None
        final_target = np.array([
            frames[-1]["j1"], frames[-1]["j2"], frames[-1]["j3"], frames[-1]["j4"]
        ], dtype=float)
        return {
            "status": "aborted_before_playback",
            "sample_count": 0,
            "post_settle_sample_count": 0,
            "planned_duration_s": round(frames[-1]["timeStamp"] - frames[0]["timeStamp"], 3),
            "measured_duration_s": 0.0,
            "post_settle_observed_s": 0.0,
            "max_tracking_error_deg": 0.0,
            "mean_tracking_error_deg": 0.0,
            "completion_error_deg": None,
            "completion_event_error_deg": None,
            "final_error_deg": None if latest is None else [
                round(float(v), 4) for v in np.abs(latest.q_actual_deg - final_target)
            ],
            "final_q_actual_deg": None if latest is None else [
                round(float(v), 4) for v in latest.q_actual_deg
            ],
            "final_target_deg": [round(float(v), 4) for v in final_target],
            "waypoints": [],
            "commands": [],
            "queued_waypoint_events": [],
        }
    start_t = events.playback_start_t
    end_t = events.playback_complete_t or (start_t + frames[-1]["timeStamp"])
    window = [s for s in samples if start_t <= s.t <= end_t]
    post_window = [s for s in samples if s.t >= start_t]
    target_tool_vectors = _frame_tool_vectors(frames)
    errors = []
    xyz_errors = []
    nearest_path_xyz_errors = []
    yaw_errors = []
    sample_rows = []
    for sample in window:
        elapsed = sample.t - start_t
        target = _interp_target(frames, elapsed)
        joint_error = float(np.max(np.abs(sample.q_actual_deg - target)))
        errors.append(joint_error)
        target_tool = _interp_tool_target(frames, target_tool_vectors, elapsed)
        xyz_error = float(np.linalg.norm(sample.tool_vector_actual[:3] - target_tool[:3]))
        path_dev = _nearest_path_xyz_error(sample.tool_vector_actual[:3], target_tool_vectors)
        yaw_error = float(abs(sample.tool_vector_actual[3] - target_tool[3]))
        xyz_errors.append(xyz_error)
        nearest_path_xyz_errors.append(path_dev)
        yaw_errors.append(yaw_error)
        sample_rows.append({
            "elapsed_s": round(float(elapsed), 4),
            "robot_mode": int(sample.robot_mode),
            "q_actual_deg": [round(float(v), 4) for v in sample.q_actual_deg],
            "q_target_feedback_deg": [round(float(v), 4) for v in sample.q_target_deg],
            "tool_actual": [round(float(v), 4) for v in sample.tool_vector_actual],
            "tool_target_interp": [round(float(v), 4) for v in target_tool],
            "joint_error_deg": round(joint_error, 4),
            "xyz_error_mm": round(xyz_error, 4),
            "path_dev_mm": round(path_dev, 4),
            "yaw_error_deg": round(yaw_error, 4),
        })

    waypoint_rows = []
    for idx, frame in enumerate(frames):
        target_t = frame["timeStamp"] - frames[0]["timeStamp"]
        q = np.array([frame["j1"], frame["j2"], frame["j3"], frame["j4"]], dtype=float)
        arrival = _first_arrival_time(
            window,
            start_t,
            q,
            tolerance_deg=1.0,
            earliest_elapsed_s=max(0.0, target_t),
        )
        target_tool = target_tool_vectors[idx]
        hit_xyz_error = None
        hit_yaw_error = None
        if arrival is not None:
            arrival_abs_t = start_t + arrival
            arrival_sample = next(
                (sample for sample in window if sample.t >= arrival_abs_t),
                None,
            )
            if arrival_sample is not None:
                hit_xyz_error = round(
                    float(np.linalg.norm(arrival_sample.tool_vector_actual[:3] - target_tool[:3])),
                    4,
                )
                hit_yaw_error = round(
                    float(abs(arrival_sample.tool_vector_actual[3] - target_tool[3])),
                    4,
                )
        waypoint_rows.append({
            "index": idx,
            "target_time_s": round(target_t, 3),
            "arrival_time_s": None if arrival is None else round(arrival, 3),
            "timing_error_s": None if arrival is None else round(arrival - target_t, 3),
            "target_deg": [round(float(v), 3) for v in q],
            "hit_xyz_error_mm": hit_xyz_error,
            "hit_yaw_error_deg": hit_yaw_error,
        })

    command_rows = []
    for command in events.commands:
        if command.t < start_t:
            phase = "go_to_start"
            elapsed = command.t - start_t
        else:
            phase = "playback"
            elapsed = command.t - start_t
        command_rows.append({
            "phase": phase,
            "elapsed_s": round(elapsed, 3),
            "command": command.command,
        })
    playback_command_names = [
        _command_name(row["command"])
        for row in command_rows
        if row["phase"] == "playback"
    ]
    command_type_counts = dict(sorted(Counter(playback_command_names).items()))

    final_sample = post_window[-1] if post_window else (window[-1] if window else samples[-1])
    completion_sample = window[-1] if window else final_sample
    playback_complete = next(
        (e for e in reversed(events.events) if e["event"] == "playback_complete"),
        {},
    )
    playback_timed_out = bool(playback_complete.get("timed_out", False))
    playback_success = bool(playback_complete.get("success", False))
    final_target = np.array([
        frames[-1]["j1"], frames[-1]["j2"], frames[-1]["j3"], frames[-1]["j4"]
    ], dtype=float)
    final_target_tool = target_tool_vectors[-1]
    return {
        "status": "ok" if playback_success else (
            "playback_timeout" if playback_timed_out else "playback_unsettled"
        ),
        "playback_success": playback_success,
        "playback_timed_out": playback_timed_out,
        "sample_count": len(window),
        "post_settle_sample_count": max(0, len(post_window) - len(window)),
        "planned_duration_s": round(frames[-1]["timeStamp"] - frames[0]["timeStamp"], 3),
        "measured_duration_s": round((end_t - start_t), 3),
        "post_settle_observed_s": round(max(0.0, final_sample.t - end_t), 3),
        "max_tracking_error_deg": round(max(errors) if errors else 0.0, 4),
        "mean_tracking_error_deg": round(float(np.mean(errors)) if errors else 0.0, 4),
        "max_xyz_error_mm": round(max(xyz_errors) if xyz_errors else 0.0, 4),
        "mean_xyz_error_mm": round(float(np.mean(xyz_errors)) if xyz_errors else 0.0, 4),
        "rmse_xyz_error_mm": round(float(np.sqrt(np.mean(np.square(xyz_errors)))) if xyz_errors else 0.0, 4),
        "max_nearest_path_xyz_error_mm": round(max(nearest_path_xyz_errors) if nearest_path_xyz_errors else 0.0, 4),
        "mean_nearest_path_xyz_error_mm": round(float(np.mean(nearest_path_xyz_errors)) if nearest_path_xyz_errors else 0.0, 4),
        "max_yaw_error_deg": round(max(yaw_errors) if yaw_errors else 0.0, 4),
        "mean_yaw_error_deg": round(float(np.mean(yaw_errors)) if yaw_errors else 0.0, 4),
        "completion_error_deg": [
            round(float(v), 4) for v in np.abs(completion_sample.q_actual_deg - final_target)
        ],
        "completion_event_success": playback_complete.get("success"),
        "completion_event_timed_out": playback_complete.get("timed_out"),
        "completion_event_error_deg": playback_complete.get("final_error_deg"),
        "final_error_deg": [
            round(float(v), 4) for v in np.abs(final_sample.q_actual_deg - final_target)
        ],
        "final_q_actual_deg": [round(float(v), 4) for v in final_sample.q_actual_deg],
        "final_target_deg": [round(float(v), 4) for v in final_target],
        "final_xyz_error_mm": round(float(np.linalg.norm(final_sample.tool_vector_actual[:3] - final_target_tool[:3])), 4),
        "final_yaw_error_deg": round(float(abs(final_sample.tool_vector_actual[3] - final_target_tool[3])), 4),
        "final_tool_actual": [round(float(v), 4) for v in final_sample.tool_vector_actual],
        "final_tool_target": [round(float(v), 4) for v in final_target_tool],
        "target_tool_path": [
            [round(float(v), 4) for v in tool]
            for tool in target_tool_vectors
        ],
        "sample_rows": sample_rows,
        "waypoints": waypoint_rows,
        "commands": command_rows,
        "motion_command_count": len(playback_command_names),
        "command_type_counts": command_type_counts,
        "queued_waypoint_events": [
            {
                "index": e["index"],
                "queued_elapsed_s": round(e["t"] - start_t, 3),
                "target_time_s": round(float(e["target_time_s"]), 3),
                "queue_lead_s": round(float(e["target_time_s"]) - (e["t"] - start_t), 3),
                "speed_j": e["speed_j"],
                "cp": e["cp"],
            }
            for e in events.events
            if e["event"] == "waypoint_queued"
        ],
    }


def run(
    out_path: Optional[Path],
    trajectory_json: Optional[Path] = None,
    post_settle_s: float = 0.5,
    *,
    speed_factor: Optional[int] = None,
) -> Dict:
    print(f"Connecting to MG400 Mock at {MOCK_IP} ...")
    dash = _connect_dashboard()
    motion = _connect_motion()
    monitor = FeedbackMonitor()
    events = ReplayEvents()

    def send_motion(command: str):
        events.commands.append(CommandEvent(time.time(), command))
        motion.sendall((command + "\n").encode())
        return True

    try:
        monitor.start()
        time.sleep(0.2)
        print(f"EnableRobot: {_dashboard_cmd(dash, 'EnableRobot()')}")
        if speed_factor is not None:
            print(f"SpeedFactor: {_dashboard_cmd(dash, f'SpeedFactor({int(speed_factor)})')}")
        _wait_mode(monitor, MODE_ENABLE, timeout_s=8.0)

        # Start from home so the replay measurement is deterministic.
        _dashboard_cmd(dash, "ResetRobot()")
        _dashboard_cmd(dash, "EnableRobot()")
        if speed_factor is not None:
            _dashboard_cmd(dash, f"SpeedFactor({int(speed_factor)})")
        send_motion("JointMovJ(0,0,0,0)")
        time.sleep(0.1)
        _wait_mode(monitor, MODE_ENABLE, timeout_s=20.0)
        _wait_near_q(
            monitor,
            np.array([0.0, 0.0, 0.0, 0.0]),
            tolerance_deg=0.05,
            timeout_s=60.0,
        )

        frames = _load_trajectory_frames(trajectory_json) if trajectory_json else _make_teach_frames()
        recorder = TrajectoryRecorder(
            command_send_fn=send_motion,
            dashboard_send_fn=lambda cmd: _dashboard_cmd(dash, cmd),
            logger=Logger(),
            get_position_fn=monitor.latest_q_rad,
            get_robot_mode_fn=monitor.latest_robot_mode,
            playback_event_callback=events.callback,
        )
        recorder.loaded_frames = frames
        recorder.loaded_name = trajectory_json.name if trajectory_json else "mock_timing_probe"

        recorder._play_worker()
        time.sleep(post_settle_s)

        result = _analyse(frames, monitor.snapshot(), events)
        result["trajectory_name"] = recorder.loaded_name
        result["execution_profile"] = str(
            getattr(motion_config, "PLAYBACK_EXECUTION_PROFILE", "preserve_timing")
        )
        if out_path is not None:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        return result
    finally:
        monitor.stop()
        dash.close()
        motion.close()


def main():
    global MOCK_IP, DASHBOARD_PORT, MOTION_PORT, FEEDBACK_PORT
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument(
        "--host",
        default=MOCK_IP,
        help="MG400 Mock host/IP. Ubuntu compose default: 172.10.0.2. "
             "macOS with published ports: 127.0.0.1.",
    )
    parser.add_argument("--dashboard-port", type=int, default=DASHBOARD_PORT)
    parser.add_argument("--motion-port", type=int, default=MOTION_PORT)
    parser.add_argument("--feedback-port", type=int, default=FEEDBACK_PORT)
    parser.add_argument("--trajectory-json", type=Path, default=None)
    parser.add_argument("--post-settle-s", type=float, default=0.5)
    parser.add_argument(
        "--speed-factor",
        type=int,
        default=None,
        help="Optional dashboard SpeedFactor to set before replay.",
    )
    parser.add_argument(
        "--execution-profile",
        choices=("preserve_timing", "fastest_path_repeat"),
        default=None,
        help="Override PLAYBACK_EXECUTION_PROFILE for this probe.",
    )
    parser.add_argument(
        "--cartesian-speed-ref-mm-s",
        type=float,
        default=None,
        help="Override CARTESIAN_SPEED_AT_100_PERCENT_MM_S for this probe.",
    )
    parser.add_argument(
        "--segment-min-speed-l",
        type=int,
        default=None,
        help="Override SEGMENT_MIN_SPEED_L for this probe.",
    )
    parser.add_argument(
        "--segment-acc-l",
        type=int,
        default=None,
        help="Override SEGMENT_ACC_L for this probe.",
    )
    parser.add_argument(
        "--absolute-timeout-s",
        type=float,
        default=None,
        help="Override PREVIEW_ABSOLUTE_TIMEOUT_SEC for this probe.",
    )
    parser.add_argument(
        "--mixed-primitives",
        choices=("on", "off"),
        default=None,
        help="Override USE_MIXED_PRIMITIVES for this probe.",
    )
    parser.add_argument(
        "--enable-arc",
        choices=("on", "off"),
        default=None,
        help="Override SEGMENT_ENABLE_ARC for this probe.",
    )
    parser.add_argument(
        "--no-waypoint-table",
        action="store_true",
        help="Suppress the verbose per-waypoint arrival table in stdout.",
    )
    args = parser.parse_args()

    MOCK_IP = args.host
    DASHBOARD_PORT = args.dashboard_port
    MOTION_PORT = args.motion_port
    FEEDBACK_PORT = args.feedback_port
    if args.cartesian_speed_ref_mm_s is not None:
        motion_config.CARTESIAN_SPEED_AT_100_PERCENT_MM_S = float(args.cartesian_speed_ref_mm_s)
    if args.execution_profile is not None:
        motion_config.PLAYBACK_EXECUTION_PROFILE = args.execution_profile
    if args.segment_min_speed_l is not None:
        motion_config.SEGMENT_MIN_SPEED_L = int(args.segment_min_speed_l)
    if args.segment_acc_l is not None:
        motion_config.SEGMENT_ACC_L = int(args.segment_acc_l)
    if args.absolute_timeout_s is not None:
        import mg400_controller.common.trajectory.trajectory_recorder as recorder_module

        recorder_module.PREVIEW_ABSOLUTE_TIMEOUT_SEC = float(args.absolute_timeout_s)
    if args.mixed_primitives is not None:
        motion_config.USE_MIXED_PRIMITIVES = args.mixed_primitives == "on"
    if args.enable_arc is not None:
        motion_config.SEGMENT_ENABLE_ARC = args.enable_arc == "on"

    result = run(
        args.out,
        trajectory_json=args.trajectory_json,
        post_settle_s=args.post_settle_s,
        speed_factor=args.speed_factor,
    )

    print("\nReplay timing result")
    print(f"  status:              {result.get('status', 'ok')}")
    print(f"  playback_success:    {result.get('playback_success')}")
    print(f"  playback_timed_out:  {result.get('playback_timed_out')}")
    print(f"  trajectory:          {result['trajectory_name']}")
    print(f"  execution_profile:   {result.get('execution_profile')}")
    print(f"  motion_commands:     {result.get('motion_command_count')}")
    print(f"  command_type_counts: {result.get('command_type_counts')}")
    print(f"  planned_duration_s:   {result['planned_duration_s']}")
    print(f"  measured_duration_s:  {result['measured_duration_s']}")
    print(f"  post_settle_s:        {result['post_settle_observed_s']}")
    print(f"  samples:              {result['sample_count']}")
    print(f"  max_joint_error_deg:  {result['max_tracking_error_deg']}")
    print(f"  mean_joint_error_deg: {result['mean_tracking_error_deg']}")
    print(f"  max_xyz_error_mm:     {result['max_xyz_error_mm']}")
    print(f"  mean_xyz_error_mm:    {result['mean_xyz_error_mm']}")
    print(f"  rmse_xyz_error_mm:    {result['rmse_xyz_error_mm']}")
    print(f"  max_path_dev_mm:      {result['max_nearest_path_xyz_error_mm']}")
    print(f"  mean_path_dev_mm:     {result['mean_nearest_path_xyz_error_mm']}")
    print(f"  max_yaw_error_deg:    {result['max_yaw_error_deg']}")
    print(f"  mean_yaw_error_deg:   {result['mean_yaw_error_deg']}")
    print(f"  completion_error_deg: {result['completion_error_deg']}")
    print(f"  final_error_deg:      {result['final_error_deg']}")
    print(f"  final_xyz_error_mm:   {result['final_xyz_error_mm']}")
    print(f"  final_yaw_error_deg:  {result['final_yaw_error_deg']}")
    if not args.no_waypoint_table:
        print("\nWaypoint arrival timing (1 deg tolerance)")
        for row in result["waypoints"]:
            print(
                f"  #{row['index']} target={row['target_time_s']:>5}s "
                f"arrival={row['arrival_time_s']}s error={row['timing_error_s']}s "
                f"xyz_hit={row['hit_xyz_error_mm']}mm yaw_hit={row['hit_yaw_error_deg']}deg "
                f"q={row['target_deg']}"
            )
        print("\nQueued waypoint events")
        for row in result["queued_waypoint_events"]:
            print(
                f"  #{row['index']} queued={row['queued_elapsed_s']:>5}s "
                f"target={row['target_time_s']:>5}s lead={row['queue_lead_s']:>5}s "
                f"SpeedJ={row['speed_j']} CP={row['cp']}"
            )


if __name__ == "__main__":
    main()
