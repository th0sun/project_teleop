#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trajectory Recorder & Sequencer — Teach-and-Repeat for MG400
=============================================================
Handles the full lifecycle:

1. **Load**    — read a saved JSON trajectory (Unity or native format).
2. **Execute** — command sequencer that keeps the taught waypoint order/path,
                 but uses operator-selected Speed/Acc/CP tuning instead of
                 trying to reproduce the demonstrator's timestamps.
3. **Stop**    — abort any operation and send the robot Home (0,0,0,0).

Topic integration (managed by vr_teleop_node.py):
  /teach/job_request    — String JSON body from Unity / simulator
  /unity/joint_cmd      — JointState live pose for realtime control
"""

import os
import time
import json
import re
import threading
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import List, Dict, Optional, Callable, Tuple, Any

from mg400_controller.common.config import motion_config
from mg400_controller.common.trajectory import (
    playback_compiler,
    playback_executor,
    trajectory_io,
)
from mg400_protocol.commands import arc, do_execute, joint_mov_j, mov_l_cartesian
from mg400_protocol.dashboard import enable_robot, reset_robot
from teaching_core.trajectory import (
    JointTimingLimits,
    TimedJointPoint,
    retime_joint_path,
    simplify_joint_path_rdp,
    speed_percent_for_segment,
)
from teaching_core.trajectory.segment_classifier import (
    SegmentType,
    classify_segments,
    segment_summary,
)


# ── Home position (degrees) ──────────────────────────────────────────────────
HOME_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0]

# ── Trajectory storage directory ─────────────────────────────────────────────
TRAJ_DIR = os.path.expanduser("~/project_teleop_ws/trajectories")

# ── Playback / arrival thresholds ────────────────────────────────────────────
PREVIEW_START_SPEEDJ = 20
PREVIEW_START_ACCJ = 50
PREVIEW_START_CP = 0
PREVIEW_STREAM_SPEEDJ = 100
PREVIEW_STREAM_MIN_SPEEDJ = 15
PREVIEW_STREAM_CP = 20
PREVIEW_FINAL_CP = 0
# Playback tuning defaults — chosen via on-robot benchmark, see EXP3/EXP4/EXP10:
#
# CP=30 (the previous default) caused 9 micro-stops per Δ=2° sweep on J1
# because the blend was too weak to mask the controller decel phase between
# commands.  CP=80 produced 0 stops in the same scenario (EXP4 5-rep
# replication) and tied CP=100 on direction-reversal runs.  We pick CP=80.
#
# SpeedJ=40 was a conservative pick for unverified trajectories.  With the
# new RDP simplifier (PATH_SIMPLIFY_TOLERANCE_DEG=3.0) every queued waypoint
# is ≥ 3° from its neighbour, putting the controller in the smooth-cruise
# regime even at higher speeds.  We raise SpeedJ default to 80 — operators
# who need slower playback (path verification, stress test) override via the
# job_request `options` payload.
PLAYBACK_DEFAULT_SPEED_J = 80   # was 40; raised because Δ ≥ 3° is now guaranteed
PLAYBACK_DEFAULT_ACC_J = 80
PLAYBACK_DEFAULT_SPEED_L = 60   # speed×CP matrix says safe so long as Cartesian CP is capped
PLAYBACK_DEFAULT_ACC_L = 80
PLAYBACK_DEFAULT_CP = 80        # was 30; EXP4 — biggest single contributor to teach jitter
PLAYBACK_DEFAULT_FINAL_CP = 0
PLAYBACK_DEFAULT_COMMAND_INTERVAL_SEC = 0.18
PLAYBACK_DEFAULT_QUEUE_LOOKAHEAD_COMMANDS = 3
PREVIEW_START_TIMEOUT_SEC = 6.0
PREVIEW_START_TOLERANCE_DEG = 3.0
PREVIEW_FINAL_TOLERANCE_DEG = 0.05
PREVIEW_FINAL_EXTRA_TIMEOUT_SEC = 2.0
PREVIEW_ABSOLUTE_TIMEOUT_SEC = 10.0
PREVIEW_TIMEOUT_PER_COMMAND_SEC = 0.35
PREVIEW_MAX_TIMEOUT_MARGIN_SEC = 60.0
PREVIEW_POLL_SEC = 0.01
PREVIEW_WAIT_POLL_SEC = 0.05
PREVIEW_LOOKAHEAD_MIN_SEC = 0.10
PREVIEW_LOOKAHEAD_MAX_SEC = 0.25
PREVIEW_LOOKAHEAD_FRAMES = 2.0
PREVIEW_JOINT_SPEED_AT_100_DEG_S = 90.0
PREVIEW_STREAM_REF_VEL_DEG_S = PREVIEW_JOINT_SPEED_AT_100_DEG_S
MG400_ROBOT_MODE_RUNNING = 7
PLAYBACK_PROFILE_PRESERVE_TIMING = "preserve_timing"
PLAYBACK_PROFILE_FASTEST_PATH_REPEAT = "fastest_path_repeat"


@dataclass(frozen=True)
class CompiledPlaybackCommand:
    index: int
    target_time_s: float
    original_target_time_s: float
    joints_deg: Tuple[float, float, float, float]
    speed_j: int
    cp: int
    command: str


@dataclass(frozen=True)
class CompiledPlaybackEventCommand:
    index: int
    target_time_s: float
    original_target_time_s: float
    kind: str
    channel: str
    value: bool
    port: int
    commands: Tuple[str, ...]
    delayed_commands: Tuple[Tuple[float, str], ...] = ()


@dataclass(frozen=True)
class CompiledPlaybackPlan:
    source_name: str
    waypoints: Tuple[Dict, ...]
    queued_commands: Tuple[CompiledPlaybackCommand, ...]
    event_commands: Tuple[CompiledPlaybackEventCommand, ...]
    original_duration_s: float
    retimed_duration_s: float
    total_duration_s: float
    time_scale: float
    original_timing_feasible: bool
    lookahead_s: float
    # Observability: how dense the recording was vs how many waypoints we
    # actually queue.  raw_waypoint_count is the unsimplified loaded_frames
    # count; len(waypoints) is what the MG400 motion queue actually sees.
    raw_waypoint_count: int = 0
    simplify_tolerance_deg: float = 0.0
    execution_profile: str = PLAYBACK_PROFILE_PRESERVE_TIMING


def compiled_playback_plan_to_dict(plan: CompiledPlaybackPlan) -> Dict[str, Any]:
    """Serialise a compiled playback plan into a JSON-safe debug artifact."""
    return {
        "artifact_kind": "mg400_compiled_playback_plan",
        "artifact_version": "0.1",
        "source_name": plan.source_name,
        "waypoint_count": len(plan.waypoints),
        "raw_waypoint_count": int(getattr(plan, "raw_waypoint_count", len(plan.waypoints))),
        "simplify_tolerance_deg": float(getattr(plan, "simplify_tolerance_deg", 0.0)),
        "execution_profile": str(
            getattr(plan, "execution_profile", PLAYBACK_PROFILE_PRESERVE_TIMING)
        ),
        "queued_command_count": len(plan.queued_commands),
        "event_command_count": len(getattr(plan, "event_commands", ())),
        "original_duration_s": plan.original_duration_s,
        "retimed_duration_s": plan.retimed_duration_s,
        "total_duration_s": plan.total_duration_s,
        "time_scale": plan.time_scale,
        "original_timing_feasible": plan.original_timing_feasible,
        "lookahead_s": plan.lookahead_s,
        "waypoints": [dict(frame) for frame in plan.waypoints],
        "queued_commands": [
            {
                "index": cmd.index,
                "target_time_s": cmd.target_time_s,
                "original_target_time_s": cmd.original_target_time_s,
                "joints_deg": list(cmd.joints_deg),
                "speed_j": cmd.speed_j,
                "cp": cmd.cp,
                "command": cmd.command,
            }
            for cmd in plan.queued_commands
        ],
        "event_commands": [
            {
                "index": event.index,
                "target_time_s": event.target_time_s,
                "original_target_time_s": event.original_target_time_s,
                "kind": event.kind,
                "channel": event.channel,
                "value": event.value,
                "port": event.port,
                "commands": list(event.commands),
                "delayed_commands": [
                    {"delay_s": delay_s, "command": command}
                    for delay_s, command in event.delayed_commands
                ],
            }
            for event in getattr(plan, "event_commands", ())
        ],
    }


class TrajectoryRecorder:
    """
    Teach-and-Repeat sequencer for MG400.

    Parameters
    ----------
    command_send_fn : callable(str) -> bool
        Sends a motion command to port 30003 (e.g. ``sender.send``).
    dashboard_send_fn : callable(str) -> bool
        Sends a dashboard command to port 29999 (e.g. ``connection.send_dashboard_cmd``).
        Used for ``wait(ms)`` timing commands.
    logger
        ROS-compatible logger with .info / .warn / .error methods.
    get_position_fn : callable() -> np.ndarray | None, optional
        Returns current joint angles in *radians* (4,).
        Required for recording mode.
    waypoint_callback : callable(np.ndarray) -> None, optional
        Called with radians (4,) for each played-back waypoint.
    """

    def __init__(self, command_send_fn: Callable, logger,
                 dashboard_send_fn: Optional[Callable] = None,
                 get_position_fn: Optional[Callable] = None,
                 waypoint_callback: Optional[Callable] = None,
                 target_callback: Optional[Callable] = None,
                 playback_event_callback: Optional[Callable] = None,
                 get_robot_mode_fn: Optional[Callable] = None,
                 traj_dir: Optional[str] = None,
                 time_fn: Optional[Callable] = None,
                 sleep_fn: Optional[Callable] = None):
        self._send = command_send_fn
        self._send_dash = dashboard_send_fn
        self._log  = logger
        self._get_pos = get_position_fn
        self._waypoint_cb = waypoint_callback
        self._target_cb = target_callback
        self._playback_event_cbs = []
        if playback_event_callback is not None:
            self._playback_event_cbs.append(playback_event_callback)
        self._get_robot_mode = get_robot_mode_fn
        self._traj_dir = TRAJ_DIR if traj_dir is None else traj_dir
        self._time_fn = time.time if time_fn is None else time_fn
        self._sleep_fn = time.sleep if sleep_fn is None else sleep_fn
        self._playback_tuning_lock = threading.Lock()
        self._playback_tuning = self._default_playback_tuning()

        # ── State ────────────────────────────────────────────────────────────
        self.is_playing    = False
        self._stop_flag    = threading.Event()
        self._play_thread: Optional[threading.Thread] = None
        self._block_until  = 0.0  # perf_counter: suppress teleop until this time

        # ── Delayed IO timer tracking ─────────────────────────────────────────
        # stop_all() cancels these so blow-off pulses don't fire after stop.
        self._pending_timers: List[threading.Timer] = []
        self._pending_timers_lock = threading.Lock()

        # ── Loaded trajectory (ready for playback) ───────────────────────────
        self.loaded_frames: List[Dict] = []
        self.loaded_events: List[Dict] = []
        self.loaded_name: str = ""

        os.makedirs(self._traj_dir, exist_ok=True)

    @staticmethod
    def _clamp_int(value, lo: int, hi: int, default: int) -> int:
        try:
            parsed = int(round(float(value)))
        except (TypeError, ValueError):
            return int(default)
        return max(int(lo), min(int(hi), parsed))

    @classmethod
    def _default_playback_tuning(cls) -> Dict[str, Any]:
        return {
            "use_recorded_timing": False,
            "speed_j": PLAYBACK_DEFAULT_SPEED_J,
            "acc_j": PLAYBACK_DEFAULT_ACC_J,
            "speed_l": PLAYBACK_DEFAULT_SPEED_L,
            "acc_l": PLAYBACK_DEFAULT_ACC_L,
            "cp": PLAYBACK_DEFAULT_CP,
            "final_cp": PLAYBACK_DEFAULT_FINAL_CP,
            "command_interval_s": PLAYBACK_DEFAULT_COMMAND_INTERVAL_SEC,
            "queue_lookahead_commands": PLAYBACK_DEFAULT_QUEUE_LOOKAHEAD_COMMANDS,
        }

    @classmethod
    def _normalize_playback_tuning(cls, options: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        base = cls._default_playback_tuning()
        if not isinstance(options, dict):
            return base

        use_recorded = options.get("use_recorded_timing", base["use_recorded_timing"])
        if isinstance(use_recorded, str):
            use_recorded = use_recorded.strip().lower() in {"1", "true", "yes", "on"}
        else:
            use_recorded = bool(use_recorded)

        speed = options.get("speed", base["speed_j"])
        acc = options.get("acc", base["acc_j"])
        base.update({
            "use_recorded_timing": use_recorded,
            "speed_j": cls._clamp_int(options.get("speed_j", speed), 1, 100, base["speed_j"]),
            "acc_j": cls._clamp_int(options.get("acc_j", acc), 1, 100, base["acc_j"]),
            "speed_l": cls._clamp_int(options.get("speed_l", speed), 1, 100, base["speed_l"]),
            "acc_l": cls._clamp_int(options.get("acc_l", acc), 1, 100, base["acc_l"]),
            "cp": cls._clamp_int(options.get("cp", base["cp"]), 0, 100, base["cp"]),
            "final_cp": cls._clamp_int(options.get("final_cp", base["final_cp"]), 0, 100, base["final_cp"]),
            "queue_lookahead_commands": cls._clamp_int(
                options.get("queue_lookahead_commands", base["queue_lookahead_commands"]),
                1,
                10,
                base["queue_lookahead_commands"],
            ),
        })
        try:
            interval = float(options.get("command_interval_s", base["command_interval_s"]))
        except (TypeError, ValueError):
            interval = base["command_interval_s"]
        base["command_interval_s"] = max(0.05, min(2.0, interval))
        return base

    def set_playback_tuning(self, options: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        tuning = self._normalize_playback_tuning(options)
        with self._playback_tuning_lock:
            self._playback_tuning = tuning
        return dict(tuning)

    def update_playback_tuning(self, options: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        with self._playback_tuning_lock:
            merged = dict(self._playback_tuning)
        if isinstance(options, dict):
            merged.update(options)
        return self.set_playback_tuning(merged)

    def playback_tuning(self) -> Dict[str, Any]:
        with self._playback_tuning_lock:
            return dict(self._playback_tuning)

    # ═════════════════════════════════════════════════════════════════════════
    #  RECORD  (smart-sampled at ~8-10 Hz)
    # ═════════════════════════════════════════════════════════════════════════




    # ═════════════════════════════════════════════════════════════════════════
    #  SAVE / LOAD
    # ═════════════════════════════════════════════════════════════════════════
    # The save / load / list methods are thin facades over
    # ``common.trajectory.trajectory_io`` — see that module for the
    # actual file shape and parser behaviour.  This wrapper keeps the
    # recorder's existing public API stable for callers that import
    # ``TrajectoryRecorder`` directly (Unity adapters, monitor GUI,
    # tests).  Behaviour is identical to the pre-extraction code.


    def load(self, name: str) -> bool:
        """Load a trajectory file by name from TRAJ_DIR."""
        path = trajectory_io.resolve_trajectory_path(self._traj_dir, name)
        frames, events = trajectory_io.load_trajectory(path, self._log)
        if frames is None:
            return False
        # Preserve recorder's prior bookkeeping: store with the resolved
        # ``.json`` suffix so loaded_name matches what's on disk.
        loaded_name = name if name.endswith(".json") else f"{name}.json"
        self.loaded_frames = frames
        self.loaded_events = events or []
        self.loaded_name = loaded_name
        dur = frames[-1]["timeStamp"] - frames[0]["timeStamp"]
        self._log.info(f"📂 Loaded {loaded_name}: {len(frames)} frames, {dur:.1f}s")
        return True

    def load_frames(
        self,
        frames: List[Dict],
        name: str = "inline_trajectory",
        events: Optional[List[Dict]] = None,
    ) -> bool:
        """Load already-materialized trajectory frames for immediate playback."""
        if not frames:
            self._log.error("Trajectory frame list is empty")
            return False
        self.loaded_frames = frames
        self.loaded_events = list(events or [])
        self.loaded_name = name
        dur = frames[-1]["timeStamp"] - frames[0]["timeStamp"]
        self._log.info(f"📂 Loaded {name}: {len(frames)} frames, {dur:.1f}s")
        return True


    # ═════════════════════════════════════════════════════════════════════════
    #  PREVIEW (Temporal Sequencer — NO decimation, uses frames directly)
    # ═════════════════════════════════════════════════════════════════════════
    def start_preview(self):
        """Begin playing back loaded trajectory in a background thread."""
        if not self.loaded_frames:
            self._log.error("No trajectory loaded for preview")
            return
        if self.is_playing:
            self._log.warn("Already playing")
            return
        self._stop_flag.clear()
        self.is_playing = True
        self._play_thread = threading.Thread(target=self._play_worker, daemon=True)
        self._play_thread.start()

    def _build_jointmovj_command(self, joints_deg, speed_j, cp, acc_j=None):
        return joint_mov_j(
            joints_deg[:4],
            speed_j=speed_j,
            acc_j=acc_j,
            cp=cp,
        ).render()

    def add_playback_event_callback(self, callback: Callable):
        """Register an additional playback event observer."""
        if callback is None:
            return
        self._playback_event_cbs.append(callback)


    def _emit_playback_event(self, event_name, **payload):
        if not self._playback_event_cbs:
            return
        for callback in list(self._playback_event_cbs):
            try:
                callback(event_name, payload)
            except Exception:
                pass

    def _get_current_position_deg(self):
        if self._get_pos is None:
            return None
        pos = self._get_pos()
        if pos is None:
            return None
        return np.degrees(np.asarray(pos[:4], dtype=float))

    # The compile-helper methods below now delegate to
    # ``common.trajectory.playback_compiler`` — see that module for the
    # actual geometry / IK logic.  These wrappers keep the recorder's
    # private API stable for callers (tests, monitor GUI hooks) that
    # reach in for ``_frame_q_deg`` / ``_line_primitive_reachable`` /
    # etc.  Behaviour is identical to the pre-extraction code.
    def _frame_q_deg(self, frame):
        return playback_compiler.frame_q_deg(frame)

    def _playback_execution_profile(self) -> str:
        profile = str(
            getattr(
                motion_config,
                "PLAYBACK_EXECUTION_PROFILE",
                PLAYBACK_PROFILE_PRESERVE_TIMING,
            )
        ).strip().lower()
        if profile in {"fast", "fastest", PLAYBACK_PROFILE_FASTEST_PATH_REPEAT}:
            return PLAYBACK_PROFILE_FASTEST_PATH_REPEAT
        return PLAYBACK_PROFILE_PRESERVE_TIMING

    def _fastest_path_repeat_enabled(self) -> bool:
        """LEGACY (2026-05) gate for the dormant ``fastest_path_repeat`` profile.

        Returns True only when the operator explicitly opts in via the
        module-level ``motion_config.PLAYBACK_EXECUTION_PROFILE = "fastest_path_repeat"``
        (or the aliases ``"fast"`` / ``"fastest"``).  The default
        ``"preserve_timing"`` profile never reaches this branch, so the
        FAST_REPEAT_* constants stay dormant in production.
        See AGENTS.md §4.3 — proof-of-death checklist required before the
        FAST_REPEAT_* constants and this branch can be removed.
        """
        return self._playback_execution_profile() == PLAYBACK_PROFILE_FASTEST_PATH_REPEAT

    def _use_recorded_timing(self) -> bool:
        return bool(self.playback_tuning().get("use_recorded_timing", False))

    def _stream_cp(self, *, is_final: bool) -> int:
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            return int(tuning["final_cp"] if is_final else tuning["cp"])
        if self._fastest_path_repeat_enabled():
            key = "FAST_REPEAT_FINAL_CP" if is_final else "FAST_REPEAT_CP"
            fallback = PREVIEW_FINAL_CP if is_final else PREVIEW_STREAM_CP
            return int(getattr(motion_config, key, fallback))
        return PREVIEW_FINAL_CP if is_final else PREVIEW_STREAM_CP

    def _stream_acc_j(self) -> Optional[int]:
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            return int(tuning["acc_j"])
        if not self._fastest_path_repeat_enabled():
            return None
        return int(getattr(motion_config, "FAST_REPEAT_ACC_J", 100))

    def _stream_acc_l(self) -> int:
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            return int(tuning["acc_l"])
        if self._fastest_path_repeat_enabled():
            return int(getattr(motion_config, "FAST_REPEAT_ACC_L", 100))
        return int(getattr(motion_config, "SEGMENT_ACC_L", 80))

    def _max_commands_per_cycle(self) -> int:
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            return int(tuning.get(
                "queue_lookahead_commands",
                PLAYBACK_DEFAULT_QUEUE_LOOKAHEAD_COMMANDS,
            ))
        if self._fastest_path_repeat_enabled():
            return max(1, int(getattr(motion_config, "FAST_REPEAT_MAX_COMMANDS_PER_CYCLE", 1)))
        return 1_000_000

    @staticmethod
    def _set_command_option(command: str, key: str, value: int) -> str:
        """Replace or append a Dobot command option such as SpeedJ=40."""
        value = int(value)
        pattern = rf"(?<![A-Za-z0-9_]){re.escape(key)}\s*=\s*[^,\)]*"
        replacement = f"{key}={value}"
        if re.search(pattern, command):
            return re.sub(pattern, replacement, command)
        end = command.rfind(")")
        if end < 0:
            return command
        prefix = command[:end].rstrip()
        suffix = command[end:]
        sep = "," if not prefix.endswith("(") else ""
        return f"{prefix}{sep}{replacement}{suffix}"

    def _apply_runtime_tuning_to_command(self, command: str, *, is_final: bool) -> str:
        """Apply the latest operator speed/acc/CP tuning before a command is sent.

        This is intentionally late-bound.  Compile validates the path and creates
        the command order, while the operator can still adjust motion tuning
        during playback; queued commands that have not been sent yet pick up the
        latest values.
        """
        tuning = self.playback_tuning()
        if tuning.get("use_recorded_timing", False):
            return command

        cp = int(tuning["final_cp"] if is_final else tuning["cp"])
        name = command.split("(", 1)[0].strip()
        updated = command
        if name == "JointMovJ":
            updated = self._set_command_option(updated, "SpeedJ", int(tuning["speed_j"]))
            updated = self._set_command_option(updated, "AccJ", int(tuning["acc_j"]))
            updated = self._set_command_option(updated, "CP", cp)
        elif name in {"MovL", "Arc"}:
            updated = self._set_command_option(updated, "SpeedL", int(tuning["speed_l"]))
            updated = self._set_command_option(updated, "AccL", int(tuning["acc_l"]))
            updated = self._set_command_option(updated, "CP", cp)
        return updated

    def _fast_repeat_schedule_times(self, frames, fallback_target_t):
        """Return command schedule times for fastest path repeat.

        In this profile timestamps are used only to order path progress, not to
        preserve the demonstrator's exact hand speed.  The schedule is a rough
        joint-speed lower bound; execution uses a large lookahead so the robot
        controller still gets queued work early.
        """
        if not self._fastest_path_repeat_enabled():
            return fallback_target_t

        speed_pct = int(getattr(motion_config, "FAST_REPEAT_SPEED_J", PREVIEW_STREAM_SPEEDJ))
        speed_deg_s = max(
            1e-3,
            PREVIEW_JOINT_SPEED_AT_100_DEG_S * max(1, min(100, speed_pct)) / 100.0,
        )
        target_t = [0.0]
        for idx in range(1, len(frames)):
            prev_q = np.asarray(self._frame_q_deg(frames[idx - 1]), dtype=float)
            q = np.asarray(self._frame_q_deg(frames[idx]), dtype=float)
            max_delta = float(np.max(np.abs(q - prev_q)))
            target_t.append(target_t[-1] + max_delta / speed_deg_s)
        return np.asarray(target_t, dtype=float)

    def _operator_schedule_times(self, frames) -> np.ndarray:
        interval = float(self.playback_tuning().get(
            "command_interval_s",
            PLAYBACK_DEFAULT_COMMAND_INTERVAL_SEC,
        ))
        return np.asarray([idx * interval for idx in range(len(frames))], dtype=float)

    def _retime_frames_for_playback(self, frames):
        """Return frames with preserved-or-stretched timestamps.

        The waypoint positions are unchanged.  Timestamps are preserved if the
        requested motion fits the configured effective MG400 joint speed; only
        too-fast segments are stretched.
        """
        points = [
            TimedJointPoint(time_s=float(frame["timeStamp"]), position=self._frame_q_deg(frame))
            for frame in frames
        ]
        timing = retime_joint_path(
            points,
            JointTimingLimits(
                max_velocity=(PREVIEW_JOINT_SPEED_AT_100_DEG_S,) * 4,
            ),
        )
        first_t = float(frames[0]["timeStamp"])
        retimed_frames = []
        for frame, point in zip(frames, timing.points):
            next_frame = dict(frame)
            next_frame["timeStamp"] = round(first_t + point.time_s, 6)
            retimed_frames.append(next_frame)

        if not timing.is_original_timing_feasible:
            self._log.warn(
                "⚠️  Trajectory too fast for configured MG400 speed; "
                f"retimed {timing.original_duration_s:.2f}s -> "
                f"{timing.retimed_duration_s:.2f}s "
                f"(x{timing.time_scale:.2f}, max segment x{timing.max_segment_stretch:.2f})"
            )
        return retimed_frames, timing

    def _segment_speed_j(self, prev_frame, frame):
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            return int(tuning["speed_j"])
        if self._fastest_path_repeat_enabled():
            return int(getattr(motion_config, "FAST_REPEAT_SPEED_J", PREVIEW_STREAM_SPEEDJ))
        dt = max(float(frame["timeStamp"] - prev_frame["timeStamp"]), 1e-3)
        return speed_percent_for_segment(
            self._frame_q_deg(prev_frame),
            self._frame_q_deg(frame),
            dt,
            (PREVIEW_JOINT_SPEED_AT_100_DEG_S,) * 4,
            min_percent=PREVIEW_STREAM_MIN_SPEEDJ,
            max_percent=PREVIEW_STREAM_SPEEDJ,
        )

    def _segment_speed_l(self, segment, frames):
        """Calculate dynamic Cartesian SpeedL percentage based on timestamp dt."""
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            return int(tuning["speed_l"])
        if self._fastest_path_repeat_enabled():
            return int(getattr(motion_config, "FAST_REPEAT_SPEED_L", 100))

        start_frame = frames[segment.start_idx]
        end_frame = frames[segment.end_idx]
        dt = max(float(end_frame["timeStamp"] - start_frame["timeStamp"]), 1e-3)

        if segment.type.name == "LINE":
            p1 = segment.cartesian_points[0].xyz()
            p2 = segment.cartesian_points[-1].xyz()
            dist = float(np.linalg.norm(p2 - p1))
        elif segment.type.name == "ARC":
            dist = 0.0
            for i in range(1, len(segment.cartesian_points)):
                p1 = segment.cartesian_points[i-1].xyz()
                p2 = segment.cartesian_points[i].xyz()
                dist += float(np.linalg.norm(p2 - p1))
        else:
            return self._segment_speed_j(start_frame, end_frame)

        v_mm_s = dist / dt
        ref_speed = getattr(motion_config, "CARTESIAN_SPEED_AT_100_PERCENT_MM_S", 1000.0)
        speed_l = (v_mm_s / ref_speed) * 100.0

        min_l = getattr(motion_config, "SEGMENT_MIN_SPEED_L", 5)
        max_l = getattr(motion_config, "SEGMENT_MAX_SPEED_L", 100)
        return int(max(min_l, min(max_l, speed_l)))


    def _line_primitive_reachable(self, seg, samples: int = 12) -> bool:
        return playback_compiler.line_primitive_reachable(seg, samples=samples)


    def _arc_primitive_reachable(self, seg, samples: int = 16) -> bool:
        return playback_compiler.arc_primitive_reachable(seg, samples=samples)

    def _raw_fit_error_mm(
        self,
        seg,
        primitive_type: SegmentType,
        source_frames,
        raw_frames,
    ) -> float:
        """Max XYZ distance from raw Unity path to the emitted primitive path.

        Classification happens after RDP simplification.  This guard checks
        the command-shaped primitive against the original dense capture so
        a sparse set of kept waypoints cannot accidentally approve an
        Arc / MovL that cuts too far away from what the user taught.
        """
        return playback_compiler.raw_fit_error_mm(
            seg, primitive_type, source_frames, raw_frames,
        )

    def _lookahead_seconds(self, target_t):
        tuning = self.playback_tuning()
        if not tuning.get("use_recorded_timing", False):
            interval = float(tuning.get("command_interval_s", PLAYBACK_DEFAULT_COMMAND_INTERVAL_SEC))
            commands = int(tuning.get("queue_lookahead_commands", PLAYBACK_DEFAULT_QUEUE_LOOKAHEAD_COMMANDS))
            return max(interval, interval * max(1, commands))
        if self._fastest_path_repeat_enabled():
            return float(getattr(motion_config, "FAST_REPEAT_LOOKAHEAD_SEC", 10.0))
        if len(target_t) < 2:
            return PREVIEW_LOOKAHEAD_MIN_SEC
        diffs = np.diff(target_t)
        positive = diffs[diffs > 1e-6]
        if len(positive) == 0:
            return PREVIEW_LOOKAHEAD_MIN_SEC
        median_dt = float(np.median(positive))
        window = median_dt * PREVIEW_LOOKAHEAD_FRAMES
        return max(PREVIEW_LOOKAHEAD_MIN_SEC, min(PREVIEW_LOOKAHEAD_MAX_SEC, window))

    def _event_bool_value(self, event):
        if "value" in event:
            return bool(event["value"])
        if "boolValue" in event:
            return bool(event["boolValue"])
        if "state" in event:
            return bool(event["state"])
        return False

    def _event_port(self, event):
        try:
            return int(event.get("port", 0) or 0)
        except (TypeError, ValueError):
            return 0

    def _event_channel(self, event):
        channel = event.get("channel") or event.get("name") or event.get("tool") or ""
        return str(channel).strip().lower()

    def _translate_digital_event(self, event):
        """Translate a robot-neutral captured IO event to MG400 dashboard cmds."""
        channel = self._event_channel(event)
        value = self._event_bool_value(event)
        port = self._event_port(event)

        if channel in {"vacuum", "suction", "suction_cup"}:
            vac_port = motion_config.VACUUM_DO_PORT
            blow_port = motion_config.BLOW_DO_PORT
            if vac_port <= 0 or blow_port <= 0:
                raise ValueError(
                    f"Vacuum/blow DO ports must be > 0 "
                    f"(VACUUM_DO_PORT={vac_port}, BLOW_DO_PORT={blow_port}). "
                    "Check motion_config.py."
                )
            if value:
                return (
                    vac_port,
                    (
                        do_execute(vac_port, True).render(),
                        do_execute(blow_port, False).render(),
                    ),
                    (),
                )
            return (
                vac_port,
                (
                    do_execute(vac_port, False).render(),
                    do_execute(blow_port, True).render(),
                ),
                (
                    (
                        float(motion_config.BLOW_DURATION),
                        do_execute(blow_port, False).render(),
                    ),
                ),
            )

        channel_ports = {
            "green_light": motion_config.GREEN_LIGHT_DO_PORT,
            "light_green": motion_config.GREEN_LIGHT_DO_PORT,
            "yellow_light": motion_config.YELLOW_LIGHT_DO_PORT,
            "light_yellow": motion_config.YELLOW_LIGHT_DO_PORT,
            "red_light": motion_config.RED_LIGHT_DO_PORT,
            "light_red": motion_config.RED_LIGHT_DO_PORT,
        }
        port = port or channel_ports.get(channel, 0)
        if port <= 0 and channel.startswith("do"):
            try:
                port = int(channel[2:])
            except ValueError:
                port = 0
        if port <= 0:
            raise ValueError(f"unsupported digital output channel: {channel!r}")

        return port, (do_execute(port, value).render(),), ()

    def _compile_event_commands(self, source_frames, target_t, source_target_t):
        if not self.loaded_events:
            return ()

        source_t0 = float(source_frames[0]["timeStamp"])
        original_duration = float(source_target_t[-1]) if len(source_target_t) else 0.0
        event_commands = []
        for idx, event in enumerate(self.loaded_events):
            kind = str(event.get("kind") or "digital_output").strip().lower()
            if kind not in {"digital_output", "io", "tool"}:
                raise ValueError(f"unsupported event kind: {kind!r}")

            original_t = max(0.0, float(event.get("timeStamp", source_t0)) - source_t0)
            if original_duration > 0:
                original_t = min(original_t, original_duration)
            retimed_t = float(np.interp(original_t, source_target_t, target_t))
            port, commands, delayed = self._translate_digital_event(event)
            event_commands.append(
                CompiledPlaybackEventCommand(
                    index=idx,
                    target_time_s=retimed_t,
                    original_target_time_s=original_t,
                    kind=kind,
                    channel=self._event_channel(event),
                    value=self._event_bool_value(event),
                    port=port,
                    commands=tuple(commands),
                    delayed_commands=tuple(delayed),
                )
            )
        return tuple(sorted(event_commands, key=lambda item: item.target_time_s))

    def _simplify_loaded_frames(
        self,
        frames: List[Dict],
        tolerance_deg: float,
    ) -> List[Dict]:
        """Drop dense intermediate frames whose joint values fall within
        ``tolerance_deg`` of the time-lerp between surrounding kept frames.

        Returns the original frame dicts verbatim — no synthetic interpolation,
        no resampling — so downstream code (timing, event mapping, artifact
        export) keeps the exact joint values the user demonstrated.
        """
        if tolerance_deg <= 0 or len(frames) <= 2:
            return list(frames)

        points = [
            TimedJointPoint(time_s=float(f["timeStamp"]), position=self._frame_q_deg(f))
            for f in frames
        ]
        kept = simplify_joint_path_rdp(points, tolerance_deg)

        # Map kept TimedJointPoints back to the original frame dicts using the
        # original timestamp as the join key (rounded to keep float equality
        # robust against the round-trip through the dataclass).
        kept_keys = {round(p.time_s, 9) for p in kept}
        return [f for f in frames if round(float(f["timeStamp"]), 9) in kept_keys]

    def compile_loaded_plan(self) -> CompiledPlaybackPlan:
        """Compile the loaded trajectory into a pre-timed MG400 playback job.

        This is the boundary we want for teach-and-repeat: compile the full job
        once, then let execution focus on dispatch/monitoring instead of
        recomputing waypoint timing inside the playback loop.

        When ``motion_config.USE_MIXED_PRIMITIVES`` is True, the pipeline
        classifies segments as LINE/ARC/GENERAL after RDP simplification
        and emits the most efficient MG400 command for each segment type:
          LINE    → single ``MovL``   (Cartesian linear)
          ARC     → single ``Arc``    (Cartesian arc via 3 defining points)
          GENERAL → per-waypoint ``JointMovJ`` chain (fallback)
        """
        if not self.loaded_frames:
            raise ValueError("No trajectory loaded for playback compilation")

        raw_frames = list(self.loaded_frames)
        raw_count = len(raw_frames)

        # Path simplification (RDP) — collapse dense recorded waypoints to the
        # critical points that actually shape the motion so the MG400 motion
        # queue does not back up while playing back long teach-and-repeat
        # trajectories.  Configured by motion_config.PATH_SIMPLIFY_TOLERANCE_DEG;
        # set to 0.0 to disable when a path must replay verbatim.
        tolerance = float(getattr(motion_config, "PATH_SIMPLIFY_TOLERANCE_DEG", 0.0))
        source_frames = self._simplify_loaded_frames(raw_frames, tolerance)
        if len(source_frames) < raw_count:
            self._log.info(
                f"📐 Path simplified: {raw_count} → {len(source_frames)} waypoints "
                f"(RDP @ {tolerance:.2f}°)"
            )

        if self._use_recorded_timing():
            frames, timing = self._retime_frames_for_playback(source_frames)
        else:
            frames = [dict(frame) for frame in source_frames]
            original_duration = float(source_frames[-1]["timeStamp"] - source_frames[0]["timeStamp"])
            operator_duration = max(
                0.0,
                (len(frames) - 1)
                * float(self.playback_tuning().get(
                    "command_interval_s",
                    PLAYBACK_DEFAULT_COMMAND_INTERVAL_SEC,
                )),
            )
            timing = type("OperatorTiming", (), {
                "original_duration_s": original_duration,
                "retimed_duration_s": operator_duration,
                "time_scale": 1.0,
                "is_original_timing_feasible": True,
            })()
        source_t0_traj = float(source_frames[0]["timeStamp"])
        t0_traj = float(frames[0]["timeStamp"])
        total_dur = float(frames[-1]["timeStamp"] - t0_traj)
        target_t = np.array([float(f["timeStamp"]) - t0_traj for f in frames])
        source_target_t = np.array(
            [float(f["timeStamp"]) - source_t0_traj for f in source_frames]
        )
        execution_profile = self._playback_execution_profile()
        if not self._use_recorded_timing():
            target_t = self._operator_schedule_times(frames)
            total_dur = float(target_t[-1]) if len(target_t) else 0.0
        else:
            target_t = self._fast_repeat_schedule_times(frames, target_t)
        if self._use_recorded_timing() and execution_profile == PLAYBACK_PROFILE_FASTEST_PATH_REPEAT:
            total_dur = max(total_dur, float(target_t[-1]) if len(target_t) else 0.0)
        lookahead = self._lookahead_seconds(target_t)

        # ── Mixed-primitive classification ────────────────────────────
        use_mixed = bool(getattr(motion_config, "USE_MIXED_PRIMITIVES", False))

        if use_mixed:
            queued_commands = self._compile_mixed_commands(
                frames,
                target_t,
                source_target_t,
                source_frames=source_frames,
                raw_frames=raw_frames,
            )
        else:
            queued_commands = self._compile_jointmovj_commands(
                frames, target_t, source_target_t,
            )

        event_commands = self._compile_event_commands(
            source_frames,
            target_t,
            source_target_t,
        )
        if event_commands:
            total_dur = max(total_dur, max(e.target_time_s for e in event_commands))
        if execution_profile == PLAYBACK_PROFILE_FASTEST_PATH_REPEAT:
            timeout_budget = len(queued_commands) * float(
                getattr(motion_config, "FAST_REPEAT_TIMEOUT_PER_COMMAND_SEC", 1.0)
            )
            total_dur = max(total_dur, timeout_budget)

        return CompiledPlaybackPlan(
            source_name=self.loaded_name or "inline_trajectory",
            waypoints=tuple(dict(frame) for frame in frames),
            queued_commands=tuple(queued_commands),
            event_commands=event_commands,
            original_duration_s=float(timing.original_duration_s),
            retimed_duration_s=float(timing.retimed_duration_s),
            total_duration_s=total_dur,
            time_scale=float(timing.time_scale),
            original_timing_feasible=bool(timing.is_original_timing_feasible),
            lookahead_s=float(lookahead),
            raw_waypoint_count=raw_count,
            simplify_tolerance_deg=tolerance,
            execution_profile=execution_profile,
        )

    # ── Command compilation strategies ───────────────────────────────

    def _compile_jointmovj_commands(
        self, frames, target_t, source_target_t,
    ) -> list:
        """Original strategy: one JointMovJ per waypoint."""
        queued_commands = []
        for idx in range(1, len(frames)):
            frame = frames[idx]
            prev_frame = frames[idx - 1]
            speed_j = self._segment_speed_j(prev_frame, frame)
            cp = self._stream_cp(is_final=idx == len(frames) - 1)
            joints = self._frame_q_deg(frame)
            queued_commands.append(
                CompiledPlaybackCommand(
                    index=idx,
                    target_time_s=float(target_t[idx]),
                    original_target_time_s=float(source_target_t[idx]),
                    joints_deg=tuple(float(v) for v in joints),
                    speed_j=int(speed_j),
                    cp=int(cp),
                    command=self._build_jointmovj_command(
                        joints,
                        speed_j=speed_j,
                        cp=cp,
                        acc_j=self._stream_acc_j(),
                    ),
                )
            )
        return queued_commands

    def _compile_mixed_commands(
        self,
        frames,
        target_t,
        source_target_t,
        *,
        source_frames=None,
        raw_frames=None,
    ) -> list:
        """Mixed-primitive strategy: LINE→MovL, ARC→Arc, GENERAL→JointMovJ.

        Orchestrator only — each phase below is its own helper:
          1. ``_classify_path_segments`` — group joint frames into
             LINE / ARC / GENERAL segments via the Cartesian
             classifier.
          2. Per segment, ``_resolve_primitive_type`` may downgrade
             LINE/ARC to GENERAL if the primitive isn't IK-feasible
             or deviates from the raw Unity path beyond
             ``SEGMENT_RAW_FIT_TOLERANCE_MM``.
          3. ``_build_line_segment_command`` /
             ``_build_arc_segment_command`` emit one Cartesian
             command per segment; ``_build_general_segment_commands``
             falls back to per-point JointMovJ.
        """
        raw_fit_tol = float(getattr(motion_config, "SEGMENT_RAW_FIT_TOLERANCE_MM", 0.0))
        acc_l = self._stream_acc_l()

        segments = self._classify_path_segments(frames)
        self._log.info(f"🔀 Mixed-primitive: {segment_summary(segments)}")

        queued_commands = []
        for seg in segments:
            end_idx = seg.end_idx
            is_final = end_idx == len(frames) - 1
            cp = self._stream_cp(is_final=is_final)
            primitive_type = self._resolve_primitive_type(
                seg, source_frames, raw_frames, raw_fit_tol
            )

            if primitive_type == SegmentType.LINE:
                queued_commands.append(
                    self._build_line_segment_command(
                        seg, frames, target_t, source_target_t, cp, acc_l
                    )
                )
            elif primitive_type == SegmentType.ARC:
                queued_commands.append(
                    self._build_arc_segment_command(
                        seg, frames, target_t, source_target_t, cp, acc_l
                    )
                )
            else:  # GENERAL — fallback per-point JointMovJ
                queued_commands.extend(
                    self._build_general_segment_commands(
                        seg, frames, target_t, source_target_t
                    )
                )

        return queued_commands

    def _classify_path_segments(self, frames):
        """Group ``frames`` into LINE/ARC/GENERAL segments using the
        Cartesian classifier. All tolerances come from
        ``motion_config.SEGMENT_*`` so behavior is tunable without
        touching this file.
        """
        joint_points = [self._frame_q_deg(f) for f in frames]
        line_tol = float(getattr(motion_config, "SEGMENT_LINE_TOLERANCE_MM", 2.0))
        arc_tol = float(getattr(motion_config, "SEGMENT_ARC_TOLERANCE_MM", 3.0))
        max_arc_radius = float(getattr(motion_config, "SEGMENT_MAX_ARC_RADIUS_MM", 10000.0))
        r_tol = float(getattr(motion_config, "SEGMENT_R_TOLERANCE_DEG", 10.0))
        min_arc = int(getattr(motion_config, "SEGMENT_MIN_POINTS_FOR_ARC", 3))
        enable_arc = bool(getattr(motion_config, "SEGMENT_ENABLE_ARC", False))
        return classify_segments(
            joint_points,
            fk_fn=self._fk_for_classifier,
            line_tol_mm=line_tol,
            arc_tol_mm=arc_tol,
            max_arc_radius_mm=max_arc_radius,
            r_tol_deg=r_tol,
            min_points_for_arc=min_arc,
            enable_arc=enable_arc,
        )

    def _resolve_primitive_type(self, seg, source_frames, raw_frames, raw_fit_tol):
        """Return the primitive type to actually emit for this segment.

        Downgrades to GENERAL if:
          1. The LINE / ARC primitive isn't IK-feasible from the
             current start pose.
          2. ``raw_fit_tol > 0`` and the primitive's emitted Cartesian
             path deviates from the raw Unity path by more than the
             tolerance.
        """
        primitive_type = seg.type
        if primitive_type == SegmentType.LINE and not self._line_primitive_reachable(seg):
            self._log.warn(
                f"⚠️  MovL segment {seg.start_idx}->{seg.end_idx} is not IK-feasible; "
                "falling back to JointMovJ waypoints"
            )
            return SegmentType.GENERAL
        if primitive_type == SegmentType.ARC and not self._arc_primitive_reachable(seg):
            self._log.warn(
                f"⚠️  Arc segment {seg.start_idx}->{seg.end_idx} is not IK-feasible; "
                "falling back to JointMovJ waypoints"
            )
            return SegmentType.GENERAL
        if raw_fit_tol > 0.0 and primitive_type in {SegmentType.LINE, SegmentType.ARC}:
            raw_error = self._raw_fit_error_mm(
                seg, primitive_type, source_frames, raw_frames
            )
            if raw_error > raw_fit_tol:
                self._log.warn(
                    f"⚠️  {primitive_type.name} segment {seg.start_idx}->{seg.end_idx} "
                    f"deviates {raw_error:.2f}mm from raw Unity path "
                    f"(limit {raw_fit_tol:.2f}mm); falling back to JointMovJ waypoints"
                )
                return SegmentType.GENERAL
        return primitive_type

    def _build_line_segment_command(
        self, seg, frames, target_t, source_target_t, cp, acc_l
    ) -> CompiledPlaybackCommand:
        """Emit a single MovL covering the whole LINE segment.

        Cartesian CP is capped by ``SEGMENT_CARTESIAN_CP_MAX``:
        high CP + high SpeedL between Cartesian segments rounds
        corners too aggressively for the servo to track and trips
        controller alarm 34322.
        """
        end_idx = seg.end_idx
        end_frame = frames[end_idx]
        speed_j = self._segment_speed_j(frames[seg.start_idx], end_frame)
        speed_l = self._segment_speed_l(seg, frames)
        cart_cp = min(
            int(cp), int(getattr(motion_config, "SEGMENT_CARTESIAN_CP_MAX", 30))
        )
        cmd_str = mov_l_cartesian(
            target_xyzr=seg.end_xyzr,
            speed_l=speed_l,
            acc_l=acc_l,
            cp=cart_cp,
        ).render()
        return CompiledPlaybackCommand(
            index=end_idx,
            target_time_s=float(target_t[end_idx]),
            original_target_time_s=float(source_target_t[end_idx]),
            joints_deg=tuple(float(v) for v in self._frame_q_deg(end_frame)),
            speed_j=int(speed_j),
            cp=int(cp),
            command=cmd_str,
        )

    def _build_arc_segment_command(
        self, seg, frames, target_t, source_target_t, cp, acc_l
    ) -> CompiledPlaybackCommand:
        """Emit a single Arc (through-point + end-point) for the ARC
        segment. Same Cartesian CP cap as LINE — see
        ``_build_line_segment_command``.
        """
        end_idx = seg.end_idx
        end_frame = frames[end_idx]
        speed_j = self._segment_speed_j(frames[seg.start_idx], end_frame)
        speed_l = self._segment_speed_l(seg, frames)
        cart_cp = min(
            int(cp), int(getattr(motion_config, "SEGMENT_CARTESIAN_CP_MAX", 30))
        )
        cmd_str = arc(
            through_xyzr=seg.through_xyzr,
            target_xyzr=seg.end_xyzr,
            speed_l=speed_l,
            acc_l=acc_l,
            cp=cart_cp,
        ).render()
        return CompiledPlaybackCommand(
            index=end_idx,
            target_time_s=float(target_t[end_idx]),
            original_target_time_s=float(source_target_t[end_idx]),
            joints_deg=tuple(float(v) for v in self._frame_q_deg(end_frame)),
            speed_j=int(speed_j),
            cp=int(cp),
            command=cmd_str,
        )

    def _build_general_segment_commands(
        self, seg, frames, target_t, source_target_t
    ) -> List[CompiledPlaybackCommand]:
        """Fallback: emit one JointMovJ per frame inside the segment.
        Used when LINE / ARC weren't IK-feasible or deviated from the
        raw Unity path beyond ``SEGMENT_RAW_FIT_TOLERANCE_MM``.
        """
        commands = []
        for idx in range(seg.start_idx + 1, seg.end_idx + 1):
            frame = frames[idx]
            prev_frame = frames[idx - 1]
            speed_j = self._segment_speed_j(prev_frame, frame)
            pt_cp = self._stream_cp(is_final=idx == len(frames) - 1)
            joints = self._frame_q_deg(frame)
            commands.append(
                CompiledPlaybackCommand(
                    index=idx,
                    target_time_s=float(target_t[idx]),
                    original_target_time_s=float(source_target_t[idx]),
                    joints_deg=tuple(float(v) for v in joints),
                    speed_j=int(speed_j),
                    cp=int(pt_cp),
                    command=self._build_jointmovj_command(
                        joints,
                        speed_j=speed_j,
                        cp=pt_cp,
                        acc_j=self._stream_acc_j(),
                    ),
                )
            )
        return commands

    @staticmethod
    def _fk_for_classifier(j1, j2, j3, j4):
        """FK adapter for segment classifier: (j1,j2,j3,j4) → (x,y,z,r)."""
        return playback_compiler.fk_for_classifier(j1, j2, j3, j4)

    def export_loaded_plan(self, path: str) -> str:
        """Compile + export the current playback job as a JSON artifact."""
        plan = self.compile_loaded_plan()
        out_path = Path(path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(
            json.dumps(compiled_playback_plan_to_dict(plan), indent=2) + "\n",
            encoding="utf-8",
        )
        self._log.info(f"🧾 Exported compiled playback plan → {out_path}")
        return str(out_path)

    def _wait_until_near_target(self, target_q_deg, tolerance_deg, timeout_sec):
        if self._get_pos is None:
            self._sleep_fn(timeout_sec)
            return not self._stop_flag.is_set()

        deadline = self._time_fn() + timeout_sec
        target = np.asarray(target_q_deg[:4], dtype=float)
        while self._time_fn() < deadline and not self._stop_flag.is_set():
            pos_deg = self._get_current_position_deg()
            if pos_deg is not None:
                err = float(np.max(np.abs(pos_deg - target)))
                if err < tolerance_deg:
                    return True
            self._sleep_fn(PREVIEW_WAIT_POLL_SEC)
        return False

    def _final_target_state(self, target_q):
        if self._get_pos is None:
            return None, None, None, None
        pos_deg = self._get_current_position_deg()
        if pos_deg is None:
            return None, None, None, None
        final_q = np.asarray(target_q[-1], dtype=float)
        per_joint_error = np.abs(pos_deg - final_q)
        robot_mode = None
        if self._get_robot_mode is not None:
            try:
                robot_mode = int(self._get_robot_mode())
            except Exception:
                robot_mode = None
        return float(np.max(per_joint_error)), per_joint_error, pos_deg, robot_mode

    def _send_go_to_start(self, first_frame):
        start_q = [first_frame["j1"], first_frame["j2"], first_frame["j3"], first_frame["j4"]]
        cmd_start = self._build_jointmovj_command(
            start_q,
            speed_j=PREVIEW_START_SPEEDJ,
            cp=PREVIEW_START_CP,
            acc_j=PREVIEW_START_ACCJ,
        )
        self._log.info(
            f"🏁 Go-to-start: ({first_frame['j1']:.1f},{first_frame['j2']:.1f},"
            f"{first_frame['j3']:.1f},{first_frame['j4']:.1f})"
        )
        self._emit_playback_event("go_to_start_command", command=cmd_start, frame=first_frame)
        self._send(cmd_start)
        return self._wait_until_near_target(
            start_q,
            tolerance_deg=PREVIEW_START_TOLERANCE_DEG,
            timeout_sec=PREVIEW_START_TIMEOUT_SEC,
        )

    # The four leaf helpers below now delegate to
    # ``common.trajectory.playback_executor``.  The recorder keeps its
    # private method names so existing callers (tests + monitor GUI)
    # do not break.  Behaviour is identical to the pre-extraction code.
    def _flush_motion_queue_after_timeout(self):
        return playback_executor.flush_motion_queue_after_timeout(
            self._send_dash, self._sleep_fn,
        )

    def _send_playback_event_command(self, command: str) -> bool:
        return playback_executor.send_playback_event_command(
            command, self._send_dash, self._log,
        )

    def _cancel_pending_timers(self) -> None:
        """Cancel all outstanding delayed IO command timers (called from stop_all)."""
        playback_executor.cancel_pending_timers(
            self._pending_timers, self._pending_timers_lock,
        )

    def _schedule_delayed_event_command(self, delay_s: float, command: str) -> None:
        def _send_later():
            try:
                self._send_playback_event_command(command)
            except Exception:
                pass
            finally:
                # Prune dead refs so the list doesn't grow indefinitely.
                with self._pending_timers_lock:
                    self._pending_timers[:] = [
                        t for t in self._pending_timers if t.is_alive()
                    ]

        timer = threading.Timer(max(0.0, float(delay_s)), _send_later)
        timer.daemon = True
        with self._pending_timers_lock:
            self._pending_timers.append(timer)
        timer.start()

    def _dispatch_event_command(self, event: CompiledPlaybackEventCommand, elapsed: float):
        for command in event.commands:
            self._send_playback_event_command(command)
        for delay_s, command in event.delayed_commands:
            self._schedule_delayed_event_command(delay_s, command)
        self._emit_playback_event(
            "io_event_queued",
            index=event.index,
            kind=event.kind,
            channel=event.channel,
            value=event.value,
            port=event.port,
            commands=list(event.commands),
            delayed_commands=[
                {"delay_s": delay_s, "command": command}
                for delay_s, command in event.delayed_commands
            ],
            target_time_s=event.target_time_s,
            original_target_time_s=event.original_target_time_s,
            elapsed_s=float(elapsed),
        )

    def _playback_complete(self, all_commands_sent, elapsed, total_dur, target_q):
        if not all_commands_sent:
            return False

        if self._get_pos is not None:
            max_err, _, _, robot_mode = self._final_target_state(target_q)
            if max_err is None:
                return False
            if max_err > PREVIEW_FINAL_TOLERANCE_DEG:
                return False
            if robot_mode == MG400_ROBOT_MODE_RUNNING:
                return False
            return True

        return elapsed >= total_dur + PREVIEW_FINAL_EXTRA_TIMEOUT_SEC


    def _playback_timed_out(self, elapsed, total_dur, command_count: Optional[int] = None):
        return playback_executor.playback_timed_out(
            elapsed,
            total_dur,
            command_count,
            absolute_sec=PREVIEW_ABSOLUTE_TIMEOUT_SEC,
            per_command_sec=PREVIEW_TIMEOUT_PER_COMMAND_SEC,
            max_margin_sec=PREVIEW_MAX_TIMEOUT_MARGIN_SEC,
        )

    def _play_worker(self):
        """Execute a precompiled playback job while publishing monitoring data.

        Top-level orchestrator. Each numbered phase below is its own
        helper method; the worker body just sequences them and owns
        the loop termination + final logging.
        """
        if not self.loaded_frames:
            return
        plan = self.compile_loaded_plan()
        frames = list(plan.waypoints)
        n = len(frames)
        total_dur = plan.total_duration_s

        self._log.info(
            f"▶️  Preview start — {n} waypoints, {total_dur:.1f}s "
            f"(compiled from {plan.source_name})"
        )

        target_t = np.array(
            [float(f["timeStamp"]) - float(frames[0]["timeStamp"]) for f in frames]
        )
        target_q = np.array([[f["j1"], f["j2"], f["j3"], f["j4"]] for f in frames])

        # 0. Move to trajectory start position before playing.
        arrived_at_start = self._send_go_to_start(frames[0])
        if self._stop_flag.is_set():
            self.is_playing = False
            return
        if not arrived_at_start:
            self.is_playing = False
            self._log.warn(
                "⚠️  Preview aborted: robot did not reach trajectory start in time"
            )
            return

        t_start = self._time_fn()
        self._emit_play_start_event(plan, n)
        command_idx, event_idx = self._run_playback_loop(
            plan, frames, target_t, target_q, total_dur, t_start
        )

        finalization = self._finalize_playback(plan, target_q, t_start)
        self.is_playing = False
        self._emit_play_complete_event(finalization, target_q)
        self._log_playback_result(finalization)

    def _emit_play_start_event(self, plan, n: int) -> None:
        """Emit the ``playback_start`` event with the plan's timing /
        retiming context so the monitor / bridge can stamp the run."""
        self._emit_playback_event(
            "playback_start",
            total_duration_s=plan.total_duration_s,
            original_duration_s=plan.original_duration_s,
            retimed_duration_s=plan.retimed_duration_s,
            time_scale=plan.time_scale,
            original_timing_feasible=plan.original_timing_feasible,
            waypoints=n,
            execution_model="compiled_queue_plan",
            execution_profile=plan.execution_profile,
        )

    def _run_playback_loop(self, plan, frames, target_t, target_q, total_dur, t_start):
        """Run the 100 Hz lookahead loop until stop / completion / timeout.

        Returns the final ``(command_idx, event_idx)`` for diagnostics
        (currently unused by the caller but kept so the loop body is
        a pure function of ``plan`` and time).
        """
        command_idx = 0
        event_idx = 0
        while not self._stop_flag.is_set():
            elapsed = self._time_fn() - t_start

            event_idx = self._dispatch_due_events(plan, elapsed, event_idx)
            command_idx = self._send_due_waypoints(plan, frames, elapsed, command_idx)
            self._publish_smooth_target(target_t, target_q, total_dur, elapsed)

            if (
                event_idx >= len(plan.event_commands)
                and command_idx >= len(plan.queued_commands)
                and self._playback_complete(True, elapsed, total_dur, target_q)
            ):
                break
            if self._playback_timed_out(elapsed, total_dur, len(plan.queued_commands)):
                break

            self._sleep_fn(PREVIEW_POLL_SEC)  # 100 Hz interpolation / polling cadence

        return command_idx, event_idx

    def _dispatch_due_events(self, plan, elapsed, event_idx):
        """Fire any event commands whose ``target_time_s`` has arrived."""
        while (
            event_idx < len(plan.event_commands)
            and plan.event_commands[event_idx].target_time_s <= elapsed
        ):
            self._dispatch_event_command(plan.event_commands[event_idx], elapsed)
            event_idx += 1
        return event_idx

    def _send_due_waypoints(self, plan, frames, elapsed, command_idx):
        """Pre-send any waypoints inside ``plan.lookahead_s`` from now.

        Capped by ``_max_commands_per_cycle`` so a long burst can't
        starve the smooth-target publish (phase 2 in the original
        loop).
        """
        sent_this_cycle = 0
        max_per_cycle = self._max_commands_per_cycle()
        while (
            command_idx < len(plan.queued_commands)
            and plan.queued_commands[command_idx].target_time_s <= elapsed + plan.lookahead_s
        ):
            compiled = plan.queued_commands[command_idx]
            is_final = command_idx == len(plan.queued_commands) - 1
            command = self._apply_runtime_tuning_to_command(
                compiled.command, is_final=is_final
            )
            self._send(command)
            self._emit_playback_event(
                "waypoint_queued",
                index=compiled.index,
                command=command,
                target_time_s=compiled.target_time_s,
                original_target_time_s=compiled.original_target_time_s,
                elapsed_s=float(elapsed),
                speed_j=int(self.playback_tuning().get("speed_j", compiled.speed_j)),
                cp=int(self.playback_tuning().get("cp", compiled.cp)),
                frame=frames[compiled.index],
            )

            # Discrete waypoint published for the red-dots graph.
            if self._waypoint_cb is not None:
                try:
                    self._waypoint_cb(np.radians(compiled.joints_deg))
                except Exception:
                    pass

            command_idx += 1
            sent_this_cycle += 1
            if sent_this_cycle >= max_per_cycle:
                break
        return command_idx

    def _publish_smooth_target(self, target_t, target_q, total_dur, elapsed):
        """Linearly-interpolated commanded target between waypoints (the
        equivalent of race.py's 'target' line). Used by the monitor's
        yellow trace for accurate per-frame error plots."""
        if self._target_cb is None:
            return
        try:
            if elapsed <= total_dur:
                q_curr = [np.interp(elapsed, target_t, target_q[:, i]) for i in range(4)]
            else:
                q_curr = target_q[-1]
            self._target_cb(np.radians(q_curr))
        except Exception:
            pass

    def _finalize_playback(self, plan, target_q, t_start):
        """Compute the per-run summary used by both the playback_complete
        event and the operator log.

        Returns a dict with: stopped, timed_out, success, queue_flushed,
        elapsed_s, final_max_error_deg, final_error_deg,
        final_q_actual_deg, final_target_deg, final_robot_mode.
        """
        final_max_error, final_error, final_position, final_robot_mode = (
            self._final_target_state(target_q)
        )
        elapsed_total = float(self._time_fn() - t_start)
        timed_out = bool(
            not self._stop_flag.is_set()
            and self._playback_timed_out(
                elapsed_total, plan.total_duration_s, len(plan.queued_commands)
            )
            and (
                final_max_error is None
                or final_max_error > PREVIEW_FINAL_TOLERANCE_DEG
                or final_robot_mode == MG400_ROBOT_MODE_RUNNING
            )
        )
        success = bool(
            not self._stop_flag.is_set()
            and not timed_out
            and (final_max_error is None or final_max_error <= PREVIEW_FINAL_TOLERANCE_DEG)
            and final_robot_mode != MG400_ROBOT_MODE_RUNNING
        )
        queue_flushed = self._flush_motion_queue_after_timeout() if timed_out else False
        return {
            "stopped": bool(self._stop_flag.is_set()),
            "timed_out": timed_out,
            "success": success,
            "queue_flushed": queue_flushed,
            "elapsed_s": elapsed_total,
            "final_max_error": final_max_error,
            "final_error": final_error,
            "final_position": final_position,
            "final_robot_mode": final_robot_mode,
        }

    def _emit_play_complete_event(self, fin: dict, target_q) -> None:
        """Emit ``playback_complete`` with the summary dict from
        ``_finalize_playback``."""
        final_max_error = fin["final_max_error"]
        final_error = fin["final_error"]
        final_position = fin["final_position"]
        final_target = np.asarray(target_q[-1], dtype=float)
        self._emit_playback_event(
            "playback_complete",
            stopped=fin["stopped"],
            timed_out=fin["timed_out"],
            success=fin["success"],
            queue_flushed=fin["queue_flushed"],
            elapsed_s=fin["elapsed_s"],
            final_max_error_deg=None if final_max_error is None else float(final_max_error),
            final_error_deg=None if final_error is None else [float(v) for v in final_error],
            final_q_actual_deg=None if final_position is None else [float(v) for v in final_position],
            final_target_deg=[float(v) for v in final_target],
            final_robot_mode=fin["final_robot_mode"],
        )

    def _log_playback_result(self, fin: dict) -> None:
        """Operator-facing one-liner summarising the run."""
        if fin["stopped"]:
            self._log.info("⏹️  Playback stopped")
        elif fin["timed_out"]:
            self._log.warn(
                "⚠️  Preview timed out before final target "
                f"(max error={fin['final_max_error']} deg, "
                f"robot_mode={fin['final_robot_mode']})"
            )
        elif not fin["success"]:
            self._log.warn(
                "⚠️  Preview ended without confirmed final settle "
                f"(max error={fin['final_max_error']} deg, "
                f"robot_mode={fin['final_robot_mode']})"
            )
        else:
            self._log.info("✅ Preview complete")

    # ═════════════════════════════════════════════════════════════════════════
    #  STOP & HOME
    # ═════════════════════════════════════════════════════════════════════════
    def stop_all(self, go_home: bool = False):
        """Stop active playback.  When go_home=True also send the robot
        Home; by default just aborts so live teleop resumes immediately.
        """
        was_playing = self.is_playing

        self._cancel_pending_timers()
        self._stop_flag.set()

        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=2.0)
        self.is_playing = False

        if was_playing:
            # Flush robot's queued commands so it stops immediately
            if self._send_dash is not None:
                self._send_dash(reset_robot().render())
                self._sleep_fn(0.2)
                self._send_dash(enable_robot().render())
            self._log.info("⏹️  Playback stopped (queue flushed)")

        if go_home:
            self._go_home()
            self._block_until = time.perf_counter() + 5.0

    def _go_home(self):
        """Send robot to Home position (0, 0, 0, 0)."""
        cmd = joint_mov_j(
            HOME_JOINTS_DEG[:4],
            speed_j=30,
            acc_j=50,
            cp=0,
        ).render()
        self._log.info("🏠 Moving to Home (0, 0, 0, 0)")
        self._send(cmd)

    # ═════════════════════════════════════════════════════════════════════════
    #  INFO
    # ═════════════════════════════════════════════════════════════════════════

    # ═════════════════════════════════════════════════════════════════════════
    #  INTERNAL
    # ═════════════════════════════════════════════════════════════════════════
    # ``_save_json`` was previously the private helper backing
    # ``save_temp`` / ``save_as``.  It has been replaced by
    # ``common.trajectory.trajectory_io.save_trajectory`` (a pure
    # function with the same wire shape); this stub remains as a
    # compatibility shim in case external tooling reached into the
    # recorder for it.  Behaviour is identical.
