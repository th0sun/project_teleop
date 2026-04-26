#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trajectory Recorder & Sequencer — Teach-and-Repeat for MG400
=============================================================
Handles the full lifecycle:

1. **Record**  — smart-sample at ~8-10 Hz: only frames with significant
                 movement (>RECORD_MIN_DELTA degrees) and at least
                 RECORD_MIN_DT apart.  Produces compact JSON identical
                 to the Unity money.json format.
2. **Save**    — persist as ``{"frames": [{timeStamp, j1..j4}, ...]}``.
3. **Load**    — read a saved JSON trajectory (Unity or native format).
4. **Preview** — temporal sequencer that replays each frame with the
                 EXACT joint values from the JSON, computing SpeedJ to
                 match original timing.  No decimation needed because
                 recording already produces clean 8-10 Hz data.
5. **Stop**    — abort any operation and send the robot Home (0,0,0,0).

Topic integration (managed by vr_teleop_node.py):
  /unity/teach_status   — String: Record | Stop | Save | Load | Preview
  /unity/trajectory_data — String: JSON body from Unity
"""

import os
import time
import json
import math
import threading
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import List, Dict, Optional, Callable, Tuple, Any

from mg400_controller.common.config import motion_config
from mg400_protocol.commands import do_execute, joint_mov_j
from mg400_protocol.dashboard import enable_robot, reset_robot
from teaching_core.trajectory import (
    JointTimingLimits,
    TimedJointPoint,
    retime_joint_path,
    simplify_joint_path_rdp,
    speed_percent_for_segment,
)


# ── Home position (degrees) ──────────────────────────────────────────────────
HOME_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0]

# ── Trajectory storage directory ─────────────────────────────────────────────
TRAJ_DIR = os.path.expanduser("~/project_teleop_ws/trajectories")

# ── Smart recording parameters ───────────────────────────────────────────────
# Inspired by money.json: ~8 Hz, 118 frames for 14.7 s, every frame has >1°
# movement.  This keeps files compact and eliminates the need for decimation
# during playback.
RECORD_MIN_DT    = 0.10   # seconds — max ~10 Hz recording rate
RECORD_MIN_DELTA = 0.5    # degrees — minimum joint movement to store a frame

# ── Playback / arrival thresholds ────────────────────────────────────────────
PREVIEW_START_SPEEDJ = 20
PREVIEW_START_ACCJ = 50
PREVIEW_START_CP = 0
PREVIEW_STREAM_SPEEDJ = 100
PREVIEW_STREAM_MIN_SPEEDJ = 15
PREVIEW_STREAM_CP = 20
PREVIEW_FINAL_CP = 0
PREVIEW_START_TIMEOUT_SEC = 6.0
PREVIEW_START_TOLERANCE_DEG = 3.0
PREVIEW_FINAL_TOLERANCE_DEG = 0.05
PREVIEW_FINAL_EXTRA_TIMEOUT_SEC = 2.0
PREVIEW_ABSOLUTE_TIMEOUT_SEC = 10.0
PREVIEW_POLL_SEC = 0.01
PREVIEW_WAIT_POLL_SEC = 0.05
PREVIEW_LOOKAHEAD_MIN_SEC = 0.10
PREVIEW_LOOKAHEAD_MAX_SEC = 0.25
PREVIEW_LOOKAHEAD_FRAMES = 2.0
PREVIEW_JOINT_SPEED_AT_100_DEG_S = 90.0
PREVIEW_STREAM_REF_VEL_DEG_S = PREVIEW_JOINT_SPEED_AT_100_DEG_S
MG400_ROBOT_MODE_RUNNING = 7


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


def compiled_playback_plan_to_dict(plan: CompiledPlaybackPlan) -> Dict[str, Any]:
    """Serialise a compiled playback plan into a JSON-safe debug artifact."""
    return {
        "artifact_kind": "mg400_compiled_playback_plan",
        "artifact_version": "0.1",
        "source_name": plan.source_name,
        "waypoint_count": len(plan.waypoints),
        "raw_waypoint_count": int(getattr(plan, "raw_waypoint_count", len(plan.waypoints))),
        "simplify_tolerance_deg": float(getattr(plan, "simplify_tolerance_deg", 0.0)),
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


def frames_from_joint_trajectory_msg(msg) -> List[Dict]:
    """Convert a ROS ``trajectory_msgs/JointTrajectory``-like message to frames.

    Unity's current `ROSPathPublisher` sends this message when the user presses
    "Send To Real Robot".  The recorder stores degrees internally, while ROS
    trajectory points use radians for revolute joints.
    """
    frames = []
    last_t = None
    for point in getattr(msg, "points", []):
        positions = getattr(point, "positions", [])
        if len(positions) < 4:
            continue
        duration = getattr(point, "time_from_start", None)
        if duration is None:
            continue
        t = float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) * 1e-9
        if last_t is not None and t <= last_t:
            continue
        q_deg = np.degrees(np.asarray(positions[:4], dtype=float))
        frames.append({
            "timeStamp": round(t, 6),
            "j1": round(float(q_deg[0]), 6),
            "j2": round(float(q_deg[1]), 6),
            "j3": round(float(q_deg[2]), 6),
            "j4": round(float(q_deg[3]), 6),
        })
        last_t = t
    return frames


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
        self._playback_event_cb = playback_event_callback
        self._playback_event_cbs = []
        if playback_event_callback is not None:
            self._playback_event_cbs.append(playback_event_callback)
        self._get_robot_mode = get_robot_mode_fn
        self._traj_dir = TRAJ_DIR if traj_dir is None else traj_dir
        self._time_fn = time.time if time_fn is None else time_fn
        self._sleep_fn = time.sleep if sleep_fn is None else sleep_fn

        # ── State ────────────────────────────────────────────────────────────
        self.is_recording  = False
        self.is_playing    = False
        self._stop_flag    = threading.Event()
        self._play_thread: Optional[threading.Thread] = None
        self._block_until  = 0.0  # perf_counter: suppress teleop until this time

        # ── Delayed IO timer tracking ─────────────────────────────────────────
        # stop_all() cancels these so blow-off pulses don't fire after stop.
        self._pending_timers: List[threading.Timer] = []
        self._pending_timers_lock = threading.Lock()

        # ── Recorded data ────────────────────────────────────────────────────
        self._frames: List[Dict] = []   # [{timeStamp, j1..j4}] degrees
        self._events: List[Dict] = []   # [{timeStamp, kind, channel, value, port?}]
        self._rec_t0 = 0.0
        self._last_rec_t = 0.0          # timestamp of last stored frame
        self._last_rec_q = np.zeros(4)   # joint values of last stored frame

        # ── Loaded trajectory (ready for playback) ───────────────────────────
        self.loaded_frames: List[Dict] = []
        self.loaded_events: List[Dict] = []
        self.loaded_name: str = ""

        os.makedirs(self._traj_dir, exist_ok=True)

    # ═════════════════════════════════════════════════════════════════════════
    #  RECORD  (smart-sampled at ~8-10 Hz)
    # ═════════════════════════════════════════════════════════════════════════
    def start_recording(self):
        if self.is_playing:
            self._log.warn("⚠️  Cannot record while playing")
            return
        self.is_recording = True
        self._block_until = 0.0
        self._frames = []
        self._events = []
        self._rec_t0 = self._time_fn()
        self._last_rec_t = -999.0
        self._last_rec_q = np.full(4, np.nan)
        self._log.info("🔴 Recording started (smart-sample ≤10 Hz, Δ≥0.5°)")

    def record_tick(self, target_q_rad):
        """Called at ~50 Hz from the control loop.
        Only stores a frame when:
          1. At least RECORD_MIN_DT (100 ms) since last stored frame, AND
          2. At least one joint moved ≥ RECORD_MIN_DELTA (0.5°) since last frame.
        Always stores the very first frame unconditionally.
        """
        if not self.is_recording or target_q_rad is None:
            return

        q_deg = np.degrees(target_q_rad[:4])
        now = self._time_fn() - self._rec_t0

        # Always store the first frame
        if not self._frames:
            self._append_frame(now, q_deg)
            return

        # Rate-limit
        if now - self._last_rec_t < RECORD_MIN_DT:
            return

        # Movement threshold
        max_delta = float(np.max(np.abs(q_deg - self._last_rec_q)))
        if max_delta < RECORD_MIN_DELTA:
            return

        self._append_frame(now, q_deg)

    def _append_frame(self, t: float, q_deg):
        self._frames.append({
            "timeStamp": round(t, 6),
            "j1": round(float(q_deg[0]), 6),
            "j2": round(float(q_deg[1]), 6),
            "j3": round(float(q_deg[2]), 6),
            "j4": round(float(q_deg[3]), 6),
        })
        self._last_rec_t = t
        self._last_rec_q = q_deg.copy()

    def stop_recording(self) -> List[Dict]:
        """Stop recording.  Appends a final frame if the last stored frame
        is older than 50 ms (captures the resting position)."""
        if self.is_recording and self._frames and self._get_pos is not None:
            try:
                q = self._get_pos()
                if q is not None:
                    now = self._time_fn() - self._rec_t0
                    if now - self._last_rec_t > 0.05:
                        self._append_frame(now, np.degrees(q[:4]))
            except Exception:
                pass

        self.is_recording = False
        n = len(self._frames)
        dur = self._frames[-1]["timeStamp"] if n else 0.0
        self._log.info(f"⏹️  Recording stopped — {n} frames, {dur:.1f}s")
        return self._frames

    # ═════════════════════════════════════════════════════════════════════════
    #  SAVE / LOAD
    # ═════════════════════════════════════════════════════════════════════════
    def save_temp(self) -> str:
        """Save last recording to temp_trajectory.json and return the path."""
        path = os.path.join(self._traj_dir, "temp_trajectory.json")
        return self._save_json(path, self._frames)

    def save_as(self, name: str) -> str:
        """Save last recording with a user-supplied name."""
        if not name.endswith(".json"):
            name += ".json"
        path = os.path.join(self._traj_dir, name)
        return self._save_json(path, self._frames)

    def save_from_unity_json(self, json_str: str) -> str:
        """Receive raw JSON from Unity /unity/trajectory_data and save."""
        try:
            data = json.loads(json_str)
            frames = data if isinstance(data, list) else data.get("frames", [])
            events = [] if isinstance(data, list) else data.get("events", [])
            filename = data.get("filename", "unity_trajectory.json") if isinstance(data, dict) else "unity_trajectory.json"
            
            self._frames = frames
            self._events = events if isinstance(events, list) else []
            return self.save_as(filename)
        except Exception as e:
            self._log.error(f"Failed to parse Unity trajectory JSON: {e}")
            return ""

    def load(self, name: str) -> bool:
        """Load a trajectory file by name from TRAJ_DIR."""
        if not name.endswith(".json"):
            name += ".json"
        path = os.path.join(self._traj_dir, name)
        if not os.path.isfile(path):
            self._log.error(f"Trajectory file not found: {path}")
            return False
        try:
            with open(path, "r") as f:
                data = json.load(f)
            frames = data if isinstance(data, list) else data.get("frames", [])
            events = [] if isinstance(data, list) else data.get("events", [])
            if not frames:
                self._log.error("Trajectory file is empty")
                return False
            self.loaded_frames = frames
            self.loaded_events = events if isinstance(events, list) else []
            self.loaded_name = name
            dur = frames[-1]["timeStamp"] - frames[0]["timeStamp"]
            self._log.info(f"📂 Loaded {name}: {len(frames)} frames, {dur:.1f}s")
            return True
        except Exception as e:
            self._log.error(f"Failed to load trajectory: {e}")
            return False

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

    def list_files(self) -> List[str]:
        """Return list of .json trajectory files."""
        try:
            return sorted(f for f in os.listdir(self._traj_dir) if f.endswith(".json"))
        except Exception:
            return []

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

    def set_playback_event_callback(self, callback: Optional[Callable]):
        """Replace playback event observers.

        Kept for integration code that needs a single owner of playback
        lifecycle events; most callers should use ``add_playback_event_callback``
        so measurement/debug observers can coexist with status publishers.
        """
        self._playback_event_cb = callback
        self._playback_event_cbs = []
        if callback is not None:
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

    def _frame_q_deg(self, frame):
        return (
            float(frame["j1"]),
            float(frame["j2"]),
            float(frame["j3"]),
            float(frame["j4"]),
        )

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
        dt = max(float(frame["timeStamp"] - prev_frame["timeStamp"]), 1e-3)
        return speed_percent_for_segment(
            self._frame_q_deg(prev_frame),
            self._frame_q_deg(frame),
            dt,
            (PREVIEW_JOINT_SPEED_AT_100_DEG_S,) * 4,
            min_percent=PREVIEW_STREAM_MIN_SPEEDJ,
            max_percent=PREVIEW_STREAM_SPEEDJ,
        )

    def _lookahead_seconds(self, target_t):
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

        frames, timing = self._retime_frames_for_playback(source_frames)
        source_t0_traj = float(source_frames[0]["timeStamp"])
        t0_traj = float(frames[0]["timeStamp"])
        total_dur = float(frames[-1]["timeStamp"] - t0_traj)
        target_t = np.array([float(f["timeStamp"]) - t0_traj for f in frames])
        source_target_t = np.array(
            [float(f["timeStamp"]) - source_t0_traj for f in source_frames]
        )
        lookahead = self._lookahead_seconds(target_t)

        queued_commands = []
        for idx in range(1, len(frames)):
            frame = frames[idx]
            prev_frame = frames[idx - 1]
            speed_j = self._segment_speed_j(prev_frame, frame)
            cp = PREVIEW_FINAL_CP if idx == len(frames) - 1 else PREVIEW_STREAM_CP
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
                    ),
                )
            )

        event_commands = self._compile_event_commands(
            source_frames,
            target_t,
            source_target_t,
        )
        if event_commands:
            total_dur = max(total_dur, max(e.target_time_s for e in event_commands))

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
        )

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

    def _flush_motion_queue_after_timeout(self):
        if self._send_dash is None:
            return False
        try:
            self._send_dash(reset_robot().render())
            self._sleep_fn(0.2)
            self._send_dash(enable_robot().render())
            return True
        except Exception:
            return False

    def _send_playback_event_command(self, command: str) -> bool:
        if self._send_dash is None:
            self._log.warn(f"⚠️  Cannot send playback IO event without dashboard channel: {command}")
            return False
        return bool(self._send_dash(command))

    def _cancel_pending_timers(self) -> None:
        """Cancel all outstanding delayed IO command timers (called from stop_all)."""
        with self._pending_timers_lock:
            for timer in self._pending_timers:
                timer.cancel()
            self._pending_timers.clear()

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

    def _playback_complete(self, idx, elapsed, total_dur, target_q):
        if idx < len(target_q):
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

    def _playback_timed_out(self, elapsed, total_dur):
        return elapsed >= total_dur + PREVIEW_ABSOLUTE_TIMEOUT_SEC

    def _play_worker(self):
        """Execute a precompiled playback job while publishing monitoring data."""
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

        target_t = np.array([float(f["timeStamp"]) - float(frames[0]["timeStamp"]) for f in frames])
        source_target_t = np.array(
            [0.0, *[cmd.original_target_time_s for cmd in plan.queued_commands]]
        )
        target_q = np.array([[f["j1"], f["j2"], f["j3"], f["j4"]] for f in frames])

        # ── 0. Move to trajectory start position before playing ────────────────
        first = frames[0]
        arrived_at_start = self._send_go_to_start(first)

        if self._stop_flag.is_set():
            self.is_playing = False
            return
        if not arrived_at_start:
            self.is_playing = False
            self._log.warn("⚠️  Preview aborted: robot did not reach trajectory start in time")
            return

        idx = 1
        event_idx = 0
        t_start = self._time_fn()
        self._emit_playback_event(
            "playback_start",
            total_duration_s=total_dur,
            original_duration_s=plan.original_duration_s,
            retimed_duration_s=plan.retimed_duration_s,
            time_scale=plan.time_scale,
            original_timing_feasible=plan.original_timing_feasible,
            waypoints=n,
            execution_model="compiled_queue_plan",
        )
        while not self._stop_flag.is_set():
            elapsed = self._time_fn() - t_start

            while (
                event_idx < len(plan.event_commands)
                and plan.event_commands[event_idx].target_time_s <= elapsed
            ):
                self._dispatch_event_command(plan.event_commands[event_idx], elapsed)
                event_idx += 1
            
            # 1. Pre-send any waypoints that fall within the current lookahead window
            while idx - 1 < len(plan.queued_commands) and target_t[idx] <= elapsed + plan.lookahead_s:
                compiled = plan.queued_commands[idx - 1]
                self._send(compiled.command)
                self._emit_playback_event(
                    "waypoint_queued",
                    index=compiled.index,
                    command=compiled.command,
                    target_time_s=compiled.target_time_s,
                    original_target_time_s=compiled.original_target_time_s,
                    elapsed_s=float(elapsed),
                    speed_j=compiled.speed_j,
                    cp=compiled.cp,
                    frame=frames[compiled.index],
                )
                
                # Publish the discrete command sent for the red dots graph
                if self._waypoint_cb is not None:
                    try:
                        self._waypoint_cb(np.radians(compiled.joints_deg))
                    except Exception:
                        pass
                        
                idx += 1
                
            # 2. Publish smooth real-time target for accurate graphing (like race.py)
            if self._target_cb is not None:
                try:
                    if elapsed <= total_dur:
                        q_curr = [np.interp(elapsed, target_t, target_q[:, i]) for i in range(4)]
                    else:
                        q_curr = target_q[-1]
                    self._target_cb(np.radians(q_curr))
                except Exception:
                    pass
                    
            # 3. Check loop termination
            if (
                event_idx >= len(plan.event_commands)
                and self._playback_complete(idx, elapsed, total_dur, target_q)
            ):
                break
            if self._playback_timed_out(elapsed, total_dur):
                break
                
            self._sleep_fn(PREVIEW_POLL_SEC) # 100Hz interpolation and polling loop

        final_max_error, final_error, final_position, final_robot_mode = self._final_target_state(target_q)
        final_target = np.asarray(target_q[-1], dtype=float)
        elapsed_total = float(self._time_fn() - t_start)
        timed_out = bool(
            not self._stop_flag.is_set()
            and self._playback_timed_out(elapsed_total, total_dur)
            and (
                final_max_error is None
                or final_max_error > PREVIEW_FINAL_TOLERANCE_DEG
                or final_robot_mode == MG400_ROBOT_MODE_RUNNING
            )
        )
        success = bool(
            not self._stop_flag.is_set()
            and not timed_out
            and (
                final_max_error is None
                or final_max_error <= PREVIEW_FINAL_TOLERANCE_DEG
            )
            and final_robot_mode != MG400_ROBOT_MODE_RUNNING
        )
        queue_flushed = self._flush_motion_queue_after_timeout() if timed_out else False

        self.is_playing = False
        self._emit_playback_event(
            "playback_complete",
            stopped=bool(self._stop_flag.is_set()),
            timed_out=timed_out,
            success=success,
            queue_flushed=queue_flushed,
            elapsed_s=elapsed_total,
            final_max_error_deg=None if final_max_error is None else float(final_max_error),
            final_error_deg=None if final_error is None else [float(v) for v in final_error],
            final_q_actual_deg=None if final_position is None else [float(v) for v in final_position],
            final_target_deg=[float(v) for v in final_target],
            final_robot_mode=final_robot_mode,
        )
        if self._stop_flag.is_set():
            self._log.info("⏹️  Playback stopped")
        elif timed_out:
            self._log.warn(
                "⚠️  Preview timed out before final target "
                f"(max error={final_max_error} deg, robot_mode={final_robot_mode})"
            )
        elif not success:
            self._log.warn(
                "⚠️  Preview ended without confirmed final settle "
                f"(max error={final_max_error} deg, robot_mode={final_robot_mode})"
            )
        else:
            self._log.info("✅ Preview complete")

    # ═════════════════════════════════════════════════════════════════════════
    #  STOP & HOME
    # ═════════════════════════════════════════════════════════════════════════
    def stop_all(self, go_home: bool = False):
        """Stop any recording/playback.  When go_home=True also move to Home.
        By default just aborts so live teleop resumes immediately.
        """
        was_recording = self.is_recording
        was_playing   = self.is_playing

        self.is_recording = False
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
        if was_recording:
            self._log.info("⏹️  Recording stopped")

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
    def get_info(self) -> Dict:
        frames = self.loaded_frames or self._frames
        if not frames:
            return {"loaded": False, "waypoints": 0, "duration": 0.0}
        dur = frames[-1]["timeStamp"] - frames[0]["timeStamp"]
        return {
            "loaded": bool(self.loaded_frames),
            "name": self.loaded_name,
            "waypoints": len(frames),
            "duration": round(dur, 2),
        }

    # ═════════════════════════════════════════════════════════════════════════
    #  INTERNAL
    # ═════════════════════════════════════════════════════════════════════════
    def _save_json(self, path: str, frames: List[Dict]) -> str:
        if not frames:
            self._log.error("No frames to save")
            return ""
        try:
            with open(path, "w") as f:
                json.dump({"frames": frames, "events": self._events}, f, indent=4)
            self._log.info(f"💾 Saved {len(frames)} frames, {len(self._events)} events → {path}")
            return path
        except Exception as e:
            self._log.error(f"Save failed: {e}")
            return ""
