#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Trajectory Recorder & Sequencer — Teach-and-Repeat for MG400
=============================================================
Handles the full lifecycle:

1. **Record**  — capture robot joint positions while user teleoperates.
2. **Save**    — persist recorded data as Unity-compatible JSON
                 (frames: [{timeStamp, j1..j4}]).
3. **Load**    — read a saved JSON trajectory (Unity or native format).
4. **Preview** — temporal sequencer that replays a trajectory on the real
                 robot, computing per-segment SpeedJ to match the original
                 timing.  Uses CP=100 for smooth continuous-path motion and
                 gates live teleop commands during playback.
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
from typing import List, Dict, Optional, Callable


# ── Home position (degrees) ──────────────────────────────────────────────────
HOME_JOINTS_DEG = [0.0, 0.0, 0.0, 0.0]

# ── Trajectory storage directory ─────────────────────────────────────────────
TRAJ_DIR = os.path.expanduser("~/project_teleop_ws/trajectories")

# ── Playback decimation ───────────────────────────────────────────────────────
# Recording is done at 50 Hz (20ms per frame).  Sending every raw frame gives
# tiny per-segment deltas → very low SpeedJ → robot crawls.  We decimate to
# PLAYBACK_HZ target rate so each segment has enough motion for a real speed.
PLAYBACK_MIN_DT = 1.0 / 30  # seconds — minimum gap between sent waypoints (≈30 Hz)


class TrajectoryRecorder:
    """
    Teach-and-Repeat sequencer for MG400.

    Parameters
    ----------
    command_send_fn : callable(str) -> bool
        Function that sends a raw TCP command string to the robot
        (e.g. ``sender.send``).
    logger
        ROS-compatible logger with .info / .warn / .error methods.
    get_position_fn : callable() -> np.ndarray | None, optional
        Returns current joint angles in *radians* (4,).
        Required for recording mode.
    """

    def __init__(self, command_send_fn: Callable, logger,
                 get_position_fn: Optional[Callable] = None,
                 waypoint_callback: Optional[Callable] = None):
        self._send = command_send_fn
        self._log  = logger
        self._get_pos = get_position_fn
        self._waypoint_cb = waypoint_callback  # called with np.array(4,) radians each played waypoint

        # ── State ────────────────────────────────────────────────────────────
        self.is_recording  = False
        self.is_playing    = False
        self._stop_flag    = threading.Event()
        self._play_thread: Optional[threading.Thread] = None
        self._block_until  = 0.0  # perf_counter: suppress teleop until this time (after go_home)

        # ── Recorded data (native format) ────────────────────────────────────
        self._frames: List[Dict] = []   # [{timeStamp, j1..j4}] degrees
        self._rec_t0 = 0.0

        # ── Loaded trajectory (ready for playback) ───────────────────────────
        self.loaded_frames: List[Dict] = []
        self.loaded_name: str = ""

        os.makedirs(TRAJ_DIR, exist_ok=True)

    # ═════════════════════════════════════════════════════════════════════════
    #  RECORD
    # ═════════════════════════════════════════════════════════════════════════
    def start_recording(self):
        if self.is_playing:
            self._log.warn("⚠️  Cannot record while playing")
            return
        self.is_recording = True
        self._block_until = 0.0  # cancel any pending homing suppression
        self._frames = []
        self._rec_t0 = time.time()
        self._log.info("🔴 Recording started")

    def record_tick(self, target_q_rad):
        """Call at ~50 Hz from the control loop while recording.
        Records the intended target from Unity/VR rather than the actual robot position
        to ensure playback matches the intended VR trajectory perfectly.
        """
        if not self.is_recording or target_q_rad is None:
            return
        
        q_deg = np.degrees(target_q_rad[:4])
        self._frames.append({
            "timeStamp": time.time() - self._rec_t0,
            "j1": float(q_deg[0]),
            "j2": float(q_deg[1]),
            "j3": float(q_deg[2]),
            "j4": float(q_deg[3]),
        })

    def stop_recording(self) -> List[Dict]:
        self.is_recording = False
        self._log.info(f"⏹️  Recording stopped — {len(self._frames)} frames, "
                       f"{self._frames[-1]['timeStamp']:.1f}s" if self._frames else "⏹️  Recording stopped — 0 frames")
        return self._frames

    # ═════════════════════════════════════════════════════════════════════════
    #  SAVE / LOAD
    # ═════════════════════════════════════════════════════════════════════════
    def save_temp(self) -> str:
        """Save last recording to temp_trajectory.json and return the path."""
        path = os.path.join(TRAJ_DIR, "temp_trajectory.json")
        return self._save_json(path, self._frames)

    def save_as(self, name: str) -> str:
        """Save last recording with a user-supplied name."""
        if not name.endswith(".json"):
            name += ".json"
        path = os.path.join(TRAJ_DIR, name)
        return self._save_json(path, self._frames)

    def save_from_unity_json(self, json_str: str) -> str:
        """Receive raw JSON from Unity /unity/trajectory_data and save."""
        try:
            data = json.loads(json_str)
            frames = data if isinstance(data, list) else data.get("frames", [])
            self._frames = frames
            return self.save_temp()
        except Exception as e:
            self._log.error(f"Failed to parse Unity trajectory JSON: {e}")
            return ""

    def load(self, name: str) -> bool:
        """Load a trajectory file by name from TRAJ_DIR."""
        if not name.endswith(".json"):
            name += ".json"
        path = os.path.join(TRAJ_DIR, name)
        if not os.path.isfile(path):
            self._log.error(f"Trajectory file not found: {path}")
            return False
        try:
            with open(path, "r") as f:
                data = json.load(f)
            frames = data if isinstance(data, list) else data.get("frames", [])
            if not frames:
                self._log.error("Trajectory file is empty")
                return False
            self.loaded_frames = frames
            self.loaded_name = name
            dur = frames[-1]["timeStamp"] - frames[0]["timeStamp"]
            self._log.info(f"📂 Loaded {name}: {len(frames)} frames, {dur:.1f}s")
            return True
        except Exception as e:
            self._log.error(f"Failed to load trajectory: {e}")
            return False

    def list_files(self) -> List[str]:
        """Return list of .json trajectory files."""
        try:
            return sorted(f for f in os.listdir(TRAJ_DIR) if f.endswith(".json"))
        except Exception:
            return []

    # ═════════════════════════════════════════════════════════════════════════
    #  PREVIEW (Temporal Sequencer)
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

    def _play_worker(self):
        """Sequencer thread: sends JointMovJ commands with computed SpeedJ."""
        raw = self.loaded_frames

        # ── Decimate: keep only frames >= PLAYBACK_MIN_DT apart ──────────────
        # Recording at 50 Hz gives tiny per-segment deltas → SpeedJ=1-2% →
        # robot crawls even though timing math is correct.  Merging to ~10 Hz
        # gives ~5x larger deltas per segment → SpeedJ 5x higher → correct speed.
        frames = [raw[0]]
        for f in raw[1:]:
            if f["timeStamp"] - frames[-1]["timeStamp"] >= PLAYBACK_MIN_DT:
                frames.append(f)
        if frames[-1] is not raw[-1]:
            frames.append(raw[-1])  # always include final pose

        n = len(frames)
        t0_traj = frames[0]["timeStamp"]
        t_wall_start = time.perf_counter()
        self._log.info(f"▶️  Preview start — {n} waypoints (decimated from {len(raw)})")

        for i, fr in enumerate(frames):
            if self._stop_flag.is_set():
                break

            j = [fr["j1"], fr["j2"], fr["j3"], fr["j4"]]

            # ── Compute SpeedJ for this segment ──────────────────────────────
            if i < n - 1:
                fr_next = frames[i + 1]
                dt = max(fr_next["timeStamp"] - fr["timeStamp"], 0.001)
                j_next = [fr_next["j1"], fr_next["j2"], fr_next["j3"], fr_next["j4"]]
                max_delta = max(abs(j_next[k] - j[k]) for k in range(4))
                required_dps = max_delta / dt
                speed_pct = max(5, min(100, int(math.ceil(required_dps / 200.0 * 100))))
            else:
                speed_pct = 20

            # ── Build and send command ───────────────────────────────────────
            cmd = (f"JointMovJ({j[0]:.4f},{j[1]:.4f},{j[2]:.4f},{j[3]:.4f},"
                   f"SpeedJ={speed_pct},AccJ=100,CP=100)")
            self._send(cmd)

            # ── Notify waypoint observers (e.g. ROS graph publishers) ────────
            if self._waypoint_cb is not None:
                try:
                    self._waypoint_cb(np.radians(j))
                except Exception:
                    pass

            # ── Check stop immediately after send ────────────────────────────
            if self._stop_flag.is_set():
                break

            # ── Wait until the next frame's wall-clock time ──────────────────
            if i < n - 1:
                next_rel = frames[i + 1]["timeStamp"] - t0_traj
                sleep_dur = (t_wall_start + next_rel) - time.perf_counter()
                if sleep_dur > 0:
                    if self._stop_flag.wait(timeout=sleep_dur):
                        break

            # Progress log every 25%
            pct = int((i + 1) / n * 100)
            if pct % 25 == 0 and pct > 0:
                self._log.info(f"   Preview: {pct}%")

        self.is_playing = False
        if self._stop_flag.is_set():
            self._log.info("⏹️  Playback stopped — returning to real-time teleop")
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
        self._stop_flag.set()

        if self._play_thread and self._play_thread.is_alive():
            self._play_thread.join(timeout=2.0)
        self.is_playing = False

        if was_recording:
            self._log.info("⏹️  Recording stopped")
        if was_playing:
            self._log.info("⏹️  Playback stopped — live teleop resumed")

        if go_home:
            self._go_home()
            self._block_until = time.perf_counter() + 5.0  # 5 s for robot to reach home

    def _go_home(self):
        """Send robot to Home position (0, 0, 0, 0)."""
        h = HOME_JOINTS_DEG
        cmd = f"JointMovJ({h[0]:.4f},{h[1]:.4f},{h[2]:.4f},{h[3]:.4f},SpeedJ=30,AccJ=50,CP=0)"
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
                json.dump({"frames": frames}, f, indent=2)
            self._log.info(f"💾 Saved {len(frames)} frames → {path}")
            return path
        except Exception as e:
            self._log.error(f"Save failed: {e}")
            return ""
