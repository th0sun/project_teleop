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
from typing import List, Dict, Optional, Callable


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
    waypoint_callback : callable(np.ndarray) -> None, optional
        Called with radians (4,) for each played-back waypoint.
    """

    def __init__(self, command_send_fn: Callable, logger,
                 get_position_fn: Optional[Callable] = None,
                 waypoint_callback: Optional[Callable] = None):
        self._send = command_send_fn
        self._log  = logger
        self._get_pos = get_position_fn
        self._waypoint_cb = waypoint_callback

        # ── State ────────────────────────────────────────────────────────────
        self.is_recording  = False
        self.is_playing    = False
        self._stop_flag    = threading.Event()
        self._play_thread: Optional[threading.Thread] = None
        self._block_until  = 0.0  # perf_counter: suppress teleop until this time

        # ── Recorded data ────────────────────────────────────────────────────
        self._frames: List[Dict] = []   # [{timeStamp, j1..j4}] degrees
        self._rec_t0 = 0.0
        self._last_rec_t = 0.0          # timestamp of last stored frame
        self._last_rec_q = np.zeros(4)   # joint values of last stored frame

        # ── Loaded trajectory (ready for playback) ───────────────────────────
        self.loaded_frames: List[Dict] = []
        self.loaded_name: str = ""

        os.makedirs(TRAJ_DIR, exist_ok=True)

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
        self._rec_t0 = time.time()
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
        now = time.time() - self._rec_t0

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
                    now = time.time() - self._rec_t0
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
    #  PREVIEW (Temporal Sequencer via Interpolation)
    # ═════════════════════════════════════════════════════════════════════════
    def start_preview(self):
        """Begin playing back loaded trajectory using the real-time control loop."""
        if not self.loaded_frames:
            self._log.error("No trajectory loaded for preview")
            return
        if self.is_playing:
            self._log.warn("Already playing")
            return
        self.is_playing = True
        self._playback_t0 = time.perf_counter()
        self._playback_idx = 0
        n = len(self.loaded_frames)
        t_traj = self.loaded_frames[0]["timeStamp"]
        self._log.info(f"▶️  Preview start — {n} waypoints, "
                       f"{self.loaded_frames[-1]['timeStamp'] - t_traj:.1f}s")

    def get_playback_target(self, now_perf: float) -> Optional[np.ndarray]:
        """Called at 50Hz from control loop. Returns interpolated target (radians)."""
        if not self.is_playing or not self.loaded_frames:
            return None
        
        frames = self.loaded_frames
        n = len(frames)
        t_traj = now_perf - self._playback_t0 + frames[0]["timeStamp"]
        
        # End of trajectory
        if t_traj >= frames[-1]["timeStamp"]:
            self.is_playing = False
            self._log.info("✅ Preview complete")
            fr = frames[-1]
            return np.radians([fr["j1"], fr["j2"], fr["j3"], fr["j4"]])

        # Find current segment
        while self._playback_idx < n - 2 and frames[self._playback_idx + 1]["timeStamp"] < t_traj:
            self._playback_idx += 1
            
        fr0 = frames[self._playback_idx]
        fr1 = frames[self._playback_idx + 1]
        
        dt = fr1["timeStamp"] - fr0["timeStamp"]
        if dt <= 0:
            return np.radians([fr0["j1"], fr0["j2"], fr0["j3"], fr0["j4"]])
            
        ratio = (t_traj - fr0["timeStamp"]) / dt
        ratio = max(0.0, min(1.0, ratio))
        
        # Linear interpolate
        j = []
        for k in ["j1", "j2", "j3", "j4"]:
            j.append(fr0[k] + ratio * (fr1[k] - fr0[k]))
            
        return np.radians(j)

    # ═════════════════════════════════════════════════════════════════════════
    #  STOP & HOME
    # ═════════════════════════════════════════════════════════════════════════
    def stop_all(self, go_home: bool = False):
        """Stop any recording/playback.  When go_home=True also move to Home."""
        was_recording = self.is_recording
        was_playing   = self.is_playing

        self.is_recording = False
        self.is_playing = False

        if was_recording:
            self._log.info("⏹️  Recording stopped")
        if was_playing:
            self._log.info("⏹️  Playback stopped")

        if go_home:
            self._go_home()
            self._block_until = time.perf_counter() + 5.0

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
                json.dump({"frames": frames}, f, indent=4)
            self._log.info(f"💾 Saved {len(frames)} frames → {path}")
            return path
        except Exception as e:
            self._log.error(f"Save failed: {e}")
            return ""
