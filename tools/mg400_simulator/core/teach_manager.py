#!/usr/bin/env python3
"""
Teach & Repeat Manager for MG400 Simulator.
Records joint-space waypoints and generates smooth or P2P trajectories.
Future: export as trajectory_msgs/JointTrajectory for ROS 2.
"""

import json
import os
import numpy as np
from dataclasses import dataclass, asdict
from typing import List, Optional


@dataclass
class Waypoint:
    name: str
    joints: List[float]   # [j1, j2, j3, j4] in degrees
    duration: float       # seconds to reach this point from previous


class TeachManager:

    def __init__(self):
        self._waypoints: List[Waypoint] = []

    # ── CRUD ──────────────────────────────────────────────────────────────────

    def add_waypoint(self, joints, name: str = '', duration: float = 1.0) -> int:
        if not name:
            name = f'P{len(self._waypoints) + 1:03d}'
        wp = Waypoint(name=name, joints=list(float(j) for j in joints),
                      duration=float(duration))
        self._waypoints.append(wp)
        return len(self._waypoints) - 1

    def update_waypoint(self, idx: int, joints=None, name: str = None,
                        duration: float = None):
        wp = self._waypoints[idx]
        if joints is not None:
            wp.joints = list(float(j) for j in joints)
        if name is not None:
            wp.name = name
        if duration is not None:
            wp.duration = float(duration)

    def delete_waypoint(self, idx: int):
        self._waypoints.pop(idx)

    def move_up(self, idx: int):
        if idx > 0:
            self._waypoints[idx - 1], self._waypoints[idx] = \
                self._waypoints[idx], self._waypoints[idx - 1]

    def move_down(self, idx: int):
        if idx < len(self._waypoints) - 1:
            self._waypoints[idx], self._waypoints[idx + 1] = \
                self._waypoints[idx + 1], self._waypoints[idx]

    def clear(self):
        self._waypoints.clear()

    def count(self) -> int:
        return len(self._waypoints)

    def get(self, idx: int) -> Waypoint:
        return self._waypoints[idx]

    def get_all(self) -> List[Waypoint]:
        return list(self._waypoints)

    # ── Trajectory generation ─────────────────────────────────────────────────

    def generate_p2p(self, speed_mult: float = 1.0, fps: int = 50) -> List[np.ndarray]:
        """
        Point-to-point: linear joint interpolation between waypoints.
        Returns list of joint arrays (degrees).
        """
        if len(self._waypoints) < 1:
            return []
        frames = []
        for i in range(1, len(self._waypoints)):
            j0  = np.array(self._waypoints[i - 1].joints)
            j1  = np.array(self._waypoints[i].joints)
            dur = self._waypoints[i].duration / max(speed_mult, 0.01)
            n   = max(2, int(dur * fps))
            for k in range(n):
                t = k / (n - 1)
                frames.append(j0 + (j1 - j0) * t)
        return frames

    def generate_smooth(self, speed_mult: float = 1.0, fps: int = 50) -> List[np.ndarray]:
        """
        Smooth: cubic ease-in/out between waypoints.
        Returns list of joint arrays (degrees).
        """
        if len(self._waypoints) < 1:
            return []
        frames = []
        for i in range(1, len(self._waypoints)):
            j0  = np.array(self._waypoints[i - 1].joints)
            j1  = np.array(self._waypoints[i].joints)
            dur = self._waypoints[i].duration / max(speed_mult, 0.01)
            n   = max(2, int(dur * fps))
            for k in range(n):
                t = k / (n - 1)
                t_s = t * t * (3.0 - 2.0 * t)   # smoothstep
                frames.append(j0 + (j1 - j0) * t_s)
        return frames

    # ── Persistence ───────────────────────────────────────────────────────────

    def save(self, filepath: str):
        data = {'waypoints': [asdict(wp) for wp in self._waypoints]}
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def load(self, filepath: str):
        with open(filepath, 'r') as f:
            data = json.load(f)
        self._waypoints = [Waypoint(**wp) for wp in data.get('waypoints', [])]

    def load_unity_trajectory(self, filepath: str) -> int:
        """Import a Unity TeachJobPublisher-style trajectory JSON file.

        Supported shapes:
        - {"frames": [{"timeStamp": ..., "j1": ..., "j2": ..., "j3": ..., "j4": ...}]}
        - {"trajectory": {"frames": [...]}}
        - raw frame list: [{"timeStamp": ..., ...}]

        Unity timestamps are often absolute app times, so they are converted to
        per-waypoint durations before storing in the simulator's program format.
        """
        with open(filepath, 'r') as f:
            data = json.load(f)

        frames = self._extract_unity_frames(data)
        if len(frames) < 1:
            raise ValueError("Unity trajectory JSON has no frames")

        source = os.path.basename(filepath)
        waypoints: List[Waypoint] = []
        prev_time = self._frame_time(frames[0], default=0.0)
        for i, frame in enumerate(frames):
            joints = self._frame_joints_deg(frame)
            if i == 0:
                duration = 0.0
            else:
                t = self._frame_time(frame, default=prev_time + 1.0)
                duration = max(0.05, float(t - prev_time))
                prev_time = t
            waypoints.append(Waypoint(
                name=f"{source}:{i + 1:03d}",
                joints=joints,
                duration=duration,
            ))

        self._waypoints = waypoints
        return len(self._waypoints)

    @staticmethod
    def _extract_unity_frames(data):
        if isinstance(data, list):
            return data
        if not isinstance(data, dict):
            raise ValueError("Unsupported Unity trajectory JSON root")
        if isinstance(data.get('frames'), list):
            return data['frames']
        trajectory = data.get('trajectory')
        if isinstance(trajectory, dict) and isinstance(trajectory.get('frames'), list):
            return trajectory['frames']
        raise ValueError("Could not find a frames list in Unity trajectory JSON")

    @staticmethod
    def _frame_time(frame, default=0.0) -> float:
        for key in ('timeStamp', 'timestamp', 'time', 't'):
            if key in frame:
                return float(frame[key])
        return float(default)

    @staticmethod
    def _frame_joints_deg(frame) -> List[float]:
        if all(k in frame for k in ('j1', 'j2', 'j3', 'j4')):
            return [float(frame[k]) for k in ('j1', 'j2', 'j3', 'j4')]

        positions = frame.get('positions')
        if isinstance(positions, list) and len(positions) >= 4:
            # JointTrajectory-style points are in radians.
            return [float(np.rad2deg(v)) for v in positions[:4]]

        raise ValueError(f"Frame is missing j1/j2/j3/j4: {frame}")

    # ── ROS 2 export ──────────────────────────────────────────────────────────

    def to_joint_trajectory_dict(self) -> dict:
        """
        Returns a dict compatible with trajectory_msgs/JointTrajectory.
        Positions are in radians. Use with ros_bridge to publish.
        """
        t = 0.0
        points = []
        for wp in self._waypoints:
            t += wp.duration
            points.append({
                'positions':       [np.deg2rad(j) for j in wp.joints],
                'velocities':      [0.0, 0.0, 0.0, 0.0],
                'time_from_start': t,
            })
        return {
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
            'points':       points,
        }

    def to_unity_trajectory_dict(self, filename: str = 'mg400_simulator_waypoints.json') -> dict:
        """
        Return the Unity teach-job trajectory shape used by TeachJobPublisher.

        This simulator records sparse waypoints.  Timestamps are synthesized
        from the waypoint durations so existing ROS-side validators can consume
        the payload, while the compiler can still retime motion later.
        """
        frames = []
        t = 0.0
        for i, wp in enumerate(self._waypoints):
            if i > 0:
                t += float(wp.duration)
            frames.append({
                'timeStamp': float(t),
                'j1': float(wp.joints[0]),
                'j2': float(wp.joints[1]),
                'j3': float(wp.joints[2]),
                'j4': float(wp.joints[3]),
            })
        return {
            'filename': filename,
            'teach_mode': 'waypoint',
            'source': 'mg400_simulator',
            'frames': frames,
            'events': [],
        }
