#!/usr/bin/env python3
"""
Teach & Repeat Manager for MG400 Simulator.
Records joint-space waypoints and generates smooth or P2P trajectories.
Future: export as trajectory_msgs/JointTrajectory for ROS 2.
"""

import json
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
