#!/usr/bin/env python3
"""NDJSON bridge for the SwiftUI Mac app.

Subscribes to the topics the Mac app needs and writes one JSON object per
line on stdout. The SwiftUI side spawns this through `docker exec` and
parses the stream as it arrives.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String

# The Mac app is a viewer/control surface, not the realtime controller. Keep
# bridge output below UI frame rate so /joint_states and /unity/joint_cmd do not
# force SwiftUI/SceneKit to repaint at the combined ROS topic rate.
JOINT_DEG_DEDUP = float(os.environ.get("MAC_APP_BRIDGE_MIN_DEG", "0.005"))
JOINT_MAX_HZ = float(os.environ.get("MAC_APP_BRIDGE_JOINT_HZ", "125"))
UNITY_MAX_HZ = float(os.environ.get("MAC_APP_BRIDGE_UNITY_HZ", "125"))
FORCE_EMIT_DEG = float(os.environ.get("MAC_APP_BRIDGE_FORCE_DEG", "0.05"))

ACTIVE_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4")

# Active MG400 joint name candidates, matched in priority order.
# Mirrors tools/mg400_simulator/core/unity_tcp_bridge.py:active_joint_positions_deg.
ACTIVE_NAME_CANDIDATES = (
    ("mg400_j1", "mg400_j2_1", "mg400_j3", "mg400_j5"),
    ("joint1", "joint2", "joint3", "joint4"),
)
FALLBACK_URDF_INDICES = (0, 1, 3, 8)


def _active_joint_positions(names: list[str], positions: list[float]) -> list[float]:
    """Return active MG400 joints [J1,J2,J3,J4].

    /joint_states uses the full URDF joint list including passive mimics, so
    this resolves by name first (URDF or Unity contract) and falls back to
    known URDF indices (0,1,3,8) instead of blindly slicing the first four.
    """
    if len(positions) < 4:
        return []

    if names:
        by_name = {name: float(pos) for name, pos in zip(names, positions)}
        for candidate in ACTIVE_NAME_CANDIDATES:
            if all(name in by_name for name in candidate):
                return [float(by_name[name]) for name in candidate]

    if len(positions) >= 9:
        return [float(positions[i]) for i in FALLBACK_URDF_INDICES]

    return [float(value) for value in positions[:4]]


def _deg(values_rad: list[float]) -> list[float]:
    return [math.degrees(float(value)) for value in values_rad]


class MacAppBridge(Node):
    def __init__(self) -> None:
        super().__init__("mac_app_bridge")
        latest_best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, latest_best_effort_qos)
        # The Mac app is only a visual/control surface.  For target display,
        # stale reliable samples are worse than dropped samples: the cyan
        # robot must always show the newest Unity pose.
        self.create_subscription(JointState, "/unity/joint_cmd", self._on_unity_joint_cmd, latest_best_effort_qos)
        self.create_subscription(String, "/unity/teleop_sample", self._on_unity_teleop_sample, latest_best_effort_qos)
        self.create_subscription(Int32, "/mg400/robot_mode", self._on_mode, 10)
        self.create_subscription(Int32, "/mg400/error_status", self._on_error, 10)
        self.create_subscription(Bool, "/vr/suction_cmd", self._on_suction, 10)
        self.create_subscription(Bool, "/mg400/light_cmd", self._on_light, 10)
        self._last_joint_emit_t = 0.0
        self._last_joint_emit_deg: list[float] = []
        self._last_unity_emit_t = 0.0
        self._last_unity_emit_deg: list[float] = []
        self._last_scalar: dict[str, object] = {}
        self._emit({"t": "ready"})

    def _emit(self, payload: dict) -> None:
        try:
            sys.stdout.write(json.dumps(payload) + "\n")
            sys.stdout.flush()
        except (BrokenPipeError, OSError):
            # SwiftUI side closed the docker exec pipe (app quit or restarted
            # the bridge). Exit immediately so we don't leave a zombie
            # subscriber inside the container stealing topic callbacks.
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(0)

    def _changed(self, prev_deg: list[float], new_deg: list[float]) -> bool:
        if len(prev_deg) != len(new_deg):
            return True
        return any(abs(a - b) >= JOINT_DEG_DEDUP for a, b in zip(new_deg, prev_deg))

    def _should_emit(self, prev_deg: list[float], new_deg: list[float], last_emit_t: float, max_hz: float) -> bool:
        if not self._changed(prev_deg, new_deg):
            return False
        if max_hz <= 0:
            return True
        now = time.monotonic()
        if now - last_emit_t >= (1.0 / max_hz):
            return True
        if len(prev_deg) != len(new_deg):
            return True
        return any(abs(a - b) >= FORCE_EMIT_DEG for a, b in zip(new_deg, prev_deg))

    def _on_joint_states(self, msg: JointState) -> None:
        positions = _active_joint_positions(list(msg.name), list(msg.position))
        if not positions:
            return
        positions_deg = _deg(positions)
        if not self._should_emit(self._last_joint_emit_deg, positions_deg, self._last_joint_emit_t, JOINT_MAX_HZ):
            return
        self._last_joint_emit_deg = positions_deg
        self._last_joint_emit_t = time.monotonic()
        self._emit(
            {
                "t": "joint_states",
                "name": list(ACTIVE_JOINT_NAMES),
                "position": positions,
                "position_deg": positions_deg,
            }
        )

    def _on_unity_joint_cmd(self, msg: JointState) -> None:
        positions = _active_joint_positions(list(msg.name), list(msg.position))
        if not positions:
            return
        self._emit_unity_positions(positions)

    def _on_unity_teleop_sample(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        positions = payload.get("joints_ik_rad")
        if not isinstance(positions, list) or len(positions) < 4:
            positions = [
                payload.get("j1_ik_rad"),
                payload.get("j2_ik_rad"),
                payload.get("j3_ik_rad"),
                payload.get("j4_ik_rad"),
            ]
        try:
            positions_rad = [float(value) for value in positions[:4]]
        except (TypeError, ValueError):
            return
        if len(positions_rad) < 4:
            return
        self._emit_unity_positions(positions_rad)

    def _emit_unity_positions(self, positions: list[float]) -> None:
        positions_deg = _deg(positions)
        if not self._should_emit(self._last_unity_emit_deg, positions_deg, self._last_unity_emit_t, UNITY_MAX_HZ):
            return
        self._last_unity_emit_deg = positions_deg
        self._last_unity_emit_t = time.monotonic()
        self._emit(
            {
                "t": "unity_joint_cmd",
                "name": list(ACTIVE_JOINT_NAMES),
                "position": positions,
                "position_deg": positions_deg,
            }
        )

    def _on_mode(self, msg: Int32) -> None:
        self._emit_scalar("robot_mode", int(msg.data))

    def _on_error(self, msg: Int32) -> None:
        self._emit_scalar("error_status", int(msg.data))

    def _on_suction(self, msg: Bool) -> None:
        self._emit_scalar("suction", bool(msg.data))

    def _on_light(self, msg: Bool) -> None:
        self._emit_scalar("light", bool(msg.data))

    def _emit_scalar(self, kind: str, value: object) -> None:
        if self._last_scalar.get(kind) == value:
            return
        self._last_scalar[kind] = value
        self._emit({"t": kind, "value": value})


def main() -> None:
    rclpy.init()
    node = MacAppBridge()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
