#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MG400 Monitor Bridge — Ubuntu/ROS2 Side
========================================
Bridges ROS 2 topics → UDP telemetry at 50Hz for the Mac monitor GUI.
Also receives commands from Mac and publishes back to ROS 2.

Port 5556: Telemetry OUT  (UDP broadcast → Mac)
Port 5557: Commands  IN   (UDP from Mac → ROS)

Run:
    cd ~/project_teleop_ws
    source install/setup.bash
    python3 project_teleop/monitor_bridge.py

Env vars (all optional):
    BRIDGE_BROADCAST_IP   default: 255.255.255.255
    BRIDGE_TELEMETRY_PORT default: 5556
    BRIDGE_CMD_PORT       default: 5557
    BRIDGE_HZ             default: 50
"""

import sys
import os
import socket
import json
import time
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32MultiArray, Int64, Int32, String

# ── Topic names (fallback if mg400_controller package not importable) ─────────
try:
    from mg400_controller.common.config.motion_config import (
        UNITY_TOPIC, SUCTION_TOPIC, LIGHT_TOPIC,
        DO_STATUS_TOPIC, ROBOT_MODE_TOPIC, ERROR_STATUS_TOPIC,
    )
except ImportError:
    UNITY_TOPIC       = "/unity/joint_cmd"
    SUCTION_TOPIC     = "/vr/suction_cmd"
    LIGHT_TOPIC       = "/mg400/light_cmd"
    DO_STATUS_TOPIC   = "/mg400/do_status"
    ROBOT_MODE_TOPIC  = "/mg400/robot_mode"
    ERROR_STATUS_TOPIC = "/mg400/error_status"

ACTUAL_TOPIC    = "/joint_states"
SENT_TOPIC      = "/teleop/sent_command"
TOOL_ACT_TOPIC  = "/mg400/tool_vector_actual"
TOOL_TGT_TOPIC  = "/mg400/tool_vector_target"
UNITY_XYZ_TOPIC = "/teleop/unity_xyz"
FLANGE_TOPIC    = "/robot/flange_actual"
TOOL_IDX_TOPIC  = "/robot/tool_index"

# ── Config ────────────────────────────────────────────────────────────────────
BCAST_IP   = os.environ.get("BRIDGE_BROADCAST_IP",   "255.255.255.255")
TARGET_IP  = os.environ.get("BRIDGE_TARGET_IP",      "")  # unicast to Mac
TELEM_PORT = int(os.environ.get("BRIDGE_TELEMETRY_PORT", "5556"))
CMD_PORT   = int(os.environ.get("BRIDGE_CMD_PORT",       "5557"))
SEND_HZ    = int(os.environ.get("BRIDGE_HZ",             "50"))


class MonitorBridge(Node):
    def __init__(self):
        super().__init__("monitor_bridge")

        # ── Shared telemetry state ────────────────────────────────────────────
        self.actual     = [0.0] * 4
        self.unity      = [0.0] * 4
        self._last_playback_t = 0.0  # perf_counter of last /teleop/playback_unity msg
        self.traj_preview         = None   # full trajectory for background dots (sent once)
        self._traj_preview_pending = False  # True until sent to Mac
        self.sent       = [0.0] * 4
        self.tool_act   = [0.0] * 6
        self.tool_tgt   = [0.0] * 6
        self.unity_xyz  = [0.0] * 6
        self.flange     = [0.0] * 6
        self.tool_idx   = -1
        self.do_status  = 0
        self.robot_mode = 7
        self.error_stat = 0

        # ── Target IP for telemetry (unicast preferred, broadcast fallback) ──
        self._target_ip = TARGET_IP if TARGET_IP else None
        self._mac_discovered = False

        # ── Frequency tracking (rolling 1s window) ───────────────────────────
        self._counts = {
            "UNITY TARGET": 0, "SENT COMMAND": 0,
            "ACTUAL FEEDBACK": 0, "TOOL VECTOR": 0,
            "ROBOT MODE": 0, "ROBOT ERROR": 0, "DIGITAL IO": 0,
        }
        self._freq     = {k: 0.0 for k in self._counts}
        self._last_hz  = time.perf_counter()

        # ── UDP sockets ───────────────────────────────────────────────────────
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)

        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.settimeout(0.5)
        self._rx.bind(("0.0.0.0", CMD_PORT))

        # ── ROS publishers ────────────────────────────────────────────────────
        self.pub_dash    = self.create_publisher(String,          "/robot/dashboard_cmd", 10)
        self.pub_suction = self.create_publisher(Bool,            SUCTION_TOPIC, 10)
        self.pub_light   = self.create_publisher(Int32MultiArray, LIGHT_TOPIC, 10)

        # ── ROS subscriptions ─────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        def sub(topic, typ, cb, qos=10):
            return self.create_subscription(typ, topic, cb, qos)

        sub(ACTUAL_TOPIC,     JointState,        self._cb_actual)
        sub(UNITY_TOPIC,      JointState,        self._cb_unity_vr, qos_be)
        sub(SENT_TOPIC,       JointState,        self._cb_sent)
        sub("/teleop/playback_unity", JointState, self._cb_unity_playback)
        sub("/teleop/traj_preview",   String,    self._cb_traj_preview)
        sub(TOOL_ACT_TOPIC,   Float64MultiArray, lambda m: self._f64(m, 'tool_act'))
        sub(TOOL_TGT_TOPIC,   Float64MultiArray, lambda m: self._f64(m, 'tool_tgt'))
        sub(UNITY_XYZ_TOPIC,  Float64MultiArray, lambda m: self._f64(m, 'unity_xyz'))
        sub(FLANGE_TOPIC,     Float64MultiArray, lambda m: self._f64(m, 'flange'))
        sub(TOOL_IDX_TOPIC,   Int32,  lambda m: setattr(self, 'tool_idx', int(m.data)))
        sub(DO_STATUS_TOPIC,  Int64,  self._cb_do)
        sub(ROBOT_MODE_TOPIC,    Int32, self._cb_mode)
        sub(ERROR_STATUS_TOPIC,  Int32, self._cb_err)

        # ── Background threads ────────────────────────────────────────────────
        self._running = True
        threading.Thread(target=self._telem_loop, daemon=True).start()
        threading.Thread(target=self._cmd_loop,   daemon=True).start()

        tgt_desc = self._target_ip if self._target_ip else f"broadcast({BCAST_IP})"
        self.get_logger().info(
            f"🔗 Monitor Bridge ready — telemetry → {tgt_desc}:{TELEM_PORT} | cmds ← :{CMD_PORT}")

    # ── ROS callbacks ─────────────────────────────────────────────────────────
    def _cb_actual(self, msg):
        self._counts["ACTUAL FEEDBACK"] += 1
        if len(msg.position) >= 9:
            self.actual = list(np.degrees([msg.position[i] for i in (0, 1, 3, 8)]))

    def _cb_unity_vr(self, msg):
        """VR/keyboard unity target — ignored for 0.5 s after receiving a playback message
        so that stale queued VR messages cannot overwrite the current playback waypoint."""
        if time.perf_counter() - self._last_playback_t < 0.5:
            return
        self._counts["UNITY TARGET"] += 1
        if len(msg.position) >= 4:
            self.unity = list(np.degrees(msg.position[:4]))

    def _cb_traj_preview(self, msg):
        """Full trajectory waypoints published once at Preview start."""
        import json as _json
        try:
            self.traj_preview = _json.loads(msg.data)
            self._traj_preview_pending = True
        except Exception:
            pass

    def _cb_unity_playback(self, msg):
        """T&R playback waypoint — always accepted; stamps _last_playback_t."""
        self._last_playback_t = time.perf_counter()
        self._counts["UNITY TARGET"] += 1
        if len(msg.position) >= 4:
            self.unity = list(np.degrees(msg.position[:4]))

    @property
    def _tr_active(self):
        return (time.perf_counter() - self._last_playback_t) < 1.5

    def _cb_sent(self, msg):
        self._counts["SENT COMMAND"] += 1
        if len(msg.position) >= 4:
            self.sent = list(np.degrees(msg.position[:4]))

    def _f64(self, msg, attr):
        if attr == 'tool_act':
            self._counts["TOOL VECTOR"] += 1
        if len(msg.data) >= 6:
            setattr(self, attr, list(msg.data))

    def _cb_mode(self, msg):
        self._counts["ROBOT MODE"] += 1
        self.robot_mode = int(msg.data)

    def _cb_err(self, msg):
        self._counts["ROBOT ERROR"] += 1
        self.error_stat = int(msg.data)

    def _cb_do(self, msg):
        self._counts["DIGITAL IO"] += 1
        self.do_status = int(msg.data)

    # ── Telemetry sender loop (50 Hz) ─────────────────────────────────────────
    def _telem_loop(self):
        interval  = 1.0 / SEND_HZ
        next_t    = time.perf_counter() + interval
        hz_period = 1.0
        next_hz   = time.perf_counter() + hz_period

        while self._running:
            now = time.perf_counter()

            # Recompute Hz every 1 s
            if now >= next_hz:
                dt = now - self._last_hz
                for k in self._counts:
                    self._freq[k] = round(self._counts[k] / dt, 2) if dt > 0 else 0.0
                    self._counts[k] = 0
                self._last_hz = now
                next_hz = now + hz_period

            # Build and broadcast packet
            try:
                pkt = json.dumps({
                    "ts":         time.time(),
                    "tr_active":  self._tr_active,
                    "traj_preview": self.traj_preview if self._traj_preview_pending else None,
                    "actual":     self.actual,
                    "unity":      self.unity,
                    "predicted":  self.unity,
                    "sent":       self.sent,
                    "tool_act":   self.tool_act,
                    "tool_tgt":   self.tool_tgt,
                    "unity_xyz":  self.unity_xyz,
                    "flange":     self.flange,
                    "tool_idx":   self.tool_idx,
                    "do_status":  self.do_status,
                    "robot_mode": self.robot_mode,
                    "error_stat": self.error_stat,
                    "freq":       self._freq,
                }, separators=(',', ':')).encode()
                dest = self._target_ip if self._target_ip else BCAST_IP
                self._tx.sendto(pkt, (dest, TELEM_PORT))
                if self._traj_preview_pending:
                    self._traj_preview_pending = False
            except Exception as e:
                self.get_logger().warn(f"Telem TX error: {e}", throttle_duration_sec=5)

            # Precise sleep
            sl = next_t - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            next_t += interval
            # Guard against burst if we fall behind
            if time.perf_counter() > next_t + interval:
                next_t = time.perf_counter() + interval

    # ── Command receiver loop ─────────────────────────────────────────────────
    def _cmd_loop(self):
        while self._running:
            try:
                raw, addr = self._rx.recvfrom(4096)
                cmd    = json.loads(raw.decode())
                action = cmd.get("action", "")
                data   = cmd.get("data")

                if action == "hello":
                    # Mac GUI announces itself — switch to unicast
                    self._target_ip = addr[0]
                    if not self._mac_discovered:
                        self._mac_discovered = True
                        self.get_logger().info(
                            f"🖥️  Mac GUI discovered at {addr[0]} — switching to unicast")
                    continue

                if action == "dashboard":
                    msg = String(); msg.data = str(data)
                    self.pub_dash.publish(msg)
                    self.get_logger().info(f"📡 Dashboard cmd ← {addr[0]}: {data}")

                elif action == "suction":
                    msg = Bool(); msg.data = bool(data)
                    self.pub_suction.publish(msg)

                elif action == "light":
                    msg = Int32MultiArray()
                    msg.data = [int(data[0]), int(data[1])]
                    self.pub_light.publish(msg)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"Cmd RX error: {e}")

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main():
    rclpy.init()
    bridge = MonitorBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down…")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
