#!/usr/bin/env python3
"""
Optional ROS 2 bridge for the MG400 Simulator.
Connects to the live teleop_logic stack when ROS 2 is available.
Gracefully degrades to a no-op when rclpy is not found.
"""

import json
import threading
from typing import Optional, Callable, List

from core.unity_tcp_bridge import UnityTcpBridge

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


def is_ros_available() -> bool:
    return _ROS_AVAILABLE


class ROSBridge:
    """
    Thin wrapper around a minimal rclpy node.

    Callbacks are invoked from the ROS spin thread – callers must be
    thread-safe (Qt signals are).
    """

    def __init__(self,
                 on_joint_update: Optional[Callable[[List[float]], None]] = None,
                 on_status_update: Optional[Callable[[dict], None]] = None,
                 on_connection_change: Optional[Callable[[bool], None]] = None):
        self.on_joint_update       = on_joint_update
        self.on_status_update      = on_status_update
        self.on_connection_change  = on_connection_change

        self._node: Optional['_SimNode'] = None
        self._thread: Optional[threading.Thread] = None
        
        self._tcp_bridge: Optional[UnityTcpBridge] = None
        self.connected = False

    # ── Public API ───────────────────────────────────────────────────────────

    def start(self, host: str = '127.0.0.1', port: int = 10000) -> bool:
        if not _ROS_AVAILABLE:
            print("[ROSBridge] rclpy not found. Falling back to UnityTcpBridge mockup for macOS/Windows clients.")
            self._tcp_bridge = UnityTcpBridge(
                on_joint_update=self.on_joint_update,
                on_status_update=self.on_status_update,
                on_connection_change=self.on_connection_change
            )
            success = self._tcp_bridge.start(host, port)
            if success:
                self.connected = True
            return success
            
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = _SimNode(
                on_joint_update=self.on_joint_update,
                on_status_update=self.on_status_update,
            )
            self._thread = threading.Thread(target=self._spin, daemon=True,
                                            name='ros_sim_spin')
            self._thread.start()
            self.connected = True
            if self.on_connection_change:
                self.on_connection_change(True)
            return True
        except Exception as exc:
            print(f'[ROSBridge] start failed: {exc}')
            return False

    def stop(self):
        if self._tcp_bridge:
            self._tcp_bridge.stop()
            self._tcp_bridge = None
            self.connected = False
            return
            
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self.connected = False
        if self.on_connection_change:
            self.on_connection_change(False)

    def publish_joint_cmd(self, joints_deg: List[float]):
        """Publish joint command to /unity/joint_cmd (degrees)."""
        if self._tcp_bridge:
            self._tcp_bridge.publish_joint_cmd(joints_deg)
            return
            
        if not self.connected or self._node is None:
            return
        msg = Float64MultiArray()
        msg.data = [float(j) for j in joints_deg]
        self._node.pub_joint_cmd.publish(msg)

    def publish_dashboard_cmd(self, cmd: str):
        """Publish a raw dashboard command string, e.g. 'EnableRobot()'."""
        if self._tcp_bridge:
            self._tcp_bridge.publish_dashboard_cmd(cmd)
            return
            
        if not self.connected or self._node is None:
            return
        msg = String()
        msg.data = cmd
        self._node.pub_dashboard.publish(msg)

    def publish_control_mode(self, mode: str):
        """Publish control-mode selection, e.g. 'jointmovj'."""
        if self._tcp_bridge:
            self._tcp_bridge.publish_control_mode(mode)
            return
            
        if not self.connected or self._node is None:
            return
        msg = String()
        msg.data = json.dumps({'control_mode': mode})
        self._node.pub_control_mode.publish(msg)

    def publish_speed(self, speed_pct: int):
        """Publish speed percentage (0-100)."""
        if self._tcp_bridge:
            self._tcp_bridge.publish_speed(speed_pct)
            return
            
        if not self.connected or self._node is None:
            return
        msg = String()
        msg.data = json.dumps({'speed': int(speed_pct)})
        self._node.pub_speed.publish(msg)

    def publish_suction(self, enable: bool):
        """Toggle suction gripper."""
        if self._tcp_bridge:
            self._tcp_bridge.publish_suction(enable)
            return
            
        if not self.connected or self._node is None:
            return
        msg = String()
        msg.data = json.dumps({'suction': int(enable)})
        self._node.pub_suction.publish(msg)

    # ── Private ──────────────────────────────────────────────────────────────

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass
        self.connected = False
        if self.on_connection_change:
            self.on_connection_change(False)


# ── Internal ROS node (only defined when rclpy is present) ───────────────────

if _ROS_AVAILABLE:
    class _SimNode(Node):
        def __init__(self, on_joint_update=None, on_status_update=None):
            super().__init__('mg400_simulator')

            self.on_joint_update  = on_joint_update
            self.on_status_update = on_status_update

            # Publishers
            self.pub_joint_cmd    = self.create_publisher(Float64MultiArray,
                                        '/unity/joint_cmd', 10)
            self.pub_dashboard    = self.create_publisher(String,
                                        '/teleop/dashboard_cmd', 10)
            self.pub_control_mode = self.create_publisher(String,
                                        '/unity/control_mode', 10)
            self.pub_speed        = self.create_publisher(String,
                                        '/unity/speed', 10)
            self.pub_suction      = self.create_publisher(String,
                                        '/unity/suction', 10)

            # Subscribers
            self.create_subscription(Float64MultiArray, '/joint_states_deg',
                                     self._cb_joints, 10)
            self.create_subscription(String, '/teleop/status',
                                     self._cb_status, 10)
            self.create_subscription(Float64MultiArray, '/teleop/ee_pose',
                                     self._cb_ee, 10)

        def _cb_joints(self, msg: Float64MultiArray):
            if self.on_joint_update and len(msg.data) >= 4:
                self.on_joint_update(list(msg.data[:4]))

        def _cb_status(self, msg: String):
            if self.on_status_update:
                try:
                    self.on_status_update(json.loads(msg.data))
                except Exception:
                    pass

        def _cb_ee(self, msg: Float64MultiArray):
            pass   # reserved for future feedback overlay
