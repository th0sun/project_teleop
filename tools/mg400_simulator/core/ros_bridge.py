#!/usr/bin/env python3
"""
Optional ROS 2 bridge for the MG400 Simulator.
Connects to the live teleop_logic stack when ROS 2 is available.
Gracefully degrades to a no-op when rclpy is not found.
"""

import json
import threading
import math
import time
import uuid
from typing import Optional, Callable, List

from core.unity_tcp_bridge import (
    UnityTcpBridge,
    active_joint_positions_deg,
    build_teleop_sample_payload,
    encode_unity_joint_frame_id,
)

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, Int32, String
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
        self.teleop_session_id = f"mac-sim-{int(time.time())}-{uuid.uuid4().hex[:8]}"
        self._unity_seq_id = 0
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
        """Publish joint command to /unity/joint_cmd.

        The simulator UI stores joint angles in degrees, while the teleop node
        consumes sensor_msgs/JointState positions in radians.
        """
        if self._tcp_bridge:
            return bool(self._tcp_bridge.publish_joint_cmd(joints_deg))
            
        if not self.connected or self._node is None:
            return False
        self._unity_seq_id += 1
        timestamp = float(time.time())
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = encode_unity_joint_frame_id(
            self.teleop_session_id,
            self._unity_seq_id,
        )
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = [math.radians(float(j)) for j in joints_deg[:4]]
        sample = String()
        sample.data = json.dumps(
            build_teleop_sample_payload(
                msg.position,
                session_id=self.teleop_session_id,
                unity_seq_id=self._unity_seq_id,
                timestamp=timestamp,
            ),
            separators=(',', ':'),
            sort_keys=True,
        )
        self._node.pub_teleop_sample.publish(sample)
        self._node.pub_joint_cmd.publish(msg)
        return True

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
        msg = Int32()
        msg.data = int(speed_pct)
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

    def publish_teach_job_request(self, action: str, trajectory: dict,
                                  target: str = 'mg400', options: dict = None) -> str:
        """Publish a TeachJobPublisher-compatible request to /teach/job_request."""
        job_id = str(uuid.uuid4())
        payload = {
            'job_id': job_id,
            'action': action,
            'target': target,
            'submitted_at_unity_sec': float(time.time()),
            'options': dict(options or {}),
            'trajectory': trajectory,
        }
        payload_json = json.dumps(payload, sort_keys=True)

        if self._tcp_bridge:
            self._tcp_bridge.publish_teach_job_request(payload_json)
            return job_id

        if not self.connected or self._node is None:
            return ''
        msg = String()
        msg.data = payload_json
        self._node.pub_teach_job_request.publish(msg)
        return job_id

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
            self.pub_joint_cmd    = self.create_publisher(JointState,
                                        '/unity/joint_cmd', 100)
            self.pub_teleop_sample = self.create_publisher(String,
                                        '/unity/teleop_sample', 512)
            self.pub_dashboard    = self.create_publisher(String,
                                        '/teleop/dashboard_cmd', 10)
            self.pub_control_mode = self.create_publisher(String,
                                        '/unity/control_mode', 10)
            self.pub_speed        = self.create_publisher(Int32,
                                        '/unity/speed_factor', 10)
            self.pub_suction      = self.create_publisher(String,
                                        '/unity/suction', 10)
            self.pub_teach_job_request = self.create_publisher(String,
                                        '/teach/job_request', 10)

            # Subscribers
            self.create_subscription(JointState, '/joint_states',
                                     self._cb_joints, 10)
            self.create_subscription(String, '/teleop/status',
                                     self._cb_status, 10)
            self.create_subscription(Float64MultiArray, '/teleop/ee_pose',
                                     self._cb_ee, 10)

        def _cb_joints(self, msg: JointState):
            if self.on_joint_update and len(msg.position) >= 4:
                self.on_joint_update(active_joint_positions_deg(
                    list(getattr(msg, 'name', []) or []),
                    list(msg.position),
                ))

        def _cb_status(self, msg: String):
            if self.on_status_update:
                try:
                    self.on_status_update(json.loads(msg.data))
                except Exception:
                    pass

        def _cb_ee(self, msg: Float64MultiArray):
            pass   # reserved for future feedback overlay
