import socket
import struct
import json
import threading
import time
import math
import uuid
from typing import Optional, Callable, List, Tuple
from urllib.parse import quote, unquote


TELEOP_PROTOCOL_VERSION = "teleop_sample_v1"

def pack_string(s: str) -> bytes:
    b = s.encode('utf-8')
    return struct.pack('<I', len(b)) + b

def cdr_empty() -> bytes:
    return b'\x00\x01\x00\x00'

def cdr_bool(val: bool) -> bytes:
    cdr = bytearray(b'\x00\x01\x00\x00')
    cdr += struct.pack('<?', val)
    return bytes(cdr)

def cdr_int32(val: int) -> bytes:
    cdr = bytearray(b'\x00\x01\x00\x00')
    while len(cdr) % 4 != 0: cdr += b'\x00'
    cdr += struct.pack('<i', val)
    return bytes(cdr)

def cdr_string(s: str) -> bytes:
    cdr = bytearray(b'\x00\x01\x00\x00')
    b = s.encode('utf-8')
    cdr += struct.pack('<I', len(b) + 1) + b + b'\x00'
    return bytes(cdr)

def cdr_int32_multi_array(vals: List[int]) -> bytes:
    cdr = bytearray(b'\x00\x01\x00\x00')
    cdr += struct.pack('<I', 0)
    cdr += struct.pack('<I', 0)
    cdr += struct.pack('<I', len(vals))
    while len(cdr) % 4 != 0: cdr += b'\x00'
    for v in vals:
        cdr += struct.pack('<i', v)
    return bytes(cdr)

def encode_unity_joint_frame_id(session_id: str, unity_seq_id: int) -> str:
    return (
        f"{TELEOP_PROTOCOL_VERSION};"
        f"session_id={quote(str(session_id), safe='')};"
        f"unity_seq_id={int(unity_seq_id)}"
    )


def parse_unity_joint_frame_id(frame_id: str) -> Optional[Tuple[str, int]]:
    if not frame_id:
        return None
    parts = str(frame_id).split(";")
    if not parts or parts[0] != TELEOP_PROTOCOL_VERSION:
        return None
    fields = {}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    session_id = unquote(fields.get("session_id", "")).strip()
    seq_value = fields.get("unity_seq_id")
    if not session_id or seq_value is None:
        return None
    try:
        return session_id, int(seq_value)
    except (TypeError, ValueError):
        return None


def cdr_joint_state(
    names: List[str],
    positions: List[float],
    *,
    frame_id: str = "sim",
    timestamp: Optional[float] = None,
) -> bytes:
    cdr = bytearray(b'\x00\x01\x00\x00')
    t = time.time() if timestamp is None else float(timestamp)
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    cdr += struct.pack('<iI', sec, nanosec)

    frame_id_bytes = str(frame_id).encode('utf-8')
    cdr += struct.pack('<I', len(frame_id_bytes) + 1) + frame_id_bytes + b'\x00'
    
    while len(cdr) % 4 != 0: cdr += b'\x00'
    cdr += struct.pack('<I', len(names))
    for n in names:
        n_bytes = n.encode('utf-8')
        cdr += struct.pack('<I', len(n_bytes) + 1) + n_bytes + b'\x00'
        while len(cdr) % 4 != 0: cdr += b'\x00'
        
    while len(cdr) % 4 != 0: cdr += b'\x00'
    cdr += struct.pack('<I', len(positions))
    
    # CDR alignment for float64 (8 bytes):
    # Alignment is from CDR data start (byte 4, after the 4-byte encapsulation header)
    # Correct check: (len(cdr) - 4) % 8 == 0  →  i.e. len(cdr) % 8 == 4
    while (len(cdr) - 4) % 8 != 0: cdr += b'\x00'
    for p in positions:
        cdr += struct.pack('<d', p)
        
    while len(cdr) % 4 != 0: cdr += b'\x00'
    cdr += struct.pack('<I', 0)  # velocity array length = 0
    cdr += struct.pack('<I', 0)  # effort array length = 0
    
    return bytes(cdr)


def build_teleop_sample_payload(
    joints_rad: List[float],
    *,
    session_id: str,
    unity_seq_id: int,
    timestamp: float,
    controller_id: str = "mac_simulator",
    control_mode: str = "joint_simulator",
) -> dict:
    joints = list(joints_rad[:4])
    if len(joints) < 4:
        joints.extend([0.0] * (4 - len(joints)))

    return {
        "protocol_version": TELEOP_PROTOCOL_VERSION,
        "session_id": session_id,
        "unity_seq_id": int(unity_seq_id),
        "controller_capture_ts": float(timestamp),
        "unity_target_ts": float(timestamp),
        "unity_ik_ts": float(timestamp),
        "unity_send_ts": float(timestamp),
        "controller_id": controller_id,
        "ctrl_pos_x_m": 0.0,
        "ctrl_pos_y_m": 0.0,
        "ctrl_pos_z_m": 0.0,
        "ctrl_rot_x": 0.0,
        "ctrl_rot_y": 0.0,
        "ctrl_rot_z": 0.0,
        "ctrl_rot_w": 1.0,
        "trigger_value": 0.0,
        "grip_value": 0.0,
        "primary_button": False,
        "secondary_button": False,
        "j1_ik_rad": float(joints[0]),
        "j2_ik_rad": float(joints[1]),
        "j3_ik_rad": float(joints[2]),
        "j4_ik_rad": float(joints[3]),
        "joints_ik_rad": [float(value) for value in joints],
        "control_mode": control_mode,
        "unity_filter_status": "accepted",
        "unity_filter_detail": "mac_simulator_joint_command",
        "is_valid": True,
        "invalid_reason": "",
    }


def _align(offset: int, base: int, boundary: int) -> int:
    while (offset - base) % boundary != 0:
        offset += 1
    return offset


def _read_u32(data: bytes, offset: int) -> Tuple[int, int]:
    return struct.unpack_from('<I', data, offset)[0], offset + 4


def _read_i32(data: bytes, offset: int) -> Tuple[int, int]:
    return struct.unpack_from('<i', data, offset)[0], offset + 4


def _read_cdr_string(data: bytes, offset: int) -> Tuple[str, int]:
    length, offset = _read_u32(data, offset)
    raw = data[offset:offset + max(0, length - 1)]
    offset += length
    return raw.decode('utf-8', errors='replace'), offset


def parse_cdr_string(data: bytes) -> str:
    if len(data) < 8:
        return ''
    offset = _align(4, 4, 4)
    text, _ = _read_cdr_string(data, offset)
    return text


def active_joint_positions_deg(names: List[str], positions: List[float]) -> List[float]:
    """Extract MG400 active joints [J1,J2,J3,J4] from a JointState message.

    `/joint_states` for RViz uses the full URDF joint list, including passive
    mimic joints.  The simulator UI needs the four controller joints.  When the
    message is the Unity command contract (`joint1`..`joint4`) this function
    returns those directly.
    """
    if len(positions) < 4:
        return []

    name_to_pos = {name: positions[idx] for idx, name in enumerate(names)}

    for active_names in (
        ("mg400_j1", "mg400_j2_1", "mg400_j3", "mg400_j5"),
        ("joint1", "joint2", "joint3", "joint4"),
    ):
        if all(name in name_to_pos for name in active_names):
            return [math.degrees(name_to_pos[name]) for name in active_names]

    # Compatibility fallback for old `/joint_states` publishers that emitted
    # the full URDF positions but forgot names.  Indices match
    # KinematicsCalculator.joint_names.
    if len(positions) >= 9:
        return [math.degrees(positions[idx]) for idx in (0, 1, 3, 8)]

    return [math.degrees(v) for v in positions[:4]]


def parse_cdr_joint_state_deg(data: bytes) -> List[float]:
    """Parse ROS-TCP serialized sensor_msgs/JointState into active MG400 joints."""
    if len(data) < 24:
        return []

    offset = 4  # CDR encapsulation header
    offset = _align(offset, 4, 4)
    _, offset = _read_i32(data, offset)  # stamp.sec
    _, offset = _read_u32(data, offset)  # stamp.nanosec

    offset = _align(offset, 4, 4)
    _, offset = _read_cdr_string(data, offset)  # header.frame_id

    offset = _align(offset, 4, 4)
    name_count, offset = _read_u32(data, offset)
    names = []
    for _ in range(name_count):
        offset = _align(offset, 4, 4)
        name, offset = _read_cdr_string(data, offset)
        names.append(name)

    offset = _align(offset, 4, 4)
    position_count, offset = _read_u32(data, offset)
    if position_count <= 0:
        return []

    offset = _align(offset, 4, 8)
    positions = []
    for _ in range(position_count):
        positions.append(struct.unpack_from('<d', data, offset)[0])
        offset += 8
    return active_joint_positions_deg(names, positions)


def parse_cdr_joint_state_frame_id(data: bytes) -> str:
    if len(data) < 16:
        return ""

    offset = 4  # CDR encapsulation header
    offset = _align(offset, 4, 4)
    _, offset = _read_i32(data, offset)  # stamp.sec
    _, offset = _read_u32(data, offset)  # stamp.nanosec

    offset = _align(offset, 4, 4)
    frame_id, _ = _read_cdr_string(data, offset)
    return frame_id

class UnityTcpBridge:
    def __init__(self,
                 on_joint_update: Optional[Callable[[List[float]], None]] = None,
                 on_status_update: Optional[Callable[[dict], None]] = None,
                 on_connection_change: Optional[Callable[[bool], None]] = None):
        self.on_joint_update = on_joint_update
        self.on_status_update = on_status_update
        self.on_connection_change = on_connection_change
        
        self.connected = False
        self.sock = None
        self._thread = None
        self._stop_event = threading.Event()
        self._send_lock = threading.Lock()
        self._connect_lock = threading.Lock()
        self.teleop_session_id = f"mac-sim-{int(time.time())}-{uuid.uuid4().hex[:8]}"
        self._unity_seq_id = 0
        self._host: Optional[str] = None
        self._port: Optional[int] = None
        self._auto_reconnect = True

    def start(self, host: str, port: int) -> bool:
        self._host = host
        self._port = port
        return self._open_connection()

    def _open_connection(self) -> bool:
        with self._connect_lock:
            if self.connected and self.sock:
                return True
            try:
                self._close_socket()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Optimization: Disable Nagle's algorithm for minimal latency
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(2.0)
                self.sock.connect((self._host, self._port))
                self.sock.settimeout(None)

                # Register Publishers (maps to SysCommands.publish on the server)
                # Server strips leading '__' and calls getattr(syscommands, 'publish').
                # JSON keys must match SysCommands.publish(topic, message_name, ...) signature.
                pubs = [
                    ("/unity/joint_cmd", "sensor_msgs/JointState", 1),
                    ("/unity/teleop_sample", "std_msgs/String", 512),
                    ("/vr/suction_cmd", "std_msgs/Bool", 10),
                    ("/mg400/light_cmd", "std_msgs/Int32MultiArray", 10),
                    ("/teach/job_request", "std_msgs/String", 10),
                    ("/unity/enable_robot", "std_msgs/Bool", 10),
                    ("/unity/clear_error", "std_msgs/Empty", 10),
                    ("/unity/emergency_stop", "std_msgs/Bool", 10),
                    ("/unity/speed_factor", "std_msgs/Int32", 10),
                    ("/unity/control_mode", "std_msgs/String", 10),
                ]
                for topic, msg_type, queue_size in pubs:
                    # Server does data.decode('utf-8')[:-1], so append trailing \x00
                    req = json.dumps({
                        "topic": topic,
                        "message_name": msg_type,
                        "queue_size": queue_size,
                    }).encode('utf-8') + b'\x00'
                    if not self._send_msg('__publish', req, retry=False):
                        raise ConnectionError(f'publisher registration failed: {topic}')

                subs = [
                    ("/joint_states", "sensor_msgs/JointState"),
                    ("/teach/job_status", "std_msgs/String"),
                ]
                for topic, msg_type in subs:
                    req = json.dumps({"topic": topic, "message_name": msg_type}).encode('utf-8') + b'\x00'
                    if not self._send_msg('__subscribe', req, retry=False):
                        raise ConnectionError(f'subscription registration failed: {topic}')

                self._set_connected(True)

                self._stop_event.clear()
                self._thread = threading.Thread(target=self._recv_loop, daemon=True)
                self._thread.start()

                return True
            except Exception as exc:
                self._set_connected(False)
                self._close_socket()
                print(f'[UnityTcpBridge] start failed: {exc}')
                return False

    def _ensure_connected(self) -> bool:
        if self.connected:
            return True
        if self._auto_reconnect and self._host and self._port:
            return self._open_connection()
        return False

    def _set_connected(self, value: bool):
        changed = self.connected != value
        self.connected = value
        if changed and self.on_connection_change:
            self.on_connection_change(value)

    def _close_socket(self):
        sock = self.sock
        self.sock = None
        if not sock:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            sock.close()
        except Exception:
            pass

    def _recv_loop(self):
        sock = self.sock
        buf = bytearray()
        try:
            while not self._stop_event.is_set() and sock:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf.extend(chunk)
                while True:
                    if len(buf) < 4:
                        break
                    dest_len = struct.unpack_from('<I', buf, 0)[0]
                    header_len = 4 + dest_len + 4
                    if len(buf) < header_len:
                        break
                    dest = bytes(buf[4:4 + dest_len]).decode('utf-8', errors='replace').rstrip('\x00')
                    data_len = struct.unpack_from('<I', buf, 4 + dest_len)[0]
                    packet_len = header_len + data_len
                    if len(buf) < packet_len:
                        break
                    data = bytes(buf[header_len:packet_len])
                    del buf[:packet_len]
                    self._handle_packet(dest, data)
        except Exception:
            pass
        if self.sock is sock:
            self._set_connected(False)
            self._close_socket()
        if self.sock is None and self._auto_reconnect and not self._stop_event.is_set() and self._host:
            self._reconnect_thread = threading.Thread(target=self._reconnect_loop, daemon=True)
            self._reconnect_thread.start()

    def _reconnect_loop(self):
        backoff = 0.5
        while not self._stop_event.is_set():
            time.sleep(backoff)
            if self._open_connection():
                return
            backoff = min(backoff * 1.5, 3.0)

    def _handle_packet(self, destination: str, data: bytes):
        if destination == "/joint_states":
            joints = parse_cdr_joint_state_deg(data)
            if joints and self.on_joint_update:
                self.on_joint_update(joints)
        elif destination == "/teach/job_status":
            if self.on_status_update:
                try:
                    text = parse_cdr_string(data)
                    payload = json.loads(text) if text else {}
                    payload.setdefault("_topic", destination)
                    self.on_status_update(payload)
                except Exception:
                    pass

    def publish_joint_cmd(self, joints_deg: List[float]):
        if not self._ensure_connected(): return False
        joints_rad = [math.radians(j) for j in joints_deg]
        timestamp = time.time()
        payload = self._publish_teleop_sample(joints_rad, timestamp=timestamp)
        frame_id = encode_unity_joint_frame_id(
            payload["session_id"],
            payload["unity_seq_id"],
        )
        cdr = cdr_joint_state(
            ['joint1', 'joint2', 'joint3', 'joint4'],
            joints_rad,
            frame_id=frame_id,
            timestamp=timestamp,
        )
        return self._send_msg('/unity/joint_cmd', cdr)

    def _publish_teleop_sample(self, joints_rad: List[float], *, timestamp: float):
        self._unity_seq_id += 1
        payload = build_teleop_sample_payload(
            joints_rad,
            session_id=self.teleop_session_id,
            unity_seq_id=self._unity_seq_id,
            timestamp=timestamp,
        )
        self._send_msg(
            '/unity/teleop_sample',
            cdr_string(json.dumps(payload, separators=(',', ':'), sort_keys=True)),
        )
        return payload

    def publish_dashboard_cmd(self, cmd: str):
        if not self._ensure_connected(): return False
        if cmd == 'EnableRobot()':
            return self._send_msg('/unity/enable_robot', cdr_bool(True))
        elif cmd == 'DisableRobot()':
            return self._send_msg('/unity/enable_robot', cdr_bool(False))
        elif cmd == 'ClearError()':
            return self._send_msg('/unity/clear_error', cdr_empty())
        elif cmd == 'EmergencyStop()':
            return self._send_msg('/unity/emergency_stop', cdr_bool(True))
        elif cmd.startswith('DO('):
            parts = cmd[3:-1].split(',')
            if len(parts) == 2:
                port = int(parts[0])
                state = int(parts[1])
                return self._send_msg('/mg400/light_cmd', cdr_int32_multi_array([port, state]))
        return False

    def publish_control_mode(self, mode: str):
        if not self._ensure_connected(): return False
        return self._send_msg('/unity/control_mode', cdr_string(mode))

    def publish_speed(self, speed_pct: int):
        if not self._ensure_connected(): return False
        return self._send_msg('/unity/speed_factor', cdr_int32(int(speed_pct)))

    def publish_suction(self, enable: bool):
        if not self._ensure_connected(): return False
        return self._send_msg('/vr/suction_cmd', cdr_bool(enable))

    def publish_teach_job_request(self, payload_json: str):
        if not self._ensure_connected(): return False
        return self._send_msg('/teach/job_request', cdr_string(payload_json))

    def _send_msg(self, dest: str, data_bytes: bytes, *, retry: bool = True) -> bool:
        packet = pack_string(dest) + struct.pack('<I', len(data_bytes)) + data_bytes
        attempts = 2 if retry else 1
        for attempt in range(attempts):
            with self._send_lock:
                try:
                    if not self.sock:
                        raise ConnectionError('ROS-TCP socket is not connected')
                    self.sock.sendall(packet)
                    return True
                except Exception:
                    self._set_connected(False)
                    self._close_socket()
            if not retry or attempt + 1 >= attempts:
                return False
            if not self._ensure_connected():
                return False
        return False

    def stop(self):
        self._auto_reconnect = False
        self._stop_event.set()
        self._close_socket()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._set_connected(False)
