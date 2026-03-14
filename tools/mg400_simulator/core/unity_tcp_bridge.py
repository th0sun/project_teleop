import socket
import struct
import json
import threading
import time
import math
from typing import Optional, Callable, List

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

def cdr_joint_state(names: List[str], positions: List[float]) -> bytes:
    cdr = bytearray(b'\x00\x01\x00\x00')
    t = time.time()
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    cdr += struct.pack('<iI', sec, nanosec)
    
    frame_id = b'sim'
    cdr += struct.pack('<I', len(frame_id) + 1) + frame_id + b'\x00'
    
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

    def start(self, host: str, port: int) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Optimization: Disable Nagle's algorithm for minimal latency
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(2.0)
            self.sock.connect((host, port))
            self.sock.settimeout(None)
            
            # Register Publishers (maps to SysCommands.publish on the server)
            # Server strips leading '__' and calls getattr(syscommands, 'publish').
            # JSON keys must match SysCommands.publish(topic, message_name, ...) signature.
            pubs = [
                ("/unity/joint_cmd", "sensor_msgs/JointState"),
                ("/vr/suction_cmd", "std_msgs/Bool"),
                ("/mg400/light_cmd", "std_msgs/Int32MultiArray"),
                ("/unity/enable_robot", "std_msgs/Bool"),
                ("/unity/clear_error", "std_msgs/Empty"),
                ("/unity/emergency_stop", "std_msgs/Bool"),
                ("/unity/speed_factor", "std_msgs/Int32"),
                ("/unity/control_mode", "std_msgs/String"),
            ]
            for topic, msg_type in pubs:
                # Server does data.decode('utf-8')[:-1], so append trailing \x00
                req = json.dumps({"topic": topic, "message_name": msg_type}).encode('utf-8') + b'\x00'
                self._send_msg('__publish', req)

            self.connected = True
            if self.on_connection_change:
                self.on_connection_change(True)
                
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
            
            return True
        except Exception as exc:
            print(f'[UnityTcpBridge] start failed: {exc}')
            return False

    def _recv_loop(self):
        try:
            while not self._stop_event.is_set() and self.sock:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
        except Exception:
            pass
        self.connected = False
        if self.on_connection_change:
            self.on_connection_change(False)

    def publish_joint_cmd(self, joints_deg: List[float]):
        if not self.connected: return
        joints_rad = [math.radians(j) for j in joints_deg]
        cdr = cdr_joint_state(['joint1', 'joint2', 'joint3', 'joint4'], joints_rad)
        self._send_msg('/unity/joint_cmd', cdr)

    def publish_dashboard_cmd(self, cmd: str):
        if not self.connected: return
        if cmd == 'EnableRobot()':
            self._send_msg('/unity/enable_robot', cdr_bool(True))
        elif cmd == 'DisableRobot()':
            self._send_msg('/unity/enable_robot', cdr_bool(False))
        elif cmd == 'ClearError()':
            self._send_msg('/unity/clear_error', cdr_empty())
        elif cmd == 'EmergencyStop()':
            self._send_msg('/unity/emergency_stop', cdr_bool(True))
        elif cmd.startswith('DO('):
            parts = cmd[3:-1].split(',')
            if len(parts) == 2:
                port = int(parts[0])
                state = int(parts[1])
                self._send_msg('/mg400/light_cmd', cdr_int32_multi_array([port, state]))

    def publish_control_mode(self, mode: str):
        if not self.connected: return
        self._send_msg('/unity/control_mode', cdr_string(mode))

    def publish_speed(self, speed_pct: int):
        if not self.connected: return
        self._send_msg('/unity/speed_factor', cdr_int32(int(speed_pct)))

    def publish_suction(self, enable: bool):
        if not self.connected: return
        self._send_msg('/vr/suction_cmd', cdr_bool(enable))

    def _send_msg(self, dest: str, data_bytes: bytes):
        packet = pack_string(dest) + struct.pack('<I', len(data_bytes)) + data_bytes
        with self._send_lock:
            try:
                self.sock.sendall(packet)
            except Exception:
                self.connected = False

    def stop(self):
        self._stop_event.set()
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self._thread:
            self._thread.join(timeout=1.0)
        self.connected = False
        if self.on_connection_change:
            self.on_connection_change(False)
