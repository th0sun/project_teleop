#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🔌 Robot Connection Manager
จัดการการเชื่อมต่อ Socket ทั้งหมดกับหุ่นยนต์

ใช้งาน:
    conn = RobotConnection(logger)
    if conn.connect():
        conn.enable_robot()
"""

import socket
import time
import threading
from mg400_controller.common.config.network_config import *
from mg400_protocol.dashboard import (
    acc_j,
    clear_error,
    enable_robot as enable_robot_cmd,
    speed_factor,
    speed_j,
)

class RobotConnection:
    def __init__(self, logger):
        self.logger = logger
        self.dashboard = None
        self.cmd_sock = None
        self.fb_sock = None
        self.connected = False
        self.dash_lock = threading.Lock() # Lock for Dashboard Port (29999) to avoid thread collision

    def _send_dashboard_locked(self, command):
        cmd_bytes = command.encode() if isinstance(command, str) else command
        if not cmd_bytes.endswith(b'\n'):
            cmd_bytes += b'\n'
        self.dashboard.send(cmd_bytes)
    
    def connect(self):
        """เชื่อมต่อกับหุ่นยนต์ทั้ง 3 channels"""
        try:
            # Dashboard Socket (สำหรับคำสั่งควบคุม)
            self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard.settimeout(SOCKET_TIMEOUT)
            self.dashboard.connect((ROBOT_IP, DASHBOARD_PORT))
            
            # Command Socket (สำหรับส่งคำสั่งเคลื่อนที่)
            self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # ปิด Nagle's Algorithm เพื่อไม่ให้ OS ดองแพ็กเกจเล็กๆ แล้วส่งรวบยอด (ลด Jitter)
            self.cmd_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.cmd_sock.settimeout(SOCKET_TIMEOUT)
            self.cmd_sock.connect((ROBOT_IP, CMD_PORT))
            
            # Feedback Socket (สำหรับรับข้อมูล real-time)
            self.fb_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.fb_sock.settimeout(SOCKET_TIMEOUT)
            self.fb_sock.connect((ROBOT_IP, FEEDBACK_PORT))
            
            self.connected = True
            self.logger.info(f"✅ Connected to Robot at {ROBOT_IP}")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Connection Failed: {e}")
            self.connected = False
            return False
    
    def enable_robot(self):
        """เปิดใช้งานหุ่นยนต์"""
        if not self.connected:
            return False
        with self.dash_lock:
            try:
                self.dashboard.send((clear_error().render() + "\n").encode())
                time.sleep(0.1)
                self.dashboard.send((enable_robot_cmd().render() + "\n").encode())
                time.sleep(0.1)

                # Unlock max robot speed limits via dashboard scalers.
                self._send_dashboard_locked(speed_factor(100).render())
                self._send_dashboard_locked(acc_j(100).render())
                self._send_dashboard_locked(speed_j(100).render())
                
                self.logger.info("🟢 Robot Enabled and Speed Limits Unlocked")
                return True
            except Exception as e:
                self.logger.error(f"❌ Reconnect failed: {e}")
                return False
        return False

    def set_realtime_speed_defaults(self):
        """Restore live teleop dashboard scalers after teach/replay tuning."""
        if not self.connected:
            return False
        with self.dash_lock:
            try:
                self._send_dashboard_locked(speed_factor(100).render())
                self._send_dashboard_locked(acc_j(100).render())
                self._send_dashboard_locked(speed_j(100).render())
                self.logger.info("🏃 Realtime speed defaults restored: SpeedFactor/AccJ/SpeedJ = 100")
                return True
            except Exception as e:
                self.logger.warn(f"⚠️ Failed to restore realtime speed defaults: {e}")
                return False

    def _reconnect_port(self, port_name):
        """Generic reconnect for a specific port"""
        target_port = DASHBOARD_PORT if port_name == 'dashboard' else (CMD_PORT if port_name == 'cmd' else FEEDBACK_PORT)
        
        try:
            self.logger.warn(f"🔄 Reconnecting {port_name.upper()} Socket ({target_port})...")
            new_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if port_name == 'cmd':
                new_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            new_sock.settimeout(SOCKET_TIMEOUT)
            new_sock.connect((ROBOT_IP, target_port))
            
            if port_name == 'dashboard':
                if self.dashboard: self.dashboard.close()
                self.dashboard = new_sock
            elif port_name == 'cmd':
                if self.cmd_sock: self.cmd_sock.close()
                self.cmd_sock = new_sock
            elif port_name == 'fb':
                if self.fb_sock: self.fb_sock.close()
                self.fb_sock = new_sock
            
            self.logger.info(f"✅ {port_name.upper()} Socket Reconnected!")
            return True
        except Exception as e:
            self.logger.error(f"❌ Reconnect {port_name} failed: {e}")
            return False


    def send_dashboard_cmd(self, command):
        """ส่งคำสั่งผ่าน Dashboard Port with Auto-Reconnect"""
        if not self.connected: 
            # Try to revive safely
            if not self._reconnect_port('dashboard'):
                return False
        
        with self.dash_lock:
            try:
                cmd_bytes = command.encode() if isinstance(command, str) else command
                if not cmd_bytes.endswith(b'\n'):
                    cmd_bytes += b'\n'
                self.dashboard.send(cmd_bytes)
                return True
            except socket.timeout:
                self.logger.warn(f"Timeout (Dashboard): {command}")
                return False
            except (OSError, socket.error) as e:
                self.logger.warn(f"Dashboard socket error: {e}. Reconnecting...")
                if self._reconnect_port('dashboard'):
                    try:
                        self.dashboard.send(cmd_bytes)
                        return True
                    except Exception as retry_e:
                        self.logger.error(f"Retry failed: {retry_e}")
                return False
            except Exception as e:
                self.logger.error(f"Dashboard command failed: {e}")
                return False
    
    def send_and_wait(self, command, timeout=2.0):
        """
        ส่งคำสั่งผ่าน Dashboard และรอรับ response
        
        Args:
            command: คำสั่งที่ต้องการส่ง (str)
            timeout: เวลารอรับ response (seconds)
            
        Returns:
            str: Response from robot, or None if failed
        """
        if not self.connected:
             if not self._reconnect_port('dashboard'):
                return None
        with self.dash_lock:
            try:
                # ส่งคำสั่ง
                cmd_bytes = command.encode() if isinstance(command, str) else command
                if not cmd_bytes.endswith(b'\n'):
                    cmd_bytes += b'\n'
                self.dashboard.send(cmd_bytes)
                
                # รอรับ response
                self.dashboard.settimeout(timeout)
                response = self.dashboard.recv(4096).decode('utf-8').strip()
                return response
                
            except socket.timeout:
                self.logger.warn(f"Timeout waiting for response to: {command}")
                return None
                
            except (OSError, socket.error) as e:
                self.logger.warn(f"Dashboard socket error in send_and_wait: {e}. Reconnecting...")
                if self._reconnect_port('dashboard'):
                    try:
                        self.dashboard.send(cmd_bytes)
                        self.dashboard.settimeout(timeout)
                        response = self.dashboard.recv(4096).decode('utf-8').strip()
                        return response
                    except Exception as retry_e:
                        self.logger.error(f"Retry failed: {retry_e}")
                        return None
                return None
                
            except Exception as e:
                self.logger.error(f"send_and_wait failed: {e}")
                return None
            finally:
                # Reset timeout
                self.dashboard.settimeout(SOCKET_TIMEOUT)
    
    def send_motion_cmd(self, command):
        """ส่งคำสั่งการเคลื่อนที่ with Auto-Reconnect"""
        if not self.connected:
            if not self._reconnect_port('cmd'):
                return False
        try:
            cmd_str = command if isinstance(command, str) else command.decode()
            if not cmd_str.endswith('\n'):
                cmd_str += '\n'
            self.cmd_sock.send(cmd_str.encode())
            return True
        except (OSError, socket.error) as e:
            self.logger.warn(f"Motion socket error: {e}. Reconnecting...")
            if self._reconnect_port('cmd'):
                try:
                    self.cmd_sock.send(cmd_str.encode())
                    return True
                except Exception as retry_e:
                    self.logger.error(f"Retry motion failed: {retry_e}")
            return False
        except Exception as e:
            self.logger.error(f"Motion command failed: {e}")
            return False
    
    def disconnect(self):
        """ปิดการเชื่อมต่อทั้งหมด"""
        for sock in [self.dashboard, self.cmd_sock, self.fb_sock]:
            if sock:
                try:
                    sock.close()
                except:
                    pass
        self.connected = False
        self.logger.info("🔌 Disconnected from robot")
