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

class RobotConnection:
    def __init__(self, logger):
        self.logger = logger
        self.dashboard = None
        self.cmd_sock = None
        self.fb_sock = None
        self.connected = False
        self.dash_lock = threading.Lock() # Lock for Dashboard Port (29999) to avoid thread collision
    
    def connect(self):
        """เชื่อมต่อกับหุ่นยนต์ทั้ง 3 channels"""
        try:
            # Dashboard Socket (สำหรับคำสั่งควบคุม)
            self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard.settimeout(SOCKET_TIMEOUT)
            self.dashboard.connect((ROBOT_IP, DASHBOARD_PORT))
            
            # Command Socket (สำหรับส่งคำสั่งเคลื่อนที่)
            self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
                self.dashboard.send(b"ClearError()\n")
                time.sleep(0.1)
                self.dashboard.send(b"EnableRobot()\n")
                self.logger.info("🟢 Robot Enabled")
                return True
            except Exception as e:
                self.logger.error(f"Enable failed: {e}")
                return False
    
    
    def _reconnect_dashboard(self):
        """Try to reconnect dashboard socket only"""
        try:
            self.logger.warn("🔄 Reconnecting Dashboard Socket...")
            if self.dashboard:
                try: self.dashboard.close()
                except: pass
            
            self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard.settimeout(SOCKET_TIMEOUT)
            self.dashboard.connect((ROBOT_IP, DASHBOARD_PORT))
            self.logger.info("✅ Dashboard Socket Reconnected!")
            return True
        except Exception as e:
            self.logger.error(f"❌ Reconnect failed: {e}")
            return False

    def send_dashboard_cmd(self, command):
        """ส่งคำสั่งผ่าน Dashboard Port with Auto-Reconnect"""
        if not self.connected: 
            # Try to revive safely
            if not self._reconnect_dashboard():
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
                if self._reconnect_dashboard():
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
             if not self._reconnect_dashboard():
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
                if self._reconnect_dashboard():
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
        """ส่งคำสั่งการเคลื่อนที่"""
        if not self.connected:
            return False
        try:
            cmd_str = command if isinstance(command, str) else command.decode()
            if not cmd_str.endswith('\n'):
                cmd_str += '\n'
            self.cmd_sock.send(cmd_str.encode())
            return True
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