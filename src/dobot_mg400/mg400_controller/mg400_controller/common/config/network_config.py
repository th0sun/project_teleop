#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🌐 Network Configuration
การตั้งค่าการเชื่อมต่อเครือข่ายกับหุ่นยนต์

📝 แก้ไขที่นี่เมื่อ:
- เปลี่ยน IP Address ของหุ่นยนต์
- ปรับ Port สำหรับโปรโตคอลต่างๆ
"""

# 🤖 Robot Network Settings
ROBOT_IP = "172.10.0.2"  # For MG400_Mock (Simulation)
# ROBOT_IP = "192.168.1.6"    # Real Robot IP

# 🔌 Communication Ports
DASHBOARD_PORT = 29999  # Dashboard commands (Enable, Disable, etc.)
CMD_PORT       = 30003  # Motion commands (MovJ, MovL, etc.)
FEEDBACK_PORT  = 30004  # Real-time feedback (position, status)

# ⏱️ Connection Timeout
SOCKET_TIMEOUT = 2.0  # seconds