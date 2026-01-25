#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🎯 Motion Control Configuration
การตั้งค่าเกี่ยวกับการเคลื่อนที่และความเร็ว

📝 แก้ไขที่นี่เมื่อ:
- ต้องการปรับความเร็ว
- เปลี่ยนการตั้งค่าความนุ่มนวล (smoothness)
- ปรับขนาดคิวคำสั่ง
"""

# ⚡ Speed & Acceleration Settings
MAX_SPEED_DEG = 300.0  # Maximum joint speed (degrees/sec)
ACC_VALUE = 100        # Acceleration value (0-100)
CP_VALUE = 80          # Continuous Path value (smoothness: 0-100)

# 📊 Queue Management
QUEUE_LIMIT = 5        # Maximum commands in queue
QUEUE_LEAK_INTERVAL = 0.03  # Leak rate (seconds) - ลดคิวทุก 30ms

# 🎮 Adaptive Speed Thresholds
# กำหนดความเร็วตามระยะห่างจากเป้าหมาย
SPEED_FAR_THRESHOLD = 0.1      # > 0.1 rad → 100% speed
SPEED_MEDIUM_THRESHOLD = 0.05  # > 0.05 rad → 90% speed
# < 0.05 rad → 70% speed (ใกล้เป้าหมาย ช้าลง)

SPEED_FAR = 100      # %
SPEED_MEDIUM = 90    # %
SPEED_NEAR = 70      # %

# 📡 ROS Topics
UNITY_TOPIC = "/unity/joint_cmd"  # รับคำสั่งจาก Unity/VR
RVIZ_TOPIC  = "/joint_states"     # ส่งสถานะไปแสดงใน RViz
DEBUG_TOPIC = "/teleop/debug"     # Debug messages