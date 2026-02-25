#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🤖 Robot Configuration
การตั้งค่าพื้นฐานของหุ่นยนต์ MG400

📝 แก้ไขที่นี่เมื่อ:
- เปลี่ยน IP Address
- ปรับขีดจำกัด Joint
- เปลี่ยนโหมดควบคุม
"""

# 🔧 Joint Limits (degrees) - MG400
JOINT_LIMITS = {
    0: (-160.0, 160.0),   # J1 - Base Rotation
    1: (-24.5, 84.5),     # J2 - Shoulder (Buffered to prevent -25.0 hard limit alarm)
    2: (-24.5, 104.5),    # J3 - Elbow (Buffered to prevent J3 hard limit)
    3: (-360.0, 360.0)    # J4 - Wrist
}

# 🦾 Elbow Angle Limit (degrees) - MG400 Physical Constraint
# This is the relative angle between J2 and J3 (J3 - J2)
# Prevents the robot from reaching physically impossible configurations
ELBOW_ANGLE_LIMIT = (-60, 60)  # (J3 - J2) must be within this range


# 🎯 Control Mode Options
# - "jointmovj" = Joint space interpolation (แนะนำ - เร็วและแม่นยำ)
# - "movj"      = Joint move with Cartesian planning
# - "movl"      = Linear Cartesian move
CONTROL_MODE = "jointmovj"

# 📏 Spatial Threshold (radians)
# กรองการเคลื่อนที่เล็กๆ ที่ไม่จำเป็น
SPATIAL_THRESHOLD = 0.0005  # ~0.03 degrees

ENABLE_GET_ERROR = False  # Disabled by default since V4 firmware uses GetErrorID