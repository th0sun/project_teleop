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
    0: (-160, 160),   # J1 - Base Rotation
    1: (-25, 85),     # J2 - Shoulder
    2: (-25, 105),    # J3 - Elbow
    3: (-360, 360)    # J4 - Wrist
}

# 🎯 Control Mode Options
# - "jointmovj" = Joint space interpolation (แนะนำ - เร็วและแม่นยำ)
# - "movj"      = Joint move with Cartesian planning
# - "movl"      = Linear Cartesian move
CONTROL_MODE = "jointmovj"

# 📏 Spatial Threshold (radians)
# กรองการเคลื่อนที่เล็กๆ ที่ไม่จำเป็น
SPATIAL_THRESHOLD = 0.0005  # ~0.03 degrees