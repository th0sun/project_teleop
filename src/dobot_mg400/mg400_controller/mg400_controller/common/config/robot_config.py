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

# 🧪 Logic Mode (TEMPORARY — for A/B testing experimental strategies)
# "default" = Current production logic (TeleopController — DO NOT REMOVE)
# "m11"     = M11_Stable  — velocity-clamped integrator, most consistent
# "m14"     = M14_Smooth  — feedforward + curvature gate, best for curves
# "m15"     = M15_Sharp   — strict velocity clamp, best for sharp corners
# "m8_raw"  = M8_RawData  — sends raw target at fixed frequency (adjustable via terminal)
LOGIC_MODE = "default"
RAW_HZ = 10  # Default frequency for M8_RawData mode

# ⚙️ Feature Flags
# Enable GetError() API for detailed error reporting
# NOTE: Set to False when using MG400_MOCK simulator (not implemented)
ENABLE_GET_ERROR = True  # Default: disabled for simulator compatibility