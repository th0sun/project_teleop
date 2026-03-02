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
CP_VALUE = 100         # Continuous Path value (smoothness: 0-100)

# 🎮 Adaptive Speed Thresholds
# กำหนดความเร็วตามระยะห่างจากเป้าหมาย
SPEED_FAR_THRESHOLD = 0.1      # > 0.1 rad → use SPEED_FAR
SPEED_MEDIUM_THRESHOLD = 0.05  # > 0.05 rad → use SPEED_MEDIUM
# < 0.05 rad → use SPEED_NEAR (ใกล้เป้าหมาย ช้าลง)

SPEED_FAR = 100      # %
SPEED_MEDIUM = 100   # %
SPEED_NEAR = 100     # %

#  Real-Time Control Parameters (Proximity + Velocity-Based Stuck Detection)
# ปรับค่านี้เพื่อควบคุมความไวและความเร็วในการตอบสนอง
PROXIMITY_THRESHOLD = 0.08       # rad (~4.5°) - Increased to allow coarser updates (drain queue)
STUCK_VELOCITY_THRESHOLD = 0.005 # rad/s - ความเร็วต่ำกว่านี้ถือว่า "นิ่ง"
STUCK_TIME_THRESHOLD = 0.15      # seconds - ต้องนิ่งนานเท่านี้ถึงจะ trigger stuck recovery
TARGET_CHANGE_THRESHOLD = 0.02   # rad (~1.1°) - Target ต้องเปลี่ยนอย่างน้อยเท่านี้
# กำหนดค่าสำหรับ Dynamic Proximity
DYNAMIC_PROXIMITY_BASE_RAD = 0.02    # rad (~1.1°) - ระยะพื้นฐานขั้นต่ำ
DYNAMIC_PROXIMITY_LOOKAHEAD_SEC = 0.25 # seconds - วินาทีสำหรับคำนวณระยะเพิ่มตามความเร็ว
RATE_FLOOR_SEC = 0.02                  # seconds (Not used by restored 24 Feb logic)

# 🎯 Motion Detection Thresholds (Data-Driven from Log Analysis)
MOTION_START_THRESHOLD = 0.002   # rad/s - Detect motion start (T4), Target: 95%+ detection
                                 # Analysis: 69% @ 0.003, 95%+ @ 0.002

# 🎯 Validation Thresholds (Strict Arrival Criteria)
# Data analysis showed false positives with loose criteria
# Old: 100% pass rate with errors up to 3° → New: strict filtering
VALID_FINAL_ERROR = 0.005        # rad (~0.3°) - Max total error for valid arrival
VALID_MAX_JOINT_ERROR = 0.008    # rad (~0.5°) - Max single joint error
VALID_VELOCITY = 0.003           # rad/s - Max velocity (must be nearly stopped)
VALID_PER_JOINT_LIMIT = 0.01     # rad (~0.6°) - All joints must be within this

# 📡 ROS Topics
UNITY_TOPIC = "/unity/joint_cmd"  # รับคำสั่งจาก Unity/VR
RVIZ_TOPIC  = "/joint_states"     # ส่งสถานะไปแสดงใน RViz
DEBUG_TOPIC = "/teleop/debug"     # Debug messages
SAFETY_TOPIC = "/mg400/safety_status" # Safety status reporting
HAPTIC_TOPIC = "/mg400/haptic_feedback" # Collision-based haptic feedback for VR
SUCTION_TOPIC = "/vr/suction_cmd" # สั่งเปิด/ปิดหัวดูดจาก Unity (std_msgs/Bool)
LIGHT_TOPIC = "/mg400/light_cmd"     # สั่งเปิด/ปิดไฟสัญญาณ (std_msgs/Int32MultiArray: [port, status])
DO_STATUS_TOPIC = "/mg400/do_status" # รับสถานะของ Digital Output (std_msgs/Int64)
ROBOT_MODE_TOPIC = "/mg400/robot_mode" # สถานะ Mode ของหุ่นยนต์ (std_msgs/Int32)
ERROR_STATUS_TOPIC = "/mg400/error_status" # สถานะ Error ของหุ่นยนต์ (std_msgs/Int32)

# 🛠️ Hardware Configuration
VACUUM_DO_PORT = 16  # หมายเลขพอร์ต Digital Output สำหรับดูด (Vacuum/Suction)
BLOW_DO_PORT = 15    # หมายเลขพอร์ต Digital Output สำหรับเป่าลม (Pressure/Blow)
GREEN_LIGHT_DO_PORT = 3  # Green Light
YELLOW_LIGHT_DO_PORT = 4 # Yellow Light
RED_LIGHT_DO_PORT = 5    # Red Light
SMART_SUCTION_ENABLED = True        # True = รอหุ่นวิ่งถึงเป้าหมายก่อนถึงสั่งดูด, False = สั่งดูดทันทีที่กดปุ่มใน VR
SUCTION_ACTIVATION_THRESHOLD = 0.05 # rad (~2.8°) - ระยะห่างที่ยอมให้หัวดูดทำงาน (ใช้เมื่อ SMART_SUCTION_ENABLED = True)
BLOW_DURATION = 0.4                 # วินาที: ระยะเวลาที่เป่าลม (Pressure) หลังจากสั่งหยุดดูด ก่อนจะปิดทั้ง 2 พอร์ต (Safety Release)