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
QUEUE_BACKLOG_GATE_RAD = 0.01          # rad (~0.57°) - queue must nearly drain before sending next motion
QUEUE_BUSY_ESCAPE_SEC = 0.30           # seconds - allow stuck recovery if queue state stays busy while motion stops

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
ROS_PING_TOPIC = "/teleop/ros_ping"
UNITY_PONG_TOPIC = "/teleop/unity_pong"
PREDICTED_TARGET_TOPIC = "/teleop/predicted_target"
SENT_COMMAND_TOPIC = "/teleop/sent_command"
UNITY_XYZ_TOPIC = "/teleop/unity_xyz"
PLAYBACK_UNITY_TOPIC = "/teleop/playback_unity"
TRAJ_PREVIEW_TOPIC = "/teleop/traj_preview"
TEACH_STATUS_TOPIC = "/unity/teach_status"
TRAJECTORY_DATA_TOPIC = "/unity/trajectory_data"
UNITY_TRAJECTORY_TOPIC = "/mg400/joint_trajectory_controller/command"
# Job-request contract: explicit teach-and-repeat job submission channel.
# Replaces the legacy "publish JointTrajectory == execute now" behaviour with a
# typed action enum (compile / preview_sim / execute / export / stop / record_*)
# so the ROS adapter can decide between dry compile, simulator preview, real
# robot execute, or artifact export without overloading topic semantics.
TEACH_JOB_REQUEST_TOPIC = "/teach/job_request"
TEACH_JOB_STATUS_TOPIC = "/teach/job_status"
TEACH_JOB_ARTIFACT_TOPIC = "/teach/job_artifact"
TOOL_ACTUAL_TOPIC = "/mg400/tool_vector_actual"
TOOL_TARGET_TOPIC = "/mg400/tool_vector_target"
FLANGE_ACTUAL_TOPIC = "/robot/flange_actual"
TOOL_INDEX_TOPIC = "/robot/tool_index"
DASHBOARD_CMD_TOPIC = "/robot/dashboard_cmd"
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

# 📐 Path simplification (RDP)
# Unity records joint state at ~20 FPS so a single demonstration produces
# hundreds of dense waypoints, each one becoming its own JointMovJ command in
# the MG400 motion queue.  The controller blends linearly between commanded
# waypoints, so collapsing near-collinear runs into their endpoints lets the
# robot follow the same path with far fewer queued commands and less risk of
# the queue backing up faster than playback can drain it.
#
# Tolerance is the max permitted joint deviation (degrees) between any
# dropped waypoint and the time-lerp of its surrounding kept waypoints.
# 0.0 disables simplification (use during diagnosis or when a path must be
# replayed verbatim).  0.5° matches the recorder's RECORD_MIN_DELTA so the
# simplifier does not collapse motion that recording considered worth
# capturing in the first place.
PATH_SIMPLIFY_TOLERANCE_DEG = 1.0   # was 0.5 — doubled: smoother curves drop more intermediate pts

# 🔀 Mixed-primitive segment classification
# After RDP simplifies the waypoint count, the segment classifier groups
# consecutive waypoints into typed segments that map to the most efficient
# MG400 motion command:
#   LINE    → MovL   (single Cartesian linear command)
#   ARC     → Arc    (single Cartesian arc via 3 defining points)
#   GENERAL → JointMovJ chain (fallback, one command per waypoint)
#
# This dramatically reduces queue depth for trajectories with long straight
# runs (pick-place) or smooth curves (freehand demonstrations).
#
# Demo default: keep the real-robot path on JointMovJ-only unless an operator
# explicitly enables mixed primitives for offline analysis.  The classifier is
# still available and tested, but recent hardware trials showed Arc/MovL can
# trigger controller alarms when the fitted path is close to fixture limits.
USE_MIXED_PRIMITIVES = False
SEGMENT_ENABLE_ARC = True           # Arc(through, end) — 1 cmd per smooth curve vs N JointMovJ
SEGMENT_LINE_TOLERANCE_MM = 2.0    # max XYZ deviation (mm) from chord for MovL classification
SEGMENT_ARC_TOLERANCE_MM = 3.0     # max 3D deviation (mm) from least-squares fitted circle for Arc
SEGMENT_RAW_FIT_TOLERANCE_MM = 5.0 # max allowed distance from original raw Unity path after RDP
SEGMENT_MAX_ARC_RADIUS_MM = 10000.0 # arc with radius > this is reclassified as LINE (effectively straight)
SEGMENT_R_TOLERANCE_DEG = 10.0      # max R/yaw deviation from endpoint interpolation for MovL/Arc
SEGMENT_MIN_POINTS_FOR_ARC = 3     # minimum waypoints to attempt arc fit
# MG400 interpolates R linearly between endpoint poses on MovL/Arc.  Small VR
# wrist wobble is acceptable, but meaningful non-linear wrist motion should
# remain GENERAL so intermediate JointMovJ waypoints preserve it.

# Dynamic Cartesian Speed Configuration
# The system calculates the physical Cartesian distance of the segment and the
# desired timestamp delta to compute the required mm/s velocity.
# SpeedL is then calculated as: (velocity_mm_s / CARTESIAN_SPEED_AT_100_PERCENT_MM_S) * 100
CARTESIAN_SPEED_AT_100_PERCENT_MM_S = 1000.0  # Reference 100% Cartesian speed (mm/s)
SEGMENT_MIN_SPEED_L = 5                       # Minimum SpeedL percentage
SEGMENT_MAX_SPEED_L = 100                     # Maximum SpeedL percentage
SEGMENT_ACC_L = 80                            # Default AccL % for Cartesian commands

# Teach-and-repeat execution profile
# preserve_timing:
#   Replay the taught timestamps as closely as possible.  This is useful for
#   studies that care about human demonstration timing, but it can command low
#   SpeedL/SpeedJ values when the hand moved slowly.
# fastest_path_repeat:
#   Preserve the taught path geometry, but do not preserve the hand timestamps.
#   The compiled program uses the configured fast speed/acc/CP limits and queues
#   ahead aggressively so the MG400 can execute as fast as its controller and
#   mechanical constraints allow.
PLAYBACK_EXECUTION_PROFILE = "preserve_timing"  # preserve_timing | fastest_path_repeat
FAST_REPEAT_SPEED_J = 100
FAST_REPEAT_ACC_J = 100
FAST_REPEAT_SPEED_L = 100
FAST_REPEAT_ACC_L = 100
FAST_REPEAT_CP = 100
FAST_REPEAT_FINAL_CP = 0
FAST_REPEAT_LOOKAHEAD_SEC = 10.0
FAST_REPEAT_TIMEOUT_PER_COMMAND_SEC = 1.0
FAST_REPEAT_MAX_COMMANDS_PER_CYCLE = 1
