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
# CP_VALUE chosen via on-robot benchmark (EXP4, 5 reps × 2 scenarios):
# CP=80 yields zero micro-stops on monotonic sweeps where CP=100 still produces
# 1.0 ± 0.0 micro-stops (over-aggressive corner cut at endpoint).  CP=80 also
# matches CP=100 on direction-reversal scenarios.  ACC_VALUE remains 100 so the
# controller has full ramp authority when the per-cmd SpeedJ scales below 100.
ACC_VALUE = 100        # Acceleration value (0-100)
CP_VALUE = 80          # Continuous Path value (smoothness: 0-100)

# (SPEED_FAR/MEDIUM/NEAR distance bands removed — they were only consumed by
# MotionPlanner.calculate_speed() / plan_motion(), neither of which had any
# external caller after the adaptive teleop controller landed.)

# 🎯 Real-Time Adaptive Control Parameters
#
# On-robot benchmarks (EXP3/EXP5/EXP7/EXP8/EXP10) showed that JointMovJ + CP
# produces 0 micro-stops only when the per-command joint delta is large enough
# that the controller has time to enter cruise phase before decel.  Boundaries:
#   J1 (no gravity load):     ≥ 2°  → 0 stops
#   J2/J3 (gravity loaded):   ≥ 3°  → 0 stops
#   ALL joints:               < 1°  → 6+ stops per sweep
# At Δ < 2° the queue-fill state (flood vs paced) makes no difference — the
# decel happens inside each command, not between commands.  See EXP10.
#
# Adaptive gate: hand velocity scales the gate threshold and the per-command
# SpeedJ together so a slow ("aiming") hand uses fine deltas at low speed (no
# perceived jitter because peak velocity is small) while a fast hand uses
# larger deltas at full speed (controller has cruise headroom).  Gate firing
# always sends "latest_target" — never replays stale queue tails.
PROXIMITY_THRESHOLD = 0.08       # rad (~4.5°) - safety/stuck threshold (stuck recovery only)
STUCK_VELOCITY_THRESHOLD = 0.005 # rad/s - ความเร็วต่ำกว่านี้ถือว่า "นิ่ง"
STUCK_TIME_THRESHOLD = 0.3       # seconds - 24/02-style stuck recovery delay
TARGET_CHANGE_THRESHOLD = 0.005  # rad (~0.3°) - stuck recovery min target change

# Adaptive gate: threshold is interpolated between MIN (slow hand) and MAX
# (fast hand) based on max-axis hand velocity.  Hand velocity above HIGH gets
# the MAX threshold; below LOW gets the MIN threshold; in between is a linear
# blend.  Per-command SpeedJ is interpolated in the same band, so big cmds run
# fast and small cmds run slow — keeps the cruise/decel ratio favourable.
REALTIME_DELTA_MIN_RAD = 0.0087   # rad ≈ 0.5°  - finest tracking (slow hand)
REALTIME_DELTA_MAX_RAD = 0.052    # rad ≈ 3.0°  - smooth tracking (fast hand)
REALTIME_HAND_VEL_LOW_RAD_S  = 0.087  # rad/s ≈ 5°/s   - below this: AIM mode
REALTIME_HAND_VEL_HIGH_RAD_S = 0.524  # rad/s ≈ 30°/s  - above this: NORMAL mode
REALTIME_SPEEDJ_MIN = 25          # % - cmd Δ at MIN_RAD uses this SpeedJ
REALTIME_SPEEDJ_MAX = 100         # % - cmd Δ at MAX_RAD uses this SpeedJ
REALTIME_ACCJ_MIN = 25            # % - matches SpeedJ to keep ramp shape consistent
REALTIME_ACCJ_MAX = 100           # %
REALTIME_CP = 80                  # CP for live teleop commands (matches CP_VALUE)
REALTIME_COMMAND_ID_RESPONSE_TIMEOUT_SEC = 0.002  # Bounded wait for port-30003 command id
UNITY_JOINT_CMD_STALE_DROP_SEC = 0.25  # Drop stale live joint targets instead of chasing queue tails.

# Diagnostics only: "passed near" matching for every queued realtime command.
# This is deliberately separate from settled-arrival logging because CP/blending
# can pass through a waypoint without stopping at it.
COMMAND_PASS_NEAR_NORM_TOLERANCE_RAD = 0.01      # ~0.57 deg 4J vector norm
COMMAND_PASS_NEAR_TOOL_XYZ_TOLERANCE_MM = 5.0    # includes current ROS/Dobot tool-frame offset
COMMAND_PASS_NEAR_TOOL_R_TOLERANCE_DEG = 1.0

# (DYNAMIC_PROXIMITY_* legacy aliases removed — no remaining consumers after
# the adaptive controller refactor.  If a downstream tool still imports them,
# update it to read REALTIME_DELTA_MIN_RAD / REALTIME_HAND_VEL_HIGH_RAD_S.)

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
UNITY_TELEOP_SAMPLE_TOPIC = "/unity/teleop_sample"  # std_msgs/String JSON trace sample
UNITY_SPEED_FACTOR_TOPIC = "/unity/speed_factor"
RVIZ_TOPIC  = "/joint_states"     # ส่งสถานะไปแสดงใน RViz
DEBUG_TOPIC = "/teleop/debug"     # Debug messages
SAFETY_TOPIC = "/mg400/safety_status" # Safety status reporting
SCENE_SAFETY_ENABLE_TOPIC = "/mg400/scene_safety_enabled"
HAPTIC_TOPIC = "/mg400/haptic_feedback" # Collision-based haptic feedback for VR
ROS_PING_TOPIC = "/teleop/ros_ping"
UNITY_PONG_TOPIC = "/teleop/unity_pong"
PREDICTED_TARGET_TOPIC = "/teleop/predicted_target"
SENT_COMMAND_TOPIC = "/teleop/sent_command"
UNITY_XYZ_TOPIC = "/teleop/unity_xyz"
PLAYBACK_UNITY_TOPIC = "/teleop/playback_unity"
TRAJ_PREVIEW_TOPIC = "/teleop/traj_preview"
# Job-request contract: explicit teach-and-repeat job submission channel.
# The production Unity scene uses this channel for compile / execute / tune /
# export / stop. Legacy teach topics were removed from the runtime wiring so
# there is one teach-repeat command path to reason about.
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

# Measured scene safety boxes (operator-authored fixture model)
# Disabled by default so existing demos keep their current behavior.  Enable
# with ROS/Unity toggle after loading a verified model.
SCENE_SAFETY_ENABLED_DEFAULT = False
SCENE_SAFETY_MODEL_PATH = ""
SCENE_SAFETY_WARN_DISTANCE_MM = 20.0
SCENE_SAFETY_BLOCK_REALTIME = True

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
# PATH_SIMPLIFY_TOLERANCE_DEG: 3.0 chosen via EXP3/EXP8 — at SpeedJ=80 + CP=80
# the controller goes from 1.25 stops/run @ Δ=2° to 0.0 stops/run @ Δ=3° on
# J2/J3 (gravity-loaded joints).  Setting RDP tolerance ≥ 3° guarantees every
# kept waypoint is at least 3° from its neighbour, so the compiled JointMovJ
# chain runs in the smooth regime end-to-end.  Cost: < 3° corners get rounded
# off — operators teaching tight contours should explicitly mark waypoints.
PATH_SIMPLIFY_TOLERANCE_DEG = 3.0   # was 1.0 — enforce smooth-regime min spacing

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
# On-hardware verification (mixed_test2.py — Arc + Arc + JointMovJ rectangle
# path on real MG400): the classifier itself runs fine, the compiled MovL/Arc
# commands execute correctly, BUT only if PLAYBACK_DEFAULT_SPEED_L is kept
# ≤ ~30%.  At SpeedL=60 the same path triggered controller alarms 96/98 +
# servo 34322 (the original bug-report alarm class) because the planner's
# Cartesian setpoint on Arc segments outruns the J1/J2 servo tracking
# limit.  USE_MIXED_PRIMITIVES is left False by default so the production
# path stays on JointMovJ-only; flip to True per session when an operator
# wants Cartesian primitives and is OK with the slower SpeedL ceiling.
# On-hardware verification (mixed_diag2.py — same Arc+Arc+JointMovJ rectangle
# path on real MG400, full speed×CP matrix):
#
#   SpeedL  CP   alarm
#   ------  --   -----
#   60      80   ❌ controller 96/98 + servo J1/J2 34322 within 0.88s
#   60      30   ✅
#   60       0   ✅
#   30      80   ✅
#   25      80   ✅
#
# Alarm decoded via the project's own RobotErrorDecoder against the
# bundled alarmServo.json:
#   34322 → "Position is out of range" / 位置给定超限保护
#           ("Commanded position exceeded limit / protection").
#           Solution per the table: "Enter the correct parameters".
# Controller alarms 96 + 98 are not present in alarmController.json
# (341 entries, range 16-4193); they appear to be drive-bus-level codes
# the manufacturer does not publish.
#
# Mechanism (best-effort interpretation — the alarm description does not
# distinguish between joint-limit / workspace / velocity / acceleration
# limits, only that the COMMANDED position fell out of allowed range):
# at the Arc1→Arc2 transition, CP=80 rounds the corner so aggressively
# that the controller's blended Cartesian setpoint at some interpolation
# tick goes out of range — joint limit, workspace boundary, or
# transient singularity passage — even though every endpoint is well
# inside the workspace (TCP at the trip was at 251mm radius, J2 at
# -10.9°, CollisionState=0).  Reducing CP keeps the blended path closer
# to the original Arc geometry and avoids the excursion; reducing
# SpeedL gives the controller more time per interpolation tick which
# also helps but is not the primary driver.
#
# This is not a speed-rate-tracking failure as the previous diagnosis
# suggested — that hypothesis turned out to be wrong once the alarm
# table was actually consulted.
#
# Mitigation in code: SEGMENT_CARTESIAN_CP_MAX caps the CP threaded into
# MovL/Arc commands during mixed-primitive compile.  JointMovJ segments
# still see the full PLAYBACK_DEFAULT_CP because joint blends do not
# trigger this Cartesian-setpoint-rate failure.
#
# USE_MIXED_PRIMITIVES is left False by default so the production path
# stays on JointMovJ-only; flip to True per session when an operator
# wants Cartesian primitives.
SEGMENT_CARTESIAN_CP_MAX = 30       # cap CP threaded into MovL/Arc — see comment above
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
#
# LEGACY (2026-05): the ``fastest_path_repeat`` branch and its
# FAST_REPEAT_* constants below are currently dormant — the default
# profile is ``preserve_timing`` and no /teach/job_request payload from
# Unity sets the alternative.  Kept enabled so that operators with a
# saved launch override (or a planned demo recording mode) do not break.
# See AGENTS.md §4.3 for the proof-of-death checklist before removal.
PLAYBACK_EXECUTION_PROFILE = "preserve_timing"  # preserve_timing | fastest_path_repeat
FAST_REPEAT_SPEED_J = 100            # LEGACY 4.3 — dormant profile constants
FAST_REPEAT_ACC_J = 100              # LEGACY 4.3
FAST_REPEAT_SPEED_L = 100            # LEGACY 4.3
FAST_REPEAT_ACC_L = 100              # LEGACY 4.3
FAST_REPEAT_CP = 100                 # LEGACY 4.3
FAST_REPEAT_FINAL_CP = 0             # LEGACY 4.3
FAST_REPEAT_LOOKAHEAD_SEC = 10.0     # LEGACY 4.3
FAST_REPEAT_TIMEOUT_PER_COMMAND_SEC = 1.0  # LEGACY 4.3
FAST_REPEAT_MAX_COMMANDS_PER_CYCLE = 1     # LEGACY 4.3
