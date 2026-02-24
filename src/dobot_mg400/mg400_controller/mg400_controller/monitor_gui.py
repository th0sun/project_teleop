#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🤖 MG400 Joint Monitor GUI
Simple GUI to visualize robot joint angles in real-time.
Now includes Target vs Actual comparison, Latency monitoring, Execution Metrics,
End Effector (XYZ) Monitoring, Real-Time 4-Line Joint Tracking Graphs,
and Auto-Session Logging.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import sys
import time
import math
import csv
import datetime
import os
from collections import deque

# --- Matplotlib ---
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

# Import Configuration
from mg400_controller.common.config.motion_config import (
    UNITY_TOPIC, SUCTION_TOPIC, LIGHT_TOPIC, 
    VACUUM_DO_PORT, BLOW_DO_PORT, 
    GREEN_LIGHT_DO_PORT, YELLOW_LIGHT_DO_PORT, RED_LIGHT_DO_PORT,
    DO_STATUS_TOPIC, ROBOT_MODE_TOPIC, ERROR_STATUS_TOPIC
)
from std_msgs.msg import Bool, Int32MultiArray, Int64, Int32

# Configuration
ACTUAL_TOPIC_NAME = "/joint_states"
TARGET_TOPIC_NAME = UNITY_TOPIC
TOOL_ACTUAL_TOPIC = "/mg400/tool_vector_actual"
TOOL_TARGET_TOPIC = "/mg400/tool_vector_target"
PREDICTED_TOPIC = "/teleop/predicted_target"
SENT_CMD_TOPIC = "/teleop/sent_command"

# ── Dark Theme Palette ────────────────────────────────────────────
BG_ROOT   = "#0d1117"   # window background
BG_CARD   = "#161b22"   # card / panel bg
BG_ROW    = "#1c2333"   # alternating row
BG_HEADER = "#21262d"   # table header row
FG_TEXT   = "#c9d1d9"   # primary text
FG_DIM    = "#8b949e"   # secondary / dim text
FG_ACCENT = "#58a6ff"   # blue accent
BORDER    = "#30363d"   # subtle border

# Light pill colors for status
COL_GOOD   = "#3fb950"   # green
COL_WARN   = "#d29922"   # yellow
COL_ERR    = "#f85149"   # red
COL_IDLE   = "#8b949e"   # gray

# Lights
COLOR_OFF    = "#30363d"
COLOR_GREEN  = "#3fb950"
COLOR_YELLOW = "#d29922"
COLOR_RED    = "#f85149"
COLOR_VACUUM = "#58a6ff"

# Graph colors (unchanged)
COL_UNITY     = "#f39c12"
COL_PREDICTED = "#9b59b6"
COL_SENT      = "#27ae60"
COL_ACTUAL    = "#2980b9"

# Fonts
FONT_HEADER    = ("Helvetica", 13, "bold")
FONT_SECTION   = ("Helvetica", 10, "bold")
FONT_LABEL     = ("Helvetica", 10)
FONT_VALUE     = ("Courier",   11, "bold")   # monospace for numbers
FONT_BIG_VALUE = ("Courier",   22, "bold")
FONT_LATENCY   = ("Helvetica",  9)
FONT_STATS     = ("Helvetica", 10)

# Graph config
GRAPH_WINDOW_SEC = 10.0
GRAPH_UPDATE_HZ  = 20

# Motion Detection Thresholds
START_THRESHOLD = 2.0
STOP_THRESHOLD  = 0.5

def _card(parent, **kw):
    """Dark card frame helper."""
    return tk.Frame(parent, bg=BG_CARD, relief="flat", **kw)

def _lbl(parent, text="", fg=FG_TEXT, font=FONT_LABEL, **kw):
    return tk.Label(parent, text=text, fg=fg, bg=BG_CARD, font=font, **kw)

def _val(parent, var, fg=FG_TEXT, font=FONT_VALUE, **kw):
    return tk.Label(parent, textvariable=var, fg=fg, bg=BG_CARD, font=font, **kw)


class SessionLogger:
    """
    Auto-starts on GUI launch.
    Creates ~/project_teleop_ws/session_logs/YYYYMMDD_HHMMSS/ per session.
    Logs all 4 joint streams (Unity, Predicted, Sent, Actual) + XYZ to CSV.
    Timestamp = real wall-clock time (UTC+7 or system local time), accurate.
    """
    BASE_DIR = os.path.expanduser("~/project_teleop_ws/session_logs")

    def __init__(self):
        # Create session folder e.g. session_logs/20260225_032100/
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.BASE_DIR, ts)
        os.makedirs(self.session_dir, exist_ok=True)

        # --- joints_tracking.csv ---
        jt_path = os.path.join(self.session_dir, "joints_tracking.csv")
        self._jt_file = open(jt_path, 'w', newline='')
        self._jt_writer = csv.writer(self._jt_file)
        self._jt_writer.writerow([
            "timestamp", "elapsed_s",
            "unity_j1", "unity_j2", "unity_j3", "unity_j4",
            "predicted_j1", "predicted_j2", "predicted_j3", "predicted_j4",
            "sent_j1", "sent_j2", "sent_j3", "sent_j4",
            "actual_j1", "actual_j2", "actual_j3", "actual_j4",
        ])

        # --- xyz_tracking.csv ---
        xyz_path = os.path.join(self.session_dir, "xyz_tracking.csv")
        self._xyz_file = open(xyz_path, 'w', newline='')
        self._xyz_writer = csv.writer(self._xyz_file)
        self._xyz_writer.writerow([
            "timestamp", "elapsed_s",
            "target_x", "target_y", "target_z",
            "actual_x", "actual_y", "actual_z",
            "diff_x", "diff_y", "diff_z",
        ])

        self._start_time = time.time()
        self._lock = threading.Lock()
        print(f"[SessionLogger] Logging to: {self.session_dir}")

    def log_joints(self, unity, predicted, sent, actual):
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed = round(time.time() - self._start_time, 3)
        row = [ts, elapsed] + \
              [round(v, 4) for v in unity] + \
              [round(v, 4) for v in predicted] + \
              [round(v, 4) for v in sent] + \
              [round(v, 4) for v in actual]
        with self._lock:
            self._jt_writer.writerow(row)

    def log_xyz(self, target, actual):
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed = round(time.time() - self._start_time, 3)
        diff = [round(actual[i] - target[i], 3) for i in range(3)]
        row = [ts, elapsed] + \
              [round(v, 3) for v in target] + \
              [round(v, 3) for v in actual] + diff
        with self._lock:
            self._xyz_writer.writerow(row)

    def flush(self):
        with self._lock:
            self._jt_file.flush()
            self._xyz_file.flush()

    def close(self):
        with self._lock:
            self._jt_file.close()
            self._xyz_file.close()


class ExecutionMonitor:
    def __init__(self):
        self.state = "IDLE" # IDLE, MOVING, ARRIVED
        self.start_time = 0.0
        self.end_time = 0.0
        self.last_duration = 0.0
        self.durations = []
        
    def update(self, total_error):
        now = time.time()
        if self.state == "IDLE" or self.state == "ARRIVED":
            if total_error > START_THRESHOLD:
                self.state = "MOVING"
                self.start_time = now
                return "STARTED"
        elif self.state == "MOVING":
            if total_error < STOP_THRESHOLD:
                self.state = "ARRIVED"
                self.end_time = now
                self.last_duration = self.end_time - self.start_time
                self.durations.append(self.last_duration)
                return "FINISHED"
        return self.state

    def get_stats(self):
        if not self.durations:
            return 0.0, 0.0, 0.0
        return np.mean(self.durations), np.min(self.durations), np.max(self.durations)


class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('mg400_joint_monitor')
        
        # Publishers for Controls
        self.pub_suction = self.create_publisher(Bool, SUCTION_TOPIC, 10)
        self.pub_light = self.create_publisher(Int32MultiArray, LIGHT_TOPIC, 10)
        
        # Joint variables
        self.latest_actual_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_target_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_predicted_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_sent_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_tool_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_tool_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_unity_xyz   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # FK of Unity input
        self.latest_flange_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # FK of actual joints
        self.latest_tool_index = -1  # active tool index from GetTool()
        self.latest_do_status = 0
        self.latest_robot_mode = 0
        self.latest_error_status = 0
        self.last_target_time = 0.0
        self.last_actual_time = 0.0

        # Subscriptions
        self.sub_actual = self.create_subscription(JointState, ACTUAL_TOPIC_NAME, self.listener_callback_actual, 10)
        self.sub_target = self.create_subscription(JointState, TARGET_TOPIC_NAME, self.listener_callback_target, 10)
        self.sub_predicted = self.create_subscription(JointState, PREDICTED_TOPIC, self.listener_callback_predicted, 10)
        self.sub_sent = self.create_subscription(JointState, SENT_CMD_TOPIC, self.listener_callback_sent, 10)
        self.sub_tool_actual = self.create_subscription(Float64MultiArray, TOOL_ACTUAL_TOPIC, self.listener_callback_tool_actual, 10)
        self.sub_tool_target = self.create_subscription(Float64MultiArray, TOOL_TARGET_TOPIC, self.listener_callback_tool_target, 10)
        self.sub_unity_xyz = self.create_subscription(Float64MultiArray, "/teleop/unity_xyz", self.listener_callback_unity_xyz, 10)
        self.sub_flange_actual = self.create_subscription(Float64MultiArray, "/robot/flange_actual", self.listener_callback_flange_actual, 10)
        self.sub_tool_index = self.create_subscription(Int32, "/robot/tool_index", self.listener_callback_tool_index, 10)
        self.sub_do_status = self.create_subscription(Int64, DO_STATUS_TOPIC, self.listener_callback_do_status, 10)
        self.sub_robot_mode = self.create_subscription(Int32, ROBOT_MODE_TOPIC, self.listener_callback_robot_mode, 10)
        self.sub_error_status = self.create_subscription(Int32, ERROR_STATUS_TOPIC, self.listener_callback_error_status, 10)

    def request_suction(self, state):
        msg = Bool()
        msg.data = state
        self.pub_suction.publish(msg)

    def request_light(self, port, state):
        msg = Int32MultiArray()
        msg.data = [port, int(state)]
        self.pub_light.publish(msg)

    def listener_callback_actual(self, msg):
        if len(msg.position) >= 9:
            q_rad = [msg.position[0], msg.position[1], msg.position[3], msg.position[8]]
            self.latest_actual_joints = list(np.degrees(q_rad))
            self.last_actual_time = time.time()

    def listener_callback_target(self, msg):
        if len(msg.position) >= 4:
            self.latest_target_joints = list(np.degrees(msg.position[:4]))
            self.last_target_time = time.time()

    def listener_callback_predicted(self, msg):
        if len(msg.position) >= 4:
            self.latest_predicted_joints = list(np.degrees(msg.position[:4]))

    def listener_callback_sent(self, msg):
        if len(msg.position) >= 4:
            self.latest_sent_joints = list(np.degrees(msg.position[:4]))
            
    def listener_callback_tool_actual(self, msg):
        if len(msg.data) >= 6: self.latest_tool_actual = list(msg.data)
            
    def listener_callback_tool_target(self, msg):
        if len(msg.data) >= 6: self.latest_tool_target = list(msg.data)

    def listener_callback_unity_xyz(self, msg):
        if len(msg.data) >= 6: self.latest_unity_xyz = list(msg.data)

    def listener_callback_flange_actual(self, msg):
        if len(msg.data) >= 6: self.latest_flange_actual = list(msg.data)

    def listener_callback_tool_index(self, msg):
        self.latest_tool_index = int(msg.data)
        
    def listener_callback_do_status(self, msg):
        self.latest_do_status = int(msg.data)
        
    def listener_callback_robot_mode(self, msg):
        self.latest_robot_mode = int(msg.data)
        
    def listener_callback_error_status(self, msg):
        self.latest_error_status = int(msg.data)


class MonitorGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.monitor = ExecutionMonitor()
        self.lockout = {}

        self.root.title("MG400 Extended Monitor")
        self.root.configure(bg=BG_ROOT)

        # ── MAIN LAYOUT ───────────────────────────────────────────
        outer = tk.Frame(root, bg=BG_ROOT)
        outer.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(outer, bg=BG_ROOT, width=620)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH)
        left_frame.pack_propagate(False)

        right_frame = tk.Frame(outer, bg=BG_ROOT)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # scrollable canvas for left panel content
        canvas_scroll = tk.Canvas(left_frame, bg=BG_ROOT, highlightthickness=0)
        scrollbar = tk.Scrollbar(left_frame, orient="vertical", command=canvas_scroll.yview)
        canvas_scroll.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas_scroll.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        main_frame = tk.Frame(canvas_scroll, bg=BG_ROOT)
        canvas_win = canvas_scroll.create_window((0, 0), window=main_frame, anchor="nw")
        main_frame.bind("<Configure>", lambda e: canvas_scroll.configure(
            scrollregion=canvas_scroll.bbox("all")))
        canvas_scroll.bind("<Configure>", lambda e: canvas_scroll.itemconfig(canvas_win, width=e.width))

        def _section(parent, title):
            """Titled dark card section."""
            tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=12, pady=(14, 0))
            hdr = tk.Frame(parent, bg=BG_HEADER)
            hdr.pack(fill=tk.X, padx=12)
            tk.Label(hdr, text=f"  {title}", font=FONT_SECTION, fg=FG_ACCENT,
                     bg=BG_HEADER, anchor="w").pack(side=tk.LEFT, ipady=4)
            card = tk.Frame(parent, bg=BG_CARD)
            card.pack(fill=tk.X, padx=12, pady=(0, 4))
            return card

        # ── TITLE ROW ─────────────────────────────────────────────
        title_row = tk.Frame(main_frame, bg=BG_ROOT)
        title_row.pack(fill=tk.X, padx=12, pady=(10, 4))
        tk.Label(title_row, text="MG400 Monitor", font=FONT_HEADER,
                 fg=FG_TEXT, bg=BG_ROOT).pack(side=tk.LEFT)
        self.is_logging = False
        self.log_start_time = 0.0
        self.btn_log = tk.Button(title_row, text="● REC", font=FONT_SECTION,
                                 bg=BG_CARD, fg=COL_IDLE, activebackground=BG_ROW,
                                 relief="flat", padx=8, command=self.toggle_logging)
        self.btn_log.pack(side=tk.RIGHT)

        # ── SESSION LOG LABEL ──────────────────────────────────────
        self.var_log_path = tk.StringVar(value="Session: starting…")
        tk.Label(main_frame, textvariable=self.var_log_path, font=FONT_LATENCY,
                 fg=FG_DIM, bg=BG_ROOT, anchor="w").pack(fill=tk.X, padx=14)

        # ── JOINTS TABLE ──────────────────────────────────────────
        card_j = _section(main_frame, "JOINT ANGLES")
        hrow = tk.Frame(card_j, bg=BG_HEADER)
        hrow.pack(fill=tk.X)
        for txt, w, fg in [("", 6, FG_DIM), ("Unity °", 10, COL_UNITY),
                            ("Actual °", 10, COL_ACTUAL), ("Δ °", 10, FG_DIM)]:
            tk.Label(hrow, text=txt, font=FONT_SECTION, fg=fg,
                     bg=BG_HEADER, width=w, anchor="e").pack(side=tk.LEFT, padx=2, pady=3)

        self.vars_target = []
        self.vars_actual = []
        self.vars_diff   = []
        self.lbls_diff   = []

        for i, name in enumerate(["J1", "J2", "J3", "J4"]):
            rowbg = BG_ROW if i % 2 else BG_CARD
            row = tk.Frame(card_j, bg=rowbg)
            row.pack(fill=tk.X)
            tk.Label(row, text=name, font=FONT_LABEL, fg=FG_DIM,
                     bg=rowbg, width=6, anchor="w").pack(side=tk.LEFT, padx=(8, 0))

            v_tgt = tk.StringVar(value="0.00")
            tk.Label(row, textvariable=v_tgt, font=FONT_VALUE, fg=COL_UNITY,
                     bg=rowbg, width=10, anchor="e").pack(side=tk.LEFT)
            self.vars_target.append(v_tgt)

            v_act = tk.StringVar(value="0.00")
            tk.Label(row, textvariable=v_act, font=FONT_VALUE, fg=COL_ACTUAL,
                     bg=rowbg, width=10, anchor="e").pack(side=tk.LEFT)
            self.vars_actual.append(v_act)

            v_diff = tk.StringVar(value="0.00")
            lbl = tk.Label(row, textvariable=v_diff, font=FONT_VALUE, fg=FG_DIM,
                           bg=rowbg, width=10, anchor="e")
            lbl.pack(side=tk.LEFT, padx=(0, 6))
            self.vars_diff.append(v_diff)
            self.lbls_diff.append(lbl)

        # ── CARTESIAN XYZ ─────────────────────────────────────────
        card_xyz = _section(main_frame, "CARTESIAN — END EFFECTOR")
        hrow2 = tk.Frame(card_xyz, bg=BG_HEADER)
        hrow2.pack(fill=tk.X)
        for txt, w, fg in [("", 5, FG_DIM), ("Unity FK", 10, COL_UNITY),
                            ("Flange", 10, FG_ACCENT), ("TCP", 10, COL_GOOD),
                            ("ToolΔ", 9, FG_DIM)]:
            tk.Label(hrow2, text=txt, font=FONT_SECTION, fg=fg,
                     bg=BG_HEADER, width=w, anchor="e").pack(side=tk.LEFT, padx=2, pady=3)

        self.vars_xyz_tgt    = []
        self.vars_xyz_flange = []
        self.vars_xyz_act    = []
        self.vars_xyz_tool   = []
        self.vars_xyz_diff   = []  # compat alias
        self.lbls_xyz_diff   = []

        for i, ax in enumerate(["X", "Y", "Z"]):
            rowbg = BG_ROW if i % 2 else BG_CARD
            row = tk.Frame(card_xyz, bg=rowbg)
            row.pack(fill=tk.X)
            tk.Label(row, text=ax, font=FONT_LABEL, fg=FG_DIM,
                     bg=rowbg, width=5, anchor="w").pack(side=tk.LEFT, padx=(8, 0))

            v_tgt = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=v_tgt, font=FONT_VALUE, fg=COL_UNITY,
                     bg=rowbg, width=10, anchor="e").pack(side=tk.LEFT)
            self.vars_xyz_tgt.append(v_tgt)

            v_fl = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=v_fl, font=FONT_VALUE, fg=FG_ACCENT,
                     bg=rowbg, width=10, anchor="e").pack(side=tk.LEFT)
            self.vars_xyz_flange.append(v_fl)

            v_act = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=v_act, font=FONT_VALUE, fg=COL_GOOD,
                     bg=rowbg, width=10, anchor="e").pack(side=tk.LEFT)
            self.vars_xyz_act.append(v_act)

            v_tool = tk.StringVar(value="+0.0")
            lbl_t = tk.Label(row, textvariable=v_tool, font=FONT_VALUE, fg=FG_DIM,
                             bg=rowbg, width=9, anchor="e")
            lbl_t.pack(side=tk.LEFT, padx=(0, 6))
            self.vars_xyz_tool.append(v_tool)
            self.vars_xyz_diff.append(v_tool)
            self.lbls_xyz_diff.append(lbl_t)

        # Active Tool row
        tool_row = tk.Frame(card_xyz, bg=BG_CARD)
        tool_row.pack(fill=tk.X, pady=(2, 4))
        tk.Label(tool_row, text="Active Tool:", font=FONT_LABEL,
                 fg=FG_DIM, bg=BG_CARD, padx=10).pack(side=tk.LEFT)
        self.var_tool_index = tk.StringVar(value="— querying…")
        tk.Label(tool_row, textvariable=self.var_tool_index, font=FONT_VALUE,
                 fg=FG_DIM, bg=BG_CARD).pack(side=tk.LEFT)

        # ── CONTROL PANEL ─────────────────────────────────────────
        card_ctrl = _section(main_frame, "DIRECT CONTROL")

        suction_row = tk.Frame(card_ctrl, bg=BG_CARD)
        suction_row.pack(fill=tk.X, padx=8, pady=6)
        tk.Label(suction_row, text="Suction", font=FONT_LABEL,
                 fg=FG_DIM, bg=BG_CARD, width=8, anchor="w").pack(side=tk.LEFT)
        self.suction_state = False
        self.btn_suction = tk.Button(suction_row, text="OFF", font=FONT_SECTION,
                                      bg=COLOR_OFF, fg=FG_TEXT, activebackground=BG_ROW,
                                      relief="flat", padx=14, command=self.toggle_suction)
        self.btn_suction.pack(side=tk.LEFT, padx=4)

        light_row = tk.Frame(card_ctrl, bg=BG_CARD)
        light_row.pack(fill=tk.X, padx=8, pady=(0, 8))
        tk.Label(light_row, text="Lights", font=FONT_LABEL,
                 fg=FG_DIM, bg=BG_CARD, width=8, anchor="w").pack(side=tk.LEFT)

        self.light_states = {"GREEN": False, "YELLOW": False, "RED": False}
        self.btns_light = {}
        for name, port, col in [("GREEN", GREEN_LIGHT_DO_PORT, COLOR_GREEN),
                                  ("YELLOW", YELLOW_LIGHT_DO_PORT, COLOR_YELLOW),
                                  ("RED", RED_LIGHT_DO_PORT, COLOR_RED)]:
            btn = tk.Button(light_row, text=name, font=FONT_SECTION,
                            bg=COLOR_OFF, fg=FG_TEXT, relief="flat", padx=10,
                            command=lambda n=name, p=port, c=col: self.toggle_light(n, p, c))
            btn.pack(side=tk.LEFT, padx=3)
            self.btns_light[name] = btn

        # ── EXECUTION METRICS ─────────────────────────────────────
        card_ex = _section(main_frame, "EXECUTION METRICS")
        ex_row1 = tk.Frame(card_ex, bg=BG_CARD)
        ex_row1.pack(fill=tk.X, padx=10, pady=(6, 0))

        self.var_status = tk.StringVar(value="IDLE")
        self.lbl_status = tk.Label(ex_row1, textvariable=self.var_status,
                                    font=FONT_SECTION, fg=COL_IDLE, bg=BG_CARD)
        self.lbl_status.pack(side=tk.LEFT)

        self.var_timer = tk.StringVar(value="0.00s")
        self.lbl_timer = tk.Label(ex_row1, textvariable=self.var_timer,
                                   font=FONT_BIG_VALUE, fg=FG_TEXT, bg=BG_CARD)
        self.lbl_timer.pack(side=tk.RIGHT)

        self.var_stats = tk.StringVar(value="Avg: —  Min: —  Max: —")
        tk.Label(card_ex, textvariable=self.var_stats, font=FONT_STATS,
                 fg=FG_DIM, bg=BG_CARD, anchor="e").pack(fill=tk.X, padx=10, pady=(0, 8))

        # ── STATUS BAR ────────────────────────────────────────────
        status_bar = tk.Frame(main_frame, bg=BG_HEADER)
        status_bar.pack(fill=tk.X, padx=12, pady=(8, 12))

        self.var_latency = tk.StringVar(value="Waiting for data…")
        tk.Label(status_bar, textvariable=self.var_latency, font=FONT_LATENCY,
                 fg=FG_DIM, bg=BG_HEADER).pack(side=tk.LEFT, padx=6, pady=4)

        self.var_do_hex = tk.StringVar(value="DO: 0x0000")
        tk.Label(status_bar, textvariable=self.var_do_hex, font=FONT_LATENCY,
                 fg=FG_DIM, bg=BG_HEADER).pack(side=tk.LEFT, padx=6)

        self.var_error = tk.StringVar(value="ERR: 00")
        self.lbl_error = tk.Label(status_bar, textvariable=self.var_error,
                                   font=FONT_SECTION, fg=COL_ERR, bg=BG_HEADER)
        self.lbl_error.pack(side=tk.RIGHT, padx=6)

        self.var_mode = tk.StringVar(value="MODE: —")
        tk.Label(status_bar, textvariable=self.var_mode, font=FONT_LATENCY,
                 fg=FG_DIM, bg=BG_HEADER).pack(side=tk.RIGHT, padx=6)

        # ── RIGHT PANEL: GRAPHS ───────────────────────────────────
        self._setup_graphs(right_frame)

        # Start update loop
        self.update_gui()

    def _setup_graphs(self, parent):
        """Create the matplotlib figure with 4 subplots for J1-J4."""
        # Legend label strip at top
        legend_frame = tk.Frame(parent, bg="#1a1a2e")
        legend_frame.pack(fill=tk.X, padx=10, pady=(10, 0))
        
        tk.Label(legend_frame, text="📊 Joint Tracking Graphs", font=("Helvetica", 13, "bold"),
                 bg="#1a1a2e", fg="white").pack(side=tk.LEFT)
        
        legend_right = tk.Frame(legend_frame, bg="#1a1a2e")
        legend_right.pack(side=tk.RIGHT)
        
        legends = [
            ("Unity Raw", COL_UNITY),
            ("Predicted", COL_PREDICTED),
            ("Cmd Sent", COL_SENT),
            ("Actual", COL_ACTUAL),
        ]
        for lname, lcolor in legends:
            tk.Label(legend_right, text=f"─ {lname}", font=("Helvetica", 10, "bold"),
                     bg="#1a1a2e", fg=lcolor).pack(side=tk.LEFT, padx=8)

        # Matplotlib Figure
        self.fig = Figure(figsize=(8, 8), dpi=96, facecolor="#1a1a2e")
        self.fig.subplots_adjust(hspace=0.4, left=0.12, right=0.97, top=0.97, bottom=0.07)
        
        joint_labels = ["J1 (°)", "J2 (°)", "J3 (°)", "J4 (°)"]
        self.axes = []
        self.graph_lines = []  # list of (unity_line, pred_line, sent_line, actual_line) per joint
        
        # Rolling time axis (relative time in seconds)
        max_points = int(GRAPH_WINDOW_SEC * 50)  # 50Hz max data rate => 500 pts
        self.time_buffer = deque(maxlen=max_points)
        self.unity_buffers = [deque(maxlen=max_points) for _ in range(4)]
        self.pred_buffers = [deque(maxlen=max_points) for _ in range(4)]
        self.sent_buffers = [deque(maxlen=max_points) for _ in range(4)]
        self.actual_buffers = [deque(maxlen=max_points) for _ in range(4)]
        self.last_sent_values = [0.0] * 4  # Hold-last for staircase sent line
        self.graph_start_time = time.time()
        
        # ✅ Auto-start session logger
        self.session_logger = SessionLogger()
        self._log_flush_counter = 0
        
        for i in range(4):
            ax = self.fig.add_subplot(4, 1, i + 1)
            ax.set_facecolor("#0d0d1a")
            ax.set_ylabel(joint_labels[i], color="white", fontsize=9)
            ax.tick_params(colors="gray", labelsize=8)
            ax.spines['bottom'].set_color('#444')
            ax.spines['left'].set_color('#444')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(True, color="#2a2a3e", linewidth=0.5, linestyle="--")

            l_unity,  = ax.plot([], [], color=COL_UNITY, lw=1.2, linestyle="--", alpha=0.8, label="Unity")
            l_pred,   = ax.plot([], [], color=COL_PREDICTED, lw=1.0, alpha=0.85, label="Predicted")
            l_sent,   = ax.plot([], [], color=COL_SENT, lw=2.0, drawstyle="steps-post", alpha=0.9, label="Sent")
            l_actual, = ax.plot([], [], color=COL_ACTUAL, lw=1.5, label="Actual")

            if i == 3:
                ax.set_xlabel("Time (s)", color="gray", fontsize=9)

            self.axes.append(ax)
            self.graph_lines.append((l_unity, l_pred, l_sent, l_actual))
        
        canvas = FigureCanvasTkAgg(self.fig, master=parent)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.canvas = canvas
        
        # Animation - drives graph redraws
        self.ani = animation.FuncAnimation(
            self.fig, self._update_graphs,
            interval=int(1000 / GRAPH_UPDATE_HZ),
            blit=True,
            cache_frame_data=False
        )

    def _update_graphs(self, frame):
        """Called by matplotlib animation to refresh graph lines."""
        now = time.time()
        rel_t = now - self.graph_start_time
        
        # Push new samples into buffers
        self.time_buffer.append(rel_t)
        for i in range(4):
            self.unity_buffers[i].append(self.node.latest_target_joints[i])
            self.pred_buffers[i].append(self.node.latest_predicted_joints[i])
            # Command Sent: hold last value (staircase) until a new one arrives
            if self.node.latest_sent_joints[i] != self.last_sent_values[i]:
                self.last_sent_values[i] = self.node.latest_sent_joints[i]
            self.sent_buffers[i].append(self.last_sent_values[i])
            self.actual_buffers[i].append(self.node.latest_actual_joints[i])

        t_arr = np.array(self.time_buffer)
        
        all_lines = []
        for i in range(4):
            l_unity, l_pred, l_sent, l_actual = self.graph_lines[i]
            ax = self.axes[i]
            
            u = np.array(self.unity_buffers[i])
            p = np.array(self.pred_buffers[i])
            s = np.array(self.sent_buffers[i])
            a = np.array(self.actual_buffers[i])
            
            l_unity.set_data(t_arr, u)
            l_pred.set_data(t_arr, p)
            l_sent.set_data(t_arr, s)
            l_actual.set_data(t_arr, a)
            
            # Auto-scale axes
            ax.set_xlim(max(0, rel_t - GRAPH_WINDOW_SEC), rel_t + 0.5)
            
            if len(a) > 0:
                all_vals = np.concatenate([u, p, s, a])
                mn, mx = np.min(all_vals), np.max(all_vals)
                pad = max(2.0, (mx - mn) * 0.15)
                ax.set_ylim(mn - pad, mx + pad)
            
            all_lines.extend([l_unity, l_pred, l_sent, l_actual])
        
        # ✅ Log to CSV (every frame = 20Hz)
        self.session_logger.log_joints(
            unity=list(self.node.latest_target_joints),
            predicted=list(self.node.latest_predicted_joints),
            sent=list(self.last_sent_values),
            actual=list(self.node.latest_actual_joints),
        )
        # Flush every ~5s (100 frames @ 20Hz)
        self._log_flush_counter += 1
        if self._log_flush_counter >= 100:
            self.session_logger.flush()
            self._log_flush_counter = 0
        
        return all_lines

    def toggle_suction(self):
        self.suction_state = not self.suction_state
        self.node.request_suction(self.suction_state)
        self.lockout[VACUUM_DO_PORT] = time.time() + 2.0
        if self.suction_state:
            self.btn_suction.config(text="VACUUM (WAIT)", bg="orange", fg="white")
        else:
            self.btn_suction.config(text="OFF (WAIT)", bg="orange", fg="black")

    def toggle_light(self, name, port, color):
        self.light_states[name] = not self.light_states[name]
        status = self.light_states[name]
        self.node.request_light(port, status)
        self.lockout[port] = time.time() + 2.0
        self.btns_light[name].config(bg="orange", text=f"{name}...")

    def toggle_logging(self):
        self.is_logging = not self.is_logging
        if self.is_logging:
            self.btn_log.config(text="■ STOP", bg=BG_CARD, fg=COL_ERR)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.fn_target = f"teleop_target_{timestamp}.csv"
            self.fn_actual = f"teleop_actual_{timestamp}.csv"
            with open(self.fn_target, 'w', newline='') as f:
                csv.writer(f).writerow(["Time", "X", "Y", "Z", "Reach", "J1", "J2", "J3", "J4", "DiffTotal"])
            with open(self.fn_actual, 'w', newline='') as f:
                csv.writer(f).writerow(["Time", "X", "Y", "Z", "Reach", "J1", "J2", "J3", "J4"])
            self.log_start_time = time.time()
            self.node.get_logger().info(f"Started logging to {self.fn_target}")
        else:
            self.btn_log.config(text="● REC", bg=BG_CARD, fg=COL_IDLE)
            self.node.get_logger().info("Stopped logging.")

    def update_gui(self):
        # Get latest data
        tgt = self.node.latest_target_joints
        act = self.node.latest_actual_joints
        
        # --- Update Joint Data ---
        total_diff = 0.0
        for i in range(4):
            self.vars_target[i].set(f"{tgt[i]:.2f}")
            self.vars_actual[i].set(f"{act[i]:.2f}")
            diff = act[i] - tgt[i]
            self.vars_diff[i].set(f"{diff:+.2f}")
            total_diff += abs(diff)
            if abs(diff) > 2.0:
                self.lbls_diff[i].configure(fg=COL_ERR)
            elif abs(diff) > 0.5:
                self.lbls_diff[i].configure(fg=COL_WARN)
            else:
                self.lbls_diff[i].configure(fg=COL_GOOD)

        # --- Update System Info ---
        mode = self.node.latest_robot_mode
        error = self.node.latest_error_status
        mode_names = {1: "INIT", 4: "DISABLED", 5: "ENABLE", 6: "DRAG", 7: "RUN", 9: "ERROR", 11: "COLLISION"}
        self.var_mode.set(f"MODE: {mode_names.get(mode, str(mode))}")
        self.var_error.set(f"ERR: {error:02X}")
        self.lbl_error.configure(fg="red" if error != 0 or mode == 9 else "gray")
        
        # --- Update Button States (DO Status Sync) ---
        do_status = self.node.latest_do_status
        now = time.time()
        
        if now > self.lockout.get(VACUUM_DO_PORT, 0):
            actual_suction = bool((do_status >> (VACUUM_DO_PORT - 1)) & 1)
            self.suction_state = actual_suction
            if self.suction_state:
                self.btn_suction.config(text="VACUUM", bg=COLOR_VACUUM, fg="white")
            else:
                self.btn_suction.config(text="OFF", bg=COLOR_OFF, fg=FG_TEXT)
        
        for name, port, color in [("GREEN", GREEN_LIGHT_DO_PORT, COLOR_GREEN), ("YELLOW", YELLOW_LIGHT_DO_PORT, COLOR_YELLOW), ("RED", RED_LIGHT_DO_PORT, COLOR_RED)]:
            if now > self.lockout.get(port, 0):
                actual_light = bool((do_status >> (port - 1)) & 1)
                self.light_states[name] = actual_light
                bg_color = color if actual_light else COLOR_OFF
                fg_color = "white" if actual_light else FG_DIM
                self.btns_light[name].config(bg=bg_color, fg=fg_color, text=name)

        # --- Update Cartesian Data ---
        xyz_unity  = self.node.latest_unity_xyz[:3]       # FK of Unity input joints (orange)
        xyz_flange = self.node.latest_flange_actual[:3]    # FK of actual joints, no tool (blue)
        xyz_tcp    = self.node.latest_tool_actual[:3]       # firmware TCP with tool offset (green)
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{xyz_unity[i]:.1f}")
            self.vars_xyz_flange[i].set(f"{xyz_flange[i]:.1f}")
            self.vars_xyz_act[i].set(f"{xyz_tcp[i]:.1f}")
            # Tool offset = TCP - Flange (live, no preconfig needed)
            tool_delta = xyz_tcp[i] - xyz_flange[i]
            self.vars_xyz_tool[i].set(f"{tool_delta:+.1f}")

        # Tool index label
        tidx = self.node.latest_tool_index
        if tidx >= 0:
            self.var_tool_index.set(f"Tool {tidx}")
        else:
            self.var_tool_index.set("— (querying...)")

        # --- Execution Monitor ---
        status = self.monitor.update(total_diff)
        if self.monitor.state == "MOVING":
            self.var_status.set("● MOVING")
            self.lbl_status.configure(fg=COL_WARN)
            self.var_timer.set(f"{time.time() - self.monitor.start_time:.2f}s")
            self.lbl_timer.configure(fg=COL_WARN)
        elif self.monitor.state == "ARRIVED":
            self.var_status.set("✓ ARRIVED")
            self.lbl_status.configure(fg=COL_GOOD)
            self.var_timer.set(f"{self.monitor.last_duration:.2f}s")
            self.lbl_timer.configure(fg=COL_GOOD)
        else:
            self.var_status.set("IDLE")
            self.lbl_status.configure(fg=COL_IDLE)
        
        avg_t, min_t, max_t = self.monitor.get_stats()
        self.var_stats.set(f"Avg: {avg_t:.2f}s | Min: {min_t:.2f}s | Max: {max_t:.2f}s | Count: {len(self.monitor.durations)}")

        # --- Latency ---
        now = time.time()
        time_since_target = now - self.node.last_target_time
        time_since_actual = now - self.node.last_actual_time
        if self.node.last_target_time == 0:
            self.var_latency.set("Status: No Target Received")
        else:
            self.var_latency.set(f"Cmd Age: {time_since_target*1000:.0f}ms | Feed Age: {time_since_actual*1000:.0f}ms")
            
        self.var_do_hex.set(f"DO: 0x{self.node.latest_do_status:04X} | Bits: {bin(self.node.latest_do_status)}")
            
        # --- CSV Logging ---
        if self.is_logging:
            t = time.time() - self.log_start_time
            reach_tgt = math.sqrt(xyz_tgt[0]**2 + xyz_tgt[1]**2)
            with open(self.fn_target, 'a', newline='') as f:
                csv.writer(f).writerow([f"{t:.3f}", f"{xyz_tgt[0]:.3f}", f"{xyz_tgt[1]:.3f}", f"{xyz_tgt[2]:.3f}", f"{reach_tgt:.3f}", f"{tgt[0]:.3f}", f"{tgt[1]:.3f}", f"{tgt[2]:.3f}", f"{tgt[3]:.3f}", f"{total_diff:.3f}"])
            reach_act = math.sqrt(xyz_act[0]**2 + xyz_act[1]**2)
            with open(self.fn_actual, 'a', newline='') as f:
                csv.writer(f).writerow([f"{t:.3f}", f"{xyz_act[0]:.3f}", f"{xyz_act[1]:.3f}", f"{xyz_act[2]:.3f}", f"{reach_act:.3f}", f"{act[0]:.3f}", f"{act[1]:.3f}", f"{act[2]:.3f}", f"{act[3]:.3f}"])

        # Schedule next update at 20Hz
        self.root.after(50, self.update_gui)


def main():
    rclpy.init()
    node = JointMonitorNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    root = tk.Tk()
    root.geometry("1350x820")
    gui = MonitorGUI(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            gui.session_logger.close()
            print("[SessionLogger] Log closed.")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
