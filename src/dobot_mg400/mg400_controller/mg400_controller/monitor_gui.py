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

# Colors for Lights
COLOR_OFF = "#d0d0d0"
COLOR_GREEN = "#2ecc71"
COLOR_YELLOW = "#f1c40f"
COLOR_RED = "#e74c3c"
COLOR_VACUUM = "#3498db"

# Graph colors
COL_UNITY = "#f39c12"      # Orange
COL_PREDICTED = "#9b59b6"  # Purple
COL_SENT = "#27ae60"       # Green
COL_ACTUAL = "#2980b9"     # Blue

# Fonts
FONT_HEADER = ("Helvetica", 14, "bold")
FONT_LABEL = ("Helvetica", 12)
FONT_VALUE = ("Helvetica", 12, "bold")
FONT_LATENCY = ("Helvetica", 10)
FONT_BIG_VALUE = ("Helvetica", 24, "bold")
FONT_STATS = ("Helvetica", 11)

# Graph config
GRAPH_WINDOW_SEC = 10.0   # Rolling window (seconds)
GRAPH_UPDATE_HZ = 20      # Update rate

# Motion Detection Thresholds
START_THRESHOLD = 2.0  # degrees (Start timer if error > this)
STOP_THRESHOLD = 0.5   # degrees (Stop timer if error < this)

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
        
        # 🛡️ Sync Lockout
        self.lockout = {}
        
        self.root.title("MG400 Extended Monitor")
        self.root.configure(bg="#1a1a2e")

        # ===== MAIN LAYOUT =====
        # Left panel: existing controls + tables
        # Right panel: real-time graphs
        outer = tk.Frame(root, bg="#1a1a2e")
        outer.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(outer, bg="#f0f0f0", width=600)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH)
        left_frame.pack_propagate(False)

        right_frame = tk.Frame(outer, bg="#1a1a2e")
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ===== LEFT PANEL (existing UI) =====
        main_frame = ttk.Frame(left_frame, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        title_frame = ttk.Frame(main_frame)
        title_frame.pack(fill=tk.X, pady=5)
        ttk.Label(title_frame, text="Real-time Monitor & Metrics", font=FONT_HEADER).pack(side=tk.LEFT)
        
        # Logging Button
        self.is_logging = False
        self.log_start_time = 0.0
        self.btn_log = ttk.Button(title_frame, text="▶ Start Logging", command=self.toggle_logging)
        self.btn_log.pack(side=tk.RIGHT)

        # --- JOINT TABLE ---
        header_frame = ttk.Frame(main_frame)
        header_frame.pack(fill=tk.X, pady=5)
        ttk.Label(header_frame, text="Joint", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        ttk.Label(header_frame, text="Target (°)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)
        ttk.Label(header_frame, text="Actual (°)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)
        ttk.Label(header_frame, text="Diff (°)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=5)

        self.vars_target = []
        self.vars_actual = []
        self.vars_diff = []
        self.lbls_diff = []
        
        joints = ["J1", "J2", "J3", "J4"]
        
        for i, name in enumerate(joints):
            frame = ttk.Frame(main_frame)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=name, font=FONT_LABEL, width=12).pack(side=tk.LEFT)
            
            v_tgt = tk.StringVar(value="0.00")
            ttk.Label(frame, textvariable=v_tgt, font=FONT_VALUE, foreground="darkgreen", width=12).pack(side=tk.LEFT)
            self.vars_target.append(v_tgt)

            v_act = tk.StringVar(value="0.00")
            ttk.Label(frame, textvariable=v_act, font=FONT_VALUE, foreground="blue", width=12).pack(side=tk.LEFT)
            self.vars_actual.append(v_act)

            v_diff = tk.StringVar(value="0.00")
            lbl_diff = ttk.Label(frame, textvariable=v_diff, font=FONT_VALUE, foreground="black", width=12)
            lbl_diff.pack(side=tk.LEFT)
            self.vars_diff.append(v_diff)
            self.lbls_diff.append(lbl_diff)

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=10)

        # --- CARTESIAN MONITOR (XYZ) ---
        ttk.Label(main_frame, text="Cartesian Coordinates (End Effector)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W)

        header_xyz = ttk.Frame(main_frame)
        header_xyz.pack(fill=tk.X, pady=2)
        ttk.Label(header_xyz, text="Axis", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Unity FK (mm)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Actual (mm)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Diff (mm)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)

        self.vars_xyz_tgt = []
        self.vars_xyz_act = []
        self.vars_xyz_diff = []
        self.lbls_xyz_diff = []

        for i, name in enumerate(["X", "Y", "Z"]):
            frame = ttk.Frame(main_frame)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=name, font=FONT_LABEL, width=12).pack(side=tk.LEFT)
            
            v_tgt = tk.StringVar(value="0.00")
            ttk.Label(frame, textvariable=v_tgt, font=FONT_VALUE, foreground="darkgreen", width=12).pack(side=tk.LEFT)
            self.vars_xyz_tgt.append(v_tgt)
            
            v_act = tk.StringVar(value="0.00")
            ttk.Label(frame, textvariable=v_act, font=FONT_VALUE, foreground="blue", width=12).pack(side=tk.LEFT)
            self.vars_xyz_act.append(v_act)
            
            v_diff = tk.StringVar(value="0.00")
            lbl_diff = ttk.Label(frame, textvariable=v_diff, font=FONT_VALUE, foreground="black", width=12)
            lbl_diff.pack(side=tk.LEFT)
            self.vars_xyz_diff.append(v_diff)
            self.lbls_xyz_diff.append(lbl_diff)

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=10)

        # --- 🎮 CONTROL PANEL ---
        control_frame = ttk.LabelFrame(main_frame, text="Robot Direct Control", padding="10")
        control_frame.pack(fill=tk.X, pady=5)

        suction_row = ttk.Frame(control_frame)
        suction_row.pack(fill=tk.X, pady=5)
        ttk.Label(suction_row, text="Suction:", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        self.suction_state = False
        self.btn_suction = tk.Button(suction_row, text="OFF", font=FONT_VALUE, width=10, bg=COLOR_OFF, command=self.toggle_suction)
        self.btn_suction.pack(side=tk.LEFT, padx=5)

        light_row = ttk.Frame(control_frame)
        light_row.pack(fill=tk.X, pady=10)
        ttk.Label(light_row, text="Lights:", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        
        self.light_states = { "GREEN": False, "YELLOW": False, "RED": False }
        self.btns_light = {}
        
        for name, port, color in [("GREEN", GREEN_LIGHT_DO_PORT, COLOR_GREEN), ("YELLOW", YELLOW_LIGHT_DO_PORT, COLOR_YELLOW), ("RED", RED_LIGHT_DO_PORT, COLOR_RED)]:
            btn = tk.Button(light_row, text=name, font=("Helvetica", 10, "bold"), width=8, bg=COLOR_OFF, 
                            command=lambda n=name, p=port, c=color: self.toggle_light(n, p, c))
            btn.pack(side=tk.LEFT, padx=2)
            self.btns_light[name] = btn

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=10)

        # --- EXECUTION METRICS ---
        metrics_frame = ttk.LabelFrame(main_frame, text="Execution Metrics", padding="10")
        metrics_frame.pack(fill=tk.X, pady=5)
        
        row1 = ttk.Frame(metrics_frame)
        row1.pack(fill=tk.X)
        self.var_status = tk.StringVar(value="IDLE")
        self.lbl_status = ttk.Label(row1, textvariable=self.var_status, font=("Helvetica", 12, "bold"), foreground="gray")
        self.lbl_status.pack(side=tk.LEFT)
        self.var_timer = tk.StringVar(value="0.00s")
        self.lbl_timer = ttk.Label(row1, textvariable=self.var_timer, font=FONT_BIG_VALUE, foreground="black")
        self.lbl_timer.pack(side=tk.RIGHT)
        
        row2 = ttk.Frame(metrics_frame)
        row2.pack(fill=tk.X, pady=5)
        self.var_stats = tk.StringVar(value="Avg: 0.00s | Min: 0.00s | Max: 0.00s")
        ttk.Label(row2, textvariable=self.var_stats, font=FONT_STATS).pack(anchor=tk.E)

        # --- STATUS BAR ---
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=10)
        
        self.var_latency = tk.StringVar(value="Waiting for data...")
        ttk.Label(status_frame, textvariable=self.var_latency, font=FONT_LATENCY).pack(side=tk.LEFT)

        self.var_mode = tk.StringVar(value="MODE: -")
        tk.Label(status_frame, textvariable=self.var_mode, font=("Arial", 10), bg="#f0f0f0").pack(side=tk.RIGHT, padx=10)
        
        self.var_error = tk.StringVar(value="ERR: 00")
        self.lbl_error = tk.Label(status_frame, textvariable=self.var_error, font=("Arial", 10, "bold"), bg="#f0f0f0", fg="red")
        self.lbl_error.pack(side=tk.RIGHT, padx=5)
        
        self.var_do_hex = tk.StringVar(value="DO: 0x0000")
        ttk.Label(status_frame, textvariable=self.var_do_hex, font=FONT_LATENCY, foreground="gray").pack(anchor=tk.W)

        # ===== RIGHT PANEL: REAL-TIME GRAPHS =====
        self._setup_graphs(right_frame)

        # Start Update Loop
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
                self.lbls_diff[i].configure(foreground="red")
            elif abs(diff) > 0.5:
                self.lbls_diff[i].configure(foreground="orange")
            else:
                self.lbls_diff[i].configure(foreground="green")

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
                self.btn_suction.config(text="OFF", bg=COLOR_OFF, fg="black")
        
        for name, port, color in [("GREEN", GREEN_LIGHT_DO_PORT, COLOR_GREEN), ("YELLOW", YELLOW_LIGHT_DO_PORT, COLOR_YELLOW), ("RED", RED_LIGHT_DO_PORT, COLOR_RED)]:
            if now > self.lockout.get(port, 0):
                actual_light = bool((do_status >> (port - 1)) & 1)
                self.light_states[name] = actual_light
                bg_color = color if actual_light else COLOR_OFF
                fg_color = "white" if actual_light else "black"
                self.btns_light[name].config(bg=bg_color, fg=fg_color, text=name)

        # --- Update Cartesian Data ---
        # Use Unity FK XYZ if available, fall back to firmware target
        xyz_tgt = self.node.latest_unity_xyz[:3] if any(v != 0 for v in self.node.latest_unity_xyz) \
                  else self.node.latest_tool_target[:3]
        xyz_act = self.node.latest_tool_actual[:3]
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{xyz_tgt[i]:.1f}")
            self.vars_xyz_act[i].set(f"{xyz_act[i]:.1f}")
            diff = xyz_act[i] - xyz_tgt[i]
            self.vars_xyz_diff[i].set(f"{diff:+.1f}")
            if abs(diff) > 10.0:
                self.lbls_xyz_diff[i].configure(foreground="red")
            elif abs(diff) > 2.0:
                self.lbls_xyz_diff[i].configure(foreground="orange")
            else:
                self.lbls_xyz_diff[i].configure(foreground="green")

        # --- Execution Monitor ---
        status = self.monitor.update(total_diff)
        if self.monitor.state == "MOVING":
            self.var_status.set("MOVING...")
            self.lbl_status.configure(foreground="red")
            self.var_timer.set(f"{time.time() - self.monitor.start_time:.2f}s")
            self.lbl_timer.configure(foreground="red")
        elif self.monitor.state == "ARRIVED":
            self.var_status.set("ARRIVED")
            self.lbl_status.configure(foreground="green")
            self.var_timer.set(f"{self.monitor.last_duration:.2f}s")
            self.lbl_timer.configure(foreground="green")
        else:
            self.var_status.set("IDLE")
            self.lbl_status.configure(foreground="gray")
        
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
