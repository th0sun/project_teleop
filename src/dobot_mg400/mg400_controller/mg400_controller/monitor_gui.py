#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🍎 MG400 Monitor GUI — Apple VisionOS Glassmorphism
Advanced Spatial Glassmorphism UI with PIL-rendered frosted glass cards,
mesh gradient background, and premium data visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32MultiArray, Int64, Int32
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

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator

# PIL for glassmorphism rendering
from PIL import Image, ImageDraw, ImageFilter, ImageTk, ImageFont

from mg400_controller.common.config.motion_config import (
    UNITY_TOPIC, SUCTION_TOPIC, LIGHT_TOPIC,
    VACUUM_DO_PORT, BLOW_DO_PORT,
    GREEN_LIGHT_DO_PORT, YELLOW_LIGHT_DO_PORT, RED_LIGHT_DO_PORT,
    DO_STATUS_TOPIC, ROBOT_MODE_TOPIC, ERROR_STATUS_TOPIC
)

# ── Topic Configuration ──────────────────────────────────
ACTUAL_TOPIC_NAME = "/joint_states"
TARGET_TOPIC_NAME = UNITY_TOPIC
TOOL_ACTUAL_TOPIC = "/mg400/tool_vector_actual"
TOOL_TARGET_TOPIC = "/mg400/tool_vector_target"
PREDICTED_TOPIC   = "/teleop/predicted_target"
SENT_CMD_TOPIC    = "/teleop/sent_command"

# ── 🎨 Apple Light Glassmorphism Palette ─────────────────
BG_BASE         = "#F2F2F7"    # System Gray 6 (light)
CARD_FILL       = "#FFFFFFB3"  # white 70% opacity (approximated)
CARD_FILL_INNER = "#FFFFFF80"  # white 50% inner elements
CARD_FILL_SOLID = "#F9F9FB"    # solid fallback for tk frames
CARD_INNER_SOLID= "#F0F0F5"    # solid inner element bg
CARD_HOVER      = "#E8E8ED"    # hover state

ISLAND_BG       = "#1C1C1ED9"  # dark 85% for Dynamic Island
ISLAND_SOLID    = "#2C2C2E"    # solid fallback

# Text colors (never pure black)
TEXT_PRIMARY     = "#1D1D1F"    # rgba(0,0,0,0.85)
TEXT_SECONDARY   = "#86868B"    # rgba(0,0,0,0.45)
TEXT_TERTIARY    = "#AEAEB2"    # rgba(0,0,0,0.3)
TEXT_ON_DARK     = "#F5F5F7"    # text on dark backgrounds

# Apple System Colors (vibrant)
SYS_BLUE        = "#007AFF"
SYS_GREEN       = "#34C759"
SYS_ORANGE      = "#FF9500"
SYS_RED         = "#FF3B30"
SYS_PURPLE      = "#AF52DE"
SYS_TEAL        = "#5AC8FA"
SYS_INDIGO      = "#5856D6"
SYS_PINK        = "#FF2D55"
SYS_MINT        = "#00C7BE"

# Gradient blob colors (for mesh gradient)
BLOB_BLUE       = "#B6D0FF"
BLOB_PURPLE     = "#D4B5FF"
BLOB_PINK       = "#FFB5C8"
BLOB_MINT       = "#A8F0E6"

# Border / Shadow
BORDER_LIGHT    = "#FFFFFF"
BORDER_DIM      = "#E5E5EA"
SHADOW_COLOR    = "#00000018"

# Fonts — Apple system stack
FONT_TITLE      = (".AppleSystemUIFont", 22, "bold")
FONT_SECTION    = (".AppleSystemUIFont", 11, "bold")
FONT_LABEL      = (".AppleSystemUIFont", 11)
FONT_LABEL_SM   = (".AppleSystemUIFont", 10)
FONT_VALUE      = ("Menlo", 13, "bold")
FONT_VALUE_SM   = ("Menlo", 11)
FONT_VALUE_LG   = ("Menlo", 22, "bold")
FONT_PILL       = (".AppleSystemUIFont", 10, "bold")
FONT_TINY       = ("Menlo", 9)

# Graph
GRAPH_WINDOW_SEC = 10.0
GRAPH_UPDATE_HZ  = 20
START_THRESHOLD  = 2.0
STOP_THRESHOLD   = 0.5

# Window
WIN_W, WIN_H     = 1200, 900


# ═════════════════════════════════════════════════════════
#  PIL GLASS RENDERER
# ═════════════════════════════════════════════════════════
class GlassRenderer:
    """Pre-renders glassmorphism assets using Pillow."""

    @staticmethod
    def mesh_gradient(w, h):
        """Create a soft mesh gradient background image."""
        base = Image.new("RGBA", (w, h), (242, 242, 247, 255))  # #F2F2F7
        overlay = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)

        blobs = [
            # (center_x%, center_y%, radius%, color_rgba)
            (0.15, 0.20, 0.40, (182, 208, 255, 70)),   # blue
            (0.80, 0.10, 0.35, (212, 181, 255, 65)),   # purple
            (0.65, 0.75, 0.38, (255, 181, 200, 55)),   # pink
            (0.20, 0.80, 0.30, (168, 240, 230, 60)),   # mint
            (0.50, 0.45, 0.25, (212, 181, 255, 40)),   # purple center
        ]
        for cx, cy, r, rgba in blobs:
            x, y, rad = int(cx * w), int(cy * h), int(r * max(w, h))
            draw.ellipse([x - rad, y - rad, x + rad, y + rad], fill=rgba)

        overlay = overlay.filter(ImageFilter.GaussianBlur(radius=90))
        result = Image.alpha_composite(base, overlay)
        return result.convert("RGB")

    @staticmethod
    def glass_card(w, h, radius=24, top_highlight=True):
        """Render a frosted glass card with rounded corners."""
        # Shadow layer
        shadow = Image.new("RGBA", (w + 20, h + 20), (0, 0, 0, 0))
        sd = ImageDraw.Draw(shadow)
        sd.rounded_rectangle([10, 12, w + 10, h + 12], radius=radius,
                             fill=(0, 0, 0, 18))
        shadow = shadow.filter(ImageFilter.GaussianBlur(radius=12))

        # Card layer
        card = Image.new("RGBA", (w + 20, h + 20), (0, 0, 0, 0))
        cd = ImageDraw.Draw(card)

        # Main fill — vertical gradient white 72% → white 45%
        for y_off in range(h):
            alpha = int(184 - (y_off / h) * 70)  # 184 → 114
            cd.line([(10, 10 + y_off), (w + 9, 10 + y_off)],
                    fill=(255, 255, 255, alpha))

        # Apply rounded mask
        mask = Image.new("L", (w + 20, h + 20), 0)
        md = ImageDraw.Draw(mask)
        md.rounded_rectangle([10, 10, w + 9, h + 9], radius=radius, fill=255)
        card.putalpha(mask)

        # Top highlight — bright white line at top edge
        if top_highlight:
            hl = ImageDraw.Draw(card)
            hl.rounded_rectangle([11, 10, w + 8, 12], radius=radius,
                                 fill=(255, 255, 255, 220))

        # Border — subtle white edge
        bd = ImageDraw.Draw(card)
        bd.rounded_rectangle([10, 10, w + 9, h + 9], radius=radius,
                             outline=(255, 255, 255, 180), width=1)

        # Composite: shadow + card
        result = Image.alpha_composite(shadow, card)
        return result

    @staticmethod
    def dynamic_island(w, h=44):
        """Render the Dynamic Island pill."""
        pad = 8
        img = Image.new("RGBA", (w + pad * 2, h + pad * 2), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        d.rounded_rectangle([pad, pad, w + pad, h + pad],
                            radius=h // 2, fill=(28, 28, 30, 220))
        # Subtle border
        d.rounded_rectangle([pad, pad, w + pad, h + pad],
                            radius=h // 2,
                            outline=(255, 255, 255, 30), width=1)
        return img


# ═════════════════════════════════════════════════════════
#  SESSION LOGGER (unchanged logic)
# ═════════════════════════════════════════════════════════
class SessionLogger:
    BASE_DIR = os.path.expanduser("~/project_teleop_ws/session_logs")

    def __init__(self):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.BASE_DIR, ts)
        os.makedirs(self.session_dir, exist_ok=True)

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


# ═════════════════════════════════════════════════════════
#  EXECUTION MONITOR (unchanged logic)
# ═════════════════════════════════════════════════════════
class ExecutionMonitor:
    def __init__(self):
        self.state = "IDLE"
        self.start_time = 0.0
        self.end_time = 0.0
        self.last_duration = 0.0
        self.durations = []

    def update(self, total_error):
        now = time.time()
        if self.state in ("IDLE", "ARRIVED"):
            if total_error > START_THRESHOLD:
                self.state = "MOVING"
                self.start_time = now
        elif self.state == "MOVING":
            if total_error < STOP_THRESHOLD:
                self.state = "ARRIVED"
                self.end_time = now
                self.last_duration = self.end_time - self.start_time
                self.durations.append(self.last_duration)
        return self.state

    def get_stats(self):
        if not self.durations:
            return 0.0, 0.0, 0.0
        return np.mean(self.durations), np.min(self.durations), np.max(self.durations)


# ═════════════════════════════════════════════════════════
#  ROS 2 NODE (unchanged logic)
# ═════════════════════════════════════════════════════════
class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('mg400_joint_monitor')
        self.pub_suction = self.create_publisher(Bool, SUCTION_TOPIC, 10)
        self.pub_light = self.create_publisher(Int32MultiArray, LIGHT_TOPIC, 10)

        self.latest_actual_joints    = [0.0]*4
        self.latest_target_joints    = [0.0]*4
        self.latest_predicted_joints = [0.0]*4
        self.latest_sent_joints      = [0.0]*4
        self.latest_tool_actual      = [0.0]*6
        self.latest_tool_target      = [0.0]*6
        self.latest_unity_xyz        = [0.0]*6
        self.latest_flange_actual    = [0.0]*6
        self.latest_tool_index       = -1
        self.latest_do_status        = 0
        self.latest_robot_mode       = 0
        self.latest_error_status     = 0
        self.last_target_time        = 0.0
        self.last_actual_time        = 0.0

        self.create_subscription(JointState,        ACTUAL_TOPIC_NAME, self.cb_act, 10)
        self.create_subscription(JointState,        TARGET_TOPIC_NAME, self.cb_tgt, 10)
        self.create_subscription(JointState,        PREDICTED_TOPIC,   self.cb_pred, 10)
        self.create_subscription(JointState,        SENT_CMD_TOPIC,    self.cb_sent, 10)
        self.create_subscription(Float64MultiArray, TOOL_ACTUAL_TOPIC, self.cb_tool, 10)
        self.create_subscription(Float64MultiArray, TOOL_TARGET_TOPIC, self.cb_tool_tgt, 10)
        self.create_subscription(Float64MultiArray, "/teleop/unity_xyz",    self.cb_uxyz, 10)
        self.create_subscription(Float64MultiArray, "/robot/flange_actual", self.cb_flange, 10)
        self.create_subscription(Int32,  "/robot/tool_index", self.cb_tidx, 10)
        self.create_subscription(Int64,  DO_STATUS_TOPIC,     self.cb_do, 10)
        self.create_subscription(Int32,  ROBOT_MODE_TOPIC,    self.cb_mode, 10)
        self.create_subscription(Int32,  ERROR_STATUS_TOPIC,  self.cb_err, 10)

    def request_suction(self, state):
        msg = Bool(); msg.data = state
        self.pub_suction.publish(msg)

    def request_light(self, port, state):
        msg = Int32MultiArray(); msg.data = [port, int(state)]
        self.pub_light.publish(msg)

    def cb_act(self, msg):
        if len(msg.position) >= 9:
            self.latest_actual_joints = list(np.degrees(
                [msg.position[0], msg.position[1], msg.position[3], msg.position[8]]))
            self.last_actual_time = time.time()
    def cb_tgt(self, msg):
        if len(msg.position) >= 4:
            self.latest_target_joints = list(np.degrees(msg.position[:4]))
            self.last_target_time = time.time()
    def cb_pred(self, msg):
        if len(msg.position) >= 4:
            self.latest_predicted_joints = list(np.degrees(msg.position[:4]))
    def cb_sent(self, msg):
        if len(msg.position) >= 4:
            self.latest_sent_joints = list(np.degrees(msg.position[:4]))
    def cb_tool(self, msg):
        if len(msg.data) >= 6: self.latest_tool_actual = list(msg.data)
    def cb_tool_tgt(self, msg):
        if len(msg.data) >= 6: self.latest_tool_target = list(msg.data)
    def cb_uxyz(self, msg):
        if len(msg.data) >= 6: self.latest_unity_xyz = list(msg.data)
    def cb_flange(self, msg):
        if len(msg.data) >= 6: self.latest_flange_actual = list(msg.data)
    def cb_tidx(self, msg): self.latest_tool_index = int(msg.data)
    def cb_do(self, msg): self.latest_do_status = int(msg.data)
    def cb_mode(self, msg): self.latest_robot_mode = int(msg.data)
    def cb_err(self, msg): self.latest_error_status = int(msg.data)


# ═════════════════════════════════════════════════════════
#  🍎 GLASSMORPHISM GUI
# ═════════════════════════════════════════════════════════
class MonitorGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.monitor = ExecutionMonitor()
        self.lockout = {}
        self._img_refs = []  # prevent GC of PhotoImages

        self.root.title("MG400 — VR Teleoperation Monitor")
        self.root.configure(bg=BG_BASE)
        self.root.geometry(f"{WIN_W}x{WIN_H}")

        # ── Render gradient background ──
        self._bg_img = GlassRenderer.mesh_gradient(WIN_W, WIN_H)
        self._bg_photo = ImageTk.PhotoImage(self._bg_img)

        self.canvas = tk.Canvas(root, width=WIN_W, height=WIN_H,
                                highlightthickness=0, bd=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.create_image(0, 0, anchor="nw", image=self._bg_photo)

        # ── Dynamic Island (top center) ──
        self._build_island()

        # ── Content Area ──
        content_frame = tk.Frame(self.canvas, bg="", bd=0,
                                 highlightthickness=0)
        content_frame.configure(bg=BG_BASE)  # fallback (hidden by gradient)
        self.canvas.create_window(0, 64, anchor="nw", window=content_frame,
                                  width=WIN_W, height=WIN_H - 64)
        # Make content frame transparent-looking
        content_frame.configure(bg="")
        try:
            content_frame.configure(bg=BG_BASE)
        except Exception:
            pass

        # ── Two-column layout ──
        left_col = tk.Frame(content_frame, bg=BG_BASE, width=440)
        left_col.pack(side=tk.LEFT, fill=tk.Y, padx=(20, 8), pady=6)
        left_col.pack_propagate(False)

        right_col = tk.Frame(content_frame, bg=BG_BASE)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                       padx=(8, 20), pady=6)

        # ── Build Cards ──
        self._build_xyz_card(left_col)
        self._build_joints_card(left_col)
        self._build_controls_card(left_col)
        self._build_metrics_card(left_col)
        self._build_do_card(left_col)

        # ── Graphs ──
        self._setup_graphs(right_col)

        # ── Start update loop ──
        self.update_gui()

    # ─────────────────────────────────────────────────────
    #  GLASS CARD HELPER
    # ─────────────────────────────────────────────────────
    def _glass_frame(self, parent, title=None):
        """Create a card frame that approximates frosted glass look."""
        outer = tk.Frame(parent, bg=BG_BASE)
        outer.pack(fill=tk.X, pady=(0, 10))

        if title:
            tk.Label(outer, text=title.upper(), font=FONT_SECTION,
                     fg=TEXT_SECONDARY, bg=BG_BASE, anchor="w").pack(
                fill=tk.X, padx=4, pady=(0, 4))

        # Card body — solid approximation of frosted glass
        card = tk.Frame(outer, bg=CARD_FILL_SOLID,
                        highlightbackground=BORDER_DIM,
                        highlightthickness=1, bd=0)
        card.pack(fill=tk.BOTH, expand=True)

        # Top highlight border (bright white line)
        highlight = tk.Frame(card, bg=BORDER_LIGHT, height=1)
        highlight.pack(fill=tk.X, side=tk.TOP)

        inner = tk.Frame(card, bg=CARD_FILL_SOLID)
        inner.pack(fill=tk.BOTH, expand=True, padx=16, pady=14)

        return inner

    # ─────────────────────────────────────────────────────
    #  DYNAMIC ISLAND
    # ─────────────────────────────────────────────────────
    def _build_island(self):
        """Dark pill header at top center."""
        island_w = 520
        island = tk.Frame(self.canvas, bg=ISLAND_SOLID, bd=0,
                          highlightthickness=0)

        # Position at top center
        self.canvas.create_window(WIN_W // 2, 12, anchor="n",
                                  window=island, width=island_w, height=42)

        # Round corners via a Canvas trick — draw rounded bg
        inner = tk.Frame(island, bg=ISLAND_SOLID)
        inner.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        # Title
        tk.Label(inner, text="VR Teleoperation", font=FONT_TITLE,
                 fg=TEXT_ON_DARK, bg=ISLAND_SOLID).pack(side=tk.LEFT, padx=12)

        # Status pills on the right
        pill_frame = tk.Frame(inner, bg=ISLAND_SOLID)
        pill_frame.pack(side=tk.RIGHT, padx=8)

        self.lbl_mode = tk.Label(pill_frame, text="INIT", font=FONT_PILL,
                                 fg=TEXT_ON_DARK, bg=SYS_INDIGO,
                                 padx=10, pady=2)
        self.lbl_mode.pack(side=tk.LEFT, padx=3)

        self.lbl_err = tk.Label(pill_frame, text="OK", font=FONT_PILL,
                                fg="#FFFFFF", bg=SYS_GREEN, padx=10, pady=2)
        self.lbl_err.pack(side=tk.LEFT, padx=3)

        # Manual log button
        self.is_logging = False
        self.log_start_time = 0.0
        self.btn_log = tk.Label(pill_frame, text="● REC", font=FONT_PILL,
                                fg=TEXT_SECONDARY, bg="#3A3A3C",
                                padx=10, pady=2, cursor="hand2")
        self.btn_log.pack(side=tk.LEFT, padx=3)
        self.btn_log.bind("<Button-1>", lambda e: self.toggle_logging())

        # Latency display
        self.var_latency = tk.StringVar(value="")
        tk.Label(inner, textvariable=self.var_latency, font=FONT_LABEL_SM,
                 fg=TEXT_SECONDARY, bg=ISLAND_SOLID).pack(side=tk.RIGHT, padx=6)

    # ─────────────────────────────────────────────────────
    #  XYZ CARD
    # ─────────────────────────────────────────────────────
    def _build_xyz_card(self, parent):
        card = self._glass_frame(parent, "End Effector — XYZ (mm)")

        self.vars_xyz_tgt = []
        self.vars_xyz_flange = []
        self.vars_xyz_act = []
        self.vars_xyz_tool = []

        # Header
        headers = [("", TEXT_PRIMARY, 3), ("Unity", SYS_ORANGE, 7),
                   ("Flange", SYS_INDIGO, 7), ("TCP", SYS_GREEN, 7),
                   ("ToolΔ", TEXT_TERTIARY, 6)]
        hdr = tk.Frame(card, bg=CARD_FILL_SOLID)
        hdr.pack(fill=tk.X, pady=(0, 6))
        for txt, col, w in headers:
            tk.Label(hdr, text=txt, font=FONT_LABEL_SM, fg=col,
                     bg=CARD_FILL_SOLID, width=w, anchor="w").pack(
                side=tk.LEFT, padx=(0, 2))

        for axis in ["X", "Y", "Z"]:
            row = tk.Frame(card, bg=CARD_INNER_SOLID)
            row.pack(fill=tk.X, pady=1)
            # Hover effect
            row.bind("<Enter>", lambda e, r=row: r.configure(bg=CARD_HOVER))
            row.bind("<Leave>", lambda e, r=row: r.configure(bg=CARD_INNER_SOLID))

            tk.Label(row, text=axis, font=FONT_VALUE_SM, fg=TEXT_SECONDARY,
                     bg=CARD_INNER_SOLID, width=3, anchor="w").pack(
                side=tk.LEFT, padx=(4, 0))

            vt = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=vt, font=FONT_VALUE_SM,
                     fg=TEXT_PRIMARY, bg=CARD_INNER_SOLID, width=7,
                     anchor="e").pack(side=tk.LEFT, padx=2)
            self.vars_xyz_tgt.append(vt)

            vf = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=vf, font=FONT_VALUE_SM,
                     fg=SYS_INDIGO, bg=CARD_INNER_SOLID, width=7,
                     anchor="e").pack(side=tk.LEFT, padx=2)
            self.vars_xyz_flange.append(vf)

            va = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=va, font=FONT_VALUE_SM,
                     fg=TEXT_PRIMARY, bg=CARD_INNER_SOLID, width=7,
                     anchor="e").pack(side=tk.LEFT, padx=2)
            self.vars_xyz_act.append(va)

            vd = tk.StringVar(value="0.0")
            tk.Label(row, textvariable=vd, font=FONT_VALUE_SM,
                     fg=TEXT_TERTIARY, bg=CARD_INNER_SOLID, width=6,
                     anchor="e").pack(side=tk.LEFT, padx=2)
            self.vars_xyz_tool.append(vd)

        # Tool Index row
        ti_row = tk.Frame(card, bg=CARD_FILL_SOLID)
        ti_row.pack(fill=tk.X, pady=(8, 0))
        tk.Label(ti_row, text="Active Tool", font=FONT_LABEL_SM,
                 fg=TEXT_SECONDARY, bg=CARD_FILL_SOLID).pack(side=tk.LEFT)
        self.var_tool_index = tk.StringVar(value="— querying…")
        tk.Label(ti_row, textvariable=self.var_tool_index, font=FONT_VALUE_SM,
                 fg=TEXT_PRIMARY, bg=CARD_FILL_SOLID).pack(side=tk.RIGHT)

    # ─────────────────────────────────────────────────────
    #  JOINTS CARD
    # ─────────────────────────────────────────────────────
    def _build_joints_card(self, parent):
        card = self._glass_frame(parent, "Joint Angles (°)")

        self.vars_jtgt = []
        self.vars_jact = []
        self.vars_jdiff = []
        self.lbls_jdiff = []

        hdr = tk.Frame(card, bg=CARD_FILL_SOLID)
        hdr.pack(fill=tk.X, pady=(0, 6))
        for txt, col, w in [("", TEXT_PRIMARY, 3), ("Target", SYS_ORANGE, 8),
                             ("Actual", SYS_GREEN, 8), ("Diff", TEXT_TERTIARY, 7)]:
            tk.Label(hdr, text=txt, font=FONT_LABEL_SM, fg=col,
                     bg=CARD_FILL_SOLID, width=w, anchor="w").pack(
                side=tk.LEFT, padx=(0, 2))

        for i in range(4):
            row = tk.Frame(card, bg=CARD_INNER_SOLID)
            row.pack(fill=tk.X, pady=1)
            row.bind("<Enter>", lambda e, r=row: r.configure(bg=CARD_HOVER))
            row.bind("<Leave>", lambda e, r=row: r.configure(bg=CARD_INNER_SOLID))

            tk.Label(row, text=f"J{i+1}", font=FONT_VALUE_SM,
                     fg=TEXT_SECONDARY, bg=CARD_INNER_SOLID, width=3,
                     anchor="w").pack(side=tk.LEFT, padx=(4, 0))

            vt = tk.StringVar(value="0.00")
            tk.Label(row, textvariable=vt, font=FONT_VALUE_SM,
                     fg=TEXT_PRIMARY, bg=CARD_INNER_SOLID, width=8,
                     anchor="e").pack(side=tk.LEFT, padx=2)
            self.vars_jtgt.append(vt)

            va = tk.StringVar(value="0.00")
            tk.Label(row, textvariable=va, font=FONT_VALUE_SM,
                     fg=TEXT_PRIMARY, bg=CARD_INNER_SOLID, width=8,
                     anchor="e").pack(side=tk.LEFT, padx=2)
            self.vars_jact.append(va)

            vd = tk.StringVar(value="0.00")
            lbl = tk.Label(row, textvariable=vd, font=FONT_VALUE_SM,
                           fg=TEXT_TERTIARY, bg=CARD_INNER_SOLID, width=7,
                           anchor="e")
            lbl.pack(side=tk.LEFT, padx=2)
            self.vars_jdiff.append(vd)
            self.lbls_jdiff.append(lbl)

    # ─────────────────────────────────────────────────────
    #  I/O CONTROLS CARD
    # ─────────────────────────────────────────────────────
    def _build_controls_card(self, parent):
        card = self._glass_frame(parent, "I/O Control")

        self.suction_state = False
        self.btn_suction = tk.Button(
            card, text="SUCTION  OFF", font=FONT_SECTION,
            bg=CARD_INNER_SOLID, fg=TEXT_PRIMARY, relief="flat",
            activebackground=CARD_HOVER, activeforeground=TEXT_PRIMARY,
            command=self.toggle_suction, highlightbackground=CARD_FILL_SOLID,
            cursor="hand2")
        self.btn_suction.pack(fill=tk.X, pady=(0, 8), ipady=6)

        lc = tk.Frame(card, bg=CARD_FILL_SOLID)
        lc.pack(fill=tk.X)
        self.light_states = {"G": False, "Y": False, "R": False}
        self.btns_light = {}

        for name, port, col in [
            ("G", GREEN_LIGHT_DO_PORT, SYS_GREEN),
            ("Y", YELLOW_LIGHT_DO_PORT, SYS_ORANGE),
            ("R", RED_LIGHT_DO_PORT, SYS_RED)
        ]:
            btn = tk.Button(
                lc, text=name, font=FONT_SECTION,
                bg=CARD_INNER_SOLID, fg=TEXT_PRIMARY, relief="flat",
                activebackground=CARD_HOVER,
                command=lambda n=name, p=port, c=col: self.toggle_light(n,p,c),
                highlightbackground=CARD_FILL_SOLID, cursor="hand2")
            btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2, ipady=6)
            self.btns_light[name] = {"btn": btn, "on_col": col}

    # ─────────────────────────────────────────────────────
    #  EXECUTION METRICS CARD
    # ─────────────────────────────────────────────────────
    def _build_metrics_card(self, parent):
        card = self._glass_frame(parent, "Execution")

        row1 = tk.Frame(card, bg=CARD_FILL_SOLID)
        row1.pack(fill=tk.X)

        self.var_mov_stat = tk.StringVar(value="IDLE")
        self.lbl_mov_stat = tk.Label(row1, textvariable=self.var_mov_stat,
                                     font=FONT_SECTION, fg=TEXT_TERTIARY,
                                     bg=CARD_FILL_SOLID)
        self.lbl_mov_stat.pack(side=tk.LEFT)

        self.var_timer = tk.StringVar(value="0.00s")
        tk.Label(row1, textvariable=self.var_timer, font=FONT_VALUE_LG,
                 fg=TEXT_PRIMARY, bg=CARD_FILL_SOLID).pack(side=tk.RIGHT)

        self.var_stats = tk.StringVar(
            value="Avg: 0.00s  Min: 0.00s  Max: 0.00s  N: 0")
        tk.Label(card, textvariable=self.var_stats, font=FONT_TINY,
                 fg=TEXT_TERTIARY, bg=CARD_FILL_SOLID).pack(
            anchor=tk.E, pady=(4, 0))

    # ─────────────────────────────────────────────────────
    #  DO HEX CARD
    # ─────────────────────────────────────────────────────
    def _build_do_card(self, parent):
        card = self._glass_frame(parent, "Digital Output")
        self.var_do_hex = tk.StringVar(value="DO: 0x0000  Bits: 0b0")
        tk.Label(card, textvariable=self.var_do_hex, font=FONT_TINY,
                 fg=TEXT_TERTIARY, bg=CARD_FILL_SOLID).pack(anchor=tk.W)

    # ─────────────────────────────────────────────────────
    #  TOGGLE ACTIONS
    # ─────────────────────────────────────────────────────
    def toggle_suction(self):
        self.suction_state = not self.suction_state
        self.node.request_suction(self.suction_state)
        self.lockout[VACUUM_DO_PORT] = time.time() + 2.0

    def toggle_light(self, name, port, color):
        self.light_states[name] = not self.light_states[name]
        self.node.request_light(port, self.light_states[name])
        self.lockout[port] = time.time() + 2.0

    def toggle_logging(self):
        self.is_logging = not self.is_logging
        if self.is_logging:
            self.btn_log.config(text="● REC", fg=SYS_RED, bg="#3A3A3C")
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_dir = os.path.expanduser("~/project_teleop_ws/session_logs")
            os.makedirs(log_dir, exist_ok=True)
            self.fn_target = os.path.join(log_dir, f"manual_target_{ts}.csv")
            self.fn_actual = os.path.join(log_dir, f"manual_actual_{ts}.csv")
            with open(self.fn_target, 'w', newline='') as f:
                csv.writer(f).writerow(["Time", "X", "Y", "Z", "Reach",
                                        "J1", "J2", "J3", "J4", "DiffTotal"])
            with open(self.fn_actual, 'w', newline='') as f:
                csv.writer(f).writerow(["Time", "X", "Y", "Z", "Reach",
                                        "J1", "J2", "J3", "J4"])
            self.log_start_time = time.time()
            self.node.get_logger().info(f"Started manual logging to {log_dir}")
        else:
            self.btn_log.config(text="● REC", fg=TEXT_SECONDARY, bg="#3A3A3C")
            self.node.get_logger().info("Stopped manual logging.")

    # ─────────────────────────────────────────────────────
    #  📊 PREMIUM GRAPHS
    # ─────────────────────────────────────────────────────
    def _setup_graphs(self, parent):
        # Glass card wrapper
        card_outer = tk.Frame(parent, bg=CARD_FILL_SOLID,
                              highlightbackground=BORDER_DIM,
                              highlightthickness=1)
        card_outer.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        # Top highlight
        tk.Frame(card_outer, bg=BORDER_LIGHT, height=1).pack(
            fill=tk.X, side=tk.TOP)

        # Legend bar
        legend = tk.Frame(card_outer, bg=CARD_FILL_SOLID)
        legend.pack(fill=tk.X, padx=16, pady=(12, 4))
        tk.Label(legend, text="REAL-TIME TRACKING", font=FONT_SECTION,
                 fg=TEXT_SECONDARY, bg=CARD_FILL_SOLID).pack(side=tk.LEFT)

        for name, col in [("Target", SYS_ORANGE), ("Predicted", SYS_PURPLE),
                          ("Sent", SYS_BLUE), ("Actual", SYS_GREEN)]:
            tk.Label(legend, text=f"━ {name}", font=FONT_LABEL_SM,
                     fg=col, bg=CARD_FILL_SOLID).pack(side=tk.LEFT, padx=8)

        # Matplotlib figure — light theme
        self.fig = Figure(figsize=(7, 7), dpi=100, facecolor=CARD_FILL_SOLID)
        self.fig.subplots_adjust(hspace=0.35, left=0.10, right=0.97,
                                top=0.97, bottom=0.04)
        self.axes = []
        self.graph_lines = []

        max_pts = int(GRAPH_WINDOW_SEC * GRAPH_UPDATE_HZ * 1.5)
        self.t_buff = deque(maxlen=max_pts)
        self.u_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.p_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.s_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.a_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.lsv = [0.0] * 4
        self.t0 = time.time()

        # Auto-start session logger
        self.session_logger = SessionLogger()
        self._log_flush_counter = 0

        for i in range(4):
            ax = self.fig.add_subplot(4, 1, i + 1)
            ax.set_facecolor("#FFFFFF")

            # Minimal spine styling
            for spine in ax.spines.values():
                spine.set_visible(False)

            # Ultra-faint grid
            ax.grid(True, color="#E5E5EA", alpha=0.5, linewidth=0.5,
                    linestyle="-")
            ax.tick_params(axis='both', colors=TEXT_TERTIARY, labelsize=8,
                          length=0)
            ax.yaxis.set_major_locator(MaxNLocator(nbins=4))

            ax.set_ylabel(f"J{i+1}  (°)", color=TEXT_SECONDARY, fontsize=10,
                          fontweight="medium", labelpad=8)

            # Lines — thick, clear, easy to read
            lu, = ax.plot([], [], color=SYS_ORANGE, lw=1.8, alpha=0.65,
                          label="Target")
            lp, = ax.plot([], [], color=SYS_PURPLE, lw=1.2, alpha=0.6,
                          label="Predicted")
            ls, = ax.plot([], [], color=SYS_BLUE, lw=1.8,
                          drawstyle="steps-post", alpha=0.7, label="Sent")
            la, = ax.plot([], [], color=SYS_GREEN, lw=2.5, alpha=0.95,
                          label="Actual")

            # Gradient fill under Actual line (subtle)
            self._fill_refs = []

            self.axes.append(ax)
            self.graph_lines.append((lu, lp, ls, la))

        canvas = FigureCanvasTkAgg(self.fig, master=card_outer)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        canvas.get_tk_widget().configure(highlightthickness=0, bd=0)

        self.ani = animation.FuncAnimation(
            self.fig, self._update_g,
            interval=1000 // GRAPH_UPDATE_HZ, blit=True)

    def _update_g(self, frame):
        now = time.time()
        rel_t = now - self.t0
        self.t_buff.append(rel_t)

        for i in range(4):
            self.u_buff[i].append(self.node.latest_target_joints[i])
            self.p_buff[i].append(self.node.latest_predicted_joints[i])
            if self.node.latest_sent_joints[i] != self.lsv[i]:
                self.lsv[i] = self.node.latest_sent_joints[i]
            self.s_buff[i].append(self.lsv[i])
            self.a_buff[i].append(self.node.latest_actual_joints[i])

        t_arr = np.array(self.t_buff)
        all_l = []
        for i in range(4):
            u, p, s, a = self.graph_lines[i]
            ax = self.axes[i]

            u.set_data(t_arr, self.u_buff[i])
            p.set_data(t_arr, self.p_buff[i])
            s.set_data(t_arr, self.s_buff[i])
            a.set_data(t_arr, self.a_buff[i])

            ax.set_xlim(max(0, rel_t - GRAPH_WINDOW_SEC), rel_t + 0.2)

            if len(self.a_buff[i]) > 0:
                vals = np.concatenate([self.u_buff[i], self.a_buff[i]])
                mn, mx = np.min(vals), np.max(vals)
                pad = max(2.0, (mx - mn) * 0.2)
                ax.set_ylim(mn - pad, mx + pad)

            all_l.extend([u, p, s, a])

        # Session CSV logging
        self.session_logger.log_joints(
            unity=list(self.node.latest_target_joints),
            predicted=list(self.node.latest_predicted_joints),
            sent=list(self.lsv),
            actual=list(self.node.latest_actual_joints),
        )
        self._log_flush_counter += 1
        if self._log_flush_counter >= 100:
            self.session_logger.flush()
            self._log_flush_counter = 0

        return all_l

    # ─────────────────────────────────────────────────────
    #  🔄 DATA UPDATE LOOP (20 Hz)
    # ─────────────────────────────────────────────────────
    def update_gui(self):
        tgt = self.node.latest_target_joints
        act = self.node.latest_actual_joints

        # ── Joint data ──
        tot_d = 0.0
        for i in range(4):
            self.vars_jtgt[i].set(f"{tgt[i]:7.2f}°")
            self.vars_jact[i].set(f"{act[i]:7.2f}°")
            d = act[i] - tgt[i]
            self.vars_jdiff[i].set(f"{d:+6.2f}°")
            tot_d += abs(d)
            if abs(d) > 2.0:
                self.lbls_jdiff[i].config(fg=SYS_RED)
            elif abs(d) > 0.5:
                self.lbls_jdiff[i].config(fg=SYS_ORANGE)
            else:
                self.lbls_jdiff[i].config(fg=TEXT_TERTIARY)

        # ── Cartesian data ──
        xyz_unity  = self.node.latest_unity_xyz[:3]
        xyz_flange = self.node.latest_flange_actual[:3]
        xyz_tcp    = self.node.latest_tool_actual[:3]
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{xyz_unity[i]:7.1f}")
            self.vars_xyz_flange[i].set(f"{xyz_flange[i]:7.1f}")
            self.vars_xyz_act[i].set(f"{xyz_tcp[i]:7.1f}")
            tool_delta = xyz_tcp[i] - xyz_flange[i]
            self.vars_xyz_tool[i].set(f"{tool_delta:+6.1f}")

        # Tool index
        tidx = self.node.latest_tool_index
        self.var_tool_index.set(f"Tool {tidx}" if tidx >= 0
                                else "— querying…")

        # XYZ session log
        self.session_logger.log_xyz(target=xyz_unity[:3], actual=xyz_tcp[:3])

        # ── Execution metrics ──
        st = self.monitor.update(tot_d)
        if st == "MOVING":
            self.var_mov_stat.set("● MOVING")
            self.lbl_mov_stat.config(fg=SYS_ORANGE)
            self.var_timer.set(f"{time.time()-self.monitor.start_time:.2f}s")
        elif st == "ARRIVED":
            self.var_mov_stat.set("✓ ARRIVED")
            self.lbl_mov_stat.config(fg=SYS_GREEN)
            self.var_timer.set(f"{self.monitor.last_duration:.2f}s")
        else:
            self.var_mov_stat.set("IDLE")
            self.lbl_mov_stat.config(fg=TEXT_TERTIARY)
            self.var_timer.set("0.00s")

        avg_t, min_t, max_t = self.monitor.get_stats()
        self.var_stats.set(
            f"Avg: {avg_t:.2f}s  Min: {min_t:.2f}s  "
            f"Max: {max_t:.2f}s  N: {len(self.monitor.durations)}")

        # ── Status pills ──
        mode = self.node.latest_robot_mode
        err = self.node.latest_error_status
        m_names = {1: "INIT", 4: "DISABLED", 5: "ENABLE", 6: "DRAG",
                   7: "RUN", 9: "ERROR", 11: "COLLIDE"}
        m_str = m_names.get(mode, str(mode))
        self.lbl_mode.config(text=m_str,
                             bg=SYS_INDIGO if mode == 5 else "#636366")
        self.lbl_err.config(text=f"ERR {err:02X}",
                            bg=SYS_RED if err != 0 else SYS_GREEN,
                            fg="#FFFFFF")

        # ── Latency ──
        tm_t = time.time() - self.node.last_target_time
        tm_a = time.time() - self.node.last_actual_time
        self.var_latency.set(f"Cmd {tm_t*1000:.0f}ms · Fbk {tm_a*1000:.0f}ms")

        # ── DO status ──
        do_s = self.node.latest_do_status
        self.var_do_hex.set(f"DO: 0x{do_s:04X}  Bits: {bin(do_s)}")

        now = time.time()
        if now > self.lockout.get(VACUUM_DO_PORT, 0):
            act_s = bool((do_s >> (VACUUM_DO_PORT - 1)) & 1)
            self.suction_state = act_s
            self.btn_suction.config(
                text="SUCTION  ON" if act_s else "SUCTION  OFF",
                fg="#FFFFFF" if act_s else TEXT_PRIMARY,
                bg=SYS_TEAL if act_s else CARD_INNER_SOLID)

        for name, port, col in [("G", GREEN_LIGHT_DO_PORT, SYS_GREEN),
                                ("Y", YELLOW_LIGHT_DO_PORT, SYS_ORANGE),
                                ("R", RED_LIGHT_DO_PORT, SYS_RED)]:
            if now > self.lockout.get(port, 0):
                act_l = bool((do_s >> (port - 1)) & 1)
                self.light_states[name] = act_l
                self.btns_light[name]["btn"].config(
                    bg=col if act_l else CARD_INNER_SOLID,
                    fg="#FFFFFF" if act_l else TEXT_PRIMARY)

        # ── Manual CSV ──
        if self.is_logging:
            t = time.time() - self.log_start_time
            reach_tgt = math.sqrt(xyz_unity[0]**2 + xyz_unity[1]**2)
            with open(self.fn_target, 'a', newline='') as f:
                csv.writer(f).writerow([
                    f"{t:.3f}", f"{xyz_unity[0]:.3f}", f"{xyz_unity[1]:.3f}",
                    f"{xyz_unity[2]:.3f}", f"{reach_tgt:.3f}",
                    f"{tgt[0]:.3f}", f"{tgt[1]:.3f}", f"{tgt[2]:.3f}",
                    f"{tgt[3]:.3f}", f"{tot_d:.3f}"])
            reach_act = math.sqrt(xyz_tcp[0]**2 + xyz_tcp[1]**2)
            with open(self.fn_actual, 'a', newline='') as f:
                csv.writer(f).writerow([
                    f"{t:.3f}", f"{xyz_tcp[0]:.3f}", f"{xyz_tcp[1]:.3f}",
                    f"{xyz_tcp[2]:.3f}", f"{reach_act:.3f}",
                    f"{act[0]:.3f}", f"{act[1]:.3f}", f"{act[2]:.3f}",
                    f"{act[3]:.3f}"])

        self.root.after(50, self.update_gui)


# ═════════════════════════════════════════════════════════
#  MAIN
# ═════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = JointMonitorNode()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    root = tk.Tk()
    gui = MonitorGUI(root, node)

    def on_close():
        try:
            gui.session_logger.close()
            print("[SessionLogger] Log closed.")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()
        sys.exit(0)


if __name__ == '__main__':
    main()
