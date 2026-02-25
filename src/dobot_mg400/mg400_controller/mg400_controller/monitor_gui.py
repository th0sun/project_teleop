#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🍎 MG400 Monitor GUI - Apple UI Style 
Clean, Dark-Mode, Minimalist Robot Teleop Monitor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int32MultiArray, Int64, Int32
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
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

from mg400_controller.common.config.motion_config import (
    UNITY_TOPIC, SUCTION_TOPIC, LIGHT_TOPIC, 
    VACUUM_DO_PORT, BLOW_DO_PORT, 
    GREEN_LIGHT_DO_PORT, YELLOW_LIGHT_DO_PORT, RED_LIGHT_DO_PORT,
    DO_STATUS_TOPIC, ROBOT_MODE_TOPIC, ERROR_STATUS_TOPIC
)

# Configuration Topics
ACTUAL_TOPIC_NAME = "/joint_states"
TARGET_TOPIC_NAME = UNITY_TOPIC
TOOL_ACTUAL_TOPIC = "/mg400/tool_vector_actual"
TOOL_TARGET_TOPIC = "/mg400/tool_vector_target"
PREDICTED_TOPIC = "/teleop/predicted_target"
SENT_CMD_TOPIC = "/teleop/sent_command"

# --- 🎨 Apple Dark Mode Palette ---
BG_MAIN       = "#000000"  # pure black background
BG_CARD       = "#1C1C1E"  # elevated card
BG_CARD_ALT   = "#2C2C2E"  # secondary elevated
FG_PRIMARY    = "#FFFFFF"  # Primary Text
FG_SECONDARY  = "#98989D"  # Secondary Text (Gray)
FG_TERTIARY   = "#636366"  # Subtle dividers

# Accent Colors (iOS)
COLOR_BLUE    = "#0A84FF"
COLOR_GREEN   = "#32D74B"
COLOR_ORANGE  = "#FF9F0A"
COLOR_RED     = "#FF453A"
COLOR_PURPLE  = "#BF5AF2"
COLOR_TEAL    = "#64D2FF"

# Fonts (Graceful fallback to Helvetica/Arial)
FONT_H1       = ("Helvetica Neue", 20, "bold")
FONT_H2       = ("Helvetica Neue", 12, "bold")
FONT_LABEL    = ("Helvetica Neue", 11)
FONT_VALUE    = ("Menlo", 13)
FONT_BIG      = ("Menlo", 18, "bold")
FONT_STATUS   = ("Helvetica Neue", 10)

GRAPH_WINDOW_SEC = 10.0
GRAPH_UPDATE_HZ  = 20
START_THRESHOLD = 2.0
STOP_THRESHOLD  = 0.5


# =========================================================
# HELPER UI COMPONENTS
# =========================================================
def create_card(parent, title, bg_color=BG_CARD):
    """Creates a beautifully padded macOS style Card with a title"""
    outer_frame = tk.Frame(parent, bg=BG_MAIN)
    outer_frame.pack(fill=tk.X, padx=16, pady=(0, 16))
    
    # Header
    title_lbl = tk.Label(outer_frame, text=title.upper(), font=FONT_H2, fg=FG_SECONDARY, bg=BG_MAIN, anchor="w")
    title_lbl.pack(fill=tk.X, padx=4, pady=(0, 4))
    
    # Inner Card box
    card = tk.Frame(outer_frame, bg=bg_color, highlightbackground=FG_TERTIARY, highlightthickness=0)
    card.pack(fill=tk.BOTH, expand=True)
    # Give some internal padding
    inner = tk.Frame(card, bg=bg_color)
    inner.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)
    
    return inner

# =========================================================
# CORE LOGIC CLASSES (Simplified for clarity)
# =========================================================
class ExecutionMonitor:
    def __init__(self):
        self.state = "IDLE" 
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


class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('mg400_joint_monitor')
        self.pub_suction = self.create_publisher(Bool, SUCTION_TOPIC, 10)
        self.pub_light = self.create_publisher(Int32MultiArray, LIGHT_TOPIC, 10)
        
        # State
        self.latest_actual_joints = [0.0]*4
        self.latest_target_joints = [0.0]*4
        self.latest_predicted_joints = [0.0]*4
        self.latest_sent_joints = [0.0]*4
        self.latest_tool_actual = [0.0]*6
        self.latest_unity_xyz   = [0.0]*6
        self.latest_flange_actual = [0.0]*6
        self.latest_tool_index = -1
        self.latest_do_status = 0
        self.latest_robot_mode = 0
        self.latest_error_status = 0
        self.last_target_time = 0.0
        self.last_actual_time = 0.0
        
        # Subscriptions
        self.create_subscription(JointState, ACTUAL_TOPIC_NAME, self.cb_act, 10)
        self.create_subscription(JointState, TARGET_TOPIC_NAME, self.cb_tgt, 10)
        self.create_subscription(JointState, PREDICTED_TOPIC, self.cb_pred, 10)
        self.create_subscription(JointState, SENT_CMD_TOPIC, self.cb_sent, 10)
        self.create_subscription(Float64MultiArray, TOOL_ACTUAL_TOPIC, self.cb_tool, 10)
        self.create_subscription(Float64MultiArray, "/teleop/unity_xyz", self.cb_uxyz, 10)
        self.create_subscription(Float64MultiArray, "/robot/flange_actual", self.cb_flange, 10)
        self.create_subscription(Int32, "/robot/tool_index", self.cb_tidx, 10)
        self.create_subscription(Int64, DO_STATUS_TOPIC, self.cb_do, 10)
        self.create_subscription(Int32, ROBOT_MODE_TOPIC, self.cb_mode, 10)
        self.create_subscription(Int32, ERROR_STATUS_TOPIC, self.cb_err, 10)

    def request_suction(self, state):
        msg = Bool()
        msg.data = state
        self.pub_suction.publish(msg)

    def request_light(self, port, state):
        msg = Int32MultiArray()
        msg.data = [port, int(state)]
        self.pub_light.publish(msg)

    def cb_act(self, msg):
        if len(msg.position) >= 9:
            self.latest_actual_joints = list(np.degrees([msg.position[0], msg.position[1], msg.position[3], msg.position[8]]))
            self.last_actual_time = time.time()
    def cb_tgt(self, msg):
        if len(msg.position) >= 4:
            self.latest_target_joints = list(np.degrees(msg.position[:4]))
            self.last_target_time = time.time()
    def cb_pred(self, msg):
        if len(msg.position) >= 4: self.latest_predicted_joints = list(np.degrees(msg.position[:4]))
    def cb_sent(self, msg):
        if len(msg.position) >= 4: self.latest_sent_joints = list(np.degrees(msg.position[:4]))
    def cb_tool(self, msg):
        if len(msg.data) >= 6: self.latest_tool_actual = list(msg.data)
    def cb_uxyz(self, msg):
        if len(msg.data) >= 6: self.latest_unity_xyz = list(msg.data)
    def cb_flange(self, msg):
        if len(msg.data) >= 6: self.latest_flange_actual = list(msg.data)
    def cb_tidx(self, msg): self.latest_tool_index = int(msg.data)
    def cb_do(self, msg): self.latest_do_status = int(msg.data)
    def cb_mode(self, msg): self.latest_robot_mode = int(msg.data)
    def cb_err(self, msg): self.latest_error_status = int(msg.data)


# =========================================================
# BEAUTIFUL GUI APPLICATION
# =========================================================
class MonitorGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.monitor = ExecutionMonitor()
        self.lockout = {}
        
        self.root.title("MG400 Monitor")
        self.root.configure(bg=BG_MAIN)
        self.root.geometry("1100x850")

        # Layout
        self.top_bar = tk.Frame(root, bg=BG_MAIN)
        self.top_bar.pack(fill=tk.X, padx=20, pady=(20, 10))
        
        # Header Title
        tk.Label(self.top_bar, text="VR Teleoperation", font=FONT_H1, fg=FG_PRIMARY, bg=BG_MAIN).pack(side=tk.LEFT)
        
        self.status_pill_frame = tk.Frame(self.top_bar, bg=BG_MAIN)
        self.status_pill_frame.pack(side=tk.LEFT, padx=16, pady=4)
        
        self.lbl_mode = tk.Label(self.status_pill_frame, text="INIT", font=FONT_STATUS, fg=FG_MAIN, bg=COLOR_PURPLE, padx=10, pady=2, relief="flat")
        self.lbl_mode.pack(side=tk.LEFT, padx=4)
        
        self.lbl_err = tk.Label(self.status_pill_frame, text="OK", font=FONT_STATUS, fg=BG_MAIN, bg=COLOR_GREEN, padx=10, pady=2)
        self.lbl_err.pack(side=tk.LEFT, padx=4)

        # Connection Latency
        self.var_latency = tk.StringVar(value="Data latency: ...")
        tk.Label(self.top_bar, textvariable=self.var_latency, font=FONT_STATUS, fg=FG_SECONDARY, bg=BG_MAIN).pack(side=tk.RIGHT)
        
        # Content Split
        self.content = tk.Frame(root, bg=BG_MAIN)
        self.content.pack(fill=tk.BOTH, expand=True)
        
        # Left Panel (Data & Controls)
        self.left_panel = tk.Frame(self.content, bg=BG_MAIN, width=400)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=4)
        self.left_panel.pack_propagate(False)

        # Right Panel (Graphs)
        self.right_panel = tk.Frame(self.content, bg=BG_MAIN)
        self.right_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        # Build Sections
        self._build_xyz_card()
        self._build_joints_card()
        self._build_controls_card()
        self._build_metrics_card()
        
        # Graphs
        self._setup_graphs(self.right_panel)

        self.update_gui()

    def _build_xyz_card(self):
        card = create_card(self.left_panel, "End Effector (XYZ)")
        
        # Grid layout for items
        self.vars_xyz_tgt = []
        self.vars_xyz_act = []
        
        # Header row
        for j, txt in enumerate(["Axis", "Unity", "Actual"]):
            col = FG_PRIMARY if j==0 else (COLOR_ORANGE if j==1 else COLOR_GREEN)
            tk.Label(card, text=txt, font=FONT_LABEL, fg=col, bg=BG_CARD).grid(row=0, column=j, sticky="w", pady=(0,8), padx=(0, 20))
            
        for i, axes in enumerate(["X", "Y", "Z"]):
            tk.Label(card, text=axes, font=FONT_VALUE, fg=FG_SECONDARY, bg=BG_CARD).grid(row=i+1, column=0, sticky="w", pady=4)
            
            vt = tk.StringVar(value="0.0")
            tk.Label(card, textvariable=vt, font=FONT_VALUE, fg=FG_PRIMARY, bg=BG_CARD).grid(row=i+1, column=1, sticky="w", pady=4, padx=(0, 20))
            self.vars_xyz_tgt.append(vt)
            
            va = tk.StringVar(value="0.0")
            tk.Label(card, textvariable=va, font=FONT_VALUE, fg=FG_PRIMARY, bg=BG_CARD).grid(row=i+1, column=2, sticky="w", pady=4)
            self.vars_xyz_act.append(va)

    def _build_joints_card(self):
        card = create_card(self.left_panel, "Joint Angles")
        
        self.vars_jtgt = []
        self.vars_jact = []
        self.lbls_jdiff = []
        self.vars_jdiff = []
        
        for j, txt in enumerate(["", "Target", "Actual", "Diff"]):
            col = FG_PRIMARY if j==0 else (COLOR_ORANGE if j==1 else (COLOR_GREEN if j==2 else FG_SECONDARY))
            tk.Label(card, text=txt, font=FONT_LABEL, fg=col, bg=BG_CARD).grid(row=0, column=j, sticky="w", pady=(0,8), padx=(0, 15))
            
        for i in range(4):
            tk.Label(card, text=f"J{i+1}", font=FONT_VALUE, fg=FG_SECONDARY, bg=BG_CARD).grid(row=i+1, column=0, sticky="w", pady=4)
            
            vt = tk.StringVar(value="0.00")
            tk.Label(card, textvariable=vt, font=FONT_VALUE, fg=FG_PRIMARY, bg=BG_CARD).grid(row=i+1, column=1, sticky="w", pady=4, padx=(0, 15))
            self.vars_jtgt.append(vt)
            
            va = tk.StringVar(value="0.00")
            tk.Label(card, textvariable=va, font=FONT_VALUE, fg=FG_PRIMARY, bg=BG_CARD).grid(row=i+1, column=2, sticky="w", pady=4, padx=(0, 15))
            self.vars_jact.append(va)
            
            vd = tk.StringVar(value="0.00")
            lbl = tk.Label(card, textvariable=vd, font=FONT_VALUE, fg=FG_SECONDARY, bg=BG_CARD)
            lbl.grid(row=i+1, column=3, sticky="w", pady=4)
            self.vars_jdiff.append(vd)
            self.lbls_jdiff.append(lbl)

    def _build_controls_card(self):
        card = create_card(self.left_panel, "I/O Control")
        
        self.suction_state = False
        self.btn_suction = tk.Button(card, text="SUCTION (OFF)", font=FONT_H2, bg=BG_CARD_ALT, fg=FG_PRIMARY, relief="flat", command=self.toggle_suction, highlightbackground=BG_CARD)
        self.btn_suction.pack(fill=tk.X, pady=(0, 12), ipady=4)
        
        # Lights container
        lc = tk.Frame(card, bg=BG_CARD)
        lc.pack(fill=tk.X)
        
        self.light_states = {"G": False, "Y": False, "R": False}
        self.btns_light = {}
        for name, port, col_off, col_on in [
            ("G", GREEN_LIGHT_DO_PORT, BG_CARD_ALT, COLOR_GREEN),
            ("Y", YELLOW_LIGHT_DO_PORT, BG_CARD_ALT, COLOR_ORANGE),
            ("R", RED_LIGHT_DO_PORT, BG_CARD_ALT, COLOR_RED)
        ]:
            btn = tk.Button(lc, text=name, font=FONT_H2, bg=col_off, fg=FG_PRIMARY, relief="flat", command=lambda n=name, p=port, c=col_on: self.toggle_light(n, p, c), highlightbackground=BG_CARD)
            btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2, ipady=4)
            self.btns_light[name] = {"btn": btn, "on_col": col_on}

    def _build_metrics_card(self):
        card = create_card(self.left_panel, "Execution")
        
        self.var_mov_stat = tk.StringVar(value="IDLE")
        self.lbl_mov_stat = tk.Label(card, textvariable=self.var_mov_stat, font=FONT_H2, fg=FG_SECONDARY, bg=BG_CARD)
        self.lbl_mov_stat.pack(side=tk.LEFT)
        
        self.var_timer = tk.StringVar(value="0.00s")
        tk.Label(card, textvariable=self.var_timer, font=FONT_BIG, fg=FG_PRIMARY, bg=BG_CARD).pack(side=tk.RIGHT)

    def toggle_suction(self):
        self.suction_state = not self.suction_state
        self.node.request_suction(self.suction_state)
        self.lockout[VACUUM_DO_PORT] = time.time() + 2.0
        
    def toggle_light(self, name, port, color):
        self.light_states[name] = not self.light_states[name]
        self.node.request_light(port, self.light_states[name])
        self.lockout[port] = time.time() + 2.0
        self._update_light_btns()

    def _update_light_btns(self):
        for name, data in self.btns_light.items():
            st = self.light_states[name]
            # using foreground mapping since bg styling on Mac tk buttons is slightly weird 
            # without ttk, but we fake it
            pass

    def _setup_graphs(self, parent):
        # Frame wrapper for styling padding
        wrapper = tk.Frame(parent, bg=BG_CARD)
        wrapper.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0,16))
        
        # Top right legends
        legend_frame = tk.Frame(wrapper, bg=BG_CARD)
        legend_frame.pack(fill=tk.X, padx=16, pady=12)
        tk.Label(legend_frame, text="Real-Time Tracking", font=FONT_H2, bg=BG_CARD, fg=FG_PRIMARY).pack(side=tk.LEFT)
        
        for name, col in [("Target", COLOR_ORANGE), ("Predicted", COLOR_PURPLE), ("Sent", COLOR_BLUE), ("Actual", COLOR_GREEN)]:
            tk.Label(legend_frame, text="● "+name, font=FONT_LABEL, bg=BG_CARD, fg=col).pack(side=tk.LEFT, padx=8)

        # Plotly-like dark styling
        self.fig = Figure(figsize=(6, 6), dpi=100, facecolor=BG_CARD)
        self.fig.subplots_adjust(hspace=0.3, left=0.08, right=0.98, top=0.95, bottom=0.05)
        
        self.axes = []
        self.graph_lines = []
        
        max_pts = int(GRAPH_WINDOW_SEC * GRAPH_UPDATE_HZ * 1.5) # safety buffer
        self.t_buff = deque(maxlen=max_pts)
        self.u_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.p_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.s_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.a_buff = [deque(maxlen=max_pts) for _ in range(4)]
        self.lsv = [0.0]*4
        self.t0 = time.time()
        
        for i in range(4):
            ax = self.fig.add_subplot(4,1,i+1)
            ax.set_facecolor(BG_CARD)
            ax.tick_params(colors=FG_TERTIARY, labelsize=8)
            for sp in ax.spines.values():
                sp.set_visible(False)
            ax.grid(True, color=FG_TERTIARY, alpha=0.3, linestyle="-", lw=0.5)
            ax.set_ylabel(f"J{i+1}", color=FG_SECONDARY, fontsize=10, rotation=0, labelpad=15)
            
            lu, = ax.plot([], [], color=COLOR_ORANGE, lw=1.5, alpha=0.7)
            lp, = ax.plot([], [], color=COLOR_PURPLE, lw=1.2, alpha=0.8)
            ls, = ax.plot([], [], color=COLOR_BLUE, lw=1.5, drawstyle="steps-post", alpha=0.9)
            la, = ax.plot([], [], color=COLOR_GREEN, lw=2.0)
            
            self.axes.append(ax)
            self.graph_lines.append((lu,lp,ls,la))

        canvas = FigureCanvasTkAgg(self.fig, master=wrapper)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        
        self.ani = animation.FuncAnimation(self.fig, self._update_g, interval=1000//GRAPH_UPDATE_HZ, blit=True)

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
                pad = max(2.0, (mx - mn)*0.15)
                ax.set_ylim(mn - pad, mx + pad)
            
            all_l.extend([u,p,s,a])
        return all_l

    def update_gui(self):
        tgt = self.node.latest_target_joints
        act = self.node.latest_actual_joints
        
        tot_d = 0.0
        for i in range(4):
            self.vars_jtgt[i].set(f"{tgt[i]:.2f}°")
            self.vars_jact[i].set(f"{act[i]:.2f}°")
            d = act[i] - tgt[i]
            self.vars_jdiff[i].set(f"{d:+.2f}°")
            tot_d += abs(d)
            if abs(d) > 2.0: self.lbls_jdiff[i].config(fg=COLOR_RED)
            elif abs(d) > 0.5: self.lbls_jdiff[i].config(fg=COLOR_ORANGE)
            else: self.lbls_jdiff[i].config(fg=FG_SECONDARY)

        ux = self.node.latest_unity_xyz
        tx = self.node.latest_tool_actual
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{ux[i]:.1f}")
            self.vars_xyz_act[i].set(f"{tx[i]:.1f}")

        # Metrics
        st = self.monitor.update(tot_d)
        if st == "MOVING":
            self.var_mov_stat.set("● MOVING")
            self.lbl_mov_stat.config(fg=COLOR_ORANGE)
            self.var_timer.set(f"{time.time() - self.monitor.start_time:.2f}s")
        elif st == "ARRIVED":
            self.var_mov_stat.set("✓ ARRIVED")
            self.lbl_mov_stat.config(fg=COLOR_GREEN)
            self.var_timer.set(f"{self.monitor.last_duration:.2f}s")
        else:
            self.var_mov_stat.set("IDLE")
            self.lbl_mov_stat.config(fg=FG_SECONDARY)
            self.var_timer.set("0.00s")

        # Status
        mode = self.node.latest_robot_mode
        err = self.node.latest_error_status
        m_names = {1:"INIT", 4:"DISABLED", 5:"ENABLE", 6:"DRAG", 7:"RUN", 9:"ERROR", 11:"COLLIDE"}
        m_str = m_names.get(mode, str(mode))
        
        self.lbl_mode.config(text=m_str, bg=COLOR_PURPLE if mode==5 else FG_TERTIARY)
        self.lbl_err.config(text=f"ERR: {err:02X}", bg=COLOR_RED if err!=0 else COLOR_GREEN, fg=BG_MAIN)

        tm_t = time.time() - self.node.last_target_time
        tm_a = time.time() - self.node.last_actual_time
        self.var_latency.set(f"Cmd: {tm_t*1000:.0f}ms • Fbk: {tm_a*1000:.0f}ms")

        # DO Sync
        now = time.time()
        do_s = self.node.latest_do_status
        if now > self.lockout.get(VACUUM_DO_PORT, 0):
            act_suc = bool((do_s >> (VACUUM_DO_PORT-1)) & 1)
            self.suction_state = act_suc
            self.btn_suction.config(
                text="SUCTION (ON)" if act_suc else "SUCTION (OFF)", 
                fg=BG_MAIN if act_suc else FG_PRIMARY,
                bg=COLOR_TEAL if act_suc else BG_CARD_ALT
            )
        
        for name, port, col_off, col_on in [("G", GREEN_LIGHT_DO_PORT, BG_CARD_ALT, COLOR_GREEN), ("Y", YELLOW_LIGHT_DO_PORT, BG_CARD_ALT, COLOR_ORANGE), ("R", RED_LIGHT_DO_PORT, BG_CARD_ALT, COLOR_RED)]:
            if now > self.lockout.get(port, 0):
                act_l = bool((do_s >> (port-1)) & 1)
                self.light_states[name] = act_l
                self.btns_light[name]["btn"].config(bg=col_on if act_l else BG_CARD_ALT, fg=BG_MAIN if act_l else FG_PRIMARY)

        self.root.after(50, self.update_gui)

def main(args=None):
    rclpy.init(args=args)
    node = JointMonitorNode()
    
    # Thread mapping
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    root = tk.Tk()
    app = MonitorGUI(root, node)
    root.protocol("WM_DELETE_WINDOW", lambda: (node.destroy_node(), rclpy.shutdown(), root.destroy()))
    root.mainloop()

if __name__ == '__main__':
    main()
