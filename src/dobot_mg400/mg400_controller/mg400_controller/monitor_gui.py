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
import tkinter as tk
from tkinter import ttk
import threading
import sys
import time

# --- Matplotlib ---
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

from std_msgs.msg import Bool, Int32MultiArray
from mg400_controller.common.utils.error.error_decoder import RobotErrorDecoder
from mg400_controller.common.utils.loggers.monitor_logging import (
    SessionLogger,
    ManualMonitorLogger,
)
from mg400_controller.common.monitor.control_panel_state import (
    COLOR_OFF,
    LIGHT_SPECS,
    MonitorControlPanelState,
)
from mg400_controller.common.monitor.execution_metrics import ExecutionMonitor
from mg400_controller.common.monitor.joint_graph_buffer import JointGraphBuffer
from mg400_controller.common.monitor.presentation_state import (
    build_cartesian_display_state,
    build_joint_display_rows,
    build_status_display_state,
)
from mg400_controller.common.ros.monitor_interfaces import (
    MonitorTelemetryState,
    create_control_publishers,
    create_monitor_subscriptions,
)
from mg400_controller.common.ros.topic_config import declare_topic_parameters

# Graph colors
COL_UNITY = "#ff7f0e"      # Matplotlib standard orange
COL_PREDICTED = "#9467bd"  # Matplotlib standard purple
COL_SENT = "#d62728"       # Matplotlib standard red
COL_ACTUAL = "#1f77b4"     # Matplotlib standard blue

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


class JointMonitorNode(Node):
    def __init__(self):
        super().__init__('mg400_joint_monitor')
        self.topics = declare_topic_parameters(self)
        self.control_publishers = create_control_publishers(self, topics=self.topics)
        self.telemetry = MonitorTelemetryState()
        self.ros_subscriptions = create_monitor_subscriptions(self, self.telemetry, topics=self.topics)

    def request_suction(self, state):
        msg = Bool()
        msg.data = state
        self.control_publishers.suction.publish(msg)

    def request_light(self, port, state):
        msg = Int32MultiArray()
        msg.data = [port, int(state)]
        self.control_publishers.light.publish(msg)


class MonitorGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.monitor = ExecutionMonitor()
        self.control_panel = MonitorControlPanelState()
        
        self.root.title("MG400 Extended Monitor")
        self.root.configure(bg="#1a1a2e")
        self.manual_logger = ManualMonitorLogger()

        # ===== MAIN LAYOUT =====
        # Left panel: existing controls + tables
        # Right panel: real-time graphs
        
        # Use a PanedWindow so the user can drag the separator between GUI and Graphs
        outer = tk.PanedWindow(root, orient=tk.HORIZONTAL, bg="#1a1a2e", sashwidth=5, sashrelief=tk.RAISED)
        outer.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(outer, bg="#f0f0f0")
        right_frame = tk.Frame(outer, bg="white")

        outer.add(left_frame, minsize=600, stretch="never") # Left side stays its natural size or min 600
        outer.add(right_frame, minsize=400, stretch="always") # Right side takes all extra expanding space

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
        self.btn_log = tk.Button(title_frame, text="▶ Start Logging", command=self.toggle_logging, bg="#f0f0f0")
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
        ttk.Label(header_xyz, text="Axis",          font=FONT_LABEL, width=6).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Unity FK (mm)", font=FONT_LABEL, foreground="darkorange",  width=13).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Flange (mm)",   font=FONT_LABEL, foreground="royalblue",   width=13).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="TCP (mm)",      font=FONT_LABEL, foreground="green4",      width=13).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="ToolΔ (mm)",   font=FONT_LABEL, foreground="gray40",      width=12).pack(side=tk.LEFT)

        self.vars_xyz_tgt    = []
        self.vars_xyz_flange = []
        self.vars_xyz_act    = []
        self.vars_xyz_tool   = []
        self.vars_xyz_diff   = []   # compat alias
        self.lbls_xyz_diff   = []

        for i, name in enumerate(["X", "Y", "Z"]):
            frame = ttk.Frame(main_frame)
            frame.pack(fill=tk.X, pady=2)
            ttk.Label(frame, text=name, font=FONT_LABEL, width=7).pack(side=tk.LEFT)

            v_tgt = tk.StringVar(value="0.0")
            ttk.Label(frame, textvariable=v_tgt, font=FONT_VALUE, foreground="darkorange", width=12).pack(side=tk.LEFT)
            self.vars_xyz_tgt.append(v_tgt)

            v_flange = tk.StringVar(value="0.0")
            ttk.Label(frame, textvariable=v_flange, font=FONT_VALUE, foreground="royalblue", width=12).pack(side=tk.LEFT)
            self.vars_xyz_flange.append(v_flange)

            v_act = tk.StringVar(value="0.0")
            ttk.Label(frame, textvariable=v_act, font=FONT_VALUE, foreground="green4", width=12).pack(side=tk.LEFT)
            self.vars_xyz_act.append(v_act)

            v_tool = tk.StringVar(value="0.0")
            lbl_tool = ttk.Label(frame, textvariable=v_tool, font=FONT_VALUE, foreground="gray40", width=11)
            lbl_tool.pack(side=tk.LEFT)
            self.vars_xyz_tool.append(v_tool)
            # compat aliases for session logger
            self.vars_xyz_diff.append(v_tool)
            self.lbls_xyz_diff.append(lbl_tool)

        # Tool Index label row (below XYZ table)
        tool_idx_row = ttk.Frame(main_frame)
        tool_idx_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(tool_idx_row, text="Active Tool:", font=FONT_LABEL, width=14).pack(side=tk.LEFT)
        self.var_tool_index = tk.StringVar(value="— (querying...)")
        ttk.Label(tool_idx_row, textvariable=self.var_tool_index, font=FONT_VALUE, foreground="gray40").pack(side=tk.LEFT)

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=10)

        # --- 🎮 CONTROL PANEL ---
        control_frame = ttk.LabelFrame(main_frame, text="Robot Direct Control", padding="10")
        control_frame.pack(fill=tk.X, pady=5)

        suction_row = ttk.Frame(control_frame)
        suction_row.pack(fill=tk.X, pady=5)
        ttk.Label(suction_row, text="Suction:", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        self.btn_suction = tk.Button(suction_row, text="OFF", font=FONT_VALUE, width=10, command=self.toggle_suction)
        self.btn_suction.pack(side=tk.LEFT, padx=5)

        light_row = ttk.Frame(control_frame)
        light_row.pack(fill=tk.X, pady=10)
        ttk.Label(light_row, text="Lights:", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        self.btns_light = {}

        for name, _, _ in LIGHT_SPECS:
            btn = tk.Button(light_row, text=name, font=("Helvetica", 10, "bold"), width=8, bg=COLOR_OFF, 
                            command=lambda n=name: self.toggle_light(n))
            btn.pack(side=tk.LEFT, padx=2)
            self.btns_light[name] = btn

        self._apply_control_panel_sync(self.control_panel.build_initial_sync())

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

        # Button moved to top title_frame

        # ===== RIGHT PANEL: REAL-TIME GRAPHS =====
        self._setup_graphs(right_frame)

        # Decoder for error messages
        self.error_decoder = RobotErrorDecoder()

        # Start Update Loop
        self.update_gui()

    def _setup_graphs(self, parent):
        """Create the matplotlib figure with 4 subplots for J1-J4."""
        # Legend label strip at top
        legend_frame = tk.Frame(parent, bg="white")
        legend_frame.pack(fill=tk.X, padx=10, pady=(10, 0))
        
        tk.Label(legend_frame, text="📊 Joint Tracking Graphs", font=("Helvetica", 13, "bold"),
                 bg="white", fg="black").pack(side=tk.LEFT)
        
        legend_right = tk.Frame(legend_frame, bg="white")
        legend_right.pack(side=tk.RIGHT)
        
        legends = [
            ("Unity Raw", COL_UNITY),
            ("Predicted", COL_PREDICTED),
            ("Cmd Sent", COL_SENT),
            ("Actual", COL_ACTUAL),
        ]
        for lname, lcolor in legends:
            tk.Label(legend_right, text=f"─ {lname}", font=("Helvetica", 10, "bold"),
                     bg="white", fg=lcolor).pack(side=tk.LEFT, padx=8)

        # Matplotlib Figure - Standard Mode
        self.fig = Figure(figsize=(8, 8), dpi=96, facecolor="white")
        self.fig.subplots_adjust(hspace=0.4, left=0.12, right=0.97, top=0.97, bottom=0.07)
        
        joint_labels = ["J1 (°)", "J2 (°)", "J3 (°)", "J4 (°)"]
        self.axes = []
        self.graph_lines = []  # list of (unity_line, pred_line, sent_line, actual_line) per joint
        self.graph_buffer = JointGraphBuffer(window_sec=GRAPH_WINDOW_SEC)
        
        # ✅ Auto-start precise session logger
        self.session_logger = SessionLogger(self._build_session_snapshot)
        
        for i in range(4):
            ax = self.fig.add_subplot(4, 1, i + 1)
            ax.set_facecolor("white")
            ax.set_ylabel(joint_labels[i], color="black", fontsize=9)
            ax.tick_params(colors="black", labelsize=8)
            ax.spines['bottom'].set_color('black')
            ax.spines['left'].set_color('black')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(True, color="#e0e0e0", linewidth=0.5, linestyle="--")

            l_unity,  = ax.plot([], [], color=COL_UNITY, lw=1.2, linestyle="--", alpha=0.8, label="Unity")
            l_pred,   = ax.plot([], [], color=COL_PREDICTED, lw=1.0, alpha=0.85, label="Predicted")
            # Changed from continuous line to discrete dots for Sent commands
            l_sent,   = ax.plot([], [], color=COL_SENT, marker='o', markersize=4, linestyle='None', alpha=0.9, label="Sent")
            l_actual, = ax.plot([], [], color=COL_ACTUAL, lw=1.5, label="Actual")

            if i == 3:
                ax.set_xlabel("Time (s)", color="gray", fontsize=9)

            self.axes.append(ax)
            self.graph_lines.append((l_unity, l_pred, l_sent, l_actual))
        
        canvas = FigureCanvasTkAgg(self.fig, master=parent)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.canvas = canvas
        
        # Animation - drives graph redraws
        # blit=False: more stable, avoids animation stopping when blitting fails
        self.ani = animation.FuncAnimation(
            self.fig, self._update_graphs,
            interval=int(1000 / GRAPH_UPDATE_HZ),
            blit=False,
            cache_frame_data=False
        )

    def _update_graphs(self, frame):
        """Called by matplotlib animation to refresh graph lines."""
        rel_t = self.graph_buffer.append_telemetry(self.node.telemetry)
        
        all_lines = []
        for i in range(4):
            l_unity, l_pred, l_sent, l_actual = self.graph_lines[i]
            ax = self.axes[i]
            t_arr, u, p, s, a = self.graph_buffer.get_joint_arrays(i)
            
            l_unity.set_data(t_arr, u)
            l_pred.set_data(t_arr, p)
            l_sent.set_data(t_arr, s)
            l_actual.set_data(t_arr, a)
            
            # Auto-scale axes
            ax.set_xlim(*self.graph_buffer.get_x_limits(rel_t))
            
            if len(a) > 0:
                limits = self.graph_buffer.get_y_limits(u, p, s, a)
                if limits is not None:
                    ax.set_ylim(*limits)
            
            all_lines.extend([l_unity, l_pred, l_sent, l_actual])
        
        return all_lines

    def toggle_suction(self):
        update = self.control_panel.toggle_suction()
        self.node.request_suction(update.state)
        self._apply_button_display(self.btn_suction, update.display)

    def toggle_light(self, name):
        update = self.control_panel.toggle_light(name)
        self.node.request_light(update.port, update.state)
        self._apply_button_display(self.btns_light[name], update.display)

    def toggle_logging(self):
        if not self.manual_logger.is_active:
            self.btn_log.config(text="⏹ Stop Logging", bg="yellow")
            self.manual_logger.start_session()
            self.node.get_logger().info(f"Started manual logging to {self.manual_logger.target_path}")
        else:
            self.manual_logger.stop_session()
            self.btn_log.config(text="▶ Start Logging", bg="#f0f0f0")
            self.node.get_logger().info("Stopped manual logging.")

    def _build_session_snapshot(self):
        return self.node.telemetry.build_session_snapshot()

    def _apply_button_display(self, button, display):
        button.config(text=display.text, bg=display.bg, fg=display.fg)

    def _apply_control_panel_sync(self, sync):
        if sync.suction is not None:
            self._apply_button_display(self.btn_suction, sync.suction)
        for name, display in sync.lights.items():
            self._apply_button_display(self.btns_light[name], display)

    def update_gui(self):
        # Get latest data
        telemetry = self.node.telemetry
        tgt = telemetry.latest_target_joints
        act = telemetry.latest_actual_joints
        joint_rows, total_diff = build_joint_display_rows(tgt, act)
        cartesian_state = build_cartesian_display_state(telemetry)
        status_state = build_status_display_state(telemetry, self.error_decoder)
        
        # --- Update Joint Data ---
        for i, row in enumerate(joint_rows):
            self.vars_target[i].set(row["target"])
            self.vars_actual[i].set(row["actual"])
            self.vars_diff[i].set(row["diff"])
            self.lbls_diff[i].configure(foreground=row["color"])

        # --- Update System Info ---
        self.var_mode.set(status_state["mode_text"])
        self.var_error.set(status_state["error_text"])
        self.lbl_error.configure(fg=status_state["error_color"])
        
        # --- Update Button States (DO Status Sync) ---
        do_status = telemetry.latest_do_status
        self._apply_control_panel_sync(self.control_panel.sync_from_do_status(do_status))

        # --- Update Cartesian Data ---
        xyz_unity = cartesian_state["unity_xyz"]
        xyz_flange = cartesian_state["flange_xyz"]
        xyz_tcp = cartesian_state["tcp_xyz"]
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{xyz_unity[i]:.1f}")
            self.vars_xyz_flange[i].set(f"{xyz_flange[i]:.1f}")
            self.vars_xyz_act[i].set(f"{xyz_tcp[i]:.1f}")
            self.vars_xyz_tool[i].set(f"{cartesian_state['tool_delta'][i]:+.1f}")

        # Tool index label
        self.var_tool_index.set(cartesian_state["tool_index_text"])

        # --- Execution Monitor ---
        self.monitor.update(total_diff)
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
        self.var_latency.set(status_state["latency_text"])
        self.var_do_hex.set(status_state["do_hex_text"])
            
        # --- CSV Logging ---
        self.manual_logger.log_sample(
            target_xyz=xyz_unity,
            actual_xyz=xyz_tcp,
            target_joints=tgt,
            actual_joints=act,
            total_diff=total_diff,
        )

        # ✅ Session Logger runs in its own background thread internally

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
        try:
            gui.manual_logger.stop_session()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
