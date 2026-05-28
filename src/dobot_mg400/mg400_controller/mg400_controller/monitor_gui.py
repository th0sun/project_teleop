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
from mg400_controller.common.utils.monitor.control_panel_state import (
    COLOR_OFF,
    LIGHT_SPECS,
    MonitorControlPanelState,
)
from mg400_controller.common.utils.monitor.execution_metrics import ExecutionMonitor
from mg400_controller.common.utils.monitor.joint_graph_buffer import JointGraphBuffer
from mg400_controller.common.utils.monitor.presentation_state import (
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
        """Construct the monitor GUI.

        Layout flow (each ``_build_*`` helper owns one UI section so
        the wall of Tk widget construction stays one screen apiece):

          - prologue: window chrome + ExecutionMonitor + control panel
            state + manual logger.
          - left pane:  title / joint table / Cartesian table / tool
                        index / control panel / execution metrics /
                        status bar.
          - right pane: matplotlib graphs.

        After widgets exist, kick off the 20 Hz Tk ``update_gui`` loop
        and the matplotlib animation.
        """
        self.root = root
        self.node = node
        self.monitor = ExecutionMonitor()
        self.control_panel = MonitorControlPanelState()
        self.manual_logger = ManualMonitorLogger()
        self.error_decoder = RobotErrorDecoder()

        self.root.title("MG400 Extended Monitor")
        self.root.configure(bg="#1a1a2e")

        main_frame, right_frame = self._build_main_layout()
        self._build_title_row(main_frame)
        self._build_joint_table(main_frame)
        self._build_cartesian_table(main_frame)
        self._build_control_panel(main_frame)
        self._build_execution_metrics(main_frame)
        self._build_status_bar(main_frame)
        self._setup_graphs(right_frame)

        # Start Update Loop
        self.update_gui()

    def _build_main_layout(self):
        """Outer PanedWindow split between left controls/tables and
        right real-time graphs. Returns ``(main_frame, right_frame)``;
        the left frame's padded interior is the parent every left-side
        builder writes to.
        """
        outer = tk.PanedWindow(
            self.root, orient=tk.HORIZONTAL, bg="#1a1a2e",
            sashwidth=5, sashrelief=tk.RAISED,
        )
        outer.pack(fill=tk.BOTH, expand=True)
        left_frame = tk.Frame(outer, bg="#f0f0f0")
        right_frame = tk.Frame(outer, bg="white")
        outer.add(left_frame, minsize=600, stretch="never")
        outer.add(right_frame, minsize=400, stretch="always")

        main_frame = ttk.Frame(left_frame, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)
        return main_frame, right_frame

    def _build_title_row(self, main_frame):
        """Title label + Start/Stop logging toggle button. Logging
        status is tracked on ``self.manual_logger.is_active``; the
        button only mutates that flag via ``toggle_logging``.
        """
        title_frame = ttk.Frame(main_frame)
        title_frame.pack(fill=tk.X, pady=5)
        ttk.Label(
            title_frame, text="Real-time Monitor & Metrics", font=FONT_HEADER
        ).pack(side=tk.LEFT)
        self.btn_log = tk.Button(
            title_frame, text="▶ Start Logging",
            command=self.toggle_logging, bg="#f0f0f0",
        )
        self.btn_log.pack(side=tk.RIGHT)

    def _build_joint_table(self, main_frame):
        """Four-row joint table (J1..J4) showing target / actual /
        diff in degrees. The text vars are kept on
        ``self.vars_target / vars_actual / vars_diff`` and the diff
        labels themselves are kept on ``self.lbls_diff`` so
        ``update_gui`` can recolour them based on threshold.
        """
        header = ttk.Frame(main_frame)
        header.pack(fill=tk.X, pady=5)
        for text, width in (("Joint", 10), ("Target (°)", 15), ("Actual (°)", 15), ("Diff (°)", 15)):
            ttk.Label(header, text=text, font=FONT_LABEL, width=width).pack(side=tk.LEFT)
        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=5)

        self.vars_target, self.vars_actual, self.vars_diff, self.lbls_diff = [], [], [], []
        for name in ("J1", "J2", "J3", "J4"):
            row = ttk.Frame(main_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=name, font=FONT_LABEL, width=12).pack(side=tk.LEFT)

            v_tgt = tk.StringVar(value="0.00")
            ttk.Label(row, textvariable=v_tgt, font=FONT_VALUE, foreground="darkgreen", width=12).pack(side=tk.LEFT)
            self.vars_target.append(v_tgt)

            v_act = tk.StringVar(value="0.00")
            ttk.Label(row, textvariable=v_act, font=FONT_VALUE, foreground="blue", width=12).pack(side=tk.LEFT)
            self.vars_actual.append(v_act)

            v_diff = tk.StringVar(value="0.00")
            lbl_diff = ttk.Label(row, textvariable=v_diff, font=FONT_VALUE, foreground="black", width=12)
            lbl_diff.pack(side=tk.LEFT)
            self.vars_diff.append(v_diff)
            self.lbls_diff.append(lbl_diff)
        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=10)

    def _build_cartesian_table(self, main_frame):
        """X/Y/Z table showing Unity-FK / Flange / TCP / Tool-delta in
        mm, plus the Active Tool index row underneath. ``vars_xyz_diff``
        and ``lbls_xyz_diff`` are compat aliases the session logger
        still reads.
        """
        ttk.Label(
            main_frame, text="Cartesian Coordinates (End Effector)",
            font=("Helvetica", 12, "bold"),
        ).pack(anchor=tk.W)

        header = ttk.Frame(main_frame)
        header.pack(fill=tk.X, pady=2)
        for text, width, fg in (
            ("Axis", 6, None),
            ("Unity FK (mm)", 13, "darkorange"),
            ("Flange (mm)", 13, "royalblue"),
            ("TCP (mm)", 13, "green4"),
            ("ToolΔ (mm)", 12, "gray40"),
        ):
            kwargs = {"font": FONT_LABEL, "width": width}
            if fg:
                kwargs["foreground"] = fg
            ttk.Label(header, text=text, **kwargs).pack(side=tk.LEFT)

        self.vars_xyz_tgt, self.vars_xyz_flange = [], []
        self.vars_xyz_act, self.vars_xyz_tool = [], []
        self.vars_xyz_diff, self.lbls_xyz_diff = [], []  # compat aliases
        for name in ("X", "Y", "Z"):
            row = ttk.Frame(main_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=name, font=FONT_LABEL, width=7).pack(side=tk.LEFT)

            v_tgt = tk.StringVar(value="0.0")
            ttk.Label(row, textvariable=v_tgt, font=FONT_VALUE, foreground="darkorange", width=12).pack(side=tk.LEFT)
            self.vars_xyz_tgt.append(v_tgt)
            v_flange = tk.StringVar(value="0.0")
            ttk.Label(row, textvariable=v_flange, font=FONT_VALUE, foreground="royalblue", width=12).pack(side=tk.LEFT)
            self.vars_xyz_flange.append(v_flange)
            v_act = tk.StringVar(value="0.0")
            ttk.Label(row, textvariable=v_act, font=FONT_VALUE, foreground="green4", width=12).pack(side=tk.LEFT)
            self.vars_xyz_act.append(v_act)
            v_tool = tk.StringVar(value="0.0")
            lbl_tool = ttk.Label(row, textvariable=v_tool, font=FONT_VALUE, foreground="gray40", width=11)
            lbl_tool.pack(side=tk.LEFT)
            self.vars_xyz_tool.append(v_tool)
            self.vars_xyz_diff.append(v_tool)
            self.lbls_xyz_diff.append(lbl_tool)

        tool_idx_row = ttk.Frame(main_frame)
        tool_idx_row.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(tool_idx_row, text="Active Tool:", font=FONT_LABEL, width=14).pack(side=tk.LEFT)
        self.var_tool_index = tk.StringVar(value="— (querying...)")
        ttk.Label(
            tool_idx_row, textvariable=self.var_tool_index,
            font=FONT_VALUE, foreground="gray40",
        ).pack(side=tk.LEFT)
        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=10)

    def _build_control_panel(self, main_frame):
        """Suction toggle + per-colour light buttons. Initial button
        display is seeded from ``self.control_panel.build_initial_sync()``
        so the GUI doesn't show stale OFF state before the first DO
        feedback row arrives.
        """
        control_frame = ttk.LabelFrame(main_frame, text="Robot Direct Control", padding="10")
        control_frame.pack(fill=tk.X, pady=5)

        suction_row = ttk.Frame(control_frame)
        suction_row.pack(fill=tk.X, pady=5)
        ttk.Label(suction_row, text="Suction:", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        self.btn_suction = tk.Button(
            suction_row, text="OFF", font=FONT_VALUE, width=10, command=self.toggle_suction,
        )
        self.btn_suction.pack(side=tk.LEFT, padx=5)

        light_row = ttk.Frame(control_frame)
        light_row.pack(fill=tk.X, pady=10)
        ttk.Label(light_row, text="Lights:", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        self.btns_light = {}
        for name, _, _ in LIGHT_SPECS:
            btn = tk.Button(
                light_row, text=name, font=("Helvetica", 10, "bold"),
                width=8, bg=COLOR_OFF, command=lambda n=name: self.toggle_light(n),
            )
            btn.pack(side=tk.LEFT, padx=2)
            self.btns_light[name] = btn
        self._apply_control_panel_sync(self.control_panel.build_initial_sync())
        ttk.Separator(main_frame, orient="horizontal").pack(fill="x", pady=10)

    def _build_execution_metrics(self, main_frame):
        """STATE / timer row + stats summary row populated by
        ``self.monitor.update`` each tick.
        """
        metrics_frame = ttk.LabelFrame(main_frame, text="Execution Metrics", padding="10")
        metrics_frame.pack(fill=tk.X, pady=5)

        row1 = ttk.Frame(metrics_frame)
        row1.pack(fill=tk.X)
        self.var_status = tk.StringVar(value="IDLE")
        self.lbl_status = ttk.Label(
            row1, textvariable=self.var_status,
            font=("Helvetica", 12, "bold"), foreground="gray",
        )
        self.lbl_status.pack(side=tk.LEFT)
        self.var_timer = tk.StringVar(value="0.00s")
        self.lbl_timer = ttk.Label(
            row1, textvariable=self.var_timer, font=FONT_BIG_VALUE, foreground="black",
        )
        self.lbl_timer.pack(side=tk.RIGHT)

        row2 = ttk.Frame(metrics_frame)
        row2.pack(fill=tk.X, pady=5)
        self.var_stats = tk.StringVar(value="Avg: 0.00s | Min: 0.00s | Max: 0.00s")
        ttk.Label(row2, textvariable=self.var_stats, font=FONT_STATS).pack(anchor=tk.E)

    def _build_status_bar(self, main_frame):
        """Bottom status bar: latency text, MODE label, ERR code, and
        DO hex readout. ``update_gui`` writes each from the
        ``build_status_display_state`` helper.
        """
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=10)

        self.var_latency = tk.StringVar(value="Waiting for data...")
        ttk.Label(status_frame, textvariable=self.var_latency, font=FONT_LATENCY).pack(side=tk.LEFT)

        self.var_mode = tk.StringVar(value="MODE: -")
        tk.Label(
            status_frame, textvariable=self.var_mode,
            font=("Arial", 10), bg="#f0f0f0",
        ).pack(side=tk.RIGHT, padx=10)

        self.var_error = tk.StringVar(value="ERR: 00")
        self.lbl_error = tk.Label(
            status_frame, textvariable=self.var_error,
            font=("Arial", 10, "bold"), bg="#f0f0f0", fg="red",
        )
        self.lbl_error.pack(side=tk.RIGHT, padx=5)

        self.var_do_hex = tk.StringVar(value="DO: 0x0000")
        ttk.Label(
            status_frame, textvariable=self.var_do_hex,
            font=FONT_LATENCY, foreground="gray",
        ).pack(anchor=tk.W)

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
        """20 Hz Tk refresh tick. Pulls a fresh telemetry snapshot,
        forwards it to each ``_refresh_*`` helper, logs to CSV, then
        schedules the next tick.
        """
        telemetry = self.node.telemetry
        tgt = telemetry.latest_target_joints
        act = telemetry.latest_actual_joints
        joint_rows, total_diff = build_joint_display_rows(tgt, act)
        cartesian_state = build_cartesian_display_state(telemetry)
        status_state = build_status_display_state(telemetry, self.error_decoder)

        self._refresh_joint_table(joint_rows)
        self._refresh_system_info(status_state)
        self._refresh_control_panel_sync(telemetry)
        self._refresh_cartesian_table(cartesian_state)
        self._refresh_execution_metrics(total_diff)
        self._refresh_status_bar(status_state)

        self.manual_logger.log_sample(
            target_xyz=cartesian_state["unity_xyz"],
            actual_xyz=cartesian_state["tcp_xyz"],
            target_joints=tgt,
            actual_joints=act,
            total_diff=total_diff,
        )
        # Session Logger runs in its own background thread internally.
        self.root.after(50, self.update_gui)  # 20 Hz cadence

    def _refresh_joint_table(self, joint_rows):
        """Write target / actual / diff text into the joint table and
        recolour the diff label per row (green/yellow/red based on
        the build_joint_display_rows threshold).
        """
        for i, row in enumerate(joint_rows):
            self.vars_target[i].set(row["target"])
            self.vars_actual[i].set(row["actual"])
            self.vars_diff[i].set(row["diff"])
            self.lbls_diff[i].configure(foreground=row["color"])

    def _refresh_system_info(self, status_state):
        """MODE label + ERR readout (text + colour)."""
        self.var_mode.set(status_state["mode_text"])
        self.var_error.set(status_state["error_text"])
        self.lbl_error.configure(fg=status_state["error_color"])

    def _refresh_control_panel_sync(self, telemetry):
        """Push the latest DO status into the suction + light buttons
        so the GUI mirrors the actual robot state instead of the last
        operator click.
        """
        self._apply_control_panel_sync(
            self.control_panel.sync_from_do_status(telemetry.latest_do_status)
        )

    def _refresh_cartesian_table(self, cartesian_state):
        """Write the four Cartesian columns (Unity FK / Flange / TCP /
        tool-delta) and the active-tool index label.
        """
        xyz_unity = cartesian_state["unity_xyz"]
        xyz_flange = cartesian_state["flange_xyz"]
        xyz_tcp = cartesian_state["tcp_xyz"]
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{xyz_unity[i]:.1f}")
            self.vars_xyz_flange[i].set(f"{xyz_flange[i]:.1f}")
            self.vars_xyz_act[i].set(f"{xyz_tcp[i]:.1f}")
            self.vars_xyz_tool[i].set(f"{cartesian_state['tool_delta'][i]:+.1f}")
        self.var_tool_index.set(cartesian_state["tool_index_text"])

    def _refresh_execution_metrics(self, total_diff):
        """Push ``total_diff`` into the ExecutionMonitor, then mirror
        its derived state (MOVING / ARRIVED / IDLE) into the status
        label + timer colour, plus the rolling Avg/Min/Max summary.
        """
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
        self.var_stats.set(
            f"Avg: {avg_t:.2f}s | Min: {min_t:.2f}s | Max: {max_t:.2f}s | "
            f"Count: {len(self.monitor.durations)}"
        )

    def _refresh_status_bar(self, status_state):
        """Bottom-bar latency text + DO hex readout."""
        self.var_latency.set(status_state["latency_text"])
        self.var_do_hex.set(status_state["do_hex_text"])


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
