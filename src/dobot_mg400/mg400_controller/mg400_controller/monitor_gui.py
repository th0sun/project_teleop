#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🤖 MG400 Joint Monitor GUI
Simple GUI to visualize robot joint angles in real-time.
Now includes Target vs Actual comparison, Latency monitoring, Execution Metrics, and End Effector (XYZ) Monitoring.
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

# Import Configuration
from mg400_controller.common.config.motion_config import UNITY_TOPIC

# Configuration
ACTUAL_TOPIC_NAME = "/joint_states"
TARGET_TOPIC_NAME = UNITY_TOPIC
TOOL_ACTUAL_TOPIC = "/mg400/tool_vector_actual"
TOOL_TARGET_TOPIC = "/mg400/tool_vector_target"

# Fonts
FONT_HEADER = ("Helvetica", 14, "bold")
FONT_LABEL = ("Helvetica", 12)
FONT_VALUE = ("Helvetica", 12, "bold")
FONT_LATENCY = ("Helvetica", 10)
FONT_BIG_VALUE = ("Helvetica", 24, "bold")
FONT_STATS = ("Helvetica", 11)

# Motion Detection Thresholds
START_THRESHOLD = 2.0  # degrees (Start timer if error > this)
STOP_THRESHOLD = 0.5   # degrees (Stop timer if error < this)

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
            # Trigger Motion Start
            if total_error > START_THRESHOLD:
                self.state = "MOVING"
                self.start_time = now
                return "STARTED"
                
        elif self.state == "MOVING":
            # Trigger Motion End
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
        
        # Subscription for Actual Robot State
        self.sub_actual = self.create_subscription(
            JointState,
            ACTUAL_TOPIC_NAME,
            self.listener_callback_actual,
            10
        )
        
        # Subscription for Target Command (from Unity/VR)
        self.sub_target = self.create_subscription(
            JointState,
            TARGET_TOPIC_NAME,
            self.listener_callback_target,
            10
        )
        
        # Tool Vectors
        self.sub_tool_actual = self.create_subscription(
            Float64MultiArray,
            TOOL_ACTUAL_TOPIC,
            self.listener_callback_tool_actual,
            10
        )
        
        self.sub_tool_target = self.create_subscription(
            Float64MultiArray,
            TOOL_TARGET_TOPIC,
            self.listener_callback_tool_target,
            10
        )

        self.latest_actual_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_target_joints = [0.0, 0.0, 0.0, 0.0]
        
        self.latest_tool_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_tool_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.last_target_time = 0.0
        self.last_actual_time = 0.0

        self.get_logger().info(f"Subscribed to {ACTUAL_TOPIC_NAME} and {TARGET_TOPIC_NAME}")

    def listener_callback_actual(self, msg):
        if len(msg.position) >= 9:
            q_rad = [
                msg.position[0], # J1
                msg.position[1], # J2
                msg.position[3], # J3
                msg.position[8]  # J4
            ]
            self.latest_actual_joints = list(np.degrees(q_rad))
            self.last_actual_time = time.time()

    def listener_callback_target(self, msg):
        if len(msg.position) >= 4:
            q_rad = msg.position[:4]
            self.latest_target_joints = list(np.degrees(q_rad))
            self.last_target_time = time.time() # Update receive time
            
    def listener_callback_tool_actual(self, msg):
        if len(msg.data) >= 6:
            self.latest_tool_actual = list(msg.data)
            
    def listener_callback_tool_target(self, msg):
        if len(msg.data) >= 6:
            self.latest_tool_target = list(msg.data)

class MonitorGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.monitor = ExecutionMonitor()
        
        self.root.title("MG400 Extended Monitor")
        self.root.geometry("600x750") # Increased height for XYZ
        self.root.configure(bg="#f0f0f0")

        # Main Container
        main_frame = ttk.Frame(root, padding="20")
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
        xyz_header = ttk.Frame(main_frame)
        xyz_header.pack(fill=tk.X, pady=5)
        ttk.Label(xyz_header, text="Cartesian Coordinates (End Effector)", font=("Helvetica", 12, "bold")).pack(anchor=tk.W)

        # XYZ Headers
        header_xyz = ttk.Frame(main_frame)
        header_xyz.pack(fill=tk.X, pady=2)
        ttk.Label(header_xyz, text="Axis", font=FONT_LABEL, width=10).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Target (mm)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Actual (mm)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)
        ttk.Label(header_xyz, text="Diff (mm)", font=FONT_LABEL, width=15).pack(side=tk.LEFT)

        self.vars_xyz_tgt = []
        self.vars_xyz_act = []
        self.vars_xyz_diff = []
        self.lbls_xyz_diff = []

        axes = ["X", "Y", "Z"]
        for i, name in enumerate(axes):
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

        ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=15)

        # --- EXECUTION METRICS ---
        metrics_frame = ttk.LabelFrame(main_frame, text="Execution Metrics", padding="10")
        metrics_frame.pack(fill=tk.X, pady=5)
        
        # Row 1: Status & Timer
        row1 = ttk.Frame(metrics_frame)
        row1.pack(fill=tk.X)
        
        self.var_status = tk.StringVar(value="IDLE")
        self.lbl_status = ttk.Label(row1, textvariable=self.var_status, font=("Helvetica", 12, "bold"), foreground="gray")
        self.lbl_status.pack(side=tk.LEFT)
        
        self.var_timer = tk.StringVar(value="0.00s")
        self.lbl_timer = ttk.Label(row1, textvariable=self.var_timer, font=FONT_BIG_VALUE, foreground="black")
        self.lbl_timer.pack(side=tk.RIGHT)
        
        # Row 2: Stats
        row2 = ttk.Frame(metrics_frame)
        row2.pack(fill=tk.X, pady=5)
        
        self.var_stats = tk.StringVar(value="Avg: 0.00s | Min: 0.00s | Max: 0.00s")
        ttk.Label(row2, textvariable=self.var_stats, font=FONT_STATS).pack(anchor=tk.E)

        # --- LATENCY BAR ---
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=10)
        
        self.var_latency = tk.StringVar(value="Waiting for data...")
        self.lbl_latency = ttk.Label(status_frame, textvariable=self.var_latency, font=FONT_LATENCY)
        self.lbl_latency.pack(anchor=tk.W)

        # Start Update Loop
        self.update_gui()

    def toggle_logging(self):
        self.is_logging = not self.is_logging
        if self.is_logging:
            self.btn_log.config(text="⏹ Stop Logging")
            # Create files with timestamp to avoid overwhelming one file
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.fn_target = f"teleop_target_{timestamp}.csv"
            self.fn_actual = f"teleop_actual_{timestamp}.csv"
            
            with open(self.fn_target, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "X", "Y", "Z", "Reach", "J1", "J2", "J3", "J4", "DiffTotal"])
            with open(self.fn_actual, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "X", "Y", "Z", "Reach", "J1", "J2", "J3", "J4"])
                
            self.log_start_time = time.time()
            self.node.get_logger().info(f"Started logging to {self.fn_target} and {self.fn_actual}")
        else:
            self.btn_log.config(text="▶ Start Logging")
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
                self.lbls_diff[i].configure(foreground="red")
            elif abs(diff) > 0.5:
                self.lbls_diff[i].configure(foreground="orange")
            else:
                self.lbls_diff[i].configure(foreground="green")

        # --- Update Cartesian Data (Robot Feedback) ---
        # Use values directly from robot controller (via FeedbackHandler)
        xyz_tgt = self.node.latest_tool_target[:3]
        xyz_act = self.node.latest_tool_actual[:3]
        
        for i in range(3):
            self.vars_xyz_tgt[i].set(f"{xyz_tgt[i]:.1f}")
            self.vars_xyz_act[i].set(f"{xyz_act[i]:.1f}")
            
            diff = xyz_act[i] - xyz_tgt[i]
            self.vars_xyz_diff[i].set(f"{diff:+.1f}")
            
            if abs(diff) > 10.0: # 1cm error
                self.lbls_xyz_diff[i].configure(foreground="red")
            elif abs(diff) > 2.0: # 2mm error
                self.lbls_xyz_diff[i].configure(foreground="orange")
            else:
                self.lbls_xyz_diff[i].configure(foreground="green")

        # --- Execution Monitor Update ---
        status = self.monitor.update(total_diff)
        
        if self.monitor.state == "MOVING":
            self.var_status.set("MOVING...")
            self.lbl_status.configure(foreground="red")
            # Real-time timer
            current_duration = time.time() - self.monitor.start_time
            self.var_timer.set(f"{current_duration:.2f}s")
            self.lbl_timer.configure(foreground="red")
            
        elif self.monitor.state == "ARRIVED":
            self.var_status.set("ARRIVED")
            self.lbl_status.configure(foreground="green")
            # Hold last duration
            self.var_timer.set(f"{self.monitor.last_duration:.2f}s")
            self.lbl_timer.configure(foreground="green")
            
        else:
            self.var_status.set("IDLE")
            self.lbl_status.configure(foreground="gray")
            # Keep showing last duration/stats
        
        # Update Stats Text
        avg_t, min_t, max_t = self.monitor.get_stats()
        self.var_stats.set(f"Avg: {avg_t:.2f}s | Min: {min_t:.2f}s | Max: {max_t:.2f}s | Count: {len(self.monitor.durations)}")

        # --- Latency Logic ---
        now = time.time()
        time_since_target = now - self.node.last_target_time
        time_since_actual = now - self.node.last_actual_time
        
        if self.node.last_target_time == 0:
             self.var_latency.set("Status: No Target Received")
        else:
            status_text = f"Cmd Age: {time_since_target*1000:.0f}ms | Feed Age: {time_since_actual*1000:.0f}ms"
            self.var_latency.set(status_text)
            
        # --- Logging Data ---
        if self.is_logging:
            t = time.time() - self.log_start_time
            
            # Target Write
            reach_tgt = math.sqrt(xyz_tgt[0]**2 + xyz_tgt[1]**2)
            with open(self.fn_target, 'a', newline='') as f:
                csv.writer(f).writerow([f"{t:.3f}", f"{xyz_tgt[0]:.3f}", f"{xyz_tgt[1]:.3f}", f"{xyz_tgt[2]:.3f}", f"{reach_tgt:.3f}", f"{tgt[0]:.3f}", f"{tgt[1]:.3f}", f"{tgt[2]:.3f}", f"{tgt[3]:.3f}", f"{total_diff:.3f}"])
                
            # Actual Write 
            reach_act = math.sqrt(xyz_act[0]**2 + xyz_act[1]**2)
            with open(self.fn_actual, 'a', newline='') as f:
                 csv.writer(f).writerow([f"{t:.3f}", f"{xyz_act[0]:.3f}", f"{xyz_act[1]:.3f}", f"{xyz_act[2]:.3f}", f"{reach_act:.3f}", f"{act[0]:.3f}", f"{act[1]:.3f}", f"{act[2]:.3f}", f"{act[3]:.3f}"])


        # Schedule next update (20Hz for smoother timer)
        self.root.after(50, self.update_gui)

def main():
    rclpy.init()
    node = JointMonitorNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    root = tk.Tk()
    gui = MonitorGUI(root, node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
