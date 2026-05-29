#!/usr/bin/env python3
"""
mg400_vr_ws/src/mg400_vr_simulator/mg400_vr_simulator/unity_simulator.py
Unity Simulator - จำลองการส่งข้อมูลจาก Unity VR (GUI Included)
รองรับ 6 โหมด:
1. Sine wave - ทดสอบการเคลื่อนไหวพื้นฐาน
2. Circle - ทดสอบการประสานงานหลาย joint
3. Random - ทดสอบความเสถียร
4. Manual - ควบคุมผ่าน Slider แบบ Real-time
5. Manual Step - ควบคุมผ่าน Slider แบบกดปุ่มส่งทีเดียว
6. Mouse 3D - ลาก Target ball พร้อม IK และ Workspace Limits
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import random
import time
import threading
import tkinter as tk
import numpy as np

# Local copy of the joint limits the simulator uses; intentionally NOT
# imported from mg400_controller.common.config.robot_config so the
# simulator stays a leaf node with no circular config dependency.
JOINT_LIMITS = [(-160, 160), (-25, 85), (-25, 105), (-360, 360)]
ELBOW_ANGLE_LIMIT = (-60, 60)

# Precision Clamping Constants (derived from limits)
# Use next representative float towards 0 to ensure strict inequality < limits
ELBOW_MIN_SAFE = np.nextafter(ELBOW_ANGLE_LIMIT[0], 0.0) 
ELBOW_MAX_SAFE = np.nextafter(ELBOW_ANGLE_LIMIT[1], 0.0)

class UnitySimulator(Node):
    _MODE_BY_CHOICE = {
        '1': 'sine',
        '2': 'circle',
        '3': 'random',
        '4': 'manual',
        '5': 'manual_step',
        '6': 'mouse_3d',
    }

    def __init__(self):
        super().__init__('unity_simulator')
        self.mode = self._prompt_mode()
        self._init_ros_interfaces()
        self._init_control_state()

        if self.mode in ('manual', 'manual_step'):
            self.init_gui()
        elif self.mode == 'mouse_3d':
            self.init_mouse_3d_gui()

        self.create_timer(1.0 / self.rate, self.publish_callback)
        self.time = 0.0
        self.get_logger().info('🎮 Unity Simulator Started!')
        self.get_logger().info(f'   Mode: {self.mode}')

    def _prompt_mode(self) -> str:
        """Ask the operator which simulation mode to run at the terminal.

        Falls back to ``manual_step`` on an unrecognised choice so the
        sim never hard-fails just because of a typo.
        """
        print("\nSelect Unity Simulator Mode:")
        print("1 = Sine wave")
        print("2 = Circle")
        print("3 = Random")
        print("4 = Manual (Continuous Stream)")
        print("5 = Manual Step (Move sliders then press Send)")
        print("6 = Mouse 3D (IK with Workspace Limits) ")
        choice = input("> ")
        mode = self._MODE_BY_CHOICE.get(choice)
        if mode is None:
            print("Invalid input, defaulting to 'manual_step'")
            return 'manual_step'
        return mode

    def _init_ros_interfaces(self):
        """Declare params (publish_rate, amplitude) + the
        ``/unity/joint_cmd`` publisher the timer streams to.
        """
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('amplitude', 0.5)
        self.rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value
        self.publisher_ = self.create_publisher(JointState, '/unity/joint_cmd', 10)

    def _init_control_state(self):
        """Initialise GUI handles, mouse-drag state, the direct
        per-joint angles for mouse_3d mode, and the keyboard-repeat
        bookkeeping. All modes share this state block; only the GUI
        builders wire the widgets that read it.
        """
        # GUI handles
        self.tk_root = None
        self.sliders = []
        self.trigger_send = False

        # Mouse control variables
        self.mouse_active = False
        self.last_mouse_x = 0
        self.last_mouse_y = 0

        # Direct joint control for mode 6 (mouse_3d)
        self.j1_angle = 0.0  # from mouse X
        self.j2_angle = 0.0  # from mouse Y (centered)
        self.j3_angle = 0.0  # from keyboard
        self.j4_angle = 0.0  # from keyboard

        # Keyboard repeat control
        self.keyboard_step = 1.0     # degrees per step
        self.key_last_seen = {}      # last press-event timestamp
        self.key_active_start = {}   # timestamp the key sequence began
        self.last_update_time = 0    # repeat-rate gate

    def init_gui(self):
        """สร้างหน้าต่าง Slider"""
        self.tk_root = tk.Tk()
        mode_title = "VR Manual (Step)" if self.mode == 'manual_step' else "VR Manual (Stream)"
        self.tk_root.title(mode_title)
        self.tk_root.geometry("620x650")
        
        tk.Label(self.tk_root, text="Joint Control (Degrees)", font=("Arial", 12, "bold")).pack(pady=10)

        joint_names = ["J1 (Base)", "J2 (Rear)", "J3 (Fore)", "J5 (Rotation)"]
        limits = [(-160, 160), (-25, 85), (-25, 105), (-360, 360)]

        for i, name in enumerate(joint_names):
            frame = tk.Frame(self.tk_root)
            frame.pack(pady=5)
            
            tk.Label(frame, text=name).pack(anchor="w")
            
            scale = tk.Scale(frame, from_=limits[i][0], to=limits[i][1], orient=tk.HORIZONTAL, length=250)
            scale.set(0)
            scale.pack()
            
            self.sliders.append(scale)
            
            if i == 1 or i == 2:
                scale.config(command=lambda val, idx=i: self.on_slider_change(idx, val))

        tk.Label(self.tk_root, text="─" * 60).pack(pady=5)
        tk.Label(self.tk_root, text="Joint Positions & Distance from Origin", font=("Arial", 10, "bold")).pack()
        
        self.distance_labels = []
        for _ in range(4):
            frame = tk.Frame(self.tk_root)
            frame.pack()
            label = tk.Label(frame, text="", font=("Courier", 9), justify="left")
            label.pack()
            self.distance_labels.append(label)

        if self.mode == 'manual_step':
            btn = tk.Button(self.tk_root, text="SEND CMD", font=("Arial", 12, "bold"), bg="#4CAF50", fg="black", command=self.on_send_click)
            btn.pack(pady=20, ipadx=20, ipady=10)
            
        # Update distance labels periodically
        self.tk_root.after(100, self.gui_update_loop)
    
    def gui_update_loop(self):
        if self.mode in ['manual', 'manual_step']:
            positions = [math.radians(s.get()) for s in self.sliders]
            self.update_distance_display(positions)
        elif self.mode == 'mouse_3d':
            self.update_joint_display()
            self.update_j3_j4_from_keyboard()
        self.tk_root.after(30, self.gui_update_loop)
    
    def init_mouse_3d_gui(self):
        """Initialize Direct Joint Control GUI"""
        self.tk_root = tk.Tk()
        self.tk_root.title("🎮 Direct Joint Control - Mouse + Keyboard")
        self.tk_root.geometry("900x650")
        
        # Bind keyboard events for J3, J4 control
        self.tk_root.bind('<KeyPress>', self.on_key_press)
        self.tk_root.bind('<KeyRelease>', self.on_key_release)
        
        # Make window stay on top and force focus
        self.tk_root.lift()
        self.tk_root.attributes('-topmost', True)
        self.tk_root.after(100, lambda: self.tk_root.attributes('-topmost', False))
        self.tk_root.focus_force()
        
        main_frame = tk.Frame(self.tk_root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Canvas for joint visualization
        canvas_frame = tk.LabelFrame(main_frame, text="Joint Control Workspace", font=("Arial", 11, "bold"))
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.canvas = tk.Canvas(canvas_frame, width=550, height=550, bg="white", highlightthickness=1)
        self.canvas.pack(padx=10, pady=10)
        
        # Bind mouse events for J1/J2 control
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<Enter>", self.on_mouse_enter)
        self.canvas.bind("<Leave>", self.on_mouse_leave)
        
        self.draw_joint_workspace()
        
        # Control Panel
        control_frame = tk.Frame(main_frame)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5)
        
        # Joint Angles Display
        joint_frame = tk.LabelFrame(control_frame, text="Joint Angles (Degrees)", font=("Arial", 10, "bold"))
        joint_frame.pack(fill=tk.X, pady=5)
        
        self.joint_label = tk.Label(joint_frame, text="J1: 0.0°\nJ2: 45.0°\nJ3: 0.0°\nJ4: 0.0°",
                                     font=("Courier", 12, "bold"), justify="left")
        self.joint_label.pack(padx=10, pady=10)
        
        # Joint Limits Info
        limits_frame = tk.LabelFrame(control_frame, text="Joint Limits", font=("Arial", 9, "bold"))
        limits_frame.pack(fill=tk.X, pady=5)
        
        limits_text = f"J1: {JOINT_LIMITS[0][0]}° to {JOINT_LIMITS[0][1]}°\n"
        limits_text += f"J2: {JOINT_LIMITS[1][0]}° to {JOINT_LIMITS[1][1]}°\n"
        limits_text += f"J3: {JOINT_LIMITS[2][0]}° to {JOINT_LIMITS[2][1]}°\n"
        limits_text += f"J4: {JOINT_LIMITS[3][0]}° to {JOINT_LIMITS[3][1]}°"
        
        tk.Label(limits_frame, text=limits_text, font=("Courier", 7), justify="left", fg="gray").pack(padx=5, pady=5)
        
        # Keyboard State Display
        keyboard_frame = tk.LabelFrame(control_frame, text="Keyboard Control", font=("Arial", 10, "bold"))
        keyboard_frame.pack(fill=tk.X, pady=5)
        
        self.keyboard_label = tk.Label(keyboard_frame, text="W/S: J3 | A/D: J4\nNo keys pressed", 
                                       font=("Courier", 9), justify="left")
        self.keyboard_label.pack(padx=10, pady=5)
        
        # Buttons
        btn_frame = tk.Frame(control_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        tk.Button(btn_frame, text="Reset", command=self.reset_joints, bg="#f44336", fg="white").pack(fill=tk.X, pady=2)
        tk.Button(btn_frame, text="Home", command=self.home_joints, bg="#4CAF50", fg="white").pack(fill=tk.X, pady=2)
       
         # Instructions
        inst_text = "\nControls:\n"
        inst_text += "• Move mouse: J1 (X) & J2 (Y)\n"
        inst_text += "• W/S keys: J3 ±0.5°\n"
        inst_text += "• A/D keys: J4 ±0.5°\n"
        inst_text += "• Hold keys for continuous\n"
        
        tk.Label(control_frame, text=inst_text, font=("Arial", 8), justify="left", fg="gray").pack(pady=10)
        
        self.update_joint_display()
    
    def draw_joint_workspace(self):
        """Draw simplified workspace for joint visualization"""
        self.canvas.delete("all")
        cx, cy = 275, 275
        
        # Draw center cross
        self.canvas.create_line(cx, 0, cx, 550, fill="lightgray", width=1)
        self.canvas.create_line(0, cy, 550, cy, fill="lightgray", width=1)
        
        # Draw circles for reference
        for r in [50, 100, 150, 200]:
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline="#e0e0e0", width=1)
        
        # Labels
        self.canvas.create_text(cx, 20, text="J2 (+)", fill="blue", font=("Arial", 10))
        self.canvas.create_text(cx, 530, text="J2 (-)", fill="blue", font=("Arial", 10))
        self.canvas.create_text(20, cy, text="J1 (-)", fill="red", font=("Arial", 10))
        self.canvas.create_text(530, cy, text="J1 (+)", fill="red", font=("Arial", 10))
        
        # Current position indicator
        self.position_dot = self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="green", outline="darkgreen", width=2)
    
    def on_mouse_move(self, event):
        """Update J1 and J2 based on mouse position with visual clamping and centered origins"""
        if not self.mouse_active:
            return
            
        canvas_size = 550.0
        
        # 1. Map Mouse -> J1 Angle (Symmetric -160 to 160)
        # Center X is 0 degrees
        j1_max = JOINT_LIMITS[0][1] # 160
        # Map 0..canvas_size to -160..160
        # Formula: (x / width - 0.5) * (2 * max)
        raW_j1 = ((event.x / canvas_size) - 0.5) * (2 * j1_max)
        self.j1_angle = np.clip(raW_j1, JOINT_LIMITS[0][0], JOINT_LIMITS[0][1])
        
        # 2. Map Mouse -> J2 Angle (Symmetric Visualization [-85, 85])
        j2_max_abs = max(abs(JOINT_LIMITS[1][0]), abs(JOINT_LIMITS[1][1])) # 85
        
        # Map Y (inverted)
        y_norm = 1.0 - (event.y / canvas_size)
        raw_j2 = (y_norm - 0.5) * (2 * j2_max_abs)
        
        # --- ELBOW CONSTRAINT FOR J2 ---
        j2_min_elbow = self.j3_angle - ELBOW_MAX_SAFE
        j2_max_elbow = self.j3_angle - ELBOW_MIN_SAFE
        
        final_j2_min = max(JOINT_LIMITS[1][0], j2_min_elbow)
        final_j2_max = min(JOINT_LIMITS[1][1], j2_max_elbow)
        
        self.j2_angle = np.clip(raw_j2, final_j2_min, final_j2_max)
        
        # 3. Valid X/Y from Clamped Angles (Visual Feedback)
        visual_x = ((self.j1_angle / (2 * j1_max)) + 0.5) * canvas_size
        j2_norm = (self.j2_angle / (2 * j2_max_abs)) + 0.5
        visual_y = (1.0 - j2_norm) * canvas_size
        
        self.canvas.coords(self.position_dot, visual_x-5, visual_y-5, visual_x+5, visual_y+5)
        
        self.update_joint_display()
    
    def on_mouse_enter(self, event):
        """Activate mouse control when entering canvas"""
        self.mouse_active = True
    
    def on_mouse_leave(self, event):
        """Deactivate mouse control when leaving canvas"""
        self.mouse_active = False
    
    
    def on_key_press(self, event):
        """Handle key press with watchdog logic"""
        # Use keysym for robustness (works better with layouts)
        key = event.keysym.lower()
        if key not in ['w', 'a', 's', 'd']:
            return
            
        current_time = time.time()
        
        # Check if this is a "New" press (gap > 200ms)
        if key not in self.key_last_seen or (current_time - self.key_last_seen.get(key, 0) > 0.2):
            self.key_active_start[key] = current_time
            # Immediate single step on fresh press
            self.process_single_step(key)
            self.update_keyboard_display(key)
            
        # Update watchdog timestamp
        self.key_last_seen[key] = current_time
    
    def on_key_release(self, event):
        # We rely on Watchdog timer, so explicit release is just for UI cleanup
        # But we don't trust it for logic to avoid "stuck" state if release is lost
        pass
    
    def update_keyboard_display(self, active_key=None):
        """Update keyboard state display"""
        if not hasattr(self, 'keyboard_label'):
            return
        if active_key:
            self.keyboard_label.config(text=f"W/S: J3 | A/D: J4\nActive: {active_key.upper()}", fg="green")
        else:
            self.keyboard_label.config(text="W/S: J3 | A/D: J4\nNo keys active", fg="gray")
    

    def process_single_step(self, key):
        """Process a single step movement for a key"""
        if key == 'w':
             self.j3_angle += self.keyboard_step
        elif key == 's':
             self.j3_angle -= self.keyboard_step
        elif key == 'd':
             self.j4_angle += self.keyboard_step
        elif key == 'a':
             self.j4_angle -= self.keyboard_step
             
        # Apply Hardware limits first
        self.j3_angle = np.clip(self.j3_angle, JOINT_LIMITS[2][0], JOINT_LIMITS[2][1])
        
        # --- ELBOW CONSTRAINT FOR J3 ---
        # Requirement: ELBOW_ANGLE_LIMIT[0] < (J3 - J2) < ELBOW_ANGLE_LIMIT[1]
        # Implies: J2 + ELBOW_MIN_SAFE < J3 < J2 + ELBOW_MAX_SAFE
        
        j3_min_elbow = self.j2_angle + ELBOW_MIN_SAFE
        j3_max_elbow = self.j2_angle + ELBOW_MAX_SAFE
        
        # Clamp J3 relative to current J2
        self.j3_angle = np.clip(self.j3_angle, j3_min_elbow, j3_max_elbow)
        self.j4_angle = np.clip(self.j4_angle, JOINT_LIMITS[3][0], JOINT_LIMITS[3][1])
        
    def update_j3_j4_from_keyboard(self):
        """Update J3 and J4 based on Watchdog timer"""
        current_time = time.time()
        
        # 1. Check Watchdog - Clear inactive keys
        active_keys = []
        for k in list(self.key_last_seen.keys()):
            if current_time - self.key_last_seen[k] > 0.15: # 150ms timeout (approx 6 frames at 60hz)
                # Key signal lost (released)
                del self.key_last_seen[k]
                if k in self.key_active_start:
                    del self.key_active_start[k]
            else:
                active_keys.append(k)
        
        if not active_keys:
            self.update_keyboard_display(None)
            return

        # 2. Continuous Movement Logic
        INITIAL_DELAY = 0.5  # 500ms hold before continuous
        REPEAT_RATE = 0.05   # 20Hz continuous speed (Faster)
        
        if current_time - self.last_update_time < REPEAT_RATE:
            return

        changed = False
        for k in active_keys:
            # Check duration
            duration = current_time - self.key_active_start.get(k, current_time)
            if duration > INITIAL_DELAY:
                self.process_single_step(k)
                changed = True
                self.update_keyboard_display(k)
        
        if changed:
            self.last_update_time = current_time
    

    
    def reset_joints(self):
        """Reset all joints to 0"""
        self.j1_angle = 0.0
        self.j2_angle = 0.0
        self.j3_angle = 0.0
        self.j4_angle = 0.0
        
        # Reset mouse position tracking if needed
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        
        self.update_joint_display()
        
        # Update canvas visual
        if hasattr(self, 'position_dot'):
            cx, cy = 275, 275
            # Reset dot to center/default
            self.canvas.coords(self.position_dot, cx-5, cy-5, cx+5, cy+5)
    
    def home_joints(self):
        """Move to home position"""
        self.j1_angle = 0.0
        self.j2_angle = 0.0  # Home for J2
        self.j3_angle = 0.0
        self.j4_angle = 0.0
        self.update_joint_display()
        
        # Update canvas visual
        if hasattr(self, 'position_dot'):
            cx, cy = 275, 275
            self.canvas.coords(self.position_dot, cx-5, cy-5, cx+5, cy+5)

    def update_joint_display(self):
        """Update joint angle display"""
        text = f"J1: {self.j1_angle:.1f}°\n"
        text += f"J2: {self.j2_angle:.1f}°\n"
        text += f"J3: {self.j3_angle:.1f}°\n"
        text += f"J4: {self.j4_angle:.1f}°"
        self.joint_label.config(text=text)
    

    
    def calculate_fk(self, q):
        """Forward Kinematics for MG400"""
        BASE_HEIGHT = 109.0
        LINK1 = np.array([43.0, 0.0, 115.0])
        LINK2 = np.array([0.0, 0.0, 175.0])
        LINK3 = np.array([175.0, 0.0, 0.0])
        LINK4 = np.array([66.0, 0.0, -57.0])
        
        j1, j2, j3, j4 = q
        
        j1_pos = np.array([0, 0, BASE_HEIGHT])
        j2_local = LINK1.copy()
        j2_pos = j1_pos + j2_local
        
        link2_rotated = np.array([
            LINK2[0] * np.cos(j2) + LINK2[2] * np.sin(j2),
            LINK2[1],
            -LINK2[0] * np.sin(j2) + LINK2[2] * np.cos(j2)
        ])
        j3_pos = j2_pos + link2_rotated
        
        combined_angle = j2 + j3
        link3_rotated = np.array([
            LINK3[0] * np.cos(combined_angle) + LINK3[2] * np.sin(combined_angle),
            LINK3[1],
            -LINK3[0] * np.sin(combined_angle) + LINK3[2] * np.cos(combined_angle)
        ])
        
        link4_rotated = np.array([
            LINK4[0] * np.cos(combined_angle) + LINK4[2] * np.sin(combined_angle),
            LINK4[1],
            -LINK4[0] * np.sin(combined_angle) + LINK4[2] * np.cos(combined_angle)
        ])
        
        ee_pos = j3_pos + link3_rotated + link4_rotated
        
        cos_j1 = np.cos(j1)
        sin_j1 = np.sin(j1)
        
        def rotate_z(pos):
            return np.array([
                pos[0] * cos_j1 - pos[1] * sin_j1,
                pos[0] * sin_j1 + pos[1] * cos_j1,
                pos[2]
            ])
        
        j2_pos = rotate_z(j2_pos)
        j3_pos = rotate_z(j3_pos)
        ee_pos = rotate_z(ee_pos)
        
        return [j1_pos, j2_pos, j3_pos, ee_pos]
    
    def update_distance_display(self, q):
        """Update coordinate labels"""
        positions_xyz = self.calculate_fk(q)
        
        names = ["J1 (Base)     ", "J2 (Shoulder) ", "J3 (Elbow)    ", "End-Effector  "]
        for i, (pos, label) in enumerate(zip(positions_xyz, self.distance_labels)):
            x, y, z = pos
            dist = np.linalg.norm(pos)
            label.config(text=f"{names[i]}: X={x:7.1f} Y={y:7.1f} Z={z:7.1f} | D={dist:7.1f} mm")

    def on_send_click(self):
        self.trigger_send = True
    
    def on_slider_change(self, joint_idx, value):
        """Check elbow constraint"""
        try:
            j2_deg = float(self.sliders[1].get())
            j3_deg = float(self.sliders[2].get())
            elbow_angle = j3_deg - j2_deg
            
            elbow_min, elbow_max = ELBOW_ANGLE_LIMIT
            
            if elbow_angle < elbow_min:
                if joint_idx == 2:
                    self.sliders[2].set(j2_deg + elbow_min)
                elif joint_idx == 1:
                    self.sliders[1].set(j3_deg - elbow_min)
            elif elbow_angle > elbow_max:
                if joint_idx == 2:
                    self.sliders[2].set(j2_deg + elbow_max)
                elif joint_idx == 1:
                    self.sliders[1].set(j3_deg - elbow_max)
        except (ValueError, tk.TclError):
            pass

    def publish_callback(self):
        """Main publish loop"""
        self.time += (1.0 / self.rate)
        
        positions = [0.0, 0.0, 0.0, 0.0]

        if self.mode == 'manual':
            if self.tk_root:
                positions = [math.radians(s.get()) for s in self.sliders]
                
        elif self.mode == 'manual_step':
            if self.tk_root:
                positions = [math.radians(s.get()) for s in self.sliders]
                if not self.trigger_send:
                    return
                self.trigger_send = False

        elif self.mode == 'sine':
            positions = self.generate_sine_wave()
        elif self.mode == 'circle':
            positions = self.generate_circle()
        elif self.mode == 'random':
            positions = self.generate_random()
        elif self.mode == 'mouse_3d':
            if self.tk_root:
                # Use all 4 joint angles directly (no IK)
                positions = [
                    math.radians(self.j1_angle),  # J1 from mouse
                    math.radians(self.j2_angle),  # J2 from mouse
                    math.radians(self.j3_angle),  # J3 from keyboard
                    math.radians(self.j4_angle)   # J4 from keyboard
                ]
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['mg400_j1', 'mg400_j2_1', 'mg400_j3', 'mg400_j5']
        msg.position = positions
        
        self.publisher_.publish(msg)

    def generate_sine_wave(self):
        return [
            self.amplitude * math.sin(self.time * 0.5),
            self.amplitude * math.sin(self.time * 1.0),
            self.amplitude * math.sin(self.time * 1.5),
            self.amplitude * math.sin(self.time * 0.8),
        ]

    def generate_circle(self):
        return [
            self.amplitude * math.cos(self.time),
            self.amplitude * math.sin(self.time),
            self.amplitude * 0.5 * math.sin(self.time * 2),
            self.amplitude * 0.3 * math.cos(self.time * 3),
        ]

    def generate_random(self):
        if not hasattr(self, 'random_positions'):
            self.random_positions = [0.0]*4
        for i in range(4):
            delta = random.uniform(-0.02, 0.02)
            self.random_positions[i] += delta
            self.random_positions[i] = max(-self.amplitude, min(self.amplitude, self.random_positions[i]))
        return self.random_positions

def main(args=None):
    rclpy.init(args=args)
    node = UnitySimulator()
    
    # Run ROS 2 spin in a separate thread so it doesn't block or get blocked by Tkinter
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        if node.tk_root:
            node.tk_root.mainloop()
        else:
            # If no GUI (e.g. sine/circle mode), just wait on the main thread
            while rclpy.ok():
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except tk.TclError:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
