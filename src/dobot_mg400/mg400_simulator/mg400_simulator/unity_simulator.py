#!/usr/bin/env python3
"""
mg400_vr_ws/src/mg400_vr_simulator/mg400_vr_simulator/unity_simulator.py
Unity Simulator - จำลองการส่งข้อมูลจาก Unity VR (GUI Included)
รองรับ 5 โหมด:
1. Sine wave - ทดสอบการเคลื่อนไหวพื้นฐาน
2. Circle - ทดสอบการประสานงานหลาย joint
3. Random - ทดสอบความเสถียร
4. Manual - ควบคุมผ่าน Slider แบบ Real-time
5. Manual Step - (NEW!) ควบคุมผ่าน Slider แบบกดปุ่มส่งทีเดียว
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import random
import tkinter as tk  # GUI Library มาตรฐานของ Python

class UnitySimulator(Node):
    def __init__(self):
        super().__init__('unity_simulator')
        
        # === 🆕 Terminal Input เลือกโหมด ===
        print("\nSelect Unity Simulator Mode:")
        print("1 = Sine wave")
        print("2 = Circle")
        print("3 = Random")
        print("4 = Manual (Continuous Stream)")
        print("5 = Manual Step (Move sliders then press Send)")
        choice = input("> ")

        if choice == '1':   self.mode = 'sine'
        elif choice == '2': self.mode = 'circle'
        elif choice == '3': self.mode = 'random'
        elif choice == '4': self.mode = 'manual'
        elif choice == '5': self.mode = 'manual_step'
        else:
            print("Invalid input, defaulting to 'manual_step'")
            self.mode = 'manual_step'

        # === Parameters ===
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('amplitude', 0.5)
        
        self.rate = self.get_parameter('publish_rate').value
        self.amplitude = self.get_parameter('amplitude').value
        
        # === Publisher ===
        self.publisher_ = self.create_publisher(
            JointState,
            '/unity/joint_cmd',
            10
        )

        # === GUI Setup (เฉพาะโหมด Manual / Manual Step) ===
        self.tk_root = None
        self.sliders = []
        self.trigger_send = False # 🆕 ธงเอาไว้เช็คว่ากดปุ่มหรือยัง
        
        if self.mode in ['manual', 'manual_step']:
            self.init_gui()

        # === Timer ===
        self.create_timer(1.0 / self.rate, self.publish_callback)
        
        # === State ===
        self.time = 0.0
        
        self.get_logger().info(f'🎮 Unity Simulator Started!')
        self.get_logger().info(f'   Mode: {self.mode}')

    def init_gui(self):
        """สร้างหน้าต่าง Slider"""
        self.tk_root = tk.Tk()
        mode_title = "VR Manual (Step)" if self.mode == 'manual_step' else "VR Manual (Stream)"
        self.tk_root.title(mode_title)
        self.tk_root.geometry("300x450") # ปรับความสูงเพิ่มนิดนึงเพื่อวางปุ่ม
        
        # ป้ายกำกับ
        tk.Label(self.tk_root, text="Joint Control (Degrees)", font=("Arial", 12, "bold")).pack(pady=10)

        joint_names = ["J1 (Base)", "J2 (Rear)", "J3 (Fore)", "J5 (Rotation)"]
        limits = [(-160, 160), (-25, 85), (-25, 105), (-360, 360)] # Limit คร่าวๆ ของ MG400

        for i, name in enumerate(joint_names):
            frame = tk.Frame(self.tk_root)
            frame.pack(pady=5)
            
            tk.Label(frame, text=name).pack(anchor="w")
            
            # สร้าง Slider (Scale)
            scale = tk.Scale(frame, from_=limits[i][0], to=limits[i][1], orient=tk.HORIZONTAL, length=250)
            scale.set(0) # เริ่มต้นที่ 0 องศา
            scale.pack()
            
            self.sliders.append(scale)

        # 🆕 เพิ่มปุ่ม SEND เฉพาะโหมดที่ 5
        if self.mode == 'manual_step':
            btn = tk.Button(self.tk_root, text="SEND CMD", font=("Arial", 12, "bold"), bg="#4CAF50", fg="black", command=self.on_send_click)
            btn.pack(pady=20, ipadx=20, ipady=10)

    # 🆕 ฟังก์ชันเมื่อกดปุ่ม
    def on_send_click(self):
        self.trigger_send = True

    def publish_callback(self):
        """ลูปหลักสำหรับส่งข้อมูล"""
        self.time += (1.0 / self.rate)
        
        positions = [0.0, 0.0, 0.0, 0.0]

        # === LOGIC เลือกโหมด ===
        if self.mode == 'manual':
            if self.tk_root:
                self.tk_root.update()
                positions = [math.radians(s.get()) for s in self.sliders]
                
        # 🆕 LOGIC สำหรับโหมดส่งเมื่อกดปุ่ม
        elif self.mode == 'manual_step':
            if self.tk_root:
                self.tk_root.update() # ต้องอัปเดตหน้าต่างเสมอ ไม่งั้นค้าง
                
                # ถ้ายังไม่ได้กดปุ่ม ให้ Return ออกไปเลย ไม่ต้องส่ง Message
                if not self.trigger_send:
                    return 
                
                # ถ้ากดปุ่มแล้ว ให้อ่านค่า
                positions = [math.radians(s.get()) for s in self.sliders]
                self.trigger_send = False # รีเซ็ตธง รอการกดครั้งต่อไป

        elif self.mode == 'sine':
            positions = self.generate_sine_wave()
        elif self.mode == 'circle':
            positions = self.generate_circle()
        elif self.mode == 'random':
            positions = self.generate_random()
        
        # === สร้าง Message ===
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # ชื่อ Joint มาตรฐาน MG400
        msg.name = ['mg400_j1', 'mg400_j2_1', 'mg400_j3', 'mg400_j5']
        msg.position = positions
        
        self.publisher_.publish(msg)

    # --- Generator Functions ---
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except tk.TclError:
        pass # กรณีปิดหน้าต่าง GUI ก่อนปิด Node
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()