#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
⌨️ Interactive Command Handler
รับคำสั่งจาก Keyboard แบบ Real-time

ใช้งาน:
    handler = InteractiveCommandHandler(connection, logger, stop_event)
    handler.start()
"""

import threading

class InteractiveCommandHandler:
    def __init__(self, robot_connection, logger, stop_event):
        self.connection = robot_connection
        self.logger = logger
        self.stop_event = stop_event
        self.thread = None
        
        # คำสั่งที่รองรับ
        self.commands = {
            'e': ('EnableRobot()', '✅ Enable command sent'),
            'd': ('DisableRobot()', '🔴 Disable command sent'),
            'c': ('ClearError()', '🧹 Clear error command sent'),
            'r': ('ResetRobot()', '🔄 Reset command sent'),
            's': ('EmergencyStop()', '🚨 EMERGENCY STOP sent'),
            'p': ('Pause()', '⏸️  Pause command sent'),
            'u': ('Continue()', '▶️  Continue command sent'),
        }
    
    def start(self):
        """เริ่มต้น thread รับคำสั่ง"""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        self.logger.info("🎮 Interactive mode ready (type command + Enter)")
        self._print_help()
    
    def _print_help(self):
        """แสดงคำสั่งที่ใช้ได้"""
        print("\n💡 Available commands:")
        print("   'e' = Enable Robot")
        print("   'd' = Disable Robot")
        print("   'c' = Clear Error")
        print("   'r' = Reset Robot")
        print("   's' = Emergency Stop")
        print("   'p' = Pause")
        print("   'u' = Continue (Unpause)")
        print("   'hz <num>' = Set send frequency for Raw Data mode (e.g., 'hz 20')")
        print("   'q' = Quit")
        print()
    
    def _run(self):
        """Main loop รับคำสั่ง"""
        while not self.stop_event.is_set():
            try:
                cmd = input().strip().lower()
                
                if cmd == 'q':
                    self.logger.info("👋 Quit requested")
                    self.stop_event.set()
                    break
                
                elif cmd in self.commands:
                    robot_cmd, msg = self.commands[cmd]
                    success = self.connection.send_dashboard_cmd(robot_cmd)
                    if success:
                        if cmd == 's':  # Emergency stop is warning
                            self.logger.warn(msg)
                        else:
                            self.logger.info(msg)
                    else:
                        self.logger.error(f"❌ Failed to send command: {robot_cmd}")
                
                elif cmd.startswith('hz'):
                    try:
                        parts = cmd.split()
                        if len(parts) >= 2:
                            new_hz = int(parts[1])
                            if new_hz > 0:
                                import mg400_controller.common.config.robot_config as cfg
                                cfg.RAW_HZ = new_hz
                                self.logger.info(f"⚙️ Raw Data Mode Hz set to: {new_hz} Hz")
                            else:
                                self.logger.warn("Hz must be > 0")
                        else:
                            self.logger.warn("Usage: hz <num>")
                    except ValueError:
                        self.logger.warn("Invalid Hz value. Must be an integer.")
                
                elif cmd:
                    self.logger.warn(f"❓ Unknown command: '{cmd}'")
                    self._print_help()
                
            except Exception as e:
                self.logger.error(f"Interactive command error: {e}")
    
    def stop(self):
        """หยุด thread"""
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)