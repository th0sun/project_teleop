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

from mg400_protocol.dashboard import (
    clear_error,
    continue_,
    disable_robot,
    emergency_stop,
    enable_robot,
    pause,
    reset_robot,
)


class InteractiveCommandHandler:
    def __init__(self, robot_connection, logger, stop_event):
        self.connection = robot_connection
        self.logger = logger
        self.stop_event = stop_event
        self.thread = None

        # Vendor-specific dashboard commands rendered through mg400_protocol
        # so this file does not own MG400 ASCII strings.
        self.commands = {
            'e': (enable_robot().render(), '✅ Enable command sent'),
            'd': (disable_robot().render(), '🔴 Disable command sent'),
            'c': (clear_error().render(), '🧹 Clear error command sent'),
            'r': (reset_robot().render(), '🔄 Reset command sent'),
            's': (emergency_stop().render(), '🚨 EMERGENCY STOP sent'),
            'p': (pause().render(), '⏸️  Pause command sent'),
            'u': (continue_().render(), '▶️  Continue command sent'),
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
