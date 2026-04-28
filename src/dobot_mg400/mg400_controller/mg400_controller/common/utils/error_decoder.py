#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from pathlib import Path

class RobotErrorDecoder:
    def __init__(self):
        self.alarm_controller = []
        self.alarm_servo = []
        self._load_alarm_files()

    def _load_alarm_files(self):
        """Load alarm JSON files from config directory (ROS2 compatible)"""
        try:
            config_path = None

            # ROS2 share directory path when installed by colcon.
            try:
                from ament_index_python.packages import get_package_share_directory

                package_path = get_package_share_directory('mg400_controller')
                candidate = Path(package_path) / 'common' / 'config'
                if candidate.exists():
                    config_path = candidate
            except Exception:
                pass

            # Source-tree fallback for tests/dev runs without installed package data.
            if config_path is None:
                config_path = Path(__file__).resolve().parents[1] / 'config'
            
            controller_file = config_path / 'alarmController.json'
            servo_file = config_path / 'alarmServo.json'

            if controller_file.exists():
                with controller_file.open('r', encoding='utf-8') as f:
                    self.alarm_controller = json.load(f)
            else:
                print(f"File not found: {controller_file}")
            
            if servo_file.exists():
                with servo_file.open('r', encoding='utf-8') as f:
                    self.alarm_servo = json.load(f)
            else:
                print(f"File not found: {servo_file}")
                    
        except Exception as e:
            print(f"Error loading alarm files: {e}")

    def decode_error(self, error_id):
        """
        Find error description by ID
        Returns: (Description_EN, Cause_EN, Solution_EN)
        """
        if int(error_id) == -2:
            return (
                "Collision detection alarm",
                "Robot reported collision detection through GetErrorID().",
                "Check the robot path and surrounding fixture, then clear the alarm after the collision cause is removed.",
            )

        # Check Controller Alarms
        for alarm in self.alarm_controller:
            if alarm['id'] == error_id:
                en_info = alarm.get('en', {})
                return (
                    en_info.get('description', 'Unknown Error'),
                    en_info.get('cause', ''),
                    en_info.get('solution', '')
                )

        # Check Servo Alarms
        for alarm in self.alarm_servo:
            if alarm['id'] == error_id:
                en_info = alarm.get('en', {})
                return (
                    en_info.get('description', 'Unknown Servo Error'),
                    en_info.get('cause', ''),
                    en_info.get('solution', '')
                )

        return (f"Unknown Error ID: {error_id}", "", "")

    def parse_error_status_binary(self, error_status_bytes):
        """
        Parse the ErrorStatus byte array (if raw access is available)
        Start from some simpler logic if we just have the ID from GetError()
        """
        pass
