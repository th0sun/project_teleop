#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import rospkg

class RobotErrorDecoder:
    def __init__(self):
        self.alarm_controller = []
        self.alarm_servo = []
        self._load_alarm_files()

    def _load_alarm_files(self):
        """Load alarm JSON files from config directory"""
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('mg400_controller')
            config_path = os.path.join(package_path, 'mg400_controller', 'common', 'config')
            
            controller_file = os.path.join(config_path, 'alarmController.json')
            servo_file = os.path.join(config_path, 'alarmServo.json')

            if os.path.exists(controller_file):
                with open(controller_file, 'r', encoding='utf-8') as f:
                    self.alarm_controller = json.load(f)
            
            if os.path.exists(servo_file):
                with open(servo_file, 'r', encoding='utf-8') as f:
                    self.alarm_servo = json.load(f)
                    
        except Exception as e:
            print(f"Error loading alarm files: {e}")

    def decode_error(self, error_id):
        """
        Find error description by ID
        Returns: (Description_EN, Cause_EN, Solution_EN)
        """
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
