#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Advanced Error Handler for MG400 Robot

Uses Dobot's GetError() API to provide detailed error information:
- English error descriptions
- Solution suggestions
- Error history tracking
- Timestamp logging

Based on: Dobot_TCP_IP_Python_V4/dobot_api.py GetError() function
"""

import time
from typing import List, Dict, Optional


class ErrorHandler:
    """
    Advanced error handler using Dobot's GetError() API
    
    Provides detailed error information including:
    - Error ID and level
    - Description and solution suggestions
    - Timestamp for each error
    - Error history tracking
    """
    
    def __init__(self, connection, logger):
        """
        Initialize error handler
        
        Args:
            connection: RobotConnection instance with dashboard access
            logger: ROS logger instance
        """
        self.connection = connection
        self.logger = logger
        self.language = "en"  # Fixed to English
        self.error_history = []
        self.max_history = 100  # Keep last 100 errors
        
    def check_errors(self) -> List[Dict]:
        """
        Get current errors from robot using GetErrorID() API
        
        Returns:
            List of error dictionaries
        """
        try:
            # Call GetErrorID() API via Dashboard port
            result = self.connection.send_and_wait("GetErrorID()")
            
            if not result or ',{' not in result:
                return []
            
            # Extract the JSON-like array block from "0,{[[...]]},GetErrorID();"
            start_idx = result.find('{')
            end_idx = result.find('}')
            
            if start_idx != -1 and end_idx != -1:
                content = result[start_idx+1:end_idx]
                
                # Extract all numbers from the arrays
                import re
                nums = re.findall(r'-?\d+', content)
                error_ids = [int(n) for n in nums if int(n) != 0]
                
                if not error_ids:
                    return []
                
                # We have errors. Use RobotErrorDecoder to get descriptions
                from mg400_controller.common.utils.error.error_decoder import RobotErrorDecoder
                decoder = RobotErrorDecoder()
                
                parsed_errors = []
                for eid in set(error_ids):
                    desc, cause, solution = decoder.decode_error(eid)
                    parsed_errors.append({
                        'id': eid,
                        'level': 2, # Assume error
                        'description': desc,
                        'cause': cause,
                        'solution': solution,
                        'mode': 9,
                        'date': '',
                        'time': ''
                    })
                return parsed_errors
                
            return []
                
        except Exception as e:
            self.logger.warn(f"GetErrorID() API call failed: {e}")
            return []
    
    def format_error_message(self, error: Dict) -> Dict:
        """
        Format error for logging/display
        
        Args:
            error: Raw error dict from GetError()
            
        Returns:
            Formatted error dict with all fields
        """
        return {
            'id': error.get('id', 0),
            'level': error.get('level', 0),
            'description': error.get('description', 'Unknown error'),
            'cause': error.get('cause', ''),
            'solution': error.get('solution', 'No solution available'),
            'mode': error.get('mode', 'Unknown mode'),
            'timestamp': f"{error.get('date', '')} {error.get('time', '')}".strip()
        }
    
    def log_errors(self, errors: List[Dict]) -> None:
        """
        Log errors with details to ROS logger
        
        Args:
            errors: List of error dicts from check_errors()
        """
        for err in errors:
            formatted = self.format_error_message(err)
            
            # Log to ROS
            severity_emoji = {
                0: "ℹ️",  # Info
                1: "⚠️",  # Warning
                2: "🔴", # Error
                3: "🚨"  # Critical
            }.get(formatted['level'], "🔴")
            
            self.logger.error(
                f"{severity_emoji} Robot Error {formatted['id']} (Level {formatted['level']}):\n"
                f"   {formatted['description']}\n"
                f"   💡 Solution: {formatted['solution']}\n"
                f"   📅 {formatted['timestamp']}"
            )
            
            # Add to history
            self.error_history.append({
                **formatted,
                'logged_at': time.time()
            })
            
            # Trim history if too long
            if len(self.error_history) > self.max_history:
                self.error_history = self.error_history[-self.max_history:]
    
    
    
    def get_highest_severity(self, errors: List[Dict]) -> int:
        """
        Get highest severity level from error list
        
        Args:
            errors: List of error dicts
            
        Returns:
            Highest severity level (0-3)
        """
        if not errors:
            return 0
        return max(err.get('level', 0) for err in errors)
