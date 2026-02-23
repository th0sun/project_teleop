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
import json
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
        Get current errors from robot using GetError() API
        
        Returns:
            List of error dictionaries, each containing:
            - id: Error code
            - level: Severity level
            - description: Error description
            - solution: Suggested solution
            - mode: Robot mode when error occurred
            - date: Date of error
            - time: Time of error
        """
        try:
            # Call GetError() API via Dashboard port
            result = self.connection.send_and_wait(
                f"GetError(language=\"{self.language}\")"
            )
            
            if not result:
                return []
            
            # Parse response format: "0,{errMsg:[...]};" or "0,{};" (no errors)
            # Remove error code prefix
            if ',' in result:
                parts = result.split(',', 1)
                if len(parts) == 2:
                    json_part = parts[1].strip()
                    
                    # Remove trailing semicolon if present
                    if json_part.endswith(';'):
                        json_part = json_part[:-1]
                    
                    try:
                        data = json.loads(json_part)
                        return data.get('errMsg', [])
                    except json.JSONDecodeError:
                        # GetError might not be supported or empty response
                        return []
            
            return []
                
        except Exception as e:
            self.logger.warn(f"GetError() API call failed: {e}")
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
    
    def get_error_history(self, count: Optional[int] = None) -> List[Dict]:
        """
        Get recent error history
        
        Args:
            count: Number of recent errors to return (None = all)
            
        Returns:
            List of formatted error dicts
        """
        if count is None:
            return self.error_history
        return self.error_history[-count:]
    
    def clear_history(self) -> None:
        """Clear error history"""
        self.error_history = []
        self.logger.info("Error history cleared")
    
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
