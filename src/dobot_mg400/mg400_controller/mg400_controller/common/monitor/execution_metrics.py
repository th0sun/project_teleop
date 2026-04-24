#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Execution timing state for the MG400 monitor GUI.
"""

import time

import numpy as np


class ExecutionMonitor:
    def __init__(self, start_threshold=2.0, stop_threshold=0.5, time_fn=None):
        self.start_threshold = start_threshold
        self.stop_threshold = stop_threshold
        self.time_fn = time_fn or time.time
        self.state = "IDLE"
        self.start_time = 0.0
        self.end_time = 0.0
        self.last_duration = 0.0
        self.durations = []

    def update(self, total_error):
        now = self.time_fn()
        if self.state == "IDLE" or self.state == "ARRIVED":
            if total_error > self.start_threshold:
                self.state = "MOVING"
                self.start_time = now
                return "STARTED"
        elif self.state == "MOVING":
            if total_error < self.stop_threshold:
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
