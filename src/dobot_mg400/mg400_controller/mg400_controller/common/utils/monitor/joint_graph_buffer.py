#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Rolling graph buffer for MG400 joint monitoring.
"""

from collections import deque
import time

import numpy as np


class JointGraphBuffer:
    def __init__(self, joint_count=4, window_sec=10.0, max_sample_hz=50, time_fn=None):
        self.joint_count = joint_count
        self.window_sec = window_sec
        self.time_fn = time_fn or time.time
        max_points = int(window_sec * max_sample_hz)
        self.time_buffer = deque(maxlen=max_points)
        self.unity_buffers = [deque(maxlen=max_points) for _ in range(joint_count)]
        self.pred_buffers = [deque(maxlen=max_points) for _ in range(joint_count)]
        self.sent_buffers = [deque(maxlen=max_points) for _ in range(joint_count)]
        self.actual_buffers = [deque(maxlen=max_points) for _ in range(joint_count)]
        self.start_time = self.time_fn()

    def append_telemetry(self, telemetry):
        rel_t = self.time_fn() - self.start_time
        self.time_buffer.append(rel_t)

        for index in range(self.joint_count):
            self.unity_buffers[index].append(telemetry.latest_target_joints[index])
            self.pred_buffers[index].append(telemetry.latest_predicted_joints[index])
            if telemetry.consume_sent_fresh(index):
                self.sent_buffers[index].append(telemetry.latest_sent_joints[index])
            else:
                self.sent_buffers[index].append(np.nan)
            self.actual_buffers[index].append(telemetry.latest_actual_joints[index])

        return rel_t

    def get_joint_arrays(self, index):
        return (
            np.array(self.time_buffer),
            np.array(self.unity_buffers[index]),
            np.array(self.pred_buffers[index]),
            np.array(self.sent_buffers[index]),
            np.array(self.actual_buffers[index]),
        )

    def get_x_limits(self, rel_t):
        return max(0, rel_t - self.window_sec), rel_t + 0.5

    @staticmethod
    def get_y_limits(*series):
        all_vals = np.concatenate(series)
        min_value = np.nanmin(all_vals)
        max_value = np.nanmax(all_vals)
        if np.isnan(min_value) or np.isnan(max_value):
            return None
        pad = max(2.0, (max_value - min_value) * 0.15)
        return min_value - pad, max_value + pad
