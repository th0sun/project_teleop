#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Reusable logging helpers for the monitor GUI.
"""

import csv
import datetime
import math
import os
import threading
import time


class SessionLogger:
    """
    Background session logger for monitor telemetry snapshots.
    """

    def __init__(
        self,
        snapshot_fn,
        base_dir=None,
        target_hz=20.0,
        autostart=True,
        error_handler=None,
    ):
        self.snapshot_fn = snapshot_fn
        self.base_dir = os.path.expanduser(base_dir or "~/project_teleop_ws/session_logs")
        self.target_hz = target_hz
        self.error_handler = error_handler or self._default_error_handler
        self._lock = threading.Lock()
        self._log_flush_counter = 0
        self._is_running = False
        self._log_thread = None

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.base_dir, timestamp)
        os.makedirs(self.session_dir, exist_ok=True)

        joints_path = os.path.join(self.session_dir, "joints_tracking.csv")
        xyz_path = os.path.join(self.session_dir, "xyz_tracking.csv")

        self._jt_file = open(joints_path, "w", newline="")
        self._jt_writer = csv.writer(self._jt_file)
        self._jt_writer.writerow([
            "timestamp", "elapsed_s",
            "unity_j1", "unity_j2", "unity_j3", "unity_j4",
            "predicted_j1", "predicted_j2", "predicted_j3", "predicted_j4",
            "sent_j1", "sent_j2", "sent_j3", "sent_j4",
            "actual_j1", "actual_j2", "actual_j3", "actual_j4",
            "robot_mode", "error_status",
        ])

        self._xyz_file = open(xyz_path, "w", newline="")
        self._xyz_writer = csv.writer(self._xyz_file)
        self._xyz_writer.writerow([
            "timestamp", "elapsed_s",
            "target_x", "target_y", "target_z",
            "actual_x", "actual_y", "actual_z",
            "diff_x", "diff_y", "diff_z",
        ])

        self._start_time = time.time()
        print(f"[SessionLogger] Logging to: {self.session_dir}")

        if autostart:
            self.start()

    def start(self):
        if self._is_running:
            return
        self._is_running = True
        self._log_thread = threading.Thread(target=self._logging_loop, daemon=True)
        self._log_thread.start()

    def _logging_loop(self):
        period = 1.0 / self.target_hz
        next_time = time.perf_counter() + period

        while self._is_running:
            try:
                self.log_snapshot(self.snapshot_fn())
                self._log_flush_counter += 1
                if self._log_flush_counter >= 100:
                    self.flush()
                    self._log_flush_counter = 0
            except Exception as exc:
                self.error_handler(exc)

            now = time.perf_counter()
            sleep_time = next_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)

            next_time += period
            if time.perf_counter() > next_time + period:
                next_time = time.perf_counter() + period

    def log_snapshot(self, snapshot):
        unity = list(snapshot["raw_unity"])
        predicted = list(snapshot["predicted"])
        sent = list(snapshot["sent"])
        actual = list(snapshot["actual"])
        tool_target = list(snapshot["tool_target"])
        tool_actual = list(snapshot["tool_actual"])
        unity_xyz = list(snapshot["unity_xyz"])

        target_xyz = unity_xyz[:3] if any(value != 0 for value in unity_xyz) else tool_target[:3]
        actual_xyz = tool_actual[:3]

        self._log_joints(
            unity,
            predicted,
            sent,
            actual,
            snapshot["robot_mode"],
            snapshot["error_status"],
        )
        self._log_xyz(target_xyz, actual_xyz)

    def _log_joints(self, unity, predicted, sent, actual, robot_mode, error_status):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed = round(time.time() - self._start_time, 3)
        row = (
            [timestamp, elapsed]
            + [round(value, 4) for value in unity]
            + [round(value, 4) for value in predicted]
            + [round(value, 4) for value in sent]
            + [round(value, 4) for value in actual]
            + [robot_mode, error_status]
        )
        with self._lock:
            self._jt_writer.writerow(row)

    def _log_xyz(self, target, actual):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed = round(time.time() - self._start_time, 3)
        diff = [round(actual[index] - target[index], 3) for index in range(3)]
        row = (
            [timestamp, elapsed]
            + [round(value, 3) for value in target]
            + [round(value, 3) for value in actual]
            + diff
        )
        with self._lock:
            self._xyz_writer.writerow(row)

    def flush(self):
        with self._lock:
            self._jt_file.flush()
            self._xyz_file.flush()

    def close(self):
        self._is_running = False
        if self._log_thread:
            self._log_thread.join(timeout=1.0)
        with self._lock:
            self._jt_file.flush()
            self._xyz_file.flush()
            self._jt_file.close()
            self._xyz_file.close()

    @staticmethod
    def _default_error_handler(exc):
        print(f"[SessionLogger] Error: {exc}")


class ManualMonitorLogger:
    """
    Reusable manual CSV logger for target/actual monitor snapshots.
    """

    def __init__(self, output_dir=None):
        self.output_dir = os.path.expanduser(output_dir or ".")
        self.is_active = False
        self._target_file = None
        self._actual_file = None
        self._target_writer = None
        self._actual_writer = None
        self._start_time = 0.0
        self.target_path = None
        self.actual_path = None

    def start_session(self):
        if self.is_active:
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.target_path = os.path.join(self.output_dir, f"teleop_target_{timestamp}.csv")
        self.actual_path = os.path.join(self.output_dir, f"teleop_actual_{timestamp}.csv")
        self._target_file = open(self.target_path, "w", newline="")
        self._actual_file = open(self.actual_path, "w", newline="")
        self._target_writer = csv.writer(self._target_file)
        self._actual_writer = csv.writer(self._actual_file)
        self._target_writer.writerow(["Time", "X", "Y", "Z", "Reach", "J1", "J2", "J3", "J4", "DiffTotal"])
        self._actual_writer.writerow(["Time", "X", "Y", "Z", "Reach", "J1", "J2", "J3", "J4"])
        self._start_time = time.time()
        self.is_active = True

    def stop_session(self):
        if not self.is_active:
            return
        self.is_active = False
        if self._target_file and not self._target_file.closed:
            self._target_file.flush()
            self._target_file.close()
        if self._actual_file and not self._actual_file.closed:
            self._actual_file.flush()
            self._actual_file.close()

    def log_sample(self, target_xyz, actual_xyz, target_joints, actual_joints, total_diff):
        if not self.is_active:
            return

        elapsed = time.time() - self._start_time
        target_reach = math.sqrt(target_xyz[0] ** 2 + target_xyz[1] ** 2)
        actual_reach = math.sqrt(actual_xyz[0] ** 2 + actual_xyz[1] ** 2)
        self._target_writer.writerow([
            f"{elapsed:.3f}",
            f"{target_xyz[0]:.3f}",
            f"{target_xyz[1]:.3f}",
            f"{target_xyz[2]:.3f}",
            f"{target_reach:.3f}",
            f"{target_joints[0]:.3f}",
            f"{target_joints[1]:.3f}",
            f"{target_joints[2]:.3f}",
            f"{target_joints[3]:.3f}",
            f"{total_diff:.3f}",
        ])
        self._actual_writer.writerow([
            f"{elapsed:.3f}",
            f"{actual_xyz[0]:.3f}",
            f"{actual_xyz[1]:.3f}",
            f"{actual_xyz[2]:.3f}",
            f"{actual_reach:.3f}",
            f"{actual_joints[0]:.3f}",
            f"{actual_joints[1]:.3f}",
            f"{actual_joints[2]:.3f}",
            f"{actual_joints[3]:.3f}",
        ])

