#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Single-file teleop session logger.

This logger replaces the older split logs for day-to-day experiments:

- teleop_analytics_*.csv: Unity raw target vs compensated target
- teleop_struct_*.csv: send decision and network/decision latency
- teleop_latency_*.csv: T1-T5 arrival latency
- unified_triple_log_*.csv: Unity / ROS command / robot feedback comparison

The file is an event log.  Each row has an ``event_type`` and a shared schema,
so one session can be analyzed without opening several CSVs.
"""

from __future__ import annotations

import csv
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


JOINT_COUNT = 4

FLOW_LABELS = {
    "unity_target": ("Unity", "ROS", "Unity -> ROS target receive"),
    "ros_command": ("ROS", "MG400 TCP", "ROS -> MG400 command send"),
    "robot_feedback": ("MG400 feedback", "ROS", "MG400 -> ROS feedback"),
    "latency_arrival": ("Unity", "MG400 feedback", "Unity -> MG400 target reached"),
    "full_sync": ("Unity / ROS / MG400", "analysis", "3-layer sync sample"),
}


class UnifiedTripleLogger:
    """Canonical one-file logger for Unity -> ROS -> MG400 experiments."""

    def __init__(self, log_dir: str | None = None):
        if log_dir is None:
            log_dir = os.path.abspath("./logs/teleop_sessions")
        else:
            log_dir = os.path.expanduser(log_dir)

        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = self.log_dir / f"teleop_session_{timestamp}.csv"

        self._file = open(self.file_path, mode="w", newline="")
        self._writer = csv.writer(self._file)
        self._lock = threading.Lock()
        self._sample_count = 0
        self._start_time = time.time()

        self._last_unity_raw_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_unity_comp_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_ros_cmd_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_robot_rad: list[float | None] = [None] * JOINT_COUNT

        self._writer.writerow(self._header())
        self._file.flush()

    @staticmethod
    def _header() -> list[str]:
        common = [
            "sample_id",
            "event_type",
            "source_layer",
            "target_layer",
            "flow_label",
            "notes",
            "elapsed_sec",
            "ros_wall_timestamp",
            "unity_raw_timestamp",
            "t1_unity_send_ros_wall",
            "t2_ros_recv_wall",
            "t3_cmd_send_wall",
            "t4_motion_start_wall",
            "t5_target_reached_wall",
            "network_delay_ms",
            "decision_delay_ms",
            "command_latency_ms",
            "robot_response_ms",
            "motion_time_ms",
            "motion_execution_ms",
            "true_end_to_end_ms",
            "send_reason",
            "robot_mode",
            "error_status",
            "queue_backlog_rad",
            "run_queued_cmd",
            "time_since_last_cmd_ms",
            "velocity_mag_rad_s",
            "final_error_rad",
            "max_joint_error_rad",
            "is_valid_arrival",
        ]
        joint_groups = []
        for prefix in (
            "unity_raw",
            "unity_compensated",
            "ros_cmd",
            "robot",
            "delta_unity_comp_to_ros_cmd",
            "delta_ros_cmd_to_robot",
        ):
            for unit in ("rad", "deg"):
                for idx in range(1, JOINT_COUNT + 1):
                    joint_groups.append(f"{prefix}_j{idx}_{unit}")
        return common + joint_groups

    @staticmethod
    def _joint4(values: Optional[Iterable[float]]) -> list[float | None]:
        if values is None:
            return [None] * JOINT_COUNT
        arr = list(values)[:JOINT_COUNT]
        if len(arr) < JOINT_COUNT:
            arr.extend([None] * (JOINT_COUNT - len(arr)))
        return arr

    @staticmethod
    def _deg(values: list[float | None]) -> list[float | None]:
        out: list[float | None] = []
        for value in values:
            out.append(None if value is None else float(np.degrees(value)))
        return out

    @staticmethod
    def _delta(a: list[float | None], b: list[float | None]) -> list[float | None]:
        out: list[float | None] = []
        for left, right in zip(a, b):
            if left is None or right is None:
                out.append(None)
            else:
                out.append(left - right)
        return out

    @staticmethod
    def _fmt(value, digits: int = 6) -> str:
        if value is None:
            return ""
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (int, np.integer)):
            return str(int(value))
        if isinstance(value, (float, np.floating)):
            return f"{float(value):.{digits}f}"
        return str(value)

    def _write_event(self, event_type: str, **fields) -> None:
        with self._lock:
            self._sample_count += 1
            elapsed = time.time() - self._start_time
            ros_ts = fields.get("ros_wall_timestamp")
            if ros_ts is None:
                ros_ts = time.time()

            raw = fields.get("unity_raw_rad")
            comp = fields.get("unity_compensated_rad")
            cmd = fields.get("ros_cmd_rad")
            robot = fields.get("robot_rad")

            if raw is not None:
                self._last_unity_raw_rad = self._joint4(raw)
            if comp is not None:
                self._last_unity_comp_rad = self._joint4(comp)
            if cmd is not None:
                self._last_ros_cmd_rad = self._joint4(cmd)
            if robot is not None:
                self._last_robot_rad = self._joint4(robot)

            delta_unity_cmd = self._delta(self._last_unity_comp_rad, self._last_ros_cmd_rad)
            delta_cmd_robot = self._delta(self._last_ros_cmd_rad, self._last_robot_rad)
            source, target, flow = FLOW_LABELS.get(event_type, ("", "", ""))

            row = [
                self._sample_count,
                event_type,
                fields.get("source_layer") or source,
                fields.get("target_layer") or target,
                fields.get("flow_label") or flow,
                fields.get("notes") or "",
                self._fmt(elapsed, 6),
                self._fmt(ros_ts, 6),
                self._fmt(fields.get("unity_raw_timestamp"), 6),
                self._fmt(fields.get("t1_unity_send_ros_wall"), 6),
                self._fmt(fields.get("t2_ros_recv_wall"), 6),
                self._fmt(fields.get("t3_cmd_send_wall"), 6),
                self._fmt(fields.get("t4_motion_start_wall"), 6),
                self._fmt(fields.get("t5_target_reached_wall"), 6),
                self._fmt(fields.get("network_delay_ms"), 3),
                self._fmt(fields.get("decision_delay_ms"), 3),
                self._fmt(fields.get("command_latency_ms"), 3),
                self._fmt(fields.get("robot_response_ms"), 3),
                self._fmt(fields.get("motion_time_ms"), 3),
                self._fmt(fields.get("motion_execution_ms"), 3),
                self._fmt(fields.get("true_end_to_end_ms"), 3),
                fields.get("send_reason") or "",
                self._fmt(fields.get("robot_mode"), 0),
                self._fmt(fields.get("error_status"), 0),
                self._fmt(fields.get("queue_backlog_rad"), 6),
                self._fmt(fields.get("run_queued_cmd"), 0),
                self._fmt(fields.get("time_since_last_cmd_ms"), 3),
                self._fmt(fields.get("velocity_mag_rad_s"), 6),
                self._fmt(fields.get("final_error_rad"), 6),
                self._fmt(fields.get("max_joint_error_rad"), 6),
                self._fmt(fields.get("is_valid_arrival")),
            ]

            groups = [
                self._last_unity_raw_rad,
                self._last_unity_comp_rad,
                self._last_ros_cmd_rad,
                self._last_robot_rad,
                delta_unity_cmd,
                delta_cmd_robot,
            ]
            for group in groups:
                row.extend(self._fmt(v, 6) for v in group)
                row.extend(self._fmt(v, 4) for v in self._deg(group))

            self._writer.writerow(row)
            if self._sample_count % 100 == 0:
                self._file.flush()

    def log_unity_target(
        self,
        unity_raw_rad,
        unity_compensated_rad,
        unity_raw_timestamp: float,
        unity_ros_timestamp: float,
        ros_recv_timestamp: float,
    ) -> None:
        self._write_event(
            "unity_target",
            ros_wall_timestamp=ros_recv_timestamp,
            unity_raw_timestamp=unity_raw_timestamp,
            t1_unity_send_ros_wall=unity_ros_timestamp,
            t2_ros_recv_wall=ros_recv_timestamp,
            network_delay_ms=max(0.0, (ros_recv_timestamp - unity_ros_timestamp) * 1000.0),
            unity_raw_rad=unity_raw_rad,
            unity_compensated_rad=unity_compensated_rad,
        )

    def log_unity_only(self, unity_joints, ros_timestamp: float) -> None:
        """Backward-compatible wrapper. Input is in degrees."""
        self._write_event(
            "unity_target",
            ros_wall_timestamp=ros_timestamp,
            unity_raw_rad=np.radians(self._joint4(unity_joints)),
            unity_compensated_rad=np.radians(self._joint4(unity_joints)),
        )

    def log_ros_cmd(
        self,
        ros_cmd_joints,
        unity_joints=None,
        ros_timestamp: float | None = None,
        *,
        t1_unity_send_ros_wall: float | None = None,
        t2_ros_recv_wall: float | None = None,
        network_delay_ms: float | None = None,
        decision_delay_ms: float | None = None,
        send_reason: str | None = None,
        robot_mode: int | None = None,
        error_status: int | None = None,
        queue_backlog_rad: float | None = None,
        run_queued_cmd: int | None = None,
        time_since_last_cmd_ms: float | None = None,
        velocity_mag_rad_s: float | None = None,
        joints_are_degrees: bool = True,
    ) -> None:
        cmd_rad = np.radians(self._joint4(ros_cmd_joints)) if joints_are_degrees else self._joint4(ros_cmd_joints)
        unity_comp_rad = None
        if unity_joints is not None:
            unity_comp_rad = np.radians(self._joint4(unity_joints)) if joints_are_degrees else self._joint4(unity_joints)
        self._write_event(
            "ros_command",
            ros_wall_timestamp=ros_timestamp,
            t1_unity_send_ros_wall=t1_unity_send_ros_wall,
            t2_ros_recv_wall=t2_ros_recv_wall,
            t3_cmd_send_wall=ros_timestamp,
            network_delay_ms=network_delay_ms,
            decision_delay_ms=decision_delay_ms,
            send_reason=send_reason,
            robot_mode=robot_mode,
            error_status=error_status,
            queue_backlog_rad=queue_backlog_rad,
            run_queued_cmd=run_queued_cmd,
            time_since_last_cmd_ms=time_since_last_cmd_ms,
            velocity_mag_rad_s=velocity_mag_rad_s,
            unity_compensated_rad=unity_comp_rad,
            ros_cmd_rad=cmd_rad,
        )

    def log_robot_feedback(
        self,
        robot_joints,
        ros_cmd_joints=None,
        ros_timestamp: float | None = None,
        *,
        robot_mode: int | None = None,
        error_status: int | None = None,
        joints_are_degrees: bool = True,
    ) -> None:
        robot_rad = np.radians(self._joint4(robot_joints)) if joints_are_degrees else self._joint4(robot_joints)
        cmd_rad = None
        if ros_cmd_joints is not None:
            cmd_rad = np.radians(self._joint4(ros_cmd_joints)) if joints_are_degrees else self._joint4(ros_cmd_joints)
        self._write_event(
            "robot_feedback",
            ros_wall_timestamp=ros_timestamp,
            robot_mode=robot_mode,
            error_status=error_status,
            ros_cmd_rad=cmd_rad,
            robot_rad=robot_rad,
        )

    def log_latency_event(self, metrics: dict) -> None:
        self._write_event(
            "latency_arrival",
            ros_wall_timestamp=metrics.get("now"),
            t1_unity_send_ros_wall=metrics.get("t1"),
            t2_ros_recv_wall=metrics.get("t2"),
            t3_cmd_send_wall=metrics.get("t3"),
            t4_motion_start_wall=metrics.get("t4"),
            t5_target_reached_wall=metrics.get("t5"),
            network_delay_ms=metrics.get("network_ms"),
            decision_delay_ms=metrics.get("decision_ms"),
            command_latency_ms=metrics.get("command_ms"),
            robot_response_ms=metrics.get("response_ms"),
            motion_time_ms=metrics.get("motion_time_ms"),
            motion_execution_ms=metrics.get("execution_ms"),
            true_end_to_end_ms=metrics.get("e2e_ms"),
            velocity_mag_rad_s=metrics.get("velocity"),
            final_error_rad=metrics.get("final_error"),
            max_joint_error_rad=metrics.get("max_error"),
            is_valid_arrival=metrics.get("is_valid"),
            ros_cmd_rad=metrics.get("target"),
            robot_rad=metrics.get("final_q"),
        )

    def log_full_sync(self, unity_joints, ros_cmd_joints, robot_joints, ros_timestamp: float) -> None:
        """Backward-compatible full-sync row. Inputs are in degrees."""
        self._write_event(
            "full_sync",
            ros_wall_timestamp=ros_timestamp,
            unity_raw_rad=np.radians(self._joint4(unity_joints)),
            unity_compensated_rad=np.radians(self._joint4(unity_joints)),
            ros_cmd_rad=np.radians(self._joint4(ros_cmd_joints)),
            robot_rad=np.radians(self._joint4(robot_joints)),
        )

    def close(self) -> None:
        with self._lock:
            if self._file and not self._file.closed:
                self._file.flush()
                self._file.close()

    def get_summary(self) -> str:
        elapsed = time.time() - self._start_time
        return f"Teleop session log: {self._sample_count} rows ({elapsed:.1f}s) -> {self.file_path.name}"

    def __del__(self):
        self.close()


def prompt_enable_triple_logging() -> bool:
    """Ask whether to record the one-file teleop session log."""
    print("\n" + "=" * 60)
    print("Teleop Session Logger")
    print("=" * 60)
    print("\nRecord one CSV for this run:")
    print("  unity_target    : Unity raw target + ROS latency-compensated target")
    print("  ros_command     : sent command + network/decision latency")
    print("  robot_feedback  : 10 Hz robot/Mock feedback samples")
    print("  latency_arrival : T1-T5 end-to-end arrival metrics")
    print("\nFile: ./logs/teleop_sessions/teleop_session_YYYYMMDD_HHMMSS.csv")
    print("=" * 60)

    response = input("\nRecord teleop session log? [Y/n]: ").strip().lower()
    return response in ("", "y", "yes")


def analyze_triple_log(file_path: str) -> dict:
    """Analyze a teleop session CSV and return simple delta statistics."""
    with open(file_path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    results = {
        "total_samples": len(rows),
        "duration_sec": max((float(r.get("elapsed_sec") or 0.0) for r in rows), default=0.0),
    }
    for i in range(1, JOINT_COUNT + 1):
        for name in ("delta_unity_comp_to_ros_cmd", "delta_ros_cmd_to_robot"):
            col = f"{name}_j{i}_rad"
            valid = []
            for row in rows:
                value = row.get(col)
                if value:
                    try:
                        valid.append(float(value))
                    except ValueError:
                        pass
            if valid:
                arr = np.asarray(valid, dtype=float)
                results[f"j{i}_{name}_mean_rad"] = float(np.mean(arr))
                results[f"j{i}_{name}_max_rad"] = float(np.max(np.abs(arr)))
    return results


if __name__ == "__main__":
    if prompt_enable_triple_logging():
        logger = UnifiedTripleLogger()
        for i in range(10):
            unity = np.radians((i * 0.5, i * 0.3, i * 0.2, i * 0.1))
            ros_cmd = unity + np.radians((0.01, 0.01, 0.01, 0.01))
            robot = unity + np.radians((0.02, 0.02, 0.02, 0.02))
            logger.log_unity_target(unity, unity, time.time(), time.time(), time.time())
            logger.log_ros_cmd(ros_cmd, unity, time.time(), joints_are_degrees=False)
            logger.log_robot_feedback(robot, ros_timestamp=time.time(), joints_are_degrees=False)
            time.sleep(0.1)
        print(logger.get_summary())
        logger.close()
        print(f"Log saved to: {logger.file_path}")
