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
import hashlib
import os
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

from mg400_controller.common.ros.unity_teleop_sample import (
    UNITY_SAMPLE_LOG_FIELDS,
    UNITY_SAMPLE_PROTOCOL_FIELDS,
)

try:
    from zoneinfo import ZoneInfo
except Exception:  # pragma: no cover - Python/tzdata fallback path.
    ZoneInfo = None


JOINT_COUNT = 4

FLOW_LABELS = {
    "unity_sample": ("Unity", "ROS", "Unity -> ROS protocol sample"),
    "unity_target": ("Unity", "ROS", "Unity -> ROS target receive"),
    "ros_command": ("ROS", "MG400 TCP", "ROS -> MG400 command send"),
    "robot_feedback": ("MG400 feedback", "ROS", "MG400 -> ROS feedback"),
    "latency_arrival": ("Unity", "MG400 feedback", "Unity -> MG400 target reached"),
    "command_result": ("MG400 feedback", "analysis", "Command id -> settled feedback result"),
    "joint_match": ("MG400 feedback", "analysis", "4J command target passed near"),
    "tool_match": ("MG400 feedback", "analysis", "XYZ/R command target passed near"),
    "full_sync": ("Unity / ROS / MG400", "analysis", "3-layer sync sample"),
}


def _sample_log_fields(unity_sample) -> dict:
    if unity_sample is None:
        return {}
    try:
        return unity_sample.to_log_fields()
    except AttributeError:
        return {}


def _hash_command(command_text: str | None) -> str | None:
    if not command_text:
        return None
    return hashlib.sha1(command_text.encode("utf-8")).hexdigest()[:12]


def _log_timezone():
    tz_name = os.environ.get("PROJECT_TELEOP_LOG_TZ") or os.environ.get("TZ") or "Asia/Bangkok"
    if ZoneInfo is not None:
        try:
            return ZoneInfo(tz_name)
        except Exception:
            pass
    if tz_name in ("Asia/Bangkok", "ICT"):
        return timezone(timedelta(hours=7), name="Asia/Bangkok")
    if tz_name in ("UTC", "Etc/UTC", "Z"):
        return timezone.utc
    return None


def _log_timestamp() -> str:
    tzinfo = _log_timezone()
    now = datetime.now(tzinfo) if tzinfo is not None else datetime.now()
    return now.strftime("%Y%m%d_%H%M%S")


# ─────────────────────────────────────────────────────────────────────────────
# CSV schema — keep these tables in sync with _build_csv_row's five blocks
# (A core, B unity sample, C joints rad+deg, D tool triples, E tool deltas /
# XYZ errors). Anything added here must also be appended to the row in the
# same position, or analysis tooling that reads columns by name will misalign.
# ─────────────────────────────────────────────────────────────────────────────

_CORE_COLUMNS: tuple[str, ...] = (
    "sample_id",
    "event_type",
    "source_layer",
    "target_layer",
    "flow_label",
    "operation_mode",
    "notes",
    "elapsed_sec",
    "ros_wall_timestamp",
    "control_command_seq",
    "ros_command_uid",
    "dobot_command_id",
    "robot_feedback_command_id",
    "command_result_status",
    "command_id_match",
    "dobot_command_response",
    "dobot_command_text",
    "dobot_command_hash",
    "command_tracking_source",
    "command_tracking_confidence",
    "feedback_command_id_before_send",
    "feedback_command_id_at_result",
    "feedback_command_id_changed",
    "settle_match_method",
    "settle_match_ambiguous",
    "settle_candidate_count",
    "settle_match_error_rad",
    "settle_second_best_error_rad",
    "settle_match_age_ms",
    "pending_command_count",
    "active_ros_command_age_ms",
    "unity_raw_timestamp",
    "t1_unity_send_ros_wall",
    "t2_ros_recv_wall",
    "t3_cmd_send_wall",
    "t4_motion_start_wall",
    "t5_target_reached_wall",
    "network_delay_ms",
    "unity_clock_offset_ms",
    "unity_to_ros_raw_offset_ms",
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
)

_JOINT_GROUP_PREFIXES: tuple[str, ...] = (
    "unity_raw",
    "unity_compensated",
    "ros_cmd",
    "robot",
    "delta_unity_comp_to_ros_cmd",
    "delta_ros_cmd_to_robot",
)

_TOOL_TRIPLE_PREFIXES: tuple[str, ...] = (
    "ros_cmd_tool_target",
    "robot_tool_actual",
    "robot_tool_target",
)
_TOOL_TRIPLE_FIELDS: tuple[str, ...] = ("x_mm", "y_mm", "z_mm", "r_deg", "aux5", "aux6")

_TOOL_DELTA_PREFIXES: tuple[str, ...] = (
    "delta_ros_cmd_tool_to_robot_actual",
    "delta_robot_tool_target_to_actual",
)
_TOOL_DELTA_FIELDS: tuple[str, ...] = ("x_mm", "y_mm", "z_mm", "r_deg")

_TOOL_METRIC_COLUMNS: tuple[str, ...] = (
    "error_ros_cmd_tool_to_robot_actual_mm",
    "error_robot_tool_target_to_actual_mm",
)


def _joint_group_columns() -> list[str]:
    """Block C column names: each joint snapshot expanded in both
    radians and degrees, JOINT_COUNT joints per snapshot."""
    cols: list[str] = []
    for prefix in _JOINT_GROUP_PREFIXES:
        for unit in ("rad", "deg"):
            for idx in range(1, JOINT_COUNT + 1):
                cols.append(f"{prefix}_j{idx}_{unit}")
    return cols


def _tool_group_columns() -> list[str]:
    """Block D column names: three tool-vector triples
    (ros_cmd_target, robot_actual, robot_target) × six fields each."""
    return [
        f"{prefix}_{name}"
        for prefix in _TOOL_TRIPLE_PREFIXES
        for name in _TOOL_TRIPLE_FIELDS
    ]


def _tool_delta_columns() -> list[str]:
    """Block E column names: two tool-delta vectors × four fields each."""
    return [
        f"{prefix}_{name}"
        for prefix in _TOOL_DELTA_PREFIXES
        for name in _TOOL_DELTA_FIELDS
    ]


class UnifiedTripleLogger:
    """Canonical one-file logger for Unity -> ROS -> MG400 experiments."""

    def __init__(self, log_dir: str | None = None):
        if log_dir is None:
            log_dir = os.path.abspath("./logs/teleop_sessions")
        else:
            log_dir = os.path.expanduser(log_dir)

        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = _log_timestamp()
        self.file_path = self.log_dir / f"teleop_session_{timestamp}.csv"
        self.unity_sample_file_path = self.log_dir / f"unity_teleop_sample_{timestamp}.csv"

        self._file = open(self.file_path, mode="w", newline="")
        self._writer = csv.writer(self._file)
        self._unity_sample_file = open(self.unity_sample_file_path, mode="w", newline="")
        self._unity_sample_writer = csv.DictWriter(
            self._unity_sample_file,
            fieldnames=list(UNITY_SAMPLE_PROTOCOL_FIELDS),
            extrasaction="ignore",
        )
        self._lock = threading.Lock()
        self._sample_count = 0
        self._unity_sample_count = 0
        self._start_time = time.time()

        self._last_unity_raw_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_unity_comp_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_ros_cmd_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_robot_rad: list[float | None] = [None] * JOINT_COUNT
        self._last_ros_command_seq: int | None = None
        self._last_ros_command_uid: str | None = None
        self._last_dobot_command_id: int | None = None
        self._last_ros_command_wall: float | None = None
        self._last_robot_feedback_command_id: int | None = None
        self._last_ros_cmd_tool_target: list[float | None] = [None] * 6
        self._last_robot_tool_actual: list[float | None] = [None] * 6
        self._last_robot_tool_target: list[float | None] = [None] * 6

        self._writer.writerow(self._header())
        self._file.flush()
        self._unity_sample_writer.writeheader()
        self._unity_sample_file.flush()

    @staticmethod
    def _header() -> list[str]:
        """CSV header in the same five-block order as _build_csv_row:
        core columns, unity sample fields, joint groups (rad+deg),
        tool triples, tool deltas + XYZ errors.
        """
        return (
            list(_CORE_COLUMNS)
            + list(UNITY_SAMPLE_LOG_FIELDS)
            + _joint_group_columns()
            + _tool_group_columns()
            + _tool_delta_columns()
            + list(_TOOL_METRIC_COLUMNS)
        )

    @staticmethod
    def _joint4(values: Optional[Iterable[float]]) -> list[float | None]:
        if values is None:
            return [None] * JOINT_COUNT
        arr = list(values)[:JOINT_COUNT]
        if len(arr) < JOINT_COUNT:
            arr.extend([None] * (JOINT_COUNT - len(arr)))
        return arr

    @staticmethod
    def _tool6(values: Optional[Iterable[float]]) -> list[float | None]:
        if values is None:
            return [None] * 6
        arr = list(values)[:6]
        if len(arr) < 6:
            arr.extend([None] * (6 - len(arr)))
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
    def _tool_delta4(actual: list[float | None], target: list[float | None]) -> list[float | None]:
        out: list[float | None] = []
        for left, right in zip(actual[:4], target[:4]):
            if left is None or right is None:
                out.append(None)
            else:
                out.append(left - right)
        return out

    @staticmethod
    def _xyz_error_mm(actual: list[float | None], target: list[float | None]) -> float | None:
        if any(value is None for value in [*actual[:3], *target[:3]]):
            return None
        return float(np.linalg.norm(np.asarray(actual[:3], dtype=float) - np.asarray(target[:3], dtype=float)))

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
        """Write one CSV row for a teleop event.

        Pipeline:
          1. Increment sample counter + resolve event timestamp.
          2. Update the rolling "last-seen" snapshots (joints, command
             identity, tool vectors) so events that don't carry a
             field can still emit a row using the most recent value.
          3. Compute per-event metrics (deltas, error mm, clock
             offsets, active-command age).
          4. Build the row + write it.

        Each phase is its own helper so the row schema is decoupled
        from the bookkeeping rules.
        """
        with self._lock:
            self._sample_count += 1
            elapsed = time.time() - self._start_time
            ros_ts = fields.get("ros_wall_timestamp") or time.time()

            identity = self._update_event_snapshots(event_type, fields, ros_ts)
            metrics = self._compute_event_metrics(fields, identity, ros_ts)
            row = self._build_csv_row(
                event_type, fields, elapsed, ros_ts, identity, metrics
            )

            self._writer.writerow(row)
            if self._sample_count % 100 == 0:
                self._file.flush()

    def _update_event_snapshots(self, event_type: str, fields: dict, ros_ts: float) -> dict:
        """Update every ``self._last_*`` rolling snapshot from ``fields``
        and return a dict of identity values resolved for this row
        (control_command_seq, ros_command_uid, dobot_command_id,
        robot_feedback_command_id, dobot_command_text + hash,
        feedback_before_send / at_result / changed).

        Pulling this out of _write_event removes a wall of
        ``if x is not None: self._last_x = ...`` from the row-building
        code path and gives the caller a single dict to thread into
        ``_build_csv_row``.
        """
        # Joint snapshots
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

        # Command sequence + uid: ros_command events auto-allocate a
        # sequence if the caller didn't pass one (legacy call sites).
        control_command_seq = fields.get("control_command_seq")
        if control_command_seq is not None:
            self._last_ros_command_seq = int(control_command_seq)
        elif event_type == "ros_command":
            self._last_ros_command_seq = (self._last_ros_command_seq or 0) + 1
            control_command_seq = self._last_ros_command_seq

        ros_command_uid = fields.get("ros_command_uid")
        if (
            ros_command_uid is None
            and event_type == "ros_command"
            and control_command_seq is not None
        ):
            ros_command_uid = f"ros_cmd_{int(control_command_seq):06d}"
        if ros_command_uid is not None:
            self._last_ros_command_uid = str(ros_command_uid)
        if event_type == "ros_command":
            self._last_ros_command_wall = ros_ts

        # Dobot / feedback IDs
        dobot_command_id = fields.get("dobot_command_id")
        if dobot_command_id is not None:
            self._last_dobot_command_id = int(dobot_command_id)
        robot_feedback_command_id = fields.get("robot_feedback_command_id")
        if robot_feedback_command_id is not None:
            self._last_robot_feedback_command_id = int(robot_feedback_command_id)

        # Command text + hash + feedback diff
        dobot_command_text = fields.get("dobot_command_text")
        dobot_command_hash = (
            fields.get("dobot_command_hash") or _hash_command(dobot_command_text)
        )
        feedback_before_send = fields.get("feedback_command_id_before_send")
        feedback_at_result = fields.get("feedback_command_id_at_result")
        if feedback_at_result is None and event_type in ("latency_arrival", "command_result"):
            feedback_at_result = robot_feedback_command_id
        feedback_id_changed = fields.get("feedback_command_id_changed")
        if (
            feedback_id_changed is None
            and feedback_before_send is not None
            and feedback_at_result is not None
        ):
            feedback_id_changed = int(feedback_before_send) != int(feedback_at_result)

        # Tool snapshots
        if (val := fields.get("ros_cmd_tool_target")) is not None:
            self._last_ros_cmd_tool_target = self._tool6(val)
        if (val := fields.get("robot_tool_actual")) is not None:
            self._last_robot_tool_actual = self._tool6(val)
        if (val := fields.get("robot_tool_target")) is not None:
            self._last_robot_tool_target = self._tool6(val)

        return {
            "control_command_seq": control_command_seq,
            "ros_command_uid": ros_command_uid,
            "dobot_command_id": dobot_command_id,
            "robot_feedback_command_id": robot_feedback_command_id,
            "dobot_command_text": dobot_command_text,
            "dobot_command_hash": dobot_command_hash,
            "feedback_before_send": feedback_before_send,
            "feedback_at_result": feedback_at_result,
            "feedback_id_changed": feedback_id_changed,
        }

    def _compute_event_metrics(self, fields: dict, identity: dict, ros_ts: float) -> dict:
        """Derive per-row metrics that aren't direct ``fields`` entries:
        joint + tool deltas, XYZ error mm, active-command age, clock-
        offset estimates inferred from unity_raw_timestamp /
        t1_unity_send_ros_wall / t2_ros_recv_wall.
        """
        delta_unity_cmd = self._delta(self._last_unity_comp_rad, self._last_ros_cmd_rad)
        delta_cmd_robot = self._delta(self._last_ros_cmd_rad, self._last_robot_rad)
        delta_ros_tool_robot = self._tool_delta4(
            self._last_robot_tool_actual, self._last_ros_cmd_tool_target
        )
        delta_robot_target_actual = self._tool_delta4(
            self._last_robot_tool_actual, self._last_robot_tool_target
        )
        error_ros_tool_robot = self._xyz_error_mm(
            self._last_robot_tool_actual, self._last_ros_cmd_tool_target
        )
        error_robot_target_actual = self._xyz_error_mm(
            self._last_robot_tool_actual, self._last_robot_tool_target
        )

        active_age_ms = None
        if self._last_ros_command_wall is not None:
            active_age_ms = (ros_ts - self._last_ros_command_wall) * 1000.0
            # Worker threads can write events after a newer command row even
            # when the event's original ROS timestamp is earlier.
            if active_age_ms < 0.0:
                active_age_ms = None

        unity_raw_ts = fields.get("unity_raw_timestamp")
        t1_unity_send = fields.get("t1_unity_send_ros_wall")
        t2_ros_recv = fields.get("t2_ros_recv_wall")
        unity_clock_offset_ms = fields.get("unity_clock_offset_ms")
        if unity_clock_offset_ms is None and unity_raw_ts is not None and t1_unity_send is not None:
            unity_clock_offset_ms = (t1_unity_send - unity_raw_ts) * 1000.0
        unity_to_ros_raw_offset_ms = fields.get("unity_to_ros_raw_offset_ms")
        if (
            unity_to_ros_raw_offset_ms is None
            and unity_raw_ts is not None
            and t2_ros_recv is not None
        ):
            unity_to_ros_raw_offset_ms = (t2_ros_recv - unity_raw_ts) * 1000.0

        return {
            "delta_unity_cmd": delta_unity_cmd,
            "delta_cmd_robot": delta_cmd_robot,
            "delta_ros_tool_robot": delta_ros_tool_robot,
            "delta_robot_target_actual": delta_robot_target_actual,
            "error_ros_tool_robot": error_ros_tool_robot,
            "error_robot_target_actual": error_robot_target_actual,
            "active_age_ms": active_age_ms,
            "unity_raw_ts": unity_raw_ts,
            "t1_unity_send": t1_unity_send,
            "t2_ros_recv": t2_ros_recv,
            "unity_clock_offset_ms": unity_clock_offset_ms,
            "unity_to_ros_raw_offset_ms": unity_to_ros_raw_offset_ms,
        }

    def _build_csv_row(
        self,
        event_type: str,
        fields: dict,
        elapsed: float,
        ros_ts: float,
        identity: dict,
        metrics: dict,
    ) -> list:
        """Materialise the row that ``_writer.writerow`` consumes.

        Built in five blocks, in column order:
          A. core columns (event id, layer labels, sequence/uid,
             command identity + timing markers, decision metrics).
          B. unity sample fields (per UNITY_SAMPLE_LOG_FIELDS).
          C. joint groups in rad + deg (raw, comp, cmd, robot,
             delta_unity_cmd, delta_cmd_robot).
          D. tool triples (ros_cmd_target, robot_actual, robot_target).
          E. tool deltas + XYZ errors.
        """
        source, target, flow = FLOW_LABELS.get(event_type, ("", "", ""))
        # Block A: core columns.
        row = [
            self._sample_count,
            event_type,
            fields.get("source_layer") or source,
            fields.get("target_layer") or target,
            fields.get("flow_label") or flow,
            fields.get("operation_mode") or "",
            fields.get("notes") or "",
            self._fmt(elapsed, 6),
            self._fmt(ros_ts, 6),
            self._fmt(
                identity["control_command_seq"]
                if identity["control_command_seq"] is not None
                else self._last_ros_command_seq,
                0,
            ),
            self._fmt(
                identity["ros_command_uid"]
                if identity["ros_command_uid"] is not None
                else self._last_ros_command_uid
            ),
            self._fmt(identity["dobot_command_id"], 0),
            self._fmt(identity["robot_feedback_command_id"], 0),
            fields.get("command_result_status") or "",
            self._fmt(fields.get("command_id_match")),
            fields.get("dobot_command_response") or "",
            identity["dobot_command_text"] or "",
            identity["dobot_command_hash"] or "",
            fields.get("command_tracking_source") or "",
            fields.get("command_tracking_confidence") or "",
            self._fmt(identity["feedback_before_send"], 0),
            self._fmt(identity["feedback_at_result"], 0),
            self._fmt(identity["feedback_id_changed"]),
            fields.get("settle_match_method") or "",
            self._fmt(fields.get("settle_match_ambiguous")),
            self._fmt(fields.get("settle_candidate_count"), 0),
            self._fmt(fields.get("settle_match_error_rad"), 6),
            self._fmt(fields.get("settle_second_best_error_rad"), 6),
            self._fmt(fields.get("settle_match_age_ms"), 3),
            self._fmt(fields.get("pending_command_count"), 0),
            self._fmt(metrics["active_age_ms"], 3),
            self._fmt(metrics["unity_raw_ts"], 6),
            self._fmt(metrics["t1_unity_send"], 6),
            self._fmt(metrics["t2_ros_recv"], 6),
            self._fmt(fields.get("t3_cmd_send_wall"), 6),
            self._fmt(fields.get("t4_motion_start_wall"), 6),
            self._fmt(fields.get("t5_target_reached_wall"), 6),
            self._fmt(fields.get("network_delay_ms"), 3),
            self._fmt(metrics["unity_clock_offset_ms"], 3),
            self._fmt(metrics["unity_to_ros_raw_offset_ms"], 3),
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

        # Block B: unity sample fields.
        unity_sample_fields = fields.get("unity_sample_fields") or {}
        for field_name in UNITY_SAMPLE_LOG_FIELDS:
            row.append(self._fmt(unity_sample_fields.get(field_name)))

        # Block C: joint groups (rad + deg).
        joint_groups = [
            self._last_unity_raw_rad,
            self._last_unity_comp_rad,
            self._last_ros_cmd_rad,
            self._last_robot_rad,
            metrics["delta_unity_cmd"],
            metrics["delta_cmd_robot"],
        ]
        for group in joint_groups:
            row.extend(self._fmt(v, 6) for v in group)
            row.extend(self._fmt(v, 4) for v in self._deg(group))

        # Block D: tool triples. ToolVectorActual/Target comes directly
        # from the Dobot feedback packet — first four values are
        # [X, Y, Z, R]; the last two are preserved as aux fields rather
        # than getting unsupported semantic names.
        for tool in (
            self._last_ros_cmd_tool_target,
            self._last_robot_tool_actual,
            self._last_robot_tool_target,
        ):
            x, y, z, r, aux5, aux6 = tool
            row.extend([
                self._fmt(x, 6),
                self._fmt(y, 6),
                self._fmt(z, 6),
                self._fmt(r, 6),
                self._fmt(aux5, 6),
                self._fmt(aux6, 6),
            ])

        # Block E: tool deltas + XYZ errors.
        for group in (metrics["delta_ros_tool_robot"], metrics["delta_robot_target_actual"]):
            row.extend(self._fmt(v, 6) for v in group)
        row.extend([
            self._fmt(metrics["error_ros_tool_robot"], 6),
            self._fmt(metrics["error_robot_target_actual"], 6),
        ])

        return row

    def log_unity_target(
        self,
        unity_raw_rad,
        unity_compensated_rad,
        unity_raw_timestamp: float,
        unity_ros_timestamp: float,
        ros_recv_timestamp: float,
        *,
        unity_sample=None,
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
            unity_sample_fields=_sample_log_fields(unity_sample),
        )

    def log_unity_sample(self, unity_sample, ros_recv_timestamp: float) -> None:
        joints = None
        try:
            joints = unity_sample.ik_joints_rad()
        except Exception:
            joints = None
        self._write_unity_sample_csv(unity_sample)
        self._write_event(
            "unity_sample",
            ros_wall_timestamp=ros_recv_timestamp,
            unity_raw_timestamp=unity_sample.unity_send_ts(),
            t2_ros_recv_wall=ros_recv_timestamp,
            unity_raw_rad=joints,
            unity_compensated_rad=joints,
            unity_sample_fields=_sample_log_fields(unity_sample),
        )

    def _write_unity_sample_csv(self, unity_sample) -> None:
        protocol_fields = {}
        try:
            protocol_fields = unity_sample.to_protocol_fields()
        except AttributeError:
            protocol_fields = {
                field_name: unity_sample.get(field_name)
                for field_name in UNITY_SAMPLE_PROTOCOL_FIELDS
            }

        row = {
            field_name: self._fmt(protocol_fields.get(field_name))
            for field_name in UNITY_SAMPLE_PROTOCOL_FIELDS
        }
        with self._lock:
            self._unity_sample_writer.writerow(row)
            self._unity_sample_count += 1
            if self._unity_sample_count % 100 == 0:
                self._unity_sample_file.flush()

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
        control_command_seq: int | None = None,
        ros_command_uid: str | None = None,
        dobot_command_id: int | None = None,
        dobot_command_response: str | None = None,
        dobot_command_text: str | None = None,
        command_tracking_source: str | None = None,
        command_tracking_confidence: str | None = None,
        feedback_command_id_before_send: int | None = None,
        ros_cmd_tool_target=None,
        unity_sample=None,
        joints_are_degrees: bool = True,
    ) -> None:
        cmd_rad = np.radians(self._joint4(ros_cmd_joints)) if joints_are_degrees else self._joint4(ros_cmd_joints)
        unity_comp_rad = None
        if unity_joints is not None:
            unity_comp_rad = np.radians(self._joint4(unity_joints)) if joints_are_degrees else self._joint4(unity_joints)
        unity_raw_timestamp = None
        if unity_sample is not None:
            try:
                unity_raw_timestamp = unity_sample.unity_send_ts()
            except AttributeError:
                unity_raw_timestamp = None
        self._write_event(
            "ros_command",
            ros_wall_timestamp=ros_timestamp,
            unity_raw_timestamp=unity_raw_timestamp,
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
            control_command_seq=control_command_seq,
            ros_command_uid=ros_command_uid,
            dobot_command_id=dobot_command_id,
            dobot_command_response=dobot_command_response,
            dobot_command_text=dobot_command_text,
            command_tracking_source=command_tracking_source,
            command_tracking_confidence=command_tracking_confidence,
            feedback_command_id_before_send=feedback_command_id_before_send,
            unity_compensated_rad=unity_comp_rad,
            ros_cmd_rad=cmd_rad,
            ros_cmd_tool_target=ros_cmd_tool_target,
            unity_sample_fields=_sample_log_fields(unity_sample),
        )

    def log_robot_feedback(
        self,
        robot_joints,
        ros_cmd_joints=None,
        ros_timestamp: float | None = None,
        *,
        robot_mode: int | None = None,
        error_status: int | None = None,
        robot_tool_actual=None,
        robot_tool_target=None,
        robot_feedback_command_id: int | None = None,
        queue_backlog_rad: float | None = None,
        run_queued_cmd: int | None = None,
        operation_mode: str | None = None,
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
            robot_feedback_command_id=robot_feedback_command_id,
            queue_backlog_rad=queue_backlog_rad,
            run_queued_cmd=run_queued_cmd,
            ros_cmd_rad=cmd_rad,
            robot_rad=robot_rad,
            robot_tool_actual=robot_tool_actual,
            robot_tool_target=robot_tool_target,
            operation_mode=operation_mode,
        )

    def log_latency_event(self, metrics: dict) -> None:
        unity_raw_timestamp = None
        unity_sample = metrics.get("unity_sample")
        if unity_sample is not None:
            try:
                unity_raw_timestamp = unity_sample.unity_send_ts()
            except AttributeError:
                unity_raw_timestamp = None
        self._write_event(
            "latency_arrival",
            ros_wall_timestamp=metrics.get("now"),
            unity_raw_timestamp=unity_raw_timestamp,
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
            operation_mode=metrics.get("operation_mode"),
            control_command_seq=metrics.get("control_command_seq"),
            ros_command_uid=metrics.get("ros_command_uid"),
            dobot_command_id=metrics.get("dobot_command_id"),
            robot_feedback_command_id=metrics.get("robot_feedback_command_id"),
            command_result_status=metrics.get("command_result_status"),
            command_id_match=metrics.get("command_id_match"),
            dobot_command_text=metrics.get("dobot_command_text"),
            command_tracking_source=metrics.get("command_tracking_source"),
            command_tracking_confidence=metrics.get("command_tracking_confidence"),
            feedback_command_id_before_send=metrics.get("feedback_command_id_before_send"),
            feedback_command_id_at_result=metrics.get("feedback_command_id_at_result"),
            feedback_command_id_changed=metrics.get("feedback_command_id_changed"),
            settle_match_method=metrics.get("settle_match_method"),
            settle_match_ambiguous=metrics.get("settle_match_ambiguous"),
            settle_candidate_count=metrics.get("settle_candidate_count"),
            settle_match_error_rad=metrics.get("settle_match_error_rad"),
            settle_second_best_error_rad=metrics.get("settle_second_best_error_rad"),
            settle_match_age_ms=metrics.get("settle_match_age_ms"),
            pending_command_count=metrics.get("pending_command_count"),
            ros_cmd_rad=metrics.get("target"),
            robot_rad=metrics.get("final_q"),
            ros_cmd_tool_target=metrics.get("ros_cmd_tool_target"),
            robot_tool_actual=metrics.get("final_tool_actual"),
            robot_tool_target=metrics.get("final_tool_target"),
            unity_sample_fields=_sample_log_fields(unity_sample),
        )

    def log_command_result(
        self,
        *,
        control_command_seq: int | None,
        ros_command_uid: str | None = None,
        dobot_command_id: int | None,
        status: str,
        command_id_match: bool | None = None,
        robot_feedback_command_id: int | None = None,
        dobot_command_text: str | None = None,
        command_tracking_source: str | None = None,
        command_tracking_confidence: str | None = None,
        feedback_command_id_before_send: int | None = None,
        feedback_command_id_at_result: int | None = None,
        feedback_command_id_changed: bool | None = None,
        settle_match_method: str | None = None,
        settle_match_ambiguous: bool | None = None,
        settle_candidate_count: int | None = None,
        settle_match_error_rad: float | None = None,
        settle_second_best_error_rad: float | None = None,
        settle_match_age_ms: float | None = None,
        pending_command_count: int | None = None,
        ros_timestamp: float | None = None,
        ros_cmd_joints=None,
        robot_joints=None,
        ros_cmd_tool_target=None,
        robot_tool_actual=None,
        robot_tool_target=None,
        final_error_rad: float | None = None,
        max_joint_error_rad: float | None = None,
        robot_mode: int | None = None,
        error_status: int | None = None,
        operation_mode: str | None = None,
        unity_sample=None,
    ) -> None:
        self._write_event(
            "command_result",
            ros_wall_timestamp=ros_timestamp,
            control_command_seq=control_command_seq,
            ros_command_uid=ros_command_uid,
            dobot_command_id=dobot_command_id,
            robot_feedback_command_id=robot_feedback_command_id,
            command_result_status=status,
            command_id_match=command_id_match,
            dobot_command_text=dobot_command_text,
            command_tracking_source=command_tracking_source,
            command_tracking_confidence=command_tracking_confidence,
            feedback_command_id_before_send=feedback_command_id_before_send,
            feedback_command_id_at_result=feedback_command_id_at_result,
            feedback_command_id_changed=feedback_command_id_changed,
            settle_match_method=settle_match_method,
            settle_match_ambiguous=settle_match_ambiguous,
            settle_candidate_count=settle_candidate_count,
            settle_match_error_rad=settle_match_error_rad,
            settle_second_best_error_rad=settle_second_best_error_rad,
            settle_match_age_ms=settle_match_age_ms,
            pending_command_count=pending_command_count,
            final_error_rad=final_error_rad,
            max_joint_error_rad=max_joint_error_rad,
            robot_mode=robot_mode,
            error_status=error_status,
            operation_mode=operation_mode,
            ros_cmd_rad=ros_cmd_joints,
            robot_rad=robot_joints,
            ros_cmd_tool_target=ros_cmd_tool_target,
            robot_tool_actual=robot_tool_actual,
            robot_tool_target=robot_tool_target,
            unity_sample_fields=_sample_log_fields(unity_sample),
        )

    def log_command_match(
        self,
        *,
        event_type: str,
        control_command_seq: int | None,
        ros_command_uid: str | None = None,
        dobot_command_id: int | None = None,
        status: str,
        robot_feedback_command_id: int | None = None,
        dobot_command_text: str | None = None,
        command_tracking_source: str | None = None,
        command_tracking_confidence: str | None = None,
        feedback_command_id_before_send: int | None = None,
        feedback_command_id_at_result: int | None = None,
        feedback_command_id_changed: bool | None = None,
        settle_match_method: str | None = None,
        settle_match_ambiguous: bool | None = None,
        settle_candidate_count: int | None = None,
        settle_match_error_rad: float | None = None,
        settle_second_best_error_rad: float | None = None,
        settle_match_age_ms: float | None = None,
        pending_command_count: int | None = None,
        ros_timestamp: float | None = None,
        ros_cmd_joints=None,
        robot_joints=None,
        ros_cmd_tool_target=None,
        robot_tool_actual=None,
        robot_tool_target=None,
        final_error_rad: float | None = None,
        max_joint_error_rad: float | None = None,
        velocity_mag_rad_s: float | None = None,
        robot_mode: int | None = None,
        error_status: int | None = None,
        operation_mode: str | None = None,
        unity_sample=None,
    ) -> None:
        if event_type not in {"joint_match", "tool_match"}:
            raise ValueError(f"Unsupported command match event_type={event_type!r}")
        self._write_event(
            event_type,
            ros_wall_timestamp=ros_timestamp,
            control_command_seq=control_command_seq,
            ros_command_uid=ros_command_uid,
            dobot_command_id=dobot_command_id,
            robot_feedback_command_id=robot_feedback_command_id,
            command_result_status=status,
            dobot_command_text=dobot_command_text,
            command_tracking_source=command_tracking_source,
            command_tracking_confidence=command_tracking_confidence,
            feedback_command_id_before_send=feedback_command_id_before_send,
            feedback_command_id_at_result=feedback_command_id_at_result,
            feedback_command_id_changed=feedback_command_id_changed,
            settle_match_method=settle_match_method,
            settle_match_ambiguous=settle_match_ambiguous,
            settle_candidate_count=settle_candidate_count,
            settle_match_error_rad=settle_match_error_rad,
            settle_second_best_error_rad=settle_second_best_error_rad,
            settle_match_age_ms=settle_match_age_ms,
            pending_command_count=pending_command_count,
            final_error_rad=final_error_rad,
            max_joint_error_rad=max_joint_error_rad,
            velocity_mag_rad_s=velocity_mag_rad_s,
            robot_mode=robot_mode,
            error_status=error_status,
            operation_mode=operation_mode,
            ros_cmd_rad=ros_cmd_joints,
            robot_rad=robot_joints,
            ros_cmd_tool_target=ros_cmd_tool_target,
            robot_tool_actual=robot_tool_actual,
            robot_tool_target=robot_tool_target,
            unity_sample_fields=_sample_log_fields(unity_sample),
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
            if self._unity_sample_file and not self._unity_sample_file.closed:
                self._unity_sample_file.flush()
                self._unity_sample_file.close()

    def get_summary(self) -> str:
        elapsed = time.time() - self._start_time
        return (
            f"Teleop session log: {self._sample_count} rows ({elapsed:.1f}s) -> {self.file_path.name}; "
            f"Unity samples: {self._unity_sample_count} rows -> {self.unity_sample_file_path.name}"
        )

    def __del__(self):
        self.close()


def prompt_enable_triple_logging() -> bool:
    """Ask whether to record the one-file teleop session log."""
    env_value = os.environ.get("PROJECT_TELEOP_RECORD_LOG")
    if env_value is not None:
        response = env_value.strip().lower()
        enabled = response in ("", "1", "true", "y", "yes", "on")
        print(f"\nTeleop session logging {'enabled' if enabled else 'disabled'} by PROJECT_TELEOP_RECORD_LOG")
        return enabled

    print("\n" + "=" * 60)
    print("Teleop Session Logger")
    print("=" * 60)
    print("\nRecord one CSV for this run:")
    print("  unity_sample    : Unity JSON protocol sample + controller/IK context")
    print("  unity_target    : Unity raw target + ROS latency-compensated target")
    print("  ros_command     : sent command + network/decision latency")
    print("  robot_feedback  : accepted MG400 30004 feedback packets at packet cadence")
    print("  latency_arrival : T1-T5 end-to-end arrival metrics")
    print("  command_result  : command id or fallback pose-settle result")
    print("  separate file   : unity_teleop_sample_YYYYMMDD_HHMMSS.csv has Unity payload only")
    print("\nFile: ./logs/teleop_sessions/teleop_session_YYYYMMDD_HHMMSS.csv")
    print("=" * 60)

    response = input("\nRecord teleop session log? [Y/n]: ").strip().lower()
    return response in ("", "y", "yes")


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
