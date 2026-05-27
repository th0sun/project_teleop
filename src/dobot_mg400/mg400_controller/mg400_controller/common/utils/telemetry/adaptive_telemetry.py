#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Adaptive realtime telemetry — kept separate from triple_logger / latency_analyzer.

This module produces a single CSV per teleop session that records every
gate decision and every periodic feedback snapshot taken from the MG400
runtime.  It is intentionally NOT wired into the existing logging pipeline
because the goal is post-hoc analysis of the adaptive controller (1a6b443+):
how often the gate fires, what delta band fires it, what per-command SpeedJ
that produces, and how the resulting motion looks at the joint level.

Output paths (created on demand):
    ~/.ros/adaptive_telemetry/YYYYMMDD_HHMMSS_<pid>.csv

Schema (one event per row):
    ts            float seconds (time.time())
    monotonic     float seconds (time.perf_counter()) — relative pacing
    event         "gate" | "send" | "feedback" | "session"
    reason        gate decision reason string (e.g. "Adaptive_d2.50deg_v15dps_S55")
    delta_deg     max-axis |latest - last_sent|, degrees
    threshold_deg adaptive gate threshold for this tick, degrees
    hand_vel_dps  max-axis |target_velocity|, deg/s
    speed_j       per-cmd SpeedJ % stashed by gate decision
    acc_j         per-cmd AccJ %
    cp            per-cmd CP %
    q1..q4        joint targets used (degrees)
    qa1..qa4      QActual snapshot (degrees) — only on event=feedback
    qd_max        max joint velocity magnitude (deg/s) — only on event=feedback
    backlog_deg   max(|QTarget - QActual|) (degrees) — only on event=feedback
    robot_mode    raw RobotMode int — only on event=feedback
    note          freeform text (session start/stop, errors, mode-change)

The writer drains a bounded queue from a daemon thread so the realtime
control loop is never blocked by disk I/O.  If the queue fills (slow disk,
heavy logging), excess rows are dropped and counted in the session footer.
"""

from __future__ import annotations

import csv
import os
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional


_DEFAULT_QUEUE_LIMIT = 8192
_FLUSH_INTERVAL_SEC = 0.25


_FIELDS: tuple[str, ...] = (
    "ts", "monotonic",
    "event", "reason",
    "delta_deg", "threshold_deg", "hand_vel_dps",
    "speed_j", "acc_j", "cp",
    "q1", "q2", "q3", "q4",
    "qa1", "qa2", "qa3", "qa4",
    "qd_max", "backlog_deg", "robot_mode",
    "note",
)


def _default_log_dir() -> Path:
    base = os.environ.get("MG400_ADAPTIVE_TELEMETRY_DIR")
    if base:
        return Path(base).expanduser()
    return Path.home() / ".ros" / "adaptive_telemetry"


class AdaptiveTelemetry:
    """Append-only CSV writer for adaptive realtime analytics.

    Construct once per teleop session.  Call log_* methods from any thread.
    Call close() in the shutdown path to flush remaining rows.

    The class is intentionally tolerant of partial data — every log method
    accepts None for fields the caller doesn't have.  Missing fields render
    as empty CSV cells, which keeps post-hoc parsers simple.
    """

    def __init__(
        self,
        session_id: Optional[str] = None,
        log_dir: Optional[Path] = None,
        queue_limit: int = _DEFAULT_QUEUE_LIMIT,
        flush_interval_sec: float = _FLUSH_INTERVAL_SEC,
    ) -> None:
        self._enabled = True
        self._dropped = 0
        self._queue: queue.Queue = queue.Queue(maxsize=int(queue_limit))
        self._stop = threading.Event()
        self._flush_interval = float(flush_interval_sec)

        log_dir_path = Path(log_dir).expanduser() if log_dir is not None else _default_log_dir()
        log_dir_path.mkdir(parents=True, exist_ok=True)
        log_dir = log_dir_path
        if session_id is None:
            session_id = datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}"
        self._path = log_dir / f"{session_id}.csv"

        try:
            self._fp = open(self._path, "w", newline="", buffering=1)
        except OSError as exc:
            # If we can't open the file, disable cleanly so callers don't crash.
            self._enabled = False
            self._fp = None
            self._writer = None
            print(f"[AdaptiveTelemetry] disabled: {exc}")
            return

        self._writer = csv.DictWriter(self._fp, fieldnames=_FIELDS, extrasaction="ignore")
        self._writer.writeheader()
        self._fp.flush()

        self._worker = threading.Thread(
            target=self._drain_loop, name="AdaptiveTelemetryWriter", daemon=True
        )
        self._worker.start()

        self.log_session(note=f"session_start path={self._path}")

    # -------- public log methods --------

    @property
    def path(self) -> Path:
        return self._path

    def log_gate(
        self,
        reason: str,
        delta_rad: float,
        threshold_rad: float,
        hand_vel_rad_s: float,
        speed_j: int,
        acc_j: int,
        cp: int,
        q_target: Optional[Iterable[float]] = None,
        accepted: bool = False,
    ) -> None:
        """Log a gate decision (one per should_send_command tick)."""
        row = {
            "event": "send" if accepted else "gate",
            "reason": reason,
            "delta_deg": _rad_to_deg(delta_rad),
            "threshold_deg": _rad_to_deg(threshold_rad),
            "hand_vel_dps": _rad_to_deg(hand_vel_rad_s),
            "speed_j": int(speed_j),
            "acc_j": int(acc_j),
            "cp": int(cp),
        }
        _set_q_fields(row, q_target, "q")
        self._enqueue(row)

    def log_feedback(
        self,
        q_actual_rad: Optional[Iterable[float]] = None,
        qd_rad_s: Optional[Iterable[float]] = None,
        backlog_rad: Optional[float] = None,
        robot_mode: Optional[int] = None,
        note: str = "",
    ) -> None:
        """Periodic snapshot of robot state for jitter analysis."""
        row: dict[str, Any] = {
            "event": "feedback",
            "note": note,
        }
        _set_q_fields(row, q_actual_rad, "qa")
        if qd_rad_s is not None:
            try:
                qd_max = max(abs(float(v)) for v in qd_rad_s)
                row["qd_max"] = round(qd_max * 180.0 / 3.141592653589793, 4)
            except (TypeError, ValueError):
                pass
        if backlog_rad is not None:
            row["backlog_deg"] = _rad_to_deg(backlog_rad)
        if robot_mode is not None:
            try:
                row["robot_mode"] = int(robot_mode)
            except (TypeError, ValueError):
                pass
        self._enqueue(row)

    def log_session(self, note: str) -> None:
        """Free-form session marker (start, stop, profile change, error)."""
        self._enqueue({"event": "session", "note": str(note)})

    def close(self) -> None:
        if not self._enabled:
            return
        self.log_session(note=f"session_end dropped={self._dropped}")
        self._stop.set()
        try:
            self._worker.join(timeout=2.0)
        except Exception:
            pass
        if self._fp is not None:
            try:
                self._fp.flush()
                self._fp.close()
            except Exception:
                pass
        self._enabled = False

    # -------- internals --------

    def _enqueue(self, row: Mapping[str, Any]) -> None:
        if not self._enabled:
            return
        full = {"ts": time.time(), "monotonic": time.perf_counter()}
        full.update(row)
        try:
            self._queue.put_nowait(full)
        except queue.Full:
            self._dropped += 1

    def _drain_loop(self) -> None:
        # Drain rows in batches to keep file syscalls cheap.  Worker exits
        # when stop is set AND queue is empty.
        last_flush = time.monotonic()
        while True:
            try:
                row = self._queue.get(timeout=self._flush_interval)
            except queue.Empty:
                if self._stop.is_set():
                    break
                row = None
            if row is not None and self._writer is not None:
                try:
                    self._writer.writerow(row)
                except Exception:
                    self._dropped += 1
            now = time.monotonic()
            if now - last_flush >= self._flush_interval and self._fp is not None:
                try:
                    self._fp.flush()
                except Exception:
                    pass
                last_flush = now
        # Final flush on shutdown.
        if self._fp is not None:
            try:
                self._fp.flush()
            except Exception:
                pass


def _rad_to_deg(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    try:
        return round(float(value) * 180.0 / 3.141592653589793, 4)
    except (TypeError, ValueError):
        return None


def _set_q_fields(row: dict, joints_rad: Optional[Iterable[float]], prefix: str) -> None:
    if joints_rad is None:
        return
    try:
        as_list = list(joints_rad)[:4]
    except TypeError:
        return
    for i, val in enumerate(as_list, start=1):
        try:
            row[f"{prefix}{i}"] = round(float(val) * 180.0 / 3.141592653589793, 4)
        except (TypeError, ValueError):
            pass
