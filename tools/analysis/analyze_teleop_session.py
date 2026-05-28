#!/usr/bin/env python3
"""Analyze one-file teleop session logs and generate report-ready figures.

Usage:
    python3 tools/analyze_teleop_session.py logs/teleop_sessions/teleop_session_*.csv

On macOS you can drag a CSV file into the terminal after the command.  If no
file is provided, the script tries to open a file picker.
"""

from __future__ import annotations

import argparse
import collections
import csv
import io
import json
import os
import sys
from pathlib import Path
from typing import Iterable

os.environ.setdefault("MPLCONFIGDIR", "/tmp/project_teleop_matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src" / "dobot_mg400" / "mg400_controller"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

try:
    from mg400_controller.common.utils.kinematics import KinematicsCalculator
except Exception:  # pragma: no cover - fallback for partial checkouts
    KinematicsCalculator = None


JOINTS = ("j1", "j2", "j3", "j4")
LAYERS = ("unity_raw", "unity_compensated", "ros_cmd", "robot")
LATENCY_COLUMNS = (
    "network_delay_ms",
    "decision_delay_ms",
    "command_latency_ms",
    "robot_response_ms",
    "motion_time_ms",
    "motion_execution_ms",
    "true_end_to_end_ms",
)
LATENCY_EVENT_FILTERS = {
    "network_delay_ms": {"unity_target"},
    "decision_delay_ms": {"ros_command"},
    "command_latency_ms": {"latency_arrival"},
    "robot_response_ms": {"latency_arrival"},
    "motion_time_ms": {"latency_arrival"},
    "motion_execution_ms": {"latency_arrival"},
    "true_end_to_end_ms": {"latency_arrival"},
}
KPI_THRESHOLD_MM = 5.0


def _float(value) -> float:
    if value is None or value == "":
        return np.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as fh:
        data_lines = [
            line
            for line in fh
            if line.strip() and not line.lstrip().startswith("#")
        ]
    return list(csv.DictReader(io.StringIO("".join(data_lines))))


def _first_valid(values: Iterable[float]) -> float:
    for value in values:
        if np.isfinite(value):
            return float(value)
    return 0.0


def _time_axis(rows: list[dict[str, str]]) -> np.ndarray:
    if not rows:
        return np.array([], dtype=float)

    elapsed = np.array([_float(r.get("elapsed_sec")) for r in rows], dtype=float)
    if np.isfinite(elapsed).any():
        return np.nan_to_num(elapsed, nan=0.0)

    timestamp = np.array([_float(r.get("Timestamp")) for r in rows], dtype=float)
    if np.isfinite(timestamp).any():
        first = _first_valid(timestamp)
        return np.nan_to_num(timestamp - first, nan=0.0)

    stamp_key = "ros_wall_timestamp"
    if stamp_key not in rows[0] and "ros_timestamp" in rows[0]:
        stamp_key = "ros_timestamp"
    stamps = np.array([_float(r.get(stamp_key)) for r in rows], dtype=float)
    first = _first_valid(stamps)
    return np.nan_to_num(stamps - first, nan=0.0)


def _joint_matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    values = np.full((len(rows), 4), np.nan, dtype=float)
    for row_idx, row in enumerate(rows):
        if prefix == "robot" and all(f"J{i}" in row for i in range(1, 5)):
            for joint_idx in range(1, 5):
                values[row_idx, joint_idx - 1] = np.radians(_float(row.get(f"J{joint_idx}")))
            continue

        for joint_idx, joint in enumerate(JOINTS, start=1):
            rad_key = f"{prefix}_{joint}_rad"
            deg_key = f"{prefix}_{joint}_deg"
            legacy_key = f"{prefix}_{joint_idx}_deg"
            legacy_unity_key = f"unity_{joint}_deg"
            if rad_key in row:
                values[row_idx, joint_idx - 1] = _float(row.get(rad_key))
            elif deg_key in row:
                values[row_idx, joint_idx - 1] = np.radians(_float(row.get(deg_key)))
            elif legacy_key in row:
                values[row_idx, joint_idx - 1] = np.radians(_float(row.get(legacy_key)))
            elif prefix in ("unity_raw", "unity_compensated") and legacy_unity_key in row:
                values[row_idx, joint_idx - 1] = np.radians(_float(row.get(legacy_unity_key)))

    if prefix == "unity_compensated" and np.isnan(values).all():
        return _joint_matrix(rows, "unity_raw")
    return values


def _fill_forward(values: np.ndarray) -> np.ndarray:
    out = values.copy()
    last = np.full(out.shape[1], np.nan)
    for i in range(out.shape[0]):
        mask = np.isfinite(out[i])
        last[mask] = out[i, mask]
        out[i, ~mask] = last[~mask]
    return out


def _finite_rows(*arrays: np.ndarray) -> np.ndarray:
    mask = np.ones(arrays[0].shape[0], dtype=bool)
    for arr in arrays:
        mask &= np.isfinite(arr).all(axis=1)
    return mask


def _norm_rows(values: np.ndarray) -> np.ndarray:
    return np.linalg.norm(values, axis=1)


def _event_type(row: dict[str, str]) -> str:
    if all(key in row for key in ("Timestamp", "J1", "X", "Y", "Z")):
        return "robot_feedback"
    return row.get("event_type") or "legacy_triple"


def _event_mask(rows: list[dict[str, str]], allowed: set[str]) -> np.ndarray:
    return np.array([_event_type(row) in allowed for row in rows], dtype=bool)


def _metric_values(rows: list[dict[str, str]], column: str) -> np.ndarray:
    allowed = LATENCY_EVENT_FILTERS.get(column)
    values = []
    for row in rows:
        if allowed is not None and _event_type(row) not in allowed:
            continue
        values.append(_float(row.get(column)))
    return np.asarray(values, dtype=float)


def _summary_stats(values: np.ndarray) -> dict[str, float | int | None]:
    valid = values[np.isfinite(values)]
    if valid.size == 0:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": int(valid.size),
        "mean": float(np.mean(valid)),
        "median": float(np.median(valid)),
        "p95": float(np.percentile(valid, 95)),
        "max": float(np.max(valid)),
    }


def _single_stat(value: float, unit: str) -> dict[str, float | int | None | str]:
    if not np.isfinite(value):
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None, "unit": unit}
    return {"count": 1, "mean": float(value), "median": float(value), "p95": float(value), "max": float(value), "unit": unit}


def _fk_xyz(joints_rad: np.ndarray) -> np.ndarray:
    xyz = np.full((joints_rad.shape[0], 3), np.nan, dtype=float)
    if KinematicsCalculator is None:
        return xyz
    kin = KinematicsCalculator()
    for i, q_rad in enumerate(joints_rad):
        if not np.isfinite(q_rad).all():
            continue
        try:
            pose = kin.forward_kinematics(np.degrees(q_rad))
            xyz[i] = np.asarray(pose[:3], dtype=float)
        except Exception:
            continue
    return xyz


def _tool_xyz_matrix(rows: list[dict[str, str]], prefix: str) -> np.ndarray:
    """Read raw MG400 ToolVector* XYZ columns when the logger captured them."""
    xyz = np.full((len(rows), 3), np.nan, dtype=float)
    keys = (f"{prefix}_x_mm", f"{prefix}_y_mm", f"{prefix}_z_mm")
    if not rows or not all(key in rows[0] for key in keys):
        return xyz
    for idx, row in enumerate(rows):
        xyz[idx] = [_float(row.get(key)) for key in keys]
    return xyz


def _save_summary_csv(path: Path, metrics: dict[str, dict]) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["metric", "count", "mean", "median", "p95", "max", "unit"])
        for name, stat in metrics.items():
            writer.writerow([
                name,
                stat.get("count", ""),
                _fmt(stat.get("mean")),
                _fmt(stat.get("median")),
                _fmt(stat.get("p95")),
                _fmt(stat.get("max")),
                stat.get("unit", ""),
            ])


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        if not np.isfinite(value):
            return ""
        return f"{float(value):.3f}"
    return str(value)


def _clean(values: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float)
    return arr[np.isfinite(arr)]


def _stats_dict(values: Iterable[float], unit: str = "") -> dict:
    valid = _clean(values)
    if valid.size == 0:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "median": None,
            "p95": None,
            "p99": None,
            "min": None,
            "max": None,
            "unit": unit,
        }
    p50 = float(np.percentile(valid, 50))
    return {
        "count": int(valid.size),
        "mean": float(np.mean(valid)),
        "p50": p50,
        "median": p50,
        "p95": float(np.percentile(valid, 95)),
        "p99": float(np.percentile(valid, 99)),
        "min": float(np.min(valid)),
        "max": float(np.max(valid)),
        "unit": unit,
    }


def _column_values(rows: list[dict[str, str]], column: str) -> np.ndarray:
    return np.asarray([_float(row.get(column)) for row in rows], dtype=float)


def _rows_with_event(rows: list[dict[str, str]], event: str) -> list[dict[str, str]]:
    return [row for row in rows if _event_type(row) == event]


def _counter(rows: list[dict[str, str]], column: str) -> dict[str, int]:
    counter: dict[str, int] = {}
    for row in rows:
        value = row.get(column)
        if value is None or value == "":
            continue
        counter[str(value)] = counter.get(str(value), 0) + 1
    return dict(sorted(counter.items(), key=lambda item: (-item[1], item[0])))


def _duration_rate_from_t(t: np.ndarray, rows_count: int) -> dict:
    valid = t[np.isfinite(t)]
    if valid.size == 0:
        return {"duration_sec": None, "effective_hz": None}
    duration = float(np.max(valid) - np.min(valid))
    rate = float(rows_count / duration) if duration > 0 else None
    return {"duration_sec": duration, "effective_hz": rate}


def _gap_stats_from_t(t: np.ndarray) -> dict:
    valid = t[np.isfinite(t)]
    if valid.size < 2:
        return _stats_dict([], "ms")
    gaps_ms = np.diff(valid) * 1000.0
    return _stats_dict(gaps_ms, "ms")


def _truthy(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _unity_sample_path_for_main(main_path: Path) -> Path | None:
    stem = main_path.stem.replace("teleop_session_", "", 1)
    candidates = [
        main_path.with_name(f"unity_teleop_sample_{stem}.csv"),
        main_path.with_name(f"unity_teleop_sample_from_teleop_session_{stem}.csv"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _file_info(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "modified_unix": path.stat().st_mtime,
    }


def _plot_joint_positions(t, joints, out: Path) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
    colors = {
        "unity_raw": "#9aa0a6",
        "unity_compensated": "#1f77b4",
        "ros_cmd": "#ff7f0e",
        "robot": "#2ca02c",
    }
    labels = {
        "unity_raw": "Unity raw",
        "unity_compensated": "Unity compensated",
        "ros_cmd": "ROS command",
        "robot": "Robot feedback",
    }
    for i, ax in enumerate(axes):
        for layer in LAYERS:
            data = np.degrees(joints[layer][:, i])
            if np.isfinite(data).any():
                ax.plot(t, data, label=labels[layer], color=colors[layer], linewidth=1.3)
        ax.set_ylabel(f"J{i + 1} (deg)")
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc="upper right", ncol=4)
    axes[-1].set_xlabel("time in session (s)")
    fig.suptitle("Joint Position by Layer")
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _plot_joint_error(t, joints, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(13, 5))
    cmd = joints["ros_cmd"]
    robot = joints["robot"]
    mask = _finite_rows(cmd, robot)
    plotted = False
    if mask.any():
        err_deg = np.degrees(cmd[mask] - robot[mask])
        for i in range(4):
            ax.plot(t[mask], err_deg[:, i], linewidth=1.1, label=f"J{i + 1}")
        ax.plot(t[mask], np.max(np.abs(err_deg), axis=1), color="black", linewidth=1.5, label="max abs")
        plotted = True
    else:
        ax.text(0.5, 0.5, "No ROS command + robot feedback overlap", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("ROS Command -> Robot Feedback Joint Error")
    ax.set_xlabel("time in session (s)")
    ax.set_ylabel("error (deg)")
    ax.grid(True, alpha=0.25)
    if plotted:
        ax.legend(ncol=5)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _plot_latency(rows, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    names = []
    means = []
    p95s = []
    for col in LATENCY_COLUMNS:
        vals = _metric_values(rows, col)
        stat = _summary_stats(vals)
        if stat["count"]:
            names.append(col.replace("_ms", "").replace("_", "\n"))
            means.append(stat["mean"])
            p95s.append(stat["p95"])
    x = np.arange(len(names))
    if names:
        ax.bar(x - 0.18, means, width=0.36, label="mean", color="#4c78a8")
        ax.bar(x + 0.18, p95s, width=0.36, label="p95", color="#f58518")
    else:
        ax.text(0.5, 0.5, "No T1-T5 latency rows in this log", ha="center", va="center", transform=ax.transAxes)
    ax.set_xticks(x, names)
    ax.set_ylabel("milliseconds")
    ax.set_title("Latency Breakdown")
    ax.grid(True, axis="y", alpha=0.25)
    if names:
        ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _plot_cartesian(t, xyz, out_xyz: Path, out_error: Path, out_3d: Path) -> None:
    colors = {
        "unity_compensated": "#1f77b4",
        "ros_cmd": "#ff7f0e",
        "robot": "#2ca02c",
    }
    fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
    for axis_idx, name in enumerate(("X", "Y", "Z")):
        ax = axes[axis_idx]
        for layer, color in colors.items():
            data = xyz[layer][:, axis_idx]
            if np.isfinite(data).any():
                ax.plot(t, data, color=color, linewidth=1.2, label=layer)
        ax.set_ylabel(f"{name} (mm)")
        ax.grid(True, alpha=0.25)
    axes[0].legend(ncol=3)
    axes[-1].set_xlabel("time in session (s)")
    fig.suptitle("Cartesian TCP Position by Layer")
    fig.tight_layout()
    fig.savefig(out_xyz, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 5))
    pairs = [
        ("unity_compensated", "robot", "Unity compensated -> Robot"),
        ("ros_cmd", "robot", "ROS command -> Robot"),
    ]
    plotted = False
    for a, b, label in pairs:
        mask = _finite_rows(xyz[a], xyz[b])
        if mask.any():
            err = _norm_rows(xyz[a][mask] - xyz[b][mask])
            ax.plot(t[mask], err, linewidth=1.3, label=label)
            plotted = True
    if not plotted:
        ax.text(0.5, 0.5, "No Cartesian layer overlap for error plot", ha="center", va="center", transform=ax.transAxes)
    ax.set_title("Cartesian Tracking Error")
    ax.set_xlabel("time in session (s)")
    ax.set_ylabel("3D error (mm)")
    ax.grid(True, alpha=0.25)
    if plotted:
        ax.legend()
    fig.tight_layout()
    fig.savefig(out_error, dpi=180)
    plt.close(fig)

    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection="3d")
    for layer, color in colors.items():
        data = xyz[layer]
        mask = np.isfinite(data).all(axis=1)
        if mask.any():
            ax.plot(data[mask, 0], data[mask, 1], data[mask, 2], color=color, linewidth=1.6, label=layer)
            ax.scatter(data[mask, 0][0], data[mask, 1][0], data[mask, 2][0], color=color, marker="o", s=24)
            ax.scatter(data[mask, 0][-1], data[mask, 1][-1], data[mask, 2][-1], color=color, marker="s", s=24)
    ax.set_title("3D TCP Path")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_3d, dpi=180)
    plt.close(fig)


def _active_window(rows: list[dict[str, str]], t: np.ndarray) -> tuple[float, float]:
    active_events = {"unity_target", "ros_command", "latency_arrival"}
    mask = _event_mask(rows, active_events)
    if mask.any():
        active_t = t[mask]
        active_t = active_t[np.isfinite(active_t)]
        if active_t.size:
            return max(0.0, float(np.min(active_t)) - 2.0), float(np.max(active_t)) + 2.0
    finite_t = t[np.isfinite(t)]
    if finite_t.size:
        return float(np.min(finite_t)), float(np.max(finite_t))
    return 0.0, 1.0


def _downsample_indices(mask: np.ndarray, max_points: int = 1800) -> np.ndarray:
    indices = np.flatnonzero(mask)
    if indices.size <= max_points:
        return indices
    step = int(np.ceil(indices.size / max_points))
    return indices[::step]


def _plot_report_latency(metrics: dict, out: Path) -> None:
    items = [
        ("network_delay_ms", "Unity -> ROS\nexcess"),
        ("decision_delay_ms", "ROS decision"),
        ("robot_response_ms", "Robot response"),
        ("true_end_to_end_ms", "End-to-end reached"),
    ]
    labels = []
    medians = []
    p95s = []
    for key, label in items:
        stat = metrics.get(key) or {}
        if stat.get("count"):
            labels.append(label)
            medians.append(float(stat.get("median") or 0.0))
            p95s.append(float(stat.get("p95") or 0.0))

    fig, ax = plt.subplots(figsize=(10, 4.8))
    y = np.arange(len(labels))
    if labels:
        ax.barh(y - 0.17, medians, height=0.34, label="median", color="#2f6f9f")
        ax.barh(y + 0.17, p95s, height=0.34, label="p95", color="#f28e2b")
        for yi, value in zip(y - 0.17, medians):
            ax.text(value + max(p95s) * 0.015, yi, f"{value:.0f} ms", va="center", fontsize=9)
        for yi, value in zip(y + 0.17, p95s):
            ax.text(value + max(p95s) * 0.015, yi, f"{value:.0f} ms", va="center", fontsize=9)
    ax.set_yticks(y, labels)
    ax.invert_yaxis()
    ax.set_xlabel("milliseconds")
    ax.set_title("Latency Summary")
    ax.grid(True, axis="x", alpha=0.25)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    plt.close(fig)


def _stat_value(metrics: dict, key: str, field: str) -> float | None:
    value = (metrics.get(key) or {}).get(field)
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(value):
        return None
    return value


def _plot_report_latency_split(metrics: dict, out: Path) -> None:
    items = [
        ("network_delay_ms", "Unity -> ROS excess"),
        ("decision_delay_ms", "ROS compute"),
        ("robot_response_ms", "Robot response"),
        ("true_end_to_end_ms", "End-to-end"),
    ]
    labels = [label for _, label in items]
    medians = [_stat_value(metrics, key, "median") for key, _ in items]
    p95s = [_stat_value(metrics, key, "p95") for key, _ in items]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    for ax, values, title, color in (
        (axes[0], medians, "Typical case (median)", "#2f6f9f"),
        (axes[1], p95s, "Worst-case tail (p95)", "#f28e2b"),
    ):
        y = np.arange(len(labels))
        clean = [0.0 if value is None else value for value in values]
        ax.barh(y, clean, color=color)
        for yi, value in zip(y, values):
            label = "n/a" if value is None else f"{value:.0f} ms"
            ax.text((value or 0.0) + max(clean or [1]) * 0.03, yi, label, va="center", fontsize=12)
        ax.set_yticks(y, labels)
        ax.invert_yaxis()
        ax.set_title(title, fontsize=15, weight="bold")
        ax.set_xlabel("milliseconds")
        ax.grid(True, axis="x", alpha=0.22)
        xmax = max(clean or [1]) * 1.25
        ax.set_xlim(0, xmax if xmax > 0 else 1)
    fig.suptitle("Latency: Normal Use vs Tail Lag", fontsize=18, weight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(out, dpi=240)
    plt.close(fig)


def _plot_report_accuracy(metrics: dict, out: Path) -> None:
    items = [
        ("target_reached_joint_error", "Joint error\nat target", "deg"),
        ("target_reached_tcp_error_ros_cmd_to_robot", "TCP error\nat target", "mm"),
    ]
    labels = []
    means = []
    p95s = []
    maxs = []
    units = []
    for key, label, unit in items:
        stat = metrics.get(key) or {}
        if stat.get("count"):
            labels.append(label)
            means.append(float(stat.get("mean") or 0.0))
            p95s.append(float(stat.get("p95") or 0.0))
            maxs.append(float(stat.get("max") or 0.0))
            units.append(unit)

    fig, axes = plt.subplots(1, max(1, len(labels)), figsize=(9.5, 4.8))
    if not isinstance(axes, np.ndarray):
        axes = [axes]
    if not labels:
        axes[0].text(0.5, 0.5, "No target-reached accuracy samples", ha="center", va="center")
    for ax, label, mean, p95, max_value, unit in zip(axes, labels, means, p95s, maxs, units):
        vals = [mean, p95, max_value]
        bars = ax.bar(["mean", "p95", "max"], vals, color=["#59a14f", "#f28e2b", "#e15759"])
        for bar, value in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.2f} {unit}", ha="center", va="bottom", fontsize=10)
        ax.set_title(label)
        ax.set_ylabel(unit)
        ax.grid(True, axis="y", alpha=0.25)
        ax.set_ylim(0, max(vals) * 1.35 if max(vals) > 0 else 1)
    fig.suptitle("Target-Reached Accuracy")
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    plt.close(fig)


def _plot_report_poster_summary(metrics: dict, out: Path, robot_tcp_source: str) -> None:
    latency_cards = [
        ("Unity -> ROS excess", _stat_value(metrics, "network_delay_ms", "median"), "ms"),
        ("ROS compute", _stat_value(metrics, "decision_delay_ms", "median"), "ms"),
        ("Robot response", _stat_value(metrics, "robot_response_ms", "median"), "ms"),
        ("End-to-end", _stat_value(metrics, "true_end_to_end_ms", "median"), "ms"),
    ]
    accuracy_cards = [
        ("Joint final error", _stat_value(metrics, "target_reached_joint_error", "mean"), "deg"),
        ("TCP final error", _stat_value(metrics, "target_reached_tcp_error_ros_cmd_to_robot", "mean"), "mm"),
    ]
    tail = _stat_value(metrics, "true_end_to_end_ms", "p95")

    fig = plt.figure(figsize=(13, 7.2))
    gs = fig.add_gridspec(2, 4, height_ratios=[1.0, 0.82], hspace=0.34, wspace=0.22)
    fig.suptitle("MG400 Teleoperation Performance", fontsize=24, weight="bold")
    fig.text(0.5, 0.91, "Real session log: Unity command -> ROS decision -> robot feedback", ha="center", fontsize=13, color="#555")

    for idx, (title, value, unit) in enumerate(latency_cards):
        ax = fig.add_subplot(gs[0, idx])
        ax.set_facecolor("#f7f8fa")
        for spine in ax.spines.values():
            spine.set_color("#cdd3da")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.text(0.06, 0.78, title, transform=ax.transAxes, fontsize=14, weight="bold")
        ax.text(0.06, 0.54, "median", transform=ax.transAxes, fontsize=11, color="#666")
        if value is None:
            text = "n/a"
        else:
            text = f"{value:.0f} {unit}"
        ax.text(0.06, 0.24, text, transform=ax.transAxes, fontsize=29, weight="bold", color="#1f4e79")

    ax_acc = fig.add_subplot(gs[1, 0:2])
    names = [x[0] for x in accuracy_cards]
    vals = [0.0 if x[1] is None else x[1] for x in accuracy_cards]
    units = [x[2] for x in accuracy_cards]
    bars = ax_acc.bar(names, vals, color=["#59a14f", "#4e79a7"], width=0.55)
    for bar, value, unit in zip(bars, vals, units):
        ax_acc.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.2f} {unit}", ha="center", va="bottom", fontsize=14, weight="bold")
    ax_acc.set_title("Replay accuracy at reached targets", fontsize=15, weight="bold")
    ax_acc.grid(True, axis="y", alpha=0.22)
    ax_acc.set_ylim(0, max(vals + [1.0]) * 1.35)

    ax_note = fig.add_subplot(gs[1, 2:4])
    ax_note.set_facecolor("#fff7e8")
    for spine in ax_note.spines.values():
        spine.set_color("#dfb56b")
    ax_note.set_xticks([])
    ax_note.set_yticks([])
    tail_text = "n/a" if tail is None else f"{tail:.0f} ms"
    ax_note.text(0.05, 0.72, "Important note", transform=ax_note.transAxes, fontsize=16, weight="bold")
    ax_note.text(0.05, 0.48, f"p95 end-to-end tail: {tail_text}", transform=ax_note.transAxes, fontsize=15, color="#8a4b00")
    ax_note.text(0.05, 0.27, "Typical response is fast, but this log still has occasional backlog/tail lag.", transform=ax_note.transAxes, fontsize=12, color="#444")
    ax_note.text(0.05, 0.10, f"TCP source here: {robot_tcp_source}", transform=ax_note.transAxes, fontsize=10, color="#666")

    fig.savefig(out, dpi=240, bbox_inches="tight")
    plt.close(fig)


def _plot_report_joint_active(t, rows, joints, out: Path) -> None:
    start, end = _active_window(rows, t)
    active = (t >= start) & (t <= end)
    fig, axes = plt.subplots(4, 1, figsize=(12, 8.5), sharex=True)
    for joint_idx, ax in enumerate(axes):
        robot_mask = active & np.isfinite(joints["robot"][:, joint_idx])
        cmd_mask = active & np.isfinite(joints["ros_cmd"][:, joint_idx])
        unity_mask = active & np.isfinite(joints["unity_compensated"][:, joint_idx])

        robot_idx = _downsample_indices(robot_mask, 1400)
        unity_idx = _downsample_indices(unity_mask, 700)
        cmd_idx = _downsample_indices(cmd_mask, 700)

        if unity_idx.size:
            ax.plot(t[unity_idx], np.degrees(joints["unity_compensated"][unity_idx, joint_idx]), color="#9aa0a6", linewidth=0.9, alpha=0.55, label="Unity target")
        if cmd_idx.size:
            ax.step(t[cmd_idx], np.degrees(joints["ros_cmd"][cmd_idx, joint_idx]), where="post", color="#f28e2b", linewidth=1.3, label="ROS command")
        if robot_idx.size:
            ax.plot(t[robot_idx], np.degrees(joints["robot"][robot_idx, joint_idx]), color="#2ca02c", linewidth=1.4, label="Robot feedback")
        ax.set_ylabel(f"J{joint_idx + 1}\n(deg)")
        ax.grid(True, alpha=0.22)
    axes[0].legend(loc="upper right", ncol=3)
    axes[-1].set_xlabel("session time (s), active window only")
    fig.suptitle("Joint Tracking During Active Teleop Window")
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    plt.close(fig)


def _plot_report_tcp_views(t, rows, xyz, out: Path, robot_tcp_source: str) -> None:
    start, end = _active_window(rows, t)
    active = (t >= start) & (t <= end)
    views = [
        ("Top view (X-Y)", 0, 1, "X (mm)", "Y (mm)"),
        ("Front view (X-Z)", 0, 2, "X (mm)", "Z (mm)"),
        ("Side view (Y-Z)", 1, 2, "Y (mm)", "Z (mm)"),
    ]
    colors = {"unity_compensated": "#9aa0a6", "ros_cmd": "#f28e2b", "robot": "#2ca02c"}
    labels = {"unity_compensated": "Unity target", "ros_cmd": "ROS command", "robot": "Robot feedback"}

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
    for ax, (title, a, b, xlabel, ylabel) in zip(axes, views):
        for layer in ("unity_compensated", "ros_cmd", "robot"):
            data = xyz[layer]
            mask = active & np.isfinite(data).all(axis=1)
            idx = _downsample_indices(mask, 1200)
            if idx.size:
                ax.plot(data[idx, a], data[idx, b], color=colors[layer], linewidth=1.5, alpha=0.88, label=labels[layer])
                ax.scatter(data[idx, a][0], data[idx, b][0], color=colors[layer], marker="o", s=26)
                ax.scatter(data[idx, a][-1], data[idx, b][-1], color=colors[layer], marker="s", s=26)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.set_aspect("equal", adjustable="datalim")
    axes[0].legend(loc="best")
    fig.suptitle(f"TCP Path Views ({robot_tcp_source})")
    fig.tight_layout()
    fig.savefig(out, dpi=220)
    plt.close(fig)


def _plot_report_metrics_card(metrics: dict, out: Path, robot_tcp_source: str) -> None:
    cards = [
        ("Unity -> ROS excess", "median", "network_delay_ms", "median", "ms"),
        ("ROS decision", "median", "decision_delay_ms", "median", "ms"),
        ("Robot response", "median", "robot_response_ms", "median", "ms"),
        ("End-to-end", "median", "true_end_to_end_ms", "median", "ms"),
        ("Joint error", "mean", "target_reached_joint_error", "mean", "deg"),
        ("TCP error", "mean", "target_reached_tcp_error_ros_cmd_to_robot", "mean", "mm"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.8))
    axes = axes.ravel()
    for ax, (title, subtitle, key, field, unit) in zip(axes, cards):
        stat = metrics.get(key) or {}
        value = stat.get(field)
        ax.set_facecolor("#f7f8fa")
        for spine in ax.spines.values():
            spine.set_color("#d0d4da")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.text(0.05, 0.82, title, transform=ax.transAxes, fontsize=16, weight="bold", va="top")
        ax.text(0.05, 0.62, subtitle, transform=ax.transAxes, fontsize=11, color="#555", va="top")
        if value is None:
            text = "n/a"
        elif unit == "ms":
            text = f"{float(value):.0f} ms"
        elif unit == "deg":
            text = f"{float(value):.2f} deg"
        else:
            text = f"{float(value):.2f} {unit}"
        ax.text(0.05, 0.28, text, transform=ax.transAxes, fontsize=26, weight="bold", color="#1f4e79", va="center")
    fig.suptitle("Teleoperation Performance Snapshot", fontsize=20, weight="bold")
    fig.text(0.5, 0.03, f"Robot TCP source for this analysis: {robot_tcp_source}", ha="center", fontsize=10, color="#555")
    fig.tight_layout(rect=(0, 0.05, 1, 0.94))
    fig.savefig(out, dpi=220)
    plt.close(fig)


def _write_cartesian_csv(path: Path, t, xyz) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        header = ["elapsed_sec"]
        for layer in ("unity_compensated", "ros_cmd", "robot"):
            header += [f"{layer}_x_mm", f"{layer}_y_mm", f"{layer}_z_mm"]
        header += ["error_unity_to_robot_mm", "error_ros_cmd_to_robot_mm"]
        writer.writerow(header)
        for idx in range(len(t)):
            row = [f"{t[idx]:.6f}"]
            for layer in ("unity_compensated", "ros_cmd", "robot"):
                row.extend(_fmt(v) for v in xyz[layer][idx])
            err_unity = np.nan
            if np.isfinite(xyz["unity_compensated"][idx]).all() and np.isfinite(xyz["robot"][idx]).all():
                err_unity = float(np.linalg.norm(xyz["unity_compensated"][idx] - xyz["robot"][idx]))
            err_cmd = np.nan
            if np.isfinite(xyz["ros_cmd"][idx]).all() and np.isfinite(xyz["robot"][idx]).all():
                err_cmd = float(np.linalg.norm(xyz["ros_cmd"][idx] - xyz["robot"][idx]))
            row.extend([_fmt(err_unity), _fmt(err_cmd)])
            writer.writerow(row)


def _plot_plate_pair(
    t: np.ndarray,
    rows: list[dict[str, str]],
    xyz: dict[str, np.ndarray],
    target_layer: str,
    actual_layer: str,
    title: str,
    out: Path,
    *,
    threshold_mm: float = KPI_THRESHOLD_MM,
) -> dict:
    start, end = _active_window(rows, t)
    target = xyz[target_layer]
    actual = xyz[actual_layer]
    mask = (t >= start) & (t <= end) & np.isfinite(target).all(axis=1) & np.isfinite(actual).all(axis=1)
    idx = _downsample_indices(mask, 3500)
    full_idx = np.flatnonzero(mask)
    summary = {
        "file": out.stem,
        "target_layer": target_layer,
        "actual_layer": actual_layer,
        "row_count": int(full_idx.size),
        "kpi_threshold_mm": float(threshold_mm),
        "points_over_kpi_5mm": 0,
        "peak_sample_index": None,
        "peak_vector_error_mm": None,
        "peak_dx_mm": None,
        "peak_dy_mm": None,
        "peak_dz_mm": None,
    }
    fig, ax = plt.subplots(figsize=(10, 7.5))
    ax.set_facecolor("#eef4fb")
    if full_idx.size:
        diff = actual[full_idx] - target[full_idx]
        err = _norm_rows(diff)
        peak_pos = int(np.argmax(err))
        peak_idx = int(full_idx[peak_pos])
        over = err > threshold_mm
        summary.update(
            {
                "points_over_kpi_5mm": int(np.count_nonzero(over)),
                "peak_sample_index": peak_idx,
                "peak_vector_error_mm": float(err[peak_pos]),
                "peak_dx_mm": float(diff[peak_pos, 0]),
                "peak_dy_mm": float(diff[peak_pos, 1]),
                "peak_dz_mm": float(diff[peak_pos, 2]),
            }
        )
        ax.plot(target[idx, 0], target[idx, 1], color="#f28e8e", linewidth=3.0, alpha=0.9, label="Target trajectory")
        ax.plot(actual[idx, 0], actual[idx, 1], color="#0b61ff", linewidth=2.2, alpha=0.95, label="Actual trajectory")
        over_idx = full_idx[over]
        over_idx = over_idx[:: max(1, int(np.ceil(over_idx.size / 250)))] if over_idx.size else over_idx
        if over_idx.size:
            ax.scatter(actual[over_idx, 0], actual[over_idx, 1], s=18, color="#d97900", edgecolor="white", linewidth=0.45, label=f"3D error > KPI ({threshold_mm:g} mm)")
        ax.scatter(actual[peak_idx, 0], actual[peak_idx, 1], s=110, marker="D", color="#b00000", edgecolor="white", linewidth=0.9, label=f"Peak row {peak_idx}")
        ax.set_title(f"{title} - plate-view trajectory | Points > KPI ({threshold_mm:g} mm): {summary['points_over_kpi_5mm']} | peak {summary['peak_vector_error_mm']:.3f} mm", fontsize=14)
    else:
        ax.text(0.5, 0.5, "No overlapping XYZ samples for this layer pair", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(f"{title} - plate-view trajectory | no overlap", fontsize=14)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.18)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.16), ncol=4, frameon=False)
    fig.tight_layout()
    fig.savefig(out, dpi=220, bbox_inches="tight")
    plt.close(fig)
    return summary


def _plot_plate_contact_sheet(figures: dict[str, Path], out: Path) -> None:
    items = [
        ("Plot_VR", figures["plate_vr"]),
        ("PLOT_UNITY_VS_ROS", figures["plate_unity_vs_ros"]),
        ("PLOT_UNITY_VS_ROBOT", figures["plate_unity_vs_robot"]),
        ("PLOT_ROS_VS_ROBOT_MANUAL", figures["plate_ros_vs_robot"]),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Teleop plate-view layer comparison", fontsize=18, weight="bold")
    for ax, (label, image_path) in zip(axes.ravel(), items):
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(label, loc="left", fontsize=10)
        if image_path.exists():
            ax.imshow(plt.imread(image_path))
        for spine in ax.spines.values():
            spine.set_color("#c9cdd2")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=180)
    plt.close(fig)


def _write_plate_summary_csv(path: Path, summaries: list[dict]) -> None:
    fields = [
        "file",
        "target_layer",
        "actual_layer",
        "row_count",
        "kpi_threshold_mm",
        "points_over_kpi_5mm",
        "peak_sample_index",
        "peak_vector_error_mm",
        "peak_dx_mm",
        "peak_dy_mm",
        "peak_dz_mm",
    ]
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for row in summaries:
            writer.writerow({field: row.get(field, "") for field in fields})


def _build_session_report(
    main_path: Path,
    rows: list[dict[str, str]],
    t: np.ndarray,
    unity_path: Path | None,
    unity_rows: list[dict[str, str]],
) -> dict:
    """Assemble the full ``report`` dict consumed by the markdown
    summary, the JSON output, and the all-metrics CSV.

    The result has two top-level keys:
      ``main``  — derived from the session CSV (rows + event-type
                  subsets + anomalies + the Unity-event window in
                  main time).
      ``unity`` — derived from the matching Unity sample CSV (seq
                  integrity + cross-file seq matching + control /
                  filter status tallies).
    """
    feedback_rows = _rows_with_event(rows, "robot_feedback")
    feedback_t = _time_axis(feedback_rows) if feedback_rows else np.array([], dtype=float)
    unity_event_rows = _rows_with_event(rows, "unity_sample")
    unity_event_t = (
        _time_axis(unity_event_rows) if unity_event_rows else np.array([], dtype=float)
    )
    command_rows = _rows_with_event(rows, "ros_command")
    arrival_rows = _rows_with_event(rows, "latency_arrival")

    main_report = _main_report_metadata(main_path, rows, t)
    main_report["robot_feedback"] = _main_report_robot_feedback(feedback_rows, feedback_t)
    main_report["commands"] = _main_report_commands(command_rows)
    main_report["latency_arrival"] = _main_report_latency_arrival(arrival_rows)
    main_report["accuracy"] = _main_report_accuracy(rows, feedback_rows)
    main_report["anomalies"] = _main_report_anomalies(rows, feedback_rows)
    if unity_event_t.size:
        main_report["unity_window_in_main"] = _main_report_unity_window(
            t, unity_event_t, rows
        )

    return {"main": main_report, "unity": _build_unity_report(unity_path, unity_rows, rows)}


def _main_report_metadata(main_path: Path, rows, t: np.ndarray) -> dict:
    """File info + row/column counts + duration/rate + per-row gap
    stats + event-type tally sorted by descending frequency.
    """
    event_counts = dict(collections.Counter(_event_type(row) for row in rows))
    event_counts = dict(sorted(event_counts.items(), key=lambda item: (-item[1], item[0])))
    return {
        "file": _file_info(main_path),
        "rows": len(rows),
        "columns": len(rows[0]) if rows else 0,
        "duration_rate": _duration_rate_from_t(t, len(rows)),
        "gap_ms": _gap_stats_from_t(t),
        "event_counts": event_counts,
        "unique_ros_command_uid": len(
            {r.get("ros_command_uid") for r in rows if r.get("ros_command_uid")}
        ),
    }


def _main_report_robot_feedback(feedback_rows, feedback_t) -> dict:
    """Robot-feedback rate + robot_mode / error_status tallies +
    nonzero-error row count.
    """
    return {
        "duration_rate": _duration_rate_from_t(feedback_t, len(feedback_rows)),
        "robot_mode_counts": _counter(feedback_rows, "robot_mode"),
        "error_status_counts": _counter(feedback_rows, "error_status"),
        "nonzero_error_status_rows": sum(
            1 for row in feedback_rows if row.get("error_status") not in ("", "0", "0.0")
        ),
    }


def _main_report_commands(command_rows) -> dict:
    """Command count + send_reason tally + time-since-last /
    network-delay / decision-delay stats.
    """
    return {
        "count": len(command_rows),
        "send_reason_counts": _counter(command_rows, "send_reason"),
        "time_since_last_cmd_ms": _stats_dict(
            _column_values(command_rows, "time_since_last_cmd_ms"), "ms"
        ),
        "network_delay_ms": _stats_dict(_column_values(command_rows, "network_delay_ms"), "ms"),
        "decision_delay_ms": _stats_dict(
            _column_values(command_rows, "decision_delay_ms"), "ms"
        ),
    }


def _main_report_latency_arrival(arrival_rows) -> dict:
    """T4/T5 arrival latency: command_latency + robot_response +
    true_end_to_end.
    """
    return {
        "count": len(arrival_rows),
        "command_latency_ms": _stats_dict(
            _column_values(arrival_rows, "command_latency_ms"), "ms"
        ),
        "robot_response_ms": _stats_dict(
            _column_values(arrival_rows, "robot_response_ms"), "ms"
        ),
        "true_end_to_end_ms": _stats_dict(
            _column_values(arrival_rows, "true_end_to_end_ms"), "ms"
        ),
    }


def _main_report_accuracy(rows, feedback_rows) -> dict:
    """Joint + tool error stats (full row set + feedback-only subset).

    The two "feedback-only" tool errors use the same column as a
    full-row scan because the column is only populated on
    robot_feedback events, but reading from the smaller list keeps
    the stats correct when other event types accumulate NaNs.
    """
    return {
        "final_error_rad": _stats_dict(_column_values(rows, "final_error_rad"), "rad"),
        "max_joint_error_rad": _stats_dict(_column_values(rows, "max_joint_error_rad"), "rad"),
        "match_tool_error_ros_cmd_to_robot_actual_mm": _stats_dict(
            _column_values(rows, "error_ros_cmd_tool_to_robot_actual_mm"), "mm"
        ),
        "robot_target_to_actual_error_mm": _stats_dict(
            _column_values(feedback_rows, "error_robot_tool_target_to_actual_mm"), "mm"
        ),
        "active_ros_cmd_to_actual_tool_error_mm": _stats_dict(
            _column_values(feedback_rows, "error_ros_cmd_tool_to_robot_actual_mm"), "mm"
        ),
    }


def _main_report_anomalies(rows, feedback_rows) -> dict:
    """Threshold-violation counts the markdown summary flags: stale
    Unity samples, ambiguous settle matches, pending-command depth,
    queue backlog, and the >10mm robot internal target error.
    """
    queue_backlog = _column_values(rows, "queue_backlog_rad")
    pending = _column_values(rows, "pending_command_count")
    stale_age = _column_values(rows, "unity_sample_age_ms")
    feedback_target_err = (
        _column_values(feedback_rows, "error_robot_tool_target_to_actual_mm")
        if feedback_rows else np.array([], dtype=float)
    )
    return {
        "stale_unity_sample_age_gt_100ms_rows": int(
            np.count_nonzero(stale_age[np.isfinite(stale_age)] > 100.0)
        ),
        "ambiguous_settle_rows": sum(
            1 for row in rows if _truthy(row.get("settle_match_ambiguous"))
        ),
        "pending_command_rows": int(np.count_nonzero(pending[np.isfinite(pending)] > 0)),
        "max_pending_command_count": (
            float(np.nanmax(pending)) if np.isfinite(pending).any() else None
        ),
        "queue_backlog_gt_0p01rad_rows": int(
            np.count_nonzero(queue_backlog[np.isfinite(queue_backlog)] > 0.01)
        ),
        "robot_internal_target_error_gt_10mm_rows": (
            int(np.count_nonzero(feedback_target_err[np.isfinite(feedback_target_err)] > 10.0))
            if feedback_rows else 0
        ),
    }


def _main_report_unity_window(t: np.ndarray, unity_event_t: np.ndarray, rows) -> dict:
    """How much of the main log sits outside the [first, last] Unity-
    sample window, plus a tally of which event types fall after the
    last Unity sample. Used to flag teleop activity that drifted past
    the Unity session.
    """
    before = t < float(np.min(unity_event_t))
    after = t > float(np.max(unity_event_t))
    t_finite_any = np.isfinite(t).any()
    return {
        "main_rows_before_first_unity": int(np.count_nonzero(before)),
        "sec_before_first_unity": (
            float(np.min(unity_event_t) - np.min(t[np.isfinite(t)])) if t_finite_any else None
        ),
        "main_rows_after_last_unity": int(np.count_nonzero(after)),
        "sec_after_last_unity": (
            float(np.max(t[np.isfinite(t)]) - np.max(unity_event_t)) if t_finite_any else None
        ),
        "events_after_last_unity": dict(
            collections.Counter(_event_type(rows[i]) for i in np.flatnonzero(after))
        ),
    }


def _build_unity_report(unity_path: Path | None, unity_rows, rows) -> dict:
    """Unity CSV summary: row count + (when rows exist) duration rate,
    seq integrity, control/filter status tallies, and the cross-file
    seq matching against the main log.
    """
    unity_report = {
        "file": _file_info(unity_path),
        "rows": len(unity_rows),
        "columns": len(unity_rows[0]) if unity_rows else 0,
    }
    if not unity_rows:
        return unity_report

    unity_t = _column_values(unity_rows, "unity_send_ts")
    if not np.isfinite(unity_t).any():
        unity_t = _column_values(unity_rows, "controller_capture_ts")
    seq = _column_values(unity_rows, "unity_seq_id")
    valid_seq = seq[np.isfinite(seq)].astype(int)
    seq_missing = 0
    duplicates = 0
    if valid_seq.size:
        seq_missing = int(
            (int(np.max(valid_seq)) - int(np.min(valid_seq)) + 1) - len(set(valid_seq.tolist()))
        )
        duplicates = int(valid_seq.size - len(set(valid_seq.tolist())))
    main_seq = {
        _float(row.get("unity_seq_id"))
        for row in rows
        if row.get("unity_seq_id") not in (None, "")
    }
    main_seq_int = {int(v) for v in main_seq if np.isfinite(v)}
    unity_seq_int = set(valid_seq.tolist())
    unity_report.update(
        {
            "duration_rate": _duration_rate_from_t(unity_t, len(unity_rows)),
            "gap_ms": _gap_stats_from_t(unity_t),
            "seq_min": int(np.min(valid_seq)) if valid_seq.size else None,
            "seq_max": int(np.max(valid_seq)) if valid_seq.size else None,
            "missing_seq": seq_missing,
            "duplicates": duplicates,
            "valid_true_rows": sum(
                1 for row in unity_rows if _truthy(row.get("is_valid", "true"))
            ),
            "control_mode_counts": _counter(unity_rows, "control_mode"),
            "filter_status_counts": _counter(unity_rows, "unity_filter_status"),
            "seq_cross_file_matched": len(main_seq_int & unity_seq_int),
            "seq_cross_file_missing": len(unity_seq_int - main_seq_int),
        }
    )
    return unity_report


def _write_all_metrics_csv(path: Path, report: dict, metrics: dict, plate_summaries: list[dict]) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["section", "metric", "field", "value", "unit"])
        for key, stat in metrics.items():
            unit = stat.get("unit", "")
            for field in ("count", "mean", "median", "p95", "p99", "min", "max"):
                if field in stat:
                    writer.writerow(["metric", key, field, _fmt(stat.get(field)), unit])
        for section, data in report.items():
            _write_flat_report_rows(writer, section, data)
        for summary in plate_summaries:
            for key, value in summary.items():
                writer.writerow(["plate_view", summary.get("file", ""), key, _fmt(value), ""])


def _write_flat_report_rows(writer, prefix: str, data) -> None:
    if isinstance(data, dict):
        for key, value in data.items():
            _write_flat_report_rows(writer, f"{prefix}.{key}", value)
    else:
        writer.writerow(["report", prefix, "value", _fmt(data), ""])


def analyze_file(path: Path, out_root: Path | None = None) -> Path:
    """Run the full teleop-session analysis pipeline for ``path``.

    Pipeline (each phase is its own helper):
      1. Load CSV + matching Unity sample file, prepare output dir.
      2. Build joint/XYZ matrices for every layer; determine which
         feedback source the robot TCP trace came from.
      3. Aggregate latency / accuracy / motion metrics.
      4. Count event and operation_mode rows for the markdown summary.
      5. Render every figure (joint, latency, cartesian, report
         posters, plate pairs, contact sheet).
      6. Build the prose session report.
      7. Write CSV / JSON / markdown artefacts.
    """
    rows, unity_path, unity_rows, out_dir, t = _load_session_data(path, out_root)
    joints, xyz, robot_tcp_source = _build_position_matrices(rows)
    metrics = _compute_session_metrics(rows, joints, xyz, t)
    event_counts, mode_counts = _count_events_and_modes(rows)

    figures, plate_summaries = _render_all_figures(
        out_dir, t, rows, joints, xyz, metrics, robot_tcp_source
    )
    report = _build_session_report(path, rows, t, unity_path, unity_rows)

    _write_session_artifacts(
        out_dir=out_dir,
        path=path,
        rows=rows,
        unity_path=unity_path,
        t=t,
        xyz=xyz,
        metrics=metrics,
        report=report,
        figures=figures,
        robot_tcp_source=robot_tcp_source,
        event_counts=event_counts,
        mode_counts=mode_counts,
        plate_summaries=plate_summaries,
    )
    return out_dir


def _load_session_data(path: Path, out_root: Path | None):
    """Read the session CSV (and the matching Unity sample CSV when one
    exists), allocate the output directory, and compute the shared
    time axis. Raises if the session CSV is empty.
    """
    rows = _read_csv(path)
    if not rows:
        raise ValueError(f"{path} has no rows")

    out_dir = out_root or (path.parent / f"{path.stem}_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)
    unity_path = _unity_sample_path_for_main(path)
    unity_rows = _read_csv(unity_path) if unity_path else []
    t = _time_axis(rows)
    return rows, unity_path, unity_rows, out_dir, t


def _build_position_matrices(rows):
    """Joint-angle and Cartesian XYZ matrices for every analysis layer.

    Returns ``(joints, xyz, robot_tcp_source)``. The robot TCP layer
    prefers (in order):
      1. MG400 ToolVectorActual feedback when finite values exist.
      2. Legacy robot X/Y/Z log columns when present in the first row.
      3. FK of the QActual joint columns as the fallback.
    """
    joints = {layer: _fill_forward(_joint_matrix(rows, layer)) for layer in LAYERS}
    xyz = {layer: _fk_xyz(values) for layer, values in joints.items()}
    robot_tool_actual_xyz = _tool_xyz_matrix(rows, "robot_tool_actual")
    robot_tool_target_xyz = _tool_xyz_matrix(rows, "robot_tool_target")
    robot_tcp_source = "FK from QActual joints"
    if np.isfinite(robot_tool_actual_xyz).any():
        xyz["robot"] = _fill_forward(robot_tool_actual_xyz)
        robot_tcp_source = "MG400 ToolVectorActual feedback"
    xyz["robot_tool_target"] = _fill_forward(robot_tool_target_xyz)
    if all(key in rows[0] for key in ("X", "Y", "Z")):
        xyz["robot"] = np.asarray(
            [[_float(row.get("X")), _float(row.get("Y")), _float(row.get("Z"))] for row in rows],
            dtype=float,
        )
        robot_tcp_source = "legacy robot X/Y/Z log columns"
    return joints, xyz, robot_tcp_source


def _compute_session_metrics(rows, joints, xyz, t):
    """All scalar metrics the markdown summary + JSON output expose.

    Includes per-column latency stats, joint + Cartesian errors at
    target-reached events, the cmd-to-robot tracking errors, the
    robot TCP path length / speed / per-axis range, and the joint-
    speed extreme when VJ feedback columns are available.
    """
    metrics: dict[str, dict] = {}
    for col in LATENCY_COLUMNS:
        stat = _summary_stats(_metric_values(rows, col))
        stat["unit"] = "ms"
        metrics[col] = stat

    arrival_mask = _event_mask(rows, {"latency_arrival"})
    arrival_joint_error_rad = np.asarray(
        [_float(row.get("max_joint_error_rad")) for row in rows], dtype=float
    )
    stat = _summary_stats(np.degrees(arrival_joint_error_rad[arrival_mask]))
    stat["unit"] = "deg"
    metrics["target_reached_joint_error"] = stat

    robot_sample_mask = _event_mask(rows, {"robot_feedback", "legacy_triple"})
    cmd_robot_mask = _finite_rows(joints["ros_cmd"], joints["robot"]) & robot_sample_mask
    if cmd_robot_mask.any():
        joint_err_deg = np.degrees(
            joints["ros_cmd"][cmd_robot_mask] - joints["robot"][cmd_robot_mask]
        )
        stat = _summary_stats(np.max(np.abs(joint_err_deg), axis=1))
        stat["unit"] = "deg"
        metrics["max_joint_error_ros_cmd_to_robot"] = stat

    for a, b, name in (
        ("unity_compensated", "robot", "cartesian_error_unity_to_robot"),
        ("ros_cmd", "robot", "cartesian_error_ros_cmd_to_robot"),
    ):
        mask = _finite_rows(xyz[a], xyz[b]) & robot_sample_mask
        stat = _summary_stats(_norm_rows(xyz[a][mask] - xyz[b][mask]) if mask.any() else np.array([]))
        stat["unit"] = "mm"
        metrics[name] = stat

    for a, b, name in (
        ("unity_compensated", "robot", "target_reached_tcp_error_unity_to_robot"),
        ("ros_cmd", "robot", "target_reached_tcp_error_ros_cmd_to_robot"),
        ("robot_tool_target", "robot", "target_reached_tcp_error_tool_target_to_actual"),
    ):
        mask = _finite_rows(xyz[a], xyz[b]) & arrival_mask
        stat = _summary_stats(_norm_rows(xyz[a][mask] - xyz[b][mask]) if mask.any() else np.array([]))
        stat["unit"] = "mm"
        metrics[name] = stat

    mask = _finite_rows(xyz["robot_tool_target"], xyz["robot"]) & robot_sample_mask
    stat = _summary_stats(
        _norm_rows(xyz["robot_tool_target"][mask] - xyz["robot"][mask])
        if mask.any() else np.array([])
    )
    stat["unit"] = "mm"
    metrics["tracking_tcp_error_tool_target_to_actual"] = stat

    _add_robot_tcp_motion_metrics(metrics, xyz, t, robot_sample_mask)
    _add_robot_joint_speed_metric(metrics, rows)
    return metrics


def _add_robot_tcp_motion_metrics(metrics, xyz, t, robot_sample_mask):
    """TCP path length, speed, and per-axis range — only added when the
    robot XYZ trace has at least one finite point after the
    robot_sample_mask is applied.
    """
    robot_xyz_mask = np.isfinite(xyz["robot"]).all(axis=1) & robot_sample_mask
    if not robot_xyz_mask.any():
        return
    robot_xyz = xyz["robot"][robot_xyz_mask]
    robot_t = t[robot_xyz_mask]
    if len(robot_xyz) > 1:
        segment_len = _norm_rows(np.diff(robot_xyz, axis=0))
        dt = np.diff(robot_t)
        valid_dt = dt > 1e-4
        speed = segment_len[valid_dt] / dt[valid_dt]
        metrics["robot_tcp_path_length"] = _single_stat(float(np.sum(segment_len)), "mm")
        stat = _summary_stats(speed)
        stat["unit"] = "mm/s"
        metrics["robot_tcp_speed"] = stat
    for axis_idx, label in ((0, "x"), (1, "y"), (2, "z")):
        rng = float(np.nanmax(robot_xyz[:, axis_idx]) - np.nanmin(robot_xyz[:, axis_idx]))
        metrics[f"robot_tcp_{label}_range"] = _single_stat(rng, "mm")


def _add_robot_joint_speed_metric(metrics, rows):
    """Max-axis joint speed across the session, only added when the
    feedback CSV carries VJ1..VJ4 columns.
    """
    if not all(f"VJ{i}" in rows[0] for i in range(1, 5)):
        return
    joint_speed = np.asarray(
        [max(abs(_float(row.get(f"VJ{i}"))) for i in range(1, 5)) for row in rows],
        dtype=float,
    )
    stat = _summary_stats(joint_speed)
    stat["unit"] = "deg/s"
    metrics["robot_max_joint_speed"] = stat


def _count_events_and_modes(rows):
    """Tally ``event_type`` and ``operation_mode`` occurrences for the
    markdown summary tables.
    """
    event_counts: dict[str, int] = {}
    mode_counts: dict[str, int] = {}
    for row in rows:
        event = _event_type(row)
        event_counts[event] = event_counts.get(event, 0) + 1
        mode = row.get("operation_mode") or "unknown"
        mode_counts[mode] = mode_counts.get(mode, 0) + 1
    return event_counts, mode_counts


def _render_all_figures(out_dir, t, rows, joints, xyz, metrics, robot_tcp_source):
    """Render every PNG the analysis emits and return the ``figures``
    path map alongside the per-plate KPI summaries.

    The plate-pair plots also return summary dicts (one per pair)
    that the CSV / JSON / markdown writers stitch into the
    `plate_clean_solid_summary` table.
    """
    figures = {
        "joint_positions": out_dir / "joint_positions_by_layer.png",
        "joint_error": out_dir / "joint_error_ros_to_robot.png",
        "latency": out_dir / "latency_breakdown.png",
        "cartesian_xyz": out_dir / "cartesian_xyz_by_layer.png",
        "cartesian_error": out_dir / "cartesian_error_mm.png",
        "path_3d": out_dir / "tcp_path_3d.png",
        "report_latency": out_dir / "report_latency_summary.png",
        "report_latency_split": out_dir / "report_latency_split.png",
        "report_accuracy": out_dir / "report_accuracy_summary.png",
        "report_joint_tracking": out_dir / "report_joint_tracking_active.png",
        "report_tcp_views": out_dir / "report_tcp_path_views.png",
        "report_metrics_card": out_dir / "report_metrics_card.png",
        "report_poster_summary": out_dir / "report_poster_summary.png",
        "plate_vr": out_dir / "Plot_VR_plate_clean_solid_kpi_peak.png",
        "plate_unity_vs_ros": out_dir / "PLOT_UNITY_VS_ROS_plate_clean_solid_kpi_peak.png",
        "plate_unity_vs_robot": out_dir / "PLOT_UNITY_VS_ROBOT_plate_clean_solid_kpi_peak.png",
        "plate_ros_vs_robot": out_dir / "PLOT_ROS_VS_ROBOT_MANUAL_plate_clean_solid_kpi_peak.png",
        "plate_contact_sheet": out_dir / "contact_sheet_plate_clean_solid.png",
    }
    _plot_joint_positions(t, joints, figures["joint_positions"])
    _plot_joint_error(t, joints, figures["joint_error"])
    _plot_latency(rows, figures["latency"])
    _plot_cartesian(t, xyz, figures["cartesian_xyz"], figures["cartesian_error"], figures["path_3d"])
    _plot_report_latency(metrics, figures["report_latency"])
    _plot_report_latency_split(metrics, figures["report_latency_split"])
    _plot_report_accuracy(metrics, figures["report_accuracy"])
    _plot_report_poster_summary(metrics, figures["report_poster_summary"], robot_tcp_source)
    _plot_report_joint_active(t, rows, joints, figures["report_joint_tracking"])
    _plot_report_tcp_views(t, rows, xyz, figures["report_tcp_views"], robot_tcp_source)
    _plot_report_metrics_card(metrics, figures["report_metrics_card"], robot_tcp_source)
    plate_summaries = [
        _plot_plate_pair(t, rows, xyz, "unity_raw", "robot", "Plot_VR", figures["plate_vr"]),
        _plot_plate_pair(t, rows, xyz, "unity_compensated", "ros_cmd", "PLOT_UNITY_VS_ROS", figures["plate_unity_vs_ros"]),
        _plot_plate_pair(t, rows, xyz, "unity_compensated", "robot", "PLOT_UNITY_VS_ROBOT", figures["plate_unity_vs_robot"]),
        _plot_plate_pair(t, rows, xyz, "ros_cmd", "robot", "PLOT_ROS_VS_ROBOT_MANUAL", figures["plate_ros_vs_robot"]),
    ]
    _plot_plate_contact_sheet(figures, figures["plate_contact_sheet"])
    return figures, plate_summaries


def _write_session_artifacts(
    *,
    out_dir: Path,
    path: Path,
    rows,
    unity_path,
    t,
    xyz,
    metrics,
    report,
    figures,
    robot_tcp_source,
    event_counts,
    mode_counts,
    plate_summaries,
) -> None:
    """Write every non-figure artefact: cartesian samples CSV, the
    summary metrics CSV / JSON, the plate KPI CSV, the all-metrics
    rollup CSV, and the markdown summary.

    Kwargs-only so the long argument list at the call site reads as
    a labelled bag rather than positional soup.
    """
    _write_cartesian_csv(out_dir / "cartesian_samples.csv", t, xyz)
    _save_summary_csv(out_dir / "summary_metrics.csv", metrics)
    _write_plate_summary_csv(out_dir / "plate_clean_solid_summary.csv", plate_summaries)
    _write_all_metrics_csv(
        out_dir / "summary_all_metrics.csv", report, metrics, plate_summaries
    )
    with (out_dir / "summary_metrics.json").open("w") as fh:
        json.dump(
            {
                "input": str(path),
                "unity_input": str(unity_path) if unity_path else None,
                "event_counts": event_counts,
                "mode_counts": mode_counts,
                "metrics": metrics,
                "report": report,
                "plate_summaries": plate_summaries,
            },
            fh,
            indent=2,
        )
    _write_markdown_summary(
        out_dir / "summary.md",
        path,
        rows,
        event_counts,
        mode_counts,
        metrics,
        figures,
        robot_tcp_source,
        report,
        plate_summaries,
    )


def _metric_line(metrics: dict, key: str, label: str) -> str:
    stat = metrics.get(key, {})
    if not stat or not stat.get("count"):
        return f"- {label}: no samples"
    unit = stat.get("unit", "")
    return (
        f"- {label}: mean {_fmt(stat.get('mean'))} {unit}, "
        f"p95 {_fmt(stat.get('p95'))} {unit}, max {_fmt(stat.get('max'))} {unit} "
        f"(n={stat.get('count')})"
    )


def _value_line(label: str, value, unit: str = "") -> str:
    suffix = f" {unit}" if unit else ""
    return f"- {label}: {_fmt(value)}{suffix}"


def _stat_line(report: dict, path_keys: tuple[str, ...], label: str, unit: str = "") -> str:
    data = report
    for key in path_keys:
        data = data.get(key, {}) if isinstance(data, dict) else {}
    if not isinstance(data, dict) or not data.get("count"):
        return f"- {label}: no samples"
    stat_unit = unit or data.get("unit", "")
    return (
        f"- {label}: n={data.get('count')}, mean {_fmt(data.get('mean'))} {stat_unit}, "
        f"p50 {_fmt(data.get('p50', data.get('median')))} {stat_unit}, "
        f"p95 {_fmt(data.get('p95'))} {stat_unit}, "
        f"p99 {_fmt(data.get('p99'))} {stat_unit}, max {_fmt(data.get('max'))} {stat_unit}"
    )


def _write_markdown_summary(
    path: Path,
    source: Path,
    rows,
    event_counts,
    mode_counts,
    metrics,
    figures,
    robot_tcp_source: str,
    report: dict,
    plate_summaries: list[dict],
) -> None:
    """Write the human-facing markdown report.

    Composes the file from per-section helpers in the same order the
    reader expects: input files -> layer flow -> log size -> event
    counts -> unity integrity -> command/latency -> accuracy ->
    status/anomalies -> plate KPIs -> poster metrics -> tables ->
    figures.
    """
    main = report.get("main", {})
    unity = report.get("unity", {})
    feedback = main.get("robot_feedback", {})
    commands = main.get("commands", {})
    latency = main.get("latency_arrival", {})
    anomalies = main.get("anomalies", {})
    unity_window = main.get("unity_window_in_main", {})

    lines: list[str] = ["# Teleop Session Analysis", ""]
    lines += _md_input_files_section(
        source, unity, rows, event_counts, mode_counts, robot_tcp_source
    )
    lines += _MD_LAYER_FLOW_SECTION
    lines += _md_log_size_section(main, unity, feedback, event_counts)
    lines += _md_event_counts_section(event_counts)
    lines += _md_unity_integrity_section(unity)
    lines += _md_command_latency_section(commands, latency)
    lines += _md_accuracy_section(main)
    lines += _md_status_anomalies_section(feedback, anomalies, unity_window)
    lines += _md_plate_kpi_section(plate_summaries)
    lines += _md_poster_metrics_section(metrics)
    lines += _MD_REPORT_TABLES_SECTION
    lines += _md_figures_section(figures)
    path.write_text("\n".join(lines))


def _md_input_files_section(
    source: Path, unity: dict, rows, event_counts, mode_counts, robot_tcp_source: str
) -> list[str]:
    """Top of the report: source file paths, row count, event /
    mode counts, and which feedback source seeded the robot TCP trace.
    """
    return [
        "## Input Files",
        "",
        f"- Input: `{source}`",
        f"- Unity input: `{unity.get('file', {}).get('path', '')}`",
        f"- Rows: {len(rows)}",
        f"- Events: {event_counts}",
        f"- Operation modes: {mode_counts}",
        f"- Robot TCP source: {robot_tcp_source}",
        "",
    ]


_MD_LAYER_FLOW_SECTION: list[str] = [
    "## Layer Flow",
    "",
    "- Layer 1 Unity sample: `/unity/teleop_sample` protocol rows from Unity controller/IK/filter output.",
    "- Layer 2 Unity target: ROS receives valid Unity target and applies timing/compensation context.",
    "- Layer 3 ROS command: adaptive gate converts target into robot command only when movement is needed.",
    "- Layer 4 Dobot/MG400 command result: command ID/result and target-reached matching rows.",
    "- Layer 5 Robot feedback: high-rate actual joint/tool/mode/error feedback from controller.",
    "- Layer 6 Analysis/match: offline comparison of Unity target, ROS command, and robot actual TCP/joints.",
    "",
]


def _md_log_size_section(main: dict, unity: dict, feedback: dict, event_counts: dict) -> list[str]:
    """Main / robot-feedback / Unity row counts + effective rates +
    per-channel inter-sample gap distributions.
    """
    return [
        "## Log Size / Rate",
        "",
        _value_line("Main rows", main.get("rows")),
        _value_line("Main columns", main.get("columns")),
        _value_line("Main duration", (main.get("duration_rate") or {}).get("duration_sec"), "s"),
        _value_line("Main effective rate", (main.get("duration_rate") or {}).get("effective_hz"), "Hz"),
        _stat_line(main, ("gap_ms",), "Main row gap", "ms"),
        _value_line("Robot feedback rows", event_counts.get("robot_feedback", 0)),
        _value_line(
            "Robot feedback effective rate",
            (feedback.get("duration_rate") or {}).get("effective_hz"),
            "Hz",
        ),
        _value_line("Unity rows", unity.get("rows")),
        _value_line("Unity effective rate", (unity.get("duration_rate") or {}).get("effective_hz"), "Hz"),
        _stat_line(unity, ("gap_ms",), "Unity sample gap", "ms"),
        "",
    ]


def _md_event_counts_section(event_counts: dict) -> list[str]:
    """Per-event-type tally — one bullet per ``event_type``."""
    lines = ["## Event Counts", ""]
    lines.extend(f"- {key}: {value}" for key, value in event_counts.items())
    lines.append("")
    return lines


def _md_unity_integrity_section(unity: dict) -> list[str]:
    """Unity seq range, gaps/duplicates, valid rows, and cross-file
    matching plus control-mode and filter-status tallies.
    """
    return [
        "## Unity Integrity",
        "",
        _value_line("Unity seq min", unity.get("seq_min")),
        _value_line("Unity seq max", unity.get("seq_max")),
        _value_line("Unity missing seq", unity.get("missing_seq")),
        _value_line("Unity duplicate seq", unity.get("duplicates")),
        _value_line("Unity valid true rows", unity.get("valid_true_rows")),
        _value_line("Unity seq matched across files", unity.get("seq_cross_file_matched")),
        _value_line("Unity seq missing across files", unity.get("seq_cross_file_missing")),
        f"- Unity control modes: {unity.get('control_mode_counts', {})}",
        f"- Unity filter statuses: {unity.get('filter_status_counts', {})}",
        "",
    ]


def _md_command_latency_section(commands: dict, latency: dict) -> list[str]:
    """ROS-command counts, send-reason tally, gap/decision/network
    delay stats, plus T4/T5 arrival latency (command latency, robot
    response, true end-to-end).
    """
    return [
        "## Command / Latency",
        "",
        _value_line("ROS commands", commands.get("count")),
        f"- Send reasons: {commands.get('send_reason_counts', {})}",
        _stat_line(commands, ("time_since_last_cmd_ms",), "Command gap / time since last command", "ms"),
        _stat_line(commands, ("network_delay_ms",), "Network / Unity-to-ROS excess delay", "ms"),
        _stat_line(commands, ("decision_delay_ms",), "ROS decision delay", "ms"),
        _value_line("Latency-arrival rows", latency.get("count")),
        _stat_line(latency, ("command_latency_ms",), "Command latency", "ms"),
        _stat_line(latency, ("robot_response_ms",), "Robot response", "ms"),
        _stat_line(latency, ("true_end_to_end_ms",), "True end-to-end", "ms"),
        "",
    ]


def _md_accuracy_section(main: dict) -> list[str]:
    """Joint + tool error stats at target-reached events and the
    cmd-to-actual tracking errors that feed the poster.
    """
    return [
        "## Accuracy / Error",
        "",
        _stat_line(main, ("accuracy", "final_error_rad"), "Final/match joint error", "rad"),
        _stat_line(main, ("accuracy", "max_joint_error_rad"), "Max joint error", "rad"),
        _stat_line(
            main,
            ("accuracy", "match_tool_error_ros_cmd_to_robot_actual_mm"),
            "Match TCP error ROS command to robot actual",
            "mm",
        ),
        _stat_line(
            main,
            ("accuracy", "robot_target_to_actual_error_mm"),
            "Robot internal target-to-actual TCP error",
            "mm",
        ),
        _stat_line(
            main,
            ("accuracy", "active_ros_cmd_to_actual_tool_error_mm"),
            "Active ROS command-to-actual TCP error",
            "mm",
        ),
        "",
    ]


def _md_status_anomalies_section(
    feedback: dict, anomalies: dict, unity_window: dict
) -> list[str]:
    """Robot-mode / error-status tally, stale-sample + pending-command
    anomalies, and the pre/post-Unity window summary that flags
    out-of-band activity.
    """
    return [
        "## Status / Anomalies",
        "",
        f"- Robot mode counts: {feedback.get('robot_mode_counts', {})}",
        f"- Error status counts: {feedback.get('error_status_counts', {})}",
        _value_line("Nonzero error_status rows", feedback.get("nonzero_error_status_rows")),
        _value_line(
            "Stale Unity sample age >100ms rows",
            anomalies.get("stale_unity_sample_age_gt_100ms_rows"),
        ),
        _value_line("Ambiguous settle rows", anomalies.get("ambiguous_settle_rows")),
        _value_line("Pending command rows", anomalies.get("pending_command_rows")),
        _value_line("Max pending command count", anomalies.get("max_pending_command_count")),
        _value_line(
            "Queue backlog >0.01rad rows", anomalies.get("queue_backlog_gt_0p01rad_rows")
        ),
        _value_line(
            "Robot internal target error >10mm rows",
            anomalies.get("robot_internal_target_error_gt_10mm_rows"),
        ),
        _value_line(
            "Rows before first Unity event", unity_window.get("main_rows_before_first_unity")
        ),
        _value_line(
            "Seconds before first Unity event", unity_window.get("sec_before_first_unity"), "s"
        ),
        _value_line(
            "Rows after last Unity event", unity_window.get("main_rows_after_last_unity")
        ),
        _value_line(
            "Seconds after last Unity event", unity_window.get("sec_after_last_unity"), "s"
        ),
        f"- Events after last Unity event: {unity_window.get('events_after_last_unity', {})}",
        "",
    ]


def _md_plate_kpi_section(plate_summaries: list[dict]) -> list[str]:
    """One bullet per plate pair: row count, KPI threshold violations,
    peak vector error and per-axis dXYZ peak.
    """
    lines = ["## Plate-View KPI Plots", ""]
    for item in plate_summaries:
        lines.append(
            f"- {item.get('file')}: rows={item.get('row_count')}, "
            f"points>KPI={item.get('points_over_kpi_5mm')}, "
            f"peak={_fmt(item.get('peak_vector_error_mm'))} mm, "
            f"peak dXYZ=({_fmt(item.get('peak_dx_mm'))}, {_fmt(item.get('peak_dy_mm'))}, "
            f"{_fmt(item.get('peak_dz_mm'))}) mm"
        )
    lines.append("")
    return lines


def _md_poster_metrics_section(metrics: dict) -> list[str]:
    """Headline metrics for the poster summary — latency split, target-
    reached errors, tracking errors, and the robot motion envelope.
    """
    return [
        "## Poster Metrics",
        "",
        _metric_line(metrics, "network_delay_ms", "Unity -> ROS excess delay after clock calibration"),
        "- Note: Unity timestamps are Unity runtime seconds, so this is delay above the calibrated clock floor, not absolute one-way network latency.",
        _metric_line(metrics, "decision_delay_ms", "ROS decision delay"),
        _metric_line(metrics, "robot_response_ms", "ROS command -> robot motion response"),
        _metric_line(metrics, "true_end_to_end_ms", "Unity -> robot target reached"),
        _metric_line(metrics, "target_reached_joint_error", "Target reached joint error"),
        _metric_line(metrics, "target_reached_tcp_error_ros_cmd_to_robot", "Target reached TCP error"),
        _metric_line(
            metrics,
            "target_reached_tcp_error_tool_target_to_actual",
            "Target reached MG400 ToolVectorTarget -> ToolVectorActual error",
        ),
        _metric_line(
            metrics, "max_joint_error_ros_cmd_to_robot", "Tracking joint error during feedback"
        ),
        _metric_line(
            metrics,
            "tracking_tcp_error_tool_target_to_actual",
            "Tracking MG400 ToolVectorTarget -> ToolVectorActual error",
        ),
        _metric_line(metrics, "cartesian_error_unity_to_robot", "Tracking Unity target -> robot TCP error"),
        _metric_line(metrics, "cartesian_error_ros_cmd_to_robot", "Tracking ROS command -> robot TCP error"),
        _metric_line(metrics, "robot_tcp_path_length", "Robot TCP path length"),
        _metric_line(metrics, "robot_tcp_speed", "Robot TCP speed"),
        _metric_line(metrics, "robot_max_joint_speed", "Robot max joint speed"),
        _metric_line(metrics, "robot_tcp_z_range", "Robot TCP Z travel range"),
        "",
    ]


_MD_REPORT_TABLES_SECTION: list[str] = [
    "## Report-Ready Tables",
    "",
    "- `summary_all_metrics.csv`: flattened table with all layer numbers for reports.",
    "- `summary_metrics.json`: structured JSON with full report, metrics, and plate summaries.",
    "- `summary_metrics.csv`: compact metric table.",
    "- `plate_clean_solid_summary.csv`: KPI/peak table matching the old plate-view workflow.",
    "- `cartesian_samples.csv`: time-aligned XYZ samples and Cartesian errors.",
    "",
]


def _md_figures_section(figures: dict) -> list[str]:
    """One bullet per generated PNG, keyed by the figure label."""
    lines = ["## Generated Figures", ""]
    for label, fig_path in figures.items():
        lines.append(f"- {label}: `{fig_path.name}`")
    lines.append("")
    return lines


def _select_files_with_dialog() -> list[Path]:
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        files = filedialog.askopenfilenames(
            title="Select teleop session CSV log(s)",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        root.destroy()
        return [Path(f) for f in files]
    except Exception:
        return []


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", help="teleop session CSV file(s); drag files here on macOS")
    parser.add_argument("--out-dir", type=Path, help="output directory for one input file")
    args = parser.parse_args(argv)

    files = [Path(f).expanduser() for f in args.files]
    if not files:
        files = _select_files_with_dialog()
    if not files:
        print("No file selected.")
        return 2

    if args.out_dir and len(files) > 1:
        print("--out-dir can only be used with one input file")
        return 2

    for file_path in files:
        out = analyze_file(file_path, args.out_dir)
        print(f"Analyzed: {file_path}")
        print(f"Output : {out}")
        print(f"Open   : {out / 'summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
