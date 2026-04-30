#!/usr/bin/env python3
"""Analyze one-file teleop session logs and generate report-ready figures.

Usage:
    python3 tools/analyze_teleop_session.py logs/teleop_sessions/teleop_session_*.csv

On macOS you can drag a CSV file into the terminal after the command.  If no
file is provided, the script tries to open a file picker.
"""

from __future__ import annotations

import argparse
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
        ("network_delay_ms", "Unity -> ROS"),
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
        ("network_delay_ms", "Unity -> ROS"),
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
    if len(labels) == 1:
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
        ("Unity -> ROS", _stat_value(metrics, "network_delay_ms", "median"), "ms"),
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
        ("Unity -> ROS", "median", "network_delay_ms", "median", "ms"),
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


def analyze_file(path: Path, out_root: Path | None = None) -> Path:
    rows = _read_csv(path)
    if not rows:
        raise ValueError(f"{path} has no rows")

    out_dir = out_root or (path.parent / f"{path.stem}_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    t = _time_axis(rows)
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

    metrics: dict[str, dict] = {}
    for col in LATENCY_COLUMNS:
        stat = _summary_stats(_metric_values(rows, col))
        stat["unit"] = "ms"
        metrics[col] = stat

    arrival_mask = _event_mask(rows, {"latency_arrival"})
    arrival_joint_error_rad = np.asarray(
        [_float(row.get("max_joint_error_rad")) for row in rows],
        dtype=float,
    )
    stat = _summary_stats(np.degrees(arrival_joint_error_rad[arrival_mask]))
    stat["unit"] = "deg"
    metrics["target_reached_joint_error"] = stat

    cmd_robot_mask = _finite_rows(joints["ros_cmd"], joints["robot"])
    robot_sample_mask = _event_mask(rows, {"robot_feedback", "legacy_triple"})
    cmd_robot_mask &= robot_sample_mask
    if cmd_robot_mask.any():
        joint_err_deg = np.degrees(joints["ros_cmd"][cmd_robot_mask] - joints["robot"][cmd_robot_mask])
        stat = _summary_stats(np.max(np.abs(joint_err_deg), axis=1))
        stat["unit"] = "deg"
        metrics["max_joint_error_ros_cmd_to_robot"] = stat

    for a, b, name in (
        ("unity_compensated", "robot", "cartesian_error_unity_to_robot"),
        ("ros_cmd", "robot", "cartesian_error_ros_cmd_to_robot"),
    ):
        mask = _finite_rows(xyz[a], xyz[b])
        mask &= robot_sample_mask
        stat = _summary_stats(_norm_rows(xyz[a][mask] - xyz[b][mask]) if mask.any() else np.array([]))
        stat["unit"] = "mm"
        metrics[name] = stat

    for a, b, name in (
        ("unity_compensated", "robot", "target_reached_tcp_error_unity_to_robot"),
        ("ros_cmd", "robot", "target_reached_tcp_error_ros_cmd_to_robot"),
        ("robot_tool_target", "robot", "target_reached_tcp_error_tool_target_to_actual"),
    ):
        mask = _finite_rows(xyz[a], xyz[b])
        mask &= arrival_mask
        stat = _summary_stats(_norm_rows(xyz[a][mask] - xyz[b][mask]) if mask.any() else np.array([]))
        stat["unit"] = "mm"
        metrics[name] = stat

    mask = _finite_rows(xyz["robot_tool_target"], xyz["robot"])
    mask &= robot_sample_mask
    stat = _summary_stats(_norm_rows(xyz["robot_tool_target"][mask] - xyz["robot"][mask]) if mask.any() else np.array([]))
    stat["unit"] = "mm"
    metrics["tracking_tcp_error_tool_target_to_actual"] = stat

    robot_xyz_mask = np.isfinite(xyz["robot"]).all(axis=1)
    robot_xyz_mask &= robot_sample_mask
    if robot_xyz_mask.any():
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
        x_range = float(np.nanmax(robot_xyz[:, 0]) - np.nanmin(robot_xyz[:, 0]))
        y_range = float(np.nanmax(robot_xyz[:, 1]) - np.nanmin(robot_xyz[:, 1]))
        z_range = float(np.nanmax(robot_xyz[:, 2]) - np.nanmin(robot_xyz[:, 2]))
        metrics["robot_tcp_x_range"] = _single_stat(x_range, "mm")
        metrics["robot_tcp_y_range"] = _single_stat(y_range, "mm")
        metrics["robot_tcp_z_range"] = _single_stat(z_range, "mm")

    if all(f"VJ{i}" in rows[0] for i in range(1, 5)):
        joint_speed = np.asarray(
            [max(abs(_float(row.get(f"VJ{i}"))) for i in range(1, 5)) for row in rows],
            dtype=float,
        )
        stat = _summary_stats(joint_speed)
        stat["unit"] = "deg/s"
        metrics["robot_max_joint_speed"] = stat

    event_counts: dict[str, int] = {}
    mode_counts: dict[str, int] = {}
    for row in rows:
        event = _event_type(row)
        event_counts[event] = event_counts.get(event, 0) + 1
        mode = row.get("operation_mode") or "unknown"
        mode_counts[mode] = mode_counts.get(mode, 0) + 1

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

    _write_cartesian_csv(out_dir / "cartesian_samples.csv", t, xyz)
    _save_summary_csv(out_dir / "summary_metrics.csv", metrics)
    with (out_dir / "summary_metrics.json").open("w") as fh:
        json.dump({"input": str(path), "event_counts": event_counts, "mode_counts": mode_counts, "metrics": metrics}, fh, indent=2)

    _write_markdown_summary(out_dir / "summary.md", path, rows, event_counts, mode_counts, metrics, figures, robot_tcp_source)
    return out_dir


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


def _write_markdown_summary(path: Path, source: Path, rows, event_counts, mode_counts, metrics, figures, robot_tcp_source: str) -> None:
    lines = [
        "# Teleop Session Analysis",
        "",
        f"- Input: `{source}`",
        f"- Rows: {len(rows)}",
        f"- Events: {event_counts}",
        f"- Operation modes: {mode_counts}",
        f"- Robot TCP source: {robot_tcp_source}",
        "",
        "## Poster Metrics",
        "",
        _metric_line(metrics, "network_delay_ms", "Unity -> ROS network delay"),
        _metric_line(metrics, "decision_delay_ms", "ROS decision delay"),
        _metric_line(metrics, "robot_response_ms", "ROS command -> robot motion response"),
        _metric_line(metrics, "true_end_to_end_ms", "Unity -> robot target reached"),
        _metric_line(metrics, "target_reached_joint_error", "Target reached joint error"),
        _metric_line(metrics, "target_reached_tcp_error_ros_cmd_to_robot", "Target reached TCP error"),
        _metric_line(metrics, "target_reached_tcp_error_tool_target_to_actual", "Target reached MG400 ToolVectorTarget -> ToolVectorActual error"),
        _metric_line(metrics, "max_joint_error_ros_cmd_to_robot", "Tracking joint error during feedback"),
        _metric_line(metrics, "tracking_tcp_error_tool_target_to_actual", "Tracking MG400 ToolVectorTarget -> ToolVectorActual error"),
        _metric_line(metrics, "cartesian_error_unity_to_robot", "Tracking Unity target -> robot TCP error"),
        _metric_line(metrics, "cartesian_error_ros_cmd_to_robot", "Tracking ROS command -> robot TCP error"),
        _metric_line(metrics, "robot_tcp_path_length", "Robot TCP path length"),
        _metric_line(metrics, "robot_tcp_speed", "Robot TCP speed"),
        _metric_line(metrics, "robot_max_joint_speed", "Robot max joint speed"),
        _metric_line(metrics, "robot_tcp_z_range", "Robot TCP Z travel range"),
        "",
        "## Generated Figures",
        "",
    ]
    for label, fig_path in figures.items():
        lines.append(f"- {label}: `{fig_path.name}`")
    lines.append("")
    path.write_text("\n".join(lines))


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
