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
    if all(key in rows[0] for key in ("X", "Y", "Z")):
        xyz["robot"] = np.asarray(
            [[_float(row.get("X")), _float(row.get("Y")), _float(row.get("Z"))] for row in rows],
            dtype=float,
        )

    metrics: dict[str, dict] = {}
    for col in LATENCY_COLUMNS:
        stat = _summary_stats(_metric_values(rows, col))
        stat["unit"] = "ms"
        metrics[col] = stat

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

    robot_xyz_mask = np.isfinite(xyz["robot"]).all(axis=1)
    if robot_xyz_mask.any():
        robot_xyz = xyz["robot"][robot_xyz_mask]
        robot_t = t[robot_xyz_mask]
        if len(robot_xyz) > 1:
            segment_len = _norm_rows(np.diff(robot_xyz, axis=0))
            dt = np.diff(robot_t)
            speed = segment_len[dt > 0] / dt[dt > 0]
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
    for row in rows:
        event = _event_type(row)
        event_counts[event] = event_counts.get(event, 0) + 1

    figures = {
        "joint_positions": out_dir / "joint_positions_by_layer.png",
        "joint_error": out_dir / "joint_error_ros_to_robot.png",
        "latency": out_dir / "latency_breakdown.png",
        "cartesian_xyz": out_dir / "cartesian_xyz_by_layer.png",
        "cartesian_error": out_dir / "cartesian_error_mm.png",
        "path_3d": out_dir / "tcp_path_3d.png",
    }
    _plot_joint_positions(t, joints, figures["joint_positions"])
    _plot_joint_error(t, joints, figures["joint_error"])
    _plot_latency(rows, figures["latency"])
    _plot_cartesian(t, xyz, figures["cartesian_xyz"], figures["cartesian_error"], figures["path_3d"])

    _write_cartesian_csv(out_dir / "cartesian_samples.csv", t, xyz)
    _save_summary_csv(out_dir / "summary_metrics.csv", metrics)
    with (out_dir / "summary_metrics.json").open("w") as fh:
        json.dump({"input": str(path), "event_counts": event_counts, "metrics": metrics}, fh, indent=2)

    _write_markdown_summary(out_dir / "summary.md", path, rows, event_counts, metrics, figures)
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


def _write_markdown_summary(path: Path, source: Path, rows, event_counts, metrics, figures) -> None:
    lines = [
        "# Teleop Session Analysis",
        "",
        f"- Input: `{source}`",
        f"- Rows: {len(rows)}",
        f"- Events: {event_counts}",
        "",
        "## Poster Metrics",
        "",
        _metric_line(metrics, "network_delay_ms", "Unity -> ROS network delay"),
        _metric_line(metrics, "decision_delay_ms", "ROS decision delay"),
        _metric_line(metrics, "robot_response_ms", "ROS command -> robot motion response"),
        _metric_line(metrics, "true_end_to_end_ms", "Unity -> robot target reached"),
        _metric_line(metrics, "max_joint_error_ros_cmd_to_robot", "ROS command -> robot joint error"),
        _metric_line(metrics, "cartesian_error_unity_to_robot", "Unity target -> robot TCP error"),
        _metric_line(metrics, "cartesian_error_ros_cmd_to_robot", "ROS command -> robot TCP error"),
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
