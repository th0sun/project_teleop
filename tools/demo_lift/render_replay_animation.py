#!/usr/bin/env python3
"""Render a beautiful 3D animation (MP4/GIF) of target vs actual replay path."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib-codex"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, PillowWriter
import numpy as np

def _load_probe(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))

def _axis_bounds(target_xyz, actual_xyz):
    points = target_xyz + actual_xyz
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    cz = (min(zs) + max(zs)) / 2.0
    radius = max(
        (max(xs) - min(xs)) / 2.0,
        (max(ys) - min(ys)) / 2.0,
        (max(zs) - min(zs)) / 2.0,
        1.0,
    ) * 1.15
    return (cx - radius, cx + radius), (cy - radius, cy + radius), (cz - radius, cz + radius)

def render_animation(probe: dict, out_path: Path, fps: int = 24, step: int = 1) -> None:
    samples = probe.get("sample_rows", [])
    if not samples:
        raise ValueError("Probe JSON has no sample_rows.")

    target_path = probe["target_tool_path"]
    target_xyz = [tool[:3] for tool in target_path]
    actual_xyz = [row["tool_actual"][:3] for row in samples]
    target_interp_xyz = [row["tool_target_interp"][:3] for row in samples]

    (xmin, xmax), (ymin, ymax), (zmin, zmax) = _axis_bounds(target_xyz, actual_xyz)

    # Style: Dark mode, neon colors
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(12, 9), dpi=150)
    ax = fig.add_subplot(111, projection="3d")

    # Custom background colors
    fig.patch.set_facecolor('#111111')
    ax.set_facecolor('#111111')
    ax.grid(color='#333333', linestyle=':', linewidth=0.5)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('#333333')
    ax.yaxis.pane.set_edgecolor('#333333')
    ax.zaxis.pane.set_edgecolor('#333333')

    ax.view_init(elev=30, azim=-60)
    ax.set_xlabel("X (mm)", color='#aaaaaa')
    ax.set_ylabel("Y (mm)", color='#aaaaaa')
    ax.set_zlabel("Z (mm)", color='#aaaaaa')
    ax.tick_params(colors='#aaaaaa')

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)

    traj_name = probe.get('trajectory_name', 'trajectory')
    ax.set_title(f"Telemetry Replay: {traj_name}", pad=20, color='#ffffff', fontsize=14, fontweight='bold')

    tx = [p[0] for p in target_xyz]
    ty = [p[1] for p in target_xyz]
    tz = [p[2] for p in target_xyz]

    # Static Target Path (Taught)
    ax.plot(tx, ty, tz, color="#00ffcc", linestyle="--", linewidth=1.0, alpha=0.4, label="Taught Path")

    # Animated elements
    actual_line, = ax.plot([], [], [], color="#ff3366", linewidth=2.5, label="Robot Actual", zorder=4)
    target_live, = ax.plot([], [], [], color="#00ffcc", linewidth=1.5, alpha=0.8, label="Robot Target", zorder=3)

    # Current position markers
    target_point = ax.scatter([], [], [], color="#00ffcc", s=80, edgecolors='#ffffff', linewidths=1.5, zorder=5)
    actual_point = ax.scatter([], [], [], color="#ff3366", s=100, edgecolors='#ffffff', linewidths=1.5, zorder=6)

    # Info panel
    text = fig.text(0.03, 0.03, "", fontsize=12, family="monospace", color="#00ffcc",
                    bbox=dict(facecolor='#000000', alpha=0.7, edgecolor='#333333', boxstyle='round,pad=0.5'))

    ax.legend(loc="upper right", facecolor='#000000', edgecolor='#333333', labelcolor='#ffffff')

    frame_indices = list(range(0, len(samples), max(1, step)))
    if frame_indices[-1] != len(samples) - 1:
        frame_indices.append(len(samples) - 1)

    if out_path.suffix.lower() == ".mp4":
        writer = FFMpegWriter(fps=fps, bitrate=2500)
    else:
        writer = PillowWriter(fps=fps)

    writer.setup(fig, str(out_path), dpi=150)

    total_frames = len(frame_indices)

    try:
        for i, idx in enumerate(frame_indices):
            row = samples[idx]
            # Use fading tail for actual path to avoid messiness (keep last 50 points)
            tail_length = 50
            start_tail = max(0, idx + 1 - tail_length)
            actual_so_far = actual_xyz[start_tail: idx + 1]
            target_so_far = target_interp_xyz[start_tail: idx + 1]

            actual_line.set_data([p[0] for p in actual_so_far], [p[1] for p in actual_so_far])
            actual_line.set_3d_properties([p[2] for p in actual_so_far])

            target_live.set_data([p[0] for p in target_so_far], [p[1] for p in target_so_far])
            target_live.set_3d_properties([p[2] for p in target_so_far])

            target_point._offsets3d = ([target_interp_xyz[idx][0]], [target_interp_xyz[idx][1]], [target_interp_xyz[idx][2]])
            actual_point._offsets3d = ([actual_xyz[idx][0]], [actual_xyz[idx][1]], [actual_xyz[idx][2]])

            # Slowly rotate camera
            ax.view_init(elev=25 + 5*np.sin(i/total_frames * np.pi), azim=-60 + (i/total_frames)*60)

            text.set_text(
                f"T: {row['elapsed_s']:05.3f}s\n"
                f"Dev: {row['path_dev_mm']:05.2f} mm\n"
                f"Mode: {row['robot_mode']}"
            )
            writer.grab_frame()
            if i % 50 == 0:
                print(f"Rendered frame {i}/{total_frames}")
    finally:
        writer.finish()

    plt.close(fig)

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("probe_json", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--step", type=int, default=2)
    args = parser.parse_args()

    probe = _load_probe(args.probe_json)
    out = args.out or args.probe_json.with_suffix(".mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    render_animation(probe, out, fps=args.fps, step=args.step)
    print(f"Wrote animation to {out}")

if __name__ == "__main__":
    main()
