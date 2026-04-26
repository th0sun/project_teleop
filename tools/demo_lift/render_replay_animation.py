#!/usr/bin/env python3
"""Render a 3D GIF of target vs actual replay path from a timing probe JSON."""

from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib-codex"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter


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


def render_gif(probe: dict, out_path: Path, fps: int = 15, step: int = 2) -> None:
    samples = probe.get("sample_rows", [])
    if not samples:
        raise ValueError("Probe JSON has no sample_rows. Re-run measure_replay_timing.py after the latest update.")

    target_path = probe["target_tool_path"]
    target_xyz = [tool[:3] for tool in target_path]
    actual_xyz = [row["tool_actual"][:3] for row in samples]
    target_interp_xyz = [row["tool_target_interp"][:3] for row in samples]
    (xmin, xmax), (ymin, ymax), (zmin, zmax) = _axis_bounds(target_xyz, actual_xyz)

    fig = plt.figure(figsize=(10, 8), dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f7f7f7")
    ax.view_init(elev=28, azim=-60)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)
    ax.set_title(f"{probe.get('trajectory_name', 'trajectory')} replay", pad=18)

    tx = [p[0] for p in target_xyz]
    ty = [p[1] for p in target_xyz]
    tz = [p[2] for p in target_xyz]
    ax.plot(tx, ty, tz, color="#2b6cb0", linestyle="--", linewidth=1.8, alpha=0.75, label="target path")

    actual_line, = ax.plot([], [], [], color="#d94841", linewidth=2.2, label="actual path")
    target_live, = ax.plot([], [], [], color="#2b6cb0", linewidth=2.0, alpha=0.35)
    target_point = ax.scatter([], [], [], color="#2b6cb0", s=45, depthshade=False, label="target")
    actual_point = ax.scatter([], [], [], color="#d94841", s=45, depthshade=False, label="actual")
    text = fig.text(0.02, 0.02, "", fontsize=10, family="monospace")
    ax.legend(loc="upper right")

    frame_indices = list(range(0, len(samples), max(1, step)))
    if frame_indices[-1] != len(samples) - 1:
        frame_indices.append(len(samples) - 1)

    writer = PillowWriter(fps=fps)
    writer.setup(fig, str(out_path), dpi=120)
    try:
        for idx in frame_indices:
            row = samples[idx]
            actual_so_far = actual_xyz[: idx + 1]
            target_so_far = target_interp_xyz[: idx + 1]

            actual_line.set_data([p[0] for p in actual_so_far], [p[1] for p in actual_so_far])
            actual_line.set_3d_properties([p[2] for p in actual_so_far])

            target_live.set_data([p[0] for p in target_so_far], [p[1] for p in target_so_far])
            target_live.set_3d_properties([p[2] for p in target_so_far])

            target_point._offsets3d = ([target_interp_xyz[idx][0]], [target_interp_xyz[idx][1]], [target_interp_xyz[idx][2]])
            actual_point._offsets3d = ([actual_xyz[idx][0]], [actual_xyz[idx][1]], [actual_xyz[idx][2]])

            text.set_text(
                "\n".join([
                    f"t = {row['elapsed_s']:.3f}s",
                    f"xyz error = {row['xyz_error_mm']:.2f} mm",
                    f"path dev = {row['path_dev_mm']:.2f} mm",
                    f"yaw error = {row['yaw_error_deg']:.2f} deg",
                    f"mode = {row['robot_mode']}",
                ])
            )
            writer.grab_frame()
    finally:
        writer.finish()

    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("probe_json", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--step", type=int, default=2)
    args = parser.parse_args()

    probe = _load_probe(args.probe_json)
    out = args.out or args.probe_json.with_suffix(".gif")
    out.parent.mkdir(parents=True, exist_ok=True)
    render_gif(probe, out, fps=args.fps, step=args.step)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
