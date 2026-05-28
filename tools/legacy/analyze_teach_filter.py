#!/usr/bin/env python3
"""Analyze the teach-and-repeat filtering pipeline on saved Unity trajectories.

This tool intentionally uses the production ``TrajectoryRecorder`` path:

    Unity/native JSON -> TrajectoryRecorder.load() -> compile_loaded_plan()

It then renders raw points, kept/simplified waypoints, and the actual queued
MG400 commands that the compiled plan would send.  The output is meant for
debugging, report figures, and generating command files for real-robot tests.

Usage from ``project_teleop``:

    PYTHONPATH=src/robot_teaching_core:src/dobot_mg400/mg400_controller:src/dobot_mg400/mg400_protocol \\
      python3 tools/demo_lift/analyze_teach_filter.py

    PYTHONPATH=... python3 tools/demo_lift/analyze_teach_filter.py \\
      --trajectory-json /path/to/my_trajectory.json \\
      --out-dir /tmp/filter_report
"""

from __future__ import annotations

import argparse
import base64
import csv
import html
import json
import math
import os
import re
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "project_teleop_matplotlib"),
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
import numpy as np  # noqa: E402

_REPO = Path(__file__).resolve().parents[2]
_WORKSPACE = _REPO.parent
for _pkg in (
    "src/robot_teaching_core",
    "src/dobot_mg400/mg400_controller",
    "src/dobot_mg400/mg400_protocol",
):
    _p = str(_REPO / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mg400_controller.common.config import motion_config  # noqa: E402
from mg400_controller.common.trajectory import (  # noqa: E402
    TrajectoryRecorder,
    compiled_playback_plan_to_dict,
)
from mg400_controller.common.utils.kinematics import KinematicsCalculator  # noqa: E402
from teaching_core.trajectory.segment_classifier import sample_command_arc_xyzr  # noqa: E402


DEFAULT_TRAJECTORIES = (
    _WORKSPACE / "_supporting_materials/data/trajectories/json_trajectories/money.json",
    _WORKSPACE / "_supporting_materials/data/trajectories/json_trajectories/Pick_place_1.json",
    _WORKSPACE / "_supporting_materials/data/trajectories/json_trajectories/Pick_place_2.json",
)

DEFAULT_JSON_DIR = _WORKSPACE / "_supporting_materials/data/trajectories/json_trajectories"

NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)")


class _CliLogger:
    def __init__(self) -> None:
        self.messages: List[str] = []

    def _add(self, level: str, msg: str) -> None:
        text = f"{level}: {msg}"
        self.messages.append(text)
        print(text)

    def info(self, msg):
        self._add("info", str(msg))

    def warn(self, msg):
        self._add("warn", str(msg))

    warning = warn

    def error(self, msg):
        self._add("error", str(msg))


@dataclass(frozen=True)
class CommandSegment:
    command_index: int
    command_type: str
    start_waypoint_index: int
    end_waypoint_index: int
    path_xyz: np.ndarray
    command: str
    raw_start_index: int = -1
    raw_end_index: int = -1
    raw_fit_max_mm: float = 0.0
    raw_fit_mean_mm: float = 0.0
    kept_fit_max_mm: float = 0.0
    kept_fit_mean_mm: float = 0.0


@dataclass(frozen=True)
class SceneFloorBox:
    name: str
    policy: str
    role: str
    min_xyz: np.ndarray
    max_xyz: np.ndarray
    raw_min_xyz: np.ndarray
    raw_max_xyz: np.ndarray
    workpiece_height_above_surface_mm: float


@dataclass(frozen=True)
class SceneRepairEvent:
    raw_index: int
    time_s: float
    source: str
    old_xyzr: Tuple[float, float, float, float]
    new_xyzr: Tuple[float, float, float, float]
    old_joints_deg: Tuple[float, float, float, float]
    new_joints_deg: Tuple[float, float, float, float]
    raised_by_mm: float
    applied: bool


TYPE_LABEL = {
    "Arc": "A",
    "MovL": "L",
    "JointMovJ": "J",
}

TYPE_LINESTYLE = {
    "Arc": "-.",
    "MovL": "-",
    "JointMovJ": "--",
}


def _frame_q_deg(frame: dict) -> Tuple[float, float, float, float]:
    return (
        float(frame["j1"]),
        float(frame["j2"]),
        float(frame["j3"]),
        float(frame["j4"]),
    )


def _fk_xyzr(kin: KinematicsCalculator, joints_deg: Iterable[float]) -> np.ndarray:
    return np.asarray(kin.forward_kinematics(list(joints_deg))[:4], dtype=float)


def _frames_to_xyzr(frames: Sequence[dict]) -> np.ndarray:
    kin = KinematicsCalculator()
    return np.asarray([_fk_xyzr(kin, _frame_q_deg(frame)) for frame in frames], dtype=float)


def _load_unity_json(path: Path) -> Tuple[List[dict], List[dict]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, list):
        return list(data), []
    frames = data.get("frames", [])
    events = data.get("events", [])
    return list(frames), list(events if isinstance(events, list) else [])


def _load_scene_floor_boxes(path: Path) -> List[SceneFloorBox]:
    model = json.loads(path.read_text(encoding="utf-8"))
    boxes: List[SceneFloorBox] = []
    for primitive in model.get("primitives", []):
        safety_box = primitive.get("safety_box", {})
        raw_box = primitive.get("raw_box", {})
        rules = primitive.get("contact_rules", {})
        if not safety_box or not raw_box:
            continue
        if not isinstance(rules, dict):
            rules = {}
        boxes.append(
            SceneFloorBox(
                name=str(primitive.get("object_name", "unknown")),
                policy=str(primitive.get("collision_policy", "avoid")),
                role=str(primitive.get("role", "")),
                min_xyz=np.asarray(safety_box["min_xyz"], dtype=float),
                max_xyz=np.asarray(safety_box["max_xyz"], dtype=float),
                raw_min_xyz=np.asarray(raw_box["min_xyz"], dtype=float),
                raw_max_xyz=np.asarray(raw_box["max_xyz"], dtype=float),
                workpiece_height_above_surface_mm=float(
                    rules.get("workpiece_height_above_support_surface_mm", 10.0)
                ),
            )
        )
    return boxes


def _xy_inside_scene_box(x: float, y: float, box: SceneFloorBox) -> bool:
    return bool(box.min_xyz[0] <= x <= box.max_xyz[0] and box.min_xyz[1] <= y <= box.max_xyz[1])


def _scene_floor_for_xy(
    x: float,
    y: float,
    boxes: Sequence[SceneFloorBox],
    *,
    contact_clearance_mm: float,
    avoid_clearance_mm: float,
) -> Tuple[float, str]:
    floor = -math.inf
    source = ""
    for box in boxes:
        if not _xy_inside_scene_box(x, y, box):
            continue
        if box.policy == "avoid":
            candidate = float(box.max_xyz[2]) + avoid_clearance_mm
            candidate_source = f"avoid:{box.name}:safe_top+{avoid_clearance_mm:g}"
        elif box.policy == "contact_allowed":
            expected_top = float(box.raw_min_xyz[2]) + float(box.workpiece_height_above_surface_mm)
            candidate = expected_top + contact_clearance_mm
            candidate_source = f"contact:{box.name}:expected_top+{contact_clearance_mm:g}"
        else:
            continue
        if candidate > floor:
            floor = candidate
            source = candidate_source
    return floor, source


def _scene_contact_floor_for_xy(
    x: float,
    y: float,
    boxes: Sequence[SceneFloorBox],
    *,
    contact_clearance_mm: float,
) -> Tuple[float, str]:
    floor = -math.inf
    source = ""
    for box in boxes:
        if box.policy != "contact_allowed" or not _xy_inside_scene_box(x, y, box):
            continue
        expected_top = float(box.raw_min_xyz[2]) + float(box.workpiece_height_above_surface_mm)
        candidate = expected_top + contact_clearance_mm
        if candidate > floor:
            floor = candidate
            source = f"contact:{box.name}:expected_top+{contact_clearance_mm:g}"
    return floor, source


def _scene_has_avoid_at_xy(x: float, y: float, boxes: Sequence[SceneFloorBox]) -> bool:
    return any(box.policy == "avoid" and _xy_inside_scene_box(x, y, box) for box in boxes)


def _avoid_escape_xy_candidates(
    x: float,
    y: float,
    boxes: Sequence[SceneFloorBox],
    *,
    outside_margin_mm: float,
) -> List[Tuple[float, float, str]]:
    candidates: List[Tuple[float, float, str]] = []
    for box in boxes:
        if box.policy != "avoid" or not _xy_inside_scene_box(x, y, box):
            continue
        options = [
            (float(box.min_xyz[0]) - outside_margin_mm, y, f"avoid_escape:{box.name}:x_min"),
            (float(box.max_xyz[0]) + outside_margin_mm, y, f"avoid_escape:{box.name}:x_max"),
            (x, float(box.min_xyz[1]) - outside_margin_mm, f"avoid_escape:{box.name}:y_min"),
            (x, float(box.max_xyz[1]) + outside_margin_mm, f"avoid_escape:{box.name}:y_max"),
        ]
        options.sort(key=lambda item: math.hypot(item[0] - x, item[1] - y))
        candidates.extend(options)
    return candidates


def _scene_repair_frames(
    frames: Sequence[dict],
    scene_model: Optional[Path],
    *,
    apply_repair: bool,
    contact_clearance_mm: float,
    avoid_clearance_mm: float,
    max_raise_mm: float,
    raise_step_mm: float,
) -> Tuple[List[dict], List[SceneRepairEvent]]:
    """Diagnose or raise unsafe taught TCP frames before simplification/classification.

    The earlier post-compile clipper could raise only MovL/Arc endpoints.  That
    is too late: the classifier may already have chosen a primitive whose start
    and fitted curve still pass below a measured fixture surface.  This repair
    happens at the raw-frame layer, then IK converts each corrected TCP pose
    back into a normal MG400 joint waypoint.  The CLI is diagnostic-only by
    default; mutation requires the explicit ``--apply-scene-repair`` flag so
    analysis cannot silently change the path the user taught.
    """
    if scene_model is None:
        return list(frames), []

    boxes = _load_scene_floor_boxes(scene_model)
    kin = KinematicsCalculator()
    repaired: List[dict] = []
    events: List[SceneRepairEvent] = []

    for idx, frame in enumerate(frames):
        old_joints = np.asarray(_frame_q_deg(frame), dtype=float)
        old_xyzr = _fk_xyzr(kin, old_joints)[:4]
        floor, source = _scene_floor_for_xy(
            float(old_xyzr[0]),
            float(old_xyzr[1]),
            boxes,
            contact_clearance_mm=contact_clearance_mm,
            avoid_clearance_mm=avoid_clearance_mm,
        )
        if math.isinf(floor) or float(old_xyzr[2]) >= floor:
            repaired.append(dict(frame))
            continue

        if not apply_repair:
            repaired.append(dict(frame))
            events.append(
                SceneRepairEvent(
                    raw_index=idx,
                    time_s=float(frame["timeStamp"]),
                    source=source,
                    old_xyzr=tuple(float(v) for v in old_xyzr[:4]),
                    new_xyzr=(float(old_xyzr[0]), float(old_xyzr[1]), float(floor), float(old_xyzr[3])),
                    old_joints_deg=tuple(float(v) for v in old_joints[:4]),
                    new_joints_deg=tuple(float(v) for v in old_joints[:4]),
                    raised_by_mm=float(floor - old_xyzr[2]),
                    applied=False,
                )
            )
            continue

        pose_candidates: List[Tuple[int, float, float, float, float, str]] = []
        old_x = float(old_xyzr[0])
        old_y = float(old_xyzr[1])
        old_z = float(old_xyzr[2])
        old_r = float(old_xyzr[3])
        inside_avoid = _scene_has_avoid_at_xy(old_x, old_y, boxes)

        if inside_avoid:
            # Avoid objects are hard obstacles.  Prefer a small horizontal escape
            # from the inflated XY box over the previous "fly over the top"
            # behavior, which could raise freehand paths by hundreds of mm.
            for esc_x, esc_y, esc_source in _avoid_escape_xy_candidates(
                old_x,
                old_y,
                boxes,
                outside_margin_mm=max(2.0, avoid_clearance_mm * 0.25),
            ):
                contact_floor, contact_source = _scene_contact_floor_for_xy(
                    esc_x,
                    esc_y,
                    boxes,
                    contact_clearance_mm=contact_clearance_mm,
                )
                esc_z = max(old_z, contact_floor) if not math.isinf(contact_floor) else old_z
                source_parts = [source, esc_source]
                if contact_source:
                    source_parts.append(contact_source)
                pose_candidates.append((0, esc_x, esc_y, esc_z, old_r, ";".join(source_parts)))

            # Last resort only: go vertically over the obstacle if there is no
            # reachable side escape.  This keeps the old behavior available but
            # makes it impossible for it to be chosen before the local detour.
            pose_candidates.append((1, old_x, old_y, floor, old_r, f"{source};vertical_last_resort"))
        else:
            pose_candidates.append((0, old_x, old_y, floor, old_r, source))

        last_error: Optional[Exception] = None
        new_joints: Optional[np.ndarray] = None
        chosen_xyzr: Optional[Tuple[float, float, float, float]] = None
        chosen_source = source
        # Try candidates closest to the demonstrated point first, then walk Z
        # upward only as much as needed to make the controller-space IK valid.
        pose_candidates.sort(key=lambda item: (item[0], math.hypot(item[1] - old_x, item[2] - old_y)))
        for _priority, cand_x, cand_y, cand_z0, cand_r, cand_source in pose_candidates:
            target_z = cand_z0
            max_z = max(cand_z0, float(old_xyzr[2]) + max_raise_mm)
            while target_z <= max_z + 1e-9:
                try:
                    candidate_joints = kin.inverse_kinematics([cand_x, cand_y, target_z, cand_r])
                    new_joints = candidate_joints
                    chosen_xyzr = (cand_x, cand_y, float(target_z), cand_r)
                    chosen_source = cand_source
                    break
                except ValueError as exc:
                    last_error = exc
                    target_z += raise_step_mm
            if new_joints is not None:
                break

        if new_joints is None:
            raise RuntimeError(
                f"scene repair failed at raw frame {idx}: "
                f"xyzr={old_xyzr.round(4).tolist()} floor={floor:.3f} "
                f"source={source} last_error={last_error}"
            )
        if chosen_xyzr is None:
            raise RuntimeError(f"scene repair invariant failed at raw frame {idx}")

        new_frame = dict(frame)
        new_frame.update({
            "j1": round(float(new_joints[0]), 6),
            "j2": round(float(new_joints[1]), 6),
            "j3": round(float(new_joints[2]), 6),
            "j4": round(float(new_joints[3]), 6),
        })
        repaired.append(new_frame)
        events.append(
            SceneRepairEvent(
                raw_index=idx,
                time_s=float(frame["timeStamp"]),
                source=chosen_source,
                old_xyzr=tuple(float(v) for v in old_xyzr[:4]),
                new_xyzr=chosen_xyzr,
                old_joints_deg=tuple(float(v) for v in old_joints[:4]),
                new_joints_deg=tuple(float(v) for v in new_joints[:4]),
                raised_by_mm=float(chosen_xyzr[2] - old_xyzr[2]),
                applied=True,
            )
        )

    return repaired, events


def _command_name(command: str) -> str:
    return command.split("(", 1)[0].strip()


def _command_numbers(command: str) -> List[float]:
    return [float(value) for value in NUMBER_RE.findall(command)]


def _sample_arc_xyz(points: np.ndarray, samples: int = 48) -> np.ndarray:
    """Sample the circular arc through current, through, target XYZ points.

    Falls back to the three control points if the geometry is degenerate.  This
    is only for visualization; the real command is still the compiled Arc text.
    """
    if points.shape[0] != 3:
        return points
    start = (points[0, 0], points[0, 1], points[0, 2], 0.0)
    through = (points[1, 0], points[1, 1], points[1, 2], 0.0)
    target = (points[2, 0], points[2, 1], points[2, 2], 0.0)
    sampled = sample_command_arc_xyzr(start, through, target, samples)
    if not sampled:
        return points
    return np.asarray([pose[:3] for pose in sampled], dtype=float)


def _point_to_polyline_distance(point: np.ndarray, polyline: np.ndarray) -> float:
    if len(polyline) == 0:
        return math.inf
    if len(polyline) == 1:
        return float(np.linalg.norm(point - polyline[0]))
    best = math.inf
    for a, b in zip(polyline[:-1], polyline[1:]):
        ab = b - a
        denom = float(np.dot(ab, ab))
        if denom < 1e-12:
            dist = float(np.linalg.norm(point - a))
        else:
            t = max(0.0, min(1.0, float(np.dot(point - a, ab) / denom)))
            dist = float(np.linalg.norm(point - (a + t * ab)))
        best = min(best, dist)
    return best


def _fit_errors(points_xyz: np.ndarray, primitive_path_xyz: np.ndarray) -> Tuple[float, float]:
    if len(points_xyz) == 0:
        return 0.0, 0.0
    distances = np.asarray([
        _point_to_polyline_distance(np.asarray(point[:3], dtype=float), primitive_path_xyz)
        for point in points_xyz
    ], dtype=float)
    return float(np.max(distances)), float(np.mean(distances))


def _command_segments(plan, waypoint_xyzr: np.ndarray) -> List[CommandSegment]:
    segments: List[CommandSegment] = []
    current_idx = 0
    current_xyz = waypoint_xyzr[0, :3]

    for command in plan.queued_commands:
        name = _command_name(command.command)
        end_idx = int(command.index)
        numbers = _command_numbers(command.command)

        if name in {"JointMovJ", "MovJ"} and len(numbers) >= 4:
            end_xyz = waypoint_xyzr[end_idx, :3]
            path = np.vstack([current_xyz, end_xyz])
            command_type = "JointMovJ"
        elif name == "MovL" and len(numbers) >= 4:
            end_xyz = np.asarray(numbers[:3], dtype=float)
            path = np.vstack([current_xyz, end_xyz])
            command_type = "MovL"
        elif name == "Arc" and len(numbers) >= 8:
            through_xyz = np.asarray(numbers[:3], dtype=float)
            end_xyz = np.asarray(numbers[4:7], dtype=float)
            path = _sample_arc_xyz(np.vstack([current_xyz, through_xyz, end_xyz]))
            command_type = "Arc"
        else:
            end_xyz = waypoint_xyzr[end_idx, :3]
            path = np.vstack([current_xyz, end_xyz])
            command_type = name or "Unknown"

        segments.append(
            CommandSegment(
                command_index=int(command.index),
                command_type=command_type,
                start_waypoint_index=current_idx,
                end_waypoint_index=end_idx,
                path_xyz=path,
                command=command.command,
            )
        )
        current_idx = end_idx
        current_xyz = path[-1]

    return segments


def _attach_fit_errors(
    segments: Sequence[CommandSegment],
    raw_xyzr: np.ndarray,
    kept_xyzr: np.ndarray,
    kept_raw_indices: Sequence[int],
) -> List[CommandSegment]:
    enriched: List[CommandSegment] = []
    for segment in segments:
        raw_start = int(kept_raw_indices[segment.start_waypoint_index])
        raw_end = int(kept_raw_indices[segment.end_waypoint_index])
        raw_slice = raw_xyzr[min(raw_start, raw_end):max(raw_start, raw_end) + 1, :3]
        kept_slice = kept_xyzr[segment.start_waypoint_index:segment.end_waypoint_index + 1, :3]
        raw_max, raw_mean = _fit_errors(raw_slice, segment.path_xyz)
        kept_max, kept_mean = _fit_errors(kept_slice, segment.path_xyz)
        enriched.append(
            CommandSegment(
                command_index=segment.command_index,
                command_type=segment.command_type,
                start_waypoint_index=segment.start_waypoint_index,
                end_waypoint_index=segment.end_waypoint_index,
                path_xyz=segment.path_xyz,
                command=segment.command,
                raw_start_index=raw_start,
                raw_end_index=raw_end,
                raw_fit_max_mm=raw_max,
                raw_fit_mean_mm=raw_mean,
                kept_fit_max_mm=kept_max,
                kept_fit_mean_mm=kept_mean,
            )
        )
    return enriched


def _set_equal_3d(ax, arrays: Sequence[np.ndarray]) -> None:
    pts = np.vstack([arr[:, :3] for arr in arrays if arr.size])
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    centers = (mins + maxs) / 2.0
    radius = max(float(np.max(maxs - mins)) / 2.0, 1.0)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def _set_equal_2d(ax, x: np.ndarray, y: np.ndarray, margin: float = 0.08) -> None:
    xmin, xmax = float(np.min(x)), float(np.max(x))
    ymin, ymax = float(np.min(y)), float(np.max(y))
    span = max(xmax - xmin, ymax - ymin, 1.0)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    r = span * (0.5 + margin)
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_aspect("equal", adjustable="box")


def _plot_projection(
    ax,
    title: str,
    x_label: str,
    y_label: str,
    raw: np.ndarray,
    kept: np.ndarray,
    dropped: np.ndarray,
    segments: Sequence[CommandSegment],
    axes: Tuple[int, int],
) -> None:
    x_i, y_i = axes

    ax.plot(raw[:, x_i], raw[:, y_i], color="#c7c7c7", linewidth=1.0, label="raw path")
    if len(dropped):
        ax.scatter(
            dropped[:, x_i],
            dropped[:, y_i],
            s=16,
            marker="x",
            color="#8e8e8e",
            alpha=0.55,
            label="filtered out",
        )

    cmap = plt.get_cmap("tab20")
    for ordinal, seg in enumerate(segments, 1):
        color = cmap((ordinal - 1) % 20)
        linestyle = TYPE_LINESTYLE.get(seg.command_type, "-")
        ax.plot(
            seg.path_xyz[:, x_i],
            seg.path_xyz[:, y_i],
            color=color,
            linestyle=linestyle,
            linewidth=2.6,
            alpha=0.92,
        )
        mid = seg.path_xyz[len(seg.path_xyz) // 2]
        ax.annotate(
            f"{ordinal}:{TYPE_LABEL.get(seg.command_type, '?')}",
            (mid[x_i], mid[y_i]),
            fontsize=7,
            color=color,
        )

    ax.scatter(
        kept[:, x_i],
        kept[:, y_i],
        s=34,
        facecolor="#1f77b4",
        edgecolor="white",
        linewidth=0.7,
        zorder=4,
        label="kept waypoints",
    )
    for idx, point in enumerate(kept):
        if idx == 0 or idx == len(kept) - 1 or idx % max(1, len(kept) // 12) == 0:
            ax.annotate(str(idx), (point[x_i], point[y_i]), fontsize=7, color="#0b3d91")

    _set_equal_2d(ax, raw[:, x_i], raw[:, y_i])
    ax.grid(True, alpha=0.25)
    ax.set_title(title)
    ax.set_xlabel(f"{x_label} (mm)")
    ax.set_ylabel(f"{y_label} (mm)")


def _render_figure(
    out_png: Path,
    title: str,
    raw_xyzr: np.ndarray,
    kept_xyzr: np.ndarray,
    dropped_xyzr: np.ndarray,
    segments: Sequence[CommandSegment],
    summary: dict,
) -> None:
    fig = plt.figure(figsize=(18, 12), constrained_layout=True)
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_top = fig.add_subplot(2, 2, 2)
    ax_front = fig.add_subplot(2, 2, 3)
    ax_side = fig.add_subplot(2, 2, 4)

    cmap = plt.get_cmap("tab20")
    ax3d.plot(raw_xyzr[:, 0], raw_xyzr[:, 1], raw_xyzr[:, 2], color="#c7c7c7", linewidth=1.0, label="raw path")
    if len(dropped_xyzr):
        ax3d.scatter(
            dropped_xyzr[:, 0],
            dropped_xyzr[:, 1],
            dropped_xyzr[:, 2],
            s=12,
            marker="x",
            color="#8e8e8e",
            alpha=0.45,
            label="filtered out",
        )
    for ordinal, seg in enumerate(segments, 1):
        color = cmap((ordinal - 1) % 20)
        linestyle = TYPE_LINESTYLE.get(seg.command_type, "-")
        ax3d.plot(
            seg.path_xyz[:, 0],
            seg.path_xyz[:, 1],
            seg.path_xyz[:, 2],
            color=color,
            linestyle=linestyle,
            linewidth=2.8,
            alpha=0.95,
        )
        mid = seg.path_xyz[len(seg.path_xyz) // 2]
        ax3d.text(
            mid[0],
            mid[1],
            mid[2],
            f"{ordinal}:{TYPE_LABEL.get(seg.command_type, '?')}",
            color=color,
            fontsize=7,
        )
    ax3d.scatter(
        kept_xyzr[:, 0],
        kept_xyzr[:, 1],
        kept_xyzr[:, 2],
        s=34,
        facecolor="#1f77b4",
        edgecolor="white",
        linewidth=0.7,
        label="kept waypoints",
    )
    ax3d.set_title("3D overview")
    ax3d.set_xlabel("X (mm)")
    ax3d.set_ylabel("Y (mm)")
    ax3d.set_zlabel("Z (mm)")
    _set_equal_3d(ax3d, [raw_xyzr, kept_xyzr])
    ax3d.view_init(elev=24, azim=-58)

    _plot_projection(ax_top, "Top view (X-Y)", "X", "Y", raw_xyzr, kept_xyzr, dropped_xyzr, segments, (0, 1))
    _plot_projection(ax_front, "Front view (X-Z)", "X", "Z", raw_xyzr, kept_xyzr, dropped_xyzr, segments, (0, 2))
    _plot_projection(ax_side, "Side view (Y-Z)", "Y", "Z", raw_xyzr, kept_xyzr, dropped_xyzr, segments, (1, 2))

    ax3d.legend(
        handles=[
            Line2D([0], [0], color="#c7c7c7", linewidth=1.4, label="raw taught path"),
            Line2D([0], [0], marker="x", color="#8e8e8e", linestyle="None", label="filtered out"),
            Line2D([0], [0], marker="o", color="#1f77b4", linestyle="None", label="kept waypoint"),
            Line2D([0], [0], color="#333333", linestyle=TYPE_LINESTYLE["Arc"], linewidth=2.4, label="A = Arc"),
            Line2D([0], [0], color="#333333", linestyle=TYPE_LINESTYLE["MovL"], linewidth=2.4, label="L = MovL"),
            Line2D([0], [0], color="#333333", linestyle=TYPE_LINESTYLE["JointMovJ"], linewidth=2.4, label="J = JointMovJ"),
        ],
        loc="upper left",
        fontsize=8,
    )
    fig.suptitle(
        (
            f"{title}\n"
            f"raw {summary['raw_waypoint_count']} -> kept {summary['kept_waypoint_count']} "
            f"-> commands {summary['queued_command_count']} | "
            f"JointMovJ {summary['command_type_counts'].get('JointMovJ', 0)}, "
            f"MovL {summary['command_type_counts'].get('MovL', 0)}, "
            f"Arc {summary['command_type_counts'].get('Arc', 0)}"
        ),
        fontsize=13,
    )
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def _render_segment_detail_figures(
    out_prefix: Path,
    title: str,
    raw_xyzr: np.ndarray,
    kept_xyzr: np.ndarray,
    segments: Sequence[CommandSegment],
    *,
    per_page: int = 8,
) -> List[Path]:
    """Render one local fit figure per emitted command segment.

    The overview figure shows the whole trajectory.  These per-command cards
    answer the surgical review question next to the human override controls:
    which raw points did this command replace, what primitive did we choose,
    and how far is that primitive from the original taught path?
    """
    if not segments:
        return []

    outputs: List[Path] = []
    cmap = plt.get_cmap("tab20")
    for ordinal, segment in enumerate(segments, start=1):
        fig, axes = plt.subplots(
            1,
            3,
            figsize=(12.8, 3.7),
            squeeze=False,
            constrained_layout=True,
        )
        row_axes = axes[0]
        color = cmap((ordinal - 1) % 20)

        raw_start = max(0, min(segment.raw_start_index, segment.raw_end_index))
        raw_end = min(len(raw_xyzr) - 1, max(segment.raw_start_index, segment.raw_end_index))
        raw_slice = raw_xyzr[raw_start:raw_end + 1, :3]
        kept_slice = kept_xyzr[
            segment.start_waypoint_index:segment.end_waypoint_index + 1,
            :3,
        ]

        title_prefix = (
            f"{ordinal}:{TYPE_LABEL.get(segment.command_type, '?')} {segment.command_type} "
            f"raw {segment.raw_start_index}->{segment.raw_end_index} "
            f"fit {segment.raw_fit_max_mm:.1f}/{segment.raw_fit_mean_mm:.1f}mm"
        )
        projections = (
            ("Top XY", (0, 1), "X", "Y"),
            ("Front XZ", (0, 2), "X", "Z"),
            ("Side YZ", (1, 2), "Y", "Z"),
        )
        for view_idx, (view_name, (x_i, y_i), x_label, y_label) in enumerate(projections):
            ax = row_axes[view_idx]
            if len(raw_slice):
                ax.plot(
                    raw_slice[:, x_i],
                    raw_slice[:, y_i],
                    color="#b9b9b9",
                    linewidth=1.2,
                    label="raw",
                )
                ax.scatter(
                    raw_slice[:, x_i],
                    raw_slice[:, y_i],
                    s=10,
                    color="#9a9a9a",
                    alpha=0.7,
                )

            linestyle = TYPE_LINESTYLE.get(segment.command_type, "-")
            ax.plot(
                segment.path_xyz[:, x_i],
                segment.path_xyz[:, y_i],
                color=color,
                linestyle=linestyle,
                linewidth=2.4,
                label=segment.command_type,
            )
            ax.scatter(
                kept_slice[:, x_i],
                kept_slice[:, y_i],
                s=28,
                facecolor="#1f77b4",
                edgecolor="white",
                linewidth=0.6,
                zorder=4,
                label="kept",
            )
            ax.scatter(
                segment.path_xyz[0, x_i],
                segment.path_xyz[0, y_i],
                marker="o",
                s=42,
                color="#2ca02c",
                zorder=5,
                label="start" if view_idx == 0 else None,
            )
            ax.scatter(
                segment.path_xyz[-1, x_i],
                segment.path_xyz[-1, y_i],
                marker="s",
                s=42,
                color="#d62728",
                zorder=5,
                label="end" if view_idx == 0 else None,
            )

            xs = np.concatenate([
                raw_slice[:, x_i] if len(raw_slice) else np.asarray([]),
                segment.path_xyz[:, x_i],
                kept_slice[:, x_i] if len(kept_slice) else np.asarray([]),
            ])
            ys = np.concatenate([
                raw_slice[:, y_i] if len(raw_slice) else np.asarray([]),
                segment.path_xyz[:, y_i],
                kept_slice[:, y_i] if len(kept_slice) else np.asarray([]),
            ])
            if len(xs) and len(ys):
                _set_equal_2d(ax, xs, ys, margin=0.18)

            ax.set_title(view_name, fontsize=8)
            ax.set_xlabel(f"{x_label} (mm)", fontsize=7)
            ax.set_ylabel(f"{y_label} (mm)", fontsize=7)
            ax.grid(True, alpha=0.22)
            ax.tick_params(labelsize=7)
            if view_idx == 0:
                ax.legend(fontsize=7, loc="best")

        out_path = out_prefix.with_name(f"{out_prefix.name}_segment_{ordinal:03d}.png")
        fig.suptitle(
            f"{title} — {title_prefix}",
            fontsize=12,
        )
        fig.savefig(out_path, dpi=170)
        plt.close(fig)
        outputs.append(out_path)
    return outputs


def _write_points_file(path: Path, plan) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Real-robot go.sh sends every non-empty line to port 30003, so this file
    # must contain Dobot commands only.  Metadata lives in *_commands_labeled.txt
    # and *_teach_filter_analysis.json.
    start = plan.waypoints[0]
    start_command = (
        f"JointMovJ({float(start['j1']):.4f},{float(start['j2']):.4f},"
        f"{float(start['j3']):.4f},{float(start['j4']):.4f},"
        "SpeedJ=30,AccJ=50,CP=0)"
    )
    lines = [start_command]
    lines.extend(command.command for command in plan.queued_commands)
    path.write_text("\n".join(lines) + "\n")


def _write_labeled_commands(path: Path, plan, segments: Sequence[CommandSegment]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Compiled MG400 commands with primitive labels",
        f"# source: {plan.source_name}",
        "",
        "# 000 START_MOVE wp 0 -> exact trajectory start pose",
        (
            f"JointMovJ({float(plan.waypoints[0]['j1']):.4f},"
            f"{float(plan.waypoints[0]['j2']):.4f},"
            f"{float(plan.waypoints[0]['j3']):.4f},"
            f"{float(plan.waypoints[0]['j4']):.4f},SpeedJ=30,AccJ=50,CP=0)"
        ),
    ]
    for i, (command, segment) in enumerate(zip(plan.queued_commands, segments), start=1):
        lines.append(
            f"# {i:03d} {segment.command_type} "
            f"wp {segment.start_waypoint_index}->{segment.end_waypoint_index} "
            f"raw {segment.raw_start_index}->{segment.raw_end_index} "
            f"t={command.target_time_s:.3f}s cp={command.cp} "
            f"raw_err max/mean={segment.raw_fit_max_mm:.2f}/{segment.raw_fit_mean_mm:.2f}mm"
        )
        lines.append(command.command)
    path.write_text("\n".join(lines) + "\n")


def _write_segments_csv(path: Path, plan, segments: Sequence[CommandSegment]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "ordinal",
        "type",
        "kept_start",
        "kept_end",
        "raw_start",
        "raw_end",
        "raw_fit_max_mm",
        "raw_fit_mean_mm",
        "kept_fit_max_mm",
        "kept_fit_mean_mm",
        "target_time_s",
        "cp",
        "command",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for i, (command, segment) in enumerate(zip(plan.queued_commands, segments), start=1):
            writer.writerow({
                "ordinal": i,
                "type": segment.command_type,
                "kept_start": segment.start_waypoint_index,
                "kept_end": segment.end_waypoint_index,
                "raw_start": segment.raw_start_index,
                "raw_end": segment.raw_end_index,
                "raw_fit_max_mm": f"{segment.raw_fit_max_mm:.6f}",
                "raw_fit_mean_mm": f"{segment.raw_fit_mean_mm:.6f}",
                "kept_fit_max_mm": f"{segment.kept_fit_max_mm:.6f}",
                "kept_fit_mean_mm": f"{segment.kept_fit_mean_mm:.6f}",
                "target_time_s": f"{command.target_time_s:.6f}",
                "cp": command.cp,
                "command": command.command,
            })


def _write_waypoint_csvs(
    raw_csv: Path,
    kept_csv: Path,
    raw_frames: Sequence[dict],
    kept_frames: Sequence[dict],
    raw_xyzr: np.ndarray,
    kept_xyzr: np.ndarray,
) -> Tuple[List[int], List[int]]:
    raw_csv.parent.mkdir(parents=True, exist_ok=True)
    kept_csv.parent.mkdir(parents=True, exist_ok=True)

    key_to_raw = {
        round(float(frame["timeStamp"]), 9): idx
        for idx, frame in enumerate(raw_frames)
    }
    kept_raw_indices = [
        key_to_raw[round(float(frame["timeStamp"]), 9)]
        for frame in kept_frames
    ]
    kept_lookup = {raw_idx: kept_idx for kept_idx, raw_idx in enumerate(kept_raw_indices)}
    dropped_raw_indices = [
        idx for idx in range(len(raw_frames))
        if idx not in kept_lookup
    ]

    columns = [
        "raw_index",
        "kept",
        "kept_index",
        "timeStamp",
        "j1",
        "j2",
        "j3",
        "j4",
        "x_mm",
        "y_mm",
        "z_mm",
        "r_deg",
    ]
    with raw_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for idx, (frame, pose) in enumerate(zip(raw_frames, raw_xyzr)):
            writer.writerow({
                "raw_index": idx,
                "kept": int(idx in kept_lookup),
                "kept_index": kept_lookup.get(idx, ""),
                "timeStamp": frame["timeStamp"],
                "j1": frame["j1"],
                "j2": frame["j2"],
                "j3": frame["j3"],
                "j4": frame["j4"],
                "x_mm": f"{pose[0]:.6f}",
                "y_mm": f"{pose[1]:.6f}",
                "z_mm": f"{pose[2]:.6f}",
                "r_deg": f"{pose[3]:.6f}",
            })

    kept_columns = [
        "kept_index",
        "raw_index",
        "timeStamp",
        "j1",
        "j2",
        "j3",
        "j4",
        "x_mm",
        "y_mm",
        "z_mm",
        "r_deg",
    ]
    with kept_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=kept_columns)
        writer.writeheader()
        for kept_idx, (raw_idx, frame, pose) in enumerate(zip(kept_raw_indices, kept_frames, kept_xyzr)):
            writer.writerow({
                "kept_index": kept_idx,
                "raw_index": raw_idx,
                "timeStamp": frame["timeStamp"],
                "j1": frame["j1"],
                "j2": frame["j2"],
                "j3": frame["j3"],
                "j4": frame["j4"],
                "x_mm": f"{pose[0]:.6f}",
                "y_mm": f"{pose[1]:.6f}",
                "z_mm": f"{pose[2]:.6f}",
                "r_deg": f"{pose[3]:.6f}",
            })

    return kept_raw_indices, dropped_raw_indices


def _write_scene_repair_csv(path: Path, events: Sequence[SceneRepairEvent]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "raw_index",
        "timeStamp",
        "source",
        "old_x_mm",
        "old_y_mm",
        "old_z_mm",
        "old_r_deg",
        "new_x_mm",
        "new_y_mm",
        "new_z_mm",
        "new_r_deg",
        "raised_by_mm",
        "old_j1",
        "old_j2",
        "old_j3",
        "old_j4",
        "new_j1",
        "new_j2",
        "new_j3",
        "new_j4",
        "applied",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for event in events:
            writer.writerow({
                "raw_index": event.raw_index,
                "timeStamp": f"{event.time_s:.6f}",
                "source": event.source,
                "old_x_mm": f"{event.old_xyzr[0]:.6f}",
                "old_y_mm": f"{event.old_xyzr[1]:.6f}",
                "old_z_mm": f"{event.old_xyzr[2]:.6f}",
                "old_r_deg": f"{event.old_xyzr[3]:.6f}",
                "new_x_mm": f"{event.new_xyzr[0]:.6f}",
                "new_y_mm": f"{event.new_xyzr[1]:.6f}",
                "new_z_mm": f"{event.new_xyzr[2]:.6f}",
                "new_r_deg": f"{event.new_xyzr[3]:.6f}",
                "raised_by_mm": f"{event.raised_by_mm:.6f}",
                "old_j1": f"{event.old_joints_deg[0]:.6f}",
                "old_j2": f"{event.old_joints_deg[1]:.6f}",
                "old_j3": f"{event.old_joints_deg[2]:.6f}",
                "old_j4": f"{event.old_joints_deg[3]:.6f}",
                "new_j1": f"{event.new_joints_deg[0]:.6f}",
                "new_j2": f"{event.new_joints_deg[1]:.6f}",
                "new_j3": f"{event.new_joints_deg[2]:.6f}",
                "new_j4": f"{event.new_joints_deg[3]:.6f}",
                "applied": "yes" if event.applied else "no",
            })


def _analyze_one(
    source: Path,
    out_dir: Path,
    points_dir: Path,
    *,
    scene_model: Optional[Path] = None,
    apply_scene_repair: bool = False,
    scene_contact_clearance_mm: float = 3.0,
    scene_avoid_clearance_mm: float = 20.0,
    scene_max_raise_mm: float = 240.0,
    scene_raise_step_mm: float = 2.0,
) -> dict:
    logger = _CliLogger()
    recorder = TrajectoryRecorder(
        command_send_fn=lambda _cmd: True,
        logger=logger,
        traj_dir=str(source.parent),
    )
    original_frames, loaded_events = _load_unity_json(source)
    repaired_frames, scene_repair_events = _scene_repair_frames(
        original_frames,
        scene_model,
        apply_repair=apply_scene_repair,
        contact_clearance_mm=scene_contact_clearance_mm,
        avoid_clearance_mm=scene_avoid_clearance_mm,
        max_raise_mm=scene_max_raise_mm,
        raise_step_mm=scene_raise_step_mm,
    )
    if not recorder.load_frames(repaired_frames, name=source.name, events=loaded_events):
        raise RuntimeError(f"failed to load {source}")

    raw_frames = list(recorder.loaded_frames)
    raw_xyzr = _frames_to_xyzr(raw_frames)
    tolerance = float(getattr(motion_config, "PATH_SIMPLIFY_TOLERANCE_DEG", 0.0))
    simplified_frames = recorder._simplify_loaded_frames(raw_frames, tolerance)
    plan = recorder.compile_loaded_plan()
    # plan.waypoints may be retimed, so use the direct simplifier output for
    # the visual kept/dropped split.  It is the same waypoint set that
    # compile_loaded_plan() feeds into retiming and mixed-primitive emission.
    kept_frames = list(simplified_frames)
    kept_xyzr = _frames_to_xyzr(kept_frames)

    kept_keys = {round(float(frame["timeStamp"]), 9) for frame in kept_frames}
    dropped_frames = [
        frame for frame in raw_frames
        if round(float(frame["timeStamp"]), 9) not in kept_keys
    ]
    dropped_xyzr = _frames_to_xyzr(dropped_frames) if dropped_frames else np.empty((0, 4))

    stem = source.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    points_dir.mkdir(parents=True, exist_ok=True)
    figure_path = out_dir / f"{stem}_teach_filter_analysis.png"
    segment_detail_prefix = out_dir / f"{stem}_teach_filter"
    json_path = out_dir / f"{stem}_teach_filter_analysis.json"
    compiled_path = out_dir / f"{stem}.compiled_playback.json"
    labeled_path = out_dir / f"{stem}_commands_labeled.txt"
    raw_csv_path = out_dir / f"{stem}_raw_waypoints.csv"
    kept_csv_path = out_dir / f"{stem}_kept_waypoints.csv"
    segments_csv_path = out_dir / f"{stem}_segments.csv"
    scene_repair_csv_path = out_dir / f"{stem}_scene_repair.csv"
    points_path = points_dir / f"{stem}_compiled_points.txt"

    compiled_path.write_text(
        json.dumps(compiled_playback_plan_to_dict(plan), indent=2, ensure_ascii=False)
        + "\n"
    )
    _write_points_file(points_path, plan)
    kept_raw_indices, dropped_raw_indices = _write_waypoint_csvs(
        raw_csv_path,
        kept_csv_path,
        raw_frames,
        kept_frames,
        raw_xyzr,
        kept_xyzr,
    )
    segments = _attach_fit_errors(_command_segments(plan, kept_xyzr), raw_xyzr, kept_xyzr, kept_raw_indices)
    command_type_counts = Counter(segment.command_type for segment in segments)
    _write_labeled_commands(labeled_path, plan, segments)
    _write_segments_csv(segments_csv_path, plan, segments)
    _write_scene_repair_csv(scene_repair_csv_path, scene_repair_events)
    segment_detail_paths = _render_segment_detail_figures(
        segment_detail_prefix,
        source.name,
        raw_xyzr,
        kept_xyzr,
        segments,
    )

    summary = {
        "source": str(source),
        "scene_repair": {
            "enabled": scene_model is not None,
            "applied": apply_scene_repair,
            "model": str(scene_model) if scene_model is not None else "",
            "contact_clearance_mm": scene_contact_clearance_mm,
            "avoid_clearance_mm": scene_avoid_clearance_mm,
            "max_raise_mm": scene_max_raise_mm,
            "raise_step_mm": scene_raise_step_mm,
            "repaired_frame_count": len(scene_repair_events),
            "applied_frame_count": sum(1 for e in scene_repair_events if e.applied),
            "max_raise_observed_mm": max((e.raised_by_mm for e in scene_repair_events), default=0.0),
            "events": [
                {
                    "raw_index": e.raw_index,
                    "timeStamp": e.time_s,
                    "source": e.source,
                    "old_xyzr": [round(v, 6) for v in e.old_xyzr],
                    "new_xyzr": [round(v, 6) for v in e.new_xyzr],
                    "raised_by_mm": round(e.raised_by_mm, 6),
                    "applied": e.applied,
                    "old_joints_deg": [round(v, 6) for v in e.old_joints_deg],
                    "new_joints_deg": [round(v, 6) for v in e.new_joints_deg],
                }
                for e in scene_repair_events
            ],
        },
        "raw_waypoint_count": len(raw_frames),
        "kept_waypoint_count": len(kept_frames),
        "filtered_out_count": len(dropped_frames),
        "queued_command_count": len(plan.queued_commands),
        "event_command_count": len(plan.event_commands),
        "command_type_counts": dict(command_type_counts),
        "kept_raw_indices": kept_raw_indices,
        "dropped_raw_indices": dropped_raw_indices,
        "original_duration_s": plan.original_duration_s,
        "retimed_duration_s": plan.retimed_duration_s,
        "total_duration_s": plan.total_duration_s,
        "time_scale": plan.time_scale,
        "original_timing_feasible": plan.original_timing_feasible,
        "execution_profile": plan.execution_profile,
        "config": {
            "PATH_SIMPLIFY_TOLERANCE_DEG": float(getattr(motion_config, "PATH_SIMPLIFY_TOLERANCE_DEG", 0.0)),
            "USE_MIXED_PRIMITIVES": bool(getattr(motion_config, "USE_MIXED_PRIMITIVES", False)),
            "SEGMENT_ENABLE_ARC": bool(getattr(motion_config, "SEGMENT_ENABLE_ARC", False)),
            "SEGMENT_LINE_TOLERANCE_MM": float(getattr(motion_config, "SEGMENT_LINE_TOLERANCE_MM", 0.0)),
            "SEGMENT_ARC_TOLERANCE_MM": float(getattr(motion_config, "SEGMENT_ARC_TOLERANCE_MM", 0.0)),
            "SEGMENT_R_TOLERANCE_DEG": float(getattr(motion_config, "SEGMENT_R_TOLERANCE_DEG", 0.0)),
            "PLAYBACK_EXECUTION_PROFILE": str(getattr(motion_config, "PLAYBACK_EXECUTION_PROFILE", "")),
        },
        "commands": [
            {
                "ordinal": i + 1,
                "type": segment.command_type,
                "start_waypoint_index": segment.start_waypoint_index,
                "end_waypoint_index": segment.end_waypoint_index,
                "target_time_s": plan.queued_commands[i].target_time_s,
                "original_target_time_s": plan.queued_commands[i].original_target_time_s,
                "joints_deg": list(plan.queued_commands[i].joints_deg),
                "cp": plan.queued_commands[i].cp,
                "raw_start_index": segment.raw_start_index,
                "raw_end_index": segment.raw_end_index,
                "raw_fit_max_mm": segment.raw_fit_max_mm,
                "raw_fit_mean_mm": segment.raw_fit_mean_mm,
                "kept_fit_max_mm": segment.kept_fit_max_mm,
                "kept_fit_mean_mm": segment.kept_fit_mean_mm,
                "command": plan.queued_commands[i].command,
            }
            for i, segment in enumerate(segments)
        ],
        "outputs": {
            "figure_png": str(figure_path),
            "analysis_json": str(json_path),
            "compiled_playback_json": str(compiled_path),
            "labeled_commands_txt": str(labeled_path),
            "raw_waypoints_csv": str(raw_csv_path),
            "kept_waypoints_csv": str(kept_csv_path),
            "segments_csv": str(segments_csv_path),
            "scene_repair_csv": str(scene_repair_csv_path),
            "segment_detail_pngs": [str(path) for path in segment_detail_paths],
            "real_robot_points_txt": str(points_path),
        },
        "pipeline_logs": logger.messages,
    }

    _render_figure(figure_path, source.name, raw_xyzr, kept_xyzr, dropped_xyzr, segments, summary)
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    return summary


def _default_out_dir() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return _WORKSPACE / "_supporting_materials/generated" / f"teach_filter_analysis_{stamp}"


def _config_override_map(args) -> dict:
    overrides = {}
    if args.use_mixed_primitives:
        overrides["USE_MIXED_PRIMITIVES"] = True
    if args.joint_only:
        overrides["USE_MIXED_PRIMITIVES"] = False
    if args.disable_arc:
        overrides["SEGMENT_ENABLE_ARC"] = False
    if args.enable_arc:
        overrides["SEGMENT_ENABLE_ARC"] = True
    if args.simplify_tol_deg is not None:
        overrides["PATH_SIMPLIFY_TOLERANCE_DEG"] = float(args.simplify_tol_deg)
    if args.line_tol_mm is not None:
        overrides["SEGMENT_LINE_TOLERANCE_MM"] = float(args.line_tol_mm)
    if args.arc_tol_mm is not None:
        overrides["SEGMENT_ARC_TOLERANCE_MM"] = float(args.arc_tol_mm)
    if args.raw_fit_tol_mm is not None:
        overrides["SEGMENT_RAW_FIT_TOLERANCE_MM"] = float(args.raw_fit_tol_mm)
    if args.r_tol_deg is not None:
        overrides["SEGMENT_R_TOLERANCE_DEG"] = float(args.r_tol_deg)
    if args.max_arc_radius_mm is not None:
        overrides["SEGMENT_MAX_ARC_RADIUS_MM"] = float(args.max_arc_radius_mm)
    return overrides


def _apply_motion_config_overrides(overrides: dict) -> dict:
    previous = {}
    for name, value in overrides.items():
        previous[name] = getattr(motion_config, name)
        setattr(motion_config, name, value)
    return previous


def _restore_motion_config(previous: dict) -> None:
    for name, value in previous.items():
        setattr(motion_config, name, value)


def _relative_path(path: Path, base: Path) -> str:
    try:
        return path.relative_to(base).as_posix()
    except ValueError:
        return path.as_posix()


def _html_escape(value) -> str:
    return html.escape(str(value), quote=True)


def _render_review_html(index_path: Path, summaries: Sequence[dict], out_dir: Path) -> Path:
    """Write a small offline review GUI for primitive decisions.

    The page deliberately has no dependencies.  It lets the operator inspect
    every command segment, assign an override/label, add a note, and download
    the annotations JSON for the next tuning pass.
    """
    rows = []
    options = ("auto", "line", "arc", "joint", "split", "reject")
    for traj_idx, summary in enumerate(summaries):
        outputs = summary["outputs"]
        fig_rel = _relative_path(Path(outputs["figure_png"]), out_dir)
        detail_rels = [_relative_path(Path(p), out_dir) for p in outputs.get("segment_detail_pngs", [])]
        counts = summary["command_type_counts"]
        rows.append(
            "<section class='trajectory'>"
            f"<h2>{_html_escape(Path(summary['source']).name)}</h2>"
            "<div class='summary'>"
            f"raw <b>{summary['raw_waypoint_count']}</b> -> kept <b>{summary['kept_waypoint_count']}</b> "
            f"-> commands <b>{summary['queued_command_count']}</b> | "
            f"J {counts.get('JointMovJ', 0)} / L {counts.get('MovL', 0)} / A {counts.get('Arc', 0)} | "
            f"max raw fit <b>{max((cmd['raw_fit_max_mm'] for cmd in summary['commands']), default=0.0):.2f} mm</b>"
            "</div>"
            "<figure class='image-card'>"
            f"<img class='overview' src='{_html_escape(fig_rel)}' alt='overview'>"
            "<figcaption>"
            "<b>Overview only</b> — ใช้ดูภาพรวมของ path ทั้งไฟล์. การเลือก/แก้ decision อยู่ใต้รูป command เล็ก ๆ ด้านล่าง."
            "</figcaption>"
            "</figure>"
        )
        if detail_rels:
            rows.append("<details open><summary>Per-command fit and override cards</summary>")
            rows.append("<div class='command-grid'>")
            for cmd, rel in zip(summary["commands"], detail_rels):
                row_id = f"t{traj_idx}_c{cmd['ordinal']}"
                option_html = "".join(
                    f"<option value='{opt}'>{opt}</option>"
                    for opt in options
                )
                rows.append(
                    "<figure class='command-card'>"
                    f"<img class='detail' src='{_html_escape(rel)}' alt='segment {cmd['ordinal']} detail'>"
                    "<figcaption>"
                    "<div class='legend-row'>"
                    "<span><b>gray</b> raw</span>"
                    "<span><b>blue dots</b> kept</span>"
                    "<span><b>green circle</b> start</span>"
                    "<span><b>red square</b> end</span>"
                    "<span><b>A</b> Arc</span>"
                    "<span><b>L</b> MovL</span>"
                    "<span><b>J</b> JointMovJ</span>"
                    "</div>"
                    "<div class='command-meta'>"
                    f"<b>#{cmd['ordinal']} {TYPE_LABEL.get(cmd['type'], '?')} {cmd['type']}</b> "
                    f"raw {cmd['raw_start_index']}->{cmd['raw_end_index']} | "
                    f"fit max/mean {cmd['raw_fit_max_mm']:.2f}/{cmd['raw_fit_mean_mm']:.2f} mm"
                    "</div>"
                    "<div class='card-controls'>"
                    "<label>ควรเป็น<select "
                    f"data-row='{row_id}' data-kind='label'>{option_html}</select></label>"
                    "<label>note<input "
                    f"data-row='{row_id}' data-kind='note' placeholder='เช่น ควร split / arc ฝั่งซ้าย'></label>"
                    "</div>"
                    "</figcaption>"
                    "</figure>"
                )
            rows.append("</div>")
            rows.append("</details>")

        rows.append("<table><thead><tr>"
                    "<th>#</th><th>auto</th><th>raw</th><th>fit max/mean</th>"
                    "<th>command</th>"
                    "</tr></thead><tbody>")
        for cmd in summary["commands"]:
            rows.append(
                "<tr>"
                f"<td>{cmd['ordinal']}</td>"
                f"<td><span class='pill {cmd['type']}'>{_html_escape(cmd['type'])}</span></td>"
                f"<td>{cmd['raw_start_index']}->{cmd['raw_end_index']}</td>"
                f"<td>{cmd['raw_fit_max_mm']:.2f}/{cmd['raw_fit_mean_mm']:.2f} mm</td>"
                f"<td><code>{_html_escape(cmd['command'])}</code></td>"
                "</tr>"
            )
        rows.append("</tbody></table></section>")

    payload = base64.b64encode(json.dumps(summaries, ensure_ascii=False).encode("utf-8")).decode("ascii")
    html_text = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Teach Filter Review</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 24px; background: #f6f7f9; color: #1f2933; }}
h1 {{ margin-bottom: 4px; }}
.toolbar {{ position: sticky; top: 0; z-index: 3; background: #f6f7f9; padding: 12px 0; border-bottom: 1px solid #d8dee8; }}
button {{ border: 0; background: #1f6feb; color: white; border-radius: 6px; padding: 9px 12px; cursor: pointer; margin-right: 8px; }}
button.secondary {{ background: #4b5563; }}
.trajectory {{ background: white; border: 1px solid #d8dee8; border-radius: 8px; padding: 16px; margin: 18px 0 28px; box-shadow: 0 1px 3px rgba(15,23,42,.05); }}
.summary {{ margin: 8px 0 12px; color: #4b5563; }}
.image-card {{ margin: 12px 0 18px; }}
.command-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(520px, 1fr)); gap: 16px; margin: 12px 0 18px; }}
.command-card {{ margin: 0; border: 1px solid #e5e7eb; border-radius: 8px; padding: 10px; background: #fbfcfe; }}
figcaption {{ max-width: 1450px; color: #4b5563; font-size: 13px; line-height: 1.45; padding: 8px 2px 0; }}
img.overview {{ width: 100%; max-width: 1450px; display: block; border: 1px solid #e5e7eb; border-radius: 6px; background: white; }}
img.detail {{ width: 100%; display: block; border: 1px solid #e5e7eb; border-radius: 6px; background: white; }}
.legend-row {{ display: flex; flex-wrap: wrap; gap: 8px 14px; margin-bottom: 6px; }}
.command-meta {{ margin: 4px 0 8px; color: #334155; }}
.card-controls {{ display: grid; grid-template-columns: minmax(130px, 180px) minmax(220px, 1fr); gap: 8px; align-items: end; }}
.card-controls label {{ display: grid; gap: 3px; color: #475569; font-weight: 600; }}
table {{ border-collapse: collapse; width: 100%; margin-top: 12px; font-size: 13px; }}
th, td {{ border-bottom: 1px solid #e5e7eb; text-align: left; vertical-align: top; padding: 8px; }}
th {{ background: #f3f4f6; position: sticky; top: 60px; z-index: 2; }}
code {{ white-space: pre-wrap; word-break: break-word; color: #334155; }}
input, select {{ width: 100%; box-sizing: border-box; padding: 6px; border: 1px solid #cbd5e1; border-radius: 5px; }}
.pill {{ display: inline-block; min-width: 68px; text-align: center; border-radius: 99px; padding: 3px 8px; color: white; font-weight: 700; font-size: 12px; }}
.Arc {{ background: #dc2626; }}
.MovL {{ background: #2563eb; }}
.JointMovJ {{ background: #7c3aed; }}
.hint {{ color: #6b7280; max-width: 980px; line-height: 1.45; }}
</style>
</head>
<body>
<h1>Teach Filter Review</h1>
<p class="hint">สี/label: <b>A/Arc</b> = โค้ง Dobot Arc, <b>L/MovL</b> = เส้นตรง Cartesian, <b>J/JointMovJ</b> = fallback รายจุด. เลือก label/note แล้วกด Download annotations เพื่อเอากลับไป tuning ได้.</p>
<div class="toolbar">
  <button onclick="downloadAnnotations()">Download annotations JSON</button>
  <button class="secondary" onclick="clearAnnotations()">Clear local choices</button>
  <span id="saveState" class="hint"></span>
</div>
{''.join(rows)}
<script>
const summaries = JSON.parse(atob("{payload}"));
const storageKey = "teach-filter-review:" + location.pathname;
function loadState() {{
  let state = {{}};
  try {{ state = JSON.parse(localStorage.getItem(storageKey) || "{{}}"); }} catch (e) {{ state = {{}}; }}
  document.querySelectorAll("[data-row]").forEach(el => {{
    const row = el.dataset.row;
    const kind = el.dataset.kind;
    if (state[row] && state[row][kind] !== undefined) el.value = state[row][kind];
    el.addEventListener("input", saveState);
    el.addEventListener("change", saveState);
  }});
}}
function saveState() {{
  const state = {{}};
  document.querySelectorAll("[data-row]").forEach(el => {{
    const row = el.dataset.row;
    const kind = el.dataset.kind;
    state[row] = state[row] || {{}};
    state[row][kind] = el.value;
  }});
  localStorage.setItem(storageKey, JSON.stringify(state));
  document.getElementById("saveState").textContent = "saved locally";
}}
function collectAnnotations() {{
  let state = {{}};
  try {{ state = JSON.parse(localStorage.getItem(storageKey) || "{{}}"); }} catch (e) {{ state = {{}}; }}
  const annotations = [];
  summaries.forEach((summary, ti) => {{
    summary.commands.forEach(cmd => {{
      const row = `t${{ti}}_c${{cmd.ordinal}}`;
      const entry = state[row] || {{}};
      if ((entry.label && entry.label !== "auto") || entry.note) {{
        annotations.push({{
          source: summary.source,
          ordinal: cmd.ordinal,
          raw_start_index: cmd.raw_start_index,
          raw_end_index: cmd.raw_end_index,
          auto_type: cmd.type,
          user_label: entry.label || "auto",
          note: entry.note || "",
          command: cmd.command,
          raw_fit_max_mm: cmd.raw_fit_max_mm,
          raw_fit_mean_mm: cmd.raw_fit_mean_mm
        }});
      }}
    }});
  }});
  return {{ generated_at: new Date().toISOString(), annotations }};
}}
function downloadAnnotations() {{
  const blob = new Blob([JSON.stringify(collectAnnotations(), null, 2)], {{ type: "application/json" }});
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "teach_filter_annotations.json";
  a.click();
  URL.revokeObjectURL(url);
}}
function clearAnnotations() {{
  localStorage.removeItem(storageKey);
  document.querySelectorAll("[data-row]").forEach(el => el.value = el.tagName === "SELECT" ? "auto" : "");
  document.getElementById("saveState").textContent = "cleared";
}}
loadState();
</script>
</body>
</html>
"""
    index_path.write_text(html_text, encoding="utf-8")
    return index_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Visualize the production teach-and-repeat filtering pipeline."
    )
    parser.add_argument(
        "--trajectory-json",
        action="append",
        default=[],
        help="Trajectory JSON to analyze. Repeat for multiple files. Defaults to money/Pick_place_1/Pick_place_2.",
    )
    parser.add_argument(
        "--all-json",
        action="store_true",
        help="Analyze every *.json file in _supporting_materials/data/trajectories/json_trajectories.",
    )
    parser.add_argument(
        "--out-dir",
        default=str(_default_out_dir()),
        help="Directory for figures and analysis JSON.",
    )
    parser.add_argument(
        "--points-out-dir",
        default=str(_WORKSPACE / "_supporting_materials/tools/robot_control_tools/generated_points"),
        help="Directory for generated MG400 command txt files for real-robot tests.",
    )
    parser.add_argument(
        "--scene-repair-model",
        default="",
        help=(
            "Optional scene safety model JSON. When set, unsafe raw frames are "
            "reported before simplification and primitive selection. "
            "Use --apply-scene-repair only for explicit experimental mutation."
        ),
    )
    parser.add_argument(
        "--apply-scene-repair",
        action="store_true",
        help=(
            "Experimental: mutate raw frames using the scene model before "
            "compilation. Default is diagnostic-only."
        ),
    )
    parser.add_argument(
        "--diagnose-scene-only",
        action="store_true",
        help=(
            "With --scene-repair-model, only report unsafe raw frames and do not "
            "mutate the generated trajectory."
        ),
    )
    parser.add_argument("--scene-contact-clearance-mm", type=float, default=3.0)
    parser.add_argument("--scene-avoid-clearance-mm", type=float, default=20.0)
    parser.add_argument("--scene-max-raise-mm", type=float, default=240.0)
    parser.add_argument("--scene-raise-step-mm", type=float, default=2.0)
    parser.add_argument("--use-mixed-primitives", action="store_true", help="Temporarily enable MovL/Arc analysis mode.")
    parser.add_argument("--joint-only", action="store_true", help="Temporarily force JointMovJ-only analysis mode.")
    parser.add_argument("--enable-arc", action="store_true", help="Temporarily enable Arc classification.")
    parser.add_argument("--disable-arc", action="store_true", help="Temporarily disable Arc classification.")
    parser.add_argument("--simplify-tol-deg", type=float, default=None, help="Override PATH_SIMPLIFY_TOLERANCE_DEG.")
    parser.add_argument("--line-tol-mm", type=float, default=None, help="Override SEGMENT_LINE_TOLERANCE_MM.")
    parser.add_argument("--arc-tol-mm", type=float, default=None, help="Override SEGMENT_ARC_TOLERANCE_MM.")
    parser.add_argument("--raw-fit-tol-mm", type=float, default=None, help="Override SEGMENT_RAW_FIT_TOLERANCE_MM.")
    parser.add_argument("--r-tol-deg", type=float, default=None, help="Override SEGMENT_R_TOLERANCE_DEG.")
    parser.add_argument("--max-arc-radius-mm", type=float, default=None, help="Override SEGMENT_MAX_ARC_RADIUS_MM.")
    parser.add_argument("--no-html-review", action="store_true", help="Skip the offline HTML review UI.")
    args = parser.parse_args()

    sources = [Path(p).expanduser().resolve() for p in args.trajectory_json]
    if args.all_json:
        sources.extend(sorted(DEFAULT_JSON_DIR.glob("*.json")))
    if not sources:
        sources = [p.resolve() for p in DEFAULT_TRAJECTORIES]
    # Preserve order but remove duplicates when --all-json and explicit paths overlap.
    deduped_sources = []
    seen = set()
    for source in sources:
        if source not in seen:
            deduped_sources.append(source)
            seen.add(source)
    sources = deduped_sources
    missing = [p for p in sources if not p.is_file()]
    if missing:
        for path in missing:
            print(f"missing trajectory: {path}", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir).expanduser().resolve()
    points_dir = Path(args.points_out_dir).expanduser().resolve()
    scene_model = Path(args.scene_repair_model).expanduser().resolve() if args.scene_repair_model else None
    if scene_model is not None and not scene_model.is_file():
        print(f"missing scene repair model: {scene_model}", file=sys.stderr)
        return 2
    apply_scene_repair = bool(scene_model is not None and args.apply_scene_repair)
    if args.diagnose_scene_only:
        apply_scene_repair = False
    overrides = _config_override_map(args)
    previous_config = _apply_motion_config_overrides(overrides)
    if overrides:
        print("analysis config overrides:")
        for name, value in sorted(overrides.items()):
            print(f"  {name}={value}")
    all_summaries = []
    try:
        for source in sources:
            print(f"\n=== analyzing {source.name} ===")
            summary = _analyze_one(
                source,
                out_dir,
                points_dir,
                scene_model=scene_model,
                apply_scene_repair=apply_scene_repair,
                scene_contact_clearance_mm=args.scene_contact_clearance_mm,
                scene_avoid_clearance_mm=args.scene_avoid_clearance_mm,
                scene_max_raise_mm=args.scene_max_raise_mm,
                scene_raise_step_mm=args.scene_raise_step_mm,
            )
            all_summaries.append(summary)
            counts = summary["command_type_counts"]
            print(
                f"{source.name}: raw {summary['raw_waypoint_count']} -> "
                f"kept {summary['kept_waypoint_count']} -> "
                f"commands {summary['queued_command_count']} "
                f"(JointMovJ={counts.get('JointMovJ', 0)}, "
                f"MovL={counts.get('MovL', 0)}, Arc={counts.get('Arc', 0)})"
            )
            repair = summary["scene_repair"]
            if repair["enabled"]:
                print(
                    f"  scene repair: {repair['repaired_frame_count']} frame(s) flagged, "
                    f"{repair['applied_frame_count']} applied, "
                    f"max raise {repair['max_raise_observed_mm']:.1f} mm"
                )
            print(f"  figure: {summary['outputs']['figure_png']}")
            print(f"  points: {summary['outputs']['real_robot_points_txt']}")
    finally:
        _restore_motion_config(previous_config)

    index_path = out_dir / "teach_filter_analysis_index.json"
    index_path.write_text(json.dumps(all_summaries, indent=2, ensure_ascii=False) + "\n")
    if not args.no_html_review:
        html_path = _render_review_html(out_dir / "teach_filter_review.html", all_summaries, out_dir)
        print(f"review_html: {html_path}")
    print(f"\nindex: {index_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
