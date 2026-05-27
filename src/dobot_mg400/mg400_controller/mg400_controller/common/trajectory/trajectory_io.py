#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Trajectory file I/O — pure functions extracted from TrajectoryRecorder.

This module contains the save / load / list helpers used by
``TrajectoryRecorder`` and the simulator's Unity-JSON importer.

Behaviour is intentionally identical to the pre-extraction code:

- ``save_trajectory`` writes ``{"frames": [...], "events": [...]}`` JSON
  in the same shape the operator-facing JSON files have always used.
- ``load_trajectory`` accepts both the wrapped ``{"frames": ...}`` form
  and the legacy bare-list form (raw Unity dumps).
- ``parse_unity_trajectory_json`` parses imported Unity trajectory files and
  keeps the default filename ``unity_trajectory.json``.
- ``list_trajectory_files`` returns sorted ``*.json`` file names from
  ``traj_dir`` and silently returns an empty list if the directory is
  missing — matching the recorder's prior behaviour.

No robot movement or ROS subscription behaviour lives in this module.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Sequence, Tuple


def save_trajectory(
    path: str,
    frames: Sequence[Dict],
    events: Sequence[Dict],
    logger,
) -> str:
    """Write a trajectory JSON file with the recorder's wire shape.

    Returns the written path on success or the empty string on failure.
    The signature accepts ``logger`` so callers can keep their existing
    ``info`` / ``error`` plumbing without this module owning a logger.
    """
    if not frames:
        logger.error("No frames to save")
        return ""
    try:
        with open(path, "w") as f:
            json.dump({"frames": list(frames), "events": list(events)}, f, indent=4)
        logger.info(
            f"💾 Saved {len(frames)} frames, {len(events)} events → {path}"
        )
        return path
    except Exception as exc:  # noqa: BLE001 — propagate via logger like recorder did
        logger.error(f"Save failed: {exc}")
        return ""


def load_trajectory(
    path: str,
    logger,
) -> Tuple[Optional[List[Dict]], Optional[List[Dict]]]:
    """Read a trajectory JSON file and return ``(frames, events)``.

    Both elements are ``None`` on failure, matching the recorder's prior
    error contract.  The function tolerates the legacy bare-list form
    (older Unity dumps that wrote ``[frame, frame, ...]`` directly).
    """
    if not os.path.isfile(path):
        logger.error(f"Trajectory file not found: {path}")
        return None, None
    try:
        with open(path, "r") as f:
            data = json.load(f)
    except Exception as exc:  # noqa: BLE001
        logger.error(f"Failed to load trajectory: {exc}")
        return None, None

    frames = data if isinstance(data, list) else data.get("frames", [])
    events = [] if isinstance(data, list) else data.get("events", [])
    if not frames:
        logger.error("Trajectory file is empty")
        return None, None
    if not isinstance(events, list):
        events = []
    return list(frames), list(events)


def parse_unity_trajectory_json(
    json_str: str,
    logger,
) -> Tuple[List[Dict], List[Dict], str]:
    """Parse an imported Unity trajectory JSON payload.

    Returns ``(frames, events, filename)``.  ``filename`` defaults to
    ``unity_trajectory.json`` if the payload does not specify one.

    On parse failure all three return values are empty / default; the
    recorder layer treats that as "do nothing".
    """
    try:
        data = json.loads(json_str)
    except Exception as exc:  # noqa: BLE001
        logger.error(f"Failed to parse Unity trajectory JSON: {exc}")
        return [], [], "unity_trajectory.json"

    if isinstance(data, list):
        return list(data), [], "unity_trajectory.json"

    frames = list(data.get("frames", []))
    events_raw = data.get("events", [])
    events = list(events_raw) if isinstance(events_raw, list) else []
    filename = data.get("filename", "unity_trajectory.json")
    return frames, events, filename


def list_trajectory_files(traj_dir: str) -> List[str]:
    """Return sorted ``*.json`` files in ``traj_dir`` (silent on missing dir)."""
    try:
        return sorted(f for f in os.listdir(traj_dir) if f.endswith(".json"))
    except Exception:
        return []


def resolve_trajectory_path(traj_dir: str, name: str) -> str:
    """Apply the recorder's "add .json if missing" convention.

    Centralised so save_as / load see the same normalisation rule.
    """
    if not name.endswith(".json"):
        name = f"{name}.json"
    return os.path.join(traj_dir, name)
