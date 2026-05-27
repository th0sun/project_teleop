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
from typing import Dict, List, Optional, Tuple


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


def resolve_trajectory_path(traj_dir: str, name: str) -> str:
    """Apply the recorder's "add .json if missing" convention.

    Centralised so save_as / load see the same normalisation rule.
    """
    if not name.endswith(".json"):
        name = f"{name}.json"
    return os.path.join(traj_dir, name)
