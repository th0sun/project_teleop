#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Measured scene safety boxes for MG400 realtime and teach-repeat guards.

The model is intentionally advisory/configurable.  With the guard disabled it
does nothing.  When enabled it blocks only clear collision risks:

* entering an ``avoid`` safety box
* going too far below a measured contact/support surface

Contact-allowed workpiece volumes are allowed so pick/place points can still be
taught near the fixture.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from mg400_controller.common.utils.kinematics import KinematicsCalculator


XYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class SafetyBox:
    name: str
    role: str
    policy: str
    raw_min: XYZ
    raw_max: XYZ
    safe_min: XYZ
    safe_max: XYZ


@dataclass(frozen=True)
class SceneSafetyResult:
    status: str
    detail: str
    distance_mm: float
    point_xyzr: Tuple[float, float, float, float]
    blocked: bool

    @property
    def ok(self) -> bool:
        return not self.blocked


class SceneSafetyGuard:
    """Check MG400 TCP targets against measured scene boxes."""

    BLOCKING_STATUSES = {"INSIDE", "SURFACE_HIT", "DEEP_CONTACT"}

    def __init__(
        self,
        *,
        model_path: str = "",
        enabled: bool = False,
        warn_distance_mm: float = 20.0,
        deep_contact_mm: float = 3.0,
        kinematics: Optional[KinematicsCalculator] = None,
        logger=None,
    ) -> None:
        self.enabled = bool(enabled)
        self.warn_distance_mm = float(warn_distance_mm)
        self.deep_contact_mm = float(deep_contact_mm)
        self.model_path = str(model_path or "")
        self._log = logger
        self._kinematics = kinematics or KinematicsCalculator()
        self._boxes: List[SafetyBox] = []
        if self.model_path:
            self.load_model(self.model_path)

    @property
    def loaded(self) -> bool:
        return bool(self._boxes)

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)

    def load_model(self, model_path: str) -> None:
        path = Path(model_path).expanduser()
        if not path.exists():
            self._boxes = []
            if self._log:
                self._log.warn(f"Scene safety model not found: {path}")
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        boxes: List[SafetyBox] = []
        for primitive in data.get("primitives", []):
            safe = primitive.get("safety_box") or {}
            raw = primitive.get("raw_box") or {}
            try:
                boxes.append(
                    SafetyBox(
                        name=str(primitive.get("object_name", "unknown")),
                        role=str(primitive.get("role", "")),
                        policy=str(primitive.get("collision_policy", "avoid")),
                        raw_min=tuple(float(v) for v in raw["min_xyz"][:3]),  # type: ignore[index]
                        raw_max=tuple(float(v) for v in raw["max_xyz"][:3]),  # type: ignore[index]
                        safe_min=tuple(float(v) for v in safe["min_xyz"][:3]),  # type: ignore[index]
                        safe_max=tuple(float(v) for v in safe["max_xyz"][:3]),  # type: ignore[index]
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                if self._log:
                    self._log.warn(f"Skipping invalid scene safety primitive: {exc}")
        self._boxes = boxes
        self.model_path = str(path)
        if self._log:
            self._log.info(f"Loaded scene safety model: {path} ({len(boxes)} boxes)")

    def check_joints_rad(self, joints_rad: Sequence[float]) -> SceneSafetyResult:
        joints_deg = np.degrees(np.asarray(joints_rad[:4], dtype=float))
        return self.check_joints_deg(joints_deg)

    def check_joints_deg(self, joints_deg: Sequence[float]) -> SceneSafetyResult:
        xyzr = self._kinematics.forward_kinematics(joints_deg)
        return self.check_tcp(xyzr)

    def check_plan(self, plan) -> List[SceneSafetyResult]:
        if not self.enabled or not self.loaded:
            return []
        results: List[SceneSafetyResult] = []
        for command in getattr(plan, "queued_commands", ()):
            result = self.check_joints_deg(getattr(command, "joints_deg", ()))
            if result.blocked:
                results.append(result)
        return results

    def check_tcp(self, xyzr: Sequence[float]) -> SceneSafetyResult:
        point = np.asarray(xyzr[:4], dtype=float)
        if not self.enabled:
            return self._result("DISABLED", "", math.inf, point, blocked=False)
        if not self.loaded:
            return self._result("NO_MODEL", "", math.inf, point, blocked=False)
        status, detail, distance = self._nearest_status(point)
        return self._result(
            status,
            detail,
            distance,
            point,
            blocked=status in self.BLOCKING_STATUSES,
        )

    def _nearest_status(self, point: np.ndarray) -> Tuple[str, str, float]:
        inside_avoid = [
            box.name
            for box in self._boxes
            if box.policy == "avoid" and self._point_inside_box(point, box)
        ]
        if inside_avoid:
            return ("INSIDE", ",".join(inside_avoid[:4]), 0.0)

        contact_boxes = [
            box
            for box in self._boxes
            if box.policy == "contact_allowed" and self._point_inside_box(point, box)
        ]
        surface_hit = [
            f"{box.name}:{box.raw_min[2] - float(point[2]):.1f}mm"
            for box in contact_boxes
            if box.role == "support_surface"
            and float(point[2]) < box.raw_min[2] - self.deep_contact_mm
        ]
        if surface_hit:
            return ("SURFACE_HIT", ",".join(surface_hit[:4]), 0.0)

        deep_contact = [
            f"{box.name}:{box.raw_min[2] - float(point[2]):.1f}mm"
            for box in contact_boxes
            if float(point[2]) < box.raw_min[2] - self.deep_contact_mm
        ]
        if deep_contact:
            return ("DEEP_CONTACT", ",".join(deep_contact[:4]), 0.0)

        if contact_boxes:
            return ("CONTACT", ",".join(box.name for box in contact_boxes[:4]), 0.0)

        avoid_boxes = [box for box in self._boxes if box.policy == "avoid"] or list(self._boxes)
        distances = [(self._distance_to_box(point, box), box.name) for box in avoid_boxes]
        if not distances:
            return ("NO_MODEL", "", math.inf)
        distance, name = min(distances)
        if distance <= self.warn_distance_mm:
            return ("NEAR", name, distance)
        return ("OK", name, distance)

    @staticmethod
    def _point_inside_box(point: np.ndarray, box: SafetyBox) -> bool:
        x, y, z = [float(v) for v in point[:3]]
        return (
            box.safe_min[0] <= x <= box.safe_max[0]
            and box.safe_min[1] <= y <= box.safe_max[1]
            and box.safe_min[2] <= z <= box.safe_max[2]
        )

    @staticmethod
    def _distance_to_box(point: np.ndarray, box: SafetyBox) -> float:
        d2 = 0.0
        for axis in range(3):
            value = float(point[axis])
            lo = box.safe_min[axis]
            hi = box.safe_max[axis]
            if value < lo:
                d2 += (lo - value) ** 2
            elif value > hi:
                d2 += (value - hi) ** 2
        return math.sqrt(d2)

    @staticmethod
    def _result(
        status: str,
        detail: str,
        distance: float,
        point: Iterable[float],
        *,
        blocked: bool,
    ) -> SceneSafetyResult:
        xyzr = tuple(float(v) for v in list(point)[:4])
        while len(xyzr) < 4:
            xyzr = (*xyzr, 0.0)
        return SceneSafetyResult(status, detail, float(distance), xyzr, blocked)
