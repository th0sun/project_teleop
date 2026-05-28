#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pending-command registry for command-id fallback matching.

When the robot does not provide a usable command id, this registry lets the
runtime match a settled feedback pose back to the most likely recent ROS
command target while explicitly flagging ambiguous matches.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


@dataclass
class PendingCommand:
    control_command_seq: int
    ros_command_uid: str
    target_rad: np.ndarray
    target_tool: Optional[np.ndarray] = None
    dobot_command_id: int | None = None
    command_text: str | None = None
    unity_sample: Any = None
    sent_wall_timestamp: float = 0.0
    feedback_command_id_before_send: int | None = None
    result_logged: bool = False
    joint_match_logged: bool = False
    tool_match_logged: bool = False


@dataclass(frozen=True)
class CommandMatchResult:
    command: PendingCommand | None
    status: str
    method: str
    confidence: str
    command_id_match: bool | None
    ambiguous: bool
    candidate_count: int
    match_error_rad: float | None
    second_best_error_rad: float | None
    match_age_ms: float | None
    pending_count: int


@dataclass(frozen=True)
class CommandPassSample:
    command: PendingCommand
    event_type: str
    status: str
    method: str
    confidence: str
    candidate_count: int
    joint_norm_error_rad: float | None
    joint_max_error_rad: float | None
    tool_xyz_error_mm: float | None
    tool_r_error_deg: float | None
    match_age_ms: float
    pending_count: int


class PendingCommandRegistry:
    def __init__(self, max_size: int = 100):
        self.max_size = max_size
        self._commands: list[PendingCommand] = []

    def register(self, command: PendingCommand) -> None:
        self._commands.append(command)
        if len(self._commands) > self.max_size:
            self._commands = self._commands[-self.max_size:]

    def active_commands(self) -> list[PendingCommand]:
        return [cmd for cmd in self._commands if not cmd.result_logged]

    def mark_logged(self, match: CommandMatchResult) -> None:
        if match.command is not None and _is_terminal_match(match):
            match.command.result_logged = True

    def pass_samples(
        self,
        q_current,
        *,
        tool_actual=None,
        now_wall: float,
        norm_tolerance_rad: float,
        per_joint_tolerance_rad: float,
        tool_xyz_tolerance_mm: float,
        tool_r_tolerance_deg: float,
    ) -> list[CommandPassSample]:
        """Return first-time pass-near samples for active commands.

        These samples are diagnostic only. They do not require the robot to stop,
        so they work for CP/blended motion where intermediate commands may be
        passed through rather than settled.
        """
        active = self.active_commands()
        pending_count = len(active)
        if not active:
            return []

        q = np.asarray(q_current, dtype=float)
        joint_scores = [_score_command(cmd, q, now_wall) for cmd in active]
        joint_hits = [
            item for item in joint_scores
            if _within_settle_tolerance(
                item,
                norm_tolerance_rad=norm_tolerance_rad,
                per_joint_tolerance_rad=per_joint_tolerance_rad,
            )
        ]

        samples: list[CommandPassSample] = []
        for item in joint_hits:
            command = item["command"]
            if command.joint_match_logged:
                continue
            command.joint_match_logged = True
            samples.append(CommandPassSample(
                command=command,
                event_type="joint_match",
                status="passed_near_4j",
                method="joint4_pass_near",
                confidence="medium",
                candidate_count=len(joint_hits),
                joint_norm_error_rad=item["norm_error"],
                joint_max_error_rad=item["max_joint_error"],
                tool_xyz_error_mm=None,
                tool_r_error_deg=None,
                match_age_ms=item["age_ms"],
                pending_count=pending_count,
            ))

        if tool_actual is None:
            return samples

        tool_scores = [
            item for item in (
                _score_tool_command(cmd, tool_actual, now_wall)
                for cmd in active
                if cmd.target_tool is not None
            )
            if item is not None
        ]
        tool_hits = [
            item for item in tool_scores
            if item["xyz_error_mm"] <= tool_xyz_tolerance_mm
            and item["r_error_deg"] <= tool_r_tolerance_deg
        ]
        for item in tool_hits:
            command = item["command"]
            if command.tool_match_logged:
                continue
            command.tool_match_logged = True
            samples.append(CommandPassSample(
                command=command,
                event_type="tool_match",
                status="passed_near_xyz",
                method="tool_xyz_pass_near",
                confidence="medium",
                candidate_count=len(tool_hits),
                joint_norm_error_rad=None,
                joint_max_error_rad=None,
                tool_xyz_error_mm=item["xyz_error_mm"],
                tool_r_error_deg=item["r_error_deg"],
                match_age_ms=item["age_ms"],
                pending_count=pending_count,
            ))

        return samples

    def match(
        self,
        q_current,
        *,
        feedback_command_id: int | None,
        now_wall: float,
        norm_tolerance_rad: float,
        per_joint_tolerance_rad: float,
    ) -> CommandMatchResult:
        active = self.active_commands()
        pending_count = len(active)
        if not active:
            return CommandMatchResult(
                command=None,
                status="no_pending_command",
                method="none",
                confidence="low",
                command_id_match=None,
                ambiguous=False,
                candidate_count=0,
                match_error_rad=None,
                second_best_error_rad=None,
                match_age_ms=None,
                pending_count=0,
            )

        q = np.asarray(q_current, dtype=float)
        scored = sorted(
            (_score_command(cmd, q, now_wall) for cmd in active),
            key=lambda item: (item["norm_error"], -item["sent_wall_timestamp"]),
        )

        usable_feedback_id = feedback_command_id not in (None, 0)
        if usable_feedback_id:
            id_matches = [
                item for item in scored
                if item["command"].dobot_command_id == feedback_command_id
            ]
            if id_matches:
                best = id_matches[0]
                if not _within_settle_tolerance(
                    best,
                    norm_tolerance_rad=norm_tolerance_rad,
                    per_joint_tolerance_rad=per_joint_tolerance_rad,
                ):
                    return CommandMatchResult(
                        command=best["command"],
                        status="command_id_pose_mismatch",
                        method="dobot_id_pose_outside_tolerance",
                        confidence="low",
                        command_id_match=True,
                        ambiguous=len(id_matches) > 1,
                        candidate_count=len(id_matches),
                        match_error_rad=best["norm_error"],
                        second_best_error_rad=_second_error(id_matches),
                        match_age_ms=best["age_ms"],
                        pending_count=pending_count,
                    )
                return CommandMatchResult(
                    command=best["command"],
                    status="reached",
                    method="dobot_id_and_pose_settle",
                    confidence="high",
                    command_id_match=True,
                    ambiguous=len(id_matches) > 1,
                    candidate_count=len(id_matches),
                    match_error_rad=best["norm_error"],
                    second_best_error_rad=_second_error(id_matches),
                    match_age_ms=best["age_ms"],
                    pending_count=pending_count,
                )

        pose_candidates = [
            item for item in scored
            if item["norm_error"] <= norm_tolerance_rad
            and item["max_joint_error"] <= per_joint_tolerance_rad
        ]
        if pose_candidates:
            best = pose_candidates[0]
            command = best["command"]
            ambiguous = len(pose_candidates) > 1
            id_mismatch = (
                usable_feedback_id
                and command.dobot_command_id is not None
                and command.dobot_command_id != feedback_command_id
            )
            if id_mismatch:
                status = "settled_pose_id_mismatch"
                method = "pose_registry_nearest_id_mismatch"
                confidence = "low"
                command_id_match = False
            else:
                status = "reached_pose_ambiguous" if ambiguous else "reached_pose_only"
                method = "pose_registry_nearest"
                confidence = "low" if ambiguous else "medium"
                command_id_match = None

            return CommandMatchResult(
                command=command,
                status=status,
                method=method,
                confidence=confidence,
                command_id_match=command_id_match,
                ambiguous=ambiguous,
                candidate_count=len(pose_candidates),
                match_error_rad=best["norm_error"],
                second_best_error_rad=_second_error(pose_candidates),
                match_age_ms=best["age_ms"],
                pending_count=pending_count,
            )

        best = scored[0]
        return CommandMatchResult(
            command=best["command"],
            status="settled_pose_outside_registry_tolerance",
            method="pose_registry_nearest_outside_tolerance",
            confidence="low",
            command_id_match=None,
            ambiguous=True,
            candidate_count=0,
            match_error_rad=best["norm_error"],
            second_best_error_rad=_second_error(scored),
            match_age_ms=best["age_ms"],
            pending_count=pending_count,
        )


def _score_command(command: PendingCommand, q_current: np.ndarray, now_wall: float) -> dict:
    target = np.asarray(command.target_rad, dtype=float)
    diff = q_current[:4] - target[:4]
    return {
        "command": command,
        "norm_error": float(np.linalg.norm(diff)),
        "max_joint_error": float(np.max(np.abs(diff))),
        "age_ms": (now_wall - command.sent_wall_timestamp) * 1000.0,
        "sent_wall_timestamp": command.sent_wall_timestamp,
    }


def _score_tool_command(command: PendingCommand, tool_actual, now_wall: float) -> dict | None:
    if command.target_tool is None:
        return None
    actual = np.asarray(tool_actual, dtype=float)
    target = np.asarray(command.target_tool, dtype=float)
    if actual.shape[0] < 4 or target.shape[0] < 4:
        return None
    xyz_delta = actual[:3] - target[:3]
    return {
        "command": command,
        "xyz_error_mm": float(np.linalg.norm(xyz_delta)),
        "r_error_deg": float(abs(actual[3] - target[3])),
        "age_ms": (now_wall - command.sent_wall_timestamp) * 1000.0,
    }


def _within_settle_tolerance(
    scored: dict,
    *,
    norm_tolerance_rad: float,
    per_joint_tolerance_rad: float,
) -> bool:
    return (
        scored["norm_error"] <= norm_tolerance_rad
        and scored["max_joint_error"] <= per_joint_tolerance_rad
    )


def _is_terminal_match(match: CommandMatchResult) -> bool:
    return match.status in {
        "reached",
        "reached_pose_only",
        "reached_pose_ambiguous",
        "settled_pose_id_mismatch",
    }


def _second_error(scored: list[dict]) -> float | None:
    if len(scored) < 2:
        return None
    return scored[1]["norm_error"]
