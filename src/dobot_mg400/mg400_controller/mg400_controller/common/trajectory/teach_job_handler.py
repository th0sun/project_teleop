#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Teach-and-Repeat Job Request handler.

Implements the ``/teach/job_request`` contract documented in
``docs/report_materials/architecture_phase_research_log.md`` (section 19).

The handler is intentionally rclpy-free so it can be unit-tested without a
running ROS graph.  Wiring into ``vr_teleop_node.py`` only needs to provide:

* an instance of ``TrajectoryRecorder`` (capture/compile/playback engine),
* a ``publish_status_fn(json_str)`` callable bound to ``/teach/job_status``,
* a ``publish_artifact_fn(json_str)`` callable bound to ``/teach/job_artifact``,
* a logger with ``info / warn / error`` methods,
* an optional ``time_fn`` for deterministic tests.

JobRequest schema (JSON over ``std_msgs/String``)::

    {
      "job_id": "uuid-v4",
      "action": "compile|execute|tune|export|stop",
      "target": "mg400" | "robot_a" | ...,
      "trajectory": {
        "filename": "...",
        "frames": [{"timeStamp": s, "j1": deg, "j2": deg, "j3": deg, "j4": deg}, ...],
        "events": [{"timeStamp": s, "kind": "digital_output", "channel": "vacuum", "value": true}, ...]
      },
      "options": {
        "speed_scale": 1.0,
        "dry_run": false,
        "export_path": "abs/path.json",
        "auto_play": false
      },
      "submitted_at_unity_sec": 12345.6
    }

JobStatus schema::

    {
      "job_id": "...",
      "stage": "received|compiled|executing|tuned|done|failed|stopped",
      "progress": 0.0..1.0,
      "message": "...",
      "error_code": null | "BAD_PAYLOAD" | "EMPTY_TRAJECTORY" | "EXECUTE_FORBIDDEN" | "ALREADY_PLAYING" | "EXPORT_FAILED" | "UNKNOWN_ACTION" | "PLAYBACK_TIMEOUT" | "PLAYBACK_FAILED",
      "ros_time_sec": 1234.5,
      "metadata": {...}
    }
"""

import json
import os
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from mg400_controller.common.trajectory.trajectory_recorder import (
    TrajectoryRecorder,
    compiled_playback_plan_to_dict,
)


# ── Action constants (string-typed enum) ─────────────────────────────────────
ACTION_COMPILE = "compile"
ACTION_EXECUTE = "execute"
ACTION_TUNE = "tune"
ACTION_EXPORT = "export"
ACTION_STOP = "stop"

VALID_ACTIONS = frozenset({
    ACTION_COMPILE,
    ACTION_EXECUTE,
    ACTION_TUNE,
    ACTION_EXPORT,
    ACTION_STOP,
})

# Stages used in JobStatus.stage.
STAGE_RECEIVED = "received"
STAGE_COMPILED = "compiled"
STAGE_EXECUTING = "executing"
STAGE_TUNED = "tuned"
STAGE_DONE = "done"
STAGE_FAILED = "failed"
STAGE_STOPPED = "stopped"

# Error codes used in JobStatus.error_code.
ERR_BAD_PAYLOAD = "BAD_PAYLOAD"
ERR_EMPTY_TRAJECTORY = "EMPTY_TRAJECTORY"
ERR_EXECUTE_FORBIDDEN = "EXECUTE_FORBIDDEN"
ERR_ALREADY_PLAYING = "ALREADY_PLAYING"
ERR_EXPORT_FAILED = "EXPORT_FAILED"
ERR_UNKNOWN_ACTION = "UNKNOWN_ACTION"
ERR_TARGET_MISMATCH = "TARGET_MISMATCH"
ERR_PLAYBACK_TIMEOUT = "PLAYBACK_TIMEOUT"
ERR_PLAYBACK_FAILED = "PLAYBACK_FAILED"
ERR_SCENE_SAFETY = "SCENE_SAFETY_BLOCKED"


@dataclass(frozen=True)
class JobRequest:
    """Parsed teach-repeat job request."""

    job_id: str
    action: str
    target: str
    trajectory: Optional[Dict[str, Any]]
    options: Dict[str, Any]
    submitted_at_unity_sec: Optional[float]
    raw: Dict[str, Any]

    def frames(self) -> List[Dict[str, Any]]:
        """Return frames list from the embedded trajectory, or []."""
        if not self.trajectory:
            return []
        frames = self.trajectory.get("frames")
        return list(frames) if isinstance(frames, list) else []

    def events(self) -> List[Dict[str, Any]]:
        """Return captured task events from the embedded trajectory, or []."""
        if not self.trajectory:
            return []
        events = self.trajectory.get("events")
        return list(events) if isinstance(events, list) else []

    def filename(self) -> str:
        if not self.trajectory:
            return ""
        return str(self.trajectory.get("filename", "") or "")


def parse_job_request(json_str: str) -> JobRequest:
    """Parse a JSON string into a ``JobRequest``.

    Raises ``ValueError`` with a human-readable reason on malformed input.  The
    handler converts that to a JobStatus with ``error_code = BAD_PAYLOAD``.
    """
    if not isinstance(json_str, str) or not json_str.strip():
        raise ValueError("empty job_request payload")
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("job_request must be a JSON object")

    action = data.get("action")
    if not isinstance(action, str) or not action.strip():
        raise ValueError("missing 'action'")

    job_id = data.get("job_id")
    if not isinstance(job_id, str) or not job_id.strip():
        job_id = str(uuid.uuid4())

    target = data.get("target") or ""
    if not isinstance(target, str):
        raise ValueError("'target' must be a string when provided")

    trajectory = data.get("trajectory")
    if trajectory is not None and not isinstance(trajectory, dict):
        raise ValueError("'trajectory' must be an object when provided")

    options_raw = data.get("options")
    if options_raw is None:
        options_raw = {}
    if not isinstance(options_raw, dict):
        raise ValueError("'options' must be an object when provided")

    submitted = data.get("submitted_at_unity_sec")
    if submitted is not None:
        try:
            submitted = float(submitted)
        except (TypeError, ValueError) as exc:
            raise ValueError("'submitted_at_unity_sec' must be a number") from exc

    return JobRequest(
        job_id=job_id.strip(),
        action=action.strip(),
        target=target.strip(),
        trajectory=trajectory,
        options=dict(options_raw),
        submitted_at_unity_sec=submitted,
        raw=data,
    )


@dataclass
class JobStatus:
    job_id: str
    stage: str
    progress: float = 0.0
    message: str = ""
    error_code: Optional[str] = None
    ros_time_sec: float = 0.0
    action: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        payload = {
            "job_id": self.job_id,
            "stage": self.stage,
            "progress": float(max(0.0, min(1.0, self.progress))),
            "message": self.message,
            "error_code": self.error_code,
            "ros_time_sec": float(self.ros_time_sec),
            "action": self.action,
            "metadata": dict(self.metadata),
        }
        return json.dumps(payload, sort_keys=True)


class TeachJobHandler:
    """Dispatch teach-and-repeat job requests to a ``TrajectoryRecorder``.

    The handler keeps the request boundary explicit:
    * ``compile`` runs the trajectory through the recorder's compile-only path
      and returns a JSON artifact.  Robot is **not** moved.
    * ``execute`` requires real-robot consent (see ``allow_real_execute_fn``)
      and starts the playback worker.
    * ``tune`` updates playback tuning while a job is running.
    * ``export`` writes the compiled artifact to disk for inspection.
    * ``stop`` aborts any running playback.
    """

    def __init__(
        self,
        recorder: TrajectoryRecorder,
        publish_status_fn: Callable[[str], None],
        publish_artifact_fn: Callable[[str], None],
        logger,
        *,
        allow_real_execute_fn: Optional[Callable[[], bool]] = None,
        accepted_targets: Tuple[str, ...] = ("mg400", ""),
        scene_safety_guard=None,
        time_fn: Optional[Callable[[], float]] = None,
    ):
        self._recorder = recorder
        self._publish_status = publish_status_fn
        self._publish_artifact = publish_artifact_fn
        self._log = logger
        self._allow_real_execute = allow_real_execute_fn or (lambda: True)
        self._accepted_targets = tuple(accepted_targets)
        self._scene_safety_guard = scene_safety_guard
        self._time_fn = time_fn or time.time
        self._active_execute_request: Optional[JobRequest] = None
        add_event_cb = getattr(self._recorder, "add_playback_event_callback", None)
        if callable(add_event_cb):
            add_event_cb(self._handle_playback_event)

    # ── Public entry point ──────────────────────────────────────────────────
    def handle(self, json_str: str) -> JobStatus:
        try:
            request = parse_job_request(json_str)
        except ValueError as exc:
            status = self._build_status(
                job_id="",
                stage=STAGE_FAILED,
                message=f"Bad job_request payload: {exc}",
                error_code=ERR_BAD_PAYLOAD,
                action=None,
            )
            self._emit(status)
            return status

        # Acknowledge receipt early so Unity knows the request landed.
        self._emit(self._build_status(
            job_id=request.job_id,
            stage=STAGE_RECEIVED,
            message=f"Received '{request.action}'",
            action=request.action,
            metadata={"target": request.target, "filename": request.filename()},
        ))

        if request.action not in VALID_ACTIONS:
            return self._fail(request, ERR_UNKNOWN_ACTION,
                              f"Unknown action '{request.action}'")

        if request.target and request.target not in self._accepted_targets:
            return self._fail(request, ERR_TARGET_MISMATCH,
                              f"Target '{request.target}' not handled by this node")

        try:
            if request.action == ACTION_COMPILE:
                return self._handle_compile(request)
            if request.action == ACTION_EXECUTE:
                return self._handle_play(request, sim=False)
            if request.action == ACTION_TUNE:
                return self._handle_tune(request)
            if request.action == ACTION_EXPORT:
                return self._handle_export(request)
            if request.action == ACTION_STOP:
                return self._handle_stop(request)
        except Exception as exc:  # pragma: no cover - defensive
            self._log.error(f"Unhandled error in teach job '{request.action}': {exc}")
            return self._fail(request, ERR_BAD_PAYLOAD, str(exc))

        # Unreachable.
        return self._fail(request, ERR_UNKNOWN_ACTION, "no dispatch")

    # ── Action handlers ─────────────────────────────────────────────────────
    def _handle_compile(self, request: JobRequest) -> JobStatus:
        self._apply_request_tuning(request)
        if not self._load_trajectory_or_fail(request):
            return self._last_status

        try:
            plan = self._recorder.compile_loaded_plan()
        except Exception as exc:
            return self._fail(request, ERR_BAD_PAYLOAD, f"Compile failed: {exc}")

        scene_violations = self._scene_safety_violations(plan)
        artifact = compiled_playback_plan_to_dict(plan)
        artifact_payload = {
            "job_id": request.job_id,
            "artifact": artifact,
        }
        try:
            self._publish_artifact(json.dumps(artifact_payload, sort_keys=True))
        except Exception as exc:
            self._log.warn(f"Failed to publish compile artifact: {exc}")

        return self._succeed(
            request,
            stage=STAGE_COMPILED,
            message=f"Compiled {len(plan.queued_commands)} commands "
                    f"({plan.total_duration_s:.2f}s)",
            metadata={
                "waypoint_count": len(plan.waypoints),
                "raw_waypoint_count": int(getattr(plan, "raw_waypoint_count", len(plan.waypoints))),
                "simplify_tolerance_deg": float(getattr(plan, "simplify_tolerance_deg", 0.0)),
                "queued_command_count": len(plan.queued_commands),
                "event_command_count": len(getattr(plan, "event_commands", ())),
                "total_duration_s": plan.total_duration_s,
                "time_scale": plan.time_scale,
                "original_timing_feasible": plan.original_timing_feasible,
                "playback_tuning": self._current_tuning(),
                "scene_safety_enabled": self._scene_safety_enabled(),
                "scene_safety_blocked": bool(scene_violations),
                "scene_safety_violations": scene_violations[:5],
            },
        )

    def _handle_play(self, request: JobRequest, *, sim: bool) -> JobStatus:
        """Real-robot playback path.

        ``sim`` is retained as a parameter for symmetry but the only caller
        passes ``sim=False``. Validation is handled by ``compile``.
        """
        if sim:
            # Defensive: should not happen via the public dispatcher, but if a
            # future caller wires sim=True back in, fail loudly instead of
            # silently moving the robot.
            return self._fail(
                request,
                ERR_EXECUTE_FORBIDDEN,
                "Sim playback path disabled: use compile for dry-run validation.",
            )

        self._apply_request_tuning(request)
        if not self._load_trajectory_or_fail(request):
            return self._last_status

        if self._recorder.is_playing:
            return self._fail(request, ERR_ALREADY_PLAYING,
                              "Playback already in progress")

        if not self._allow_real_execute():
            return self._fail(request, ERR_EXECUTE_FORBIDDEN,
                              "Real-robot execute denied (robot disconnected or gated)")

        try:
            plan = self._recorder.compile_loaded_plan()
        except Exception as exc:
            self._active_execute_request = None
            return self._fail(request, ERR_BAD_PAYLOAD, f"Playback compile failed: {exc}")

        scene_violations = self._scene_safety_violations(plan)
        if scene_violations:
            first = scene_violations[0]
            return self._fail(
                request,
                ERR_SCENE_SAFETY,
                "Scene safety blocked execute: "
                f"{first.get('status')} {first.get('detail')} at {first.get('point_xyzr')}",
                metadata={
                    "scene_safety_enabled": True,
                    "scene_safety_violations": scene_violations[:5],
                },
            )

        try:
            self._active_execute_request = request
            self._recorder.start_preview()
        except Exception as exc:
            self._active_execute_request = None
            return self._fail(request, ERR_BAD_PAYLOAD, f"Playback start failed: {exc}")

        return self._succeed(
            request,
            stage=STAGE_EXECUTING,
            message="Real-robot execute started",
            metadata={
                "sim": False,
                "real_robot_moved": True,
                "waypoint_count": len(self._recorder.loaded_frames),
                "playback_tuning": self._current_tuning(),
                "scene_safety_enabled": self._scene_safety_enabled(),
            },
        )

    def _handle_tune(self, request: JobRequest) -> JobStatus:
        tuning = self._apply_request_tuning(request, merge=True)
        return self._succeed(
            request,
            stage=STAGE_TUNED,
            message="Playback tuning updated",
            metadata={"playback_tuning": tuning},
        )

    def _scene_safety_enabled(self) -> bool:
        guard = self._scene_safety_guard
        return bool(getattr(guard, "enabled", False) and getattr(guard, "loaded", False))

    def _scene_safety_violations(self, plan) -> List[Dict[str, Any]]:
        guard = self._scene_safety_guard
        if not self._scene_safety_enabled() or guard is None:
            return []
        check_plan = getattr(guard, "check_plan", None)
        if not callable(check_plan):
            return []
        violations = []
        for result in check_plan(plan):
            violations.append({
                "status": result.status,
                "detail": result.detail,
                "distance_mm": result.distance_mm,
                "point_xyzr": list(result.point_xyzr),
            })
        return violations

    def _handle_export(self, request: JobRequest) -> JobStatus:
        self._apply_request_tuning(request)
        if not self._load_trajectory_or_fail(request):
            return self._last_status

        export_path = request.options.get("export_path")
        if not isinstance(export_path, str) or not export_path.strip():
            export_path = os.path.join(
                self._recorder._traj_dir,
                f"compiled_{request.job_id[:8]}.json",
            )

        try:
            written = self._recorder.export_loaded_plan(export_path)
        except Exception as exc:
            return self._fail(request, ERR_EXPORT_FAILED, f"Export failed: {exc}")

        return self._succeed(
            request,
            stage=STAGE_DONE,
            message=f"Compiled artifact written to {written}",
            metadata={"export_path": written},
        )

    def _handle_stop(self, request: JobRequest) -> JobStatus:
        go_home = bool(request.options.get("go_home", False))
        try:
            self._recorder.stop_all(go_home=go_home)
        except Exception as exc:
            return self._fail(request, ERR_BAD_PAYLOAD, f"Stop failed: {exc}")
        return self._succeed(
            request,
            stage=STAGE_STOPPED,
            message="Recorder stopped" + (" (home)" if go_home else ""),
            metadata={"go_home": go_home},
        )

    def _handle_playback_event(self, event_name: str, payload: Dict[str, Any]):
        """Publish terminal job status when the recorder finishes playback."""
        if event_name != "playback_complete" or self._active_execute_request is None:
            return

        request = self._active_execute_request
        self._active_execute_request = None
        metadata = dict(payload or {})
        if metadata.get("stopped"):
            self._emit(self._build_status(
                job_id=request.job_id,
                stage=STAGE_STOPPED,
                message="Playback stopped",
                action=request.action,
                metadata=metadata,
            ))
            return

        if metadata.get("timed_out"):
            self._emit(self._build_status(
                job_id=request.job_id,
                stage=STAGE_FAILED,
                message="Playback timed out before final target was reached",
                error_code=ERR_PLAYBACK_TIMEOUT,
                action=request.action,
                metadata=metadata,
            ))
            return

        if not metadata.get("success", False):
            self._emit(self._build_status(
                job_id=request.job_id,
                stage=STAGE_FAILED,
                message="Playback ended without confirmed final settle",
                error_code=ERR_PLAYBACK_FAILED,
                action=request.action,
                metadata=metadata,
            ))
            return

        self._emit(self._build_status(
            job_id=request.job_id,
            stage=STAGE_DONE,
            message="Playback complete",
            action=request.action,
            metadata=metadata,
        ))

    # ── Helpers ─────────────────────────────────────────────────────────────
    def _apply_request_tuning(self, request: JobRequest, *, merge: bool = False) -> Dict[str, Any]:
        if merge and hasattr(self._recorder, "update_playback_tuning"):
            return dict(self._recorder.update_playback_tuning(request.options))
        if hasattr(self._recorder, "set_playback_tuning"):
            return dict(self._recorder.set_playback_tuning(request.options))
        return {}

    def _current_tuning(self) -> Dict[str, Any]:
        if hasattr(self._recorder, "playback_tuning"):
            return dict(self._recorder.playback_tuning())
        return {}

    def _load_trajectory_or_fail(self, request: JobRequest) -> bool:
        frames = request.frames()
        if frames:
            ok = self._recorder.load_frames(
                frames,
                name=request.filename() or "job_request_inline",
                events=request.events(),
            )
            if not ok:
                self._fail(request, ERR_EMPTY_TRAJECTORY,
                           "Trajectory frames invalid")
                return False
            return True

        # Fall back to whatever the recorder has already loaded (e.g. from a
        # previous Load: step or a prior compile request).
        if self._recorder.loaded_frames:
            return True

        self._fail(request, ERR_EMPTY_TRAJECTORY,
                   "No trajectory frames in request and recorder is empty")
        return False

    def _emit(self, status: JobStatus) -> JobStatus:
        try:
            self._publish_status(status.to_json())
        except Exception as exc:
            self._log.warn(f"Failed to publish job_status: {exc}")
        self._last_status = status
        return status

    def _build_status(self, *, job_id, stage, message, error_code=None,
                      action=None, metadata=None) -> JobStatus:
        return JobStatus(
            job_id=job_id,
            stage=stage,
            progress=1.0 if stage == STAGE_DONE else (
                0.0 if stage in (STAGE_RECEIVED, STAGE_FAILED) else 0.5
            ),
            message=message,
            error_code=error_code,
            ros_time_sec=float(self._time_fn()),
            action=action,
            metadata=dict(metadata or {}),
        )

    def _succeed(self, request: JobRequest, *, stage, message, metadata) -> JobStatus:
        status = self._build_status(
            job_id=request.job_id,
            stage=stage,
            message=message,
            action=request.action,
            metadata=metadata,
        )
        self._log.info(f"🎓 job_request[{request.action}/{request.job_id[:8]}]: {message}")
        return self._emit(status)

    def _fail(
        self,
        request: JobRequest,
        error_code: str,
        message: str,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> JobStatus:
        status = self._build_status(
            job_id=request.job_id,
            stage=STAGE_FAILED,
            message=message,
            error_code=error_code,
            action=request.action,
            metadata=metadata,
        )
        self._log.error(f"❌ job_request[{request.action}/{request.job_id[:8]}]: {message}")
        return self._emit(status)
