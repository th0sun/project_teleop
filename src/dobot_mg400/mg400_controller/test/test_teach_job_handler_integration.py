"""Phase 2 software-only integration test for the teach-repeat job bridge.

Exercises ``TeachJobHandler`` end-to-end with a real Unity-format JSON
trajectory captured from the editor::

    _supporting_materials/data/trajectories/json_trajectories/unity_mock_test.json

The test reproduces the request payload that
``TeleOp/Assets/Scripts/TeachJobPublisher.cs`` builds at runtime and asserts
the contract from architecture_phase_research_log.md §21:

* A1  ``compile``      → received → compiled, artifact published
* A2  ``preview_sim``  → received → preview_ready, real_robot_moved=False
* A3  ``execute``      → received → executing, playback engaged
* A4  ``execute`` while gated → received → failed/EXECUTE_FORBIDDEN

No ROS runtime is required.  The handler is rclpy-free; we substitute a
fake recorder that exposes only the surface ``TeachJobHandler`` actually
touches so the test stays deterministic and fast.
"""

import json
import os
import unittest
from pathlib import Path

from mg400_controller.common.trajectory.teach_job_handler import (
    ACTION_COMPILE,
    ACTION_EXECUTE,
    ACTION_PREVIEW_SIM,
    ERR_EXECUTE_FORBIDDEN,
    STAGE_COMPILED,
    STAGE_EXECUTING,
    STAGE_FAILED,
    STAGE_PREVIEW_READY,
    STAGE_RECEIVED,
    TeachJobHandler,
)


# ── Locate Unity sample JSON ────────────────────────────────────────────────
# The repo layout is:
#   project_teleop_ws/                <- workspace root
#     project_teleop/                 <- ROS repo (this test lives here)
#     _supporting_materials/data/...  <- shared sample assets (sibling dir)
#
# parents[0]=test/ [1]=mg400_controller/ [2]=dobot_mg400/ [3]=src/
# [4]=project_teleop/ [5]=project_teleop_ws/.  Fall back to an explicit env
# override for CI flexibility.
_DEFAULT_SAMPLE = (
    Path(__file__).resolve().parents[5]
    / "_supporting_materials"
    / "data"
    / "trajectories"
    / "json_trajectories"
    / "unity_mock_test.json"
)


def _resolve_sample_path() -> Path:
    override = os.environ.get("TEACH_REPEAT_SAMPLE_JSON")
    if override:
        return Path(override).expanduser().resolve()
    return _DEFAULT_SAMPLE


# ── Minimal fakes (mirror test_teach_job_handler.py style) ──────────────────
class _Logger:
    def __init__(self):
        self.errors = []
        self.warns = []

    def info(self, msg):
        pass

    def warn(self, msg, *a, **kw):
        self.warns.append(msg)

    def error(self, msg, *a, **kw):
        self.errors.append(msg)


class _CompiledPlanStub:
    def __init__(self, frames):
        self.source_name = "unity_mock_test"
        self.waypoints = tuple(frames)
        self.queued_commands = ()
        self.original_duration_s = float(frames[-1]["timeStamp"] - frames[0]["timeStamp"])
        self.retimed_duration_s = self.original_duration_s
        self.total_duration_s = self.original_duration_s
        self.time_scale = 1.0
        self.original_timing_feasible = True
        self.lookahead_s = 0.1


class _FakeRecorder:
    """Records every interaction so the test can assert no real-robot motion."""

    def __init__(self, traj_dir: str):
        self._traj_dir = traj_dir
        self.loaded_frames = []
        self.loaded_name = ""
        self.is_playing = False
        self.is_recording = False
        # Observability for the test:
        self.start_preview_calls = 0
        self.compile_calls = 0
        self.export_calls = 0
        self.sent_motion_strings = []  # would-be JointMovJ payloads

    def load_frames(self, frames, name="inline"):
        if not frames:
            return False
        self.loaded_frames = list(frames)
        self.loaded_name = name
        return True

    def compile_loaded_plan(self):
        self.compile_calls += 1
        if not self.loaded_frames:
            raise RuntimeError("no frames loaded")
        return _CompiledPlanStub(self.loaded_frames)

    def export_loaded_plan(self, path):
        self.export_calls += 1
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text("{}", encoding="utf-8")
        return path

    def start_preview(self):
        # If this is ever called from a preview_sim path the test will FAIL
        # because A2 forbids real-robot motion for that action.
        self.start_preview_calls += 1
        # Simulate "would have sent JointMovJ ..." so we can assert nothing
        # leaks for sim:
        self.sent_motion_strings.append("JointMovJ(...stub...)")

    def stop_all(self, go_home=False):
        pass


# ── Helpers ─────────────────────────────────────────────────────────────────
def _build_unity_request(action: str, frames: list, *, file_name: str,
                        job_id: str, target: str = "mg400",
                        options: dict | None = None) -> str:
    """Reproduce the JSON envelope built by Unity TeachJobPublisher.cs."""
    payload = {
        "job_id": job_id,
        "action": action,
        "target": target,
        "submitted_at_unity_sec": 0.0,
        "options": options or {},
        "trajectory": {"filename": file_name, "frames": frames},
    }
    return json.dumps(payload, sort_keys=True)


class _CapturingHandler:
    """Bundles a TeachJobHandler with status / artifact buffers for assertions."""

    def __init__(self, *, allow_real_execute: bool, traj_dir: Path):
        self.statuses = []
        self.artifacts = []
        self.recorder = _FakeRecorder(str(traj_dir))
        self.handler = TeachJobHandler(
            recorder=self.recorder,
            publish_status_fn=lambda payload: self.statuses.append(json.loads(payload)),
            publish_artifact_fn=lambda payload: self.artifacts.append(json.loads(payload)),
            logger=_Logger(),
            allow_real_execute_fn=lambda: allow_real_execute,
            time_fn=lambda: 1700000000.0,
        )

    def stages(self):
        return [s["stage"] for s in self.statuses]

    def terminal(self):
        return self.statuses[-1]


class TeachJobBridgeUnityIntegrationTest(unittest.TestCase):
    """Software-only end-to-end: real Unity JSON → TeachJobHandler → status."""

    @classmethod
    def setUpClass(cls):
        sample = _resolve_sample_path()
        if not sample.is_file():
            raise unittest.SkipTest(
                f"Unity sample JSON missing at {sample}; set "
                "TEACH_REPEAT_SAMPLE_JSON to override."
            )
        cls.sample_path = sample
        with sample.open("r", encoding="utf-8") as f:
            data = json.load(f)
        cls.frames = data["frames"]
        cls.file_name = sample.name
        if len(cls.frames) < 2:
            raise unittest.SkipTest(f"Sample trajectory too short: {sample}")

    # -- A1 ----------------------------------------------------------------
    def test_compile_action_produces_received_then_compiled_with_artifact(self):
        cap = _CapturingHandler(allow_real_execute=True, traj_dir=self._tmp())
        payload = _build_unity_request(
            ACTION_COMPILE, self.frames, file_name=self.file_name,
            job_id="phase2-A1",
        )

        cap.handler.handle(payload)

        self.assertEqual(cap.stages(), [STAGE_RECEIVED, STAGE_COMPILED])
        terminal = cap.terminal()
        self.assertEqual(terminal["job_id"], "phase2-A1")
        self.assertIsNone(terminal["error_code"])
        self.assertGreaterEqual(
            terminal["metadata"]["waypoint_count"], len(self.frames)
        )

        self.assertEqual(len(cap.artifacts), 1)
        artifact = cap.artifacts[0]
        self.assertEqual(artifact["job_id"], "phase2-A1")
        self.assertEqual(
            artifact["artifact"]["artifact_kind"],
            "mg400_compiled_playback_plan",
        )

    # -- A2 ----------------------------------------------------------------
    def test_preview_sim_emits_preview_ready_without_engaging_real_robot(self):
        cap = _CapturingHandler(allow_real_execute=True, traj_dir=self._tmp())
        payload = _build_unity_request(
            ACTION_PREVIEW_SIM, self.frames, file_name=self.file_name,
            job_id="phase2-A2",
        )

        cap.handler.handle(payload)

        self.assertEqual(cap.stages(), [STAGE_RECEIVED, STAGE_PREVIEW_READY])
        terminal = cap.terminal()
        self.assertTrue(terminal["metadata"]["sim"])
        self.assertFalse(terminal["metadata"]["real_robot_moved"])

        # Defining safety assertion for A2: no motion-side calls reached the
        # recorder.  start_preview() would be the path that builds JointMovJ
        # commands and dispatches them to port 30003 in production.
        self.assertEqual(cap.recorder.start_preview_calls, 0)
        self.assertEqual(cap.recorder.sent_motion_strings, [])

        # Artifact emitted with the preview_sim flag so downstream tooling can
        # tell it apart from a regular compile artifact.
        self.assertEqual(len(cap.artifacts), 1)
        self.assertTrue(cap.artifacts[0]["preview_sim"])

    # -- A3 ----------------------------------------------------------------
    def test_execute_action_engages_playback_when_robot_allowed(self):
        cap = _CapturingHandler(allow_real_execute=True, traj_dir=self._tmp())
        payload = _build_unity_request(
            ACTION_EXECUTE, self.frames, file_name=self.file_name,
            job_id="phase2-A3",
        )

        cap.handler.handle(payload)

        self.assertEqual(cap.stages(), [STAGE_RECEIVED, STAGE_EXECUTING])
        self.assertEqual(cap.recorder.start_preview_calls, 1)
        self.assertTrue(cap.terminal()["metadata"]["real_robot_moved"])
        self.assertFalse(cap.terminal()["metadata"]["sim"])

    # -- A4 ----------------------------------------------------------------
    def test_execute_action_blocked_when_robot_disconnected(self):
        cap = _CapturingHandler(allow_real_execute=False, traj_dir=self._tmp())
        payload = _build_unity_request(
            ACTION_EXECUTE, self.frames, file_name=self.file_name,
            job_id="phase2-A4",
        )

        cap.handler.handle(payload)

        terminal = cap.terminal()
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_EXECUTE_FORBIDDEN)
        self.assertEqual(cap.recorder.start_preview_calls, 0)

    # -- helpers -----------------------------------------------------------
    def _tmp(self):
        # tempfile per test for isolation; cleaned up by addCleanup.
        import tempfile
        td = tempfile.TemporaryDirectory()
        self.addCleanup(td.cleanup)
        return Path(td.name)


if __name__ == "__main__":
    unittest.main()
