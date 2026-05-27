import json
import tempfile
import unittest
from pathlib import Path


from mg400_controller.common.trajectory.teach_job_handler import (  # noqa: E402
    ACTION_COMPILE,
    ACTION_EXECUTE,
    ACTION_EXPORT,
    ACTION_STOP,
    ACTION_TUNE,
    ERR_ALREADY_PLAYING,
    ERR_BAD_PAYLOAD,
    ERR_EMPTY_TRAJECTORY,
    ERR_EXECUTE_FORBIDDEN,
    ERR_PLAYBACK_FAILED,
    ERR_PLAYBACK_TIMEOUT,
    ERR_SCENE_SAFETY,
    ERR_TARGET_MISMATCH,
    ERR_UNKNOWN_ACTION,
    JobRequest,
    JobStatus,
    STAGE_COMPILED,
    STAGE_DONE,
    STAGE_EXECUTING,
    STAGE_FAILED,
    STAGE_RECEIVED,
    STAGE_STOPPED,
    STAGE_TUNED,
    TeachJobHandler,
    parse_job_request,
)


class FakeLogger:
    def __init__(self):
        self.info_msgs = []
        self.warn_msgs = []
        self.error_msgs = []

    def info(self, msg):
        self.info_msgs.append(msg)

    def warn(self, msg, *args, **kwargs):
        self.warn_msgs.append(msg)

    def error(self, msg, *args, **kwargs):
        self.error_msgs.append(msg)


class FakeRecorder:
    """Stand-in for ``TrajectoryRecorder`` covering only the surface the
    handler touches.  Avoids spawning playback threads in unit tests.
    """

    def __init__(self, traj_dir: str):
        self._traj_dir = traj_dir
        self.loaded_frames = []
        self.loaded_name = ""
        self.is_recording = False
        self.is_playing = False
        self._compile_plan = None
        self._compile_should_raise = False
        self._export_should_raise = False
        self.start_preview_called = 0
        self.stop_all_called_with = None
        self.playback_event_callbacks = []
        self._playback_tuning = {}

    # -- stub plan ------------------------------------------------------
    def set_compile_plan(self, plan):
        self._compile_plan = plan

    def set_compile_raises(self, value=True):
        self._compile_should_raise = value

    def set_export_raises(self, value=True):
        self._export_should_raise = value

    # -- TrajectoryRecorder surface ------------------------------------
    def load_frames(self, frames, name="inline", events=None):
        if not frames:
            return False
        self.loaded_frames = list(frames)
        self.loaded_events = list(events or [])
        self.loaded_name = name
        return True

    def compile_loaded_plan(self):
        if self._compile_should_raise:
            raise RuntimeError("simulated compile failure")
        if self._compile_plan is None:
            raise RuntimeError("no compile plan stubbed")
        return self._compile_plan

    def export_loaded_plan(self, path):
        if self._export_should_raise:
            raise RuntimeError("simulated export failure")
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text("{}", encoding="utf-8")
        return path

    def start_preview(self):
        self.start_preview_called += 1

    def stop_all(self, go_home=False):
        self.stop_all_called_with = go_home

    def add_playback_event_callback(self, callback):
        self.playback_event_callbacks.append(callback)

    def set_playback_tuning(self, options):
        self._playback_tuning = dict(options or {})
        return dict(self._playback_tuning)

    def update_playback_tuning(self, options):
        self._playback_tuning.update(dict(options or {}))
        return dict(self._playback_tuning)

    def playback_tuning(self):
        return dict(self._playback_tuning)

    def emit_playback_complete(self, **payload):
        for callback in list(self.playback_event_callbacks):
            callback("playback_complete", payload)


class FakeCompiledPlan:
    """Replicates CompiledPlaybackPlan attributes used by the handler."""

    def __init__(self):
        self.source_name = "fake"
        self.waypoints = (
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 1.0, "j1": 10.0, "j2": -5.0, "j3": 0.0, "j4": 0.0},
        )
        self.queued_commands = ()
        self.event_commands = ()
        self.original_duration_s = 1.0
        self.retimed_duration_s = 1.0
        self.total_duration_s = 1.0
        self.time_scale = 1.0
        self.original_timing_feasible = True
        self.lookahead_s = 0.1


class FakeSceneResult:
    def __init__(self):
        self.status = "INSIDE"
        self.detail = "camera_post"
        self.distance_mm = 0.0
        self.point_xyzr = (5.0, 5.0, 5.0, 0.0)
        self.blocked = True


class FakeSceneSafetyGuard:
    enabled = True
    loaded = True

    def __init__(self, violations=True):
        self.violations = violations

    def check_plan(self, plan):
        return [FakeSceneResult()] if self.violations else []


def _frames_payload():
    return {
        "filename": "demo.json",
        "frames": [
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 1.0, "j1": 5.0, "j2": -3.0, "j3": 0.0, "j4": 0.0},
        ],
    }


def _frames_with_events_payload():
    payload = _frames_payload()
    payload["events"] = [
        {"timeStamp": 0.5, "kind": "digital_output", "channel": "vacuum", "value": True},
        {"timeStamp": 0.9, "kind": "digital_output", "channel": "green_light", "value": True},
    ]
    return payload


class TeachJobRequestParseTest(unittest.TestCase):
    def test_parses_minimal_valid_request(self):
        payload = json.dumps({
            "job_id": "abc",
            "action": "compile",
            "trajectory": _frames_payload(),
        })
        req = parse_job_request(payload)
        self.assertEqual(req.job_id, "abc")
        self.assertEqual(req.action, "compile")
        self.assertEqual(req.filename(), "demo.json")
        self.assertEqual(len(req.frames()), 2)

    def test_parses_embedded_task_events(self):
        payload = json.dumps({
            "job_id": "abc",
            "action": "compile",
            "trajectory": _frames_with_events_payload(),
        })
        req = parse_job_request(payload)
        self.assertEqual(len(req.events()), 2)
        self.assertEqual(req.events()[0]["channel"], "vacuum")

    def test_assigns_uuid_when_job_id_missing(self):
        payload = json.dumps({"action": "stop"})
        req = parse_job_request(payload)
        self.assertTrue(req.job_id)
        self.assertEqual(req.action, "stop")

    def test_rejects_empty_payload(self):
        with self.assertRaises(ValueError):
            parse_job_request("")

    def test_rejects_non_json(self):
        with self.assertRaises(ValueError):
            parse_job_request("not-json")

    def test_rejects_non_object(self):
        with self.assertRaises(ValueError):
            parse_job_request("[1,2]")

    def test_rejects_missing_action(self):
        with self.assertRaises(ValueError):
            parse_job_request(json.dumps({"job_id": "x"}))

    def test_rejects_non_string_target(self):
        with self.assertRaises(ValueError):
            parse_job_request(json.dumps({"action": "compile", "target": 5}))

    def test_rejects_non_object_trajectory(self):
        with self.assertRaises(ValueError):
            parse_job_request(json.dumps({"action": "compile", "trajectory": []}))

    def test_rejects_non_object_options(self):
        with self.assertRaises(ValueError):
            parse_job_request(json.dumps({"action": "compile", "options": []}))


class TeachJobHandlerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.recorder = FakeRecorder(self.tmp.name)
        self.recorder.set_compile_plan(FakeCompiledPlan())
        self.status_msgs = []
        self.artifact_msgs = []
        self.logger = FakeLogger()
        self.handler = TeachJobHandler(
            recorder=self.recorder,
            publish_status_fn=self.status_msgs.append,
            publish_artifact_fn=self.artifact_msgs.append,
            logger=self.logger,
            allow_real_execute_fn=lambda: True,
            time_fn=lambda: 1700000000.0,
        )

    def _decoded_status(self):
        return [json.loads(payload) for payload in self.status_msgs]

    def _decoded_artifact(self):
        return [json.loads(payload) for payload in self.artifact_msgs]

    # -- happy paths ----------------------------------------------------
    def test_compile_emits_compiled_status_and_artifact(self):
        payload = json.dumps({
            "job_id": "j-compile",
            "action": ACTION_COMPILE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)

        statuses = self._decoded_status()
        stages = [s["stage"] for s in statuses]
        self.assertEqual(stages, [STAGE_RECEIVED, STAGE_COMPILED])
        self.assertEqual(statuses[-1]["job_id"], "j-compile")
        self.assertIsNone(statuses[-1]["error_code"])

        artifacts = self._decoded_artifact()
        self.assertEqual(len(artifacts), 1)
        self.assertEqual(artifacts[0]["job_id"], "j-compile")
        self.assertEqual(
            artifacts[0]["artifact"]["artifact_kind"],
            "mg400_compiled_playback_plan",
        )

    def test_compile_passes_embedded_events_to_recorder(self):
        payload = json.dumps({
            "job_id": "j-events",
            "action": ACTION_COMPILE,
            "trajectory": _frames_with_events_payload(),
        })
        self.handler.handle(payload)

        self.assertEqual(len(self.recorder.loaded_events), 2)
        self.assertEqual(self.recorder.loaded_events[0]["channel"], "vacuum")
        statuses = self._decoded_status()
        self.assertEqual(statuses[-1]["stage"], STAGE_COMPILED)

    def test_execute_starts_playback_when_allowed(self):
        payload = json.dumps({
            "job_id": "j-exec",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
            "options": {"speed_j": 35, "acc_j": 70, "cp": 25},
        })
        self.handler.handle(payload)

        self.assertEqual(self.recorder.start_preview_called, 1)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_EXECUTING)
        self.assertFalse(terminal["metadata"]["sim"])
        self.assertEqual(terminal["metadata"]["playback_tuning"]["speed_j"], 35)
        self.assertEqual(self.recorder.playback_tuning()["cp"], 25)

    def test_tune_updates_playback_tuning_without_trajectory(self):
        payload = json.dumps({
            "job_id": "j-tune",
            "action": ACTION_TUNE,
            "options": {"speed_j": 55, "acc_j": 65, "speed_l": 40, "cp": 10},
        })
        self.handler.handle(payload)

        statuses = self._decoded_status()
        self.assertEqual([s["stage"] for s in statuses], [STAGE_RECEIVED, STAGE_TUNED])
        terminal = statuses[-1]
        self.assertEqual(terminal["metadata"]["playback_tuning"]["speed_j"], 55)
        self.assertEqual(self.recorder.playback_tuning()["acc_j"], 65)

    def test_execute_emits_done_when_recorder_reports_success(self):
        payload = json.dumps({
            "job_id": "j-exec-done",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        self.recorder.emit_playback_complete(
            stopped=False,
            timed_out=False,
            success=True,
            final_max_error_deg=0.0,
        )

        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["job_id"], "j-exec-done")
        self.assertEqual(terminal["stage"], STAGE_DONE)
        self.assertIsNone(terminal["error_code"])
        self.assertEqual(terminal["metadata"]["final_max_error_deg"], 0.0)

    def test_execute_emits_failed_when_recorder_times_out(self):
        payload = json.dumps({
            "job_id": "j-exec-timeout",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        self.recorder.emit_playback_complete(
            stopped=False,
            timed_out=True,
            success=False,
            final_max_error_deg=8.5,
        )

        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_PLAYBACK_TIMEOUT)
        self.assertEqual(terminal["metadata"]["final_max_error_deg"], 8.5)

    def test_execute_emits_failed_when_final_settle_not_confirmed(self):
        payload = json.dumps({
            "job_id": "j-exec-unsettled",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        self.recorder.emit_playback_complete(
            stopped=False,
            timed_out=False,
            success=False,
            final_max_error_deg=1.25,
        )

        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_PLAYBACK_FAILED)

    def test_execute_blocked_when_real_robot_disallowed(self):
        self.handler = TeachJobHandler(
            recorder=self.recorder,
            publish_status_fn=self.status_msgs.append,
            publish_artifact_fn=self.artifact_msgs.append,
            logger=self.logger,
            allow_real_execute_fn=lambda: False,
        )
        payload = json.dumps({
            "job_id": "j-exec-blocked",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        self.assertEqual(self.recorder.start_preview_called, 0)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_EXECUTE_FORBIDDEN)

    def test_execute_blocked_when_scene_safety_finds_violation(self):
        self.handler = TeachJobHandler(
            recorder=self.recorder,
            publish_status_fn=self.status_msgs.append,
            publish_artifact_fn=self.artifact_msgs.append,
            logger=self.logger,
            allow_real_execute_fn=lambda: True,
            scene_safety_guard=FakeSceneSafetyGuard(violations=True),
        )
        payload = json.dumps({
            "job_id": "j-scene-blocked",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)

        self.assertEqual(self.recorder.start_preview_called, 0)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_SCENE_SAFETY)
        self.assertTrue(terminal["metadata"]["scene_safety_enabled"])
        self.assertEqual(
            terminal["metadata"]["scene_safety_violations"][0]["detail"],
            "camera_post",
        )

    def test_compile_reports_scene_safety_violations_without_moving(self):
        self.handler = TeachJobHandler(
            recorder=self.recorder,
            publish_status_fn=self.status_msgs.append,
            publish_artifact_fn=self.artifact_msgs.append,
            logger=self.logger,
            scene_safety_guard=FakeSceneSafetyGuard(violations=True),
        )
        payload = json.dumps({
            "job_id": "j-scene-compile",
            "action": ACTION_COMPILE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)

        self.assertEqual(self.recorder.start_preview_called, 0)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_COMPILED)
        self.assertTrue(terminal["metadata"]["scene_safety_enabled"])
        self.assertTrue(terminal["metadata"]["scene_safety_blocked"])

    def test_export_writes_file_and_reports_path(self):
        export_path = str(Path(self.tmp.name) / "out" / "plan.json")
        payload = json.dumps({
            "job_id": "j-export",
            "action": ACTION_EXPORT,
            "trajectory": _frames_payload(),
            "options": {"export_path": export_path},
        })
        self.handler.handle(payload)
        self.assertTrue(Path(export_path).exists())
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_DONE)
        self.assertEqual(terminal["metadata"]["export_path"], export_path)

    def test_stop_invokes_recorder_stop(self):
        payload = json.dumps({
            "job_id": "j-stop",
            "action": ACTION_STOP,
            "options": {"go_home": True},
        })
        self.handler.handle(payload)
        self.assertEqual(self.recorder.stop_all_called_with, True)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_STOPPED)

    # -- failure paths --------------------------------------------------
    def test_unknown_action_emits_failed_status(self):
        payload = json.dumps({"job_id": "j-bad", "action": "bogus"})
        self.handler.handle(payload)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_UNKNOWN_ACTION)

    def test_target_mismatch_rejected(self):
        payload = json.dumps({
            "job_id": "j-bad",
            "action": ACTION_COMPILE,
            "target": "ur5e",
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_TARGET_MISMATCH)

    def test_empty_trajectory_rejected(self):
        payload = json.dumps({
            "job_id": "j-empty",
            "action": ACTION_COMPILE,
            "trajectory": {"frames": []},
        })
        self.handler.handle(payload)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_EMPTY_TRAJECTORY)

    def test_already_playing_rejects_new_play(self):
        self.recorder.is_playing = True
        payload = json.dumps({
            "job_id": "j-busy",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_ALREADY_PLAYING)

    def test_bad_payload_emits_bad_payload_error(self):
        self.handler.handle("not-json")
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_FAILED)
        self.assertEqual(terminal["error_code"], ERR_BAD_PAYLOAD)

    def test_compile_without_trajectory_uses_recorder_loaded_frames(self):
        # Pre-load via load_frames (simulates a prior Load step) then send a
        # compile request with no embedded trajectory.
        self.recorder.load_frames(_frames_payload()["frames"], name="prev")
        payload = json.dumps({"job_id": "j-reuse", "action": ACTION_COMPILE})
        self.handler.handle(payload)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_COMPILED)


class JobStatusJsonTest(unittest.TestCase):
    def test_to_json_clamps_progress(self):
        status = JobStatus(
            job_id="x",
            stage=STAGE_DONE,
            progress=2.5,
            ros_time_sec=1.0,
        )
        data = json.loads(status.to_json())
        self.assertEqual(data["progress"], 1.0)

    def test_to_json_round_trip_preserves_metadata(self):
        status = JobStatus(
            job_id="x",
            stage=STAGE_COMPILED,
            ros_time_sec=1.0,
            metadata={"k": 5},
        )
        data = json.loads(status.to_json())
        self.assertEqual(data["metadata"], {"k": 5})


if __name__ == "__main__":
    unittest.main()
