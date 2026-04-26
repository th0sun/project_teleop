import json
import tempfile
import unittest
from pathlib import Path


from mg400_controller.common.trajectory.teach_job_handler import (  # noqa: E402
    ACTION_COMPILE,
    ACTION_EXECUTE,
    ACTION_EXPORT,
    ACTION_PREVIEW_SIM,
    ACTION_RECORD_START,
    ACTION_RECORD_STOP,
    ACTION_STOP,
    ERR_ALREADY_PLAYING,
    ERR_BAD_PAYLOAD,
    ERR_EMPTY_TRAJECTORY,
    ERR_EXECUTE_FORBIDDEN,
    ERR_TARGET_MISMATCH,
    ERR_UNKNOWN_ACTION,
    JobRequest,
    JobStatus,
    STAGE_COMPILED,
    STAGE_DONE,
    STAGE_EXECUTING,
    STAGE_FAILED,
    STAGE_PREVIEW_READY,
    STAGE_PREVIEW_STARTED,
    STAGE_RECEIVED,
    STAGE_STOPPED,
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
        self.start_recording_called = 0
        self.stop_recording_called = 0

    # -- stub plan ------------------------------------------------------
    def set_compile_plan(self, plan):
        self._compile_plan = plan

    def set_compile_raises(self, value=True):
        self._compile_should_raise = value

    def set_export_raises(self, value=True):
        self._export_should_raise = value

    # -- TrajectoryRecorder surface ------------------------------------
    def load_frames(self, frames, name="inline"):
        if not frames:
            return False
        self.loaded_frames = list(frames)
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

    def start_recording(self):
        self.start_recording_called += 1
        self.is_recording = True

    def stop_recording(self):
        self.stop_recording_called += 1
        self.is_recording = False
        return list(self.loaded_frames)


class FakeCompiledPlan:
    """Replicates CompiledPlaybackPlan attributes used by the handler."""

    def __init__(self):
        self.source_name = "fake"
        self.waypoints = (
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 1.0, "j1": 10.0, "j2": -5.0, "j3": 0.0, "j4": 0.0},
        )
        self.queued_commands = ()
        self.original_duration_s = 1.0
        self.retimed_duration_s = 1.0
        self.total_duration_s = 1.0
        self.time_scale = 1.0
        self.original_timing_feasible = True
        self.lookahead_s = 0.1


def _frames_payload():
    return {
        "filename": "demo.json",
        "frames": [
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 1.0, "j1": 5.0, "j2": -3.0, "j3": 0.0, "j4": 0.0},
        ],
    }


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

    def test_preview_sim_is_compile_only_and_does_not_move_real_robot(self):
        """preview_sim must publish the compiled artifact + a preview_ready
        status without ever calling the recorder's start_preview().  Until a
        dedicated sim backend lands, an action whose name says "sim" must
        not drive the MG400.
        """
        payload = json.dumps({
            "job_id": "j-sim",
            "action": ACTION_PREVIEW_SIM,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)

        # The defining safety assertion: real robot was not engaged.
        self.assertEqual(self.recorder.start_preview_called, 0)

        statuses = self._decoded_status()
        terminal = statuses[-1]
        self.assertEqual(terminal["stage"], STAGE_PREVIEW_READY)
        self.assertTrue(terminal["metadata"]["sim"])
        self.assertFalse(terminal["metadata"]["real_robot_moved"])

        artifacts = self._decoded_artifact()
        self.assertEqual(len(artifacts), 1)
        self.assertEqual(artifacts[0]["job_id"], "j-sim")
        self.assertTrue(artifacts[0]["preview_sim"])
        self.assertEqual(
            artifacts[0]["artifact"]["artifact_kind"],
            "mg400_compiled_playback_plan",
        )

    def test_preview_sim_does_not_move_robot_even_when_real_execute_allowed(self):
        """Even if the operator has the robot connected, preview_sim must
        stay compile-only.  Guards against an accidental rewire that flips
        the sim path back into start_preview().
        """
        # allow_real_execute defaults to True in this test class, exercising
        # the code path most likely to regress.
        payload = json.dumps({
            "job_id": "j-sim-2",
            "action": ACTION_PREVIEW_SIM,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)
        self.assertEqual(self.recorder.start_preview_called, 0)
        self.assertEqual(self.recorder.stop_all_called_with, None)

    def test_execute_starts_playback_when_allowed(self):
        payload = json.dumps({
            "job_id": "j-exec",
            "action": ACTION_EXECUTE,
            "trajectory": _frames_payload(),
        })
        self.handler.handle(payload)

        self.assertEqual(self.recorder.start_preview_called, 1)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_EXECUTING)
        self.assertFalse(terminal["metadata"]["sim"])

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

    def test_record_start_and_stop_round_trip(self):
        self.recorder.loaded_frames = [
            {"timeStamp": 0.0, "j1": 0, "j2": 0, "j3": 0, "j4": 0}
        ]
        start = json.dumps({"job_id": "rec1", "action": ACTION_RECORD_START})
        stop = json.dumps({"job_id": "rec1-stop", "action": ACTION_RECORD_STOP})
        self.handler.handle(start)
        self.handler.handle(stop)
        self.assertEqual(self.recorder.start_recording_called, 1)
        self.assertEqual(self.recorder.stop_recording_called, 1)
        terminal = self._decoded_status()[-1]
        self.assertEqual(terminal["stage"], STAGE_DONE)
        self.assertEqual(terminal["metadata"]["frame_count"], 1)

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
