import json
import tempfile
import unittest

from mg400_controller.common.teleop.teach_job_handler import (
    ACTION_COMPILE,
    ACTION_EXECUTE,
    ERR_EXECUTE_FORBIDDEN,
    STAGE_COMPILED,
    STAGE_FAILED,
    STAGE_RECEIVED,
    TeachJobHandler,
)
from mg400_controller.common.trajectory.trajectory_recorder import TrajectoryRecorder


UNITY_STYLE_FRAMES = [
    {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
    {"timeStamp": 0.5, "j1": 10.0, "j2": 10.0, "j3": 10.0, "j4": 0.0},
    {"timeStamp": 1.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
]

UNITY_STYLE_EVENTS = [
    {"timeStamp": 0.25, "kind": "digital_output", "channel": "vacuum", "value": True},
    {"timeStamp": 0.75, "kind": "digital_output", "channel": "vacuum", "value": False},
]


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


class TeachJobAcceptanceTest(unittest.TestCase):
    """Acceptance-level checks for the Unity -> ROS teach-job contract.

    These tests use the real ``TrajectoryRecorder`` compile path rather than
    the fake recorder used by the unit tests.  They intentionally stop before
    ROS graph or TCP hardware I/O, so they can run on macOS without ROS2 while
    still proving the job payload, compile artifact, and safety gates line up.
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sent_motion_commands = []
        self.sent_dashboard_commands = []
        self.status_msgs = []
        self.artifact_msgs = []
        self.logger = FakeLogger()
        self.recorder = TrajectoryRecorder(
            command_send_fn=self.sent_motion_commands.append,
            dashboard_send_fn=self.sent_dashboard_commands.append,
            logger=self.logger,
            traj_dir=self.tmp.name,
        )

    def _make_handler(self, *, allow_real_execute):
        return TeachJobHandler(
            recorder=self.recorder,
            publish_status_fn=self.status_msgs.append,
            publish_artifact_fn=self.artifact_msgs.append,
            logger=self.logger,
            allow_real_execute_fn=lambda: allow_real_execute,
            time_fn=lambda: 1700000000.0,
        )

    def _payload(self, action):
        return json.dumps({
            "job_id": f"accept-{action}",
            "action": action,
            "target": "mg400",
            "trajectory": {
                "filename": "unity_mock_test.json",
                "frames": UNITY_STYLE_FRAMES,
                "events": UNITY_STYLE_EVENTS,
            },
            "options": {},
            "submitted_at_unity_sec": 1.25,
        })

    def _statuses(self):
        return [json.loads(msg) for msg in self.status_msgs]

    def _artifacts(self):
        return [json.loads(msg) for msg in self.artifact_msgs]

    def test_compile_accepts_unity_payload_and_publishes_compiled_artifact(self):
        handler = self._make_handler(allow_real_execute=False)
        handler.handle(self._payload(ACTION_COMPILE))

        statuses = self._statuses()
        self.assertEqual([s["stage"] for s in statuses], [STAGE_RECEIVED, STAGE_COMPILED])
        self.assertEqual(statuses[-1]["metadata"]["queued_command_count"], 2)
        self.assertEqual(statuses[-1]["metadata"]["event_command_count"], 2)
        self.assertEqual(statuses[-1]["metadata"]["waypoint_count"], 3)

        artifacts = self._artifacts()
        self.assertEqual(len(artifacts), 1)
        self.assertEqual(artifacts[0]["job_id"], "accept-compile")
        self.assertEqual(
            artifacts[0]["artifact"]["artifact_kind"],
            "mg400_compiled_playback_plan",
        )
        self.assertEqual(artifacts[0]["artifact"]["queued_command_count"], 2)
        self.assertEqual(artifacts[0]["artifact"]["event_command_count"], 2)
        self.assertIn(
            "DOExecute(16,1)",
            artifacts[0]["artifact"]["event_commands"][0]["commands"],
        )

        self.assertEqual(self.sent_motion_commands, [])
        self.assertEqual(self.sent_dashboard_commands, [])

    def test_execute_is_blocked_when_robot_is_disconnected(self):
        handler = self._make_handler(allow_real_execute=False)
        handler.handle(self._payload(ACTION_EXECUTE))

        statuses = self._statuses()
        self.assertEqual([s["stage"] for s in statuses], [STAGE_RECEIVED, STAGE_FAILED])
        self.assertEqual(statuses[-1]["error_code"], ERR_EXECUTE_FORBIDDEN)
        self.assertEqual(self.sent_motion_commands, [])
        self.assertEqual(self.sent_dashboard_commands, [])


if __name__ == "__main__":
    unittest.main()
