"""Tripwire: assert the three LEGACY Unity teach topics stay subscribed.

`test_teleop_ros_wiring.py` already asserts the topic strings on the
``subscriptions`` named tuple.  This file is the explicit *LEGACY-marked*
tripwire: it pins the three deprecated callbacks by name and verifies
their topic strings match the public contract.

Why: a future "Removal Pass v1" that drops a subscription will have to
update this test in the same commit, forcing the change through code
review.  See AGENTS.md §4.1 / §11.

Topics covered:
- ``/unity/teach_status``       (older Unity Record/Stop/Save/Load surface)
- ``/unity/trajectory_data``    (older Unity Save-button JSON dump)
- ``/mg400/joint_trajectory_controller/command``  (older JointTrajectory
                                                   auto-play path)
"""

import sys
import types
import unittest


sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
trajectory_msgs = types.ModuleType("trajectory_msgs")
trajectory_msgs_msg = types.ModuleType("trajectory_msgs.msg")


class _DummyMsg:
    pass


sensor_msgs_msg.JointState = _DummyMsg
std_msgs_msg.String = _DummyMsg
std_msgs_msg.Float64MultiArray = _DummyMsg
std_msgs_msg.Bool = _DummyMsg
std_msgs_msg.Int32MultiArray = _DummyMsg
std_msgs_msg.Int64 = _DummyMsg
std_msgs_msg.Int32 = _DummyMsg
trajectory_msgs_msg.JointTrajectory = _DummyMsg
sensor_msgs.msg = sensor_msgs_msg
std_msgs.msg = std_msgs_msg
trajectory_msgs.msg = trajectory_msgs_msg
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs_msg
sys.modules["trajectory_msgs"] = trajectory_msgs
sys.modules["trajectory_msgs.msg"] = trajectory_msgs_msg

from mg400_controller.common.config import motion_config  # noqa: E402
from mg400_controller.common.ros.teleop_interfaces import (  # noqa: E402
    create_subscriptions,
)


class FakeNode:
    def __init__(self):
        self.subscription_calls = []
        self.parameters = {}

    def create_publisher(self, msg_type, topic, depth):
        return types.SimpleNamespace(msg_type=msg_type, topic=topic, depth=depth)

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = types.SimpleNamespace(
            msg_type=msg_type, topic=topic, callback=callback, qos=qos,
        )
        self.subscription_calls.append(sub)
        return sub

    def declare_parameter(self, name, value):
        resolved = self.parameters.get(name, value)
        self.parameters[name] = resolved
        return types.SimpleNamespace(value=resolved)


class LegacyUnityTopicsTripwireTest(unittest.TestCase):
    """LEGACY (2026-05): see AGENTS.md §4.1."""

    @staticmethod
    def _make_subs(node):
        callbacks = {
            "unity_pong_callback": lambda msg: None,
            "suction_callback": lambda msg: None,
            "light_callback": lambda msg: None,
            "scene_safety_callback": lambda msg: None,
            "dashboard_cmd_callback": lambda msg: None,
            "teach_status_callback": lambda msg: None,
            "traj_data_callback": lambda msg: None,
            "joint_trajectory_callback": lambda msg: None,
            "speed_factor_callback": lambda msg: None,
            "teach_job_request_callback": lambda msg: None,
        }
        return create_subscriptions(node, **callbacks)

    def test_teach_status_topic_is_subscribed(self):
        node = FakeNode()
        subs = self._make_subs(node)
        self.assertEqual(subs.teach_status.topic, "/unity/teach_status")
        self.assertEqual(motion_config.TEACH_STATUS_TOPIC, "/unity/teach_status")

    def test_trajectory_data_topic_is_subscribed(self):
        node = FakeNode()
        subs = self._make_subs(node)
        self.assertEqual(subs.traj_data.topic, "/unity/trajectory_data")
        self.assertEqual(motion_config.TRAJECTORY_DATA_TOPIC, "/unity/trajectory_data")

    def test_joint_trajectory_command_topic_is_subscribed(self):
        node = FakeNode()
        subs = self._make_subs(node)
        self.assertEqual(
            subs.unity_trajectory.topic,
            "/mg400/joint_trajectory_controller/command",
        )
        self.assertEqual(
            motion_config.UNITY_TRAJECTORY_TOPIC,
            "/mg400/joint_trajectory_controller/command",
        )

    def test_legacy_callbacks_attached_to_legacy_topics(self):
        node = FakeNode()
        teach_status_cb = lambda msg: None
        traj_data_cb = lambda msg: None
        joint_traj_cb = lambda msg: None
        create_subscriptions(
            node,
            unity_pong_callback=lambda msg: None,
            suction_callback=lambda msg: None,
            light_callback=lambda msg: None,
            scene_safety_callback=lambda msg: None,
            dashboard_cmd_callback=lambda msg: None,
            teach_status_callback=teach_status_cb,
            traj_data_callback=traj_data_cb,
            joint_trajectory_callback=joint_traj_cb,
            speed_factor_callback=lambda msg: None,
            teach_job_request_callback=lambda msg: None,
        )
        by_topic = {sub.topic: sub for sub in node.subscription_calls}
        self.assertIs(by_topic["/unity/teach_status"].callback, teach_status_cb)
        self.assertIs(by_topic["/unity/trajectory_data"].callback, traj_data_cb)
        self.assertIs(
            by_topic["/mg400/joint_trajectory_controller/command"].callback,
            joint_traj_cb,
        )


if __name__ == "__main__":
    unittest.main()
