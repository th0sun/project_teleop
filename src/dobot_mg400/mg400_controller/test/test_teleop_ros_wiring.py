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

from mg400_controller.common.config.motion_config import (  # noqa: E402
    DASHBOARD_CMD_TOPIC,
    ROS_PING_TOPIC,
    SAFETY_TOPIC,
    TOOL_ACTUAL_TOPIC,
    TRAJECTORY_DATA_TOPIC,
    UNITY_TRAJECTORY_TOPIC,
    UNITY_PONG_TOPIC,
    UNITY_TOPIC,
)
from mg400_controller.common.ros.teleop_interfaces import (  # noqa: E402
    attach_unity_subscription,
    create_publishers,
    create_subscriptions,
)


class FakeNode:
    def __init__(self):
        self.publish_calls = []
        self.subscription_calls = []

    def create_publisher(self, msg_type, topic, depth):
        publisher = types.SimpleNamespace(msg_type=msg_type, topic=topic, depth=depth)
        self.publish_calls.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos):
        subscription = types.SimpleNamespace(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscription_calls.append(subscription)
        return subscription


class TeleopRosWiringTest(unittest.TestCase):
    def test_create_publishers_returns_named_publishers(self):
        node = FakeNode()

        publishers = create_publishers(node)

        self.assertEqual(publishers.heartbeat.topic, ROS_PING_TOPIC)
        self.assertEqual(publishers.safety.topic, SAFETY_TOPIC)
        self.assertEqual(publishers.tool_actual.topic, TOOL_ACTUAL_TOPIC)
        self.assertEqual(len(node.publish_calls), 17)

    def test_create_subscriptions_binds_expected_topics(self):
        node = FakeNode()
        callbacks = {
            "unity_pong_callback": lambda msg: None,
            "suction_callback": lambda msg: None,
            "light_callback": lambda msg: None,
            "dashboard_cmd_callback": lambda msg: None,
            "teach_status_callback": lambda msg: None,
            "traj_data_callback": lambda msg: None,
            "joint_trajectory_callback": lambda msg: None,
        }

        subscriptions = create_subscriptions(node, **callbacks)

        self.assertEqual(subscriptions.pong.topic, UNITY_PONG_TOPIC)
        self.assertEqual(subscriptions.dashboard_cmd.topic, DASHBOARD_CMD_TOPIC)
        self.assertEqual(subscriptions.traj_data.topic, TRAJECTORY_DATA_TOPIC)
        self.assertEqual(subscriptions.unity_trajectory.topic, UNITY_TRAJECTORY_TOPIC)
        self.assertEqual(len(node.subscription_calls), 7)

    def test_attach_unity_subscription_uses_unity_topic(self):
        node = FakeNode()
        callback = lambda msg: None

        subscription = attach_unity_subscription(node, callback, qos_profile=object())

        self.assertEqual(subscription.topic, UNITY_TOPIC)
        self.assertIs(subscription.callback, callback)


if __name__ == "__main__":
    unittest.main()
