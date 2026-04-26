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
    TEACH_JOB_ARTIFACT_TOPIC,
    TEACH_JOB_REQUEST_TOPIC,
    TEACH_JOB_STATUS_TOPIC,
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
from mg400_controller.common.ros.topic_config import (  # noqa: E402
    TeleopTopicConfig,
    declare_topic_parameters,
)


class FakeNode:
    def __init__(self):
        self.publish_calls = []
        self.subscription_calls = []
        self.parameters = {}

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

    def declare_parameter(self, name, value):
        resolved = self.parameters.get(name, value)
        self.parameters[name] = resolved
        return types.SimpleNamespace(value=resolved)


class TeleopRosWiringTest(unittest.TestCase):
    def test_create_publishers_returns_named_publishers(self):
        node = FakeNode()

        publishers = create_publishers(node)

        self.assertEqual(publishers.heartbeat.topic, ROS_PING_TOPIC)
        self.assertEqual(publishers.safety.topic, SAFETY_TOPIC)
        self.assertEqual(publishers.tool_actual.topic, TOOL_ACTUAL_TOPIC)
        self.assertEqual(publishers.teach_job_status.topic, TEACH_JOB_STATUS_TOPIC)
        self.assertEqual(publishers.teach_job_artifact.topic, TEACH_JOB_ARTIFACT_TOPIC)
        self.assertEqual(len(node.publish_calls), 19)

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
            "teach_job_request_callback": lambda msg: None,
        }

        subscriptions = create_subscriptions(node, **callbacks)

        self.assertEqual(subscriptions.pong.topic, UNITY_PONG_TOPIC)
        self.assertEqual(subscriptions.dashboard_cmd.topic, DASHBOARD_CMD_TOPIC)
        self.assertEqual(subscriptions.traj_data.topic, TRAJECTORY_DATA_TOPIC)
        self.assertEqual(subscriptions.unity_trajectory.topic, UNITY_TRAJECTORY_TOPIC)
        self.assertEqual(subscriptions.teach_job_request.topic, TEACH_JOB_REQUEST_TOPIC)
        self.assertEqual(len(node.subscription_calls), 8)

    def test_attach_unity_subscription_uses_unity_topic(self):
        node = FakeNode()
        callback = lambda msg: None

        subscription = attach_unity_subscription(node, callback, qos_profile=object())

        self.assertEqual(subscription.topic, UNITY_TOPIC)
        self.assertIs(subscription.callback, callback)

    def test_topic_parameters_can_override_unity_contract_topics(self):
        node = FakeNode()
        node.parameters["topics.unity_joint_cmd"] = "/robot_a/unity/joint_cmd"
        node.parameters["topics.unity_trajectory"] = "/robot_a/program"
        node.parameters["topics.suction"] = "/robot_a/tool/suction"

        topics = declare_topic_parameters(node)

        self.assertEqual(topics.unity_joint_cmd, "/robot_a/unity/joint_cmd")
        self.assertEqual(topics.unity_trajectory, "/robot_a/program")
        self.assertEqual(topics.suction, "/robot_a/tool/suction")

    def test_custom_topics_are_used_by_subscriptions(self):
        node = FakeNode()
        callbacks = {
            "unity_pong_callback": lambda msg: None,
            "suction_callback": lambda msg: None,
            "light_callback": lambda msg: None,
            "dashboard_cmd_callback": lambda msg: None,
            "teach_status_callback": lambda msg: None,
            "traj_data_callback": lambda msg: None,
            "joint_trajectory_callback": lambda msg: None,
            "teach_job_request_callback": lambda msg: None,
        }
        topics = TeleopTopicConfig(
            unity_joint_cmd="/robot_a/unity/joint_cmd",
            unity_trajectory="/robot_a/program",
            suction="/robot_a/tool/suction",
        )

        subscriptions = create_subscriptions(node, **callbacks, topics=topics)
        unity_subscription = attach_unity_subscription(node, lambda msg: None, object(), topics=topics)

        self.assertEqual(subscriptions.suction.topic, "/robot_a/tool/suction")
        self.assertEqual(subscriptions.unity_trajectory.topic, "/robot_a/program")
        self.assertEqual(unity_subscription.topic, "/robot_a/unity/joint_cmd")


if __name__ == "__main__":
    unittest.main()
