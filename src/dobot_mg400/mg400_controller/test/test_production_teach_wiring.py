"""Production teach-repeat wiring uses only /teach/job_request."""

import sys
import types
import unittest


_ORIGINAL_ROS_MODULES = {
    name: sys.modules.get(name)
    for name in ("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")
}

sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")


class _DummyMsg:
    pass


sensor_msgs_msg.JointState = _DummyMsg
std_msgs_msg.String = _DummyMsg
std_msgs_msg.Float64MultiArray = _DummyMsg
std_msgs_msg.Bool = _DummyMsg
std_msgs_msg.Int32MultiArray = _DummyMsg
std_msgs_msg.Int64 = _DummyMsg
std_msgs_msg.Int32 = _DummyMsg
sensor_msgs.msg = sensor_msgs_msg
std_msgs.msg = std_msgs_msg
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs_msg

from mg400_controller.common.config import motion_config  # noqa: E402
from mg400_controller.common.ros.teleop_interfaces import create_subscriptions  # noqa: E402

for _name, _module in _ORIGINAL_ROS_MODULES.items():
    if _module is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _module


class FakeNode:
    def __init__(self):
        self.subscription_calls = []

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = types.SimpleNamespace(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos=qos,
        )
        self.subscription_calls.append(sub)
        return sub


class ProductionTeachWiringTest(unittest.TestCase):
    def test_legacy_teach_topics_are_not_runtime_contract(self):
        for name in (
            "TEACH_STATUS_TOPIC",
            "TRAJECTORY_DATA_TOPIC",
            "UNITY_TRAJECTORY_TOPIC",
        ):
            self.assertFalse(hasattr(motion_config, name), name)

    def test_create_subscriptions_does_not_subscribe_legacy_teach_topics(self):
        node = FakeNode()
        create_subscriptions(
            node,
            unity_pong_callback=lambda msg: None,
            suction_callback=lambda msg: None,
            light_callback=lambda msg: None,
            scene_safety_callback=lambda msg: None,
            dashboard_cmd_callback=lambda msg: None,
            speed_factor_callback=lambda msg: None,
            teach_job_request_callback=lambda msg: None,
        )
        topics = {sub.topic for sub in node.subscription_calls}
        self.assertNotIn("/unity/teach_status", topics)
        self.assertNotIn("/unity/trajectory_data", topics)
        self.assertNotIn("/mg400/joint_trajectory_controller/command", topics)
        self.assertIn(motion_config.TEACH_JOB_REQUEST_TOPIC, topics)


if __name__ == "__main__":
    unittest.main()
