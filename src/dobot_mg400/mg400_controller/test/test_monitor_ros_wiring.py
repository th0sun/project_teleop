import sys
import types
import unittest


sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")


class _DummyMsg:
    pass


sensor_msgs_msg.JointState = _DummyMsg
std_msgs_msg.Bool = _DummyMsg
std_msgs_msg.String = _DummyMsg
std_msgs_msg.Float64MultiArray = _DummyMsg
std_msgs_msg.Int32 = _DummyMsg
std_msgs_msg.Int32MultiArray = _DummyMsg
std_msgs_msg.Int64 = _DummyMsg

sensor_msgs.msg = sensor_msgs_msg
std_msgs.msg = std_msgs_msg
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs_msg

from mg400_controller.common.config.motion_config import (  # noqa: E402
    DO_STATUS_TOPIC,
    LIGHT_TOPIC,
    RVIZ_TOPIC,
    SUCTION_TOPIC,
)
from mg400_controller.common.ros.monitor_interfaces import (  # noqa: E402
    MonitorTelemetryState,
    create_control_publishers,
    create_monitor_subscriptions,
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


class MonitorRosWiringTest(unittest.TestCase):
    def test_create_control_publishers_binds_control_topics(self):
        node = FakeNode()
        publishers = create_control_publishers(node)

        self.assertEqual(publishers.suction.topic, SUCTION_TOPIC)
        self.assertEqual(publishers.light.topic, LIGHT_TOPIC)
        self.assertEqual(len(node.publish_calls), 2)

    def test_create_monitor_subscriptions_binds_expected_topics(self):
        node = FakeNode()
        telemetry = MonitorTelemetryState()

        subscriptions = create_monitor_subscriptions(node, telemetry)

        self.assertEqual(subscriptions.actual.topic, RVIZ_TOPIC)
        self.assertEqual(subscriptions.do_status.topic, DO_STATUS_TOPIC)
        self.assertEqual(len(node.subscription_calls), 13)

    def test_telemetry_state_updates_and_consumes_sent_flag(self):
        telemetry = MonitorTelemetryState()

        actual_msg = types.SimpleNamespace(position=[0.0, 0.1, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.3])
        sent_msg = types.SimpleNamespace(position=[0.1, 0.2, 0.3, 0.4])
        do_msg = types.SimpleNamespace(data=17)

        telemetry.on_actual(actual_msg)
        telemetry.on_sent(sent_msg)
        telemetry.on_do_status(do_msg)

        self.assertEqual(len(telemetry.latest_actual_joints), 4)
        self.assertEqual(telemetry.latest_do_status, 17)
        self.assertTrue(telemetry.consume_sent_fresh(0))
        self.assertFalse(telemetry.consume_sent_fresh(0))


if __name__ == "__main__":
    unittest.main()
