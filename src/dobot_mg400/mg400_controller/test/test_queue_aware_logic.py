import math
import struct
import sys
import types
import unittest

import numpy as np


sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")


class FakeJointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.name = []
        self.position = []


sensor_msgs_msg.JointState = FakeJointState
sensor_msgs.msg = sensor_msgs_msg
sys.modules.setdefault("sensor_msgs", sensor_msgs)
sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

from mg400_controller.common.core.feedback_handler import FeedbackHandler
from mg400_controller.common.logic.teleop_controller import TeleopController


class DummyLogger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class DummyValidator:
    def validate_and_clamp(self, q_target):
        return np.array(q_target), False


class DummyPlanner:
    def format_command(self, q_safe, speed_percent):
        return f"CMD:{speed_percent}"


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class DummyClock:
    class _Now:
        @staticmethod
        def to_msg():
            return None

    def now(self):
        return self._Now()


class DummyConnection:
    fb_sock = None


class QueueAwareLogicTest(unittest.TestCase):
    def test_initial_send_does_not_mutate_state_until_send_success(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        latest_target = np.array([0.05, 0.0, 0.0, 0.0])

        should_send, reason = controller.should_send_command(
            latest_target,
            np.zeros(4),
            now=10.0,
            queue_backlog_rad=1.0,
            run_queued_cmd=1,
        )

        self.assertTrue(should_send)
        self.assertEqual(reason, "Init")
        self.assertIsNone(controller.last_sent_target)

        controller.mark_command_sent(latest_target, 10.0)
        self.assertTrue(np.allclose(controller.last_sent_target, latest_target))
        self.assertEqual(controller.last_sent_time, 10.0)

    def test_reset_reference_anchors_controller_after_external_motion(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.mark_command_sent(np.array([1.0, 0.0, 0.0, 0.0]), 5.0)
        controller.robot_velocity = np.ones(4)
        controller.stuck_start_time = 4.0

        q_current = np.array([0.2, 0.1, 0.0, 0.0])
        controller.reset_reference(q_current, now=10.0)

        self.assertTrue(np.allclose(controller.last_sent_target, q_current))
        self.assertTrue(np.allclose(controller.last_robot_q, q_current))
        self.assertTrue(np.allclose(controller.robot_velocity, np.zeros(4)))
        self.assertEqual(controller.last_sent_time, 10.0)
        self.assertEqual(controller.stuck_start_time, 0.0)

        should_send, reason = controller.should_send_command(
            np.array([0.25, 0.1, 0.0, 0.0]),
            q_current,
            now=10.1,
        )
        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("DynProx_"))

    def test_dynamic_proximity_sends_latest_target_when_robot_reaches_last_target(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.last_sent_target = np.zeros(4)
        controller.robot_velocity = np.zeros(4)

        q_current = np.array([0.001, 0.0, 0.0, 0.0])
        latest_target = np.array([0.05, 0.0, 0.0, 0.0])

        should_send, reason = controller.should_send_command(
            latest_target,
            q_current,
            now=10.0,
            queue_backlog_rad=0.02,
            run_queued_cmd=1,
        )
        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("DynProx_"))

    def test_dynamic_proximity_defers_fast_sweeping_target(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.last_sent_target = np.zeros(4)
        controller.robot_velocity = np.zeros(4)

        should_send, reason = controller.should_send_command(
            np.array([0.08, 0.0, 0.0, 0.0]),
            np.array([0.001, 0.0, 0.0, 0.0]),
            now=10.0,
            target_velocity=np.array([2.0, 0.0, 0.0, 0.0]),
        )

        self.assertFalse(should_send)
        self.assertTrue(reason.startswith("TargetMovingFast_"))

        should_send, reason = controller.should_send_command(
            np.array([0.08, 0.0, 0.0, 0.0]),
            np.array([0.001, 0.0, 0.0, 0.0]),
            now=10.1,
            target_velocity=np.array([0.1, 0.0, 0.0, 0.0]),
        )

        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("DynProx_"))

    def test_dynamic_proximity_does_not_send_while_robot_is_far_from_last_target(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.last_sent_target = np.zeros(4)
        controller.robot_velocity = np.zeros(4)

        should_send, reason = controller.should_send_command(
            np.array([0.08, 0.0, 0.0, 0.0]),
            np.array([0.02, 0.0, 0.0, 0.0]),
            now=10.0,
            queue_backlog_rad=0.03,
            run_queued_cmd=1,
        )

        self.assertFalse(should_send)
        self.assertEqual(reason, "Wait")

    def test_queue_feedback_does_not_force_extra_live_sends(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.last_sent_target = np.zeros(4)
        controller.robot_velocity = np.zeros(4)

        should_send, reason = controller.should_send_command(
            np.array([0.08, 0.0, 0.0, 0.0]),
            np.array([0.03, 0.0, 0.0, 0.0]),
            now=10.0,
            queue_backlog_rad=0.0,
            run_queued_cmd=0,
        )
        self.assertFalse(should_send)
        self.assertEqual(reason, "Wait")

        should_send, reason = controller.should_send_command(
            np.array([0.08, 0.0, 0.0, 0.0]),
            np.array([0.03, 0.0, 0.0, 0.0]),
            now=10.1,
            queue_backlog_rad=1.0,
            run_queued_cmd=1,
        )
        self.assertFalse(should_send)
        self.assertEqual(reason, "Wait")

    def test_stuck_recovery_can_retrigger_when_robot_stops_far_from_last_target(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.last_sent_target = np.array([0.2, 0.0, 0.0, 0.0])
        controller.robot_velocity = np.zeros(4)

        q_current = np.zeros(4)
        latest_target = np.array([0.25, 0.0, 0.0, 0.0])

        should_send, reason = controller.should_send_command(
            latest_target,
            q_current,
            now=10.0,
            queue_backlog_rad=0.2,
            run_queued_cmd=1,
        )
        self.assertFalse(should_send)
        self.assertEqual(reason, "Wait")

        should_send, reason = controller.should_send_command(
            latest_target,
            q_current,
            now=10.35,
            queue_backlog_rad=0.2,
            run_queued_cmd=1,
        )
        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("Stuck_"))

    def test_feedback_handler_parses_queue_target_and_running_state(self):
        handler = FeedbackHandler(
            DummyConnection(),
            DummyPublisher(),
            DummyClock(),
            DummyLogger(),
            stop_event=types.SimpleNamespace(is_set=lambda: True),
        )

        packet = bytearray(1440)
        struct.pack_into("<Q", packet, 48, 0x0123456789ABCDEF)
        struct.pack_into("<Q", packet, 24, 7)
        struct.pack_into("<d", packet, 64, 100.0)
        struct.pack_into("<6d", packet, 192, 1.5, 2.0, 3.0, 4.0, 0.0, 0.0)
        struct.pack_into("<6d", packet, 432, 1.0, 2.0, 3.0, 4.0, 0.0, 0.0)
        struct.pack_into("<Q", packet, 1112, 42)
        packet[1014] = 1

        handler._process_packet(bytes(packet))

        self.assertEqual(handler.get_run_queued_cmd(), 1)
        self.assertEqual(handler.get_command_id(), 42)
        self.assertTrue(np.allclose(
            handler.get_current_position(),
            np.radians([1.0, 2.0, 3.0, 4.0]),
        ))
        self.assertTrue(np.allclose(
            handler.get_target_position(),
            np.radians([1.5, 2.0, 3.0, 4.0]),
        ))
        self.assertTrue(math.isclose(handler.get_queue_backlog(), math.radians(0.5), rel_tol=1e-6))

    def test_should_send_command_uses_monotonic_loop_time_without_second_velocity_update(self):
        controller = TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())
        controller.last_sent_target = np.zeros(4)

        controller.update_robot_state(np.zeros(4), 10.0)
        q_current = np.array([0.01, 0.0, 0.0, 0.0])
        controller.update_robot_state(q_current, 10.1)

        prev_velocity = controller.robot_velocity.copy()
        prev_last_robot_time = controller.last_robot_time
        latest_target = np.array([0.05, 0.0, 0.0, 0.0])

        should_send, reason = controller.should_send_command(
            latest_target,
            q_current,
            now=10.1,
            queue_backlog_rad=0.0,
            run_queued_cmd=0,
        )

        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("DynProx_"))
        self.assertEqual(controller.last_robot_time, prev_last_robot_time)
        self.assertTrue(np.allclose(controller.robot_velocity, prev_velocity))


if __name__ == "__main__":
    unittest.main()
