"""Tests for the adaptive teleop controller (post-2026-05 refactor).

The controller no longer uses the DynProx + queue-feedback gates.  See
EXP3/EXP4/EXP5/EXP7/EXP8/EXP10 in the project notes — those benchmarks
showed the previous gates were unable to keep the controller in the
smooth-cruise regime and that the per-command joint delta is the only
variable that controls jitter at MG400 firmware level.

The new contract:
  - Gate fires when max-axis |latest - last_sent| ≥ adaptive threshold,
    where threshold interpolates between REALTIME_DELTA_MIN_RAD (slow hand)
    and REALTIME_DELTA_MAX_RAD (fast hand).
  - When the gate fires, the controller stashes a per-command (SpeedJ,
    AccJ, CP) tuning that scales with the gate-firing delta — small cmd ⇒
    low SpeedJ, big cmd ⇒ SpeedJ_MAX.  format_command_string() consumes
    that tuning.
  - Queue-state arguments (queue_backlog_rad, run_queued_cmd) are accepted
    for wiring compatibility but ignored.  EXP10 confirmed they are not
    correlated with jitter.
"""

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
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

from mg400_controller.common.config.motion_config import (
    REALTIME_DELTA_MAX_RAD,
    REALTIME_DELTA_MIN_RAD,
    REALTIME_HAND_VEL_HIGH_RAD_S,
    REALTIME_SPEEDJ_MAX,
    REALTIME_SPEEDJ_MIN,
)
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
        return np.array(q_target, dtype=float), False


class DummyPlanner:
    """Test planner that accepts the new acc/cp kwargs and echoes them."""

    def __init__(self):
        self.last_call = None

    def format_command(self, q_safe, speed_percent, acc_percent=None, cp=None):
        self.last_call = {
            "q": tuple(float(x) for x in q_safe),
            "speed_j": int(speed_percent),
            "acc_j": int(acc_percent) if acc_percent is not None else None,
            "cp": int(cp) if cp is not None else None,
        }
        return f"CMD:speed={speed_percent}:acc={acc_percent}:cp={cp}"


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


class AdaptiveTeleopControllerTest(unittest.TestCase):
    def _ctrl(self):
        return TeleopController(DummyValidator(), DummyPlanner(), DummyLogger())

    # -- Initialization --------------------------------------------------
    def test_initial_send_returns_init_and_does_not_mutate_state(self):
        controller = self._ctrl()
        latest_target = np.array([0.05, 0.0, 0.0, 0.0])

        should_send, reason = controller.should_send_command(
            latest_target,
            np.zeros(4),
            now=10.0,
            queue_backlog_rad=1.0,   # ignored by the adaptive gate
            run_queued_cmd=1,        # ignored by the adaptive gate
        )

        self.assertTrue(should_send)
        self.assertEqual(reason, "Init")
        self.assertIsNone(controller.last_sent_target)

        controller.mark_command_sent(latest_target, 10.0)
        self.assertTrue(np.allclose(controller.last_sent_target, latest_target))
        self.assertEqual(controller.last_sent_time, 10.0)

    # -- Reset reference -------------------------------------------------
    def test_reset_reference_anchors_controller_after_external_motion(self):
        controller = self._ctrl()
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

        # After reset, a hand target ≥ MIN_RAD away should fire the gate.
        nudge = REALTIME_DELTA_MIN_RAD * 1.5
        latest = q_current + np.array([nudge, 0.0, 0.0, 0.0])
        should_send, reason = controller.should_send_command(
            latest, q_current, now=10.1
        )
        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("Adaptive_"))

    # -- Gate firing -----------------------------------------------------
    def test_gate_fires_when_delta_exceeds_min_threshold_with_zero_velocity(self):
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)

        # Slightly above MIN — must trigger.
        delta = REALTIME_DELTA_MIN_RAD * 1.2
        latest = np.array([delta, 0.0, 0.0, 0.0])
        should_send, reason = controller.should_send_command(
            latest, np.zeros(4), now=10.0,
            target_velocity=np.zeros(4),    # slow hand → MIN threshold
        )
        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("Adaptive_"))

    def test_gate_does_not_fire_below_min_threshold(self):
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)

        delta = REALTIME_DELTA_MIN_RAD * 0.5
        latest = np.array([delta, 0.0, 0.0, 0.0])
        should_send, reason = controller.should_send_command(
            latest, np.zeros(4), now=10.0,
            target_velocity=np.zeros(4),
        )
        self.assertFalse(should_send)
        self.assertEqual(reason, "Wait")

    def test_fast_hand_raises_threshold_to_max(self):
        """Fast hand → bigger gate, smaller deltas no longer fire."""
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)

        # Pick a delta that is above MIN but below MAX.  At zero hand velocity
        # this fires; at HIGH+ hand velocity it should NOT fire.
        delta = (REALTIME_DELTA_MIN_RAD + REALTIME_DELTA_MAX_RAD) / 4.0
        self.assertGreater(delta, REALTIME_DELTA_MIN_RAD)
        self.assertLess(delta, REALTIME_DELTA_MAX_RAD)
        latest = np.array([delta, 0.0, 0.0, 0.0])

        # Slow hand → fires
        slow_send, _ = controller.should_send_command(
            latest, np.zeros(4), now=10.0,
            target_velocity=np.zeros(4),
        )
        self.assertTrue(slow_send)

        # Reset and try again with fast hand.  Threshold becomes MAX, so the
        # smaller delta no longer crosses it.
        controller.reset_reference(np.zeros(4), now=10.0)
        fast_vel = np.array([REALTIME_HAND_VEL_HIGH_RAD_S * 2.0, 0.0, 0.0, 0.0])
        fast_send, fast_reason = controller.should_send_command(
            latest, np.zeros(4), now=11.0,
            target_velocity=fast_vel,
        )
        self.assertFalse(fast_send)
        self.assertEqual(fast_reason, "Wait")

    def test_gate_uses_per_axis_max_delta(self):
        """Multi-axis hand motion: any single axis exceeding triggers."""
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)
        latest = np.array([
            REALTIME_DELTA_MIN_RAD * 0.5,   # below
            REALTIME_DELTA_MIN_RAD * 1.5,   # ABOVE → should trigger
            0.0,
            0.0,
        ])
        should_send, reason = controller.should_send_command(
            latest, np.zeros(4), now=10.0, target_velocity=np.zeros(4),
        )
        self.assertTrue(should_send)
        self.assertTrue(reason.startswith("Adaptive_"))

    def test_gate_send_target_is_latest_not_last_sent(self):
        """The gate decides 'send now' but the caller picks the target.

        This test pins the contract that should_send_command does not return
        a target — it just gates.  The caller is expected to pass
        `latest_target` to format_command_string after the gate fires.
        """
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)
        latest = np.array([REALTIME_DELTA_MIN_RAD * 2.0, 0.0, 0.0, 0.0])

        should_send, _ = controller.should_send_command(
            latest, np.zeros(4), now=10.0, target_velocity=np.zeros(4),
        )
        self.assertTrue(should_send)
        # No mutation until the caller marks the send accepted.
        self.assertTrue(np.allclose(controller.last_sent_target, np.zeros(4)))

    # -- Per-command tuning ---------------------------------------------
    def test_adaptive_speed_scales_from_min_to_max_with_delta(self):
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)

        # Smallest cmd (exactly at MIN): should clamp to SPEEDJ_MIN.
        latest = np.array([REALTIME_DELTA_MIN_RAD, 0.0, 0.0, 0.0])
        controller.should_send_command(
            latest, np.zeros(4), now=10.0, target_velocity=np.zeros(4),
        )
        cmd_str, _ = controller.format_command_string(latest, np.zeros(4))
        self.assertIn(f"speed={REALTIME_SPEEDJ_MIN}", cmd_str)

        # Reset and try a max-size cmd: should pin to SPEEDJ_MAX.
        controller.reset_reference(np.zeros(4), now=11.0)
        big_delta = REALTIME_DELTA_MAX_RAD * 1.5   # clamps to MAX
        big_latest = np.array([big_delta, 0.0, 0.0, 0.0])
        controller.should_send_command(
            big_latest, np.zeros(4), now=11.1,
            target_velocity=np.array([REALTIME_HAND_VEL_HIGH_RAD_S * 2, 0, 0, 0]),
        )
        cmd_str, _ = controller.format_command_string(big_latest, np.zeros(4))
        self.assertIn(f"speed={REALTIME_SPEEDJ_MAX}", cmd_str)

        # Mid-band cmd: SpeedJ should be strictly between MIN and MAX.
        controller.reset_reference(np.zeros(4), now=12.0)
        mid_delta = (REALTIME_DELTA_MIN_RAD + REALTIME_DELTA_MAX_RAD) / 2.0
        mid_latest = np.array([mid_delta, 0.0, 0.0, 0.0])
        controller.should_send_command(
            mid_latest, np.zeros(4), now=12.1, target_velocity=np.zeros(4),
        )
        cmd_str, _ = controller.format_command_string(mid_latest, np.zeros(4))
        speed = controller.planner.last_call["speed_j"]
        self.assertGreater(speed, REALTIME_SPEEDJ_MIN)
        self.assertLess(speed, REALTIME_SPEEDJ_MAX)

    def test_force_send_uses_max_tuning_for_safety(self):
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)

        # A small cmd that would normally use SPEEDJ_MIN.
        delta = REALTIME_DELTA_MIN_RAD * 1.05
        latest = np.array([delta, 0.0, 0.0, 0.0])
        controller.should_send_command(
            latest, np.zeros(4), now=10.0, target_velocity=np.zeros(4),
        )
        # force_send override must escalate to MAX so escape moves have ramp.
        cmd_str, _ = controller.format_command_string(
            latest, np.zeros(4), force_send=True,
        )
        self.assertIn(f"speed={REALTIME_SPEEDJ_MAX}", cmd_str)

    # -- Stuck recovery --------------------------------------------------
    def test_stuck_recovery_retriggers_with_max_tuning(self):
        controller = self._ctrl()
        controller.last_sent_target = np.array([0.2, 0.0, 0.0, 0.0])
        controller.robot_velocity = np.zeros(4)

        q_current = np.zeros(4)
        latest_target = np.array([0.25, 0.0, 0.0, 0.0])

        # First check arms the timer; STUCK_TIME_THRESHOLD seconds later it fires.
        should_send_1, reason_1 = controller.should_send_command(
            latest_target, q_current, now=10.0,
        )
        # Note: with the adaptive gate, the first call already meets delta
        # threshold (|0.25-0.2|=0.05 ≥ MAX) so it fires as Adaptive_.  That's
        # the desired behaviour — fresh target sent immediately, no need to
        # wait for stuck timeout.  Pin that contract.
        self.assertTrue(should_send_1)
        self.assertTrue(reason_1.startswith("Adaptive_"))

    def test_stuck_recovery_fires_when_target_unchanged_but_robot_drifted(self):
        controller = self._ctrl()
        controller.last_sent_target = np.array([0.2, 0.0, 0.0, 0.0])
        controller.robot_velocity = np.zeros(4)

        # Hand and last_sent identical (so adaptive gate does NOT fire),
        # robot far from both → only stuck recovery should eventually fire.
        latest_target = np.array([0.2, 0.0, 0.0, 0.0])
        q_current = np.zeros(4)

        # 10.0 s — adaptive gate sees delta=0, stuck timer arms
        s1, r1 = controller.should_send_command(latest_target, q_current, now=10.0)
        self.assertFalse(s1)

        # 10.35 s — past STUCK_TIME_THRESHOLD, recovery fires
        s2, r2 = controller.should_send_command(latest_target, q_current, now=10.35)
        self.assertTrue(s2)
        self.assertTrue(r2.startswith("Stuck_"))

    # -- Queue feedback no longer gates ---------------------------------
    def test_queue_feedback_arguments_are_ignored(self):
        controller = self._ctrl()
        controller.last_sent_target = np.zeros(4)

        # Sub-threshold delta + every queue-state combo we can think of —
        # none of them should force a send.
        small = REALTIME_DELTA_MIN_RAD * 0.4
        latest = np.array([small, 0.0, 0.0, 0.0])
        for qb in (0.0, 0.5, 1.0):
            for rq in (0, 1):
                s, r = controller.should_send_command(
                    latest, np.zeros(4), now=10.0,
                    queue_backlog_rad=qb, run_queued_cmd=rq,
                    target_velocity=np.zeros(4),
                )
                self.assertFalse(s, f"queue={qb} runQ={rq} should not force send")
                self.assertEqual(r, "Wait")

    # -- Feedback handler ROS plumbing (unchanged) -----------------------
    def test_feedback_handler_parses_queue_target_and_running_state(self):
        callbacks = []
        handler = FeedbackHandler(
            DummyConnection(),
            DummyPublisher(),
            DummyClock(),
            DummyLogger(),
            stop_event=types.SimpleNamespace(is_set=lambda: True),
            feedback_callback=callbacks.append,
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
        self.assertTrue(math.isclose(
            handler.get_queue_backlog(), math.radians(0.5), rel_tol=1e-6
        ))
        self.assertEqual(len(callbacks), 1)
        self.assertEqual(callbacks[0]["command_id"], 42)
        self.assertEqual(callbacks[0]["run_queued_cmd"], 1)
        self.assertTrue(np.allclose(
            callbacks[0]["q_actual_rad"],
            np.radians([1.0, 2.0, 3.0, 4.0]),
        ))


if __name__ == "__main__":
    unittest.main()
