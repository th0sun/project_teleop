"""Characterisation tripwire: lock the adaptive gate's tuning constants.

The behaviour of ``TeleopController`` is already covered end-to-end by
``test_queue_aware_logic.py`` (13 tests).  This file is a *configuration*
tripwire: it asserts the exact numerical values of the constants the
gate reads from ``motion_config``.

Why: a future refactor that adjusts one of these constants silently
shifts realtime feel.  Forcing the change to also update this test
makes the tuning decision visible in the code review (and in the
adaptive gate's git blame for that line).

If a value here legitimately needs to change:
1. Run the EXP scripts on real hardware to confirm the new value keeps
   jitter low.
2. Update both the constant in ``motion_config.py`` AND the expected
   value here in the same commit.
3. Reference the experiment in the commit message so the next reader
   knows why.

LEGACY note: do NOT delete this file.  See AGENTS.md §10/§11 — the
tripwire stays as a guardrail through the Removal Pass.
"""

import sys
import types
import unittest


# The teleop_controller module pulls in sensor_msgs.msg via vr_teleop_node
# imports; stub it so this test runs without ROS installed.
sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")


class _FakeJointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.name = []
        self.position = []


sensor_msgs_msg.JointState = _FakeJointState
sensor_msgs.msg = sensor_msgs_msg
sys.modules.setdefault("sensor_msgs", sensor_msgs)
sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

from mg400_controller.common.config import motion_config


class AdaptiveGateConstantsTest(unittest.TestCase):
    """Lock the adaptive gate's tuning surface.

    Each constant is verified against the value that was on-robot at the
    May 2026 hardware sign-off.  See AGENTS.md §2 for the verification
    context and the commit chronology that landed these defaults.
    """

    # -- Delta band (radians) -------------------------------------------
    def test_delta_min_rad_is_half_degree(self):
        """Smallest cmd delta the gate is willing to send is 0.5°."""
        self.assertAlmostEqual(motion_config.REALTIME_DELTA_MIN_RAD, 0.0087, places=4)

    def test_delta_max_rad_is_three_degrees(self):
        """Largest adaptive cmd delta is 3° — smooth-cruise floor for J2/J3."""
        self.assertAlmostEqual(motion_config.REALTIME_DELTA_MAX_RAD, 0.052, places=3)

    def test_delta_max_greater_than_min(self):
        self.assertGreater(
            motion_config.REALTIME_DELTA_MAX_RAD,
            motion_config.REALTIME_DELTA_MIN_RAD,
        )

    # -- Hand-velocity band (radians/s) ---------------------------------
    def test_hand_vel_low_is_five_deg_per_sec(self):
        self.assertAlmostEqual(
            motion_config.REALTIME_HAND_VEL_LOW_RAD_S, 0.087, places=3
        )

    def test_hand_vel_high_is_thirty_deg_per_sec(self):
        self.assertAlmostEqual(
            motion_config.REALTIME_HAND_VEL_HIGH_RAD_S, 0.524, places=3
        )

    def test_hand_vel_high_greater_than_low(self):
        self.assertGreater(
            motion_config.REALTIME_HAND_VEL_HIGH_RAD_S,
            motion_config.REALTIME_HAND_VEL_LOW_RAD_S,
        )

    # -- Speed / Acc band (percent) -------------------------------------
    def test_speed_j_min_is_25_pct(self):
        self.assertEqual(motion_config.REALTIME_SPEEDJ_MIN, 25)

    def test_speed_j_max_is_100_pct(self):
        self.assertEqual(motion_config.REALTIME_SPEEDJ_MAX, 100)

    def test_acc_j_band_matches_speed_j(self):
        """SpeedJ and AccJ share the same band so ramp shape is consistent."""
        self.assertEqual(
            motion_config.REALTIME_ACCJ_MIN, motion_config.REALTIME_SPEEDJ_MIN
        )
        self.assertEqual(
            motion_config.REALTIME_ACCJ_MAX, motion_config.REALTIME_SPEEDJ_MAX
        )

    # -- CP -------------------------------------------------------------
    def test_realtime_cp_is_80(self):
        """CP=80 chosen via EXP4 5-rep replication."""
        self.assertEqual(motion_config.REALTIME_CP, 80)

    def test_cartesian_cp_max_caps_at_30(self):
        """Cartesian primitives capped at CP=30 to avoid alarm 34322."""
        self.assertEqual(motion_config.SEGMENT_CARTESIAN_CP_MAX, 30)

    def test_cartesian_cp_max_lower_than_realtime_cp(self):
        self.assertLess(
            motion_config.SEGMENT_CARTESIAN_CP_MAX,
            motion_config.REALTIME_CP,
        )

    # -- Path simplification --------------------------------------------
    def test_path_simplify_tolerance_is_three_degrees(self):
        """3° simplify enforces smooth-regime min waypoint spacing."""
        self.assertEqual(motion_config.PATH_SIMPLIFY_TOLERANCE_DEG, 3.0)

    # -- Stuck-recovery thresholds (unchanged across refactor) ----------
    def test_stuck_time_threshold_is_three_hundred_ms(self):
        self.assertEqual(motion_config.STUCK_TIME_THRESHOLD, 0.3)

    def test_proximity_threshold_for_stuck_is_four_point_five_degrees(self):
        """The 0.08 rad floor is only used by the stuck-recovery branch."""
        self.assertAlmostEqual(motion_config.PROXIMITY_THRESHOLD, 0.08, places=4)


if __name__ == "__main__":
    unittest.main()
