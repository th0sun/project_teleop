"""Tripwire: the dormant ``fastest_path_repeat`` profile stays revivable.

Default playback uses ``PLAYBACK_EXECUTION_PROFILE = "preserve_timing"``
and no current Unity build sends a job_request payload that flips the
profile.  But the code path *exists* and the FAST_REPEAT_* constants
define its tuning surface.

This file pins two contracts:

1. The dormant constants in ``motion_config.py`` keep their committed
   values, so a refactor of the constants block can't silently change
   what fastest_path_repeat would do if revived.
2. ``TrajectoryRecorder._fastest_path_repeat_enabled`` toggles
   correctly when the operator-set execution_profile is the activation
   string.  No real motion is sent — only the gate predicate is
   evaluated.

LEGACY (2026-05): see AGENTS.md §4.3.  Removal of the FAST_REPEAT_*
profile must delete this test in the same commit.
"""

import tempfile
import unittest

from mg400_controller.common.config import motion_config
from mg400_controller.common.trajectory.trajectory_recorder import (
    PLAYBACK_PROFILE_FASTEST_PATH_REPEAT,
    PLAYBACK_PROFILE_PRESERVE_TIMING,
    TrajectoryRecorder,
)


class FakeLogger:
    def info(self, msg): pass
    def warn(self, msg): pass
    def error(self, msg): pass


class FastestPathRepeatTripwireTest(unittest.TestCase):
    """LEGACY (2026-05): see AGENTS.md §4.3."""

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self._saved_profile = motion_config.PLAYBACK_EXECUTION_PROFILE

    def tearDown(self):
        motion_config.PLAYBACK_EXECUTION_PROFILE = self._saved_profile
        self.temp_dir.cleanup()

    # -- Constants ------------------------------------------------------
    def test_default_profile_is_preserve_timing(self):
        """The shipping default must keep fastest_path_repeat dormant."""
        self.assertEqual(
            motion_config.PLAYBACK_EXECUTION_PROFILE, "preserve_timing"
        )

    def test_fast_repeat_speed_constants_locked(self):
        self.assertEqual(motion_config.FAST_REPEAT_SPEED_J, 100)
        self.assertEqual(motion_config.FAST_REPEAT_ACC_J, 100)
        self.assertEqual(motion_config.FAST_REPEAT_SPEED_L, 100)
        self.assertEqual(motion_config.FAST_REPEAT_ACC_L, 100)

    def test_fast_repeat_cp_constants_locked(self):
        self.assertEqual(motion_config.FAST_REPEAT_CP, 100)
        self.assertEqual(motion_config.FAST_REPEAT_FINAL_CP, 0)

    def test_fast_repeat_lookahead_and_timeout_locked(self):
        self.assertEqual(motion_config.FAST_REPEAT_LOOKAHEAD_SEC, 10.0)
        self.assertEqual(motion_config.FAST_REPEAT_TIMEOUT_PER_COMMAND_SEC, 1.0)
        self.assertEqual(motion_config.FAST_REPEAT_MAX_COMMANDS_PER_CYCLE, 1)

    # -- Profile-selector predicate ------------------------------------
    def _make_recorder(self):
        return TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )

    def test_predicate_off_by_default(self):
        rec = self._make_recorder()
        self.assertFalse(rec._fastest_path_repeat_enabled())

    def test_predicate_on_when_motion_config_sets_profile(self):
        """Operator override path is the motion_config module attribute."""
        motion_config.PLAYBACK_EXECUTION_PROFILE = "fastest_path_repeat"
        rec = self._make_recorder()
        self.assertTrue(rec._fastest_path_repeat_enabled())

    def test_predicate_accepts_fast_and_fastest_aliases(self):
        for alias in ("fast", "fastest", PLAYBACK_PROFILE_FASTEST_PATH_REPEAT):
            motion_config.PLAYBACK_EXECUTION_PROFILE = alias
            rec = self._make_recorder()
            self.assertTrue(
                rec._fastest_path_repeat_enabled(),
                f"alias {alias!r} should activate the fastest profile",
            )

    def test_predicate_falls_back_to_preserve_timing_on_unknown_string(self):
        motion_config.PLAYBACK_EXECUTION_PROFILE = "totally_made_up"
        rec = self._make_recorder()
        self.assertFalse(rec._fastest_path_repeat_enabled())
        # And the resolved profile name normalises to the default
        self.assertEqual(
            rec._playback_execution_profile(),
            PLAYBACK_PROFILE_PRESERVE_TIMING,
        )


if __name__ == "__main__":
    unittest.main()
