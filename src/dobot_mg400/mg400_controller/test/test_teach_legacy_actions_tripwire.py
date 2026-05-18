"""Tripwire: assert the LEGACY teach actions remain wired.

The full behaviour of ``preview_sim``, ``record_start`` and
``record_stop`` is exercised by ``test_teach_job_handler.py``
(``test_preview_sim_*`` + ``test_record_start_and_stop_round_trip``).
This file is a *contract* tripwire:

- the three action constants exist
- they are listed in ``VALID_ACTIONS``
- their string values match the operator-facing job_request schema

If the next engineer's "Removal Pass v1" deletes any of them, this test
must be updated in the same commit — forcing the removal to be a
deliberate, reviewable action rather than an accidental edit.

See AGENTS.md §4.2 for the proof-of-death checklist before removal.
"""

import unittest

from mg400_controller.common.trajectory import teach_job_handler as tjh


class TeachLegacyActionsTripwireTest(unittest.TestCase):
    """LEGACY (2026-05): see AGENTS.md §4.2."""

    def test_preview_sim_action_constant_exists(self):
        self.assertEqual(tjh.ACTION_PREVIEW_SIM, "preview_sim")

    def test_record_start_action_constant_exists(self):
        self.assertEqual(tjh.ACTION_RECORD_START, "record_start")

    def test_record_stop_action_constant_exists(self):
        self.assertEqual(tjh.ACTION_RECORD_STOP, "record_stop")

    def test_preview_sim_is_valid_action(self):
        self.assertIn(tjh.ACTION_PREVIEW_SIM, tjh.VALID_ACTIONS)

    def test_record_start_is_valid_action(self):
        self.assertIn(tjh.ACTION_RECORD_START, tjh.VALID_ACTIONS)

    def test_record_stop_is_valid_action(self):
        self.assertIn(tjh.ACTION_RECORD_STOP, tjh.VALID_ACTIONS)

    def test_legacy_actions_do_not_overlap_with_active_actions(self):
        legacy = {
            tjh.ACTION_PREVIEW_SIM,
            tjh.ACTION_RECORD_START,
            tjh.ACTION_RECORD_STOP,
        }
        active = {
            tjh.ACTION_COMPILE,
            tjh.ACTION_EXECUTE,
            tjh.ACTION_TUNE,
            tjh.ACTION_EXPORT,
            tjh.ACTION_STOP,
        }
        self.assertEqual(legacy & active, set())

    def test_valid_actions_covers_all_known_constants(self):
        """If a new action is added, VALID_ACTIONS must list it."""
        known = {
            tjh.ACTION_COMPILE,
            tjh.ACTION_PREVIEW_SIM,
            tjh.ACTION_EXECUTE,
            tjh.ACTION_TUNE,
            tjh.ACTION_EXPORT,
            tjh.ACTION_STOP,
            tjh.ACTION_RECORD_START,
            tjh.ACTION_RECORD_STOP,
        }
        self.assertEqual(tjh.VALID_ACTIONS, known)


if __name__ == "__main__":
    unittest.main()
