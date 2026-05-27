import unittest

import numpy as np

from mg400_controller.common.utils.pending_command_registry import (
    PendingCommand,
    PendingCommandRegistry,
)


class PendingCommandRegistryTest(unittest.TestCase):
    def test_matches_by_robot_command_id_when_available(self):
        registry = PendingCommandRegistry()
        registry.register(PendingCommand(
            control_command_seq=1,
            ros_command_uid="ros_cmd_000001",
            target_rad=np.array([0.1, 0.2, 0.3, 0.4]),
            dobot_command_id=456,
            sent_wall_timestamp=10.0,
        ))

        match = registry.match(
            np.array([0.101, 0.199, 0.3, 0.4]),
            feedback_command_id=456,
            now_wall=10.2,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=0.01,
        )

        self.assertEqual(match.command.ros_command_uid, "ros_cmd_000001")
        self.assertEqual(match.method, "dobot_id_and_pose_settle")
        self.assertEqual(match.confidence, "high")
        self.assertTrue(match.command_id_match)

    def test_command_id_match_outside_pose_tolerance_stays_pending(self):
        registry = PendingCommandRegistry()
        registry.register(PendingCommand(
            control_command_seq=1,
            ros_command_uid="ros_cmd_000001",
            target_rad=np.array([0.1, 0.2, 0.3, 0.4]),
            dobot_command_id=456,
            sent_wall_timestamp=10.0,
        ))

        match = registry.match(
            np.array([0.5, 0.5, 0.5, 0.5]),
            feedback_command_id=456,
            now_wall=10.2,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=0.01,
        )
        registry.mark_logged(match)

        self.assertEqual(match.command.ros_command_uid, "ros_cmd_000001")
        self.assertEqual(match.status, "command_id_pose_mismatch")
        self.assertEqual(match.method, "dobot_id_pose_outside_tolerance")
        self.assertEqual(match.confidence, "low")
        self.assertTrue(match.command_id_match)
        self.assertFalse(match.ambiguous)
        self.assertEqual(match.candidate_count, 1)
        self.assertGreater(match.match_error_rad, 0.01)
        self.assertEqual(len(registry.active_commands()), 1)

    def test_matches_unambiguous_nearest_pose_without_robot_id(self):
        registry = PendingCommandRegistry()
        registry.register(PendingCommand(
            control_command_seq=1,
            ros_command_uid="ros_cmd_000001",
            target_rad=np.array([0.1, 0.2, 0.3, 0.4]),
            sent_wall_timestamp=10.0,
        ))
        registry.register(PendingCommand(
            control_command_seq=2,
            ros_command_uid="ros_cmd_000002",
            target_rad=np.array([0.4, 0.2, 0.3, 0.4]),
            sent_wall_timestamp=10.1,
        ))

        match = registry.match(
            np.array([0.101, 0.201, 0.3, 0.4]),
            feedback_command_id=0,
            now_wall=10.3,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=0.01,
        )

        self.assertEqual(match.command.ros_command_uid, "ros_cmd_000001")
        self.assertEqual(match.status, "reached_pose_only")
        self.assertEqual(match.confidence, "medium")
        self.assertFalse(match.ambiguous)
        self.assertEqual(match.candidate_count, 1)

    def test_flags_ambiguous_pose_matches_and_keeps_latest_tie(self):
        registry = PendingCommandRegistry()
        registry.register(PendingCommand(
            control_command_seq=1,
            ros_command_uid="ros_cmd_000001",
            target_rad=np.array([0.1, 0.2, 0.3, 0.4]),
            sent_wall_timestamp=10.0,
        ))
        registry.register(PendingCommand(
            control_command_seq=2,
            ros_command_uid="ros_cmd_000002",
            target_rad=np.array([0.1, 0.2, 0.3, 0.4]),
            sent_wall_timestamp=10.1,
        ))

        match = registry.match(
            np.array([0.1, 0.2, 0.3, 0.4]),
            feedback_command_id=0,
            now_wall=10.3,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=0.01,
        )

        self.assertEqual(match.command.ros_command_uid, "ros_cmd_000002")
        self.assertEqual(match.status, "reached_pose_ambiguous")
        self.assertEqual(match.confidence, "low")
        self.assertTrue(match.ambiguous)
        self.assertEqual(match.candidate_count, 2)
        self.assertEqual(match.match_error_rad, 0.0)
        self.assertEqual(match.second_best_error_rad, 0.0)

    def test_pass_samples_log_joint_and_tool_matches_once(self):
        registry = PendingCommandRegistry()
        registry.register(PendingCommand(
            control_command_seq=1,
            ros_command_uid="ros_cmd_000001",
            target_rad=np.array([0.1, 0.2, 0.3, 0.4]),
            target_tool=np.array([250.0, -90.0, 100.0, -20.0, 0.0, 0.0]),
            sent_wall_timestamp=10.0,
        ))

        samples = registry.pass_samples(
            np.array([0.101, 0.201, 0.3, 0.4]),
            tool_actual=np.array([251.0, -91.0, 103.0, -20.2, 0.0, 0.0]),
            now_wall=10.2,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=0.01,
            tool_xyz_tolerance_mm=5.0,
            tool_r_tolerance_deg=1.0,
        )

        self.assertEqual([sample.event_type for sample in samples], ["joint_match", "tool_match"])
        self.assertEqual([sample.status for sample in samples], ["passed_near_4j", "passed_near_xyz"])
        self.assertAlmostEqual(samples[0].joint_norm_error_rad, np.sqrt(0.001**2 + 0.001**2))
        self.assertAlmostEqual(samples[1].tool_xyz_error_mm, np.sqrt(11.0))

        duplicate_samples = registry.pass_samples(
            np.array([0.1, 0.2, 0.3, 0.4]),
            tool_actual=np.array([250.0, -90.0, 100.0, -20.0, 0.0, 0.0]),
            now_wall=10.3,
            norm_tolerance_rad=0.01,
            per_joint_tolerance_rad=0.01,
            tool_xyz_tolerance_mm=5.0,
            tool_r_tolerance_deg=1.0,
        )

        self.assertEqual(duplicate_samples, [])


if __name__ == "__main__":
    unittest.main()
