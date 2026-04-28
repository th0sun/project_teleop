import unittest
import tempfile

import numpy as np

from mg400_controller.common.trajectory.trajectory_recorder import (
    PREVIEW_FINAL_TOLERANCE_DEG,
    PREVIEW_START_TOLERANCE_DEG,
    compiled_playback_plan_to_dict,
    TrajectoryRecorder,
    frames_from_joint_trajectory_msg,
)
from mg400_controller.common.utils.kinematics import KinematicsCalculator


class FakeLogger:
    def info(self, msg):
        self.last_info = msg

    def warn(self, msg):
        self.last_warn = msg

    def error(self, msg):
        self.last_error = msg


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def sleep(self, amount):
        self.value += amount


class PositionFeed:
    def __init__(self, positions_deg):
        self.positions_deg = list(positions_deg)
        self.index = 0

    def __call__(self):
        if self.index >= len(self.positions_deg):
            current = self.positions_deg[-1]
        else:
            current = self.positions_deg[self.index]
            self.index += 1
        return np.radians(current)


class TrajectoryRecorderTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_send_go_to_start_uses_start_motion_and_waits_until_arrival(self):
        sent_commands = []
        clock = FakeClock()
        feed = PositionFeed([
            [15.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_commands.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )

        arrived = recorder._send_go_to_start({
            "j1": 0.0,
            "j2": 0.0,
            "j3": 0.0,
            "j4": 0.0,
        })

        self.assertTrue(arrived)
        self.assertEqual(len(sent_commands), 1)
        self.assertIn("SpeedJ=20", sent_commands[0])
        self.assertIn("AccJ=50", sent_commands[0])
        self.assertIn("CP=0", sent_commands[0])

    def test_wait_until_near_target_times_out_when_position_never_arrives(self):
        clock = FakeClock()
        feed = PositionFeed([
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )

        arrived = recorder._wait_until_near_target(
            [0.0, 0.0, 0.0, 0.0],
            tolerance_deg=PREVIEW_START_TOLERANCE_DEG,
            timeout_sec=0.12,
        )

        self.assertFalse(arrived)

    def test_playback_complete_waits_for_robot_to_reach_final_target(self):
        clock = FakeClock()
        target_q = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
        ])
        feed = PositionFeed([
            [25.0, 10.0, 0.0, 0.0],
            [20.4, 10.2, 0.0, 0.0],
            [20.03, 10.01, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )

        self.assertFalse(recorder._playback_complete(False, 1.2, 1.0, target_q))
        self.assertFalse(recorder._playback_complete(True, 1.0, 1.0, target_q))
        self.assertFalse(recorder._playback_complete(True, 1.1, 1.0, target_q))
        self.assertTrue(recorder._playback_complete(True, 1.2, 1.0, target_q))
        self.assertLessEqual(PREVIEW_FINAL_TOLERANCE_DEG, 0.05)

    def test_playback_complete_waits_for_robot_mode_to_leave_running(self):
        target_q = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
        ])
        feed = PositionFeed([
            [20.0, 10.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
        ])
        modes = iter([7, 5])
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=feed,
            get_robot_mode_fn=lambda: next(modes),
            traj_dir=self.temp_dir.name,
        )

        self.assertFalse(recorder._playback_complete(True, 1.0, 1.0, target_q))
        self.assertTrue(recorder._playback_complete(True, 1.1, 1.0, target_q))

    def test_playback_complete_uses_extra_timeout_without_feedback(self):
        clock = FakeClock()
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=None,
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        target_q = np.array([[0.0, 0.0, 0.0, 0.0]])

        self.assertFalse(recorder._playback_complete(False, 3.1, 1.0, target_q))
        self.assertFalse(recorder._playback_complete(True, 1.5, 1.0, target_q))
        self.assertTrue(recorder._playback_complete(True, 3.1, 1.0, target_q))

    def test_playback_complete_does_not_succeed_on_absolute_timeout_with_feedback(self):
        target_q = np.array([
            [0.0, 0.0, 0.0, 0.0],
            [20.0, 10.0, 0.0, 0.0],
        ])
        feed = PositionFeed([
            [40.0, 30.0, 0.0, 0.0],
            [40.0, 30.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            get_position_fn=feed,
            traj_dir=self.temp_dir.name,
        )

        self.assertFalse(recorder._playback_complete(True, 999.0, 1.0, target_q))
        self.assertTrue(recorder._playback_timed_out(999.0, 1.0))

    def test_timeout_flushes_motion_queue_when_dashboard_channel_exists(self):
        dashboard_commands = []
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            dashboard_send_fn=dashboard_commands.append,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )

        self.assertTrue(recorder._flush_motion_queue_after_timeout())
        self.assertEqual(dashboard_commands, ["ResetRobot()", "EnableRobot()"])

    def test_segment_speed_j_scales_with_delta_over_time(self):
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        prev_frame = {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0}
        slow_frame = {"timeStamp": 0.20, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0}
        fast_frame = {"timeStamp": 0.05, "j1": 5.0, "j2": 0.0, "j3": 0.0, "j4": 0.0}

        slow_speed = recorder._segment_speed_j(prev_frame, slow_frame)
        fast_speed = recorder._segment_speed_j(prev_frame, fast_frame)

        self.assertGreater(fast_speed, slow_speed)
        self.assertGreaterEqual(slow_speed, 15)
        self.assertLessEqual(fast_speed, 100)

    def test_mg400_ik_round_trips_known_joint_pose(self):
        kin = KinematicsCalculator()
        joints = np.array([20.0, 10.0, 15.0, -5.0])
        tool = kin.forward_kinematics(joints)
        solved = kin.inverse_kinematics(tool[:4])

        np.testing.assert_allclose(solved, joints, atol=1e-4)

    def test_line_primitive_reachability_rejects_unreachable_midpoint(self):
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )

        class FakeSegment:
            start_xyzr = (250.0, 0.0, 120.0, 0.0)
            end_xyzr = (600.0, 0.0, 120.0, 0.0)

        self.assertFalse(recorder._line_primitive_reachable(FakeSegment(), samples=4))

    def test_compile_loaded_plan_precomputes_commands_and_timing(self):
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        recorder.load_frames(
            [
                {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 0.20, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 0.30, "j1": 10.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            ],
            name="demo.json",
        )

        plan = recorder.compile_loaded_plan()

        self.assertEqual(plan.source_name, "demo.json")
        self.assertEqual(len(plan.waypoints), 3)
        # With mixed primitives, the 3 waypoints (all j1-only, others zero)
        # may be collapsed into fewer commands than 2.  The key invariant is
        # that we get at least 1 queued command and the plan compiles.
        self.assertGreaterEqual(len(plan.queued_commands), 1)
        self.assertAlmostEqual(plan.total_duration_s, 0.3)
        self.assertTrue(plan.original_timing_feasible)
        # Last command must settle (CP=0)
        self.assertEqual(plan.queued_commands[-1].cp, 0)

    def test_fastest_path_repeat_uses_fast_caps_instead_of_hand_timestamps(self):
        self._set_simplify_tolerance(0.0)
        self._set_mixed_primitives(False)
        self._set_execution_profile("fastest_path_repeat")
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        recorder.load_frames(
            [
                {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 2.0, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 4.0, "j1": 4.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            ],
            name="slow_hand.json",
        )

        plan = recorder.compile_loaded_plan()
        payload = compiled_playback_plan_to_dict(plan)

        self.assertEqual(plan.execution_profile, "fastest_path_repeat")
        self.assertEqual(payload["execution_profile"], "fastest_path_repeat")
        self.assertEqual([cmd.speed_j for cmd in plan.queued_commands], [100, 100])
        self.assertLess(plan.queued_commands[0].target_time_s, 2.0)
        self.assertLess(plan.queued_commands[1].target_time_s, 4.0)
        self.assertIn("AccJ=100", plan.queued_commands[0].command)
        self.assertIn("CP=100", plan.queued_commands[0].command)
        self.assertIn("CP=0", plan.queued_commands[-1].command)

    def test_compile_loaded_plan_includes_robot_neutral_io_events(self):
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        recorder.load_frames(
            [
                {"timeStamp": 10.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 11.0, "j1": 10.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            ],
            name="with_events.json",
            events=[
                {"timeStamp": 10.25, "kind": "digital_output", "channel": "vacuum", "value": True},
                {"timeStamp": 10.50, "kind": "digital_output", "channel": "green_light", "value": True},
            ],
        )

        plan = recorder.compile_loaded_plan()
        payload = compiled_playback_plan_to_dict(plan)

        self.assertEqual(len(plan.event_commands), 2)
        self.assertEqual(payload["event_command_count"], 2)
        self.assertEqual(plan.event_commands[0].channel, "vacuum")
        self.assertIn("DOExecute(16,1)", plan.event_commands[0].commands)
        self.assertIn("DOExecute(15,0)", plan.event_commands[0].commands)
        self.assertEqual(plan.event_commands[1].port, 3)
        self.assertIn("DOExecute(3,1)", plan.event_commands[1].commands)

    # ── Path simplification (RDP) ──────────────────────────────────────────────
    # The recorder ships with motion_config.PATH_SIMPLIFY_TOLERANCE_DEG = 0.5°
    # so dense Unity-recorded waypoints collapse before they hit the MG400
    # motion queue.  These tests run the full compile + _play_worker pipeline
    # against a behavioural robot mock (PositionFeed for feedback, list.append
    # for the dashboard / motion send channels) to verify the simplified
    # command stream actually reaches the wire.

    def _set_simplify_tolerance(self, value):
        """Patch the global tolerance config and undo it on test teardown."""
        import mg400_controller.common.config.motion_config as cfg
        original = cfg.PATH_SIMPLIFY_TOLERANCE_DEG
        cfg.PATH_SIMPLIFY_TOLERANCE_DEG = value
        self.addCleanup(setattr, cfg, "PATH_SIMPLIFY_TOLERANCE_DEG", original)

    def _set_mixed_primitives(self, value):
        """Patch USE_MIXED_PRIMITIVES and undo it on test teardown."""
        import mg400_controller.common.config.motion_config as cfg
        original = getattr(cfg, "USE_MIXED_PRIMITIVES", True)
        cfg.USE_MIXED_PRIMITIVES = value
        self.addCleanup(setattr, cfg, "USE_MIXED_PRIMITIVES", original)

    def _set_execution_profile(self, value):
        """Patch PLAYBACK_EXECUTION_PROFILE and undo it on test teardown."""
        import mg400_controller.common.config.motion_config as cfg
        original = getattr(cfg, "PLAYBACK_EXECUTION_PROFILE", "preserve_timing")
        cfg.PLAYBACK_EXECUTION_PROFILE = value
        self.addCleanup(setattr, cfg, "PLAYBACK_EXECUTION_PROFILE", original)

    def test_compile_collapses_dense_straight_line_to_endpoints(self):
        self._set_simplify_tolerance(0.5)
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        # 11 evenly-spaced waypoints along a perfectly straight 0°→10° motion
        # in j1.  RDP should drop everything between the endpoints.
        recorder.load_frames([
            {"timeStamp": i * 0.1, "j1": float(i), "j2": 0.0, "j3": 0.0, "j4": 0.0}
            for i in range(11)
        ], name="line.json")

        plan = recorder.compile_loaded_plan()

        self.assertEqual(plan.raw_waypoint_count, 11)
        self.assertEqual(len(plan.waypoints), 2,
                         "straight line should collapse to endpoints in the playback plan")
        self.assertEqual(len(plan.queued_commands), 1,
                         "straight line should produce exactly one queued JointMovJ command")
        self.assertAlmostEqual(plan.simplify_tolerance_deg, 0.5)

    def test_compile_preserves_l_corner_waypoint(self):
        self._set_simplify_tolerance(0.5)
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        # Right-then-up L: 5 points right, corner at t=0.5, 5 points up
        frames = []
        for i in range(6):
            frames.append({"timeStamp": i * 0.1, "j1": float(i), "j2": 0.0, "j3": 0.0, "j4": 0.0})
        for i in range(1, 6):
            frames.append({"timeStamp": 0.5 + i * 0.1, "j1": 5.0, "j2": float(i), "j3": 0.0, "j4": 0.0})
        recorder.load_frames(frames, name="corner.json")

        plan = recorder.compile_loaded_plan()

        self.assertEqual(plan.raw_waypoint_count, 11)
        self.assertEqual(len(plan.waypoints), 3)
        # Endpoints + corner — the corner waypoint is the one with j1=5, j2=0.
        corner = plan.waypoints[1]
        self.assertAlmostEqual(corner["timeStamp"], 0.5)
        self.assertAlmostEqual(corner["j1"], 5.0)
        self.assertAlmostEqual(corner["j2"], 0.0)

    def test_compile_keeps_curve_proportional_to_curvature(self):
        import math
        self._set_simplify_tolerance(0.5)
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        # 21 samples of a 10° sine arc — should keep more than 2 (curve) but
        # fewer than 21 (not every dense sample).
        frames = [
            {"timeStamp": i / 20.0,
             "j1": 10.0 * math.sin(math.pi * i / 20.0),
             "j2": 0.0, "j3": 0.0, "j4": 0.0}
            for i in range(21)
        ]
        recorder.load_frames(frames, name="curve.json")

        plan = recorder.compile_loaded_plan()

        self.assertGreater(len(plan.waypoints), 2)
        self.assertLess(len(plan.waypoints), 21)

    def test_compile_skips_simplification_when_tolerance_zero(self):
        self._set_simplify_tolerance(0.0)
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        recorder.load_frames([
            {"timeStamp": i * 0.1, "j1": float(i), "j2": 0.0, "j3": 0.0, "j4": 0.0}
            for i in range(11)
        ], name="line.json")

        plan = recorder.compile_loaded_plan()

        # Tolerance 0 disables simplification — every recorded frame stays.
        self.assertEqual(len(plan.waypoints), 11)
        self.assertEqual(plan.raw_waypoint_count, 11)

    def test_play_worker_dispatches_only_simplified_commands_to_robot_mock(self):
        """End-to-end: dense line → compile → _play_worker → motion port mock.

        The mock collects every JointMovJ string that would hit port 30003 on
        a real MG400.  With simplification on, only the start go-to-start
        plus the single endpoint command should reach the wire."""
        self._set_simplify_tolerance(0.5)

        sent_motion = []
        events = []
        clock = FakeClock()
        # PositionFeed simulates the robot reaching the start, holding, then
        # arriving at the endpoint — same behavioural pattern existing tests
        # use to mock the real arm's feedback loop.
        feed = PositionFeed([
            [0.0, 0.0, 0.0, 0.0],   # arrived at start
            [0.0, 0.0, 0.0, 0.0],   # still at start
            [10.0, 0.0, 0.0, 0.0],  # arrived at endpoint
            [10.0, 0.0, 0.0, 0.0],  # holding
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_motion.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            playback_event_callback=lambda e, p: events.append((e, p)),
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        recorder.load_frames([
            {"timeStamp": i * 0.1, "j1": float(i), "j2": 0.0, "j3": 0.0, "j4": 0.0}
            for i in range(11)
        ])

        recorder._play_worker()

        # 1 go-to-start + 1 simplified queued command = 2 wire-level commands.
        # Without simplification this would be 1 + 10 = 11 commands.
        self.assertEqual(
            len(sent_motion), 2,
            f"simplified straight line should hit wire as 2 commands; got {sent_motion}",
        )
        self.assertIn("JointMovJ(0.0000", sent_motion[0])  # go-to-start
        # Endpoint command: may be JointMovJ or MovL depending on mixed mode
        self.assertIn("10.0000", sent_motion[1])  # endpoint j1 or x-coord
        # Final command must settle (CP=0), not blend.
        self.assertIn("CP=0", sent_motion[1])

        queued = [e for e in events if e[0] == "waypoint_queued"]
        self.assertEqual(len(queued), 1, "exactly one waypoint should be queued")

    def test_play_worker_completes_when_mixed_command_count_is_less_than_frames(self):
        """Regression: command pointer must not be compared to frame count."""
        self._set_simplify_tolerance(0.5)
        self._set_mixed_primitives(True)
        sent_motion = []
        events = []
        clock = FakeClock()
        feed = PositionFeed([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_motion.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            playback_event_callback=lambda e, p: events.append((e, p)),
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        recorder.load_frames([
            {"timeStamp": i * 0.1, "j1": float(i), "j2": 0.0, "j3": 0.0, "j4": 0.0}
            for i in range(11)
        ])

        recorder._play_worker()

        completion = [event for event in events if event[0] == "playback_complete"]
        self.assertEqual(len(completion), 1)
        self.assertTrue(completion[0][1]["success"], completion[0][1])
        self.assertFalse(completion[0][1]["timed_out"], completion[0][1])
        self.assertLess(clock.value - 100.0, 5.0)

    def test_simplification_preserves_io_event_timing_alignment(self):
        """Events at intermediate timestamps must still fire at the correct
        retimed point even when their original frame is dropped by RDP."""
        self._set_simplify_tolerance(0.5)
        sent_motion = []
        sent_dashboard = []
        events = []
        clock = FakeClock()
        feed = PositionFeed([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_motion.append,
            dashboard_send_fn=sent_dashboard.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            playback_event_callback=lambda e, p: events.append((e, p)),
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        # Straight line in j1, vacuum-on event at the midpoint.  RDP will drop
        # the middle motion frames but the event must still fire at retimed
        # midpoint via np.interp on the simplified source_target_t.
        recorder.load_frames(
            [
                {"timeStamp": i * 0.1, "j1": float(i), "j2": 0.0, "j3": 0.0, "j4": 0.0}
                for i in range(11)
            ],
            events=[
                {"timeStamp": 0.5, "kind": "digital_output", "channel": "vacuum", "value": True},
            ],
        )

        recorder._play_worker()

        # Vacuum DOExecute must have hit the dashboard channel.
        self.assertTrue(
            any("DOExecute(16,1)" in cmd for cmd in sent_dashboard),
            f"vacuum-on must dispatch DOExecute(16,1); got {sent_dashboard}",
        )
        io = [e for e in events if e[0] == "io_event_queued"]
        self.assertEqual(len(io), 1)
        # Retimed event time should land roughly at the midpoint of the
        # simplified 0→1.0s path.  Allow ±0.1s slack for retiming math.
        self.assertAlmostEqual(io[0][1]["target_time_s"], 0.5, delta=0.1)

    def test_translate_digital_event_raises_when_vacuum_port_misconfigured(self):
        import mg400_controller.common.config.motion_config as cfg
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        original = cfg.VACUUM_DO_PORT
        try:
            cfg.VACUUM_DO_PORT = 0
            with self.assertRaises(ValueError, msg="VACUUM_DO_PORT=0 must raise ValueError at compile time"):
                recorder._translate_digital_event(
                    {"kind": "digital_output", "channel": "vacuum", "value": True}
                )
        finally:
            cfg.VACUUM_DO_PORT = original

    def test_stop_all_cancels_pending_delayed_io_timers(self):
        import time as real_time
        sent_dashboard = []
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            dashboard_send_fn=sent_dashboard.append,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        # Schedule a blow-off timer with a real 10-second delay (would never fire
        # in a fast test run without cancellation).
        recorder._schedule_delayed_event_command(10.0, "DOExecute(15,0)")
        self.assertEqual(len(recorder._pending_timers), 1)

        recorder.stop_all()

        # Brief pause — timer must NOT have fired.
        real_time.sleep(0.05)
        self.assertNotIn("DOExecute(15,0)", sent_dashboard,
                         "stop_all() must cancel pending delayed IO timers")
        self.assertEqual(len(recorder._pending_timers), 0)

    def test_export_loaded_plan_writes_json_artifact(self):
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        recorder.load_frames(
            [
                {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 0.2, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            ],
            name="export_demo.json",
        )

        artifact_path = recorder.export_loaded_plan(
            f"{self.temp_dir.name}/compiled/export_demo.compiled_playback.json"
        )

        import json

        with open(artifact_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)

        self.assertEqual(data["artifact_kind"], "mg400_compiled_playback_plan")
        self.assertEqual(data["source_name"], "export_demo.json")
        self.assertEqual(data["waypoint_count"], 2)
        self.assertEqual(data["queued_command_count"], 1)
        cmd = data["queued_commands"][0]["command"]
        self.assertTrue(
            "JointMovJ" in cmd or "MovL" in cmd or "Arc" in cmd,
            f"Expected a motion command, got: {cmd}",
        )

    def test_compiled_playback_plan_to_dict_is_json_safe(self):
        recorder = TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )
        recorder.load_frames(
            [
                {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 0.2, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            ]
        )

        payload = compiled_playback_plan_to_dict(recorder.compile_loaded_plan())

        self.assertEqual(payload["artifact_version"], "0.1")
        self.assertIsInstance(payload["waypoints"], list)
        self.assertIsInstance(payload["queued_commands"][0]["joints_deg"], list)

    def test_play_worker_skips_duplicate_start_and_uses_segment_speed(self):
        self._set_mixed_primitives(False)  # Test JointMovJ-only pipeline
        sent_commands = []
        events = []
        clock = FakeClock()
        feed = PositionFeed([
            [15.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_commands.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            playback_event_callback=lambda event, payload: events.append((event, payload)),
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        recorder.loaded_frames = [
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 0.20, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 0.30, "j1": 10.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
        ]

        recorder._play_worker()

        self.assertEqual(len(sent_commands), 3)
        self.assertIn("SpeedJ=20", sent_commands[0])  # go-to-start
        self.assertIn("JointMovJ(2.0000", sent_commands[1])
        self.assertIn("SpeedJ=15", sent_commands[1])  # 2 deg / 0.2 s fits below min preview speed
        self.assertIn("JointMovJ(10.0000", sent_commands[2])
        self.assertIn("SpeedJ=89", sent_commands[2])  # 8 deg / 0.1 s -> 80 deg/s of 90 deg/s at SpeedJ=100
        self.assertIn("CP=0", sent_commands[2])  # final command should settle, not blend
        self.assertEqual(events[0][0], "go_to_start_command")
        self.assertEqual(events[1][0], "playback_start")
        self.assertTrue(events[1][1]["original_timing_feasible"])
        self.assertEqual(events[1][1]["execution_model"], "compiled_queue_plan")
        queued = [event for event in events if event[0] == "waypoint_queued"]
        self.assertEqual([event[1]["index"] for event in queued], [1, 2])

    def test_play_worker_dispatches_io_events_on_dashboard_channel(self):
        sent_motion = []
        sent_dashboard = []
        events = []
        clock = FakeClock()
        feed = PositionFeed([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_motion.append,
            dashboard_send_fn=sent_dashboard.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            playback_event_callback=lambda event, payload: events.append((event, payload)),
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        recorder.load_frames(
            [
                {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
                {"timeStamp": 0.2, "j1": 2.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            ],
            events=[
                {"timeStamp": 0.0, "kind": "digital_output", "channel": "green_light", "value": True},
            ],
        )

        recorder._play_worker()

        self.assertIn("DOExecute(3,1)", sent_dashboard)
        io_events = [event for event in events if event[0] == "io_event_queued"]
        self.assertEqual(len(io_events), 1)
        self.assertEqual(io_events[0][1]["channel"], "green_light")

    def test_play_worker_retimes_too_fast_segments_instead_of_decimating(self):
        self._set_mixed_primitives(False)  # Test JointMovJ-only pipeline
        sent_commands = []
        events = []
        clock = FakeClock()
        feed = PositionFeed([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [30.0, 0.0, 0.0, 0.0],
        ])
        recorder = TrajectoryRecorder(
            command_send_fn=sent_commands.append,
            logger=FakeLogger(),
            get_position_fn=feed,
            playback_event_callback=lambda event, payload: events.append((event, payload)),
            traj_dir=self.temp_dir.name,
            time_fn=clock,
            sleep_fn=clock.sleep,
        )
        recorder.loaded_frames = [
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 0.05, "j1": 30.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
        ]

        recorder._play_worker()

        self.assertEqual(len(sent_commands), 2)
        self.assertIn("JointMovJ(30.0000", sent_commands[1])
        self.assertIn("SpeedJ=100", sent_commands[1])
        playback_start = events[1][1]
        self.assertFalse(playback_start["original_timing_feasible"])
        self.assertAlmostEqual(playback_start["original_duration_s"], 0.05)
        self.assertAlmostEqual(playback_start["retimed_duration_s"], 30.0 / 90.0)
        queued = [event for event in events if event[0] == "waypoint_queued"]
        self.assertAlmostEqual(queued[0][1]["original_target_time_s"], 0.05)
        self.assertAlmostEqual(queued[0][1]["target_time_s"], 30.0 / 90.0, places=5)

    def test_converts_unity_joint_trajectory_msg_to_internal_degree_frames(self):
        class Duration:
            def __init__(self, sec, nanosec):
                self.sec = sec
                self.nanosec = nanosec

        class Point:
            def __init__(self, positions, sec, nanosec):
                self.positions = positions
                self.time_from_start = Duration(sec, nanosec)

        msg = type("Msg", (), {})()
        msg.points = [
            Point(np.radians([0.0, 0.0, 0.0, 0.0]), 0, 0),
            Point(np.radians([10.0, -5.0, 0.0, 0.0]), 0, 500_000_000),
            Point(np.radians([20.0, -10.0, 0.0, 0.0]), 1, 0),
        ]

        frames = frames_from_joint_trajectory_msg(msg)

        self.assertEqual(len(frames), 3)
        self.assertEqual(frames[0]["timeStamp"], 0.0)
        self.assertEqual(frames[1]["timeStamp"], 0.5)
        self.assertEqual(frames[2]["j1"], 20.0)
        self.assertEqual(frames[2]["j2"], -10.0)


if __name__ == "__main__":
    unittest.main()
