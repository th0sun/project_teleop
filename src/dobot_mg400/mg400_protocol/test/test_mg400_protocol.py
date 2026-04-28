import unittest
from pathlib import Path

import numpy as np

from mg400_protocol.alarms import AlarmCatalog
from mg400_protocol.commands import (
    arc,
    circle,
    do_execute,
    joint_mov_j,
    mov_j,
    mov_l,
    mov_l_cartesian,
    mov_l_io,
)
from mg400_protocol.dashboard import (
    acc_j,
    clear_error,
    continue_,
    continue_script,
    disable_robot,
    emergency_stop,
    enable_robot,
    get_pose,
    get_tool,
    pause,
    pause_script,
    reset_robot,
    run_script,
    speed_factor,
    speed_j,
    stop_script,
)
from mg400_protocol.feedback import (
    FEEDBACK_PACKET_SIZE,
    FEEDBACK_TEST_VALUE,
    MG400_FEEDBACK_DTYPE,
    parse_feedback_packet,
)

REPO_ROOT = Path(__file__).resolve().parents[4]


class MG400ProtocolTest(unittest.TestCase):
    def test_command_builders_render_vendor_ascii_format(self):
        motion = joint_mov_j(
            (0.0, 5.72957795, -5.72957795, 0.0),
            speed_j=40,
            acc_j=100,
            cp=100,
        )
        digital_output = do_execute(16, True)

        self.assertEqual(
            motion.render(),
            "JointMovJ(0.0000,5.7296,-5.7296,0.0000,SpeedJ=40,AccJ=100,CP=100)",
        )
        self.assertEqual(digital_output.render(), "DOExecute(16,1)")

    def test_cp_zero_is_preserved_unlike_speed_clamp(self):
        # CP=0 means "no blending" in the MG400 vendor protocol; SpeedJ/AccJ=0
        # do not. Clamps must not collapse them together.
        cmd = joint_mov_j(
            (0.0, 0.0, 0.0, 0.0), speed_j=20, acc_j=50, cp=0,
        )
        self.assertIn("CP=0", cmd.render())
        self.assertIn("SpeedJ=20", cmd.render())

    def test_legacy_motion_builders_match_existing_runtime_format(self):
        self.assertEqual(
            mov_j((1.0, 2.0, 3.0, 4.0), speed_j=50, acc_j=100, cp=100).render(),
            "MovJ(1.0000,2.0000,3.0000,4.0000,SpeedJ=50,AccJ=100,CP=100)",
        )
        self.assertEqual(
            mov_l((1.0, 2.0, 3.0, 4.0), speed_j=50, acc_j=100, cp=100).render(),
            "MovL(1.0000,2.0000,3.0000,4.0000,SpeedJ=50,AccJ=100,CP=100)",
        )

    def test_feedback_layout_matches_1440_byte_vendor_packet(self):
        self.assertEqual(MG400_FEEDBACK_DTYPE.itemsize, FEEDBACK_PACKET_SIZE)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["QTarget"][1], 192)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["QActual"][1], 432)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["ToolVectorActual"][1], 624)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["ToolVectorTarget"][1], 768)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["RunQueuedCmd"][1], 1014)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["ErrorStatus"][1], 1029)
        self.assertEqual(MG400_FEEDBACK_DTYPE.fields["CurrentCommandId"][1], 1112)

    def test_parse_feedback_packet_returns_named_snapshot(self):
        record = np.zeros(1, dtype=MG400_FEEDBACK_DTYPE)
        record["TestValue"][0] = FEEDBACK_TEST_VALUE
        record["QActual"][0][:4] = [1.0, 2.0, 3.0, 4.0]
        record["QTarget"][0][:4] = [5.0, 6.0, 7.0, 8.0]
        record["RobotMode"][0] = 5
        record["RunQueuedCmd"][0] = 1
        record["CurrentCommandId"][0] = 123
        record["ToolVectorActual"][0] = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]

        snapshot = parse_feedback_packet(record.tobytes())

        self.assertIsNotNone(snapshot)
        self.assertEqual(snapshot.q_actual_deg, (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(snapshot.q_target_deg, (5.0, 6.0, 7.0, 8.0))
        self.assertEqual(snapshot.robot_mode, 5)
        self.assertEqual(snapshot.run_queued_cmd, 1)
        self.assertEqual(snapshot.current_command_id, 123)
        self.assertEqual(snapshot.tool_vector_actual[0], 10.0)

    def test_dashboard_state_commands_render_vendor_format(self):
        self.assertEqual(enable_robot().render(), "EnableRobot()")
        self.assertEqual(disable_robot().render(), "DisableRobot()")
        self.assertEqual(clear_error().render(), "ClearError()")
        self.assertEqual(reset_robot().render(), "ResetRobot()")
        self.assertEqual(emergency_stop().render(), "EmergencyStop()")
        self.assertEqual(pause().render(), "Pause()")
        self.assertEqual(continue_().render(), "Continue()")

    def test_dashboard_speed_commands_clamp_to_vendor_range(self):
        self.assertEqual(speed_factor(100).render(), "SpeedFactor(100)")
        self.assertEqual(speed_factor(0).render(), "SpeedFactor(1)")    # clamps to 1
        self.assertEqual(speed_factor(150).render(), "SpeedFactor(100)")  # clamps to 100
        self.assertEqual(speed_j(50).render(), "SpeedJ(50)")
        self.assertEqual(acc_j(75).render(), "AccJ(75)")

    def test_dashboard_introspection_commands(self):
        self.assertEqual(get_tool().render(), "GetTool()")
        self.assertEqual(get_pose().render(), "GetPose()")

    def test_run_script_renders_with_double_quoted_project_name(self):
        # Reference: 4-axis TCP/IP Remote Control Interface Guide
        # V1.6.0.0 (2024/04/19) example: ``RunScript("demo")``.
        # The manual prescribes the project name wrapped in double quotes;
        # this asserts our builder matches that wire format exactly.
        self.assertEqual(run_script("demo").render(), 'RunScript("demo")')
        self.assertEqual(
            run_script("vr_lesson_42").render(), 'RunScript("vr_lesson_42")'
        )

    def test_run_script_rejects_empty_or_unsafe_project_names(self):
        for bad in ("", "   ", '"abc"', "abc(def)", "abc,def", "a\nb"):
            with self.assertRaises(ValueError, msg=f"should reject {bad!r}"):
                run_script(bad)
        with self.assertRaises(TypeError):
            run_script(None)  # type: ignore[arg-type]

    def test_runscript_lifecycle_commands_match_manual_verbs(self):
        # Reference: 4-axis manual sections "StopScript / PauseScript /
        # ContinueScript (Immediate command)".  Vendor Python SDK uses the
        # generic Stop / Pause / Continue which also affect RunScript per
        # its docstring; we expose the manual-named verbs explicitly so
        # callers can distinguish project-lifecycle from motion-queue
        # lifecycle.
        self.assertEqual(stop_script().render(), "StopScript()")
        self.assertEqual(pause_script().render(), "PauseScript()")
        self.assertEqual(continue_script().render(), "ContinueScript()")

    def test_run_script_rejects_backslash_to_avoid_quote_escape_confusion(self):
        # Project name is wrapped in double quotes per the 4-axis manual
        # example RunScript("demo"); a stray backslash could be interpreted
        # as a quote-escape on the controller side and shift the parser's
        # quote boundary, so reject it at the builder.
        with self.assertRaises(ValueError):
            run_script("demo\\")
        with self.assertRaises(ValueError):
            run_script("a\\b")

    def test_runscript_builders_re_exported_from_package_root(self):
        # Once a vendor verb is documented in the 4-axis PDF and shipped
        # via dashboard.py, downstream adapter / runtime code should be
        # able to grab it from the package root the same way the existing
        # builders are surfaced.
        from mg400_protocol import (
            continue_script as continue_script_root,
            pause_script as pause_script_root,
            run_script as run_script_root,
            stop_script as stop_script_root,
        )

        self.assertEqual(run_script_root("demo").render(), 'RunScript("demo")')
        self.assertEqual(stop_script_root().render(), "StopScript()")
        self.assertEqual(pause_script_root().render(), "PauseScript()")
        self.assertEqual(continue_script_root().render(), "ContinueScript()")

    def test_alarm_catalog_loads_vendor_alarm_json(self):
        catalog = AlarmCatalog.from_files(
            REPO_ROOT / "Dobot_TCP_IP_Python_V4/files/alarmController.json",
            REPO_ROOT / "Dobot_TCP_IP_Python_V4/files/alarmServo.json",
        )

        shoulder_singularity = catalog.lookup(16)
        servo_current = catalog.lookup(8752)

        self.assertGreater(len(catalog), 300)
        self.assertEqual(shoulder_singularity.source, "controller")
        self.assertIn("singularity", shoulder_singularity.description.lower())
        self.assertEqual(servo_current.source, "servo")
        self.assertIn("overcurrent", servo_current.description.lower())

    # ── Mixed-primitive motion command builders ──────────────────────

    def test_arc_renders_manual_example_format(self):
        # Reference: 4-axis TCP/IP Guide, Arc section
        # Arc(-350,-200,200,150,-300,-250,200,150)
        cmd = arc(
            through_xyzr=(-350.0, -200.0, 200.0, 150.0),
            target_xyzr=(-300.0, -250.0, 200.0, 150.0),
        )
        self.assertEqual(
            cmd.render(),
            "Arc(-350.0000,-200.0000,200.0000,150.0000,"
            "-300.0000,-250.0000,200.0000,150.0000)",
        )

    def test_arc_with_speed_and_cp(self):
        cmd = arc(
            through_xyzr=(100.0, 200.0, 50.0, 0.0),
            target_xyzr=(150.0, 250.0, 50.0, 0.0),
            speed_l=60,
            acc_l=80,
            cp=50,
        )
        rendered = cmd.render()
        self.assertIn("SpeedL=60", rendered)
        self.assertIn("AccL=80", rendered)
        self.assertIn("CP=50", rendered)

    def test_arc_rejects_wrong_point_count(self):
        with self.assertRaises(ValueError):
            arc(through_xyzr=(1, 2, 3), target_xyzr=(4, 5, 6, 7))
        with self.assertRaises(ValueError):
            arc(through_xyzr=(1, 2, 3, 4), target_xyzr=(5, 6, 7))

    def test_circle_renders_manual_example_format(self):
        # Reference: Circle(1,{-350,-200,200,150},{-300,-250,200,150})
        cmd = circle(
            count=1,
            p1_xyzr=(-350.0, -200.0, 200.0, 150.0),
            p2_xyzr=(-300.0, -250.0, 200.0, 150.0),
        )
        self.assertEqual(
            cmd.render(),
            "Circle(1,{-350.0000,-200.0000,200.0000,150.0000},"
            "{-300.0000,-250.0000,200.0000,150.0000})",
        )

    def test_circle_rejects_zero_count(self):
        with self.assertRaises(ValueError):
            circle(0, (1, 2, 3, 4), (5, 6, 7, 8))

    def test_mov_l_io_renders_with_io_trigger(self):
        # Reference: MovLIO(-100,100,200,150,{0,50,1,0})
        cmd = mov_l_io(
            target_xyzr=(-100.0, 100.0, 200.0, 150.0),
            io_triggers=[(0, 50, 1, 0)],
        )
        self.assertEqual(
            cmd.render(),
            "MovLIO(-100.0000,100.0000,200.0000,150.0000,{0,50,1,0})",
        )

    def test_mov_l_io_multiple_triggers(self):
        cmd = mov_l_io(
            target_xyzr=(100.0, 200.0, 50.0, 0.0),
            io_triggers=[(0, 50, 1, 0), (1, 10, 2, 1)],
            speed_l=70,
        )
        rendered = cmd.render()
        self.assertIn("{0,50,1,0}", rendered)
        self.assertIn("{1,10,2,1}", rendered)
        self.assertIn("SpeedL=70", rendered)

    def test_mov_l_io_rejects_bad_trigger_shape(self):
        with self.assertRaises(ValueError):
            mov_l_io(
                target_xyzr=(1, 2, 3, 4),
                io_triggers=[(0, 50, 1)],  # missing status
            )

    def test_mov_l_cartesian_uses_speed_l_not_speed_j(self):
        cmd = mov_l_cartesian(
            target_xyzr=(100.0, 200.0, 50.0, 0.0),
            speed_l=60,
            acc_l=80,
            cp=50,
        )
        rendered = cmd.render()
        self.assertIn("SpeedL=60", rendered)
        self.assertIn("AccL=80", rendered)
        self.assertNotIn("SpeedJ", rendered)

    def test_new_builders_re_exported_from_package_root(self):
        from mg400_protocol import (
            arc as arc_root,
            circle as circle_root,
            mov_l_cartesian as mov_l_cartesian_root,
            mov_l_io as mov_l_io_root,
        )
        self.assertEqual(
            arc_root((-1, 0, 0, 0), (1, 0, 0, 0)).name,
            "Arc",
        )
        self.assertEqual(circle_root(1, (0, 0, 0, 0), (1, 0, 0, 0)).name, "Circle")
        self.assertEqual(mov_l_cartesian_root((0, 0, 0, 0)).name, "MovL")
        self.assertEqual(
            mov_l_io_root((0, 0, 0, 0), [(0, 50, 1, 0)]).name,
            "MovLIO",
        )


if __name__ == "__main__":
    unittest.main()
