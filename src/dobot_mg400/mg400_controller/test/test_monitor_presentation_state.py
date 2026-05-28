import types
import unittest

from mg400_controller.common.utils.monitor.presentation_state import (
    build_cartesian_display_state,
    build_joint_display_rows,
    build_status_display_state,
)


class FakeDecoder:
    def decode_error(self, error):
        return ("Decoded Alarm", None, None)


class MonitorPresentationStateTest(unittest.TestCase):
    def test_build_joint_display_rows_formats_values_and_total_diff(self):
        rows, total_diff = build_joint_display_rows(
            [0.0, 1.0, 2.0, 3.0],
            [0.1, 2.5, 1.0, 3.0],
        )

        self.assertEqual(rows[0]["diff"], "+0.10")
        self.assertEqual(rows[0]["color"], "green")
        self.assertEqual(rows[1]["color"], "orange")
        self.assertEqual(rows[2]["color"], "orange")
        self.assertAlmostEqual(total_diff, 2.6)

    def test_build_cartesian_display_state_computes_tool_delta(self):
        telemetry = types.SimpleNamespace(
            latest_unity_xyz=[10.0, 20.0, 30.0, 0.0, 0.0, 0.0],
            latest_flange_actual=[1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            latest_tool_actual=[4.0, 6.0, 8.0, 0.0, 0.0, 0.0],
            latest_tool_index=2,
        )

        state = build_cartesian_display_state(telemetry)

        self.assertEqual(state["tool_delta"], [3.0, 4.0, 5.0])
        self.assertEqual(state["tool_index_text"], "Tool 2")

    def test_build_status_display_state_formats_mode_error_and_latency(self):
        telemetry = types.SimpleNamespace(
            latest_robot_mode=7,
            latest_error_status=5,
            latest_do_status=3,
            last_target_time=99.8,
            last_actual_time=99.9,
        )

        state = build_status_display_state(telemetry, FakeDecoder(), now=100.0)

        self.assertEqual(state["mode_text"], "🤖 MODE: RUN")
        self.assertEqual(state["error_text"], "❌ ERR 05: Decoded Alarm")
        self.assertEqual(state["error_color"], "red")
        self.assertEqual(state["latency_text"], "Cmd Age: 200ms | Feed Age: 100ms")
        self.assertEqual(state["do_hex_text"], "DO: 0x0003 | Bits: 0b11")


if __name__ == "__main__":
    unittest.main()
