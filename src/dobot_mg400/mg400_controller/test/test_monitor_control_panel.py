import unittest

from mg400_controller.common.config.motion_config import (
    GREEN_LIGHT_DO_PORT,
    VACUUM_DO_PORT,
)
from mg400_controller.common.monitor.control_panel_state import (
    MonitorControlPanelState,
)


class FakeClock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value

    def advance(self, amount):
        self.value += amount


class MonitorControlPanelStateTest(unittest.TestCase):
    def test_toggle_suction_creates_pending_command_and_sync_updates_after_lockout(self):
        clock = FakeClock()
        panel = MonitorControlPanelState(lockout_sec=2.0, time_fn=clock)

        command = panel.toggle_suction()
        self.assertEqual(command.port, VACUUM_DO_PORT)
        self.assertTrue(command.state)
        self.assertEqual(command.display.text, "VACUUM (WAIT)")

        busy_sync = panel.sync_from_do_status(0)
        self.assertIsNone(busy_sync.suction)

        clock.advance(2.1)
        do_status = 1 << (VACUUM_DO_PORT - 1)
        ready_sync = panel.sync_from_do_status(do_status)
        self.assertEqual(ready_sync.suction.text, "VACUUM")
        self.assertEqual(ready_sync.suction.fg, "white")

    def test_toggle_light_uses_named_port_and_recovers_actual_state_after_lockout(self):
        clock = FakeClock()
        panel = MonitorControlPanelState(lockout_sec=1.0, time_fn=clock)

        command = panel.toggle_light("GREEN")
        self.assertEqual(command.port, GREEN_LIGHT_DO_PORT)
        self.assertTrue(command.state)
        self.assertEqual(command.display.text, "GREEN...")

        clock.advance(1.1)
        do_status = 1 << (GREEN_LIGHT_DO_PORT - 1)
        sync = panel.sync_from_do_status(do_status)
        self.assertIn("GREEN", sync.lights)
        self.assertEqual(sync.lights["GREEN"].text, "GREEN")
        self.assertEqual(sync.lights["GREEN"].fg, "white")

    def test_build_initial_sync_exposes_default_off_state(self):
        panel = MonitorControlPanelState()

        sync = panel.build_initial_sync()

        self.assertEqual(sync.suction.text, "OFF")
        self.assertEqual(sync.lights["RED"].text, "RED")
        self.assertEqual(sync.lights["RED"].fg, "black")


if __name__ == "__main__":
    unittest.main()
