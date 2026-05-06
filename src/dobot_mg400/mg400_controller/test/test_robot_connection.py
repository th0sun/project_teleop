import unittest

from mg400_controller.common.core.robot_connection import RobotConnection


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakeSocket:
    def __init__(self):
        self.sent = []

    def send(self, payload):
        self.sent.append(payload.decode())
        return len(payload)


class RobotConnectionTest(unittest.TestCase):
    def test_set_realtime_speed_defaults_restores_dashboard_scalers(self):
        logger = FakeLogger()
        conn = RobotConnection(logger)
        conn.connected = True
        conn.dashboard = FakeSocket()

        self.assertTrue(conn.set_realtime_speed_defaults())

        self.assertEqual(
            conn.dashboard.sent,
            [
                "SpeedFactor(100)\n",
                "AccJ(100)\n",
                "SpeedJ(100)\n",
            ],
        )


if __name__ == "__main__":
    unittest.main()
