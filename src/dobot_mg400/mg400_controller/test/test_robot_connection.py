import unittest
import socket

from mg400_controller.common.core.robot_connection import RobotConnection
from mg400_controller.common.config.network_config import SOCKET_TIMEOUT


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
    def __init__(self, responses=None, stale_responses=None, fail_send=False):
        self.sent = []
        self.responses = []
        self.responses_after_send = list(responses or [])
        self.stale_responses = list(stale_responses or [])
        self.fail_send = fail_send
        self.timeouts = []
        self.current_timeout = None

    def send(self, payload):
        if self.fail_send:
            raise OSError("send failed")
        self.sent.append(payload.decode())
        self.responses.extend(self.responses_after_send)
        self.responses_after_send = []
        return len(payload)

    def recv(self, _size):
        if self.current_timeout == 0.0 and self.stale_responses:
            response = self.stale_responses.pop(0)
            return response.encode() if isinstance(response, str) else response
        if self.current_timeout == 0.0:
            raise socket.timeout()
        if not self.responses:
            raise socket.timeout()
        response = self.responses.pop(0)
        return response.encode() if isinstance(response, str) else response

    def settimeout(self, timeout):
        self.current_timeout = timeout
        self.timeouts.append(timeout)


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

    def test_send_motion_cmd_with_response_captures_immediate_response(self):
        logger = FakeLogger()
        conn = RobotConnection(logger)
        conn.connected = True
        conn.cmd_sock = FakeSocket(responses=["0,{321},JointMovJ(...);"])

        success, response = conn.send_motion_cmd_with_response(
            "JointMovJ(1,2,3,4)",
            response_timeout=0.02,
        )

        self.assertTrue(success)
        self.assertEqual(response, "0,{321},JointMovJ(...);")
        self.assertEqual(conn.cmd_sock.sent, ["JointMovJ(1,2,3,4)\n"])
        self.assertEqual(conn.cmd_sock.timeouts, [0.0, SOCKET_TIMEOUT, 0.02, SOCKET_TIMEOUT])

    def test_send_motion_cmd_with_response_drops_stale_response_before_send(self):
        logger = FakeLogger()
        conn = RobotConnection(logger)
        conn.connected = True
        conn.cmd_sock = FakeSocket(
            stale_responses=["0,{111},JointMovJ(old);"],
            responses=["0,{222},JointMovJ(new);"],
        )

        success, response = conn.send_motion_cmd_with_response(
            "JointMovJ(1,2,3,4)",
            response_timeout=0.02,
        )

        self.assertTrue(success)
        self.assertEqual(response, "0,{222},JointMovJ(new);")
        self.assertEqual(conn.cmd_sock.stale_responses, [])
        self.assertEqual(conn.cmd_sock.sent, ["JointMovJ(1,2,3,4)\n"])

    def test_send_motion_cmd_with_response_keeps_success_on_ack_timeout(self):
        logger = FakeLogger()
        conn = RobotConnection(logger)
        conn.connected = True
        conn.cmd_sock = FakeSocket()

        success, response = conn.send_motion_cmd_with_response(
            "JointMovJ(1,2,3,4)",
            response_timeout=0.01,
        )

        self.assertTrue(success)
        self.assertIsNone(response)
        self.assertEqual(conn.cmd_sock.sent, ["JointMovJ(1,2,3,4)\n"])

    def test_send_motion_cmd_preserves_legacy_bool_contract(self):
        logger = FakeLogger()
        conn = RobotConnection(logger)
        conn.connected = True
        conn.cmd_sock = FakeSocket(responses=["0,{321},JointMovJ(...);"])

        self.assertTrue(conn.send_motion_cmd("JointMovJ(1,2,3,4)"))
        self.assertEqual(conn.cmd_sock.sent, ["JointMovJ(1,2,3,4)\n"])
        self.assertEqual(conn.cmd_sock.responses, ["0,{321},JointMovJ(...);"])
        self.assertEqual(conn.cmd_sock.responses_after_send, [])


if __name__ == "__main__":
    unittest.main()
