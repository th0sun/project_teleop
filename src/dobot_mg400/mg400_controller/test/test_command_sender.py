import unittest

from mg400_controller.common.core.command_sender import CommandSender


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakeConnection:
    def __init__(self, *, success=True, response=None):
        self.success = success
        self.response = response
        self.calls = []

    def send_motion_cmd_with_response(self, command, response_timeout=0.02):
        self.calls.append((command, response_timeout))
        return self.success, self.response

    def send_and_wait(self, _command):
        return None


class CommandSenderTest(unittest.TestCase):
    def test_send_with_command_id_parses_robot_ack(self):
        conn = FakeConnection(response="0,{456},JointMovJ(...);")
        sender = CommandSender(conn, feedback_handler=None, logger=FakeLogger())

        result = sender.send_with_command_id(
            "JointMovJ(1,2,3,4)",
            response_timeout=0.03,
        )

        self.assertTrue(result.success)
        self.assertEqual(result.command_id, 456)
        self.assertEqual(result.response, "0,{456},JointMovJ(...);")
        self.assertEqual(conn.calls, [("JointMovJ(1,2,3,4)", 0.03)])

    def test_send_with_command_id_allows_missing_ack(self):
        conn = FakeConnection(response=None)
        sender = CommandSender(conn, feedback_handler=None, logger=FakeLogger())

        result = sender.send_with_command_id("JointMovJ(1,2,3,4)")

        self.assertTrue(result.success)
        self.assertIsNone(result.command_id)
        self.assertIsNone(result.response)

    def test_send_with_command_id_reports_send_failure(self):
        conn = FakeConnection(success=False, response=None)
        logger = FakeLogger()
        sender = CommandSender(conn, feedback_handler=None, logger=logger)

        result = sender.send_with_command_id("JointMovJ(1,2,3,4)")

        self.assertFalse(result.success)
        self.assertIsNone(result.command_id)
        self.assertTrue(
            any(
                level == "error" and "Failed to send motion command" in message
                for level, message in logger.messages
            )
        )


if __name__ == "__main__":
    unittest.main()
