import unittest

from mg400_controller.common.utils.error.error_decoder import RobotErrorDecoder
from mg400_controller.common.utils.error.error_handler import ErrorHandler


class DummyLogger:
    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass


class DummyConnection:
    def __init__(self, response):
        self.response = response

    def send_and_wait(self, command):
        assert command == "GetErrorID()"
        return self.response


class RobotErrorDecoderTest(unittest.TestCase):
    def test_loads_alarm_json_without_rospkg_and_decodes_sdk_entries(self):
        decoder = RobotErrorDecoder()

        desc, cause, solution = decoder.decode_error(18)
        self.assertIn("Inverse kinematics", desc)
        self.assertIn("Reselect", solution)

        desc, cause, solution = decoder.decode_error(8992)
        self.assertIn("over-current", desc)
        self.assertIn("restart controller", solution)

    def test_decodes_collision_placeholder_from_manual(self):
        decoder = RobotErrorDecoder()

        desc, cause, solution = decoder.decode_error(-2)

        self.assertIn("Collision", desc)
        self.assertIn("fixture", solution)

    def test_error_handler_preserves_solution_field(self):
        handler = ErrorHandler(
            DummyConnection("0,{[[18],[],[8992],[],[],[]]},GetErrorID();"),
            DummyLogger(),
        )

        errors = {err["id"]: err for err in handler.check_errors()}

        self.assertIn(18, errors)
        self.assertIn(8992, errors)
        self.assertIn("cause", errors[18])
        self.assertIn("Reselect", errors[18]["solution"])
        self.assertIn("restart controller", errors[8992]["solution"])

    def test_format_error_message_includes_cause_for_unity_alarm_ui(self):
        handler = ErrorHandler(
            DummyConnection("0,{[[18],[],[],[],[],[]]},GetErrorID();"),
            DummyLogger(),
        )

        formatted = handler.format_error_message(handler.check_errors()[0])

        self.assertEqual(formatted["id"], 18)
        self.assertIn("Inverse kinematics", formatted["description"])
        self.assertIn("cause", formatted)


if __name__ == "__main__":
    unittest.main()
