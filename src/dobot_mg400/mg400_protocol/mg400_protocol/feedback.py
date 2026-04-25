"""MG400 1440-byte feedback layout and parser.

The layout is derived from Dobot_TCP_IP_Python_V4/dobot_api.py and
the 4-axis TCP/IP remote-control manual. Runtime code should read
named fields from this module instead of open-coding offsets.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

FEEDBACK_PACKET_SIZE = 1440
FEEDBACK_TEST_VALUE = 0x0123456789ABCDEF

MG400_FEEDBACK_DTYPE = np.dtype([
    ("len", np.uint16),
    ("reserve", np.byte, (6,)),
    ("DigitalInputs", np.uint64),
    ("DigitalOutputs", np.uint64),
    ("RobotMode", np.uint64),
    ("TimeStamp", np.uint64),
    ("RunTime", np.uint64),
    ("TestValue", np.uint64),
    ("reserve2", np.byte, (8,)),
    ("SpeedScaling", np.float64),
    ("reserve3", np.byte, (16,)),
    ("VRobot", np.float64),
    ("IRobot", np.float64),
    ("ProgramState", np.float64),
    ("SafetyOIn", np.uint16),
    ("SafetyOOut", np.uint16),
    ("reserve4", np.byte, (76,)),
    ("QTarget", np.float64, (6,)),
    ("QDTarget", np.float64, (6,)),
    ("QDDTarget", np.float64, (6,)),
    ("ITarget", np.float64, (6,)),
    ("MTarget", np.float64, (6,)),
    ("QActual", np.float64, (6,)),
    ("QDActual", np.float64, (6,)),
    ("IActual", np.float64, (6,)),
    ("ActualTCPForce", np.float64, (6,)),
    ("ToolVectorActual", np.float64, (6,)),
    ("TCPSpeedActual", np.float64, (6,)),
    ("TCPForce", np.float64, (6,)),
    ("ToolVectorTarget", np.float64, (6,)),
    ("TCPSpeedTarget", np.float64, (6,)),
    ("MotorTemperatures", np.float64, (6,)),
    ("JointModes", np.float64, (6,)),
    ("VActual", np.float64, (6,)),
    ("HandType", np.byte, (4,)),
    ("User", np.byte),
    ("Tool", np.byte),
    ("RunQueuedCmd", np.byte),
    ("PauseCmdFlag", np.byte),
    ("VelocityRatio", np.byte),
    ("AccelerationRatio", np.byte),
    ("reserve5", np.byte),
    ("XYZVelocityRatio", np.byte),
    ("RVelocityRatio", np.byte),
    ("XYZAccelerationRatio", np.byte),
    ("RAccelerationRatio", np.byte),
    ("reserve6", np.byte, (2,)),
    ("BrakeStatus", np.byte),
    ("EnableStatus", np.byte),
    ("DragStatus", np.byte),
    ("RunningStatus", np.byte),
    ("ErrorStatus", np.byte),
    ("JogStatusCR", np.byte),
    ("CRRobotType", np.byte),
    ("DragButtonSignal", np.byte),
    ("EnableButtonSignal", np.byte),
    ("RecordButtonSignal", np.byte),
    ("ReappearButtonSignal", np.byte),
    ("JawButtonSignal", np.byte),
    ("SixForceOnline", np.byte),
    ("CollisionState", np.byte),
    ("ArmApproachState", np.byte),
    ("J4ApproachState", np.byte),
    ("J5ApproachState", np.byte),
    ("J6ApproachState", np.byte),
    ("reserve7", np.byte, (61,)),
    ("VibrationDisZ", np.float64),
    ("CurrentCommandId", np.uint64),
    ("MActual", np.float64, (6,)),
    ("Load", np.float64),
    ("CenterX", np.float64),
    ("CenterY", np.float64),
    ("CenterZ", np.float64),
    ("UserValue", np.float64, (6,)),
    ("ToolValue", np.float64, (6,)),
    ("reserve8", np.byte, (8,)),
    ("SixForceValue", np.float64, (6,)),
    ("TargetQuaternion", np.float64, (4,)),
    ("ActualQuaternion", np.float64, (4,)),
    ("AutoManualMode", np.uint16),
    ("ExportStatus", np.uint16),
    ("SafetyState", np.byte),
    ("reserve9", np.byte, (19,)),
])

if MG400_FEEDBACK_DTYPE.itemsize != FEEDBACK_PACKET_SIZE:
    raise RuntimeError(
        "MG400 feedback dtype must describe exactly "
        f"{FEEDBACK_PACKET_SIZE} bytes, got {MG400_FEEDBACK_DTYPE.itemsize}"
    )


@dataclass(frozen=True)
class FeedbackSnapshot:
    """Robot feedback fields used by the adapter/runtime."""

    q_actual_deg: tuple[float, ...]
    q_target_deg: tuple[float, ...]
    robot_mode: int
    digital_inputs: int
    digital_outputs: int
    speed_scaling: float
    run_queued_cmd: int
    error_status: int
    collision_state: int
    current_command_id: int
    tool_vector_actual: tuple[float, ...]
    tool_vector_target: tuple[float, ...]
    test_value: int


def parse_feedback_packet(
    packet: bytes,
    *,
    allow_mock_zero_test_value: bool = True,
) -> Optional[FeedbackSnapshot]:
    """Parse a Dobot feedback packet.

    Returns ``None`` for invalid packets. The mock can emit ``TestValue=0``;
    real hardware should emit ``FEEDBACK_TEST_VALUE``.
    """
    if len(packet) < FEEDBACK_PACKET_SIZE:
        return None

    record = np.frombuffer(packet[:FEEDBACK_PACKET_SIZE], dtype=MG400_FEEDBACK_DTYPE, count=1)[0]
    test_value = int(record["TestValue"])
    valid_test_values = {FEEDBACK_TEST_VALUE}
    if allow_mock_zero_test_value:
        valid_test_values.add(0)
    if test_value not in valid_test_values:
        return None

    q_actual = tuple(float(value) for value in record["QActual"][:4])
    if test_value == 0 and sum(abs(value) for value in q_actual) < 0.000001:
        return None

    return FeedbackSnapshot(
        q_actual_deg=q_actual,
        q_target_deg=tuple(float(value) for value in record["QTarget"][:4]),
        robot_mode=int(record["RobotMode"]),
        digital_inputs=int(record["DigitalInputs"]),
        digital_outputs=int(record["DigitalOutputs"]),
        speed_scaling=float(record["SpeedScaling"]),
        run_queued_cmd=int(record["RunQueuedCmd"]),
        error_status=int(record["ErrorStatus"]),
        collision_state=int(record["CollisionState"]),
        current_command_id=int(record["CurrentCommandId"]),
        tool_vector_actual=tuple(float(value) for value in record["ToolVectorActual"]),
        tool_vector_target=tuple(float(value) for value in record["ToolVectorTarget"]),
        test_value=test_value,
    )
