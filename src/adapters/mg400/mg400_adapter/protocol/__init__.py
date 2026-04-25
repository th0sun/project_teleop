"""MG400 vendor-protocol onboarding artifacts.

This package is the MG400 case-study answer to: "what information does
the system need when adding a robot?"  It keeps Dobot command strings,
feedback layout, and alarm-code data behind adapter-local APIs instead
of scattering vendor literals through translator/executor code.
"""

from mg400_adapter.protocol.alarms import AlarmCatalog, AlarmInfo  # noqa: F401
from mg400_adapter.protocol.commands import (  # noqa: F401
    DobotCommand,
    do_execute,
    joint_mov_j,
)
from mg400_adapter.protocol.feedback import (  # noqa: F401
    FEEDBACK_PACKET_SIZE,
    FEEDBACK_TEST_VALUE,
    MG400_FEEDBACK_DTYPE,
    FeedbackSnapshot,
    parse_feedback_packet,
)
