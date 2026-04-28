"""MG400 vendor-protocol onboarding artifacts.

This package is the MG400 case-study answer to: "what information does
the system need when adding a robot?"  It keeps Dobot command strings,
feedback layout, and alarm-code data behind adapter-local APIs instead
of scattering vendor literals through translator/executor code.
"""

from mg400_protocol.alarms import AlarmCatalog, AlarmInfo  # noqa: F401
from mg400_protocol.commands import (  # noqa: F401
    DobotCommand,
    arc,
    circle,
    do_execute,
    joint_mov_j,
    mov_j,
    mov_l,
    mov_l_cartesian,
    mov_l_io,
)
from mg400_protocol.dashboard import (  # noqa: F401
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
from mg400_protocol.feedback import (  # noqa: F401
    FEEDBACK_PACKET_SIZE,
    FEEDBACK_TEST_VALUE,
    MG400_FEEDBACK_DTYPE,
    FeedbackSnapshot,
    parse_feedback_packet,
)
