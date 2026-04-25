"""Dashboard-port (29999) command builders for MG400.

Dashboard commands are vendor controller-state commands (enable,
disable, pause, error reset, speed scaling, etc.) sent over a
separate TCP socket from motion commands.

Owning these here keeps Dobot dashboard literals out of the controller
runtime so the adapter remains the single MG400 command surface.
"""

from __future__ import annotations

from mg400_protocol.commands import DobotCommand, _speed


# ---- Robot state ----------------------------------------------------


def enable_robot() -> DobotCommand:
    """Build ``EnableRobot()``."""
    return DobotCommand("EnableRobot")


def disable_robot() -> DobotCommand:
    """Build ``DisableRobot()``."""
    return DobotCommand("DisableRobot")


def clear_error() -> DobotCommand:
    """Build ``ClearError()``."""
    return DobotCommand("ClearError")


def reset_robot() -> DobotCommand:
    """Build ``ResetRobot()`` (flush motion queue + restore enabled state)."""
    return DobotCommand("ResetRobot")


def emergency_stop() -> DobotCommand:
    """Build ``EmergencyStop()``."""
    return DobotCommand("EmergencyStop")


def pause() -> DobotCommand:
    """Build ``Pause()``."""
    return DobotCommand("Pause")


def continue_() -> DobotCommand:
    """Build ``Continue()``. Trailing underscore avoids the Python keyword."""
    return DobotCommand("Continue")


# ---- Speed scaling --------------------------------------------------


def speed_factor(percent: float) -> DobotCommand:
    """Build ``SpeedFactor(P)`` with ``P`` clamped to [1, 100]."""
    return DobotCommand("SpeedFactor", (_speed(percent),))


def speed_j(percent: float) -> DobotCommand:
    """Build ``SpeedJ(P)`` (joint speed scaling) with clamp to [1, 100]."""
    return DobotCommand("SpeedJ", (_speed(percent),))


def acc_j(percent: float) -> DobotCommand:
    """Build ``AccJ(P)`` (joint acc scaling) with clamp to [1, 100]."""
    return DobotCommand("AccJ", (_speed(percent),))


# ---- Status / introspection -----------------------------------------


def get_tool() -> DobotCommand:
    """Build ``GetTool()`` — returns currently selected tool index."""
    return DobotCommand("GetTool")


def get_pose() -> DobotCommand:
    """Build ``GetPose()`` — returns current TCP Cartesian pose."""
    return DobotCommand("GetPose")
