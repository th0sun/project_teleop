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


# ---- Controller-side project execution (RunScript) ------------------
#
# Source: Dobot 4-axis "TCP/IP Remote Control Interface Guide" V1.6.0.0
# (docs/reference_manuals/dobot/TCP_IP Remote Control Interface Guide
# (4axis)_20240419_en.pdf), section "RunScript / StopScript / PauseScript /
# ContinueScript (Immediate command)" on the dashboard port (29999).
#
# The manual prescribes the wire form ``RunScript("demo")`` -- the project
# name is enclosed in double quotes -- and documents StopScript / PauseScript
# / ContinueScript as separate dedicated immediate commands.
#
# The vendor Python SDK (``Dobot_TCP_IP_Python_V4/dobot_api.py``) exposes
# only ``RunScript / Stop / Pause / Continue``; its ``Stop / Pause / Continue``
# docstrings explicitly note that those generic commands ALSO operate on a
# running ``RunScript`` project, so callers may use either family.  We keep
# the manual-named builders here because they are documented primary-source
# verbs and let callers be explicit about project lifecycle vs motion-queue
# lifecycle.
#
# This adds the protocol surface only.  No deployer / Wireshark / SSH upload
# of the project file itself is implemented in this change -- that gap is
# tracked in architecture_phase_research_log.md.


_RUN_SCRIPT_FORBIDDEN = ('"', "\\", "(", ")", ",", "\n", "\r", "\t", "\0")


def _run_script_project_name(project_name: str) -> str:
    """Validate and quote a project name for ``RunScript("...")``.

    The manual example uses double quotes; we forbid characters that would
    break that wire form so a malformed name cannot inject a second command
    or unbalance the parser on the controller side.
    """
    if not isinstance(project_name, str):
        raise TypeError("project_name must be a string")
    if not project_name.strip():
        raise ValueError("project_name must be a non-empty string")
    for ch in _RUN_SCRIPT_FORBIDDEN:
        if ch in project_name:
            raise ValueError(
                f"project_name contains forbidden character {ch!r} for RunScript"
            )
    return f'"{project_name}"'


def run_script(project_name: str) -> DobotCommand:
    """Build ``RunScript("<project>")`` (dashboard port 29999).

    Asks the MG400 controller to start running an on-controller project
    (DobotStudio Pro Blockly graph or Lua script) by name.  This is the
    primary "controller-side execution" entry point: the host issues one
    short command and the controller plays the project itself, instead of
    streaming per-waypoint motion commands over TCP at runtime.

    The wire format follows the manual example (``RunScript("demo")``):
    ``project_name`` is wrapped in double quotes and validated to reject
    characters that would break the vendor parser.
    """
    return DobotCommand("RunScript", (_run_script_project_name(project_name),))


def stop_script() -> DobotCommand:
    """Build ``StopScript()`` -- abort a running on-controller project.

    Documented in the 4-axis TCP/IP manual as a dedicated immediate command
    independent of the generic motion-queue ``Stop()``.  Per the vendor
    Python SDK, the generic ``Stop()`` (already exposed elsewhere) also
    halts a running RunScript project; prefer this builder when the intent
    is specifically to stop a project rather than the motion queue.
    """
    return DobotCommand("StopScript")


def pause_script() -> DobotCommand:
    """Build ``PauseScript()`` -- pause a running on-controller project.

    Documented separately from generic ``Pause()`` in the 4-axis manual.
    """
    return DobotCommand("PauseScript")


def continue_script() -> DobotCommand:
    """Build ``ContinueScript()`` -- resume a paused on-controller project.

    Documented separately from generic ``Continue()`` in the 4-axis manual.
    """
    return DobotCommand("ContinueScript")
