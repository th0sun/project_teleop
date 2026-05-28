"""RobotCapabilityProfile + supporting structs (proposal §4.4).

The profile is the *adapter's view* of what the robot can do. Core
reads it to decide what it can ask for; it never mutates the profile
at runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal, Optional, Tuple

from teaching_core.calibration.feedback import (
    WorkspaceKind,
    WorkspaceModel,
    WorkspaceModelValidationError,
    validate_workspace_model,
)
from teaching_core.capability.kinematics import (
    KinematicsContract,
    KinematicsContractError,
    KinematicsKind,
    validate_contract,
)


class OrientationAuthority(str, Enum):
    FULL_6DOF = "full_6dof"
    YAW_ONLY_SCARA = "yaw_only_scara"
    TRANSLATION_ONLY_DELTA = "translation_only_delta"
    PLANAR_XY_RZ = "planar_xy_rz"
    CUSTOM = "custom"


@dataclass(frozen=True)
class MotionSupport:
    joint: bool
    linear: bool
    circular: bool
    spline: bool
    servo_stream: bool
    blending: Literal["none", "radius", "cp_percent"]


@dataclass(frozen=True)
class ExecutionSupport:
    offline_program: bool
    queued: bool
    supervised_playback: bool
    trajectory_action: bool
    streaming: bool

    def any(self) -> bool:
        return any(
            (self.offline_program, self.queued, self.supervised_playback,
             self.trajectory_action, self.streaming)
        )


@dataclass(frozen=True)
class ToolModel:
    kind: Literal["none", "vacuum", "gripper_binary", "gripper_analog"]
    io_port: Optional[int] = None


class ProfileValidationError(ValueError):
    """Raised when a RobotCapabilityProfile is internally inconsistent."""


@dataclass(frozen=True)
class RobotCapabilityProfile:
    robot_id: str
    dof: int
    joint_names: Tuple[str, ...]
    joint_limits_rad: Tuple[Tuple[float, float], ...]

    kinematics: KinematicsContract
    ros2_control_command_interfaces: Tuple[str, ...]
    ros2_control_state_interfaces: Tuple[str, ...]

    base_frame: str
    tcp_frame: str
    workspace: WorkspaceModel

    motion: MotionSupport
    execution: ExecutionSupport
    tool: ToolModel

    orientation_authority: OrientationAuthority
    orientation_axis_mask: Optional[Tuple[bool, bool, bool]] = None  # only if CUSTOM

    max_tcp_speed_m_s: float = 0.0
    max_joint_speed_rad_s: Tuple[float, ...] = ()
    timing_contract: Literal["queued", "hard_realtime", "best_effort"] = "best_effort"


_VALID_INTERFACES = {"position", "velocity", "effort"}


def validate_profile(p: RobotCapabilityProfile) -> None:
    """Run v0.1 capability-profile invariants. Raise on violation.

    The invariants are grouped by concern so each helper can be
    invoked independently from a notebook / a custom builder when
    only one section needs to be re-checked after an edit:

      1. identity / shape — robot_id, dof, joint name + limit shapes
      2. frames           — base_frame, tcp_frame non-empty
      3. workspace        — geometry + ANALYTIC_DELTA orientation rule
      4. control surfaces — ros2_control command + state interfaces
      5. speed limits     — joint + TCP speed bounds
      6. orientation      — authority vs orientation_axis_mask
      7. kinematics       — contract + NONE-only-when-offline rule
      8. execution        — must enable at least one mode
    """
    _validate_identity_and_shape(p)
    _validate_frames(p)
    _validate_workspace(p)
    _validate_control_interfaces(p)
    _validate_speed_limits(p)
    _validate_orientation_authority(p)
    _validate_kinematics(p)
    if not p.execution.any():
        raise ProfileValidationError(
            "ExecutionSupport must enable at least one mode"
        )


def _validate_identity_and_shape(p: RobotCapabilityProfile) -> None:
    """robot_id non-empty + DoF >= 1 + joint_names / joint_limits_rad
    lengths match DoF + each (lo, hi) limit is non-degenerate."""
    if not p.robot_id:
        raise ProfileValidationError("robot_id must be non-empty")
    if p.dof < 1:
        raise ProfileValidationError(f"dof must be >= 1, got {p.dof}")
    if len(p.joint_names) != p.dof:
        raise ProfileValidationError(
            f"joint_names length ({len(p.joint_names)}) != dof ({p.dof})"
        )
    if len(p.joint_limits_rad) != p.dof:
        raise ProfileValidationError(
            f"joint_limits_rad length != dof ({p.dof})"
        )
    for i, (lo, hi) in enumerate(p.joint_limits_rad):
        if lo >= hi:
            raise ProfileValidationError(
                f"joint_limits_rad[{i}]: low {lo} must be < high {hi}"
            )


def _validate_frames(p: RobotCapabilityProfile) -> None:
    """base_frame + tcp_frame must be non-empty so downstream URDF /
    TF consumers can resolve poses without guessing.
    """
    if not p.base_frame:
        raise ProfileValidationError("base_frame must be non-empty")
    if not p.tcp_frame:
        raise ProfileValidationError("tcp_frame must be non-empty")


def _validate_workspace(p: RobotCapabilityProfile) -> None:
    """Delegate to ``validate_workspace_model`` and then enforce the
    ANALYTIC_DELTA -> TRANSLATION_ONLY_DELTA orientation rule. Any
    WorkspaceModelValidationError is wrapped as a ProfileValidationError
    so callers can catch a single exception type.
    """
    try:
        validate_workspace_model(p.workspace)
    except WorkspaceModelValidationError as exc:
        raise ProfileValidationError(f"workspace: {exc}") from exc
    if p.workspace.kind == WorkspaceKind.ANALYTIC_DELTA and (
        p.orientation_authority != OrientationAuthority.TRANSLATION_ONLY_DELTA
    ):
        raise ProfileValidationError(
            "workspace.kind=ANALYTIC_DELTA requires "
            "orientation_authority=TRANSLATION_ONLY_DELTA"
        )


def _validate_control_interfaces(p: RobotCapabilityProfile) -> None:
    """ros2_control command + state interface names must all be in
    ``_VALID_INTERFACES``. Same allow-list for both directions.
    """
    for label, ifaces in (
        ("command", p.ros2_control_command_interfaces),
        ("state", p.ros2_control_state_interfaces),
    ):
        for iface in ifaces:
            if iface not in _VALID_INTERFACES:
                raise ProfileValidationError(
                    f"unknown {label} interface {iface!r}; "
                    f"expected one of {_VALID_INTERFACES}"
                )


def _validate_speed_limits(p: RobotCapabilityProfile) -> None:
    """max_joint_speed_rad_s length must match DoF and every entry
    must be > 0. max_tcp_speed_m_s must also be > 0.
    """
    if len(p.max_joint_speed_rad_s) != p.dof:
        raise ProfileValidationError(
            f"max_joint_speed_rad_s length ({len(p.max_joint_speed_rad_s)}) "
            f"!= dof ({p.dof})"
        )
    if any(speed <= 0.0 for speed in p.max_joint_speed_rad_s):
        raise ProfileValidationError("max_joint_speed_rad_s values must be > 0")
    if p.max_tcp_speed_m_s <= 0.0:
        raise ProfileValidationError("max_tcp_speed_m_s must be > 0")


def _validate_orientation_authority(p: RobotCapabilityProfile) -> None:
    """orientation_axis_mask is a (rx, ry, rz) bool triple that is
    required if and only if orientation_authority == CUSTOM.
    """
    if p.orientation_authority == OrientationAuthority.CUSTOM:
        if p.orientation_axis_mask is None or len(p.orientation_axis_mask) != 3:
            raise ProfileValidationError(
                "orientation_authority=CUSTOM requires "
                "orientation_axis_mask = (rx, ry, rz) of bools"
            )
    elif p.orientation_axis_mask is not None:
        raise ProfileValidationError(
            "orientation_axis_mask is only valid with "
            "orientation_authority=CUSTOM"
        )


def _validate_kinematics(p: RobotCapabilityProfile) -> None:
    """Delegate to ``validate_contract`` for kinematics self-consistency
    then enforce the NONE-kinematics-implies-offline-only rule.
    Wrapping KinematicsContractError preserves the single-exception
    contract callers expect from validate_profile.
    """
    try:
        validate_contract(p.kinematics)
    except KinematicsContractError as exc:
        raise ProfileValidationError(f"kinematics: {exc}") from exc
    if p.kinematics.kind == KinematicsKind.NONE:
        e = p.execution
        if not e.offline_program or any(
            (e.queued, e.supervised_playback, e.trajectory_action, e.streaming)
        ):
            raise ProfileValidationError(
                "kinematics.kind=NONE is only valid when execution supports "
                "exclusively offline_program (no other modes)"
            )
