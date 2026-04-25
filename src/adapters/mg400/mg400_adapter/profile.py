"""MG400 capability profile."""

from __future__ import annotations

from math import radians

from mg400_controller.common.config import motion_config
from mg400_controller.common.config.robot_config import JOINT_LIMITS
from teaching_core.calibration.feedback import WorkspaceKind, WorkspaceModel
from teaching_core.capability.kinematics import KinematicsContract, KinematicsKind
from teaching_core.capability.profile import (
    ExecutionSupport,
    MotionSupport,
    OrientationAuthority,
    RobotCapabilityProfile,
    ToolModel,
)

MG400_PROVIDER_ID = "mg400_4axis_fk"


def _joint_limits_rad():
    return tuple(
        (radians(JOINT_LIMITS[index][0]), radians(JOINT_LIMITS[index][1]))
        for index in range(4)
    )


def make_mg400_profile() -> RobotCapabilityProfile:
    """Return the conservative v0.1 MG400 adapter capability profile."""
    return RobotCapabilityProfile(
        robot_id="dobot_mg400",
        dof=4,
        joint_names=("J1", "J2", "J3", "J4"),
        joint_limits_rad=_joint_limits_rad(),
        kinematics=KinematicsContract(
            kind=KinematicsKind.NAMED,
            provider_id=MG400_PROVIDER_ID,
            fk_only=True,
        ),
        ros2_control_command_interfaces=("position",),
        ros2_control_state_interfaces=("position", "velocity"),
        base_frame="mg400_base",
        tcp_frame="mg400_tcp",
        workspace=WorkspaceModel(
            kind=WorkspaceKind.BOX,
            frame="mg400_base",
            margin_m=0.02,
            source="repo_conservative_placeholder",
            payload={
                "min_m": [-0.40, -0.40, 0.00],
                "max_m": [0.40, 0.40, 0.45],
            },
        ),
        motion=MotionSupport(
            joint=True,
            linear=False,
            circular=False,
            spline=False,
            servo_stream=False,
            blending="cp_percent",
        ),
        execution=ExecutionSupport(
            offline_program=True,
            queued=True,
            supervised_playback=False,
            trajectory_action=False,
            streaming=False,
        ),
        tool=ToolModel(kind="vacuum", io_port=motion_config.VACUUM_DO_PORT),
        orientation_authority=OrientationAuthority.YAW_ONLY_SCARA,
        max_tcp_speed_m_s=1.0,
        max_joint_speed_rad_s=(3.0, 3.0, 3.0, 6.0),
        timing_contract="queued",
    )
