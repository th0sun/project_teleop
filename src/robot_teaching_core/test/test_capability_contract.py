"""KinematicsContract + RobotCapabilityProfile validation tests.

Pins down the v0.1 invariants from proposal §4.4 so the next worker
cannot quietly weaken them.
"""

from __future__ import annotations

import unittest

from teaching_core.calibration.feedback import WorkspaceKind, WorkspaceModel
from teaching_core.capability.kinematics import (
    KinematicsContract,
    KinematicsContractError,
    KinematicsKind,
    validate_contract,
)
from teaching_core.capability.profile import (
    ExecutionSupport,
    MotionSupport,
    OrientationAuthority,
    ProfileValidationError,
    RobotCapabilityProfile,
    ToolModel,
    validate_profile,
)


def _full_exec() -> ExecutionSupport:
    return ExecutionSupport(
        offline_program=False,
        queued=True,
        supervised_playback=False,
        trajectory_action=False,
        streaming=True,
    )


def _offline_only_exec() -> ExecutionSupport:
    return ExecutionSupport(
        offline_program=True,
        queued=False,
        supervised_playback=False,
        trajectory_action=False,
        streaming=False,
    )


def _motion() -> MotionSupport:
    return MotionSupport(
        joint=True,
        linear=True,
        circular=False,
        spline=False,
        servo_stream=True,
        blending="cp_percent",
    )


def _tool() -> ToolModel:
    return ToolModel(kind="vacuum", io_port=1)


def _box_workspace() -> WorkspaceModel:
    return WorkspaceModel(
        kind=WorkspaceKind.BOX,
        frame="mg400_base",
        margin_m=0.02,
        source="vendor_spec",
        payload={"min_m": [-0.4, -0.4, 0.0], "max_m": [0.4, 0.4, 0.45]},
    )


def _delta_workspace() -> WorkspaceModel:
    return WorkspaceModel(
        kind=WorkspaceKind.ANALYTIC_DELTA,
        frame="delta_base",
        margin_m=0.01,
        source="analytic_model",
        payload={"radius_m": 0.35, "z_min_m": -0.55, "z_max_m": -0.10},
    )


def _mg400_profile(
    *,
    kinematics: KinematicsContract,
    execution: ExecutionSupport,
    orientation_authority: OrientationAuthority = OrientationAuthority.YAW_ONLY_SCARA,
    orientation_axis_mask=None,
) -> RobotCapabilityProfile:
    return RobotCapabilityProfile(
        robot_id="mg400",
        dof=4,
        joint_names=("J1", "J2", "J3", "J4"),
        joint_limits_rad=((-3.14, 3.14),) * 4,
        kinematics=kinematics,
        ros2_control_command_interfaces=("position",),
        ros2_control_state_interfaces=("position", "velocity"),
        base_frame="mg400_base",
        tcp_frame="mg400_tcp",
        workspace=_box_workspace(),
        motion=_motion(),
        execution=execution,
        tool=_tool(),
        orientation_authority=orientation_authority,
        orientation_axis_mask=orientation_axis_mask,
        max_tcp_speed_m_s=1.0,
        max_joint_speed_rad_s=(3.0,) * 4,
        timing_contract="queued",
    )


class TestKinematicsContract(unittest.TestCase):

    def test_named_requires_provider_id(self) -> None:
        c = KinematicsContract(kind=KinematicsKind.NAMED)
        with self.assertRaises(KinematicsContractError):
            validate_contract(c)

    def test_named_with_provider_id_validates(self) -> None:
        c = KinematicsContract(
            kind=KinematicsKind.NAMED, provider_id="mg400_4axis_fk"
        )
        validate_contract(c)  # must not raise

    def test_urdf_requires_path(self) -> None:
        c = KinematicsContract(kind=KinematicsKind.URDF)
        with self.assertRaises(KinematicsContractError):
            validate_contract(c)

    def test_urdf_with_path_validates(self) -> None:
        c = KinematicsContract(kind=KinematicsKind.URDF, urdf_path="/tmp/x.urdf")
        validate_contract(c)

    def test_named_must_not_carry_urdf_path(self) -> None:
        c = KinematicsContract(
            kind=KinematicsKind.NAMED,
            provider_id="x",
            urdf_path="/tmp/x.urdf",
        )
        with self.assertRaises(KinematicsContractError):
            validate_contract(c)

    def test_external_requires_provider_id(self) -> None:
        c = KinematicsContract(kind=KinematicsKind.EXTERNAL)
        with self.assertRaises(KinematicsContractError):
            validate_contract(c)

    def test_none_must_be_empty(self) -> None:
        c = KinematicsContract(kind=KinematicsKind.NONE, provider_id="x")
        with self.assertRaises(KinematicsContractError):
            validate_contract(c)

    def test_none_clean_validates(self) -> None:
        validate_contract(KinematicsContract(kind=KinematicsKind.NONE))


class TestProfileValidation(unittest.TestCase):

    def test_minimal_mg400_profile_validates(self) -> None:
        profile = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="mg400_4axis_fk"
            ),
            execution=_full_exec(),
        )
        validate_profile(profile)

    def test_joint_count_must_match_dof(self) -> None:
        profile = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="mg400_4axis_fk"
            ),
            execution=_full_exec(),
        )
        bad = RobotCapabilityProfile(
            robot_id=profile.robot_id,
            dof=4,
            joint_names=("J1", "J2", "J3"),  # only 3
            joint_limits_rad=profile.joint_limits_rad,
            kinematics=profile.kinematics,
            ros2_control_command_interfaces=profile.ros2_control_command_interfaces,
            ros2_control_state_interfaces=profile.ros2_control_state_interfaces,
            base_frame=profile.base_frame,
            tcp_frame=profile.tcp_frame,
            workspace=profile.workspace,
            motion=profile.motion,
            execution=profile.execution,
            tool=profile.tool,
            orientation_authority=profile.orientation_authority,
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)

    def test_unknown_command_interface_rejected(self) -> None:
        profile = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="x"
            ),
            execution=_full_exec(),
        )
        bad = RobotCapabilityProfile(
            **{**profile.__dict__,
               "ros2_control_command_interfaces": ("torque",)}
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)

    def test_kinematics_none_only_for_offline_only(self) -> None:
        # Mixing NONE with non-offline modes must fail.
        bad = _mg400_profile(
            kinematics=KinematicsContract(kind=KinematicsKind.NONE),
            execution=_full_exec(),  # has queued + streaming
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)

    def test_kinematics_none_with_offline_only_validates(self) -> None:
        ok = _mg400_profile(
            kinematics=KinematicsContract(kind=KinematicsKind.NONE),
            execution=_offline_only_exec(),
        )
        validate_profile(ok)

    def test_custom_orientation_requires_axis_mask(self) -> None:
        bad = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="x"
            ),
            execution=_full_exec(),
            orientation_authority=OrientationAuthority.CUSTOM,
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)

    def test_axis_mask_only_with_custom(self) -> None:
        bad = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="x"
            ),
            execution=_full_exec(),
            orientation_authority=OrientationAuthority.YAW_ONLY_SCARA,
            orientation_axis_mask=(False, False, True),
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)

    def test_at_least_one_execution_mode_required(self) -> None:
        empty_exec = ExecutionSupport(
            offline_program=False,
            queued=False,
            supervised_playback=False,
            trajectory_action=False,
            streaming=False,
        )
        bad = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="x"
            ),
            execution=empty_exec,
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)

    def test_delta_translation_only_workspace_validates(self) -> None:
        profile = RobotCapabilityProfile(
            robot_id="delta_fake",
            dof=3,
            joint_names=("A", "B", "C"),
            joint_limits_rad=((-1.0, 1.0),) * 3,
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="delta_fake_fk"
            ),
            ros2_control_command_interfaces=("position",),
            ros2_control_state_interfaces=("position",),
            base_frame="delta_base",
            tcp_frame="delta_tcp",
            workspace=_delta_workspace(),
            motion=MotionSupport(
                joint=True,
                linear=True,
                circular=False,
                spline=False,
                servo_stream=False,
                blending="none",
            ),
            execution=ExecutionSupport(
                offline_program=False,
                queued=False,
                supervised_playback=True,
                trajectory_action=False,
                streaming=False,
            ),
            tool=ToolModel(kind="gripper_binary"),
            orientation_authority=OrientationAuthority.TRANSLATION_ONLY_DELTA,
            max_tcp_speed_m_s=2.0,
            max_joint_speed_rad_s=(4.0, 4.0, 4.0),
            timing_contract="best_effort",
        )
        validate_profile(profile)

    def test_delta_workspace_requires_delta_orientation_authority(self) -> None:
        bad = _mg400_profile(
            kinematics=KinematicsContract(
                kind=KinematicsKind.NAMED, provider_id="mg400_4axis_fk"
            ),
            execution=_full_exec(),
        )
        bad = RobotCapabilityProfile(
            **{
                **bad.__dict__,
                "workspace": _delta_workspace(),
                "orientation_authority": OrientationAuthority.YAW_ONLY_SCARA,
            }
        )
        with self.assertRaises(ProfileValidationError):
            validate_profile(bad)


if __name__ == "__main__":
    unittest.main()
