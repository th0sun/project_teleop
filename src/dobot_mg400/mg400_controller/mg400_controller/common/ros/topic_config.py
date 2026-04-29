#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS topic profile for the current MG400 teleop stack.

The defaults intentionally preserve the existing Unity/MG400 demo contract.
Deployments can override these names through ROS parameters under `topics.*`
without changing robot-control code.
"""

from dataclasses import dataclass, fields

import mg400_controller.common.config.motion_config as motion_config


TOPIC_PARAMETER_PREFIX = "topics."


@dataclass(frozen=True)
class TeleopTopicConfig:
    ros_ping: str = motion_config.ROS_PING_TOPIC
    joint_states: str = motion_config.RVIZ_TOPIC
    debug: str = motion_config.DEBUG_TOPIC
    safety: str = motion_config.SAFETY_TOPIC
    scene_safety_enabled: str = motion_config.SCENE_SAFETY_ENABLE_TOPIC
    do_status: str = motion_config.DO_STATUS_TOPIC
    robot_mode: str = motion_config.ROBOT_MODE_TOPIC
    error_status: str = motion_config.ERROR_STATUS_TOPIC
    tool_actual: str = motion_config.TOOL_ACTUAL_TOPIC
    tool_target: str = motion_config.TOOL_TARGET_TOPIC
    flange_actual: str = motion_config.FLANGE_ACTUAL_TOPIC
    tool_index: str = motion_config.TOOL_INDEX_TOPIC
    predicted_target: str = motion_config.PREDICTED_TARGET_TOPIC
    sent_command: str = motion_config.SENT_COMMAND_TOPIC
    unity_xyz: str = motion_config.UNITY_XYZ_TOPIC
    playback_unity: str = motion_config.PLAYBACK_UNITY_TOPIC
    traj_preview: str = motion_config.TRAJ_PREVIEW_TOPIC
    haptic: str = motion_config.HAPTIC_TOPIC
    unity_pong: str = motion_config.UNITY_PONG_TOPIC
    suction: str = motion_config.SUCTION_TOPIC
    light: str = motion_config.LIGHT_TOPIC
    dashboard_cmd: str = motion_config.DASHBOARD_CMD_TOPIC
    teach_status: str = motion_config.TEACH_STATUS_TOPIC
    trajectory_data: str = motion_config.TRAJECTORY_DATA_TOPIC
    unity_trajectory: str = motion_config.UNITY_TRAJECTORY_TOPIC
    unity_joint_cmd: str = motion_config.UNITY_TOPIC
    unity_speed_factor: str = motion_config.UNITY_SPEED_FACTOR_TOPIC
    teach_job_request: str = motion_config.TEACH_JOB_REQUEST_TOPIC
    teach_job_status: str = motion_config.TEACH_JOB_STATUS_TOPIC
    teach_job_artifact: str = motion_config.TEACH_JOB_ARTIFACT_TOPIC


DEFAULT_TELEOP_TOPICS = TeleopTopicConfig()


def _parameter_value(parameter, default):
    if parameter is None:
        return default
    return getattr(parameter, "value", default)


def _validate_topic(name, value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"ROS topic parameter {TOPIC_PARAMETER_PREFIX}{name} must be a non-empty string")
    return value.strip()


def declare_topic_parameters(node, defaults=DEFAULT_TELEOP_TOPICS):
    """Declare `topics.*` parameters and return the resolved topic profile."""
    resolved = {}
    for field in fields(defaults):
        default_value = getattr(defaults, field.name)
        parameter = node.declare_parameter(
            f"{TOPIC_PARAMETER_PREFIX}{field.name}",
            default_value,
        )
        value = _parameter_value(parameter, default_value)
        resolved[field.name] = _validate_topic(field.name, value)
    return TeleopTopicConfig(**resolved)
