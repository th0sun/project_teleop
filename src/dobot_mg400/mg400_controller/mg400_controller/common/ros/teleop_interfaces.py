#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS publisher/subscriber wiring for the MG400 teleop node.

Keeping topic setup here reduces noise in `vr_teleop_node.py` and makes the
runtime ROS surface easier to inspect and evolve.
"""

from dataclasses import dataclass

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray, Bool, Int32MultiArray, Int64, Int32

import mg400_controller.common.config.motion_config as motion_config
from mg400_controller.common.config.motion_config import (
    RVIZ_TOPIC,
    DEBUG_TOPIC,
    SAFETY_TOPIC,
    HAPTIC_TOPIC,
    SUCTION_TOPIC,
    LIGHT_TOPIC,
    ROS_PING_TOPIC,
    UNITY_PONG_TOPIC,
    PREDICTED_TARGET_TOPIC,
    SENT_COMMAND_TOPIC,
    UNITY_XYZ_TOPIC,
    PLAYBACK_UNITY_TOPIC,
    TRAJ_PREVIEW_TOPIC,
    TEACH_STATUS_TOPIC,
    TRAJECTORY_DATA_TOPIC,
    TOOL_ACTUAL_TOPIC,
    TOOL_TARGET_TOPIC,
    FLANGE_ACTUAL_TOPIC,
    TOOL_INDEX_TOPIC,
    DASHBOARD_CMD_TOPIC,
    UNITY_TOPIC,
)


@dataclass
class TeleopPublishers:
    heartbeat: object
    rviz: object
    debug: object
    safety: object
    do_status: object
    robot_mode: object
    error_status: object
    tool_actual: object
    tool_target: object
    flange_actual: object
    tool_index: object
    predicted_target: object
    sent_command: object
    unity_xyz: object
    playback_unity: object
    traj_preview: object
    haptic: object


@dataclass
class TeleopSubscriptions:
    pong: object
    suction: object
    lights: object
    dashboard_cmd: object
    teach_status: object
    traj_data: object
    unity: object = None


def create_publishers(node):
    return TeleopPublishers(
        heartbeat=node.create_publisher(Int64, ROS_PING_TOPIC, 10),
        rviz=node.create_publisher(JointState, RVIZ_TOPIC, 10),
        debug=node.create_publisher(String, DEBUG_TOPIC, 10),
        safety=node.create_publisher(String, SAFETY_TOPIC, 10),
        do_status=node.create_publisher(Int64, motion_config.DO_STATUS_TOPIC, 10),
        robot_mode=node.create_publisher(Int32, motion_config.ROBOT_MODE_TOPIC, 10),
        error_status=node.create_publisher(Int32, motion_config.ERROR_STATUS_TOPIC, 10),
        tool_actual=node.create_publisher(Float64MultiArray, TOOL_ACTUAL_TOPIC, 10),
        tool_target=node.create_publisher(Float64MultiArray, TOOL_TARGET_TOPIC, 10),
        flange_actual=node.create_publisher(Float64MultiArray, FLANGE_ACTUAL_TOPIC, 10),
        tool_index=node.create_publisher(Int32, TOOL_INDEX_TOPIC, 10),
        predicted_target=node.create_publisher(JointState, PREDICTED_TARGET_TOPIC, 10),
        sent_command=node.create_publisher(JointState, SENT_COMMAND_TOPIC, 10),
        unity_xyz=node.create_publisher(Float64MultiArray, UNITY_XYZ_TOPIC, 10),
        playback_unity=node.create_publisher(JointState, PLAYBACK_UNITY_TOPIC, 10),
        traj_preview=node.create_publisher(String, TRAJ_PREVIEW_TOPIC, 10),
        haptic=node.create_publisher(String, HAPTIC_TOPIC, 10),
    )


def create_subscriptions(
    node,
    *,
    unity_pong_callback,
    suction_callback,
    light_callback,
    dashboard_cmd_callback,
    teach_status_callback,
    traj_data_callback,
):
    return TeleopSubscriptions(
        pong=node.create_subscription(String, UNITY_PONG_TOPIC, unity_pong_callback, 10),
        suction=node.create_subscription(Bool, SUCTION_TOPIC, suction_callback, 10),
        lights=node.create_subscription(Int32MultiArray, LIGHT_TOPIC, light_callback, 10),
        dashboard_cmd=node.create_subscription(
            String,
            DASHBOARD_CMD_TOPIC,
            dashboard_cmd_callback,
            10,
        ),
        teach_status=node.create_subscription(
            String,
            TEACH_STATUS_TOPIC,
            teach_status_callback,
            10,
        ),
        traj_data=node.create_subscription(
            String,
            TRAJECTORY_DATA_TOPIC,
            traj_data_callback,
            10,
        ),
    )


def attach_unity_subscription(node, unity_callback, qos_profile):
    return node.create_subscription(JointState, UNITY_TOPIC, unity_callback, qos_profile)
