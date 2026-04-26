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
from trajectory_msgs.msg import JointTrajectory

from mg400_controller.common.ros.topic_config import (
    DEFAULT_TELEOP_TOPICS,
    TeleopTopicConfig,
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
    teach_job_status: object
    teach_job_artifact: object


@dataclass
class TeleopSubscriptions:
    pong: object
    suction: object
    lights: object
    dashboard_cmd: object
    teach_status: object
    traj_data: object
    unity_trajectory: object
    teach_job_request: object
    unity: object = None


def create_publishers(node, topics: TeleopTopicConfig = DEFAULT_TELEOP_TOPICS):
    return TeleopPublishers(
        heartbeat=node.create_publisher(Int64, topics.ros_ping, 10),
        rviz=node.create_publisher(JointState, topics.joint_states, 10),
        debug=node.create_publisher(String, topics.debug, 10),
        safety=node.create_publisher(String, topics.safety, 10),
        do_status=node.create_publisher(Int64, topics.do_status, 10),
        robot_mode=node.create_publisher(Int32, topics.robot_mode, 10),
        error_status=node.create_publisher(Int32, topics.error_status, 10),
        tool_actual=node.create_publisher(Float64MultiArray, topics.tool_actual, 10),
        tool_target=node.create_publisher(Float64MultiArray, topics.tool_target, 10),
        flange_actual=node.create_publisher(Float64MultiArray, topics.flange_actual, 10),
        tool_index=node.create_publisher(Int32, topics.tool_index, 10),
        predicted_target=node.create_publisher(JointState, topics.predicted_target, 10),
        sent_command=node.create_publisher(JointState, topics.sent_command, 10),
        unity_xyz=node.create_publisher(Float64MultiArray, topics.unity_xyz, 10),
        playback_unity=node.create_publisher(JointState, topics.playback_unity, 10),
        traj_preview=node.create_publisher(String, topics.traj_preview, 10),
        haptic=node.create_publisher(String, topics.haptic, 10),
        teach_job_status=node.create_publisher(String, topics.teach_job_status, 10),
        teach_job_artifact=node.create_publisher(String, topics.teach_job_artifact, 10),
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
    joint_trajectory_callback,
    teach_job_request_callback,
    topics: TeleopTopicConfig = DEFAULT_TELEOP_TOPICS,
):
    return TeleopSubscriptions(
        pong=node.create_subscription(String, topics.unity_pong, unity_pong_callback, 10),
        suction=node.create_subscription(Bool, topics.suction, suction_callback, 10),
        lights=node.create_subscription(Int32MultiArray, topics.light, light_callback, 10),
        dashboard_cmd=node.create_subscription(
            String,
            topics.dashboard_cmd,
            dashboard_cmd_callback,
            10,
        ),
        teach_status=node.create_subscription(
            String,
            topics.teach_status,
            teach_status_callback,
            10,
        ),
        traj_data=node.create_subscription(
            String,
            topics.trajectory_data,
            traj_data_callback,
            10,
        ),
        unity_trajectory=node.create_subscription(
            JointTrajectory,
            topics.unity_trajectory,
            joint_trajectory_callback,
            10,
        ),
        teach_job_request=node.create_subscription(
            String,
            topics.teach_job_request,
            teach_job_request_callback,
            10,
        ),
    )


def attach_unity_subscription(
    node,
    unity_callback,
    qos_profile,
    topics: TeleopTopicConfig = DEFAULT_TELEOP_TOPICS,
):
    return node.create_subscription(JointState, topics.unity_joint_cmd, unity_callback, qos_profile)
