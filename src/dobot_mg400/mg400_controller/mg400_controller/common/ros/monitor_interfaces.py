#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Reusable ROS wiring and telemetry state for the monitor GUI.
"""

from dataclasses import dataclass
import time

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32, Int32MultiArray, Int64

from mg400_controller.common.ros.topic_config import (
    DEFAULT_TELEOP_TOPICS,
    TeleopTopicConfig,
)


@dataclass
class MonitorControlPublishers:
    suction: object
    light: object


@dataclass
class MonitorSubscriptions:
    actual: object
    target: object
    predicted: object
    sent: object
    raw_unity: object
    tool_actual: object
    tool_target: object
    unity_xyz: object
    flange_actual: object
    tool_index: object
    do_status: object
    robot_mode: object
    error_status: object


class MonitorTelemetryState:
    def __init__(self):
        self.latest_actual_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_target_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_predicted_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_sent_joints = [0.0, 0.0, 0.0, 0.0]
        self.sent_fresh = [False, False, False, False]
        self.latest_raw_unity_joints = [0.0, 0.0, 0.0, 0.0]
        self.latest_tool_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_tool_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_unity_xyz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_flange_actual = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_tool_index = -1
        self.latest_do_status = 0
        self.latest_robot_mode = 0
        self.latest_error_status = 0
        self.last_target_time = 0.0
        self.last_actual_time = 0.0

    def on_actual(self, msg):
        if len(msg.position) >= 9:
            q_rad = [msg.position[0], msg.position[1], msg.position[3], msg.position[8]]
            self.latest_actual_joints = list(np.degrees(q_rad))
            self.last_actual_time = time.time()

    def on_target(self, msg):
        if len(msg.position) >= 4:
            self.latest_target_joints = list(np.degrees(msg.position[:4]))
            self.last_target_time = time.time()

    def on_raw_unity(self, msg):
        if len(msg.position) >= 4:
            self.latest_raw_unity_joints = list(np.degrees(msg.position[:4]))

    def on_predicted(self, msg):
        if len(msg.position) >= 4:
            self.latest_predicted_joints = list(np.degrees(msg.position[:4]))

    def on_sent(self, msg):
        if len(msg.position) >= 4:
            self.latest_sent_joints = list(np.degrees(msg.position[:4]))
            self.sent_fresh = [True, True, True, True]

    def on_tool_actual(self, msg):
        if len(msg.data) >= 6:
            self.latest_tool_actual = list(msg.data)

    def on_tool_target(self, msg):
        if len(msg.data) >= 6:
            self.latest_tool_target = list(msg.data)

    def on_unity_xyz(self, msg):
        if len(msg.data) >= 6:
            self.latest_unity_xyz = list(msg.data)

    def on_flange_actual(self, msg):
        if len(msg.data) >= 6:
            self.latest_flange_actual = list(msg.data)

    def on_tool_index(self, msg):
        self.latest_tool_index = int(msg.data)

    def on_do_status(self, msg):
        self.latest_do_status = int(msg.data)

    def on_robot_mode(self, msg):
        self.latest_robot_mode = int(msg.data)

    def on_error_status(self, msg):
        self.latest_error_status = int(msg.data)

    def consume_sent_fresh(self, index):
        was_fresh = self.sent_fresh[index]
        self.sent_fresh[index] = False
        return was_fresh

    def build_session_snapshot(self):
        return {
            "raw_unity": list(self.latest_raw_unity_joints),
            "predicted": list(self.latest_predicted_joints),
            "sent": list(self.latest_sent_joints),
            "actual": list(self.latest_actual_joints),
            "tool_target": list(self.latest_tool_target),
            "tool_actual": list(self.latest_tool_actual),
            "unity_xyz": list(self.latest_unity_xyz),
            "robot_mode": self.latest_robot_mode,
            "error_status": self.latest_error_status,
        }


def create_control_publishers(node, topics: TeleopTopicConfig = DEFAULT_TELEOP_TOPICS):
    return MonitorControlPublishers(
        suction=node.create_publisher(Bool, topics.suction, 10),
        light=node.create_publisher(Int32MultiArray, topics.light, 10),
    )


def create_monitor_subscriptions(node, telemetry, topics: TeleopTopicConfig = DEFAULT_TELEOP_TOPICS):
    return MonitorSubscriptions(
        actual=node.create_subscription(JointState, topics.joint_states, telemetry.on_actual, 10),
        target=node.create_subscription(JointState, topics.unity_joint_cmd, telemetry.on_target, 10),
        predicted=node.create_subscription(JointState, topics.predicted_target, telemetry.on_predicted, 10),
        sent=node.create_subscription(JointState, topics.sent_command, telemetry.on_sent, 10),
        raw_unity=node.create_subscription(JointState, topics.unity_joint_cmd, telemetry.on_raw_unity, 10),
        tool_actual=node.create_subscription(Float64MultiArray, topics.tool_actual, telemetry.on_tool_actual, 10),
        tool_target=node.create_subscription(Float64MultiArray, topics.tool_target, telemetry.on_tool_target, 10),
        unity_xyz=node.create_subscription(Float64MultiArray, topics.unity_xyz, telemetry.on_unity_xyz, 10),
        flange_actual=node.create_subscription(Float64MultiArray, topics.flange_actual, telemetry.on_flange_actual, 10),
        tool_index=node.create_subscription(Int32, topics.tool_index, telemetry.on_tool_index, 10),
        do_status=node.create_subscription(Int64, topics.do_status, telemetry.on_do_status, 10),
        robot_mode=node.create_subscription(Int32, topics.robot_mode, telemetry.on_robot_mode, 10),
        error_status=node.create_subscription(Int32, topics.error_status, telemetry.on_error_status, 10),
    )
