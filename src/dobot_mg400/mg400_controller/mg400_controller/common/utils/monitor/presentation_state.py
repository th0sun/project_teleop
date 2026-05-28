#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Presentation-oriented monitor state derived from telemetry.
"""

import time


MODE_NAMES = {
    1: "INIT",
    4: "DISABLED",
    5: "ENABLE",
    6: "DRAG",
    7: "RUN",
    9: "ERROR",
    11: "COLLISION",
}


def build_joint_display_rows(target_joints, actual_joints):
    rows = []
    total_diff = 0.0

    for target, actual in zip(target_joints, actual_joints):
        diff = actual - target
        total_diff += abs(diff)
        if abs(diff) > 2.0:
            color = "red"
        elif abs(diff) > 0.5:
            color = "orange"
        else:
            color = "green"
        rows.append({
            "target": f"{target:.2f}",
            "actual": f"{actual:.2f}",
            "diff": f"{diff:+.2f}",
            "color": color,
        })

    return rows, total_diff


def build_cartesian_display_state(telemetry):
    unity_xyz = telemetry.latest_unity_xyz[:3]
    flange_xyz = telemetry.latest_flange_actual[:3]
    tcp_xyz = telemetry.latest_tool_actual[:3]
    tool_delta = [tcp_xyz[index] - flange_xyz[index] for index in range(3)]

    return {
        "unity_xyz": unity_xyz,
        "flange_xyz": flange_xyz,
        "tcp_xyz": tcp_xyz,
        "tool_delta": tool_delta,
        "tool_index_text": f"Tool {telemetry.latest_tool_index}" if telemetry.latest_tool_index >= 0 else "— (querying...)",
    }


def build_status_display_state(telemetry, error_decoder, now=None):
    now = time.time() if now is None else now
    mode = telemetry.latest_robot_mode
    error = telemetry.latest_error_status

    if error != 0:
        description, _, _ = error_decoder.decode_error(error)
        error_text = description if description else "Unknown Error"
        error_label = f"❌ ERR {error:02X}: {error_text}"
        error_color = "red"
    else:
        error_label = "✅ ERR: 00 (Clear)"
        error_color = "gray"

    if telemetry.last_target_time == 0:
        latency_text = "Status: No Target Received"
    else:
        time_since_target = now - telemetry.last_target_time
        time_since_actual = now - telemetry.last_actual_time
        latency_text = f"Cmd Age: {time_since_target*1000:.0f}ms | Feed Age: {time_since_actual*1000:.0f}ms"

    return {
        "mode_text": f"🤖 MODE: {MODE_NAMES.get(mode, str(mode))}",
        "error_text": error_label,
        "error_color": error_color,
        "latency_text": latency_text,
        "do_hex_text": f"DO: 0x{telemetry.latest_do_status:04X} | Bits: {bin(telemetry.latest_do_status)}",
    }
