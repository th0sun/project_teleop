#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unity teleop sample protocol parsing.

The protocol is sent as ``std_msgs/String`` JSON on ``/unity/teleop_sample``.
It gives ROS a stable Unity-side identity and controller/IK context that the
legacy ``sensor_msgs/JointState`` command topic cannot carry.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Optional
from urllib.parse import quote, unquote


UNITY_TELEOP_PROTOCOL_VERSION = "teleop_sample_v1"
UNITY_JOINT_FRAME_PREFIX = UNITY_TELEOP_PROTOCOL_VERSION
ROS_JOINT_CMD_RX_SOURCE = "ros_joint_cmd_rx"

UNITY_SAMPLE_PROTOCOL_FIELDS = (
    "protocol_version",
    "session_id",
    "unity_seq_id",
    "source",
    "controller_capture_ts",
    "unity_target_ts",
    "unity_ik_ts",
    "unity_send_ts",
    "controller_id",
    "ctrl_pos_x_m",
    "ctrl_pos_y_m",
    "ctrl_pos_z_m",
    "ctrl_rot_x",
    "ctrl_rot_y",
    "ctrl_rot_z",
    "ctrl_rot_w",
    "trigger_value",
    "grip_value",
    "primary_button",
    "secondary_button",
    "target_raw_x_mm",
    "target_raw_y_mm",
    "target_raw_z_mm",
    "target_raw_r_deg",
    "target_filt_x_mm",
    "target_filt_y_mm",
    "target_filt_z_mm",
    "target_filt_r_deg",
    "unity_filter_status",
    "unity_filter_detail",
    "j1_ik_deg",
    "j2_ik_deg",
    "j3_ik_deg",
    "j4_ik_deg",
    "control_mode",
    "is_valid",
    "invalid_reason",
)

UNITY_SAMPLE_LEGACY_IK_RAD_FIELDS = (
    "j1_ik_rad",
    "j2_ik_rad",
    "j3_ik_rad",
    "j4_ik_rad",
)

UNITY_SAMPLE_MATCH_FIELDS = (
    "unity_sample_match_method",
    "unity_sample_age_ms",
)

UNITY_SAMPLE_LOG_FIELDS = (
    UNITY_SAMPLE_PROTOCOL_FIELDS
    + UNITY_SAMPLE_LEGACY_IK_RAD_FIELDS
    + UNITY_SAMPLE_MATCH_FIELDS
)


class UnityTeleopSampleError(ValueError):
    """Raised when a Unity teleop sample cannot be parsed safely."""


@dataclass(frozen=True)
class UnityTeleopSample:
    """Parsed Unity teleop sample.

    ``raw`` is retained so future Unity fields can be logged without breaking
    callers that only understand protocol v1.
    """

    raw: Mapping[str, Any]
    protocol_version: str
    session_id: str
    unity_seq_id: int

    def get(self, key: str, default: Any = None) -> Any:
        return self.raw.get(key, default)

    def unity_send_ts(self) -> Optional[float]:
        return _optional_float(self.raw.get("unity_send_ts"))

    def is_valid(self) -> bool:
        value = self.raw.get("is_valid")
        if value in (None, ""):
            return True
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "y")
        return bool(value)

    def invalid_reason(self) -> str:
        return str(self.raw.get("invalid_reason") or "")

    def ik_joints_deg(self) -> Optional[list[float]]:
        array_value = self.raw.get("joints_ik_deg", self.raw.get("j_ik_deg"))
        if array_value is not None:
            try:
                joints = [_required_float(value, "joints_ik_deg") for value in array_value]
            except TypeError as exc:
                raise UnityTeleopSampleError("joints_ik_deg must be an array") from exc
            return joints[:4] if len(joints) >= 4 else None

        keys = ("j1_ik_deg", "j2_ik_deg", "j3_ik_deg", "j4_ik_deg")
        if all(key in self.raw for key in keys):
            return [_required_float(self.raw[key], key) for key in keys]

        joints_rad = self._legacy_ik_joints_rad()
        if joints_rad is None:
            return None
        return [math.degrees(value) for value in joints_rad]

    def ik_joints_rad(self) -> Optional[list[float]]:
        joints_deg = self.ik_joints_deg()
        if joints_deg is not None:
            return [math.radians(value) for value in joints_deg]
        return self._legacy_ik_joints_rad()

    def _legacy_ik_joints_rad(self) -> Optional[list[float]]:
        array_value = self.raw.get("joints_ik_rad", self.raw.get("j_ik_rad"))
        if array_value is not None:
            try:
                joints = [_required_float(value, "joints_ik_rad") for value in array_value]
            except TypeError as exc:
                raise UnityTeleopSampleError("joints_ik_rad must be an array") from exc
            return joints[:4] if len(joints) >= 4 else None

        keys = ("j1_ik_rad", "j2_ik_rad", "j3_ik_rad", "j4_ik_rad")
        if not all(key in self.raw for key in keys):
            return None
        return [_required_float(self.raw[key], key) for key in keys]

    def to_log_fields(self) -> dict[str, Any]:
        return {field: self.raw.get(field) for field in UNITY_SAMPLE_LOG_FIELDS}

    def to_protocol_fields(self) -> dict[str, Any]:
        return {field: self.raw.get(field) for field in UNITY_SAMPLE_PROTOCOL_FIELDS}


def parse_unity_teleop_sample(payload: str) -> UnityTeleopSample:
    """Parse and validate one Unity teleop sample JSON payload."""

    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise UnityTeleopSampleError(f"invalid JSON: {exc.msg}") from exc

    if not isinstance(data, dict):
        raise UnityTeleopSampleError("sample JSON must be an object")

    data = _normalise_sample_payload(data)

    version = str(data.get("protocol_version") or "").strip()
    if not version:
        raise UnityTeleopSampleError("missing protocol_version")

    session_id = str(data.get("session_id") or "").strip()
    if not session_id:
        raise UnityTeleopSampleError("missing session_id")

    seq_value = data.get("unity_seq_id", data.get("seq_id"))
    if seq_value is None:
        raise UnityTeleopSampleError("missing unity_seq_id")
    unity_seq_id = _required_int(seq_value, "unity_seq_id")

    if "unity_seq_id" not in data and "seq_id" in data:
        data = dict(data)
        data["unity_seq_id"] = unity_seq_id

    return UnityTeleopSample(
        raw=data,
        protocol_version=version,
        session_id=session_id,
        unity_seq_id=unity_seq_id,
    )


def build_ros_joint_cmd_rx_sample(
    joints_rad: list[float],
    *,
    session_id: str,
    rx_seq_id: int,
    unity_send_ts: float | None,
    ros_recv_ts: float,
) -> UnityTeleopSample:
    """Build the canonical control identity for one received /unity/joint_cmd.

    Unity's JointState command topic cannot carry a custom sequence field, so
    ROS assigns the control id at receive time.  The JSON teleop sample stream
    can still be logged separately, but live control and ros_command rows should
    key off this receive id instead of trusting a Unity-authored frame_id.
    """

    joints = [float(value) for value in joints_rad[:4]]
    if len(joints) < 4:
        joints.extend([0.0] * (4 - len(joints)))
    joints_deg = [math.degrees(value) for value in joints]
    send_ts = unity_send_ts if unity_send_ts and unity_send_ts > 0.0 else ros_recv_ts
    raw = {
        "protocol_version": UNITY_TELEOP_PROTOCOL_VERSION,
        "session_id": str(session_id),
        "unity_seq_id": int(rx_seq_id),
        "source": ROS_JOINT_CMD_RX_SOURCE,
        "controller_capture_ts": send_ts,
        "unity_target_ts": send_ts,
        "unity_ik_ts": send_ts,
        "unity_send_ts": send_ts,
        "controller_id": ROS_JOINT_CMD_RX_SOURCE,
        "j1_ik_deg": joints_deg[0],
        "j2_ik_deg": joints_deg[1],
        "j3_ik_deg": joints_deg[2],
        "j4_ik_deg": joints_deg[3],
        "j1_ik_rad": joints[0],
        "j2_ik_rad": joints[1],
        "j3_ik_rad": joints[2],
        "j4_ik_rad": joints[3],
        "joints_ik_rad": joints,
        "control_mode": "joint_cmd",
        "is_valid": True,
        "invalid_reason": "",
        "unity_filter_status": "valid",
        "unity_filter_detail": "ros_assigned_joint_cmd_id",
        "unity_sample_match_method": "ros_assigned_joint_cmd_rx",
        "unity_sample_age_ms": max(0.0, (ros_recv_ts - send_ts) * 1000.0),
    }
    return UnityTeleopSample(
        raw=raw,
        protocol_version=UNITY_TELEOP_PROTOCOL_VERSION,
        session_id=str(session_id),
        unity_seq_id=int(rx_seq_id),
    )


def _normalise_sample_payload(data: Mapping[str, Any]) -> dict[str, Any]:
    """Return a flat Teleop Logging v1 payload.

    Early Unity builds emitted all fields at the top level.  The current Unity
    contract groups them into semantic objects such as ``meta``, ``timestamps``,
    ``controller_raw``, ``targets``, ``ik_result`` and ``flags``.  The ROS side
    stores/logs the stable flat column names, so accept both wire shapes here.
    Top-level keys win to preserve backwards compatibility with existing tests
    and bag fixtures.
    """

    flat = dict(data)

    for group_name in (
        "meta",
        "timestamps",
        "controller",
        "controller_raw",
        "targets",
        "ik",
        "ik_result",
        "flags",
    ):
        group = data.get(group_name)
        if isinstance(group, Mapping):
            _copy_missing(flat, group)

    meta = data.get("meta")
    if isinstance(meta, Mapping):
        if "protocol_version" not in flat and "version" in meta:
            flat["protocol_version"] = meta["version"]

    controller = data.get("controller_raw", data.get("controller"))
    if isinstance(controller, Mapping):
        _normalise_controller_fields(flat, controller)

    targets = data.get("targets")
    if isinstance(targets, Mapping):
        _normalise_target_pose(flat, targets, "target_raw", "raw")
        _normalise_target_pose(flat, targets, "target_filt", "filtered")
        _normalise_target_pose(flat, targets, "target_filt", "filt")
        if "unity_filter_status" not in flat and "filter_status" in targets:
            flat["unity_filter_status"] = targets["filter_status"]
        if "unity_filter_detail" not in flat and "filter_detail" in targets:
            flat["unity_filter_detail"] = targets["filter_detail"]

    ik = data.get("ik_result", data.get("ik"))
    if isinstance(ik, Mapping):
        _normalise_ik_fields(flat, ik)

    _derive_ik_unit_fields(flat)
    return flat


def _copy_missing(flat: dict[str, Any], source: Mapping[str, Any]) -> None:
    for key, value in source.items():
        if key not in flat:
            flat[key] = value


def _normalise_controller_fields(flat: dict[str, Any], controller: Mapping[str, Any]) -> None:
    pos = controller.get("pos_m", controller.get("pos"))
    if isinstance(pos, Mapping):
        _copy_axis_fields(flat, pos, "ctrl_pos", "_m", aliases={"x": "x", "y": "y", "z": "z"})
    elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
        flat.setdefault("ctrl_pos_x_m", pos[0])
        flat.setdefault("ctrl_pos_y_m", pos[1])
        flat.setdefault("ctrl_pos_z_m", pos[2])

    rot = controller.get("rot_quat", controller.get("rot"))
    if isinstance(rot, Mapping):
        _copy_axis_fields(flat, rot, "ctrl_rot", "", aliases={"x": "x", "y": "y", "z": "z", "w": "w"})
    elif isinstance(rot, (list, tuple)) and len(rot) >= 4:
        flat.setdefault("ctrl_rot_x", rot[0])
        flat.setdefault("ctrl_rot_y", rot[1])
        flat.setdefault("ctrl_rot_z", rot[2])
        flat.setdefault("ctrl_rot_w", rot[3])

    for canonical, aliases in (
        ("trigger_value", ("trigger", "trigger_value")),
        ("grip_value", ("grip", "grip_value")),
        ("primary_button", ("primary_button", "button_primary")),
        ("secondary_button", ("secondary_button", "button_secondary")),
        ("controller_id", ("controller_id", "hand")),
    ):
        if canonical in flat:
            continue
        for alias in aliases:
            if alias in controller:
                flat[canonical] = controller[alias]
                break


def _normalise_target_pose(
    flat: dict[str, Any],
    targets: Mapping[str, Any],
    canonical_prefix: str,
    nested_name: str,
) -> None:
    pose = targets.get(f"{nested_name}_mm", targets.get(nested_name))
    if isinstance(pose, Mapping):
        _copy_axis_fields(flat, pose, canonical_prefix, "_mm", aliases={"x": "x", "y": "y", "z": "z"})
        if f"{canonical_prefix}_r_deg" not in flat:
            for alias in ("r_deg", "r", "yaw_deg"):
                if alias in pose:
                    flat[f"{canonical_prefix}_r_deg"] = pose[alias]
                    break
    elif isinstance(pose, (list, tuple)) and len(pose) >= 4:
        flat.setdefault(f"{canonical_prefix}_x_mm", pose[0])
        flat.setdefault(f"{canonical_prefix}_y_mm", pose[1])
        flat.setdefault(f"{canonical_prefix}_z_mm", pose[2])
        flat.setdefault(f"{canonical_prefix}_r_deg", pose[3])


def _normalise_ik_fields(flat: dict[str, Any], ik: Mapping[str, Any]) -> None:
    joints_deg = ik.get("joints_ik_deg", ik.get("j_ik_deg"))
    if isinstance(joints_deg, (list, tuple)) and len(joints_deg) >= 4:
        _set_ik_fields(flat, "deg", joints_deg)

    joints_rad = ik.get("joints_ik_rad", ik.get("j_ik_rad"))
    if isinstance(joints_rad, (list, tuple)) and len(joints_rad) >= 4:
        _set_ik_fields(flat, "rad", joints_rad)


def _set_ik_fields(flat: dict[str, Any], unit: str, joints) -> None:
    for idx in range(4):
        flat.setdefault(f"j{idx + 1}_ik_{unit}", joints[idx])


def _derive_ik_unit_fields(flat: dict[str, Any]) -> None:
    joints_deg = _read_ik_fields(flat, "deg")
    joints_rad = _read_ik_fields(flat, "rad")
    if joints_deg is None and joints_rad is not None:
        _set_ik_fields(flat, "deg", [math.degrees(value) for value in joints_rad])
    elif joints_rad is None and joints_deg is not None:
        _set_ik_fields(flat, "rad", [math.radians(value) for value in joints_deg])


def _read_ik_fields(flat: Mapping[str, Any], unit: str) -> Optional[list[float]]:
    keys = tuple(f"j{idx}_ik_{unit}" for idx in range(1, 5))
    if not all(key in flat for key in keys):
        return None
    return [_required_float(flat[key], key) for key in keys]


def _copy_axis_fields(
    flat: dict[str, Any],
    source: Mapping[str, Any],
    prefix: str,
    suffix: str,
    *,
    aliases: Mapping[str, str],
) -> None:
    for source_key, axis in aliases.items():
        canonical = f"{prefix}_{axis}{suffix}"
        if canonical not in flat and source_key in source:
            flat[canonical] = source[source_key]


def encode_unity_joint_frame_id(session_id: str, unity_seq_id: int) -> str:
    """Encode sample identity into JointState.header.frame_id.

    `/unity/joint_cmd` cannot carry custom fields, so Teleop Logging v1 uses
    the standard header frame id as a lightweight identity envelope.  This lets
    ROS match the joint command to the exact `/unity/teleop_sample` row before
    falling back to age-based matching for legacy clients.
    """

    return (
        f"{UNITY_JOINT_FRAME_PREFIX};"
        f"session_id={quote(str(session_id), safe='')};"
        f"unity_seq_id={int(unity_seq_id)}"
    )


def parse_unity_joint_frame_id(frame_id: str | None) -> tuple[str, int] | None:
    if not frame_id:
        return None
    parts = str(frame_id).split(";")
    if not parts or parts[0] != UNITY_JOINT_FRAME_PREFIX:
        return None

    fields: dict[str, str] = {}
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value

    session_id = unquote(fields.get("session_id", "")).strip()
    seq_value = fields.get("unity_seq_id")
    if not session_id or seq_value is None:
        return None
    try:
        return session_id, _required_int(seq_value, "unity_seq_id")
    except UnityTeleopSampleError:
        return None


def _required_int(value: Any, field_name: str) -> int:
    try:
        if isinstance(value, bool):
            raise ValueError
        return int(value)
    except (TypeError, ValueError) as exc:
        raise UnityTeleopSampleError(f"{field_name} must be an integer") from exc


def _required_float(value: Any, field_name: str) -> float:
    try:
        if isinstance(value, bool):
            raise ValueError
        return float(value)
    except (TypeError, ValueError) as exc:
        raise UnityTeleopSampleError(f"{field_name} must be numeric") from exc


def _optional_float(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    return _required_float(value, "timestamp")
