"""Teach-and-repeat ↔ live-teleop bridge.

The teach/repeat path (compile, execute, tune, export, stop) is owned
by ``TeachJobHandler`` over in ``common/trajectory/teach_job_handler.py``.
This module hosts the *bridge* between that handler / its
``TrajectoryRecorder`` and the live VR teleop loop on the node:

  * Forwarding /teach/job_request payloads into the handler.
  * Anchoring the live teleop reference at the robot's pose whenever
    playback starts or completes (so the operator's next VR motion
    doesn't snap the arm back to a stale target).
  * Restoring realtime SpeedFactor defaults after teach/repeat had
    swapped them out.
  * Publishing the recorder's per-waypoint / per-target streams onto
    the ROS topics the monitor draws from.

Every function takes the owning ``TeleopNode`` as its first argument
so all the shared state (latest_target, controller, target_compensator,
ros_publishers, ...) stays in one place and the bridge stays a pure
function module — no extra class state to track.
"""
from __future__ import annotations

import json
import time

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def teach_job_request_callback(node, msg) -> None:
    """``/teach/job_request`` (String JSON) — dispatch to ``TeachJobHandler``.

    The handler emits status updates on ``/teach/job_status`` and (for
    compile actions) the compiled artifact on ``/teach/job_artifact``;
    see ``teach_job_handler.py`` for the full schema.
    """
    try:
        payload = msg.data if isinstance(msg.data, str) else str(msg.data)
    except Exception:
        node.get_logger().error("teach_job_request: cannot read .data field")
        return

    payload_len = len(payload)
    action = None
    job_id = None
    try:
        envelope = json.loads(payload)
        if isinstance(envelope, dict):
            action = envelope.get("action")
            job_id = envelope.get("job_id")
    except Exception:
        pass

    if action:
        node.get_logger().info(
            f"teach_job_request received action={action!r} "
            f"job_id={job_id or ''!r} bytes={payload_len}"
        )
    else:
        snippet = payload[:160].replace("\n", " ")
        node.get_logger().warn(
            "teach_job_request received unparsable envelope "
            f"bytes={payload_len} head={snippet!r}"
        )

    status = node.teach_job_handler.handle(payload)
    node.get_logger().info(
        f"teach_job_request handled stage={getattr(status, 'stage', '?')!r} "
        f"action={getattr(status, 'action', None)!r} "
        f"job_id={getattr(status, 'job_id', '')!r} "
        f"error={getattr(status, 'error_code', None)!r}"
    )


def reset_live_teleop_reference(node, reason: str) -> None:
    """Anchor live teleop at the robot's current pose after playback.

    Why: when teach/repeat finishes, the VR operator's hand is almost
    certainly somewhere else than the last target the recorder sent.
    If we leave the controller's reference at the stale Unity target,
    the next live joint_cmd snaps the arm back through that point.
    Resetting the reference to the robot's *current* pose makes the
    next live target a small delta and keeps motion smooth.
    """
    try:
        q_current = node.feedback.get_current_position()
    except Exception:
        q_current = None

    now_mono = time.perf_counter()
    now_wall = time.time()
    if q_current is None:
        node.latest_target = None
        node.controller.reset_reference(None, now=now_mono)
        try:
            node.target_compensator.reset()
        except AttributeError:
            pass
        node.get_logger().warn(
            f"⚠️  Live teleop reference cleared after {reason}; "
            "waiting for next Unity target"
        )
        return

    q_current = np.asarray(q_current[:4], dtype=float)
    node.latest_target = q_current.copy()
    node.controller.reset_reference(q_current, now=now_mono)
    try:
        node.target_compensator.reset(q_current, target_time=now_wall)
    except AttributeError:
        pass
    node.target_recv_time = now_wall
    node.unity_send_time = now_wall
    node.get_logger().info(
        f"🔁 Live teleop reference reset after {reason}: "
        f"{np.degrees(q_current).round(2).tolist()} deg"
    )


def playback_lifecycle_callback(node, event_name: str, payload) -> None:
    """Recorder fires this on ``playback_start`` / ``playback_complete``.

    Two responsibilities:
      1. Anchor live teleop at the current pose (avoids the post-play
         snap-back described in ``reset_live_teleop_reference``).
      2. Arm a one-shot "restore realtime SpeedFactor defaults" flag
         that the control loop consumes the next time a fresh Unity
         target arrives — so the two speed domains (teach vs live)
         stay explicit and don't clobber each other mid-run.
    """
    if event_name in {"playback_start", "playback_complete"}:
        reset_live_teleop_reference(node, event_name)
    if event_name == "playback_start":
        # Teach/repeat owns SpeedFactor while executing. Don't let an
        # earlier armed realtime reset stomp a fresh teach run.
        node._realtime_speed_defaults_pending = False
    elif event_name == "playback_complete":
        # Keep the teach/repeat speed profile available for repeated
        # runs. Realtime speed is restored only when the next live
        # Unity target arrives, so the two speed domains stay explicit.
        node._realtime_speed_defaults_pending = True
        node.get_logger().info(
            "🏁 Teach/repeat playback complete; realtime SpeedFactor(100) "
            "is armed for the next /unity/joint_cmd target"
        )


def restore_realtime_speed_defaults(node, reason: str) -> bool:
    """Make live teleop fast again after teach/repeat changed SpeedFactor.

    Returns True on success, False if the robot isn't reachable (and
    re-arms the pending flag so the next live target tries again).
    """
    if not node.connection.connected:
        node._realtime_speed_defaults_pending = True
        node.get_logger().warn(
            f"⚠️ Cannot restore realtime SpeedFactor after {reason}; robot disconnected"
        )
        return False
    ok = node.connection.set_realtime_speed_defaults()
    node._realtime_speed_defaults_pending = not ok
    if ok:
        node.get_logger().info(
            f"🏃 Realtime mode speed restored after {reason}: "
            "SpeedFactor/SpeedJ/AccJ = 100"
        )
    return ok


def playback_waypoint_callback(node, q_rad) -> None:
    """Recorder hook: a waypoint just got queued / sent to the robot.

    Publishes only to ``/teleop/sent_command`` so the monitor's
    "commanded" trace updates the moment the recorder hands a frame
    over to the dashboard.
    """
    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
    js.position = list(q_rad)
    node.ros_publishers.sent_command.publish(js)


def playback_target_callback(node, q_rad) -> None:
    """Recorder hook: ~100 Hz interpolated target (equivalent to
    race.py's "target" line).

    Publishes the target on ``/teleop/playback_unity`` (the monitor
    draws the yellow target trace from this) and the matching XYZ
    Cartesian pose on ``/unity/xyz`` so the live preview stays in
    sync during teach/repeat playback.
    """
    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
    js.position = list(q_rad)
    node.ros_publishers.playback_unity.publish(js)

    try:
        xyz = node.feedback.kinematics.forward_kinematics(np.degrees(q_rad))
        xyz_msg = Float64MultiArray()
        xyz_msg.data = xyz.tolist()
        node.ros_publishers.unity_xyz.publish(xyz_msg)
    except Exception:
        pass
