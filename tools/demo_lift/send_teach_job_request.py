#!/usr/bin/env python3
"""Build (and optionally publish) a /teach/job_request payload from a Unity
trajectory JSON file.

Phase 2 helper: lets the operator drive the teach-repeat job bridge from
the command line, which is much easier than wiring up a full Quest 3 build
just to validate ROS-side behaviour.  Defaults to a dry-run so the script
also doubles as documentation of the wire format that
``TeleOp/Assets/Scripts/TeachJobPublisher.cs`` produces.

Wire format reference: ``architecture_phase_research_log.md`` §19.

Usage examples
--------------

Print the JSON envelope a Save would produce::

    python3 tools/demo_lift/send_teach_job_request.py \\
      --trajectory _supporting_materials/data/trajectories/json_trajectories/unity_mock_test.json \\
      --action compile --print

Publish to a live ROS graph (requires rclpy)::

    python3 tools/demo_lift/send_teach_job_request.py \\
      --trajectory .../unity_mock_test.json \\
      --action execute --publish

The ``--publish`` flag pulls in ``rclpy`` lazily so ``--print`` /
``--dry-run`` modes work on environments without a ROS install.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional


# Action identifiers must stay in lock-step with
# ``mg400_controller.common.trajectory.teach_job_handler.VALID_ACTIONS``.
VALID_ACTIONS = (
    "compile",
    "preview_sim",
    "execute",
    "export",
    "stop",
    "record_start",
    "record_stop",
)

DEFAULT_TOPIC = "/teach/job_request"


def _load_trajectory(path: Path) -> Dict[str, Any]:
    """Load a Unity-format trajectory JSON.

    Accepts either ``{"frames": [...]}`` (Unity / TrajectoryRecorder format)
    or a bare list of frames.  Frames must each contain ``timeStamp`` plus
    ``j1..j4`` in degrees.
    """
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, dict):
        frames = data.get("frames", [])
    elif isinstance(data, list):
        frames = data
    else:
        raise ValueError(f"Unsupported trajectory shape in {path}: {type(data).__name__}")

    if not isinstance(frames, list) or not frames:
        raise ValueError(f"No frames found in {path}")

    for i, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f"Frame {i} in {path} is not an object")
        for key in ("timeStamp", "j1", "j2", "j3", "j4"):
            if key not in frame:
                raise ValueError(f"Frame {i} in {path} missing {key!r}")

    return {"filename": path.name, "frames": frames}


def build_request(
    *,
    action: str,
    trajectory: Optional[Dict[str, Any]],
    target: str,
    options: Dict[str, Any],
    job_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Assemble a JobRequest payload identical in shape to TeachJobPublisher.cs."""
    payload: Dict[str, Any] = {
        "job_id": job_id or str(uuid.uuid4()),
        "action": action,
        "target": target,
        "submitted_at_unity_sec": float(time.time()),
        "options": dict(options),
    }
    if trajectory is not None:
        payload["trajectory"] = trajectory
    return payload


def _parse_options(option_strings: List[str]) -> Dict[str, Any]:
    """Parse repeated ``--option key=value`` flags.  ``true``/``false``/numbers
    are coerced; everything else stays a string.
    """
    out: Dict[str, Any] = {}
    for raw in option_strings:
        if "=" not in raw:
            raise SystemExit(f"--option expects key=value, got {raw!r}")
        key, value = raw.split("=", 1)
        out[key.strip()] = _coerce_option(value.strip())
    return out


def _coerce_option(value: str) -> Any:
    lowered = value.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def _publish(topic: str, payload: str, timeout: float) -> None:
    """Publish the payload to ``topic`` via rclpy.

    Imported lazily so ``--print`` / ``--dry-run`` work without a ROS
    install on dev laptops.
    """
    try:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from std_msgs.msg import String  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on env
        raise SystemExit(
            f"--publish requires rclpy + std_msgs, but they are not available: {exc}"
        )

    rclpy.init(args=None)
    try:
        node = Node("send_teach_job_request_cli")
        publisher = node.create_publisher(String, topic, 5)
        # Spin briefly so the publisher actually registers with discovery
        # before we send.  Without this the message can land in the void on
        # cold ROS daemons.
        deadline = time.time() + max(0.5, timeout)
        while time.time() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

        msg = String()
        msg.data = payload
        publisher.publish(msg)
        # One more spin to flush DDS.
        rclpy.spin_once(node, timeout_sec=0.1)

        sub_count = publisher.get_subscription_count()
        if sub_count == 0:
            print(
                f"[warn] no subscribers detected on {topic} during the {timeout:.1f}s wait; "
                "message may have been dropped",
                file=sys.stderr,
            )
        else:
            print(f"[ok] published to {topic} (subscribers={sub_count})")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--trajectory",
        type=Path,
        help="Path to a Unity-format trajectory JSON.  Required for compile / "
             "preview_sim / execute / export.",
    )
    parser.add_argument(
        "--action",
        choices=VALID_ACTIONS,
        required=True,
        help="Job action to submit.",
    )
    parser.add_argument(
        "--target",
        default="mg400",
        help="Robot target string (default: mg400).  Use '' for accept-any.",
    )
    parser.add_argument(
        "--option",
        dest="options",
        action="append",
        default=[],
        metavar="key=value",
        help="Repeatable.  Forwarded into the request 'options' field.  "
             "Booleans/numbers coerced automatically.",
    )
    parser.add_argument(
        "--job-id",
        help="Override the auto-generated UUID.  Useful for correlating in "
             "logs across runs.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="ROS topic to publish on (default: /teach/job_request).",
    )

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--print",
        dest="print_mode",
        action="store_true",
        help="Print the JSON payload to stdout.  Default mode.",
    )
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate inputs and print a one-line summary without publishing.",
    )
    mode.add_argument(
        "--publish",
        action="store_true",
        help="Publish the payload to --topic via rclpy.  Requires a working ROS install.",
    )

    parser.add_argument(
        "--publish-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for a subscriber when --publish is set (default: 2.0).",
    )

    args = parser.parse_args(argv)

    actions_needing_trajectory = {"compile", "preview_sim", "execute", "export"}
    trajectory: Optional[Dict[str, Any]] = None
    if args.trajectory is not None:
        trajectory = _load_trajectory(args.trajectory.expanduser().resolve())
    elif args.action in actions_needing_trajectory:
        parser.error(f"--trajectory is required for action '{args.action}'")

    options = _parse_options(args.options)
    request = build_request(
        action=args.action,
        trajectory=trajectory,
        target=args.target,
        options=options,
        job_id=args.job_id,
    )
    payload = json.dumps(request, sort_keys=True)

    if args.publish:
        _publish(args.topic, payload, timeout=args.publish_timeout)
        return 0

    if args.dry_run:
        frame_count = len(trajectory["frames"]) if trajectory else 0
        print(
            f"[dry-run] action={args.action} target={args.target!r} "
            f"job_id={request['job_id']} frames={frame_count} "
            f"options={options} bytes={len(payload)}"
        )
        return 0

    # default: --print
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
