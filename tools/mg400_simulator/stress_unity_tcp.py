#!/usr/bin/env python3
"""Headless Unity-like ROS-TCP publisher for runtime smoke tests."""

import argparse
import math
import sys
import time

from core.unity_tcp_bridge import UnityTcpBridge


def build_joints_deg(index: int, hz: float) -> list[float]:
    t = index / hz
    return [
        8.0 * math.sin(t * 1.7),
        18.0 + 6.0 * math.sin(t * 1.1),
        -28.0 + 5.0 * math.cos(t * 1.3),
        12.0 * math.sin(t * 0.9),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Publish /unity/teleop_sample + /unity/joint_cmd like Unity at a fixed rate."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--warmup-sec", type=float, default=0.25)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    args = parser.parse_args()

    if args.hz <= 0:
        raise SystemExit("--hz must be positive")
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")

    bridge = UnityTcpBridge()
    if not bridge.start(args.host, args.port):
        return 2
    if args.warmup_sec > 0:
        time.sleep(args.warmup_sec)

    period = 1.0 / args.hz
    next_tick = time.perf_counter()
    try:
        for index in range(args.samples):
            bridge.publish_joint_cmd(build_joints_deg(index, args.hz))
            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        if args.settle_sec > 0:
            time.sleep(args.settle_sec)
        bridge.stop()

    print(
        "stress complete: "
        f"sent={args.samples} hz={args.hz:g} "
        f"session_id={bridge.teleop_session_id} "
        f"last_unity_seq_id={bridge._unity_seq_id}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
