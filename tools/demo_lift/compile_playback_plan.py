#!/usr/bin/env python3
"""Compile a saved trajectory JSON into an exported MG400 playback job.

Usage (from repo root):

    PYTHONPATH=src/dobot_mg400/mg400_controller:src/dobot_mg400/mg400_protocol \\
      python3 tools/demo_lift/compile_playback_plan.py \\
      --trajectory-json _supporting_materials/data/trajectories/json_trajectories/money.json \\
      --out demo_out/money.compiled_playback.json
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
for _pkg in (
    "src/robot_teaching_core",
    "src/dobot_mg400/mg400_controller",
    "src/dobot_mg400/mg400_protocol",
):
    _p = str(_REPO / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mg400_controller.common.trajectory import TrajectoryRecorder  # noqa: E402


class _CliLogger:
    def info(self, msg):
        print(msg)

    def warn(self, msg):
        print(msg)

    warning = warn

    def error(self, msg):
        print(msg, file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compile a trajectory JSON into an exported MG400 playback plan."
    )
    parser.add_argument(
        "--trajectory-json",
        required=True,
        help="Path to a saved trajectory JSON (Unity/native format).",
    )
    parser.add_argument(
        "--out",
        required=True,
        help="Output path for the compiled playback plan JSON.",
    )
    args = parser.parse_args()

    recorder = TrajectoryRecorder(
        command_send_fn=lambda _cmd: True,
        logger=_CliLogger(),
        traj_dir=str(Path(args.trajectory_json).resolve().parent),
    )

    source = Path(args.trajectory_json).resolve()
    if not source.is_file():
        print(f"Trajectory file not found: {source}", file=sys.stderr)
        return 1

    if not recorder.load(source.name):
        return 1

    out_path = recorder.export_loaded_plan(args.out)
    print(f"compiled playback plan written to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
