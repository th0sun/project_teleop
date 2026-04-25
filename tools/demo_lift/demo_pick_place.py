#!/usr/bin/env python3
"""Pick-place vertical slice demo.

Demonstrates the full teaching pipeline without a real robot:

    synthetic session
        → lift_session (dwell segmentation + FK via MG400 provider)
        → canonical CanonicalProgram  [robot-neutral IR]
        → wrap with vacuum on / wait / vacuum off
        → translate_program (MG400 command translator)
        → two human-readable artifacts

Usage (from repo root):

    PYTHONPATH=src/robot_teaching_core:src/adapters/mg400:\\
              src/dobot_mg400/mg400_controller:\\
              src/dobot_mg400/mg400_protocol \\
        python3 tools/demo_lift/demo_pick_place.py [--out demo_out]

Outputs written to <out>/:
    canonical_program.json  — robot-neutral canonical IR (schema-validated)
    mg400_plan.json         — MG400-specific command plan (human-readable)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

# ---------------------------------------------------------------------------
# Auto-configure sys.path so the script is runnable without venv install.
# Script lives at tools/demo_lift/; repo root is two levels up.
# ---------------------------------------------------------------------------
_REPO = Path(__file__).resolve().parents[2]
for _pkg in (
    "src/robot_teaching_core",
    "src/adapters/mg400",
    "src/dobot_mg400/mg400_controller",
    "src/dobot_mg400/mg400_protocol",
):
    _p = str(_REPO / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from mg400_adapter import register_mg400_provider  # noqa: E402
from mg400_adapter.translator import plan_to_dict, translate_program  # noqa: E402
from teaching_core.lifter.segmenter import LifterConfig, lift_session  # noqa: E402
from teaching_core.lifter.session import SessionStream  # noqa: E402
from teaching_core.program.io import dump_program  # noqa: E402
from teaching_core.program.types import (  # noqa: E402
    OrientationIntent,
    ToolStep,
    WaitStep,
)


# ---------------------------------------------------------------------------
# Synthetic teaching session: approach → pick-point (dwell) → retreat
# ---------------------------------------------------------------------------

def _pick_place_session() -> SessionStream:
    """Four-joint stream mimicking a hand-guided pick motion.

    J1 sweeps 0 → 20 °, J2 dips 0 → -15 ° during approach.
    Robot holds at the pick point for 0.28 s (triggers dwell detection).
    Retreat brings J2 back to 0 while J1 stays at 20 °.

    Joint values in radians; all in-range for MG400.
    """
    pairs: list = []

    # approach — 5 samples, 0.04 s apart
    for i in range(5):
        t = i * 0.04
        j1 = math.radians(i * 5)        # 0 → 20 °
        j2 = math.radians(-i * 3.75)    # 0 → -15 °
        pairs.append((t, (j1, j2, 0.0, 0.0)))

    # dwell at pick point — 7 samples → 0.28 s hold (> dwell_window_s 0.20)
    for i in range(5, 12):
        t = i * 0.04
        pairs.append((t, (math.radians(20), math.radians(-15), 0.0, 0.0)))

    # retreat — J2 back to 0, J1 stays at 20 °, 5 samples
    for i in range(12, 17):
        t = i * 0.04
        j2 = math.radians(-15 + (i - 11) * 3)   # -15 → 0 °
        pairs.append((t, (math.radians(20), j2, 0.0, 0.0)))

    return SessionStream.from_pairs(pairs)


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

def build_pick_place_program(provider):
    """Lift the teaching session and wrap it into a complete pick-place program.

    Motion waypoints come from lift_session (robot FK via provider).
    Vacuum on/off and a settle wait are added around the motion steps,
    forming the kind of task program a real teaching pipeline would emit.
    """
    session = _pick_place_session()
    cfg = LifterConfig(
        program_id="mg400_pick_place_v0",
        capture_id="demo_synth_001",
        captured_at="2026-04-25T19:00:00Z",
        demonstrator="hand_authored",
        default_orientation_intent=OrientationIntent.YAW_ONLY,
        task_label="demo_pick_place",
    )
    motion_program = lift_session(session, provider=provider, config=cfg)

    # Assemble: vacuum on → approach/pick/retreat moves → settle → vacuum off.
    # CanonicalProgram is frozen; replace() creates a new instance.
    full_steps = (
        ToolStep(tool="vacuum", action="on"),
        *motion_program.steps,
        WaitStep(duration_ms=300),
        ToolStep(tool="vacuum", action="off"),
    )
    return replace(motion_program, steps=full_steps)


def run_demo(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    provider = register_mg400_provider()
    program = build_pick_place_program(provider)

    # --- artifact 1: canonical program (robot-neutral) ----------------------
    canonical_path = out_dir / "canonical_program.json"
    dump_program(program, canonical_path)

    step_summary = ", ".join(
        f"{s.kind}" + (f"[{s.motion.value}]" if hasattr(s, "motion") else "")
        for s in program.steps
    )
    print(f"[1/2] canonical program  →  {canonical_path}")
    print(f"      {len(program.steps)} steps: {step_summary}")

    # --- artifact 2: MG400 command plan ------------------------------------
    plan = translate_program(program)
    plan_dict = plan_to_dict(plan)
    mg400_path = out_dir / "mg400_plan.json"
    mg400_path.write_text(
        json.dumps(plan_dict, indent=2) + "\n", encoding="utf-8"
    )

    cmd_summary = ", ".join(c["kind"] for c in plan_dict["commands"])
    print(f"[2/2] MG400 plan         →  {mg400_path}")
    print(f"      {plan_dict['command_count']} commands: {cmd_summary}")


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Pick-place vertical slice demo: session → canonical → MG400 plan"
    )
    parser.add_argument(
        "--out", default="demo_out",
        help="Directory to write artifacts (default: demo_out/)",
    )
    args = parser.parse_args()
    run_demo(Path(args.out))
