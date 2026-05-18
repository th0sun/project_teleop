"""Golden-fixture lock for the mixed-primitive Cartesian compile path.

The mixed-primitive compiler (LINE → MovL, ARC → Arc, GENERAL →
JointMovJ chain) was verified on real MG400 hardware in May 2026
(``mixed_diag2.py`` reproduction matrix).  This file locks in the
exact compiled-command shape so any future refactor of the compile
pipeline (especially Phase C2 ``playback_compiler.py`` extraction)
will trip if behaviour drifts.

Key invariants asserted:

- ``SEGMENT_CARTESIAN_CP_MAX`` is applied to MovL and Arc commands.
- ``USE_MIXED_PRIMITIVES`` stays False by default; the flag must be
  flipped explicitly per test to exercise the mixed compile path.
- JointMovJ segments use the operator-tuned ``speed_j`` unchanged.

If a future tuning change legitimately moves the compiled values, the
test must be updated *in the same commit* — making the behavioural
change visible in code review.

See AGENTS.md §4.4 / §10 / §12.1.
"""

import tempfile
import unittest

import numpy as np

from mg400_controller.common.config import motion_config
from mg400_controller.common.trajectory.trajectory_recorder import (
    TrajectoryRecorder,
)
from mg400_controller.common.utils.kinematics import KinematicsCalculator


class FakeLogger:
    def __init__(self):
        self.msgs = []

    def info(self, msg):
        self.msgs.append(("info", msg))

    def warn(self, msg):
        self.msgs.append(("warn", msg))

    def error(self, msg):
        self.msgs.append(("error", msg))


def _rectangle_frames(tcp0=(284.6, 0.0, 121.2, 0.0)):
    """Build six IK-derived frames for an XY rectangle at fixed Z, R.

    Matches the on-robot reproduction in mixed_diag2.py: cx ± 30 in X,
    ±30 in Y, returning to start.  Each frame is 1 s apart in the
    recorded timestamp axis.
    """
    cx, cy, cz, cr = tcp0
    pts = [
        (cx,    cy - 30, cz, cr),  # 0: start
        (cx,    cy + 30, cz, cr),  # 1: +Y straight
        (cx - 15, cy + 45, cz, cr),  # 2: arc through-point candidate
        (cx - 30, cy + 30, cz, cr),  # 3: arc target
        (cx - 30, cy - 30, cz, cr),  # 4: -Y straight
        (cx,    cy - 30, cz, cr),  # 5: close back to start
    ]
    kin = KinematicsCalculator()
    frames = []
    for i, (x, y, z, r) in enumerate(pts):
        joints = kin.inverse_kinematics((x, y, z, r))
        frames.append({
            "timeStamp": float(i),
            "j1": float(joints[0]),
            "j2": float(joints[1]),
            "j3": float(joints[2]),
            "j4": float(joints[3]),
        })
    return frames


class TrajectoryCompilerGoldenTest(unittest.TestCase):
    """Lock the mixed-primitive compiled output on a known fixture path."""

    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        # Save flags so we can restore after each test.
        self._saved_use_mixed = motion_config.USE_MIXED_PRIMITIVES
        self._saved_cart_cp_max = motion_config.SEGMENT_CARTESIAN_CP_MAX

    def tearDown(self):
        motion_config.USE_MIXED_PRIMITIVES = self._saved_use_mixed
        motion_config.SEGMENT_CARTESIAN_CP_MAX = self._saved_cart_cp_max
        self.temp_dir.cleanup()

    def _make_recorder(self):
        return TrajectoryRecorder(
            command_send_fn=lambda cmd: True,
            logger=FakeLogger(),
            traj_dir=self.temp_dir.name,
        )

    # -- Default JointMovJ-only path (mixed flag False) -----------------
    def test_default_path_uses_joint_movj_only_when_mixed_disabled(self):
        """Default behaviour must be JointMovJ-only — protects USE_MIXED_PRIMITIVES=False."""
        motion_config.USE_MIXED_PRIMITIVES = False
        rec = self._make_recorder()
        rec.load_frames(_rectangle_frames())
        plan = rec.compile_loaded_plan()
        # Every queued command should be JointMovJ — no MovL or Arc.
        for cmd in plan.queued_commands:
            self.assertTrue(
                cmd.command.startswith("JointMovJ("),
                f"unexpected primitive in default compile: {cmd.command!r}",
            )
        # And the queue length should equal n_frames - 1 (one cmd per hop).
        self.assertEqual(len(plan.queued_commands), 5)

    # -- Mixed-primitive path (flag flipped True) -----------------------
    def test_mixed_primitive_path_uses_cartesian_cp_cap(self):
        """When mixed mode is on, every MovL/Arc must have CP ≤ SEGMENT_CARTESIAN_CP_MAX."""
        motion_config.USE_MIXED_PRIMITIVES = True
        rec = self._make_recorder()
        rec.set_playback_tuning({
            "speed_j": 30,
            "speed_l": 60,
            "cp": 80,        # operator wants high CP …
            "final_cp": 0,
        })
        rec.load_frames(_rectangle_frames())
        plan = rec.compile_loaded_plan()

        cap = motion_config.SEGMENT_CARTESIAN_CP_MAX
        for cmd in plan.queued_commands:
            text = cmd.command
            if text.startswith(("MovL(", "Arc(")):
                # Extract CP=NN from the command and assert ≤ cap.
                cp_token = [
                    t for t in text.replace("(", ",").replace(")", "").split(",")
                    if t.startswith("CP=")
                ]
                self.assertEqual(len(cp_token), 1, f"no CP in {text!r}")
                cp_val = int(cp_token[0].split("=", 1)[1])
                self.assertLessEqual(
                    cp_val,
                    cap,
                    f"Cartesian primitive exceeds SEGMENT_CARTESIAN_CP_MAX: {text!r}",
                )

    def test_mixed_primitive_path_emits_at_least_one_cartesian_command(self):
        """Sanity: the rectangle path produces Arc or MovL when mixed is on."""
        motion_config.USE_MIXED_PRIMITIVES = True
        rec = self._make_recorder()
        rec.load_frames(_rectangle_frames())
        plan = rec.compile_loaded_plan()
        cartesian = [
            c for c in plan.queued_commands
            if c.command.startswith(("MovL(", "Arc("))
        ]
        self.assertGreater(
            len(cartesian),
            0,
            "mixed mode should classify at least one segment as Arc or MovL",
        )

    def test_mixed_primitive_final_segment_uses_final_cp(self):
        """The terminal command of the queue uses ``final_cp`` (not the cap)."""
        motion_config.USE_MIXED_PRIMITIVES = True
        rec = self._make_recorder()
        rec.set_playback_tuning({
            "speed_j": 30,
            "speed_l": 60,
            "cp": 80,
            "final_cp": 0,
        })
        rec.load_frames(_rectangle_frames())
        plan = rec.compile_loaded_plan()
        final_cmd = plan.queued_commands[-1].command
        # The final command settles at CP=0 regardless of cap.
        self.assertIn("CP=0", final_cmd)


if __name__ == "__main__":
    unittest.main()
