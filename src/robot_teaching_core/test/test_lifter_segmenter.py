"""Lifter v0 segmentation + provider-injection tests.

Verifies that:
- the lifter is robot-neutral (uses an injected, in-test FK provider);
- segmentation emits start + end + each detected dwell;
- ``orientation_intent`` is always written (R15);
- the resulting program round-trips through schema validation;
- the lifter rejects DOF mismatches against the provider.
"""

from __future__ import annotations

import unittest
from typing import Optional, Sequence

from teaching_core.kinematics.provider import (
    KinematicsProvider,
    PoseTuple,
)
from teaching_core.lifter.segmenter import (
    LifterConfig,
    SegmentationError,
    lift_session,
)
from teaching_core.lifter.session import SessionStream
from teaching_core.program.io import program_to_dict
from teaching_core.program.schema import validate_program_dict
from teaching_core.program.types import (
    OrientationIntent,
    MoveStep,
)


class _LinearFKProvider:
    """Robot-neutral FK stub: position = (q0, q1, q2); identity quaternion.

    Useful for lifter tests because it makes pose values trivially
    predictable from joint values without depending on any concrete
    robot URDF.
    """

    def __init__(self, dof: int = 4) -> None:
        self.provider_id = "test_linear_fk"
        self.dof = dof
        self.base_frame = "test_base"
        self.tcp_frame = "test_tcp"

    def fk(self, q_rad: Sequence[float]) -> PoseTuple:
        # Use first three joints as XYZ to keep the test pure.
        x, y, z = (float(q_rad[0]), float(q_rad[1]), float(q_rad[2]))
        return ((x, y, z), (0.0, 0.0, 0.0, 1.0))

    def solve_ik(
        self,
        target_pose: PoseTuple,
        seed_rad: Optional[Sequence[float]] = None,
    ):
        raise NotImplementedError  # not needed for lifter v0


def _config(**overrides) -> LifterConfig:
    base = dict(
        program_id="lift_test_001",
        capture_id="cap_synth_001",
        captured_at="2026-04-25T19:00:00Z",
    )
    base.update(overrides)
    return LifterConfig(**base)


class TestSegmentation(unittest.TestCase):

    def test_runtime_check_provider_protocol(self) -> None:
        prov = _LinearFKProvider()
        self.assertIsInstance(prov, KinematicsProvider)

    def test_constant_pose_yields_two_endpoints(self) -> None:
        # Joints never change; segmenter still emits start + end.
        stream = SessionStream.from_pairs(
            (i * 0.1, (1.0, 2.0, 0.5, 0.0)) for i in range(10)
        )
        program = lift_session(
            stream, provider=_LinearFKProvider(), config=_config()
        )
        moves = [s for s in program.steps if isinstance(s, MoveStep)]
        self.assertGreaterEqual(len(moves), 2)
        for m in moves:
            self.assertEqual(m.pose.position_m, (1.0, 2.0, 0.5))

    def test_settle_inserts_intermediate_waypoint(self) -> None:
        # Move, hold for ~0.3 s, move. Expect 3+ waypoints.
        pairs = []
        for i in range(5):
            pairs.append((i * 0.05, (i * 0.1, 0.0, 0.0, 0.0)))
        # hold
        for i in range(5, 12):
            pairs.append((i * 0.05, (0.4, 0.0, 0.0, 0.0)))
        # move again
        for i in range(12, 18):
            pairs.append((i * 0.05, (0.4 + (i - 11) * 0.1, 0.0, 0.0, 0.0)))
        stream = SessionStream.from_pairs(pairs)
        program = lift_session(
            stream, provider=_LinearFKProvider(), config=_config(),
        )
        positions = [s.pose.position_m[0] for s in program.steps]
        self.assertGreaterEqual(len(positions), 3)
        self.assertEqual(positions[0], 0.0)
        self.assertAlmostEqual(positions[-1], pairs[-1][1][0])
        # at least one settle near 0.4 between start and end
        self.assertTrue(any(abs(p - 0.4) < 1e-9 for p in positions[1:-1]))

    def test_orientation_intent_is_always_written(self) -> None:
        stream = SessionStream.from_pairs(
            (i * 0.1, (i * 0.1, 0.0, 0.0, 0.0)) for i in range(5)
        )
        program = lift_session(
            stream,
            provider=_LinearFKProvider(),
            config=_config(default_orientation_intent=OrientationIntent.YAW_ONLY),
        )
        for s in program.steps:
            self.assertIsInstance(s, MoveStep)
            self.assertEqual(s.orientation_intent, OrientationIntent.YAW_ONLY)

    def test_program_validates_under_schema(self) -> None:
        stream = SessionStream.from_pairs(
            (i * 0.1, (i * 0.05, 0.0, 0.0, 0.0)) for i in range(8)
        )
        program = lift_session(
            stream, provider=_LinearFKProvider(), config=_config()
        )
        validate_program_dict(program_to_dict(program))

    def test_dof_mismatch_rejected(self) -> None:
        stream = SessionStream.from_pairs(
            ((0.0, (0.0, 0.0, 0.0)), (0.1, (0.1, 0.0, 0.0)))
        )
        with self.assertRaises(SegmentationError):
            lift_session(
                stream, provider=_LinearFKProvider(dof=4), config=_config()
            )

    def test_empty_stream_rejected(self) -> None:
        with self.assertRaises(SegmentationError):
            lift_session(
                SessionStream.from_pairs(()),
                provider=_LinearFKProvider(),
                config=_config(),
            )

    def test_non_monotonic_timestamps_rejected(self) -> None:
        stream = SessionStream.from_pairs(
            [
                (0.0, (0.0, 0.0, 0.0, 0.0)),
                (0.2, (0.1, 0.0, 0.0, 0.0)),
                (0.1, (0.2, 0.0, 0.0, 0.0)),
            ]
        )
        with self.assertRaises(SegmentationError):
            lift_session(
                stream, provider=_LinearFKProvider(), config=_config()
            )


if __name__ == "__main__":
    unittest.main()
