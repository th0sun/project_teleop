"""Lifter input model: a robot-neutral session stream.

The lifter's input is a sequence of timestamped joint vectors. MCAP
parsing lives in ``capture/`` (later PR); the lifter itself does not
know whether the bytes came from MCAP, JSON, or an in-memory queue.
This keeps the segmenter testable without a ROS install.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Iterator, Sequence, Tuple


@dataclass(frozen=True)
class SessionFrame:
    """A single demonstration sample.

    ``q_rad`` length must match the kinematics provider's ``dof``.
    The lifter validates this on first frame.
    """

    timestamp_s: float
    q_rad: Tuple[float, ...]


class SessionStream:
    """Iterable wrapper over :class:`SessionFrame` records.

    Pure data; no IO. Lifter callers materialize a stream from MCAP,
    a list of ``(t, q)`` tuples, or any other source.
    """

    def __init__(self, frames: Iterable[SessionFrame]) -> None:
        # Eagerly materialize to a tuple so the stream can be iterated
        # multiple times if a future caller needs to.
        self._frames: Tuple[SessionFrame, ...] = tuple(frames)

    @classmethod
    def from_pairs(
        cls, pairs: Iterable[Tuple[float, Sequence[float]]],
    ) -> "SessionStream":
        """Convenience: build from ``(t, [q1, q2, ...])`` pairs."""
        return cls(
            SessionFrame(timestamp_s=float(t), q_rad=tuple(float(v) for v in q))
            for t, q in pairs
        )

    def __iter__(self) -> Iterator[SessionFrame]:
        return iter(self._frames)

    def __len__(self) -> int:
        return len(self._frames)

    @property
    def frames(self) -> Tuple[SessionFrame, ...]:
        return self._frames
