"""Identity-keyed Unity teleop sample registry.

Every Unity teleop sample arrives stamped with a ``(session_id,
unity_seq_id)`` pair. Holding the recent samples in an LRU cache keyed
on that pair gives the node a way to correlate any later message (a
joint command, a feedback ack, a logged event) back to the exact Unity
sample that caused it, without having to fall back to "whichever
sample arrived nearest in time."

The class also tracks the latest receive time so the node can decide
whether session logging is still meaningful (Unity samples can go
stale while the joint-cmd channel keeps flowing).

Today the live consumer is the unified triple logger via
``latest_recv_time``. The identity LRU is kept populated so future
exact-identity matchers (e.g. settling pending commands by the
originating Unity frame) have a stable cache to read from without
needing a separate ingest path.
"""
from __future__ import annotations

from typing import Optional, Tuple


_LRU_LIMIT = 256


class UnitySampleMatcher:
    """Owns the latest-sample timestamp and the identity-keyed LRU
    cache. State and methods that used to live on ``TeleopNode`` as
    ``self.latest_unity_sample``, ``self.unity_samples_by_identity``,
    and ``self._remember_unity_sample``.
    """

    def __init__(self):
        self.latest: Optional[object] = None
        self.latest_recv_time: float = 0.0
        # key: (session_id, unity_seq_id) -> (sample, recv_time)
        self._by_identity: dict[Tuple[str, str], tuple] = {}

    def remember(self, sample, recv_time: float) -> None:
        """Update the latest-seen tracker and the identity LRU.

        Evicts the oldest entry when the cache grows past
        ``_LRU_LIMIT`` so the registry can't grow unbounded under
        sustained Unity traffic.
        """
        self.latest = sample
        self.latest_recv_time = recv_time
        self._by_identity[(sample.session_id, sample.unity_seq_id)] = (
            sample,
            recv_time,
        )
        if len(self._by_identity) <= _LRU_LIMIT:
            return
        oldest_key = min(
            self._by_identity,
            key=lambda key: self._by_identity[key][1],
        )
        self._by_identity.pop(oldest_key, None)

    def clear_identity_cache(self) -> None:
        """Drop every cached sample. Called when the Unity session id
        changes (Unity restart, scene reload) so a new session can't
        accidentally collide with an old session's seq ids.
        """
        self._by_identity.clear()

    def lookup(self, session_id: str, unity_seq_id: str):
        """Return ``(sample, recv_time)`` for the given identity, or
        ``None`` if no matching sample is in the cache.
        """
        return self._by_identity.get((session_id, unity_seq_id))
