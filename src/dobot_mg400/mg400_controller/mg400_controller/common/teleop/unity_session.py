"""Unity input firehose handling: pong heartbeat, JSON-sample
ingest, and session-id transitions.

Owns the queue + worker thread that decouples the ROS callback (which
must return fast or the executor backs up) from the heavier
parse/log/match work that runs in its own thread.

Wiring from the node:

  self.unity_session = UnitySessionHandler(
      logger=self.get_logger(),
      stop_event=self.stop_event,
      sample_matcher=self.sample_matcher,
      get_triple_logger=lambda: self.triple_logger,
      on_session_changed=self._reset_pending_command_registry,
  )
  self.unity_session.start_worker()

  # Subscriptions:
  pong_callback = self.unity_session.pong_callback
  unity_teleop_sample_callback = self.unity_session.sample_callback
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Optional

from mg400_controller.common.config import motion_config
from mg400_controller.common.ros.unity_teleop_sample import (
    UnityTeleopSampleError,
    parse_unity_teleop_sample,
)


class UnitySessionHandler:
    """Drains /unity/teleop_sample into the triple logger + sample
    matcher, tracks the Unity session id, and exposes a no-op pong
    handler so Unity's RTT publishes don't error on missing subscriber.

    Designed so the owning node hands over the *whole* Unity firehose
    by binding three callbacks (pong, sample) and one worker.
    """

    def __init__(
        self,
        *,
        logger,
        stop_event: threading.Event,
        sample_matcher,
        get_triple_logger: Callable[[], object],
        on_session_changed: Callable[[], None],
    ):
        self._log = logger
        self._stop_event = stop_event
        self._sample_matcher = sample_matcher
        # triple_logger may not exist at construction time; resolved
        # via the getter at message-arrival time.
        self._get_triple_logger = get_triple_logger
        self._on_session_changed = on_session_changed

        # Public counters (for diagnostics / future shutdown logs).
        self.enqueued_count = 0
        self.processed_count = 0

        self._queue: "queue.SimpleQueue[tuple[float, str]]" = queue.SimpleQueue()
        self._current_session_id: Optional[str] = None
        self._last_sample_warning: float = 0.0
        self._worker_thread: Optional[threading.Thread] = None

    # ── ROS subscription callbacks ─────────────────────────────────────

    def pong_callback(self, msg) -> None:
        """``/unity/pong`` (String "ros_ping_ns,unity_ts_sec") — no-op.

        Earlier this estimated ``unity_offset = unity_ts - (ros_ping
        + rtt/2)``, but ClockCalibrator's min-window method proved
        more robust against jitter and is the active offset source.
        Subscription kept so Unity's pong publishes don't error, and
        so future work can plug an estimator back in here.
        """
        return

    def sample_callback(self, msg) -> None:
        """``/unity/teleop_sample`` (String JSON) — fast path.

        Captures wall-time recv stamp + raw payload and returns
        immediately; the heavy parse/log/match work happens in the
        worker thread.
        """
        recv_time = time.time()
        self.enqueued_count += 1
        self._queue.put((recv_time, msg.data))

    # ── Worker lifecycle ───────────────────────────────────────────────

    def start_worker(self) -> None:
        """Spawn the daemon thread that drains the sample queue."""
        if self._worker_thread is not None:
            return
        self._worker_thread = threading.Thread(
            target=self._worker_loop, daemon=True
        )
        self._worker_thread.start()

    def _worker_loop(self) -> None:
        """Block on the queue with a 100 ms timeout so the loop can
        notice ``stop_event`` and exit cleanly. Drains queued samples
        even after stop is set so we don't lose tail samples on
        shutdown.
        """
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                recv_time, payload = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.process_payload(payload, recv_time)
                self.processed_count += 1
            except Exception as exc:
                self._log.error(f"Unity teleop sample worker error: {exc}")

    # ── Payload processing ─────────────────────────────────────────────

    def process_payload(
        self,
        payload: str,
        now_ros_sec: float,
        *,
        log_sample: bool = True,
        remember_sample: bool = True,
        handle_session: bool = True,
    ) -> None:
        """Parse one Unity sample and dispatch the side effects.

        ``/unity/teleop_sample`` never drives the robot directly;
        ``/unity/joint_cmd`` is the live robot-control topic. This
        sample is for logging + correlating that joint command back
        to Unity-side controller / IK context.
        """
        try:
            sample = parse_unity_teleop_sample(payload)
        except UnityTeleopSampleError as exc:
            if now_ros_sec - self._last_sample_warning > 1.0:
                self._log.warn(f"Invalid /unity/teleop_sample ignored: {exc}")
                self._last_sample_warning = now_ros_sec
            return

        if handle_session:
            self._handle_session_transition(sample)
        if remember_sample:
            self._sample_matcher.remember(sample, now_ros_sec)

        if log_sample:
            triple_logger = self._get_triple_logger()
            if triple_logger is not None:
                triple_logger.log_unity_sample(sample, now_ros_sec)

    def _handle_session_transition(self, sample) -> None:
        """When Unity sends a sample with a new ``session_id``, drop the
        identity-keyed sample cache and ask the node to reset the
        pending-command registry so old-session keys can't collide
        with new ones.
        """
        session_id = sample.session_id
        if self._current_session_id is None:
            self._current_session_id = session_id
            return
        if session_id == self._current_session_id:
            return

        self._current_session_id = session_id
        self._sample_matcher.clear_identity_cache()
        self._on_session_changed()

    # ── Logging-gate hook (control loop reads this every tick) ─────────

    def session_logging_active(self, now_wall: float) -> bool:
        """Unconditionally active in this build.

        Kept as a method so the control loop reads as a guarded
        check, and so a future "log only when a fresh Unity sample
        exists within N ms" rule can attach here without touching
        any caller.
        """
        # Hook for future freshness gating; keep ``motion_config`` import
        # available so the rule lives in one place when it returns.
        _ = motion_config  # noqa: F841 — referenced for future implementation
        _ = now_wall
        return True
