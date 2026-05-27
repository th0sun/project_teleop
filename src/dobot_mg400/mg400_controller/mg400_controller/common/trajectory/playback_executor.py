#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Playback-executor leaf helpers extracted from TrajectoryRecorder.

The recorder still owns the ``start_preview`` / ``_play_worker`` loop
because they are tightly bound to recorder state (stop flag, timer
list, callbacks, kinematics-feedback fns).  This module hosts the
*leaf* helpers that have no such coupling so they can be unit-tested
and reused without instantiating a recorder:

- ``flush_motion_queue_after_timeout`` — issue ResetRobot+EnableRobot
  on the dashboard channel.
- ``send_playback_event_command`` — best-effort dashboard send with a
  guarded warning when no dashboard channel exists.
- ``cancel_pending_timers`` — cancel every ``threading.Timer`` in a
  list under a lock and clear the list.
- ``playback_timeout_margin`` — derive the absolute timeout margin
  for the playback loop from the queued-command count.
- ``playback_timed_out`` — predicate on top of ``playback_timeout_margin``.

Behaviour matches the pre-extraction methods exactly.  The recorder's
private method names remain as thin facades for back-compat.
"""

from __future__ import annotations

import threading
from typing import Callable, List, Optional


# Re-exported here to keep the recorder facade able to default the
# constants without importing from playback_executor twice.
PREVIEW_ABSOLUTE_TIMEOUT_SEC = 10.0
PREVIEW_TIMEOUT_PER_COMMAND_SEC = 0.35
PREVIEW_MAX_TIMEOUT_MARGIN_SEC = 60.0


def flush_motion_queue_after_timeout(
    send_dash_fn: Optional[Callable[[str], bool]],
    sleep_fn: Callable[[float], None],
) -> bool:
    """ResetRobot + EnableRobot on the dashboard channel.

    Returns False if no dashboard channel is wired or if either send
    raises — matching the pre-extraction method's swallow-and-return
    contract.
    """
    if send_dash_fn is None:
        return False
    # Local imports keep this module light at import time and avoid a
    # circular if mg400_protocol expands its surface later.
    from mg400_protocol.dashboard import enable_robot, reset_robot
    try:
        send_dash_fn(reset_robot().render())
        sleep_fn(0.2)
        send_dash_fn(enable_robot().render())
        return True
    except Exception:
        return False


def send_playback_event_command(
    command: str,
    send_dash_fn: Optional[Callable[[str], bool]],
    logger,
) -> bool:
    """Best-effort dashboard send used by teach event dispatch."""
    if send_dash_fn is None:
        logger.warn(
            f"⚠️  Cannot send playback IO event without dashboard channel: {command}"
        )
        return False
    return bool(send_dash_fn(command))


def cancel_pending_timers(
    pending_timers: List[threading.Timer],
    pending_timers_lock: threading.Lock,
) -> None:
    """Cancel every queued ``Timer`` and clear the list under a lock."""
    with pending_timers_lock:
        for timer in pending_timers:
            timer.cancel()
        pending_timers.clear()


def playback_timeout_margin(
    command_count: Optional[int] = None,
    *,
    absolute_sec: float = PREVIEW_ABSOLUTE_TIMEOUT_SEC,
    per_command_sec: float = PREVIEW_TIMEOUT_PER_COMMAND_SEC,
    max_margin_sec: float = PREVIEW_MAX_TIMEOUT_MARGIN_SEC,
) -> float:
    """Return the absolute timeout margin used by ``playback_timed_out``."""
    margin = absolute_sec
    if command_count is not None:
        margin = max(margin, float(command_count) * per_command_sec)
    return min(margin, max_margin_sec)


def playback_timed_out(
    elapsed: float,
    total_dur: float,
    command_count: Optional[int] = None,
    *,
    absolute_sec: float = PREVIEW_ABSOLUTE_TIMEOUT_SEC,
    per_command_sec: float = PREVIEW_TIMEOUT_PER_COMMAND_SEC,
    max_margin_sec: float = PREVIEW_MAX_TIMEOUT_MARGIN_SEC,
) -> bool:
    """Predicate: has the playback loop exceeded its allowed wall-clock budget?"""
    return elapsed >= total_dur + playback_timeout_margin(
        command_count,
        absolute_sec=absolute_sec,
        per_command_sec=per_command_sec,
        max_margin_sec=max_margin_sec,
    )
