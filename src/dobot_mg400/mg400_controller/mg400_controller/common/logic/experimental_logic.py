#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🧪 Experimental Control Logic Modes (TEMPORARY)

Three experimental strategies for sending motion commands,
derived from benchmark testing on MG400 Mock Server.

These modes REPLACE should_send_command() + format_command_string()
with their own decision + formatting logic.  The existing
TeleopController and MotionPlanner are NOT modified.

Usage from vr_teleop_node.py:
    if LOGIC_MODE != "default":
        strategy = ExperimentalStrategy(LOGIC_MODE, planner, logger)
        ...
        should_send, cmd_str, q_safe, info = strategy.process(latest_target, q_current, now)

After A/B testing, the winning strategy should be merged into
TeleopController and this file DELETED.
"""

import time
from collections import deque
from typing import Optional, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════
# Constants shared by all experimental modes
# ═══════════════════════════════════════════════════════════════

# Proximity / micro-motion (shared by all 3 modes)
_BASE_PROX_RAD = 0.050          # rad — proximity trigger base
_LOOKAHEAD_SEC = 0.30           # velocity-scaled lookahead
_SPATIAL_THRESHOLD = 0.0005     # rad — sub-noise filter
_MICRO_DEAD_RAD = np.radians(0.5)    # dead-zone
_MICRO_DELTA_RAD = np.radians(0.15)  # tiny-move filter
_LARGE_JUMP_RAD = np.radians(5.0)    # large-jump threshold


# ═══════════════════════════════════════════════════════════════
# Helper: Velocity estimation from position deltas
# ═══════════════════════════════════════════════════════════════

class _VelocityTracker:
    """EMA-filtered velocity from joint-position deltas (rad/s)."""

    def __init__(self, alpha: float = 0.3):
        self._alpha = alpha
        self._prev_q: Optional[np.ndarray] = None
        self._prev_t: float = 0.0
        self.ema_vel: float = 0.0

    def update(self, q: np.ndarray, t: float) -> float:
        if self._prev_q is not None and t - self._prev_t > 0.001:
            dt = t - self._prev_t
            raw = float(np.max(np.abs(q - self._prev_q))) / dt
            self.ema_vel = self._alpha * raw + (1.0 - self._alpha) * self.ema_vel
        self._prev_q = q.copy()
        self._prev_t = t
        return self.ema_vel


# ═══════════════════════════════════════════════════════════════
# Helper: Curvature computation with gate (used by M14, M15)
# ═══════════════════════════════════════════════════════════════

class _CurvatureTracker:
    """Gated curvature: returns 0 on sustained straight segments."""

    def __init__(self, alpha=0.40, gate_thresh_rad=np.radians(5.0),
                 gate_wait=3, buf_len=6):
        self._buf: deque = deque(maxlen=buf_len)
        self._alpha = alpha
        self._gate_thresh = gate_thresh_rad
        self._gate_wait = gate_wait
        self._smooth_ema: float = 0.0
        self._smooth_count: int = 0

    def update(self, q: np.ndarray) -> float:
        self._buf.append(q[:4].copy())
        if len(self._buf) < 3:
            return 0.0

        b = list(self._buf)
        v1 = b[-2] - b[-3]
        v2 = b[-1] - b[-2]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-8 or n2 < 1e-8:
            raw = 0.0
        else:
            cos_a = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
            raw = float(np.arccos(cos_a))  # rad

        self._smooth_ema = self._alpha * raw + (1.0 - self._alpha) * self._smooth_ema

        if raw < self._gate_thresh:
            self._smooth_count += 1
        else:
            self._smooth_count = 0

        if self._smooth_count >= self._gate_wait:
            return 0.0
        return self._smooth_ema

    def reset(self):
        self._buf.clear()
        self._smooth_ema = 0.0
        self._smooth_count = 0


# ═══════════════════════════════════════════════════════════════
# M11 Stable — Velocity-Clamped Integrator + Adaptive Rate Floor
# ═══════════════════════════════════════════════════════════════

class _M11Logic:
    """
    Velocity-clamped integrator with adaptive 10/25 Hz rate floor.
    - Integrates toward target at max VEL_CLAMP rad/s
    - Switches to 25 Hz when error is high
    - Proximity trigger for micro-motion accuracy
    """
    VEL_CLAMP = np.radians(250.0)   # rad/s
    RATE_NORMAL = 0.10              # 10 Hz
    RATE_HIGH = 0.04                # 25 Hz
    HIGH_ERR_THR = np.radians(20.0)
    LOW_ERR_THR = np.radians(10.0)

    def __init__(self):
        self._cmd_pos: Optional[np.ndarray] = None
        self._last_sent: Optional[np.ndarray] = None
        self._last_send_t: float = 0.0
        self._high_rate: bool = False
        self._vel = _VelocityTracker()

    def reset(self):
        self._cmd_pos = None
        self._last_sent = None
        self._last_send_t = 0.0
        self._high_rate = False

    def _integrate(self, target: np.ndarray, dt: float) -> np.ndarray:
        max_d = self.VEL_CLAMP * max(dt, 0.001)
        delta = target[:4] - self._cmd_pos
        return self._cmd_pos + np.clip(delta, -max_d, max_d)

    def process(self, q_target: np.ndarray, q_current: np.ndarray,
                now: float) -> Tuple[bool, Optional[np.ndarray], str]:
        """Returns (should_send, cmd_rad, reason)."""

        vel = self._vel.update(q_current, now)
        error = float(np.max(np.abs(q_target[:4] - q_current[:4])))

        # Bootstrap
        if self._cmd_pos is None:
            self._cmd_pos = q_current[:4].copy()
            self._last_send_t = now
            cmd = self._integrate(q_target, self.RATE_NORMAL)
            self._cmd_pos = cmd.copy()
            self._last_sent = cmd.copy()
            self._last_send_t = now
            return True, cmd, "Init"

        # Adaptive rate hysteresis
        if error > self.HIGH_ERR_THR:
            self._high_rate = True
        elif error < self.LOW_ERR_THR:
            self._high_rate = False
        rate_interval = self.RATE_HIGH if self._high_rate else self.RATE_NORMAL

        # Spatial threshold
        if self._last_sent is not None:
            change = float(np.max(np.abs(q_target[:4] - self._last_sent)))
            if change < _SPATIAL_THRESHOLD:
                return False, None, "SpatialSkip"

        # Micro dead-zone
        if (error < _MICRO_DEAD_RAD and self._last_sent is not None and
                float(np.max(np.abs(q_target[:4] - self._last_sent))) < _MICRO_DELTA_RAD):
            return False, None, "MicroSkip"

        # Proximity trigger
        if self._last_sent is not None:
            dist = float(np.max(np.abs(q_current[:4] - self._last_sent)))
            trigger = _BASE_PROX_RAD + vel * _LOOKAHEAD_SEC
            if dist < trigger:
                dt = max(now - self._last_send_t, rate_interval)
                cmd = self._integrate(q_target, dt)
                self._cmd_pos = cmd.copy()
                self._last_sent = cmd.copy()
                self._last_send_t = now
                return True, cmd, "Prox"

        # Rate floor
        if (now - self._last_send_t) >= rate_interval:
            dt = now - self._last_send_t
            cmd = self._integrate(q_target, dt)
            self._cmd_pos = cmd.copy()
            self._last_sent = cmd.copy()
            self._last_send_t = now
            return True, cmd, "Floor" + ("H" if self._high_rate else "")

        return False, None, "Wait"


# ═══════════════════════════════════════════════════════════════
# M14 Smooth — Adaptive Rate + Curvature Gate + Feedforward
# ═══════════════════════════════════════════════════════════════

class _M14Logic:
    """
    Curvature-gated adaptive rate with kinematic feedforward (80ms lead).
    Best for smooth continuous curves (sine, circle).
    """
    K_RATE = 1.2
    K_DOT = 0.15
    K_CURVE = 0.020
    EPS = np.radians(0.30)
    MIN_INTERVAL = 0.020   # 50 Hz
    MAX_INTERVAL = 0.120   # 8.3 Hz

    VEL_CLAMP = np.radians(300.0)

    # Feedforward
    LEAD_TIME = 0.080      # seconds
    MAX_LEAD = np.radians(5.0)

    def __init__(self):
        self._last_sent: Optional[np.ndarray] = None
        self._last_send_t: float = 0.0
        self._prev_error: float = 0.0
        self._last_state_t: float = 0.0
        self._vel = _VelocityTracker()
        self._curve = _CurvatureTracker()

        # Feedforward state
        self._prev_tgt: Optional[np.ndarray] = None
        self._prev_tgt_t: float = 0.0
        self._ema_tgt_vel: Optional[np.ndarray] = None

    def reset(self):
        self._last_sent = None
        self._last_send_t = 0.0
        self._prev_error = 0.0
        self._last_state_t = 0.0
        self._curve.reset()
        self._prev_tgt = None
        self._prev_tgt_t = 0.0
        self._ema_tgt_vel = None

    def _feedforward(self, raw: np.ndarray, now: float) -> np.ndarray:
        dt = now - self._prev_tgt_t
        if self._prev_tgt is None or dt < 0.001:
            self._prev_tgt = raw.copy()
            self._prev_tgt_t = now
            if self._ema_tgt_vel is None:
                self._ema_tgt_vel = np.zeros(4)
            return raw.copy()

        raw_vel = (raw[:4] - self._prev_tgt[:4]) / dt
        if self._ema_tgt_vel is None:
            self._ema_tgt_vel = raw_vel.copy()
        else:
            self._ema_tgt_vel = 0.3 * raw_vel + 0.7 * self._ema_tgt_vel

        self._prev_tgt = raw.copy()
        self._prev_tgt_t = now

        lead = self._ema_tgt_vel * self.LEAD_TIME
        norm = float(np.max(np.abs(lead)))
        if norm > self.MAX_LEAD:
            lead = lead * (self.MAX_LEAD / norm)
        return raw[:4] + lead

    def _interval(self, error: float, error_dot: float, curve: float) -> float:
        dot_s = min(abs(error_dot), np.radians(50.0))
        denom = error + self.K_DOT * dot_s + self.K_CURVE * curve + self.EPS
        return float(np.clip(self.K_RATE / denom, self.MIN_INTERVAL, self.MAX_INTERVAL))

    def _clamp(self, cmd: np.ndarray, dt: float):
        if self._last_sent is None:
            return cmd.copy(), False
        max_d = self.VEL_CLAMP * max(dt, 0.001)
        delta = cmd[:4] - self._last_sent
        clamped = bool(np.any(np.abs(delta) > max_d))
        if clamped:
            cmd = self._last_sent + np.clip(delta, -max_d, max_d)
        return cmd, clamped

    def _commit(self, cmd, now):
        self._last_sent = cmd[:4].copy()
        self._last_send_t = now

    def process(self, q_target: np.ndarray, q_current: np.ndarray,
                now: float) -> Tuple[bool, Optional[np.ndarray], str]:

        vel = self._vel.update(q_current, now)
        raw_tgt = q_target[:4].copy()

        # Curvature + feedforward
        self._curve.update(raw_tgt)
        ff_tgt = self._feedforward(raw_tgt, now)

        error = float(np.max(np.abs(raw_tgt - q_current[:4])))
        dt_state = max(now - self._last_state_t, 0.001)
        error_dot = (error - self._prev_error) / dt_state
        self._prev_error = error
        self._last_state_t = now

        eff_curve = self._curve._smooth_ema if self._curve._smooth_count < self._curve._gate_wait else 0.0
        interval = self._interval(error, error_dot, eff_curve)

        # Bootstrap
        if self._last_sent is None:
            cmd, _ = self._clamp(ff_tgt, interval)
            self._commit(cmd, now)
            return True, cmd, "Init"

        jump = float(np.max(np.abs(ff_tgt - self._last_sent)))
        dt_send = now - self._last_send_t

        # Large jump path
        if jump > np.radians(5.0):
            dist = float(np.max(np.abs(q_current[:4] - self._last_sent)))
            trigger = _BASE_PROX_RAD + vel * _LOOKAHEAD_SEC
            if dist < trigger:
                cmd, cl = self._clamp(ff_tgt, dt_send)
                self._commit(cmd, now)
                return True, cmd, "Prox_LJ"
            if dt_send >= interval:
                cmd, cl = self._clamp(ff_tgt, dt_send)
                self._commit(cmd, now)
                return True, cmd, "LJump"
            return False, None, "Wait"

        # Spatial filter
        if jump < _SPATIAL_THRESHOLD:
            return False, None, "SpatialSkip"

        # Micro filter
        if error < _MICRO_DEAD_RAD and jump < _MICRO_DELTA_RAD:
            return False, None, "MicroSkip"

        # Proximity trigger
        dist = float(np.max(np.abs(q_current[:4] - self._last_sent)))
        trigger = _BASE_PROX_RAD + vel * _LOOKAHEAD_SEC
        if dist < trigger:
            cmd, cl = self._clamp(ff_tgt, dt_send)
            self._commit(cmd, now)
            return True, cmd, "Prox"

        # Adaptive rate
        if dt_send >= interval:
            cmd, cl = self._clamp(ff_tgt, dt_send)
            self._commit(cmd, now)
            return True, cmd, "Rate"

        return False, None, "Wait"


# ═══════════════════════════════════════════════════════════════
# M15 Sharp — Adaptive Rate + Curvature Gate + Strict Vel Clamp
# ═══════════════════════════════════════════════════════════════

class _M15Logic:
    """
    Curvature-gated adaptive rate with strict velocity clamp (260°/s).
    No feedforward. Best for sharp corners (square, zigzag).
    """
    K_RATE = 1.2
    K_DOT = 0.15
    K_CURVE = 0.020
    EPS = np.radians(0.30)
    MIN_INTERVAL = 0.025   # 40 Hz
    MAX_INTERVAL = 0.120   # 8.3 Hz

    VEL_CLAMP = np.radians(260.0)   # strict clamp

    # Gate thresholds (tighter than M14)
    GATE_THRESH = np.radians(2.0)
    GATE_WAIT = 5

    def __init__(self):
        self._last_sent: Optional[np.ndarray] = None
        self._last_send_t: float = 0.0
        self._prev_error: float = 0.0
        self._last_state_t: float = 0.0
        self._vel = _VelocityTracker()
        self._curve = _CurvatureTracker(gate_thresh_rad=self.GATE_THRESH,
                                         gate_wait=self.GATE_WAIT)

    def reset(self):
        self._last_sent = None
        self._last_send_t = 0.0
        self._prev_error = 0.0
        self._last_state_t = 0.0
        self._curve.reset()

    def _interval(self, error, error_dot, curve):
        dot_s = min(abs(error_dot), np.radians(50.0))
        denom = error + self.K_DOT * dot_s + self.K_CURVE * curve + self.EPS
        return float(np.clip(self.K_RATE / denom, self.MIN_INTERVAL, self.MAX_INTERVAL))

    def _clamp(self, cmd, dt):
        if self._last_sent is None:
            return cmd[:4].copy(), False
        max_d = self.VEL_CLAMP * max(dt, 0.001)
        delta = cmd[:4] - self._last_sent
        clamped = bool(np.any(np.abs(delta) > max_d))
        if clamped:
            cmd = self._last_sent + np.clip(delta, -max_d, max_d)
        return cmd, clamped

    def _commit(self, cmd, now):
        self._last_sent = cmd[:4].copy()
        self._last_send_t = now

    def process(self, q_target, q_current, now):
        vel = self._vel.update(q_current, now)
        tgt = q_target[:4].copy()

        self._curve.update(tgt)
        error = float(np.max(np.abs(tgt - q_current[:4])))
        dt_state = max(now - self._last_state_t, 0.001)
        error_dot = (error - self._prev_error) / dt_state
        self._prev_error = error
        self._last_state_t = now

        eff_curve = self._curve._smooth_ema if self._curve._smooth_count < self._curve._gate_wait else 0.0
        interval = self._interval(error, error_dot, eff_curve)

        if self._last_sent is None:
            cmd, _ = self._clamp(tgt, interval)
            self._commit(cmd, now)
            return True, cmd, "Init"

        jump = float(np.max(np.abs(tgt - self._last_sent)))
        dt_send = now - self._last_send_t

        if jump > np.radians(5.0):
            dist = float(np.max(np.abs(q_current[:4] - self._last_sent)))
            trigger = _BASE_PROX_RAD + vel * _LOOKAHEAD_SEC
            if dist < trigger:
                cmd, _ = self._clamp(tgt, dt_send)
                self._commit(cmd, now)
                return True, cmd, "Prox_LJ"
            if dt_send >= interval:
                cmd, _ = self._clamp(tgt, dt_send)
                self._commit(cmd, now)
                return True, cmd, "LJump"
            return False, None, "Wait"

        if jump < _SPATIAL_THRESHOLD:
            return False, None, "SpatialSkip"
        if error < _MICRO_DEAD_RAD and jump < _MICRO_DELTA_RAD:
            return False, None, "MicroSkip"

        dist = float(np.max(np.abs(q_current[:4] - self._last_sent)))
        trigger = _BASE_PROX_RAD + vel * _LOOKAHEAD_SEC
        if dist < trigger:
            cmd, _ = self._clamp(tgt, dt_send)
            self._commit(cmd, now)
            return True, cmd, "Prox"

        if dt_send >= interval:
            cmd, _ = self._clamp(tgt, dt_send)
            self._commit(cmd, now)
            return True, cmd, "Rate"

        return False, None, "Wait"


# ═══════════════════════════════════════════════════════════════
# Public API — ExperimentalStrategy (used by vr_teleop_node.py)
# ═══════════════════════════════════════════════════════════════

class ExperimentalStrategy:
    """
    Wraps M11/M14/M15 logic into a single interface compatible
    with the main control loop in vr_teleop_node.py.

    Usage:
        strategy = ExperimentalStrategy("m11", planner, logger)
        should_send, cmd_str, q_rad, info = strategy.process(
            q_target_rad, q_current_rad, now
        )
    """

    MODE_MAP = {
        "m11": ("M11_Stable", _M11Logic),
        "m14": ("M14_Smooth", _M14Logic),
        "m15": ("M15_Sharp", _M15Logic),
    }

    def __init__(self, mode_key: str, planner, logger):
        if mode_key not in self.MODE_MAP:
            raise ValueError(f"Unknown experimental mode: {mode_key!r}. "
                             f"Use one of: {list(self.MODE_MAP.keys())}")
        self.mode_name, logic_cls = self.MODE_MAP[mode_key]
        self._logic = logic_cls()
        self._planner = planner   # MotionPlanner — for format_command()
        self._logger = logger
        self._cmd_times: deque = deque(maxlen=60)

    def reset(self):
        self._logic.reset()

    def process(self, q_target_rad: np.ndarray, q_current_rad: np.ndarray,
                now: float) -> Tuple[bool, Optional[str], Optional[np.ndarray], str]:
        """
        Run one cycle of the experimental logic.

        Args:
            q_target_rad: Latest target joints in radians (4,)
            q_current_rad: Current robot joints in radians (4,)
            now: Current time (seconds)

        Returns:
            (should_send, cmd_string, q_safe_rad, reason)
            cmd_string is a ready-to-send TCP command string (e.g. "JointMovJ(...)")
            q_safe_rad is the actual joint target that was sent (may differ from input due to clamping)
        """
        should_send, cmd_rad, reason = self._logic.process(q_target_rad, q_current_rad, now)

        if not should_send or cmd_rad is None:
            return False, None, None, reason

        # Format using the existing planner (preserves control_mode setting)
        cmd_str = self._planner.format_command(cmd_rad, speed_percent=100)

        # Hz tracking
        self._cmd_times.append(now)
        hz = 0
        if len(self._cmd_times) > 2:
            span = self._cmd_times[-1] - self._cmd_times[0]
            hz = int((len(self._cmd_times) - 1) / max(span, 0.001))

        info = f"[{self.mode_name}/{reason}] Hz={hz}"
        return True, cmd_str, cmd_rad, info
