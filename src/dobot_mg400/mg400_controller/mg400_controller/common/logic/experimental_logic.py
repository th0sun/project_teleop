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
        strategy = ExperimentalStrategy(LOGIC_MODE, CONTROL_MODE, logger)
        ...
        should_send, cmd_str, q_safe, info = strategy.process(raw_target, q_current, now)

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
# M8 Raw — Blind interval-based sending (No logic, adjustable Hz)
# ═══════════════════════════════════════════════════════════════

class _RawLogic:
    """
    Fixed-rate raw stream without clamping, prediction, or proximity gating.
    Reads frequency dynamically from robot_config.RAW_HZ.
    """
    def __init__(self):
        self._last_send_t: float = 0.0

    def reset(self):
        self._last_send_t = 0.0

    def process(self, q_target: np.ndarray, q_current: np.ndarray,
                now: float) -> Tuple[bool, Optional[np.ndarray], str]:
        import mg400_controller.common.config.robot_config as cfg
        
        interval = 1.0 / max(cfg.RAW_HZ, 1)

        # Use a small tolerance (e.g. 5ms) to prevent timer aliasing with the 50Hz main loop
        if self._last_send_t == 0.0 or (now - self._last_send_t) >= (interval - 0.005):
            self._last_send_t = now
            return True, q_target[:4].copy(), f"Raw_Timer ({cfg.RAW_HZ}Hz)"

        return False, None, "Wait"


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
        if (now - self._last_send_t) >= (rate_interval - 0.005):
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
        if dt_send >= (interval - 0.005):
            cmd, clamped = self._clamp(ff_tgt, dt_send)
            self._commit(cmd, now)
            return True, cmd, "Floor_C" if clamped else "Floor"

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
# M16 AdaptCP — LPF + Feedforward + Step-Limit + Adaptive CP
# ═══════════════════════════════════════════════════════════════

class _M16Logic:
    """
    M16_AdaptCP: Physics-aware teleoperation mode.

    Signal pipeline (per 50 ms tick):
      1. LPF(α=0.25) on raw Unity target        → smoothed noise
      2. Feedforward (70 ms lead) on smoothed   → predicted target
      3. Queue gate: skip if |q_robot – last_cmd| > 5° (robot still chasing)
      4. Fixed 50 ms send interval
      5. Step-limiter: advance ≤ MAX_STEP_RAD toward predicted
      6. Adaptive CP based on per-command step size (degrees):
           step > 3°  → CP = 25
           step 1–3°  → CP = 12
           step < 1°  → CP = 3

    Physics note (MG400 Mock, SpeedFactor=100, SpeedL=50):
      r_blend = (CP/100) × speed_l² / (2×acc_l) = (CP/100) × 375 mm
      With CP=3 → r_blend ≈ 11 mm;  step=1° ≈ 5 mm → fires immediately.
      On the real robot (SpeedL=10) r_blend scales to 1/25 of the mock
      values, making CP=25 physically meaningful (r_blend ≈ 15 mm).
    """
    CMD_INTERVAL  = 0.040              # 25 Hz
    LPF_ALPHA     = 0.20               # stronger smoothing weight
    FF_LEAD_SEC   = 0.080              # 80 ms feedforward to lead the robot
    MAX_FF_LEAD   = np.radians(10.0)   # cap feedforward
    MAX_STEP_RAD  = np.radians(1.5)    # max 1.5° per command
    GATE_RAD      = np.radians(4.0)    # 4° queue-depth gate

    CP_TABLE = [                       # (min_step_rad, cp_value)
        (np.radians(2.0), 50),         # step > 2°
        (np.radians(0.5), 25),         # step 0.5–2.0°
        (np.radians(0.0), 10),         # step < 0.5° (keep CP decent to avoid full stops)
    ]

    def __init__(self):
        self._lpf: Optional[np.ndarray]      = None
        self._last_cmd: Optional[np.ndarray] = None
        self._last_send_t: float             = 0.0
        # Feedforward state
        self._prev_tgt: Optional[np.ndarray] = None
        self._prev_tgt_t: float              = 0.0
        self._ema_vel: Optional[np.ndarray]  = None
        self.last_cp: int = 3               # exposed for ExperimentalStrategy

    def reset(self):
        self._lpf = None
        self._last_cmd = None
        self._last_send_t = 0.0
        self._prev_tgt = None
        self._prev_tgt_t = 0.0
        self._ema_vel = None
        self.last_cp = 3

    # ── internal helpers ──────────────────────────────────────────

    def _apply_lpf(self, target: np.ndarray) -> np.ndarray:
        if self._lpf is None:
            self._lpf = target[:4].copy()
        else:
            self._lpf = (self.LPF_ALPHA * target[:4]
                         + (1.0 - self.LPF_ALPHA) * self._lpf)
        return self._lpf.copy()

    def _apply_ff(self, smoothed: np.ndarray, now: float) -> np.ndarray:
        dt = now - self._prev_tgt_t
        if self._prev_tgt is None or dt < 0.001:
            self._prev_tgt  = smoothed.copy()
            self._prev_tgt_t = now
            if self._ema_vel is None:
                self._ema_vel = np.zeros(4)
            return smoothed.copy()

        raw_vel = (smoothed - self._prev_tgt) / dt
        if self._ema_vel is None:
            self._ema_vel = np.zeros(4)
        self._ema_vel = 0.3 * raw_vel + 0.7 * self._ema_vel
        self._prev_tgt  = smoothed.copy()
        self._prev_tgt_t = now

        lead = self._ema_vel * self.FF_LEAD_SEC
        norm = float(np.max(np.abs(lead)))
        if norm > self.MAX_FF_LEAD:
            lead = lead * (self.MAX_FF_LEAD / norm)
        return smoothed + lead

    def _adaptive_cp(self, step_rad: float) -> int:
        for thresh, cp in self.CP_TABLE:
            if step_rad >= thresh:
                return cp
        return self.CP_TABLE[-1][1]

    # ── main process ──────────────────────────────────────────────

    def process(self, q_target: np.ndarray, q_current: np.ndarray,
                now: float) -> Tuple[bool, Optional[np.ndarray], str]:

        # 1. LPF
        smoothed = self._apply_lpf(q_target)

        # 2. Feedforward prediction
        predicted = self._apply_ff(smoothed, now)

        # Bootstrap on first call
        if self._last_cmd is None:
            self._last_cmd    = q_current[:4].copy()
            self._last_send_t = now - self.CMD_INTERVAL  # fire immediately

        # 3. Queue gate: robot is still far behind last command → wait
        #    (bounds queue depth to ≤ GATE_RAD / MAX_STEP ≈ 2 commands)
        dist_to_last = float(np.max(np.abs(q_current[:4] - self._last_cmd)))
        if dist_to_last > self.GATE_RAD:
            return False, None, "Gate"

        # 4. Fixed interval
        if (now - self._last_send_t) < (self.CMD_INTERVAL - 0.005):
            return False, None, "Wait"

        # 5. Step-limiting: advance from LAST COMMAND (smooth) toward predicted target
        #    Using q_current as the base introduces feedback noise/jitter.
        delta      = predicted - self._last_cmd
        delta_norm = float(np.max(np.abs(delta)))
        if delta_norm > self.MAX_STEP_RAD:
            delta = delta * (self.MAX_STEP_RAD / delta_norm)
        cmd        = self._last_cmd + delta

        # Spatial noise floor
        actual_step = float(np.max(np.abs(delta)))
        if actual_step < _SPATIAL_THRESHOLD:
            return False, None, "SpatialSkip"

        # 6. Adaptive CP
        self.last_cp   = self._adaptive_cp(actual_step)
        self._last_cmd = cmd.copy()
        self._last_send_t = now

        return True, cmd, (f"M16/CP={self.last_cp}"
                           f"/step={np.degrees(actual_step):.2f}deg")


# ═══════════════════════════════════════════════════════════════
# M17 CleanFF — Fixed 25 Hz + LPF + FF=40ms + CP=100 (no dropout spikes)
# ═══════════════════════════════════════════════════════════════

_MODE_ENABLE  = 5
_MODE_RUNNING = 7


class _CleanFFLogic:
    """
    M17_CleanFF: Default + LPF + Feedforward.  Same CP=100 as Default.

    Key insight: CP=100 with SpeedL=10 gives r_blend ≈ 11 mm > step ≈ 6 mm,
    so commands chain IMMEDIATELY.  This is what keeps Default's queue at
    depth 1-2.  Changing CP lower breaks that and grows the queue to 3-5x,
    collapsing effective update rate from 25 Hz to 8-10 Hz.

    Improvements over Default (Mode 0):
      - LPF(α=0.20) on raw VR target → VR tracker noise suppression
      - Feedforward (100 ms EMA velocity) → compensates AccJ=100 execution lag
        (Default has zero feedforward — the phase lag is entirely uncompensated)
      - target_drift check (vs last_cmd, not vs q_current) → correct spatial guard
      - Fixed 25 Hz rate floor only (no dynamic proximity trigger)
    """
    CMD_INTERVAL = 0.040             # 25 Hz
    LPF_ALPHA    = 0.20
    FF_LEAD_SEC  = 0.040             # 40 ms = one command interval (exact 1-step lag compensation)
    MAX_FF_LEAD  = np.radians(5.0)   # 5° cap (prevents peak overshoot)
    SPATIAL_TH   = np.radians(0.10)  # 0.1° noise floor (vs target drift)
    GATE_RAD     = np.radians(8.0)   # 8° safety queue gate (drops commands if queue builds up)
    last_cp      = 100               # KEEP CP=100: r_blend > step → immediate chain

    def __init__(self):
        self._lpf: Optional[np.ndarray]      = None
        self._prev_tgt: Optional[np.ndarray] = None
        self._prev_tgt_t: float              = 0.0
        self._ema_vel: Optional[np.ndarray]  = None
        self._last_cmd: Optional[np.ndarray] = None
        self._last_send_t: float             = 0.0

    def reset(self):
        self._lpf         = None
        self._prev_tgt    = None
        self._prev_tgt_t  = 0.0
        self._ema_vel     = None
        self._last_cmd    = None
        self._last_send_t = 0.0

    def process(self, q_target: np.ndarray, q_current: np.ndarray,
                now: float) -> Tuple[bool, Optional[np.ndarray], str]:

        # 1. LPF on raw target
        # Large sudden jump (e.g. Home button → 0,0,0,0): reset filter instantly
        # so only ONE command is sent, not 2-3 overshooting ones from EMA velocity
        _JUMP_RESET_RAD = np.radians(15.0)
        if self._lpf is not None:
            jump = float(np.max(np.abs(q_target[:4] - self._lpf)))
            if jump > _JUMP_RESET_RAD:
                self._lpf    = q_target[:4].copy()
                self._ema_vel = np.zeros(4)
                self._prev_tgt = q_target[:4].copy()
                self._prev_tgt_t = now
        if self._lpf is None:
            self._lpf = q_target[:4].copy()
        else:
            self._lpf = (self.LPF_ALPHA * q_target[:4]
                         + (1.0 - self.LPF_ALPHA) * self._lpf)
        smoothed = self._lpf.copy()

        # 2. Feedforward velocity EMA
        dt = now - self._prev_tgt_t
        if self._prev_tgt is None or dt < 0.001:
            self._prev_tgt   = smoothed.copy()
            self._prev_tgt_t = now
            if self._ema_vel is None:
                self._ema_vel = np.zeros(4)
        else:
            raw_vel       = (smoothed - self._prev_tgt) / dt
            self._ema_vel = 0.3 * raw_vel + 0.7 * self._ema_vel
            self._prev_tgt   = smoothed.copy()
            self._prev_tgt_t = now
        ff   = self._ema_vel * self.FF_LEAD_SEC
        norm = float(np.max(np.abs(ff)))
        if norm > self.MAX_FF_LEAD:
            ff = ff * (self.MAX_FF_LEAD / norm)
        predicted = smoothed + ff

        # Bootstrap
        if self._last_cmd is None:
            self._last_cmd    = q_current[:4].copy()
            self._last_send_t = now - self.CMD_INTERVAL

        # 3. SAFETY GATE: prevent queue buildup. If the robot is too far behind
        #    the last command we sent, it means the queue is backing up or the
        #    robot is stuck.
        dist_to_last = float(np.max(np.abs(q_current[:4] - self._last_cmd)))
        if dist_to_last > self.GATE_RAD:
            # IMPORTANT: Update last_cmd to current position so we don't deadlock!
            # If we don't do this, and the robot is genuinely stuck or makes a huge
            # sudden jump, dist_to_last will stay > 8° forever and we'll never send again.
            self._last_cmd = q_current[:4].copy()
            return False, None, "QueueGate_Reset"

        # 4. Fixed 25 Hz rate floor
        if (now - self._last_send_t) < (self.CMD_INTERVAL - 0.005):
            return False, None, "Wait"

        # 5. Spatial noise floor: how much has the predicted target moved
        #    since the last command?  (NOT how far robot is from target —
        #    that would be ~0 when tracking is good, causing false skips)
        target_drift = float(np.max(np.abs(predicted - self._last_cmd)))
        if target_drift < self.SPATIAL_TH:
            return False, None, "SpatialSkip"

        self._last_cmd    = predicted.copy()
        self._last_send_t = now
        return True, predicted, "CleanFF"


# ═══════════════════════════════════════════════════════════════
# Public API — ExperimentalStrategy (used by vr_teleop_node.py)
# ═══════════════════════════════════════════════════════════════

class ExperimentalStrategy:
    """
    Wraps M8/M11/M14/M15 logic into a single interface compatible
    with the main control loop in vr_teleop_node.py.

    ⚠️ BYPASS MODE: This class formats commands DIRECTLY without
    going through TeleopController or MotionPlanner.  The only
    shared utility is JointValidator (called upstream in the node).

    Usage:
        strategy = ExperimentalStrategy("m11", control_mode, logger)
        should_send, cmd_str, q_rad, info = strategy.process(
            q_target_rad, q_current_rad, now
        )
    """

    MODE_MAP = {
        "m8_raw": ("M8_RawData",   _RawLogic),
        "m11":    ("M11_Stable",   _M11Logic),
        "m14":    ("M14_Smooth",   _M14Logic),
        "m15":    ("M15_Sharp",    _M15Logic),
        "m16":    ("M16_AdaptCP",  _M16Logic),
        "m17":    ("M17_CleanFF",   _CleanFFLogic),
    }

    def __init__(self, mode_key: str, control_mode: str, logger):
        if mode_key not in self.MODE_MAP:
            raise ValueError(f"Unknown experimental mode: {mode_key!r}. "
                             f"Use one of: {list(self.MODE_MAP.keys())}")
        self.mode_name, logic_cls = self.MODE_MAP[mode_key]
        self._logic = logic_cls()
        self._control_mode = control_mode  # "jointmovj" / "movj" / "movl"
        self._logger = logger
        self._cmd_times: deque = deque(maxlen=60)

    def reset(self):
        self._logic.reset()

    @staticmethod
    def _format_cmd(q_rad: np.ndarray, control_mode: str,
                    speed: int = 100, acc: int = 100, cp: int = 100) -> str:
        """Build TCP command string directly — no MotionPlanner needed."""
        q_deg = np.degrees(q_rad[:4])
        args = (f"{q_deg[0]:.4f},{q_deg[1]:.4f},"
                f"{q_deg[2]:.4f},{q_deg[3]:.4f},"
                f"SpeedJ={speed},AccJ={acc},CP={cp}")
        if control_mode == "movj":
            return f"MovJ({args})"
        elif control_mode == "movl":
            return f"MovL({args})"
        return f"JointMovJ({args})"  # default: jointmovj

    def process(self, q_target_rad: np.ndarray, q_current_rad: np.ndarray,
                now: float, robot_mode: int = _MODE_ENABLE
                ) -> Tuple[bool, Optional[str], Optional[np.ndarray], str]:
        """
        Run one cycle of the experimental logic.

        Args:
            q_target_rad: Latest RAW validated target in radians (4,)
                          (NOT Kalman-predicted — bypasses TargetPredictor)
            q_current_rad: Current robot joints in radians (4,)
            now: Current time (seconds)
            robot_mode: Current robot mode integer (5=ENABLE, 7=RUNNING).
                        Required by M17_ReactiveFF; ignored by other modes.

        Returns:
            (should_send, cmd_string, q_safe_rad, reason)
            cmd_string is a ready-to-send TCP command string
            q_safe_rad is the actual joint target (may differ due to clamping)
        """
        if hasattr(self._logic, 'set_robot_mode'):
            self._logic.set_robot_mode(robot_mode)
        should_send, cmd_rad, reason = self._logic.process(q_target_rad, q_current_rad, now)

        if not should_send or cmd_rad is None:
            return False, None, None, reason

        # Format command DIRECTLY (bypass MotionPlanner entirely)
        # M16 exposes last_cp for adaptive per-command CP values
        cp = getattr(self._logic, 'last_cp', 100)
        cmd_str = self._format_cmd(cmd_rad, self._control_mode, cp=cp)

        # Hz tracking
        self._cmd_times.append(now)
        hz = 0
        if len(self._cmd_times) > 2:
            span = self._cmd_times[-1] - self._cmd_times[0]
            hz = int((len(self._cmd_times) - 1) / max(span, 0.001))

        info = f"[{self.mode_name}/{reason}] Hz={hz}"
        return True, cmd_str, cmd_rad, info
