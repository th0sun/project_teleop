#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🧠 Teleop Controller — Adaptive Per-Command Tuning

Decision logic for live teleop: when do we send a JointMovJ to the MG400, and
what SpeedJ/AccJ/CP should that command carry?  The previous design used a
fixed deadband (TARGET_CHANGE_THRESHOLD = 0.0005 rad ≈ 0.03°) and a single
DynProx gate, which produced 11–42 micro-stops per typical demo because the
per-command joint delta was much smaller than the controller's accel-cruise-
decel ramp window.  See EXP10 — at Δ < 2° the queue fill state (flood vs
paced vs proximity) makes no difference; the decel happens *inside* each
command, not between commands.

This rewrite implements the adaptive scheme verified in EXP3/EXP5/EXP7/EXP8:

  - Gate threshold and per-command SpeedJ both interpolate between
    REALTIME_DELTA_MIN_RAD/SPEEDJ_MIN (slow hand → fine tracking, low speed
    so any micro-stop is below perception) and REALTIME_DELTA_MAX_RAD/
    SPEEDJ_MAX (fast hand → smooth-regime delta, full speed for catch-up).
  - Hand velocity is sourced from the latency compensator's filtered Unity
    target velocity (target_velocity arg).  Robot-side velocity is still
    tracked for stuck detection only.
  - When the gate fires, the caller MUST send `latest_target` — not any
    historical/queued target.  This controller never replays stale state.

Stuck detection is preserved: if the robot stops moving while the latest
target is still distant, we re-trigger a send to escape geometric corner
cases (singularity-near-axis, momentum-into-wall, etc.).
"""

import numpy as np
from mg400_controller.common.config.robot_config import SPATIAL_THRESHOLD
from mg400_controller.common.config.motion_config import (
    PROXIMITY_THRESHOLD, STUCK_VELOCITY_THRESHOLD, STUCK_TIME_THRESHOLD,
    TARGET_CHANGE_THRESHOLD,
    REALTIME_DELTA_MIN_RAD, REALTIME_DELTA_MAX_RAD,
    REALTIME_HAND_VEL_LOW_RAD_S, REALTIME_HAND_VEL_HIGH_RAD_S,
    REALTIME_SPEEDJ_MIN, REALTIME_SPEEDJ_MAX,
    REALTIME_ACCJ_MIN, REALTIME_ACCJ_MAX,
    REALTIME_CP,
)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _lerp(a, b, t):
    return a + (b - a) * t


class TeleopController:
    def __init__(self, validator, planner, logger):
        self.validator = validator
        self.planner = planner
        self.logger = logger

        # Last accepted-by-MG400 command target (4 joints, radians).  Drives
        # both gate "delta-since-last-send" and adaptive SpeedJ scaling.
        self.last_sent_target = None
        self.last_sent_time = 0.0
        # Per-command tuning produced by the most recent should_send_command
        # decision.  format_command_string() consumes these so the cmd that
        # actually goes on the wire matches the gate's intent.
        self._pending_speed_j = REALTIME_SPEEDJ_MAX
        self._pending_acc_j = REALTIME_ACCJ_MAX
        self._pending_cp = REALTIME_CP
        self._pending_delta_rad = 0.0

        # Robot-side velocity tracking (for stuck detection only — no longer
        # gates the send).  Kept here so legacy callers that read
        # self.robot_velocity stay working.
        self.last_robot_q = np.zeros(4)
        self.last_robot_time = 0.0
        self.robot_velocity = np.zeros(4)

        self.stuck_start_time = 0.0
        self.is_stuck = False
        self.last_stuck_check_time = 0.0

    def reset_reference(self, q_current=None, now=0.0):
        """Reset live-teleop pacing after an external owner moved the robot.

        Teach-and-repeat playback owns the MG400 command stream while it is
        running.  When live teleop resumes, the controller must not compare the
        new hand target against the pre-playback command, otherwise it may chase
        stale queue state or wait for a target the robot no longer owns.
        """
        if q_current is None:
            self.last_sent_target = None
            self.last_robot_q = np.zeros(4)
        else:
            q_current = np.asarray(q_current[:4], dtype=float)
            self.last_sent_target = q_current.copy()
            self.last_robot_q = q_current.copy()
        self.last_sent_time = float(now)
        self.last_robot_time = float(now)
        self.robot_velocity = np.zeros(4)
        self.stuck_start_time = 0.0
        self.is_stuck = False
        self.last_stuck_check_time = 0.0
        self._pending_speed_j = REALTIME_SPEEDJ_MAX
        self._pending_acc_j = REALTIME_ACCJ_MAX
        self._pending_cp = REALTIME_CP
        self._pending_delta_rad = 0.0

    def update_robot_state(self, q_current, now):
        """Track filtered robot joint velocity for stuck detection."""
        dt = now - self.last_robot_time
        if dt > 0.001:
            delta_q = np.abs(q_current - self.last_robot_q)
            instantaneous = delta_q / dt
            alpha = 0.3
            self.robot_velocity = alpha * instantaneous + (1 - alpha) * self.robot_velocity
            self.last_robot_q = q_current.copy()
            self.last_robot_time = now
        return self.robot_velocity

    def check_stuck_condition(self, velocity_mag, dist_to_target, now):
        if velocity_mag < STUCK_VELOCITY_THRESHOLD and dist_to_target > PROXIMITY_THRESHOLD:
            if self.stuck_start_time == 0:
                self.stuck_start_time = now
            elif (now - self.stuck_start_time) > STUCK_TIME_THRESHOLD:
                return True
        else:
            self.stuck_start_time = 0
        return False

    def _adaptive_gate_threshold(self, hand_vel_mag_rad_s):
        """Map hand velocity to the gate threshold (rad).

        Linear interpolation between REALTIME_DELTA_MIN_RAD at HAND_VEL_LOW
        and REALTIME_DELTA_MAX_RAD at HAND_VEL_HIGH, clamped at both ends.
        """
        span = REALTIME_HAND_VEL_HIGH_RAD_S - REALTIME_HAND_VEL_LOW_RAD_S
        if span <= 0:
            return REALTIME_DELTA_MAX_RAD
        t = _clamp((hand_vel_mag_rad_s - REALTIME_HAND_VEL_LOW_RAD_S) / span, 0.0, 1.0)
        return _lerp(REALTIME_DELTA_MIN_RAD, REALTIME_DELTA_MAX_RAD, t)

    def _adaptive_speed_acc(self, delta_rad):
        """Map per-command joint delta to SpeedJ/AccJ percentage.

        Bigger commands get higher SpeedJ so the controller can enter cruise
        phase before decel.  Smaller commands stay slow so the absolute peak
        velocity remains low — micro-stops at low absolute speed are below
        operator perception even when the qd-trace shows them.
        """
        span = REALTIME_DELTA_MAX_RAD - REALTIME_DELTA_MIN_RAD
        if span <= 0:
            return REALTIME_SPEEDJ_MAX, REALTIME_ACCJ_MAX
        t = _clamp((delta_rad - REALTIME_DELTA_MIN_RAD) / span, 0.0, 1.0)
        speed_j = int(round(_lerp(REALTIME_SPEEDJ_MIN, REALTIME_SPEEDJ_MAX, t)))
        acc_j = int(round(_lerp(REALTIME_ACCJ_MIN, REALTIME_ACCJ_MAX, t)))
        return speed_j, acc_j

    def should_send_command(
        self,
        latest_target,
        q_current,
        now=None,
        queue_backlog_rad=None,
        run_queued_cmd=None,
        target_velocity=None,
    ):
        """Adaptive gate: should we send `latest_target` to the MG400 now?

        Args:
            latest_target: newest target joint position in radians (4-vector).
            q_current: current robot joint position in radians.
            now: monotonic timestamp (perf_counter).
            queue_backlog_rad / run_queued_cmd: accepted for wiring
                compatibility; the adaptive scheme does not consult queue
                state because EXP10 showed queue fill is irrelevant when the
                per-command delta is too small for the controller to reach
                cruise.
            target_velocity: filtered Unity hand velocity (rad/s, 4-vector).
                Used to size the gate threshold.  If None, the gate falls
                back to REALTIME_DELTA_MAX_RAD so callers without velocity
                wiring still get smooth motion.

        Returns:
            (bool, str) — should_send flag and a short reason for telemetry.
            On True, the corresponding (speed_j, acc_j, cp) tuning is stashed
            on the controller and consumed by format_command_string().
        """
        if now is None:
            now = self.last_robot_time

        # Initial command after reset: always send.  Use MAX tuning so the
        # robot starts moving toward the first received hand pose without lag.
        if self.last_sent_target is None:
            self._pending_speed_j = REALTIME_SPEEDJ_MAX
            self._pending_acc_j = REALTIME_ACCJ_MAX
            self._pending_cp = REALTIME_CP
            self._pending_delta_rad = REALTIME_DELTA_MAX_RAD
            return True, "Init"

        # Per-axis max delta to last-sent target.  Multi-axis hand motion can
        # trigger from any single axis exceeding the gate.
        delta_per_axis = np.abs(latest_target[:4] - self.last_sent_target[:4])
        delta = float(np.max(delta_per_axis))

        # Hand velocity magnitude — peak axis.  Falls back to robot-side
        # velocity if Unity hand velocity not wired.
        if target_velocity is not None:
            hand_vel = float(np.max(np.abs(np.asarray(target_velocity)[:4])))
        else:
            hand_vel = float(np.max(self.robot_velocity))

        threshold = self._adaptive_gate_threshold(hand_vel)

        if delta >= threshold:
            speed_j, acc_j = self._adaptive_speed_acc(delta)
            self._pending_speed_j = speed_j
            self._pending_acc_j = acc_j
            self._pending_cp = REALTIME_CP
            self._pending_delta_rad = delta
            return True, (
                f"Adaptive_d{np.degrees(delta):.2f}deg"
                f"_v{np.degrees(hand_vel):.0f}dps"
                f"_S{speed_j}"
            )

        # Stuck recovery — robot hasn't reached last-sent target and has
        # near-zero velocity for STUCK_TIME_THRESHOLD seconds.  Throttled to
        # 10 Hz so the recovery doesn't spam logs.
        velocity_mag = float(np.max(self.robot_velocity))
        if (now - self.last_stuck_check_time) > 0.1:
            self.last_stuck_check_time = now
            error_to_last = float(np.max(np.abs(q_current[:4] - self.last_sent_target[:4])))
            error_to_latest = float(np.max(np.abs(q_current[:4] - latest_target[:4])))
            stuck_error = max(error_to_last, error_to_latest)
            if self.check_stuck_condition(velocity_mag, stuck_error, now):
                change_in_target = float(np.max(np.abs(
                    latest_target[:4] - self.last_sent_target[:4]
                )))
                if change_in_target > TARGET_CHANGE_THRESHOLD or stuck_error > PROXIMITY_THRESHOLD:
                    self.logger.warn(
                        f"⚠️ Stuck Detected (Vel: {velocity_mag:.4f}) - Retriggering"
                    )
                    # Stuck send uses MAX tuning so the recovery move has
                    # cruise headroom to pull free.
                    self._pending_speed_j = REALTIME_SPEEDJ_MAX
                    self._pending_acc_j = REALTIME_ACCJ_MAX
                    self._pending_cp = REALTIME_CP
                    self._pending_delta_rad = max(delta, REALTIME_DELTA_MAX_RAD)
                    return True, (
                        f"Stuck_Vel{velocity_mag:.4f}_Delta{change_in_target:.3f}"
                    )

        return False, "Wait"

    def mark_command_sent(self, q_target, sent_time):
        """Update controller state only after the motion socket accepts the command."""
        self.last_sent_target = q_target.copy()
        self.last_sent_time = sent_time
        self.stuck_start_time = 0

    def format_command_string(self, q_target, q_current=None, force_send=False):
        """Validate, clamp, and render the JointMovJ string.

        The per-command (speed_j, acc_j, cp) tuning is whatever the most
        recent should_send_command() decided, so a small adaptive cmd carries
        a low SpeedJ and a large adaptive cmd carries SpeedJ=100.  Callers
        that bypass should_send_command() (force_send / stuck recovery) get
        the MAX tuning, which is intentional — they need cruise authority.
        """
        q_safe, is_clamped = self.validator.validate_and_clamp(q_target)
        if is_clamped:
            self.logger.warn("⚠️ Joint command exceeded limits - clamped to safe range")

        speed_j = self._pending_speed_j
        acc_j = self._pending_acc_j
        cp = self._pending_cp

        if force_send:
            speed_j = REALTIME_SPEEDJ_MAX
            acc_j = REALTIME_ACCJ_MAX
            cp = REALTIME_CP

        cmd_str = self.planner.format_command(
            q_safe, speed_percent=speed_j, acc_percent=acc_j, cp=cp,
        )

        return cmd_str, q_safe
