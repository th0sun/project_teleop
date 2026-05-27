"""Unity tool / dashboard command handlers.

Encapsulates everything triggered by Unity-side UI controls that map
straight onto the robot's digital outputs and dashboard channel:

  - Suction / vacuum trigger (with "smart" mode that waits for the
    robot to reach its target pose before firing).
  - Generic digital-output light control.
  - Pass-through dashboard commands sent from a GUI / Unity slider.
  - Global SpeedFactor adjustments.

The owning node holds a single ``ToolCommandHandlers`` instance and
binds each ROS subscription's ``msg_callback`` to the corresponding
method here. The class owns its own suction state machine, so the
node no longer carries ``self.suction_*`` fields.
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

import numpy as np

from mg400_controller.common.config import motion_config
from mg400_protocol.dashboard import speed_factor


class ToolCommandHandlers:
    """Stateful holder for the suction state machine + the small set
    of "Unity button" ROS callbacks.

    Dependencies are injected so the class is testable without a live
    ROS node:
      - ``connection``      — must expose ``connected`` (bool) and
                              ``send_dashboard_cmd(str) -> None``.
      - ``sender``          — must expose
                              ``set_digital_output(port, bool) -> None``.
      - ``logger``          — ROS logger (info/warn/error).
      - ``get_latest_target_fn`` — returns the most recent joint
                              target (np.ndarray) or None; used by
                              smart-suction to remember where we wanted
                              the robot to be when the operator pressed
                              the trigger.
    """

    def __init__(
        self,
        *,
        connection,
        sender,
        logger,
        get_latest_target_fn: Callable[[], Optional[np.ndarray]],
    ):
        self._connection = connection
        self._sender = sender
        self._log = logger
        self._get_latest_target = get_latest_target_fn

        # Public state: control-loop reads `pending` + `target_q` each tick.
        self.state: bool = False
        self.pending: bool = False
        self.requested_state: bool = False
        self.target_q: Optional[np.ndarray] = None

    # ── ROS subscription callbacks ─────────────────────────────────────

    def suction_callback(self, msg) -> None:
        """``/unity/suction`` (Bool) — VR trigger toggle for the gripper.

        If the operator turned suction ON and ``SMART_SUCTION_ENABLED``
        is set, the actual DO write is deferred: the target joint
        configuration at trigger time is remembered, and the control
        loop fires the suction once the robot has settled near that
        target. Turning suction OFF is always immediate.
        """
        requested_state = msg.data

        if requested_state == self.state:
            return
        self.requested_state = requested_state

        latest_target = self._get_latest_target()
        if (
            requested_state
            and motion_config.SMART_SUCTION_ENABLED
            and latest_target is not None
        ):
            self.target_q = latest_target.copy()
            self.pending = True
            self._log.info(
                "🔘 Smart Suction queued: ON (Waiting for robot to reach target)"
            )
        else:
            # Immediate mode: release sequence or smart suction disabled.
            self.handle_suction_cmd(requested_state)

    def light_callback(self, msg) -> None:
        """``/unity/light`` (Int64MultiArray) — set DO[port]=state.

        Payload: ``msg.data == [port, state]``.
        """
        if len(msg.data) < 2:
            return
        port = msg.data[0]
        state = bool(msg.data[1])
        if not self._connection.connected:
            self._log.warn(f"⚠️ Cannot set light port {port}; Robot disconnected.")
            return
        self._sender.set_digital_output(port, state)

    def dashboard_cmd_callback(self, msg) -> None:
        """``/unity/dashboard_cmd`` (String) — raw Dashboard command
        forwarded as-is to the Dobot dashboard socket. Appends the
        protocol-required newline if missing.
        """
        cmd = msg.data.strip()
        if not cmd:
            return
        if not self._connection.connected:
            self._log.warn(f"⚠️ Cannot send dashboard cmd '{cmd}'; Robot disconnected.")
            return
        self._log.info(f"📨 Dashboard Command received from GUI: {cmd}")
        if not cmd.endswith("\n"):
            cmd += "\n"
        self._connection.send_dashboard_cmd(cmd)

    def speed_factor_callback(self, msg) -> None:
        """``/unity/speed_factor`` (Int32-like) — global SpeedFactor
        (1..100) from a GUI slider.
        """
        try:
            value = int(msg.data)
        except (TypeError, ValueError):
            self._log.warn(f"⚠️ Invalid SpeedFactor payload: {msg.data!r}")
            return
        value = max(1, min(100, value))
        if not self._connection.connected:
            self._log.warn(f"⚠️ Cannot set SpeedFactor({value}); Robot disconnected.")
            return
        cmd = speed_factor(value).render()
        self._log.info(f"🏃 SpeedFactor update from GUI: {value}%")
        self._connection.send_dashboard_cmd(cmd + "\n")

    # ── Smart-suction hook called from the control loop ────────────────

    def maybe_fire_smart_suction(self, q_current, stuck_start_time: float) -> None:
        """Called every 50 Hz tick. Fires the deferred ON command once
        the robot has either come within ``SUCTION_ACTIVATION_THRESHOLD``
        of the target joint configuration, or the controller has flagged
        the motion as stuck near it (so the suction doesn't hang
        forever if the robot can't quite hit the target).
        """
        if not (self.pending and self.target_q is not None):
            return
        dist = np.max(np.abs(q_current - self.target_q))
        # ระยะใกล้ Threshold = ถึงเป้า / หรือ Stuck = ค้าง ก็ยิงเลยกัน hang
        if dist < motion_config.SUCTION_ACTIVATION_THRESHOLD or stuck_start_time > 0:
            self.handle_suction_cmd(self.requested_state)
            self.pending = False

    # ── Suction state machine ──────────────────────────────────────────

    def handle_suction_cmd(self, state: bool) -> None:
        """Execute the suction toggle.

        ON:    VACUUM DO = True, BLOW DO = False  (suction holds part)
        OFF:   VACUUM DO = False, BLOW DO = True  (release blow pulse),
               then BLOW DO = False after ``motion_config.BLOW_DURATION``
               seconds so the blow port is idle when no one is asking.
        """
        if not self._connection.connected:
            self._log.warn("⚠️ Cannot toggle suction; Robot disconnected.")
            return

        if state:
            # 🟢 Suck
            self._sender.set_digital_output(motion_config.VACUUM_DO_PORT, True)
            self._sender.set_digital_output(motion_config.BLOW_DO_PORT, False)
            self.state = True
            self._log.info("吸 [SUCK] Vacuum ON, Blow OFF")
            return

        # 🔴 Release sequence: Vacuum OFF -> Blow ON -> Auto-Off via timer.
        self._sender.set_digital_output(motion_config.VACUUM_DO_PORT, False)
        self._sender.set_digital_output(motion_config.BLOW_DO_PORT, True)
        self._log.info(
            f"💨 [RELEASE] Vacuum OFF, Blow ON (for {motion_config.BLOW_DURATION}s)"
        )

        def turn_off_blow():
            try:
                self._sender.set_digital_output(motion_config.BLOW_DO_PORT, False)
                self._log.info("🛑 [IDLE] Blow OFF, All suction ports closed")
                self.state = False
            except Exception as exc:
                self._log.error(f"Error in turn_off_blow timer: {exc}")

        threading.Timer(motion_config.BLOW_DURATION, turn_off_blow).start()
