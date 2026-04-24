#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Monitor-domain state for digital-output control widgets.
"""

from dataclasses import dataclass
import time

from mg400_controller.common.config.motion_config import (
    GREEN_LIGHT_DO_PORT,
    RED_LIGHT_DO_PORT,
    VACUUM_DO_PORT,
    YELLOW_LIGHT_DO_PORT,
)


COLOR_OFF = "#d0d0d0"
COLOR_GREEN = "#2ecc71"
COLOR_YELLOW = "#f1c40f"
COLOR_RED = "#e74c3c"
COLOR_VACUUM = "#3498db"
COLOR_PENDING = "orange"

LIGHT_SPECS = (
    ("GREEN", GREEN_LIGHT_DO_PORT, COLOR_GREEN),
    ("YELLOW", YELLOW_LIGHT_DO_PORT, COLOR_YELLOW),
    ("RED", RED_LIGHT_DO_PORT, COLOR_RED),
)


@dataclass(frozen=True)
class ButtonDisplay:
    text: str
    bg: str
    fg: str


@dataclass(frozen=True)
class ControlCommand:
    port: int
    state: bool
    display: ButtonDisplay


@dataclass(frozen=True)
class ControlPanelSync:
    suction: object
    lights: dict


def _build_suction_display(is_active, pending=False):
    if pending:
        if is_active:
            return ButtonDisplay(text="VACUUM (WAIT)", bg=COLOR_PENDING, fg="white")
        return ButtonDisplay(text="OFF (WAIT)", bg=COLOR_PENDING, fg="black")

    if is_active:
        return ButtonDisplay(text="VACUUM", bg=COLOR_VACUUM, fg="white")
    return ButtonDisplay(text="OFF", bg=COLOR_OFF, fg="black")


def _build_light_display(name, color, is_active, pending=False):
    if pending:
        return ButtonDisplay(text=f"{name}...", bg=COLOR_PENDING, fg="black")
    if is_active:
        return ButtonDisplay(text=name, bg=color, fg="white")
    return ButtonDisplay(text=name, bg=COLOR_OFF, fg="black")


class MonitorControlPanelState:
    def __init__(self, lockout_sec=2.0, time_fn=None):
        self.lockout_sec = lockout_sec
        self.time_fn = time.time if time_fn is None else time_fn
        self.lockout = {}
        self.suction_state = False
        self.light_states = {name: False for name, _, _ in LIGHT_SPECS}
        self.light_ports = {name: port for name, port, _ in LIGHT_SPECS}
        self.light_colors = {name: color for name, _, color in LIGHT_SPECS}

    def build_initial_sync(self):
        return ControlPanelSync(
            suction=_build_suction_display(self.suction_state),
            lights={
                name: _build_light_display(name, color, self.light_states[name])
                for name, _, color in LIGHT_SPECS
            },
        )

    def toggle_suction(self):
        self.suction_state = not self.suction_state
        self.lockout[VACUUM_DO_PORT] = self.time_fn() + self.lockout_sec
        return ControlCommand(
            port=VACUUM_DO_PORT,
            state=self.suction_state,
            display=_build_suction_display(self.suction_state, pending=True),
        )

    def toggle_light(self, name):
        state = not self.light_states[name]
        self.light_states[name] = state
        port = self.light_ports[name]
        self.lockout[port] = self.time_fn() + self.lockout_sec
        return ControlCommand(
            port=port,
            state=state,
            display=_build_light_display(name, self.light_colors[name], state, pending=True),
        )

    def sync_from_do_status(self, do_status):
        now = self.time_fn()
        suction_update = None
        light_updates = {}

        if now > self.lockout.get(VACUUM_DO_PORT, 0):
            actual_suction = bool((do_status >> (VACUUM_DO_PORT - 1)) & 1)
            self.suction_state = actual_suction
            suction_update = _build_suction_display(actual_suction)

        for name, port, color in LIGHT_SPECS:
            if now > self.lockout.get(port, 0):
                actual_light = bool((do_status >> (port - 1)) & 1)
                self.light_states[name] = actual_light
                light_updates[name] = _build_light_display(name, color, actual_light)

        return ControlPanelSync(suction=suction_update, lights=light_updates)
