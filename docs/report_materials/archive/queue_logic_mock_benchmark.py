#!/usr/bin/env python3

import argparse
import math
import socket
import statistics
import struct
import time
from dataclasses import dataclass

import numpy as np


PACKET_SIZE = 1440
HOST = "172.10.0.2"
DASHBOARD_PORT = 29999
MOTION_PORT = 30003
FEEDBACK_PORT = 30004
QUEUE_BACKLOG_GATE_RAD = 0.01
QUEUE_BUSY_ESCAPE_SEC = 0.30
STUCK_VELOCITY_THRESHOLD = 0.005


@dataclass
class FeedbackState:
    q_target: np.ndarray
    q_actual: np.ndarray
    robot_mode: int
    run_queued_cmd: int


class MockRobotClient:
    def __init__(self, host: str):
        self.host = host
        self.dashboard = socket.create_connection((host, DASHBOARD_PORT), timeout=2.0)
        self.motion = socket.create_connection((host, MOTION_PORT), timeout=2.0)
        self.feedback = socket.create_connection((host, FEEDBACK_PORT), timeout=2.0)
        self.feedback.settimeout(2.0)
        self._feedback_buffer = b""

    def close(self):
        for sock in (self.dashboard, self.motion, self.feedback):
            try:
                sock.close()
            except Exception:
                pass

    def dashboard_cmd(self, cmd: str) -> str:
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.dashboard.sendall(cmd.encode())
        data = self.dashboard.recv(4096)
        return data.decode(errors="ignore").strip()

    def motion_cmd(self, cmd: str):
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.motion.sendall(cmd.encode())

    def latest_feedback(self) -> FeedbackState:
        while True:
            chunk = self.feedback.recv(4096)
            if not chunk:
                raise RuntimeError("feedback socket closed")
            self._feedback_buffer += chunk
            if len(self._feedback_buffer) >= PACKET_SIZE:
                break

        latest = None
        while len(self._feedback_buffer) >= PACKET_SIZE:
            latest = self._feedback_buffer[:PACKET_SIZE]
            self._feedback_buffer = self._feedback_buffer[PACKET_SIZE:]

        if latest is None:
            raise RuntimeError("no feedback packet")

        q_target_deg = struct.unpack_from("<6d", latest, 192)[:4]
        q_actual_deg = struct.unpack_from("<6d", latest, 432)[:4]
        robot_mode = struct.unpack_from("<Q", latest, 24)[0]
        run_queued_cmd = latest[1014]
        return FeedbackState(
            q_target=np.radians(q_target_deg),
            q_actual=np.radians(q_actual_deg),
            robot_mode=robot_mode,
            run_queued_cmd=run_queued_cmd,
        )


def target_trajectory(t: float) -> np.ndarray:
    if t < 1.0:
        deg = np.array([0.0, 0.0, 0.0, 0.0])
    elif t < 3.0:
        x = t - 1.0
        deg = np.array([
            20.0 * math.sin(2.0 * math.pi * 0.6 * x),
            12.0 * math.sin(2.0 * math.pi * 0.6 * x + 0.8),
            18.0 * math.sin(2.0 * math.pi * 0.6 * x + 1.2),
            10.0 * math.sin(2.0 * math.pi * 0.8 * x),
        ])
    elif t < 4.0:
        deg = np.array([25.0, 10.0, -20.0, 10.0])
    elif t < 6.0:
        x = t - 4.0
        square = 1.0 if math.sin(2.0 * math.pi * 1.2 * x) >= 0 else -1.0
        deg = np.array([18.0 * square, 8.0 * square, -15.0 * square, 6.0 * square])
    else:
        x = t - 6.0
        deg = np.array([
            6.0 * math.sin(2.0 * math.pi * 1.5 * x),
            3.0 * math.sin(2.0 * math.pi * 1.2 * x + 0.4),
            -4.0 * math.sin(2.0 * math.pi * 1.7 * x + 0.7),
            2.5 * math.sin(2.0 * math.pi * 1.3 * x),
        ])
    return np.radians(deg)


def format_jointmovj(q_rad: np.ndarray) -> str:
    q_deg = np.degrees(q_rad[:4])
    return (
        f"JointMovJ({q_deg[0]:.4f},{q_deg[1]:.4f},{q_deg[2]:.4f},{q_deg[3]:.4f},"
        "SpeedJ=100,AccJ=100,CP=100)"
    )


class StrategyBase:
    def __init__(self):
        self.last_sent_target = None
        self.last_sent_time = 0.0
        self.last_robot_q = np.zeros(4)
        self.last_robot_t = 0.0
        self.robot_velocity = np.zeros(4)
        self.prev_target = None
        self.prev_target_t = None
        self.ema_target_vel = np.zeros(4)
        self.queue_busy_start_time = 0.0

    def update_robot_velocity(self, q_current: np.ndarray, now: float) -> float:
        dt = now - self.last_robot_t
        if dt > 1e-3:
            delta_q = np.abs(q_current - self.last_robot_q)
            raw = delta_q / dt
            self.robot_velocity = 0.3 * raw + 0.7 * self.robot_velocity
            self.last_robot_q = q_current.copy()
            self.last_robot_t = now
        return float(np.max(self.robot_velocity))

    def compute_latency_comp_target(self, q_target: np.ndarray, now: float) -> np.ndarray:
        if self.prev_target is None:
            self.prev_target = q_target.copy()
            self.prev_target_t = now
            self.ema_target_vel = np.zeros(4)
            return q_target.copy()
        dt = now - self.prev_target_t
        if 0.002 < dt < 0.5:
            raw_vel = (q_target - self.prev_target) / dt
            self.ema_target_vel = 0.3 * raw_vel + 0.7 * self.ema_target_vel
        self.prev_target = q_target.copy()
        self.prev_target_t = now
        return q_target + self.ema_target_vel * 0.08

    def compute_predictive_target(self, q_target: np.ndarray, now: float) -> np.ndarray:
        if self.prev_target is None:
            self.prev_target = q_target.copy()
            self.prev_target_t = now
            self.ema_target_vel = np.zeros(4)
            return q_target.copy()
        dt = now - self.prev_target_t
        if 0.002 < dt < 0.5:
            raw_vel = (q_target - self.prev_target) / dt
            self.ema_target_vel = 0.3 * raw_vel + 0.7 * self.ema_target_vel
        self.prev_target = q_target.copy()
        self.prev_target_t = now
        return q_target + self.ema_target_vel * 0.08

    def mark_sent(self, q_target: np.ndarray, now: float):
        self.last_sent_target = q_target.copy()
        self.last_sent_time = now


class OldCurrentStrategy(StrategyBase):
    def step(self, latest_target: np.ndarray, fb: FeedbackState, now: float):
        if self.last_sent_target is None:
            return [latest_target.copy()], "Init"

        vel = self.update_robot_velocity(fb.q_actual, now)
        dist_to_last = float(np.max(np.abs(fb.q_actual - self.last_sent_target)))
        change = float(np.max(np.abs(latest_target - self.last_sent_target)))
        trigger = 0.02 + vel * 0.25
        if dist_to_last < trigger and change > 0.0005:
            return [latest_target.copy()], f"DynProx(trigger={trigger:.4f})"
        return [], "Wait"


class NewQueueAwareStrategy(StrategyBase):
    def step(self, latest_target: np.ndarray, fb: FeedbackState, now: float):
        if self.last_sent_target is None:
            return [latest_target.copy()], "Init"

        vel = self.update_robot_velocity(fb.q_actual, now)
        backlog = float(np.max(np.abs(fb.q_target - fb.q_actual)))
        queue_busy = bool(fb.run_queued_cmd and backlog > QUEUE_BACKLOG_GATE_RAD)
        if queue_busy:
            if self.queue_busy_start_time == 0.0:
                self.queue_busy_start_time = now
            queue_busy_duration = now - self.queue_busy_start_time
            can_check_stuck = (
                vel < STUCK_VELOCITY_THRESHOLD and
                queue_busy_duration > QUEUE_BUSY_ESCAPE_SEC
            )
            if not can_check_stuck:
                return [], f"QueueBusy(backlog={backlog:.4f})"
        else:
            self.queue_busy_start_time = 0.0

        dist_to_last = float(np.max(np.abs(fb.q_actual - self.last_sent_target)))
        change = float(np.max(np.abs(latest_target - self.last_sent_target)))
        trigger = 0.02 + vel * 0.25
        if dist_to_last < trigger and change > 0.0005:
            return [latest_target.copy()], f"DynProx(trigger={trigger:.4f})"
        return [], "Wait"


class NewQueueAwareProxyStrategy(StrategyBase):
    def step(self, latest_target: np.ndarray, fb: FeedbackState, now: float):
        if self.last_sent_target is None:
            return [latest_target.copy()], "Init"

        vel = self.update_robot_velocity(fb.q_actual, now)
        backlog = float(np.max(np.abs(fb.q_target - fb.q_actual)))
        queue_busy = backlog > QUEUE_BACKLOG_GATE_RAD
        if queue_busy:
            if self.queue_busy_start_time == 0.0:
                self.queue_busy_start_time = now
            queue_busy_duration = now - self.queue_busy_start_time
            can_check_stuck = (
                vel < STUCK_VELOCITY_THRESHOLD and
                queue_busy_duration > QUEUE_BUSY_ESCAPE_SEC
            )
            if not can_check_stuck:
                return [], f"QueueBusyProxy(backlog={backlog:.4f})"
        else:
            self.queue_busy_start_time = 0.0

        dist_to_last = float(np.max(np.abs(fb.q_actual - self.last_sent_target)))
        change = float(np.max(np.abs(latest_target - self.last_sent_target)))
        trigger = 0.02 + vel * 0.25
        if dist_to_last < trigger and change > 0.0005:
            return [latest_target.copy()], f"DynProx(trigger={trigger:.4f})"
        return [], "Wait"


class Old240224Strategy(StrategyBase):
    def step(self, latest_target: np.ndarray, fb: FeedbackState, now: float):
        if self.last_sent_target is None:
            return [latest_target.copy()], "Init"

        vel = self.update_robot_velocity(fb.q_actual, now)
        dist_to_last = float(np.max(np.abs(fb.q_actual - self.last_sent_target)))
        change = float(np.max(np.abs(latest_target - self.last_sent_target)))
        trigger = 0.005 + vel * 0.25
        if dist_to_last < trigger and change > 0.0005:
            if vel < 0.1:
                steps = []
                for alpha in (1.0 / 3.0, 2.0 / 3.0, 1.0):
                    steps.append(fb.q_actual + alpha * (latest_target - fb.q_actual))
                return steps, f"BatchDynProx(trigger={trigger:.4f})"
            return [latest_target.copy()], f"DynProx(trigger={trigger:.4f})"
        return [], "Wait"


def run_strategy(name: str, strategy, host: str, duration: float = 8.0, control_dt: float = 0.02):
    client = MockRobotClient(host)
    try:
        client.dashboard_cmd("ClearError()")
        client.dashboard_cmd("EnableRobot()")
        start = time.perf_counter()
        next_tick = start
        samples = []
        commands_sent = 0

        # Prime feedback stream.
        fb = client.latest_feedback()

        while True:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed > duration:
                break
            if now < next_tick:
                time.sleep(min(0.001, next_tick - now))
                continue
            next_tick += control_dt

            fb = client.latest_feedback()
            hand_target = target_trajectory(elapsed)
            if isinstance(strategy, Old240224Strategy):
                latest_target = strategy.compute_predictive_target(hand_target, now)
            else:
                latest_target = strategy.compute_latency_comp_target(hand_target, now)

            outgoing, reason = strategy.step(latest_target, fb, now)
            for q_target in outgoing:
                client.motion_cmd(format_jointmovj(q_target))
                strategy.mark_sent(q_target, now)
                commands_sent += 1

            latest_error = float(np.max(np.abs(fb.q_actual - hand_target)))
            backlog = float(np.max(np.abs(fb.q_target - fb.q_actual)))
            samples.append({
                "latest_error": latest_error,
                "backlog": backlog,
                "run_queued": int(fb.run_queued_cmd),
                "reason": reason,
            })

        # Let the queue drain a bit for a fair final snapshot.
        drain_until = time.perf_counter() + 1.0
        while time.perf_counter() < drain_until:
            fb = client.latest_feedback()
            time.sleep(0.01)

        return summarize(name, samples, commands_sent)
    finally:
        client.close()


def summarize(name, samples, commands_sent):
    latest_errors = [s["latest_error"] for s in samples]
    backlogs = [s["backlog"] for s in samples]
    queue_busy_ratio = sum(1 for s in samples if s["run_queued"]) / max(len(samples), 1)
    return {
        "name": name,
        "samples": len(samples),
        "commands_sent": commands_sent,
        "mean_latest_error_deg": math.degrees(statistics.mean(latest_errors)),
        "p95_latest_error_deg": math.degrees(np.percentile(latest_errors, 95)),
        "max_latest_error_deg": math.degrees(max(latest_errors)),
        "mean_backlog_deg": math.degrees(statistics.mean(backlogs)),
        "max_backlog_deg": math.degrees(max(backlogs)),
        "queue_running_ratio": queue_busy_ratio,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--strategy", choices=["old24", "oldcurrent", "new", "newproxy"], required=True)
    parser.add_argument("--host", default=HOST)
    args = parser.parse_args()

    strategies = {
        "old24": Old240224Strategy,
        "oldcurrent": OldCurrentStrategy,
        "new": NewQueueAwareStrategy,
        "newproxy": NewQueueAwareProxyStrategy,
    }
    result = run_strategy(args.strategy, strategies[args.strategy](), host=args.host)
    for key, value in result.items():
        print(f"{key}={value}")


if __name__ == "__main__":
    main()
