#!/usr/bin/env python3
"""
CP Benchmark for MG400 Mock Server.

Tests Continuous-Path (CP) behaviour from CP=0 to CP=100 over a rectangular
waypoint path and generates multi-panel plots showing trajectory in XY,
position/speed/joint-angle time-series, and XYZ 3-D view.

Usage:
    # 1. Start the mock server
    cd /path/to/MG400_Mock/app/src && python main.py

    # 2. Run this script (from any directory)
    python cp_benchmark.py
"""

import socket
import time
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ---------------------------------------------------------------------------
# Server configuration
# ---------------------------------------------------------------------------
HOST = "127.0.0.1"
DASHBOARD_PORT = 29999
MOTION_PORT = 30003
FEEDBACK_PORT = 30004

# ---------------------------------------------------------------------------
# Realtime feedback packet dtype  (must match app/src/tcp_interface/realtime_packet.py)
# ---------------------------------------------------------------------------
_PACKET_DTYPE = np.dtype([
    ("len",                  np.uint16),
    ("not_used_01",          np.uint16,  (3,)),
    ("digital_inputs",       np.uint64),
    ("digital_outputs",      np.uint64),
    ("robot_mode",           np.uint64),
    ("not_used_02",          np.uint64),
    ("not_used_03",          np.uint64),
    ("test_value",           np.uint64),
    ("not_used_04",          np.float64),
    ("speed_scaling",        np.float64),
    ("not_used_05",          np.float64),
    ("not_used_06",          np.float64),
    ("not_used_07",          np.float64),
    ("not_used_08",          np.float64),
    ("not_used_09",          np.float64),
    ("not_used_10",          np.float64),
    ("not_used_11",          np.float64, (3,)),
    ("not_used_12",          np.float64, (3,)),
    ("not_used_13",          np.float64, (3,)),
    ("q_target",             np.float64, (6,)),
    ("qd_target",            np.float64, (6,)),
    ("qdd_target",           np.float64, (6,)),
    ("i_target",             np.float64, (6,)),
    ("m_target",             np.float64, (6,)),
    ("q_actual",             np.float64, (6,)),
    ("qd_actual",            np.float64, (6,)),
    ("i_actual",             np.float64, (6,)),
    ("actual_i_TCP_force",   np.float64, (6,)),
    ("tool_vector_actual",   np.float64, (6,)),
    ("TCP_speed_actual",     np.float64, (6,)),
    ("TCP_force",            np.float64, (6,)),
    ("tool_vector_target",   np.float64, (6,)),
    ("TCP_speed_target",     np.float64, (6,)),
    ("not_used_14",          np.float64, (6,)),
    ("not_used_15",          np.float64, (6,)),
    ("not_used_16",          np.float64, (6,)),
    ("not_used_17",          np.uint64,  (14,)),
    ("not_used_18",          np.float64, (6,)),
    ("load",                 np.float64),
    ("center_x",             np.float64),
    ("center_y",             np.float64),
    ("center_z",             np.float64),
    ("not_used_19",          np.float64, (6,)),
    ("not_used_20",          np.float64, (6,)),
    ("not_used_21",          np.float64),
    ("not_used_22",          np.float64, (6,)),
    ("not_used_23",          np.float64, (4,)),
    ("not_used_24",          np.float64, (4,)),
    ("not_used_25",          np.uint8,   (24,)),
])
PACKET_SIZE: int = _PACKET_DTYPE.itemsize

# Robot mode IDs (from robot_mode.py)
MODE_ENABLE  = 5
MODE_RUNNING = 7

# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------
# Home position (all joints at 0 → FK gives (284, 0, 118, 0))
HOME = (284.0, 0.0, 118.0, 0.0)

# Rectangular path with sharp corners — ideal for showing CP rounding.
# Keep waypoints well inside the workspace (distance from base < 400 mm).
WAYPOINTS: List[Tuple[float, float, float, float]] = [
    (284.0,   80.0, 118.0, 0.0),   # P1 – move +Y
    (350.0,   80.0, 118.0, 0.0),   # P2 – move +X  (corner)
    (350.0,  -80.0, 118.0, 0.0),   # P3 – move -Y  (corner)
    (284.0,  -80.0, 118.0, 0.0),   # P4 – move -X  (corner)
    (284.0,    0.0, 118.0, 0.0),   # P5 – return home (corner)
]

# CP values to benchmark (0 = full stop at each waypoint, 100 = maximum blend)
CP_VALUES: List[int] = [0, 20, 40, 60, 80, 100]

# Speed percentage.  Lower speed → smaller blend radius → cleaner corner
# rounding relative to segment length.
# At SPEED=20: speed_l≈150 mm/s → r_blend(CP=100)≈15 mm.
# At SPEED=40: speed_l≈300 mm/s → r_blend(CP=100)≈60 mm (too large for 66 mm segs).
SPEED = 20

# ---------------------------------------------------------------------------
# Feedback recorder (background thread)
# ---------------------------------------------------------------------------

class FeedbackRecorder:
    """Continuously reads realtime packets from port 30004."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._running = False
        self._start_time: float = time.time()

        self._times:     List[float]      = []
        self._positions: List[np.ndarray] = []   # [X, Y, Z, R]
        self._joints:    List[np.ndarray] = []   # [J1, J2, J3, J4]
        self._modes:     List[int]        = []
        self._speeds:    List[np.ndarray] = []   # [vx, vy, vz]

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._host, self._port))
        self._sock.settimeout(2.0)

    def clear(self) -> None:
        with self._lock:
            self._times.clear()
            self._positions.clear()
            self._joints.clear()
            self._modes.clear()
            self._speeds.clear()
        self._start_time = time.time()

    def start(self) -> None:
        self.clear()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _loop(self) -> None:
        buf = b""
        while self._running:
            try:
                chunk = self._sock.recv(8192)  # type: ignore[union-attr]
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= PACKET_SIZE:
                    raw, buf = buf[:PACKET_SIZE], buf[PACKET_SIZE:]
                    try:
                        pkt = np.frombuffer(raw, dtype=_PACKET_DTYPE)[0]
                        t = time.time() - self._start_time
                        with self._lock:
                            self._times.append(t)
                            self._positions.append(np.array(pkt["tool_vector_actual"][:4], dtype=float))
                            self._joints.append(np.array(pkt["q_actual"][:4], dtype=float))
                            self._modes.append(int(pkt["robot_mode"]))
                            self._speeds.append(np.array(pkt["TCP_speed_actual"][:3], dtype=float))
                    except Exception:
                        pass
            except socket.timeout:
                continue
            except Exception:
                break

    def get_data(self) -> Tuple[List, List, List, List, List]:
        with self._lock:
            return (
                list(self._times),
                [p.copy() for p in self._positions],
                [j.copy() for j in self._joints],
                list(self._modes),
                [s.copy() for s in self._speeds],
            )

    def close(self) -> None:
        if self._sock:
            self._sock.close()


# ---------------------------------------------------------------------------
# Socket helpers
# ---------------------------------------------------------------------------

def send_dashboard(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.06)
    try:
        return sock.recv(1024).decode().strip()
    except Exception:
        return ""


def send_motion(sock: socket.socket, cmd: str) -> None:
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.005)


# ---------------------------------------------------------------------------
# Wait helpers
# ---------------------------------------------------------------------------

def wait_for_running(rec: FeedbackRecorder, timeout: float = 5.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        _, _, _, modes, _ = rec.get_data()
        if modes and modes[-1] == MODE_RUNNING:
            return True
        time.sleep(0.01)
    return False


def wait_for_complete(
    rec: FeedbackRecorder,
    final_xyz: Tuple[float, float, float],
    timeout: float = 60.0,
    tolerance: float = 3.0,
) -> bool:
    """
    Wait until the robot is in ENABLE mode AND its TCP position is within
    *tolerance* mm of *final_xyz*.  This correctly handles CP=0, which
    creates multiple short RUNNING→ENABLE cycles (one per waypoint segment),
    by only returning True when the robot is also near the final target.
    """
    t0 = time.time()
    saw_running = False
    while time.time() - t0 < timeout:
        _, positions, _, modes, _ = rec.get_data()
        if modes and positions:
            if not saw_running and MODE_RUNNING in modes:
                saw_running = True
            if saw_running and modes[-1] == MODE_ENABLE:
                dist = float(np.linalg.norm(
                    np.array(positions[-1][:3]) - np.array(final_xyz)
                ))
                if dist < tolerance:
                    return True
        time.sleep(0.015)
    return False


# ---------------------------------------------------------------------------
# Single CP run
# ---------------------------------------------------------------------------

def run_cp_test(
    cp: int,
    dash: socket.socket,
    motion: socket.socket,
    rec: FeedbackRecorder,
) -> dict:
    """
    Execute the full waypoint path with a given CP ratio.
    Returns a dict with times, positions, joints, modes, speeds, cmd_times.
    """
    # Configure
    send_dashboard(dash, "ResetRobot()")
    time.sleep(0.2)
    send_dashboard(dash, f"CP({cp})")
    send_dashboard(dash, f"SpeedL({SPEED})")
    send_dashboard(dash, f"SpeedJ({SPEED})")
    time.sleep(0.1)

    # Start fresh recording
    rec.clear()
    t0 = rec._start_time

    cmd_times: List[float] = []

    # Send all waypoints in quick succession → they queue up in __motion_que
    for x, y, z, r in WAYPOINTS:
        ct = time.time() - t0
        send_motion(motion, f"MovL({x},{y},{z},{r})")
        cmd_times.append(ct)

    # Wait for motion to start then finish
    wait_for_running(rec, timeout=5.0)
    final_xyz = (WAYPOINTS[-1][0], WAYPOINTS[-1][1], WAYPOINTS[-1][2])
    ok = wait_for_complete(rec, final_xyz, timeout=60.0)
    if not ok:
        print(f"  ⚠ CP={cp} timed out")

    time.sleep(0.1)
    times, positions, joints, modes, speeds = rec.get_data()
    return {
        "times":     times,
        "positions": positions,
        "joints":    joints,
        "modes":     modes,
        "speeds":    speeds,
        "cmd_times": cmd_times,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(results: Dict[int, dict]) -> None:
    # Blue (CP=0, sharp corners) → Red (CP=100, maximum rounding)
    cmap_cp = matplotlib.colormaps["coolwarm"]
    norm_cp = mcolors.Normalize(vmin=0, vmax=100)
    colors = {cp: cmap_cp(norm_cp(cp)) for cp in CP_VALUES}

    corner_wps = list(WAYPOINTS[:-1])   # P1–P4 (the four actual corners)
    wp_labels   = ["Home", "P1", "P2", "P3", "P4", "Home"]
    all_pts     = [HOME] + list(WAYPOINTS)
    wp_x = [p[0] for p in all_pts]
    wp_y = [p[1] for p in all_pts]

    # ── Figure: 2 rows — big XY on top, 3 time-series on bottom ─────────────
    fig = plt.figure(figsize=(16, 11), layout="constrained")
    fig.suptitle(
        f"MG400 Mock — CP Corner-Rounding Benchmark   "
        f"(SpeedL = SpeedJ = {SPEED}%)",
        fontsize=13, fontweight="bold",
    )

    gs     = fig.add_gridspec(2, 3, height_ratios=[1.5, 1])
    ax_xy  = fig.add_subplot(gs[0, :])          # top row: full width
    ax_x   = fig.add_subplot(gs[1, 0])          # bottom-left
    ax_y   = fig.add_subplot(gs[1, 1])          # bottom-mid
    ax_spd = fig.add_subplot(gs[1, 2])          # bottom-right

    # ── XY Trajectory ────────────────────────────────────────────────────────
    ax_xy.set_title(
        "XY Trajectory  —  higher CP = smoother corner rounding",
        fontsize=11,
    )
    for cp in CP_VALUES:
        d = results.get(cp)
        if not d or not d["positions"]:
            continue
        pos = np.array(d["positions"])
        dur = d["times"][-1] if d["times"] else 0.0
        lw  = 2.8 if cp == 0 else 1.8
        ls  = "--" if cp == 0 else "-"
        ax_xy.plot(
            pos[:, 0], pos[:, 1],
            color=colors[cp], linewidth=lw, linestyle=ls, alpha=0.92,
            label=f"CP={cp:3d}   ({dur:.2f} s)",
            zorder=4,
        )

    # Ideal right-angle reference path
    ax_xy.plot(wp_x, wp_y, color="black", linewidth=1.0,
               linestyle=":", alpha=0.35, label="Ideal path (right-angle)", zorder=2)

    # Corner waypoints
    cx = [p[0] for p in corner_wps]
    cy = [p[1] for p in corner_wps]
    ax_xy.scatter(cx, cy, c="black", s=90, marker="D", zorder=10, label="Corners (P1–P4)")

    # Home marker
    ax_xy.scatter([HOME[0]], [HOME[1]], c="forestgreen", s=130,
                  marker="*", zorder=11)

    # Labels for each waypoint
    label_offsets = {"Home": (7, 5), "P1": (7, 5), "P2": (7, 5),
                     "P3": (7, -13), "P4": (-30, -13)}
    for lbl, x, y in zip(wp_labels[:-1], wp_x[:-1], wp_y[:-1]):
        dx, dy = label_offsets.get(lbl, (7, 5))
        ax_xy.annotate(lbl, (x, y), textcoords="offset points",
                       xytext=(dx, dy), fontsize=9, fontweight="bold", zorder=12)

    # Tight limits around actual waypoint extents + 12 mm margin on each side
    x_min = min(p[0] for p in all_pts) - 12
    x_max = max(p[0] for p in all_pts) + 12
    y_min = min(p[1] for p in all_pts) - 12
    y_max = max(p[1] for p in all_pts) + 12
    ax_xy.set_xlabel("X (mm)", fontsize=10)
    ax_xy.set_ylabel("Y (mm)", fontsize=10)
    ax_xy.set_xlim(x_min, x_max)
    ax_xy.set_ylim(y_min, y_max)
    ax_xy.grid(True, alpha=0.2)
    ax_xy.legend(
        loc="upper right", fontsize=9,
        title="CP   (total time)", title_fontsize=9,
        framealpha=0.92,
    )

    # Colorbar on the right of the XY axis
    sm = cm.ScalarMappable(cmap=cmap_cp, norm=norm_cp)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax_xy, fraction=0.018, pad=0.01)
    cbar.set_label("CP ratio", fontsize=9)
    cbar.set_ticks(CP_VALUES)

    # ── Time-series helper ────────────────────────────────────────────────────
    t_max = max(
        (d["times"][-1] for d in results.values() if d and d["times"]),
        default=10.0,
    ) * 1.05   # 5% margin

    def _plot_ts(ax: plt.Axes, key: str, ylabel: str, title: str,  # type: ignore[name-defined]
                 show_legend: bool = False) -> None:
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_xlim(0, t_max)
        ax.grid(True, alpha=0.2)

        for cp in CP_VALUES:
            d = results.get(cp)
            if not d or not d["positions"]:
                continue
            t    = np.array(d["times"])
            pos  = np.array(d["positions"])
            spds = np.array(d["speeds"])

            if key == "x":
                vals = pos[:, 0]
            elif key == "y":
                vals = pos[:, 1]
            else:   # speed magnitude
                vals = (np.linalg.norm(spds, axis=1)
                        if spds.ndim == 2 else np.zeros(len(t)))

            lw = 2.2 if cp == 0 else 1.5
            ls = "--" if cp == 0 else "-"
            ax.plot(t, vals, color=colors[cp], linewidth=lw,
                    linestyle=ls, label=f"CP={cp}", alpha=0.9)

            # Vertical dotted lines at command-send instants
            for ct in d["cmd_times"]:
                ax.axvline(ct, color=colors[cp], linewidth=0.7,
                           linestyle=":", alpha=0.45)

        if show_legend:
            ax.legend(fontsize=7, ncol=2, loc="best")

    _plot_ts(ax_x,   "x",   "X (mm)",     "X position vs Time", show_legend=True)
    _plot_ts(ax_y,   "y",   "Y (mm)",     "Y position vs Time")
    _plot_ts(ax_spd, "spd", "|v| (mm/s)",
             "TCP Speed vs Time\n(CP=0 shows full stops at each corner)")

    out_file = "cp_benchmark_result.png"
    plt.savefig(out_file, dpi=150, bbox_inches="tight")
    print(f"Plot saved → {out_file}")
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 60)
    print("  MG400 Mock — CP Benchmark")
    print(f"  Server : {HOST}  (ports {DASHBOARD_PORT}/{MOTION_PORT}/{FEEDBACK_PORT})")
    print(f"  CP values : {CP_VALUES}")
    print(f"  Speed  : SpeedL=SpeedJ={SPEED}%")
    print(f"  Packet size: {PACKET_SIZE} bytes")
    print("=" * 60)

    # ── Connect ──────────────────────────────────────────────────────────────
    print("Connecting…")
    dash = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    dash.connect((HOST, DASHBOARD_PORT))
    dash.settimeout(5.0)

    motion = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    motion.connect((HOST, MOTION_PORT))
    motion.settimeout(1.0)

    resp = send_dashboard(dash, "EnableRobot()")
    print(f"  EnableRobot → {resp or '(no response)'}")
    time.sleep(0.3)

    # ── Feedback recorder ────────────────────────────────────────────────────
    rec = FeedbackRecorder(HOST, FEEDBACK_PORT)
    rec.connect()
    rec.start()
    time.sleep(0.15)

    # ── Run tests ────────────────────────────────────────────────────────────
    results: Dict[int, dict] = {}
    for cp in CP_VALUES:
        print(f"\nCP={cp:3d} …", end=" ", flush=True)
        data = run_cp_test(cp, dash, motion, rec)
        results[cp] = data
        n = len(data["times"])
        dur = data["times"][-1] if n else 0.0
        print(f"✓  {n} samples, {dur:.2f}s total")

        # Return to HOME with CP=0 before next run (safety reset)
        send_dashboard(dash, "CP(0)")
        x, y, z, r = HOME
        send_motion(motion, f"MovL({x},{y},{z},{r})")
        time.sleep(0.1)
        wait_for_complete(rec, (HOME[0], HOME[1], HOME[2]), timeout=20.0)
        time.sleep(0.25)

    # ── Cleanup ──────────────────────────────────────────────────────────────
    rec.stop()
    rec.close()
    dash.close()
    motion.close()
    print("\nAll runs complete. Generating plots…")

    # ── Plot ─────────────────────────────────────────────────────────────────
    plot_results(results)


if __name__ == "__main__":
    main()
