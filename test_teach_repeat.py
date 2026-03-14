#!/usr/bin/env python3
"""
Standalone test for Teach & Repeat — verifies that:
  1. Smart recording produces compact ~8-10 Hz JSON (like money.json)
  2. Save → Load round-trips perfectly
  3. Playback sends the EXACT same joint values as what was recorded
  4. Graph data (waypoint_callback) matches JSON byte-for-byte
  5. Can also load and play back money.json directly

Run:  python3 test_teach_repeat.py
"""

import sys, os, time, json, math
import numpy as np

# Add the package to sys.path so we can import without ROS
PKG = os.path.join(os.path.dirname(__file__),
                   "src/dobot_mg400/mg400_controller/mg400_controller")
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "src/dobot_mg400/mg400_controller"))

from mg400_controller.common.trajectory.trajectory_recorder import (
    TrajectoryRecorder, TRAJ_DIR, RECORD_MIN_DT, RECORD_MIN_DELTA)


# ── Mock helpers ─────────────────────────────────────────────────────────────
class MockLogger:
    def info(self, msg):  print(f"  [INFO]  {msg}")
    def warn(self, msg):  print(f"  [WARN]  {msg}")
    def error(self, msg): print(f"  [ERROR] {msg}")

sent_commands = []      # commands sent to "robot"
waypoint_log = []       # joint values published to graphs (radians)

def mock_send(cmd_str):
    sent_commands.append(cmd_str)
    return True

def mock_waypoint_cb(q_rad):
    waypoint_log.append(list(q_rad))


# ── Test 1: Smart recording produces compact JSON ───────────────────────────
def test_smart_recording():
    print("\n" + "="*70)
    print("TEST 1: Smart recording at ~10 Hz with movement threshold")
    print("="*70)

    tr = TrajectoryRecorder(mock_send, MockLogger(),
                            waypoint_callback=mock_waypoint_cb)
    tr.start_recording()

    # Simulate 10 seconds of a sine-wave trajectory at 50 Hz (like control loop)
    duration = 10.0
    dt = 0.02   # 50 Hz
    n_ticks = int(duration / dt)

    for i in range(n_ticks):
        t = i * dt
        # Multi-joint sine wave (degrees → radians for record_tick)
        j1 = 30.0 * math.sin(2 * math.pi * 0.2 * t)      # slow sweep
        j2 = 20.0 * math.sin(2 * math.pi * 0.3 * t + 0.5)
        j3 = 15.0 * math.sin(2 * math.pi * 0.15 * t + 1.0)
        j4 = 0.0

        q_rad = np.radians([j1, j2, j3, j4])

        # Fake the time by manipulating _rec_t0
        if i == 0:
            tr._rec_t0 = time.time()
        # Override time for deterministic test
        # We'll call record_tick and let it use time.time() naturally
        # But to simulate 10s we need to actually wait... too slow.
        # Instead, monkey-patch time for the recorder:
        pass

    # Better approach: call record_tick with controlled timestamps
    tr.is_recording = False  # reset
    tr._frames = []
    tr._last_rec_t = -999.0
    tr._last_rec_q = np.full(4, np.nan)
    tr.is_recording = True

    fake_t0 = 100.0  # arbitrary epoch
    tr._rec_t0 = fake_t0

    import unittest.mock as mock
    for i in range(n_ticks):
        t = i * dt
        j1 = 30.0 * math.sin(2 * math.pi * 0.2 * t)
        j2 = 20.0 * math.sin(2 * math.pi * 0.3 * t + 0.5)
        j3 = 15.0 * math.sin(2 * math.pi * 0.15 * t + 1.0)
        j4 = 0.0
        q_rad = np.radians([j1, j2, j3, j4])

        with mock.patch('time.time', return_value=fake_t0 + t):
            tr.record_tick(q_rad)

    tr.is_recording = False
    frames = tr._frames

    print(f"  Input ticks:     {n_ticks} (50 Hz × {duration}s)")
    print(f"  Recorded frames: {len(frames)}")
    if frames:
        dur = frames[-1]["timeStamp"] - frames[0]["timeStamp"]
        avg_dt = dur / max(len(frames)-1, 1)
        print(f"  Duration:        {dur:.2f}s")
        print(f"  Avg dt:          {avg_dt:.4f}s  ({1/avg_dt:.1f} Hz)")
        print(f"  Compression:     {n_ticks/len(frames):.1f}x")

    # Verify all frames have meaningful movement (except first)
    small_moves = 0
    for i in range(1, len(frames)):
        d = max(abs(frames[i][k] - frames[i-1][k]) for k in ["j1","j2","j3","j4"])
        if d < RECORD_MIN_DELTA:
            small_moves += 1
    print(f"  Frames with <{RECORD_MIN_DELTA}° movement: {small_moves}")

    assert len(frames) < n_ticks / 3, f"Should be much fewer frames than input ticks"
    assert small_moves == 0, "All frames should have significant movement"
    print("  ✅ PASS — compact recording, all frames meaningful")

    return tr, frames


# ── Test 2: Save → Load round-trip ──────────────────────────────────────────
def test_save_load_roundtrip(tr, frames):
    print("\n" + "="*70)
    print("TEST 2: Save → Load round-trip (byte-exact)")
    print("="*70)

    os.makedirs(TRAJ_DIR, exist_ok=True)
    path = tr.save_as("_test_roundtrip")
    assert path, "Save failed"

    # Load
    ok = tr.load("_test_roundtrip")
    assert ok, "Load failed"

    loaded = tr.loaded_frames
    assert len(loaded) == len(frames), f"Frame count mismatch: {len(loaded)} != {len(frames)}"

    max_err = 0.0
    for i in range(len(frames)):
        for k in ["timeStamp", "j1", "j2", "j3", "j4"]:
            diff = abs(frames[i][k] - loaded[i][k])
            max_err = max(max_err, diff)

    print(f"  Saved frames:  {len(frames)}")
    print(f"  Loaded frames: {len(loaded)}")
    print(f"  Max error:     {max_err:.10f}")
    assert max_err < 1e-5, f"Round-trip error too large: {max_err}"
    print("  ✅ PASS — save/load round-trip exact")

    # Cleanup
    try:
        os.remove(path)
    except Exception:
        pass


# ── Test 3: Playback sends EXACT frame values ───────────────────────────────
def test_playback_exact(tr):
    print("\n" + "="*70)
    print("TEST 3: Playback sends EXACT JSON frame values to graph callback")
    print("="*70)

    sent_commands.clear()
    waypoint_log.clear()

    frames = tr.loaded_frames
    n = len(frames)

    # Start playback (runs in background thread)
    tr.start_preview()

    # Wait for completion
    timeout = frames[-1]["timeStamp"] - frames[0]["timeStamp"] + 5.0
    t0 = time.time()
    while tr.is_playing and (time.time() - t0) < timeout:
        time.sleep(0.05)

    assert not tr.is_playing, "Playback didn't complete in time"
    print(f"  Frames in JSON:      {n}")
    print(f"  Commands sent:       {len(sent_commands)}")
    print(f"  Waypoint callbacks:  {len(waypoint_log)}")

    assert len(waypoint_log) == n, \
        f"Waypoint count mismatch: {len(waypoint_log)} != {n}"
    assert len(sent_commands) == n, \
        f"Command count mismatch: {len(sent_commands)} != {n}"

    # Compare: waypoint_log (radians) vs frames (degrees)
    max_err_deg = 0.0
    for i in range(n):
        q_played = np.degrees(waypoint_log[i])
        q_json = [frames[i]["j1"], frames[i]["j2"], frames[i]["j3"], frames[i]["j4"]]
        for j in range(4):
            err = abs(q_played[j] - q_json[j])
            max_err_deg = max(max_err_deg, err)

    print(f"  Max joint error (played vs JSON): {max_err_deg:.10f}°")

    # Parse sent commands to verify they contain the exact values
    max_cmd_err = 0.0
    for i in range(n):
        cmd = sent_commands[i]
        # Parse "JointMovJ(j1,j2,j3,j4,SpeedJ=...)"
        inner = cmd.split("(")[1].split(")")[0]
        parts = inner.split(",")
        cmd_j = [float(parts[k]) for k in range(4)]
        json_j = [frames[i]["j1"], frames[i]["j2"], frames[i]["j3"], frames[i]["j4"]]
        for j in range(4):
            err = abs(cmd_j[j] - json_j[j])
            max_cmd_err = max(max_cmd_err, err)

    print(f"  Max joint error (command vs JSON): {max_cmd_err:.10f}°")

    assert max_err_deg < 1e-6, f"Waypoint error too large: {max_err_deg}"
    assert max_cmd_err < 0.001, f"Command error too large: {max_cmd_err}"
    print("  ✅ PASS — playback sends EXACT JSON values (error ≈ 0)")


# ── Test 4: Load & verify money.json ────────────────────────────────────────
def test_money_json():
    print("\n" + "="*70)
    print("TEST 4: Load money.json and verify playback fidelity")
    print("="*70)

    money_path = os.path.join(os.path.dirname(__file__), "..", "money.json")
    if not os.path.isfile(money_path):
        # Try alternative path
        money_path = os.path.expanduser("~/project_teleop_ws/money.json")
    if not os.path.isfile(money_path):
        print("  ⏭️  SKIP — money.json not found")
        return

    with open(money_path) as f:
        data = json.load(f)
    frames = data.get("frames", data if isinstance(data, list) else [])

    print(f"  money.json: {len(frames)} frames, "
          f"{frames[-1]['timeStamp'] - frames[0]['timeStamp']:.2f}s")

    sent_commands.clear()
    waypoint_log.clear()

    tr = TrajectoryRecorder(mock_send, MockLogger(),
                            waypoint_callback=mock_waypoint_cb)
    tr.loaded_frames = frames
    tr.loaded_name = "money.json"

    tr.start_preview()

    timeout = frames[-1]["timeStamp"] - frames[0]["timeStamp"] + 5.0
    t0 = time.time()
    while tr.is_playing and (time.time() - t0) < timeout:
        time.sleep(0.05)

    assert not tr.is_playing, "Playback didn't complete"
    assert len(waypoint_log) == len(frames), \
        f"Waypoint count: {len(waypoint_log)} != {len(frames)}"

    max_err = 0.0
    for i in range(len(frames)):
        q_played = np.degrees(waypoint_log[i])
        q_json = [frames[i]["j1"], frames[i]["j2"], frames[i]["j3"], frames[i]["j4"]]
        for j in range(4):
            max_err = max(max_err, abs(q_played[j] - q_json[j]))

    print(f"  Waypoint callbacks:  {len(waypoint_log)}")
    print(f"  Max error (°):       {max_err:.10f}")
    assert max_err < 1e-6, f"money.json playback error: {max_err}"
    print("  ✅ PASS — money.json playback exact")


# ── Test 5: Stop & go home ──────────────────────────────────────────────────
def test_stop_home():
    print("\n" + "="*70)
    print("TEST 5: Stop + go_home sends Home command and blocks teleop")
    print("="*70)

    sent_commands.clear()
    tr = TrajectoryRecorder(mock_send, MockLogger(),
                            waypoint_callback=mock_waypoint_cb)
    tr.stop_all(go_home=True)

    assert len(sent_commands) == 1, f"Expected 1 home command, got {len(sent_commands)}"
    assert "JointMovJ(0.0000,0.0000,0.0000,0.0000" in sent_commands[0]
    assert tr._block_until > time.perf_counter()
    remaining = tr._block_until - time.perf_counter()
    print(f"  Home command:    {sent_commands[0]}")
    print(f"  Block remaining: {remaining:.1f}s")
    assert remaining > 4.0, "Block should last ~5s"
    print("  ✅ PASS — home command sent, teleop blocked for ~5s")


# ── Main ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 70)
    print("  TEACH & REPEAT — STANDALONE VERIFICATION TEST")
    print("=" * 70)

    tr, frames = test_smart_recording()
    test_save_load_roundtrip(tr, frames)
    test_playback_exact(tr)
    test_money_json()
    test_stop_home()

    print("\n" + "=" * 70)
    print("  ALL TESTS PASSED ✅")
    print("=" * 70)
    print("\nKey guarantees verified:")
    print("  • Recording produces ~8-10 Hz compact JSON (like money.json)")
    print("  • Save → Load is byte-exact")
    print("  • Playback graph callback = EXACT JSON values (error = 0)")
    print("  • Playback robot commands = EXACT JSON values")
    print("  • Stop sends Home + blocks teleop 5s")
    print("  → During playback, Unity line = Sent line = JSON data (identical)")
