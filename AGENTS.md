# AGENTS.md — Project Teleop Handoff

> **Audience**: the next engineer (or AI agent) picking up this repository.
> Read this first. Everything else (docs/, code comments, ROS interfaces) is
> deeper detail — this file is the map.

## 1. Project goal

Unity VR teleop of a **Dobot MG400** 4-DOF SCARA robot, over **ROS2 Humble**.
Two operating modes:

1. **Realtime** — operator holds a Quest 3 VR controller, hand pose streams
   from Unity through ROS into the MG400 motion port. Robot follows the hand
   in real time.
2. **Teach-and-Repeat** — operator records a demonstration in VR, the system
   simplifies + classifies the trajectory, then plays it back on the real
   robot (or a mock).

The repo is the ROS-side host of this loop. Unity lives in a sibling repo;
the contract between them is a set of ROS topics + a `/teach/job_request`
schema (see §6).

Thesis context: **multi-robot teaching via VR with smooth realtime + accurate
replay**. The `robot_teaching_core/` package is the vendor-neutral abstraction
being prepared for a second robot family.

## 2. Current state (2026-05-08)

Active branch: **`realtime-smooth-adaptive`** (`origin` synced).

Last hardware-verified run: **2026-05-07** on real MG400 (Ubuntu rig,
192.168.1.6). Both modes work cleanly:

- Realtime: adaptive gate (REALTIME_DELTA_MIN_RAD = 0.5°, MAX_RAD = 3°)
  + per-cmd SpeedJ scaling 25→100% + CP=80. Jitter resolved by enforcing
  per-command joint delta ≥ smooth-cruise floor.
- Teach playback: PLAYBACK_DEFAULT_CP = 80, PLAYBACK_DEFAULT_SPEED_J = 80,
  PATH_SIMPLIFY_TOLERANCE_DEG = 3°. Cartesian primitives capped at
  `SEGMENT_CARTESIAN_CP_MAX = 30` to avoid alarm 34322 at corner blends.

Telemetry sink: `~/.ros/adaptive_telemetry/<session>.csv` (separate from
the existing triple-logger).

## 3. What's done — commit chronology

Diverged from `feat/multi-robot-teaching-architecture` at `0ecdd35`. 20
commits on top, oldest first:

| Commit | Why |
|---|---|
| `1a6b443` | feat: adaptive realtime + smooth teach playback — replace DynProx 3-gate scheme with single delta-based gate that lerps with hand velocity; teach CP 30→80 |
| `6f8d791` | feat: adaptive realtime telemetry sink — CSV writer for analysis, isolated from triple_logger |
| `b654032` | chore: drop unused DYNAMIC_PROXIMITY_* aliases (post-refactor cleanup of unreferenced symbols only) |
| `8d3cf0f` | chore: drop unused MotionPlanner entrypoints + SPEED_* bands (no external callers) |
| `026531f` | fix(mixed-primitive): first attempt at the alarm 34322 fix — lower SpeedL default. Misdiagnosis kept in history for context. |
| `3ad7e1b` | fix(mixed-primitive): real fix — cap CP for Cartesian primitives, restore SpeedL=60 |
| `d3706c6` | docs(mixed-primitive): correct the alarm 34322 interpretation in code comments (it is "position out of range", not "command too large") |
| `49533ca` | feat(logging): wire control_command_seq + tool_target through latency / triple logger so post-hoc analysis can correlate cmd → motion |
| `4c664ee` | docs: AGENTS.md handoff + repo-local progress-report skill + LEGACY markers |
| `ecda77e` | test(B1): tripwire for adaptive gate tuning constants |
| `aa857fd` | test(B2): tripwire for legacy teach actions in /teach/job_request |
| `08dc5bb` | test(B3): tripwire for the three LEGACY Unity teach topics |
| `39a237d` | test(B4): golden fixture for mixed-primitive Cartesian compile |
| `6d60746` | test(B5): tripwire for the dormant fastest_path_repeat profile |
| `089822f` | refactor(C1): extract trajectory_io module behind TrajectoryRecorder facade |
| `4d89d81` | refactor(C2): extract playback_compiler geometry/IK helpers behind recorder facade |
| `82e58c5` | refactor(C3): extract playback_executor leaf helpers behind recorder facade |
| `b5b48a1` | refactor(D1): group ROS subscription wiring into _register_ros_subscriptions helper |
| `6c7c34c` | feat(E1): expose USE_MIXED_PRIMITIVES as a ROS param (default unchanged) |
| `d0e070f` | docs: complete Phase A LEGACY markers (config + teach handler + legacy guide header) |

## 4. What's deferred — known smells (do NOT delete yet)

Each item is intentionally left enabled because we don't yet have proof
that no consumer (Unity build, mock, monitor GUI, an analyser, an
operator's launch file) depends on it. Each entry says **what would
prove it dead** so a future cleanup pass can justify removal.

### 4.1 Three legacy Unity teach callbacks (in `vr_teleop_node.py`)
- `_teach_status_callback` (subscribes `/unity/teach_status`)
- `_traj_data_callback` (subscribes `/unity/trajectory_data`)
- `_joint_trajectory_callback` (subscribes `/mg400/joint_trajectory_controller/command`)

Their docstrings already say "DEPRECATED in favour of `/teach/job_request`".
Unity's modern code path uses the job_request channel, but the old topics
are still published by older Unity builds and some test bench fixtures.

**Proof of death needed**: confirm with the Unity team that no shipping
build publishes to `/unity/teach_status` or `/unity/trajectory_data` for
one full release. Capture a `ros2 bag record` from a real demo and grep
for the topic names — must be empty.

### 4.2 `record_start`, `record_stop`, `preview_sim` actions (in `teach_job_handler.py`)

The handler's own docstring admits "Unity owns capture today", yet the
actions are wired and tested. They exist so a host-side recorder can be
re-introduced (multi-robot architecture work may want it).

**Proof of death needed**: AGENTS.md "Removal Pass" can drop them once the
multi-robot work confirms Unity stays the recorder of record.

### 4.3 `FAST_REPEAT_*` + `fastest_path_repeat` profile (in `motion_config.py` + `trajectory_recorder.py`)

The `PLAYBACK_EXECUTION_PROFILE = "preserve_timing"` default never touches
the dormant branch. The fast-repeat profile reads a parallel set of
constants (FAST_REPEAT_SPEED_J/ACC_J/SPEED_L/ACC_L/CP/FINAL_CP/
LOOKAHEAD_SEC/TIMEOUT_PER_COMMAND_SEC/MAX_COMMANDS_PER_CYCLE) and was the
historical "play it as fast as possible" path.

**Proof of death needed**: no `job_request.options.execution_profile =
"fastest_path_repeat"` payload appears in any saved Unity demo log or in
any tool under `tools/`.

### 4.4 `USE_MIXED_PRIMITIVES = False` default

The mixed-primitive Cartesian compile path (LINE → MovL, ARC → Arc,
GENERAL → JointMovJ chain) is fully verified on real hardware. The flag
stays False because (a) it was the historical default and (b) we have not
yet swept enough demo paths through the classifier in production. Phase E1
of the refactor below exposes it as a ROS param so opt-in is easy.

## 5. File map (you'll edit these often)

```
project_teleop/
├── AGENTS.md                          ← this file
├── .claude-handoff/                   ← repo-local progress-report skill
├── docs/                              ← architecture + setup + legacy
├── tools/
│   ├── analyze_teleop_session.py      ← offline log analyser
│   ├── monitor/                       ← PyQt6 operator panel
│   ├── log_replay/                    ← (after F1) three-layer review pipeline
│   └── demo_lift/                     ← demo scripts
├── src/
│   ├── dobot_mg400/
│   │   ├── mg400_protocol/            ← TCP codec + alarm tables
│   │   ├── mg400_controller/          ← *** MAIN PACKAGE — most edits here ***
│   │   │   ├── vr_teleop_node.py      ← ROS node entry; reads Unity, writes MG400
│   │   │   ├── monitor_gui.py         ← embedded PyQt monitor
│   │   │   ├── common/
│   │   │   │   ├── config/motion_config.py        ← all tuning constants
│   │   │   │   ├── config/robot_config.py         ← joint limits, control mode
│   │   │   │   ├── core/robot_connection.py       ← TCP sockets to MG400
│   │   │   │   ├── core/feedback_handler.py       ← parses 1440-byte feedback
│   │   │   │   ├── core/command_sender.py         ← background queue → port 30003
│   │   │   │   ├── logic/teleop_controller.py     ← *** adaptive realtime gate ***
│   │   │   │   ├── logic/motion_planner.py        ← renders MG400 command strings
│   │   │   │   ├── logic/joint_validator.py       ← clamp to joint limits
│   │   │   │   ├── logic/target_compensator.py    ← Unity latency comp
│   │   │   │   ├── logic/scene_safety_guard.py    ← workspace box guard
│   │   │   │   ├── trajectory/trajectory_recorder.py ← *** facade after C1-C3 split ***
│   │   │   │   ├── trajectory/teach_job_handler.py   ← /teach/job_request router
│   │   │   │   ├── utils/error_decoder.py         ← *** USE THIS for alarm text ***
│   │   │   │   ├── utils/kinematics.py            ← MG400 FK + IK
│   │   │   │   ├── utils/adaptive_telemetry.py    ← per-cmd analytics CSV
│   │   │   │   ├── utils/unified_triple_logger.py ← Unity→ROS→robot CSV log
│   │   │   │   └── utils/clock_calibrator.py      ← Unity-ROS clock drift
│   │   │   └── test/                              ← unittest suite
│   │   ├── mg400_bringup/             ← launch files
│   │   └── mg400_description/         ← URDF + meshes
│   ├── robot_teaching_core/           ← vendor-neutral teaching abstractions
│   ├── adapters/mg400/                ← bridge teaching_core → mg400_protocol
│   └── ROS-TCP-Endpoint/              ← vendored Unity bridge
├── Dobot_TCP_IP_Python_V4/            ← vendor SDK (do not touch)
└── MG400_Mock/                        ← docker mock target
```

## 6. ROS interface contract

Topics published / subscribed by `vr_teleop_node`:

| Topic | Direction | Status | Purpose |
|---|---|---|---|
| `/unity/joint_cmd` | sub | active | Unity → realtime joint targets |
| `/unity/speed_factor` | sub | active | Unity → SpeedFactor 0-100% |
| `/teach/job_request` | sub | active | Unity → typed teach action (compile/preview_sim/execute/export/stop/record_*) |
| `/teach/job_status` | pub | active | ROS → Unity job lifecycle |
| `/teach/job_artifact` | pub | active | ROS → Unity compiled plan / preview frames |
| `/unity/teach_status` | sub | **LEGACY 4.1** | older Unity teach control surface |
| `/unity/trajectory_data` | sub | **LEGACY 4.1** | older Unity trajectory dump |
| `/mg400/joint_trajectory_controller/command` | sub | **LEGACY 4.1** | older JointTrajectory-style publisher |
| `/vr/suction_cmd` | sub | active | Unity → vacuum gripper bool |
| `/mg400/light_cmd` | sub | active | Unity → digital output light |
| `/joint_states` | pub | active | RViz feedback |
| `/mg400/robot_mode` | pub | active | Int32 enable/running/error |
| `/mg400/error_status` | pub | active | Int32 controller error |

`teach_job_request.action` values currently routed: `compile`, `preview_sim`
(LEGACY 4.2), `execute`, `export`, `stop`, `record_start` (LEGACY 4.2),
`record_stop` (LEGACY 4.2).

## 7. How to run

### Real robot on Ubuntu

```bash
cd ~/project_teleop_ws
colcon build --packages-select mg400_protocol mg400_controller mg400_bringup
source install/setup.bash

# Robot IP can be overridden via env var
export ROBOT_IP=192.168.1.6
ros2 launch mg400_bringup main.launch.py

# Or via the convenience launcher (tmux panes)
./start_teleop.sh
```

### MG400 mock (no real hardware)

```bash
# Start the docker mock in another terminal
cd MG400_Mock && docker compose up
# Point the controller at it
export ROBOT_IP=127.0.0.1
ros2 launch mg400_bringup main.launch.py
```

### Unity bridge

Make sure ROS-TCP-Endpoint is running (it is launched by
`main.launch.py`). Unity client connects to the host on port 10000 by
default.

### Monitor GUI

```bash
ros2 run mg400_controller monitor_gui
# or
python3 -m mg400_controller.monitor_gui
```

## 8. How to test

### Full unittest sweep (from the controller package dir)

```bash
cd src/dobot_mg400/mg400_controller
PYTHONPATH=".:../mg400_protocol:../../robot_teaching_core" \
  python3 -m unittest discover -s test -p "test_*.py" -v
```

### Single test file

```bash
PYTHONPATH=".:../mg400_protocol:../../robot_teaching_core" \
  python3 test/test_queue_aware_logic.py -v
```

### Skill-driven progress report

```bash
.claude-handoff/run_tests.sh                 # prints PASS/FAIL summary
```

### colcon (preferred for CI)

```bash
colcon test --packages-select mg400_controller robot_teaching_core
colcon test-result --verbose
```

### Hardware smoke (manual)

1. Bring up `main.launch.py` against a real robot.
2. In Unity, enter realtime mode, do a small wrist motion. **Expectation**: smooth, no jitter, no alarm.
3. Switch to teach mode, record a 5-second path. Press play. **Expectation**: same smooth motion at the chosen playback Speed/Acc.
4. Watch `~/.ros/adaptive_telemetry/*.csv` grow.
5. Confirm `/teach/job_status` reaches `stage: done`.

## 9. Branch state

```
feat/multi-robot-teaching-architecture
   │
   └── 0ecdd35 (baseline)
        │
        └── realtime-smooth-adaptive  ← we are here, 8 commits ahead
```

Merge plan: keep `realtime-smooth-adaptive` open for behaviour-preserving
refactor commits (see §10 next-steps queue). Merge back to
`feat/multi-robot-teaching-architecture` once handoff doc + skill +
characterisation tests land. Removal Pass v1 (§11) is a separate branch.

## 10. Next-steps queue (refactor)

- [x] **A** docs + .claude-handoff + LEGACY markers (commits 4c664ee, d0e070f)
- [x] **B1** characterisation: adaptive gate tuning constants (ecda77e)
- [x] **B2** characterisation: teach legacy actions tripwire (aa857fd)
- [x] **B3** characterisation: ROS wiring legacy topics tripwire (08dc5bb)
- [x] **B4** characterisation: mixed-primitive Cartesian golden fixture (39a237d)
- [x] **B5** characterisation: fastest_path_repeat dormant profile (6d60746)
- [x] **C1** refactor: extract `trajectory_io.py` (089822f)
- [x] **C2** refactor: extract `playback_compiler.py` (4d89d81)
- [x] **C3** refactor: extract `playback_executor.py` (82e58c5) — leaf helpers only;
      `start_preview` / `_play_worker` orchestrator stays on recorder
- [x] **D1** refactor: group vr_teleop_node wiring (`_register_ros_subscriptions`) (b5b48a1)
- [x] **E1** feat: expose `USE_MIXED_PRIMITIVES` as ROS param (6c7c34c)
- [ ] **F1** chore: organise `tools/log_replay/` — deferred; the
      `build_three_layer_review.py` / `build_trim_review.py` /
      `build_validation_analysis_dashboard.py` /
      `export_clean_three_layer_csv.py` / `validation_analysis_gui.py`
      scripts referenced by the analyser docs are **not present** on
      this branch.  Land them under `tools/log_replay/` when they
      arrive (sibling-branch import).
- [ ] **C4 (follow-up)** finish the executor extraction: move
      `start_preview` / `_play_worker` / `_send_go_to_start` /
      `_wait_until_near_target` / `_dispatch_event_command` /
      `_schedule_delayed_event_command` / `_playback_complete` from
      recorder into `playback_executor.PlaybackExecutor` class once
      the recorder's state-holder surface is isolated.  Blocked on a
      design pass — recorder owns the stop flag, timer list, kinematics
      feedback fn, and a half-dozen callbacks.

## 11. Removal Pass v1 (deferred — separate plan)

After all of §10 is shipped and the next engineer has had at least one
release of bake time, the following candidates can be removed in a single
deliberate cleanup. **Do not do this without a fresh ADR**:

1. Legacy callbacks §4.1 (after Unity team signs off).
2. Legacy teach actions §4.2 (after multi-robot work confirms Unity
   remains recorder of record).
3. `fastest_path_repeat` profile §4.3 (after demo logs show no
   operator override exists).

Each removal must keep the characterisation test from §10 as a
tripwire: deleting the legacy code requires deleting its test in the
same PR.

## 12. Gotchas

### 12.1 Alarm 34322 — "Position is out of range"
**Not** a speed-tracking failure. Look at `motion_config.py` block comment.
Root cause: CP=80 corner-cut between two Cartesian Arc commands at SpeedL=60
produces an interpolated setpoint that briefly leaves the reachable joint
configuration. Fix: `SEGMENT_CARTESIAN_CP_MAX = 30` caps the CP threaded
into MovL/Arc commands in the mixed compile path. SpeedJ for JointMovJ is
**unaffected** — only Cartesian primitives get the cap.

### 12.2 `CurrentCommandId` (offset 1112 in feedback packet)
Defined in `mg400_protocol/feedback.py`. Does **not** track JointMovJ on
the firmware we tested — stays at 0. The adaptive realtime gate therefore
uses backlog + hand velocity instead of CommandId rising-edges. Do not
rebuild the gate around CommandId without verifying on the operator's
firmware.

### 12.3 Unity J* columns ≠ robot session joint columns
`ValidationPath_*.csv` carries Unity-side J1-J4 (Unity local frame, with
Unity's own conventions). The robot session `teleop_session_*.csv` carries
`unity_raw_j*_deg` (what Unity sent ROS) and `robot_j*_deg` (what the MG400
reports). Pre-compaction matching via `exact_sync_data.json` reconciles
them with FK transforms — do not assume `csv["J1"] == csv["robot_j1_deg"]`.

### 12.4 SpeedFactor 50 in the logs
`feedback_handler.py` info-logs `🚀 SpeedFactor Confirm: 50` every 3 s when
the controller reports SpeedScaling=50. This is not a bug — it is a status
print. Source of 50 is usually Unity's `/unity/speed_factor` slider or a
prior Dobot Studio session. Override at runtime with
`ros2 topic pub --once /unity/speed_factor std_msgs/Float64 "data: 100.0"`.

### 12.5 `stop_all(go_home=True)`
The `TrajectoryRecorder.stop_all` path sends the robot to joint (0,0,0,0)
*and* blocks live-teleop sends for 5 s afterwards (`_block_until`). If a
teach job aborts mid-playback, expect a 5 s blackout before realtime
resumes.

### 12.6 Clock drift in `capture/clock.py`
`ClockCalibrator` estimates the Unity→ROS clock offset over a 50-sample
window with min-window filtering and drift compensation. First few seconds
of a session are noisy — give it ~3 s before trusting `T1` timestamps in
the triple logger.

### 12.7 Mock joint match
`MG400_FEEDBACK_TEST_VALUE = 0x0123456789ABCDEF`. Mock simulators emit
`test_value=0` instead; `feedback.parse_packet(allow_mock_zero_test_value=True)`
accepts that. Real packets must match the magic number — a 0 from the real
robot means a corrupt packet was received.

### 12.8 Adaptive telemetry path
`~/.ros/adaptive_telemetry/<session>.csv`. Override with
`MG400_ADAPTIVE_TELEMETRY_DIR=/some/path`. Lives separately from
`unified_triple_logger`'s output so post-hoc analysis of the adaptive
gate doesn't mix with the production cmd-vs-feedback CSV.

## 13. Where to look when X breaks

| Symptom | First place to look |
|---|---|
| Jitter in realtime | `teleop_controller.should_send_command` + `motion_config.REALTIME_*` |
| Teach playback stops mid-path | `trajectory_recorder._play_worker` + `_playback_timed_out` |
| Alarm during teach | `error_decoder.decode_error(...)`; for 34322 see §12.1 |
| Unity command arrives but robot does not move | `RobotConnection.connected`, `RobotMode != ENABLE (5)`, `RequestControl()` returning -10000 |
| Logger CSV columns wrong | `unified_triple_logger._build_columns` |
| Test fails post-refactor | run B1-B5 characterisation suite; one of them will catch the regression by design |

## 14. Contact / authoring notes

This handoff doc was drafted during the May 2026 thesis push. The plan
file in `/Users/thesun/.claude/plans/spicy-purring-tome.md` carries the
detailed reasoning for §10.

When editing AGENTS.md, keep the §4 "deferred / known smells" entries
in lockstep with the corresponding LEGACY comments in the source. If
you remove an item from §4, you must remove its LEGACY comment in the
same commit.
