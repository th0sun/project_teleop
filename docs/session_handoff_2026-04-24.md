# Session Handoff - 2026-04-24

This note records the work and decisions from the current cleanup/refactor
session so another person or AI agent can continue without relying on chat
history.

Companion guide:

- `docs/controller_continuation_guide.md` is the shorter operational map
- this file is the fuller session log and decision history

## Starting Context

Workspace root:

```text
/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws
```

Main project repo:

```text
/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop
```

Important distinction:

- `project_teleop_ws` is not a git repository.
- `project_teleop` is the main git repository.

Current working branch created during this session:

```text
chore/prepare-mg400-mock-cleanup
```

## Initial Scan

The workspace was scanned before making changes.

Findings:

- Root `project_teleop_ws` was a mixed workspace containing the real ROS2
  project plus many support files.
- Main ROS2 project folder is `project_teleop`.
- Root workspace itself is not a git repo.
- `project_teleop` is a git repo.
- External/supporting materials were outside the main project.

## Workspace Organization

Support files outside `project_teleop` were organized into:

```text
_supporting_materials/
```

Main categories created:

- `_supporting_materials/unity/project_teleop_unity`
- `_supporting_materials/docs/api_and_plans`
- `_supporting_materials/docs/robot_reference`
- `_supporting_materials/docs/web_demos`
- `_supporting_materials/data/session_logs/raw_sessions`
- `_supporting_materials/data/trajectories/json_trajectories`
- `_supporting_materials/experiments/runs`
- `_supporting_materials/presentation/deck_assets`
- `_supporting_materials/presentation/slide`
- `_supporting_materials/tools/robot_control_tools`
- `_supporting_materials/external/mg400_mock_digital_twin`
- `_supporting_materials/local_runtime/python_envs/.venv`
- `_supporting_materials/local_runtime/python_envs/venv_monitor`
- `_supporting_materials/local_runtime/cache`
- `_supporting_materials/config/pyrightconfig.json`

Nothing in `project_teleop` was deleted for that organization step.

## Git State Observed

`project_teleop` had existing dirty state before this cleanup branch:

- `Dobot_TCP_IP_Python_V4` had submodule/untracked state at the start of the
  cleanup branch.
- `canvas_test.py`, `flow_canvas.py`, `test_dual_db.py`,
  `test_teach_repeat.py` were already deleted.
- `src/dobot_mg400/mg400_controller/mg400_controller/common/trajectory/trajectory_recorder.py`
  was already modified.

Those existing changes were not reverted.

## Unity-Like Simulator Location

The "Unity simulation" the user meant was not the real Unity Editor project.
The relevant Unity-like GUI simulator is:

```text
project_teleop/tools/mg400_simulator
```

Key files:

- `tools/mg400_simulator/main.py`
- `tools/mg400_simulator/run.sh`
- `tools/mg400_simulator/ui/main_window.py`
- `tools/mg400_simulator/ui/robot_viewport.py`
- `tools/mg400_simulator/core/urdf_loader.py`
- `tools/mg400_simulator/core/unity_tcp_bridge.py`
- `tools/mg400_simulator/core/ros_bridge.py`

Purpose:

- PyQt5 + OpenGL desktop simulator.
- Unity-like GUI stand-in.
- Supports robot viewport, joint sliders, end-effector dragging, Teach & Repeat,
  and ROS/ROS-TCP bridge paths.

## MG400_Mock Investigation

Two MG400 mock copies were compared:

1. Project copy:

```text
project_teleop/MG400_Mock
```

2. Extra downloaded upstream-ish copy:

```text
_supporting_materials/external/mg400_mock_digital_twin
```

The extra downloaded repo points to:

```text
https://github.com/HarvestX/MG400_Mock.git
```

It was at:

```text
a681ee867454e7866d98ea326b2701d0b1cc6870
```

The project copy was not the same as upstream. It had project-specific changes:

- CP blending support.
- Command streaming while robot is in `MODE_RUNNING`.
- TCP packet buffering/splitting fixes.
- More dashboard query/control commands such as `GetTool()`, `RobotMode()`,
  `GetAngle()`, `GetPose()`, `Pause()`, `Continue()`, `EmergencyStop()`.
- CP benchmark tooling.
- Old docker layout under `MG400_Mock/docker/`.

Important conclusion:

- The modified `MG400_Mock` had been adapted for the project.
- Returning to upstream `MG400_Mock` is acceptable only if the real robot is the
  target and Mock compatibility is not required.
- The user confirmed Mock not supporting some project CP behavior is fine as
  long as the real robot supports it.

## Submodule Restoration

The user wanted dependencies restored as submodules.

Submodules now recorded in `.gitmodules`:

```text
[submodule "src/ROS-TCP-Endpoint"]
	path = src/ROS-TCP-Endpoint
	url = https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
	branch = main-ros2

[submodule "MG400_Mock"]
	path = MG400_Mock
	url = https://github.com/HarvestX/MG400_Mock.git

[submodule "Dobot_TCP_IP_Python_V4"]
	path = Dobot_TCP_IP_Python_V4
	url = https://github.com/Dobot-Arm/TCP-IP-Python-V4.git
```

Current submodule commits:

```text
65a19c9e18f7c67869136620a330cbd5f48fc0b3 Dobot_TCP_IP_Python_V4
a681ee867454e7866d98ea326b2701d0b1cc6870 MG400_Mock
54c1a64b6d5ef6ffa0a0431570bb74329b79b15b src/ROS-TCP-Endpoint
```

`git submodule status` passes after adding the missing
`Dobot_TCP_IP_Python_V4` mapping.

The old modified `MG400_Mock` was backed up before replacing it with upstream
submodule checkout:

```text
_supporting_materials/backups/MG400_Mock_before_submodule
```

## README Update

`README.md` was updated to reflect:

- `Dobot_TCP_IP_Python_V4` is a submodule.
- `MG400_Mock` is a submodule.
- `src/ROS-TCP-Endpoint` is a submodule.
- `tools/mg400_simulator` is the PyQt/OpenGL Unity-like GUI simulator.
- Current upstream `MG400_Mock` docker commands use root `docker-compose.yml`:

```bash
cd MG400_Mock
docker compose up
docker compose down
docker compose -f test-docker-compose.yml run test_dobot python3 -m unittest discover -s tests
```

## start_teleop.sh Update

Because upstream `MG400_Mock` now uses root `docker-compose.yml` instead of
`docker/docker-compose.yml`, `start_teleop.sh` was updated to choose:

1. `MG400_Mock/docker-compose.yml`
2. fallback `MG400_Mock/docker/docker-compose.yml`

This preserves compatibility if a fork or older mock layout is used later.

## Simulator Bridge Fix

Problem found:

- `tools/mg400_simulator/core/ros_bridge.py` native ROS path published
  `/unity/joint_cmd` as `std_msgs/Float64MultiArray` and kept GUI degrees.
- `vr_teleop_node.py` subscribes to `/unity/joint_cmd` as
  `sensor_msgs/JointState` and expects radians.
- The fallback `UnityTcpBridge` already did the correct conversion.

Fix made:

- Native ROS path now publishes `sensor_msgs/JointState`.
- GUI degrees are converted to radians before publishing.
- Joint names are set to `joint1` to `joint4`.
- `header.stamp` is set from the ROS node clock.

Verification:

```bash
python3 -m py_compile project_teleop/tools/mg400_simulator/core/ros_bridge.py
```

passed.

## Teleop Command Logic Documentation

Created:

```text
docs/teleop_command_logic.md
```

This file documents the data path from Unity/GUI to robot:

```text
Unity / Unity-like GUI
        |
        v
/unity/joint_cmd
sensor_msgs/JointState.position in radians
        |
        v
TeleopNode._unity_callback()
        |
        v
JointValidator.validate_and_clamp()
        |
        v
Clock correction + latency compensation
        |
        v
latest_raw_target / latest_target
        |
        v
TeleopNode._high_precision_control_loop()
        |
        v
TeleopController.should_send_command()
        |
        v
TeleopController.format_command_string()
        |
        v
MotionPlanner.format_command()
        |
        v
CommandSender.send()
        |
        v
RobotConnection.send_motion_cmd()
        |
        v
TCP 30003 -> MG400
```

## Current Teleop Logic Summary

The teleop node has two layers of mode selection at startup:

- Motion command type:
  - `jointmovj` -> `JointMovJ(...)`
  - `movj` -> `MovJ(...)`
  - `movl` -> `MovL(...)`
- Teleop logic mode:
  - `default`: production logic through `TeleopController` and
    `MotionPlanner`.
  - `m8_raw`, `m11`, `m14`, `m15`, `m16`, `m17`: experimental logic through
    `ExperimentalStrategy`.

Important correction: the flow below is the default path only. If
`LOGIC_MODE != "default"`, the node uses `latest_raw_target`, bypasses
`TeleopController`, `MotionPlanner`, and `TargetPredictor`, formats the Dobot
command directly, sends it through `CommandSender`, and returns before the
default path runs.

Git history check for the logic modes:

- `9196b01` at `2026-02-24 17:43:33 +0700` is the relevant end-of-day snapshot
  for comparing current default logic.
- By `2026-02-24`, the default path already used dynamic proximity and
  `TargetPredictor`, and it also used `plan_batch_motion()` for low-speed
  fine aiming.
- `M11/M14/M15` were added on `2026-03-02` in `59782e9`.
- `M8_RawData` was added on `2026-03-02` in `9a475d6`.
- `ExperimentalStrategy` was refactored on `2026-03-02` in `087f36c`.
- Dynamic `LOGIC_MODE` import was fixed on `2026-03-03` in `05b445f`.
- `M16_AdaptCP` and `M17_CleanFF` were added on `2026-03-14` in `9058b70`.
- The current default logic differs from `2026-02-24` mainly in target leading
  and low-speed formatting: it replaced predictor-based forwarding with
  timing-based latency compensation, removed low-speed batch interpolation from
  the default path, widened the dynamic proximity base, shortened stuck time,
  raised CP from 80 to 100, and changed stuck target-change tuning.
- Default-only comparison was added to `docs/teleop_command_logic.md`: current
  default mode stores latency-compensated `latest_target`, while
  `2026-02-24` used `TargetPredictor`; current `MotionPlanner` still has
  `plan_batch_motion()`, but default mode does not call it.
- Added a queue-semantics audit based on the `4axis` TCP/IP guide:
  MG400 motion commands are queued commands on port `30003`, so "follow latest
  hand value" depends more on keeping queue depth shallow than on increasing
  send frequency. Current default mode already avoids batch interpolation, but
  it still does not use feedback fields like `RunQueuedCmd` or `QTarget` to
  gate queue depth.
- Added a design-options section in `docs/teleop_command_logic.md` covering:
  heuristic-only tuning, queue-aware backlog gate, command-ID-assisted gating,
  stop-and-replace, and reintroducing micro-step batching. Recommended path is
  queue-aware backlog gating as the new default direction.
- Implemented first-pass queue-aware default gating on this branch:
  `FeedbackHandler` now parses `QTarget` and `RunQueuedCmd`; default mode gates
  sends when backlog `max(abs(QTarget[:4] - QActual[:4]))` exceeds
  `QUEUE_BACKLOG_GATE_RAD = 0.01` rad; `TeleopController` send-state is now
  committed only after successful socket send. Added local unit tests in
  `src/dobot_mg400/mg400_controller/test/test_queue_aware_logic.py`, and
  verified with `py_compile` plus targeted `unittest`.

`TeleopNode._unity_callback()`:

- Receives `/unity/joint_cmd`.
- Ignores disconnected robot or fewer than 4 positions.
- Rejects NaN input.
- Validates/clamps joints.
- Corrects Unity timestamp with `ClockCalibrator`.
- Estimates input velocity by EMA.
- Applies latency compensation capped at 80 ms.
- Re-validates compensated target.
- Stores `latest_raw_target` and `latest_target`.
- Does not send directly.

`TeleopNode._high_precision_control_loop()`:

- Runs at 200 Hz.
- Calls `_control_loop_step()`.
- Reads latest robot feedback.
- Blocks teleop during Teach & Repeat playback/homing.
- Publishes monitor/status data.
- Then asks `TeleopController.should_send_command()` whether to send.

`TeleopController.should_send_command()`:

- Sends first command immediately.
- Uses dynamic proximity:

```text
trigger_distance = DYNAMIC_PROXIMITY_BASE_RAD + velocity_mag * DYNAMIC_PROXIMITY_LOOKAHEAD_SEC
```

- Sends when robot is close enough to the last command and target changed more
  than `SPATIAL_THRESHOLD`.
- Has stuck recovery when velocity is low and robot remains far from target.

`MotionPlanner.format_command()`:

- Converts radians to degrees.
- Emits one of:

```text
JointMovJ(...,SpeedJ=100,AccJ=100,CP=100)
MovJ(...,SpeedJ=100,AccJ=100,CP=100)
MovL(...,SpeedJ=100,AccJ=100,CP=100)
```

`RobotConnection.send_motion_cmd()`:

- Sends final motion command to TCP port `30003`.
- Appends newline when missing.
- Reconnects motion socket if needed.

## Timing Fix Status

The timing bug described earlier has been fixed in this session.

What changed:

- `vr_teleop_node.py` now passes monotonic `now` into
  `TeleopController.should_send_command(...)`
- `TeleopController.should_send_command(...)` no longer calls
  `update_robot_state()` internally
- controller velocity and gating now share the same monotonic time source from
  `_control_loop_step()`

Validation:

- `python3 -m py_compile` passed for the edited files
- `python3 -m unittest .../test_queue_aware_logic.py` passed with `3/3` tests,
  including a new deterministic regression test for the timing path

## Queue Gate Escape Hatch (Follow-Up)

After review, the queue-aware guard had one remaining risk: it returned
`QueueBusy` before stuck recovery could run.

Fix applied:

- added `QUEUE_BUSY_ESCAPE_SEC = 0.30`
- added `TeleopController.queue_busy_start_time`
- when `RunQueuedCmd` stays busy and backlog remains above
  `QUEUE_BACKLOG_GATE_RAD`, the controller still blocks new commands while the
  robot is moving
- if robot velocity stays below `STUCK_VELOCITY_THRESHOLD` longer than
  `QUEUE_BUSY_ESCAPE_SEC`, the controller allows the normal stuck recovery path
  to run
- `mark_command_sent(...)` resets the queue-busy timer after a send succeeds

Validation after the fix:

- `python3 -m py_compile` passed
- `python3 -m unittest .../test_queue_aware_logic.py` passed with `5/5` tests
- MG400_Mock Docker benchmark still runs after the script update

MG400_Mock reminder:

- `queue_running_ratio` was still `0.0` in mock runs because upstream
  `MG400_Mock` does not populate `RunQueuedCmd`
- this means the mock can catch socket/script regressions, but cannot prove the
  real queue gate behavior

## Production-Only Refactor For Submission

The user clarified that the final project should keep only the real production
runtime in the ROS2 package. Experimental modes should not remain in the runtime
source path.

Changes applied:

- removed runtime selection for old experimental modes from `vr_teleop_node.py`
- removed the experimental branch that imported and ran `ExperimentalStrategy`
- removed `latest_raw_target`, because production no longer needs the raw
  experimental input path
- removed `LOGIC_MODE` and `RAW_HZ` from `robot_config.py`
- removed the interactive `hz <num>` command that only existed for Raw mode
- moved the old experimental source snapshot out of the package to:
  `docs/report_materials/archive/experimental_logic_reference.py`
- added report-only material under:
  `docs/report_materials/`

Current boundary:

- production code lives under `src/dobot_mg400/...`
- report/story material lives under `docs/report_materials/`
- `README.md` describes the project only and does not discuss report-writing
  material or archived experiments

Validation after refactor:

- `python3 -m py_compile` passed for the edited production files
- `python3 -m unittest .../test_queue_aware_logic.py` passed with `5/5` tests
- search confirmed no `ExperimentalStrategy`, `LOGIC_MODE`, `RAW_HZ`,
  `latest_raw_target`, or old experimental mode keys remain in production source,
  README, or `docs/teleop_command_logic.md`

## Project-Wide Refactor Pass (2026-04-25)

The user requested a heavier real-project cleanup for final project submission.

Organization changes:

- moved monitor tools from repository root to `tools/monitor/`
  - `monitor_bridge.py`
  - `mg400_monitor_pyqt6.py`
  - `requirements_monitor.txt` -> `tools/monitor/requirements.txt`
  - `ros_interface.json`
- added `tools/monitor/README.md`
- moved project setup and architecture docs into `docs/`
  - `SETUP_GUIDE.md` -> `docs/setup_guide.md`
  - `teleop_architecture.md` -> `docs/teleop_architecture.md`
- moved old/generated report material into `docs/report_materials/archive/`
  - `teleop_report.html`
  - `plot_kalman_performance.py`
  - old `target_predictor.py` snapshot as `target_predictor_reference.py`

Runtime cleanup:

- removed unused `TargetPredictor` import and instance from `vr_teleop_node.py`
- removed unused `current_cmd_target` field from `TeleopNode`
- removed unused `MAX_SPEED_DEG` and `RATE_FLOOR_SEC` constants
- removed obsolete predictor/Kalman wording from the production callback comment
- updated `start_teleop.sh` to resolve paths from the script location and run
  `tools/monitor/monitor_bridge.py`
- updated `docs/setup_guide.md` paths for monitor tools
- kept `README.md` focused on the project itself, not report-writing material

Validation after this pass:

- `python3 -m py_compile` passed for production files and moved monitor bridge
- `python3 -m unittest .../test_queue_aware_logic.py` passed with `5/5` tests
- `bash -n start_teleop.sh` passed
- `git diff --check` passed for touched files

## Files Added Or Edited In This Session

Added:

- `.gitmodules`
- `docs/teleop_command_logic.md`
- `docs/session_handoff_2026-04-24.md`

Edited:

- `README.md`
- `start_teleop.sh`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/logic/teleop_controller.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py`
- `tools/mg400_simulator/core/ros_bridge.py`

Converted/replaced in git index:

- `MG400_Mock` from tracked directory back to submodule gitlink.

Preserved existing unrelated dirty state:

- Deleted root test/canvas files remained deleted.
- `trajectory_recorder.py` had pre-existing modifications and was not cleaned up.
- Dobot manuals were moved out of `Dobot_TCP_IP_Python_V4` into
  `docs/reference_manuals/dobot/` so the submodule working tree is clean again.

## Suggested Continuation Order

1. Decide whether `force_send` should be removed or implemented.
2. Decide whether speed stays fixed at 100 or returns to adaptive speed.
3. Run ROS-side tests/build where available.
4. When hardware is available, validate `RunQueuedCmd`-based queue gating on
   real MG400 feedback.
5. Only after logic is stable, consider committing the submodule cleanup and docs.

## Mock Benchmark Follow-Up (2026-04-24)

We tested the new queue-aware default logic against the upstream `MG400_Mock`
Docker setup before trying the real robot.

### What Was Added

- `tools/queue_logic_mock_benchmark.py`

This benchmark replays one scripted joint trajectory and compares:

- `old24`: approximation of the `2026-02-24` default path
- `oldcurrent`: current default before queue-aware gating
- `new`: new queue-aware default path
- `newproxy`: mock-only proxy that gates on backlog even when
  `RunQueuedCmd == 0`

### Benchmark Results

- `old24`
  - commands_sent: `12`
  - mean_latest_error_deg: `20.2530462671072`
  - p95_latest_error_deg: `43.0735`
  - max_latest_error_deg: `54.65395034722223`
  - mean_backlog_deg: `12.37694741985145`
  - max_backlog_deg: `43.14350720586672`
- `oldcurrent`
  - commands_sent: `4`
  - mean_latest_error_deg: `18.932852751295638`
  - p95_latest_error_deg: `45.099759567923385`
  - max_latest_error_deg: `52.169511112983436`
  - mean_backlog_deg: `7.348323923322442`
  - max_backlog_deg: `53.059`
- `new`
  - commands_sent: `9`
  - mean_latest_error_deg: `18.926407293573774`
  - p95_latest_error_deg: `44.16749688143903`
  - max_latest_error_deg: `51.13933718802147`
  - mean_backlog_deg: `9.401295084971943`
  - max_backlog_deg: `53.275450000000006`
- `newproxy`
  - commands_sent: `9`
  - mean_latest_error_deg: `24.72211700472992`
  - p95_latest_error_deg: `46.9302`
  - max_latest_error_deg: `49.27972878363826`
  - mean_backlog_deg: `10.697742050030056`
  - max_backlog_deg: `46.92935`

### Critical Limitation

The upstream mock does not currently populate `RunQueuedCmd` in its feedback
packet and does not implement `CP` handling for `JointMovJ`.

Implication:

- the real queue-aware branch in the new code was not actually exercised in a
  faithful way by this mock
- CP-related behavior cannot be compared here either

### Current Recommendation

The mock results still reinforce one useful point:

- the `2026-02-24` style low-speed micro-step batching is a poor default if the
  main goal is to follow the latest hand target

But the new queue-aware gate must still be validated on the real MG400, not on
the upstream mock alone.

## Refactor Continuation (2026-04-25)

The current refactor is being done with reusable boundaries in mind, not just
line-count reduction in `vr_teleop_node.py`.

### Reusable Components Added

- `common/utils/mode_selection.py`
  - owns interactive selection of `CONTROL_MODE`
  - keeps startup/operator UI out of the ROS node implementation
- `common/utils/async_event_logger.py`
  - reusable background event logger for CSV rows and named event handlers
  - keeps disk I/O out of ROS callbacks and control-loop timing
- `common/logic/target_compensator.py`
  - owns Unity/VR target latency compensation
  - has no ROS dependency and is covered by unit tests
- `common/config/motion_config.py`
  - now owns shared ROS topic names used by both `vr_teleop_node.py` and
    `monitor_gui.py`
  - removes duplicated topic strings from the teleop/monitor boundary
- `common/ros/teleop_interfaces.py`
  - now owns publisher/subscriber creation for the teleop node
  - makes the ROS2 interface layer explicit instead of hiding it under generic
    utilities
- `common/utils/monitor_logging.py`
  - now owns monitor session logging and manual CSV logging
  - removes file-writing state and background log loop from `monitor_gui.py`
- `common/ros/monitor_interfaces.py`
  - now owns monitor control publishers, monitor subscriptions, and telemetry
    callback state
  - makes the monitor ROS2 boundary explicit and reusable
- `common/monitor/execution_metrics.py`
  - now owns monitor execution timing state
  - removes execution timing transitions from `monitor_gui.py`
- `common/monitor/joint_graph_buffer.py`
  - now owns rolling time-series buffers and axis helpers for monitor graphs
  - removes graph-buffer state and auto-scale math from `monitor_gui.py`
- `common/monitor/presentation_state.py`
  - now owns status text, Cartesian display mapping, and joint diff formatting
  - removes display-specific formatting and threshold coloring from `monitor_gui.py`
- `common/monitor/control_panel_state.py`
  - now owns digital-output lockout state for suction and lights
  - removes DO bit decode, pending button state, and lockout timing from `monitor_gui.py`

### Behavior-Sensitive Cleanup

- `vr_teleop_node.py` now uses monotonic time when updating
  `TeleopController.last_sent_time`.
- `LatencyAnalyzer.format_sent_report(...)` can receive the already-computed
  `time_since_last` so it does not mix wall-clock time with monotonic control
  time.
- `execute_motion_command(...)` now calls `mark_command_sent(...)` only after
  `send_command_with_sync(...)` reports success.
- hardcoded topic strings were removed from `vr_teleop_node.py` and
  `monitor_gui.py`; shared names such as `PREDICTED_TARGET_TOPIC`,
  `SENT_COMMAND_TOPIC`, `TOOL_ACTUAL_TOPIC`, and `TOOL_INDEX_TOPIC` now live in
  `motion_config.py`.
- teleop ROS publisher/subscriber setup now goes through
  `create_publishers(...)`, `create_subscriptions(...)`, and
  `attach_unity_subscription(...)` in `common/ros/teleop_interfaces.py`.
- monitor ROS publishers/subscriptions and telemetry state now go through
  `MonitorTelemetryState`, `create_control_publishers(...)`, and
  `create_monitor_subscriptions(...)` in `common/ros/monitor_interfaces.py`.
- monitor graph buffering and execution timing now go through
  `JointGraphBuffer` and `ExecutionMonitor` in `common/monitor/`.
- monitor display formatting now goes through
  `build_joint_display_rows(...)`, `build_cartesian_display_state(...)`, and
  `build_status_display_state(...)` in `common/monitor/presentation_state.py`.
- monitor digital-output control sync now goes through
  `MonitorControlPanelState` in `common/monitor/control_panel_state.py`.
- monitor logging now goes through `SessionLogger` and
  `ManualMonitorLogger` in `common/utils/monitor_logging.py`.
- Teach & Repeat playback start alignment now goes through small reusable
  helpers inside `TrajectoryRecorder`, including go-to-start command
  formatting, wait-until-near-target polling, and playback-complete checks.
- `TrajectoryRecorder` now also accepts injected `traj_dir`, `time_fn`, and
  `sleep_fn` so playback behavior can be unit-tested without touching the
  default filesystem path.
- the old monitor manual logging path also had a GUI-level bug where
  `update_gui()` referenced `xyz_tgt` and `xyz_act`; the new reusable logger API
  now receives the correct Cartesian values explicitly.

### Validation Added

- `test_async_event_logger.py`
- `test_target_compensator.py`

Validation commands that passed:

```bash
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_trajectory_recorder.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_monitor_control_panel.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_monitor_presentation_state.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_monitor_logging.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_monitor_domain_state.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_monitor_ros_wiring.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_teleop_ros_wiring.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_target_compensator.py
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest src/dobot_mg400/mg400_controller/test/test_async_event_logger.py src/dobot_mg400/mg400_controller/test/test_queue_aware_logic.py
python3 -m py_compile src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py src/dobot_mg400/mg400_controller/mg400_controller/common/logic/target_compensator.py src/dobot_mg400/mg400_controller/mg400_controller/common/utils/async_event_logger.py src/dobot_mg400/mg400_controller/mg400_controller/common/utils/mode_selection.py
python3 -m py_compile src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py src/dobot_mg400/mg400_controller/mg400_controller/monitor_gui.py src/dobot_mg400/mg400_controller/mg400_controller/common/config/motion_config.py
python3 -m py_compile src/dobot_mg400/mg400_controller/mg400_controller/common/trajectory/trajectory_recorder.py
python3 -m py_compile src/dobot_mg400/mg400_controller/mg400_controller/monitor_gui.py src/dobot_mg400/mg400_controller/mg400_controller/common/utils/monitor_logging.py src/dobot_mg400/mg400_controller/mg400_controller/common/ros/monitor_interfaces.py src/dobot_mg400/mg400_controller/mg400_controller/common/ros/teleop_interfaces.py src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/control_panel_state.py src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/execution_metrics.py src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/joint_graph_buffer.py src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/presentation_state.py
```
