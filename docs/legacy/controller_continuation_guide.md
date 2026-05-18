# MG400 Controller Continuation Guide

This guide is for maintainers, teammates, or AI agents continuing work on the
MG400 teleoperation runtime.

It is intentionally operational: where the production path lives now, what to
touch first, what not to break, and how to validate changes quickly.

Read this first for project intent:

- `docs/project_objective.md`
- `docs/teleop_command_logic.md`

## Scope

This guide covers:

- `src/dobot_mg400/mg400_controller/`
- the production teleop runtime
- the monitor GUI only where it shares interfaces with the runtime

It does not cover thesis/report writing material. Historical experiments live
under `docs/report_materials/`.

## Current Production Path

```text
Unity-like GUI / VR
    -> /unity/joint_cmd
    -> vr_teleop_node.py
    -> TargetLatencyCompensator
    -> TeleopController
    -> MotionPlanner
    -> RobotConnection / CommandSender
    -> MG400 motion queue
```

Current production assumptions:

- teleop logic mode is the queue-aware `default`
- robot command mode is selected at startup, defaulting to `jointmovj`
- the real robot is the primary target
- upstream `MG400_Mock` is useful for basic flow checks but does not fully
  model real queue-state behavior

## Source Of Truth

Use these files first when continuing work:

- [vr_teleop_node.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py)
  - orchestration node
  - owns ROS publishers/subscribers and high-level runtime wiring
- [teleop_controller.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/logic/teleop_controller.py)
  - queue-aware send decision
  - stuck recovery
  - send-state contract
- [target_compensator.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/logic/target_compensator.py)
  - latency compensation for incoming Unity targets
- [motion_config.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/config/motion_config.py)
  - runtime thresholds
  - shared ROS topics for teleop and monitor
- [teleop_interfaces.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/ros/teleop_interfaces.py)
  - ROS2 interface layer for the teleop node
  - first place to change topic plumbing without reopening the whole node
- [feedback_handler.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/core/feedback_handler.py)
  - parses MG400 realtime feedback packet
  - queue backlog and `RunQueuedCmd` come from here
- [monitor_gui.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/monitor_gui.py)
  - telemetry consumer
  - should consume ROS-backed monitor state rather than owning callbacks
- [monitor_interfaces.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/ros/monitor_interfaces.py)
  - ROS2 interface/state layer for the monitor node
  - owns subscriptions, control publishers, and telemetry callbacks
- [execution_metrics.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/execution_metrics.py)
  - monitor-domain motion timing state
  - keeps execution timing logic out of the GUI class
- [joint_graph_buffer.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/joint_graph_buffer.py)
  - monitor-domain rolling graph buffer
  - keeps graph sample history and axis math out of the GUI class
- [presentation_state.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/presentation_state.py)
  - monitor-domain presentation mapping for status, Cartesian, and joint rows
  - keeps display formatting and threshold coloring out of the GUI class
- [control_panel_state.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/control_panel_state.py)
  - monitor-domain digital-output control state for suction/lights
  - keeps DO bit decode, lockout windows, and pending button state out of the GUI class
- [monitor_logging.py](/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller/common/utils/monitor_logging.py)
  - owns reusable monitor session/manual CSV logging
  - keeps file I/O and logging state out of the Tk GUI class

## Runtime Boundaries

The current structure is meant to stay reusable:

- `common/core/`
  - sockets, feedback parsing, robot I/O
- `common/logic/`
  - decisions and transforms that should be testable without ROS
- `common/ros/`
  - ROS2 node interface layer
  - publishers, subscriptions, topic-facing telemetry state
- `common/monitor/`
  - monitor-domain state for visualization and runtime metrics
  - graph buffers, execution timing, and other non-ROS monitor behavior
- `common/utils/`
  - runtime helpers that are not themselves the ROS boundary

Recent reusable components:

- `TargetLatencyCompensator`
- `AsyncEventLogger`
- `select_control_mode()`
- `common.ros.teleop_interfaces`
- `common.ros.monitor_interfaces`
- `common.monitor.execution_metrics`
- `common.monitor.joint_graph_buffer`
- `common.monitor.presentation_state`
- `common.monitor.control_panel_state`
- `monitor_logging`

If a new behavior can be tested without ROS, it should usually live in
`common/logic/` or a small utility rather than inside `vr_teleop_node.py`.
If the behavior is specifically about ROS topics, callbacks, or message-facing
state, it should usually live in `common/ros/`.

## Timing Rules

Two time domains are used on purpose:

- `time.perf_counter()`
  - control-loop timing
  - velocity estimation
  - queue-busy hold duration
  - stuck detection
- `time.time()`
  - analytics and wall-clock logging
  - Unity/ROS timestamp comparison

Do not mix them in the control decision path.

Important contract:

- `TeleopController.mark_command_sent(...)` should be called only after a
  motion command is actually accepted/sent successfully.
- `last_sent_time` inside `TeleopController` is monotonic loop time.

## Realtime Teleop Logic

Current behavior:

- keep only the newest Unity/VR target on the host side
- send only when the robot is physically near the previous accepted target
- compute the send window from dynamic proximity:
  `base + robot_velocity * lookahead`
- suppress sends while the filtered Unity/VR target velocity is above
  `LIVE_TARGET_FAST_VELOCITY_RAD_S`; this prevents fast sweep-through samples
  from entering the MG400 FIFO just before the operator stops
- commit `last_sent_target` only after the motion socket accepts the command
  through `mark_command_sent(...)`
- keep the existing stuck-recovery path for the case where the robot stops far
  away from the accepted target
- parse queue-state feedback for diagnostics, but do not use a host-side queue
  depth estimate as the realtime send gate

Why this matters:

- MG400 follows queued motion commands serially
- overfeeding the queue makes the robot chase old targets instead of the latest
  hand target
- a hard `queue busy -> block` gate feels too loyal to the previous target
- a fixed time-based send floor reintroduces queue buildup
- the short-pipeline estimator also failed in practice because it could refill
  at the wrong time and still grow stale queue
- the current path is therefore a 2026-02-24-style dynamic proximity gate with
  the safer 2026-04 state-commit fix

## Mock Versus Real Robot

Use caution when judging queue logic with upstream `MG400_Mock`.

Known limitation:

- upstream mock does not faithfully populate `RunQueuedCmd`
- upstream mock does not model all `JointMovJ(..., CP=...)` behavior the same
  way as the real robot

Implication:

- mock can validate basic send flow
- mock cannot fully validate the real queue-aware gate

## Safe Refactor Targets

These are reasonable next refactor steps:

- extract monitor graph buffering/update math from `monitor_gui.py`
- add more small unit-tested logic classes before touching executor/threading
- keep `motion_config.py` as the shared topic/threshold surface
- keep `common/ros/teleop_interfaces.py` as the shared teleop ROS surface
  instead of reintroducing topic setup noise into `vr_teleop_node.py`
- keep `common/ros/monitor_interfaces.py` as the shared monitor ROS surface
  instead of reintroducing callbacks/state collectors into `monitor_gui.py`
- keep `common/monitor/` as the shared monitor-domain layer instead of
  reintroducing graph/time-series state into `monitor_gui.py`
- keep `common/monitor/presentation_state.py` as the shared status/cartesian/joint
  presentation layer instead of rebuilding formatting inside `update_gui()`
- keep `common/monitor/control_panel_state.py` as the shared digital-output
  control layer instead of rebuilding DO sync/lockout logic inside the GUI
- keep `monitor_logging.py` as the shared monitor logging surface instead of
  reintroducing file-writing logic into the GUI update loop

These are higher-risk areas:

- `TrajectoryRecorder`
- thread/executor shutdown flow
- motion socket sync behavior
- changing the semantics of `last_sent_target` or `last_sent_time`

Recent progress on `TrajectoryRecorder`:

- playback start alignment now goes through a dedicated go-to-start step before
  queued playback begins
- arrival waiting and playback-complete checks are split into small helper
  methods instead of living only inside the main playback loop
- `traj_dir`, `time_fn`, and `sleep_fn` can now be injected for testing and
  reuse in non-default runtime environments
- the playback boundary is covered by `test_trajectory_recorder.py`

## Validation Checklist

Fast checks after logic changes:

```bash
PYTHONPATH=src/dobot_mg400/mg400_controller python3 -m unittest \
  src/dobot_mg400/mg400_controller/test/test_trajectory_recorder.py \
  src/dobot_mg400/mg400_controller/test/test_monitor_control_panel.py \
  src/dobot_mg400/mg400_controller/test/test_monitor_presentation_state.py \
  src/dobot_mg400/mg400_controller/test/test_monitor_domain_state.py \
  src/dobot_mg400/mg400_controller/test/test_monitor_logging.py \
  src/dobot_mg400/mg400_controller/test/test_monitor_ros_wiring.py \
  src/dobot_mg400/mg400_controller/test/test_teleop_ros_wiring.py \
  src/dobot_mg400/mg400_controller/test/test_queue_aware_logic.py \
  src/dobot_mg400/mg400_controller/test/test_async_event_logger.py \
  src/dobot_mg400/mg400_controller/test/test_target_compensator.py

python3 -m py_compile \
  src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py \
  src/dobot_mg400/mg400_controller/mg400_controller/monitor_gui.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/trajectory/trajectory_recorder.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/config/motion_config.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/logic/teleop_controller.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/logic/target_compensator.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/ros/teleop_interfaces.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/ros/monitor_interfaces.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/control_panel_state.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/execution_metrics.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/joint_graph_buffer.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/monitor/presentation_state.py \
  src/dobot_mg400/mg400_controller/mg400_controller/common/utils/monitor_logging.py

git diff --check -- \
  src/dobot_mg400/mg400_controller \
  docs/controller_continuation_guide.md \
  docs/teleop_command_logic.md
```

Environment note:

- `colcon` and `ros2` were not available in the current local shell during this
  refactor session, so ROS build validation still needs to be run in a proper
  ROS environment.

## Reading Order For A New Contributor

If someone new joins the project, this order is the fastest path:

1. `README.md`
2. `docs/setup_guide.md`
3. `docs/teleop_command_logic.md`
4. `docs/controller_continuation_guide.md`

That sequence gives:

- project overview
- how to run it
- how command logic works
- where maintainable code now lives
