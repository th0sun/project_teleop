# Production And Experimental Teleop Modes

This document separates the final project logic from earlier experimental
strategies so the codebase is easier to submit, explain, and maintain.

## Final Production Mode

The final mode for the submitted project is:

```text
CONTROL_MODE = jointmovj
runtime logic = default queue-aware production path
```

Runtime selection in `vr_teleop_node.py` now keeps teleoperation on the default
queue-aware production path. The older experimental modes are not offered in
the startup menu.

## Production Data Flow

```text
Unity-like GUI / VR
    -> /unity/joint_cmd (sensor_msgs/JointState, radians)
    -> vr_teleop_node.py
    -> JointValidator
    -> latency compensation
    -> latest_target overwrite
    -> FeedbackHandler queue state
    -> TeleopController queue-aware gate
    -> MotionPlanner command formatting
    -> TCP 30003 JointMovJ(...)
    -> MG400 controller motion queue
```

The important design choice is that the system keeps only the newest useful
target from the operator and avoids filling the robot's internal motion queue
with old intermediate targets.

## Short-Pipeline Queue Logic

The production controller uses robot feedback to avoid sending too many queued
commands while still giving `CP` a next command to blend into.

```text
queue_backlog = max(abs(QTarget[:4] - QActual[:4]))
```

The runtime keeps a tiny rolling horizon:

```text
current command + next queued tail target
```

Unity/VR may publish targets much faster than the robot can execute them.  ROS
therefore keeps only the latest target on the host side and sends it only when
the short pipeline has capacity or the previous tail target is nearly consumed.

Default parameters:

```text
REALTIME_PIPELINE_TARGET = 2
REALTIME_PIPELINE_MAX = 2
REALTIME_TAIL_CHANGE_RAD = 0.01
```

To avoid waiting forever when the robot is stuck or feedback state remains busy,
the controller still has an escape hatch:

```text
QUEUE_BUSY_ESCAPE_SEC = 0.30
```

If the robot stays below `STUCK_VELOCITY_THRESHOLD` longer than this while the
queue remains busy, the controller allows normal stuck recovery to run.

## Why This Became The Final Mode

The Dobot MG400 TCP/IP 4-axis guide describes motion commands on port `30003`
as queued commands. They are accepted into the robot controller and executed by
the algorithm queue instead of replacing the current motion instantly.

Because of that robot limitation, the best production strategy is not "send as
fast as possible." Sending too fast makes the robot follow old hand positions.

The chosen default strategy therefore focuses on:

- keeping the queue shallow
- keeping enough queued work for `CP` to blend
- overwriting intermediate hand targets with the newest target
- sending only when the short pipeline has capacity
- avoiding low-speed batch interpolation in production mode
- using CP as a smoothness helper, not as the main control mechanism

## Lesson Learned: Do Not Reintroduce Time-Based Sends

Earlier experiments added a fixed `RateFloor` to force command sends on a
timer.  That felt responsive in some cases but violated the production rule:
the robot should not receive old hand samples just because a clock tick fired.
For MG400 TCP, every extra motion command enters a FIFO controller queue.  A
time floor can therefore make the robot chase stale targets.

The final realtime rule is:

```text
send as little as possible, but keep a 2-command pipeline alive for CP
```

This replaced the older drain-gate-only behavior because a hard
`queue busy -> block` rule made the robot too loyal to its previous target.
Short-pipeline coalescing is the middle ground: it feeds just enough future
motion for blending while still discarding intermediate Unity samples before
they become robot queue entries.

## Archived Experimental Modes

The following modes are kept outside the production source tree as research
history/reference:

```text
M8_RawData
M11_Stable
M14_Smooth
M15_Sharp
M16_AdaptCP
M17_CleanFF
```

They live in:

```text
docs/report_materials/archive/experimental_logic_reference.py
```

They are not the submitted production path because they bypass the normal
production controller stack:

```text
ExperimentalStrategy
    -> raw target
    -> experimental formatter
    -> TCP 30003
```

This bypasses:

- `TeleopController`
- queue-aware default gating
- `MotionPlanner`
- the final production stuck-recovery behavior

## How To Discuss These Modes In The Report

Recommended wording:

```text
During development, several experimental motion strategies were implemented to
compare responsiveness, smoothness, and command timing. After reviewing the
Dobot MG400 queued-command behavior, the final submitted system uses the
queue-aware default mode as the production control logic. The experimental
modes are retained in the repository as development history and comparison
material, but are not used by the final runtime.
```

Use the experimental modes as evidence of iteration, not as the final design.

## Current Validation Status

Validated locally:

- unit-level queue-aware controller tests
- feedback packet parsing tests for `QTarget`, `QActual`, `RunQueuedCmd`, and
  `CurrentCommandId`
- timing regression test for monotonic controller time
- MG400_Mock socket-level benchmark smoke test

Known limitation:

- upstream `MG400_Mock` does not populate `RunQueuedCmd`
- upstream `MG400_Mock` does not implement `CP` options for `JointMovJ`

Therefore, the mock can validate integration paths, but the final queue-aware
behavior still needs real MG400 validation when hardware is available.
