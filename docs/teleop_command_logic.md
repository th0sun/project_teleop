# Teleop Command Logic Notes

This note documents how Unity or the Unity-like simulator sends joint targets
to the MG400 controller, how the project filters those targets, and where the
final TCP command is sent to the robot.

## Scope

Main files:

- `src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/logic/joint_validator.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/logic/teleop_controller.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/logic/motion_planner.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/core/command_sender.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/core/robot_connection.py`
- `src/dobot_mg400/mg400_controller/mg400_controller/common/core/feedback_handler.py`

The normal real-time teleop path uses the real robot TCP ports:

- Dashboard: `29999`
- Motion command: `30003`
- Feedback: `30004`

Mock compatibility is not the priority for this path. The real robot API is the
target behavior.

## Data Flow

This is the default production path when logic mode is `default`.

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
latest_target
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

## Runtime Mode Selection

`vr_teleop_node.py` asks for the Dobot motion command type when the node starts.
The teleop command logic itself is fixed to the default queue-aware production
path.

### Motion Command Type

This controls the final Dobot command name:

- `1`: `jointmovj` -> `JointMovJ(...)`
- `2`: `movj` -> `MovJ(...)`
- `3`: `movl` -> `MovL(...)`

The selected value is stored in `robot_config.CONTROL_MODE`.

The production logic path uses `latest_target`, which is latency-compensated in
`_unity_callback()`, then paced by `TeleopController`.

## Git History Around 2026-02-24

Checked commit:

```text
9196b01 2026-02-24 17:43:33 +0700
feat: Implement Batch Interpolation (Streaming micro-steps) for fine-aiming stability
```

This is the end-of-day snapshot for the default teleop path on
`2026-02-24`, after dynamic proximity and batch interpolation had already been
introduced.

Development experiments are documented separately in
`docs/report_materials/production_vs_experimental_modes.md`. This project logic
note focuses on the current production path.

## Default Logic: 2026-02-24 vs Current

This section compares the default path at the end of `2026-02-24` with the
current code.

### Input Target Handling

On `2026-02-24` (`9196b01`), `_unity_callback()` already did predictive target
processing:

```text
Unity JointState.position
        |
        v
JointValidator.validate_and_clamp()
        |
        v
ClockCalibrator.calibrate()
        |
        v
TargetPredictor.update_and_predict(..., prediction_horizon_sec=0.08)
        |
        v
latest_target = predicted_q
```

Current default mode still starts with the same validator, but it no longer
uses `TargetPredictor`. The stored target is now latency-compensated:

```text
Unity JointState.position
        |
        v
JointValidator.validate_and_clamp()
        |
        v
ClockCalibrator.calibrate()
        |
        v
TargetLatencyCompensator.compensate()
        |
        v
latest_target = q_compensated_safe
```

So the main change is:

- `2026-02-24`: forward prediction through `TargetPredictor`
- current: timing-based latency compensation using EMA target velocity

Both versions aim to lead the robot forward by about `80 ms`, but they do it by
different methods.

The compensation itself now lives in
`mg400_controller.common.logic.target_compensator.TargetLatencyCompensator`.
That class has no ROS dependency, so it can be reused by tests, simulator-side
tools, or future nodes that receive the same Unity joint target stream.

### Send Decision

The default send gate on `2026-02-24` already matched the modern
dynamic-proximity shape:

```text
trigger_distance = DYNAMIC_PROXIMITY_BASE_RAD
                 + velocity_mag * DYNAMIC_PROXIMITY_LOOKAHEAD_SEC

if dist_to_last < trigger_distance
and change_in_target > SPATIAL_THRESHOLD:
    send
```

So this part is not a conceptual rewrite from `2026-02-24` to now. The logic is
the same family, but the tuning changed.

Parameter differences:

- `2026-02-24`: `DYNAMIC_PROXIMITY_BASE_RAD = 0.005` rad, about `0.3` degrees
- current: `DYNAMIC_PROXIMITY_BASE_RAD = 0.02` rad, about `1.1` degrees
- `DYNAMIC_PROXIMITY_LOOKAHEAD_SEC = 0.25` seconds.
- `PROXIMITY_THRESHOLD = 0.08` rad still exists, but default send gating no
  longer uses it for normal proximity; it is still used inside stuck logic.

Stuck recovery thresholds also changed:

- `2026-02-24`: `STUCK_TIME_THRESHOLD = 0.3`
- current: `STUCK_TIME_THRESHOLD = 0.15`
- `2026-02-24`: `TARGET_CHANGE_THRESHOLD = 0.005`
- current: `TARGET_CHANGE_THRESHOLD = 0.02`

The stuck resend condition itself is effectively the same shape in both
versions:

```text
if robot is slow and far from last target
and (
    change_in_target > TARGET_CHANGE_THRESHOLD
    or error_to_last_target > PROXIMITY_THRESHOLD
):
    send
```

### Command Formatting

Both versions can still emit single commands through
`MotionPlanner.format_command()`:

```text
JointMovJ(...,SpeedJ=100,AccJ=100,CP=...)
MovJ(...,SpeedJ=100,AccJ=100,CP=...)
MovL(...,SpeedJ=100,AccJ=100,CP=...)
```

The speed remains effectively fixed at 100 percent in default mode.

The CP value changed:

- `2026-02-24`: `CP_VALUE = 80`
- Current: `CP_VALUE = 100`

Important difference in default path behavior:

- `2026-02-24`: `TeleopController.format_command_string()` used
  `planner.plan_batch_motion(..., num_steps=3)` when robot velocity was below
  `0.1 rad/s`
- current: default mode no longer uses batch interpolation; it always sends a
  single formatted command

`MotionPlanner.plan_batch_motion()` still exists in the current file, but the
current default path does not call it.

### Practical Meaning

Compared with the end of `2026-02-24`, the current default mode changed in
these practical ways:

- it uses latency compensation instead of predictor-based forward projection;
- the dynamic proximity base window is wider, so it is easier to trigger a new
  send even at lower robot speed;
- stuck detection reacts sooner in time (`0.15s` vs `0.3s`) but requires a
  larger target change threshold (`0.02` vs `0.005`) unless the robot is still
  far from the last target;
- it no longer streams 3 micro-steps during low-speed fine aiming;
- uses CP 100 for maximum blending.

Risk to check next: the current default mode may feel smoother and less stop-go
because of wider dynamic proximity plus CP 100, but it may also behave
differently in fine aiming because the old low-speed batch interpolation path is
gone.

## Queue Semantics From 4-Axis Guide

The `4axis` TCP/IP guide is the authoritative source for MG400 behavior.

Important queue facts from the guide:

- motion commands on port `30003` are queued commands;
- queued commands return immediately after delivery and enter the background
  algorithm queue before execution;
- immediate commands can run before queued motion is completed;
- `Sync()` blocks until previously queued commands are executed;
- `CP` controls blending between the current motion command and the next motion
  command.

Practical consequence for teleoperation:

- MG400 does not "jump" directly to the newest hand target;
- if too many motion commands are delivered, the robot follows old queued
  targets first;
- for "follow the latest hand value", the main objective is not maximum send
  rate. The main objective is keeping queue depth shallow.

## Default Path Audit For Queue Growth

Current default path:

```text
Unity target
        |
        v
latest_target overwritten to newest value
        |
        v
TeleopController.should_send_command()
        |
        v
MotionPlanner.format_command()
        |
        v
send on port 30003
```

Good parts for latest-target behavior:

- `_unity_callback()` overwrites `latest_target` instead of storing every
  intermediate hand sample;
- default mode sends a single motion command at a time, not 3 micro-steps;
- feedback thread discards stale feedback packets and keeps only the latest
  1440-byte packet.

Current risks that can still grow queue unnecessarily:

### 1. Send gating does not use queue-state feedback

The project receives feedback fields corresponding to queue state and target
state, such as:

- `RunQueuedCmd`
- `QTarget`
- `QActual`
- `CurrentCommandId`

But current default logic only gates on:

- `q_current` vs `last_sent_target`
- `latest_target` vs `last_sent_target`
- estimated robot velocity

It does not check whether the robot still has queued motion pending.

That means a new command can be sent while the robot is still effectively
chasing previously queued motion, as long as the local proximity heuristic says
"close enough".

### 2. Dynamic proximity is based on actual position, not queue target

Current gate:

```text
dist_to_last = max(abs(q_current - last_sent_target))
trigger = base + velocity * lookahead
```

This is only an indirect estimate of queue depth.

For a queued robot, the more informative signal would be closer to:

```text
queue_backlog = max(abs(QTarget - QActual))
```

or a boolean queue-running signal such as `RunQueuedCmd`.

Without those signals, the controller may keep feeding commands based on where
the robot body is, instead of where the internal queue target already is.

### 3. Wider dynamic base makes sending easier

Current:

- `DYNAMIC_PROXIMITY_BASE_RAD = 0.02` rad, about `1.1` degrees

On `2026-02-24`:

- `DYNAMIC_PROXIMITY_BASE_RAD = 0.005` rad, about `0.3` degrees

A wider base is good for smoothness, but it also lowers the barrier for adding
the next queued command before the previous one is fully "drained".

### 4. CP=100 increases blending and can hide queue buildup

`CP=100` is good for continuous motion, but it also means the robot will
transition aggressively between queued targets.

That helps smoothness, but it can make it less obvious from `q_current` alone
how many future targets are already waiting.

### 5. Send-state is updated before send success is confirmed

`TeleopController.should_send_command()` updates `last_sent_target` and
`last_sent_time` before the code actually confirms `sender.send(...)` success.

If motion send fails, local controller state can temporarily believe a command
was accepted when the robot never received it.

This is not only a reliability issue; it also weakens queue reasoning because
local "last sent" state can drift away from real robot queue state.

## Why The 6-Axis API File Is Not The Main Constraint

`Dobot_TCP_IP_Python_V4/dobot_api.py` is a broader shared API layer and uses
6-element feedback arrays in its binary packet struct.

That does not automatically mean the project is behaving like a 6-axis robot.

For MG400 in this project:

- the command syntax used by default mode is 4-axis syntax such as
  `JointMovJ(J1,J2,J3,J4,...)`;
- the controller actively slices and uses only the first 4 joints from
  feedback;
- the 4-axis guide is the correct behavioral reference for queue semantics and
  motion command meaning.

So the real issue is not "the wrapper is 6-axis". The real issue is that the
project currently leaves useful queue-state feedback unused, even though the
packet struct already contains fields that could help.

## Best Direction If The Goal Is "Follow Latest Hand Value"

For MG400, the best default strategy is likely:

1. Keep single-command sending in default mode.
2. Keep overwriting `latest_target` with the newest hand target.
3. Add queue-aware gating using feedback such as `RunQueuedCmd`, `QTarget`,
   `QActual`, or `CurrentCommandId`.
4. Only send a new command when queue backlog is sufficiently shallow.
5. Keep `CP` high enough for blending, but do not use batch interpolation in the
   default realtime path.

In other words:

- do not try to make the robot follow the latest hand value by pushing more
  commands;
- make it follow the latest hand value by reducing stale queued commands.

## Design Options For Default Mode

Below are practical design options for improving the default mode under the
MG400 queue constraint.

### Option A: Tune Current Heuristic Only

Keep the current structure:

- `latest_target` overwrite
- dynamic proximity
- stuck recovery
- single-command send

Only tune parameters such as:

- `DYNAMIC_PROXIMITY_BASE_RAD`
- `DYNAMIC_PROXIMITY_LOOKAHEAD_SEC`
- `STUCK_TIME_THRESHOLD`
- `TARGET_CHANGE_THRESHOLD`
- `CP_VALUE`

Pros:

- smallest code change
- lowest regression risk
- easy to test quickly on the real robot
- no protocol or feedback-struct change needed

Cons:

- still blind to real queue depth
- can only indirectly reduce backlog
- behavior may depend heavily on hand-tuned thresholds

Weak points:

- if the robot queue behaves differently from the local heuristic, the logic can
  still send too early or too late
- tuning may become fragile across different speeds or paths

Best use:

- quick stabilization pass
- short-term fallback if we want the safest minimal change

### Option B: Queue-Aware Gate Using Feedback Backlog

Keep the current default flow, but add explicit queue-state awareness from
feedback:

- parse `RunQueuedCmd`
- parse `QTarget`
- compute `queue_backlog_rad = max(abs(QTarget[:4] - QActual[:4]))`

Then gate new sends like this:

- if queue backlog is above threshold, do not send
- when backlog becomes shallow enough, send the newest `latest_target`

Pros:

- directly matches the real robot constraint from the 4-axis guide
- best fit for "follow latest target" without adding aggressive motion hacks
- preserves the current overwrite-to-latest behavior
- avoids filling queue with stale intermediate targets

Cons:

- requires changing `FeedbackHandler` and `TeleopController` interfaces
- needs real-robot validation because mock behavior may not match
- requires threshold tuning for what counts as "shallow enough"

Weak points:

- if `QTarget` semantics are noisy or laggy in practice, gating may become too
  conservative
- if threshold is too strict, motion can feel sticky
- if threshold is too loose, queue growth problem returns

Best use:

- strongest candidate for new default mode
- best balance between correctness and maintainability

### Option C: Queue-Aware Gate Plus Command-ID Tracking

Extend Option B by also using:

- `CurrentCommandId`
- optionally dashboard `GetCurrentCommandID()`

Idea:

- only send again when command progress has advanced, or when queue backlog is
  shallow and command ID is moving as expected

Pros:

- gives stronger visibility into execution progress
- can help distinguish "robot still running queued command" from "robot is
  stuck"
- useful for debugging and telemetry

Cons:

- more complexity than Option B
- command ID is harder to reason about than plain backlog distance
- can become brittle if connection timing and feedback timing do not line up

Weak points:

- risks overengineering the first pass
- may add a lot of state handling without giving much more benefit than backlog
  gating alone

Best use:

- second-stage refinement after Option B
- diagnostics and logging even if not used as the main gate

### Option D: Stop-And-Replace Strategy

When the hand target changes a lot:

- call `Stop()`
- clear the local send state
- send the newest target immediately

Pros:

- strongest possible "forget stale queue" behavior
- easy to understand conceptually

Cons:

- likely to feel jerky
- can break motion smoothness badly
- higher operational risk on real hardware
- interacts poorly with CP blending

Weak points:

- may produce visible stops, snaps, or repeated braking
- can be worse than queue lag for teleoperation feel

Best use:

- emergency recovery behavior only
- not recommended for the default realtime path

### Option E: Reintroduce Low-Speed Micro-Step Batching

Bring back the `2026-02-24` style idea:

- when robot velocity is low, send `plan_batch_motion(..., num_steps=3)`

Pros:

- may improve fine aiming smoothness
- can make tiny corrections look cleaner in some cases

Cons:

- directly increases queued motion depth
- works against the "follow latest target" goal
- makes the robot more likely to chase stale targets

Weak points:

- solves a different problem than the one we care about now
- good for precision feel, bad for latest-target responsiveness

Best use:

- optional precision mode
- not recommended for the main default teleop mode

## Recommended Path

Recommended implementation order:

1. Start with Option B.
2. Add lightweight logging from Option C for visibility.
3. Use a small part of Option A only for final threshold tuning.
4. Keep Option D only as a possible recovery tool, not normal flow.
5. Do not use Option E in default mode.

Why this is the best balance:

- it respects the real MG400 queue behavior from the 4-axis guide
- it keeps the current "latest target overwrite" model
- it attacks the actual cause of lag, which is stale queued motion
- it stays simpler and safer than building a full command-ID state machine first

If we want one sentence:

- the best default design is "latest-target overwrite plus queue-backlog gate"
  rather than "send faster" or "send more micro-steps"

## Implemented On 2026-04-24

The current branch now implements the first pass of Option B:

- `FeedbackHandler` parses `QTarget`, `RunQueuedCmd`, and exposes
  `get_target_position()`, `get_queue_backlog()`, and `get_run_queued_cmd()`.
- default mode now passes queue-state feedback into
  `TeleopController.should_send_command(...)`.
- default mode blocks new sends with reason
  `QueueBusy_Backlog...` when:

```text
RunQueuedCmd == 1
and
max(abs(QTarget[:4] - QActual[:4])) > QUEUE_BACKLOG_GATE_RAD
```

- `QUEUE_BACKLOG_GATE_RAD` is currently `0.01` rad, about `0.57` degrees.
- `TeleopController` no longer mutates `last_sent_target` inside
  `should_send_command()`. Send-state is now committed only after
  `sender.send(...)` succeeds, through `mark_command_sent(...)`.
- runtime mode selection was moved to
  `mg400_controller.common.utils.mode_selection.select_control_mode()`.
- background CSV/event logging was moved to
  `mg400_controller.common.utils.async_event_logger.AsyncEventLogger()`.
- Unity latency compensation was moved to
  `mg400_controller.common.logic.target_compensator.TargetLatencyCompensator()`.
- ROS publisher/subscriber setup was moved to the ROS interface layer:
  `mg400_controller.common.ros.teleop_interfaces`.

Local validation completed:

- `python3 -m py_compile` on the edited runtime files
- `python3 -m unittest src/dobot_mg400/mg400_controller/test/test_queue_aware_logic.py`

Current limitation of this first pass:

- the queue gate is binary and conservative;
- it uses a single backlog threshold, not hysteresis;
- it does not yet combine backlog with command-ID progress or robot mode beyond
  the existing logic.

## Unity Callback

`TeleopNode._unity_callback()` receives `/unity/joint_cmd`.

Current behavior:

1. Ignore input when robot is disconnected or `msg.position` has fewer than 4 joints.
2. Convert `msg.position` to `q_target`.
3. Reject the command if any joint is `NaN`.
4. Run `JointValidator.validate_and_clamp(q_target)`.
5. Read Unity timestamp from `msg.header.stamp`.
6. Use `ClockCalibrator` to correct Unity time against ROS host time.
7. Estimate target velocity with EMA from recent Unity targets.
8. Compensate measured network latency by shifting the target forward:
   `q_compensated = q_safe + ema_target_vel * latency_comp_sec`
9. Cap latency compensation at 80 ms.
10. Re-run joint validation on the compensated target.
11. Store:
   - `latest_target = q_compensated_safe`
   - `target_recv_time`
   - `unity_send_time`

Important: the Unity callback does not send commands directly. It only updates
the latest target. Actual sending happens in the control loop.

## Validation

`JointValidator.validate_and_clamp()` is the first safety filter.

It checks:

- Input length must include at least 4 joints.
- NaN values are rejected.
- Each joint is clamped to `JOINT_LIMITS`.
- J3 relative to J2 is constrained by `ELBOW_ANGLE_LIMIT`.
- A small formatting margin is applied so the final command string rounded to
  4 decimal degrees does not land exactly on a limit.

The validator works in radians internally but compares limits configured in
degrees.

## Control Loop

`TeleopNode._high_precision_control_loop()` runs in a dedicated thread at 200 Hz.
Each iteration calls `_control_loop_step()`.

Before sending motion, `_control_loop_step()` also:

- Reads current robot position from `FeedbackHandler.get_current_position()`.
- Blocks teleop motion during Teach & Repeat playback or post-stop homing.
- Records targets when Teach & Repeat recording is active.
- Publishes tool vectors, flange FK, DO status, robot mode, and error status.
- Periodically sends `GetTool()` through dashboard TCP.
- Auto-clears robot mode 9 errors every 3 seconds.
- Updates latency tracking metrics.

If Teach & Repeat is playing, normal teleop commands are skipped and the
sequencer owns motion commands.

## Send Decision

Default production logic uses `TeleopController.should_send_command()`.

The decision logic has two main gates:

### 1. First Command

If no command has ever been sent, it sends immediately with reason `Init`.

### 2. Dynamic Proximity

It sends a new command when the robot is close enough to the last sent target
and the requested target has changed meaningfully.

Relevant values:

- `dist_to_last = max(abs(q_current - last_sent_target))`
- `change_in_target = max(abs(latest_target - last_sent_target))`
- `trigger_distance = DYNAMIC_PROXIMITY_BASE_RAD + velocity_mag * DYNAMIC_PROXIMITY_LOOKAHEAD_SEC`

If:

```text
dist_to_last < trigger_distance
and
change_in_target > SPATIAL_THRESHOLD
```

then it sends with reason like:

```text
DynProx_Dist0.012_Thr0.025
```

This is the main smoothing strategy: the next command is queued when the robot
is near the previous target. Faster robot velocity increases the trigger
distance, so the controller can send the next command earlier.

### 3. Stuck Recovery

At most 10 Hz, the controller checks whether the robot appears stopped while it
is still far from the last sent target.

If:

```text
velocity_mag < STUCK_VELOCITY_THRESHOLD
and
error_to_last_target > PROXIMITY_THRESHOLD
for longer than STUCK_TIME_THRESHOLD
```

and either the user's target changed enough or the robot is far enough from the
last target, it resends with reason like:

```text
Stuck_Vel0.0001_Delta0.045
```

## Command Formatting

`TeleopController.format_command_string()` re-validates and clamps the target,
then asks `MotionPlanner.format_command()` to build the robot TCP command.

Current default speed is fixed at 100 percent in `format_command_string()`.

`MotionPlanner.format_command()` converts radians to degrees and emits one of:

```text
JointMovJ(J1,J2,J3,J4,SpeedJ=100,AccJ=100,CP=100)
MovJ(J1,J2,J3,J4,SpeedJ=100,AccJ=100,CP=100)
MovL(J1,J2,J3,J4,SpeedJ=100,AccJ=100,CP=100)
```

The selected mode comes from `CONTROL_MODE`:

- `jointmovj`
- `movj`
- `movl`

`ACC_VALUE` and `CP_VALUE` come from `motion_config.py`.

## TCP Send

`CommandSender.send()` calls `RobotConnection.send_motion_cmd()`.

`RobotConnection.send_motion_cmd()`:

- Uses motion TCP socket on port `30003`.
- Appends `\n` when missing.
- Sends bytes to the robot.
- Reconnects the motion socket if an OS/socket error occurs.

Dashboard commands such as `EnableRobot()`, `GetTool()`, `ClearError()`, and
`DOExecute(...)` go through dashboard port `29999`, not the motion port.

## Feedback Loop

`FeedbackHandler` reads binary feedback packets from port `30004`.

It parses:

- current joints from offset 432
- robot mode from offset 24
- DI/DO status from offsets 8 and 16
- speed scaling from offset 64
- error and collision status
- command ID
- tool vector actual and target

It publishes `/joint_states` and stores the latest 4 active joints for the
control loop.

## Timing Fix Applied

The timing bug in the velocity/gating path has now been fixed.

Previous problem:

- `_control_loop_step()` used `time.perf_counter()` and called
  `self.controller.update_robot_state(q_current, now)`
- `TeleopController.should_send_command()` used `time.time()` and called
  `update_robot_state()` again

That mixed monotonic time with wall-clock time inside the same controller state.

Current behavior:

- `_control_loop_step()` updates robot velocity once per loop using monotonic
  `time.perf_counter()`
- `should_send_command(...)` now accepts `now` from the caller and uses that
  same monotonic timestamp for stuck-check timing
- `should_send_command(...)` no longer performs a second velocity update

Regression coverage:

- `test_queue_aware_logic.py` now includes a deterministic timestamp test that
  checks `should_send_command(...)` does not mutate `last_robot_time` or
  `robot_velocity` a second time

## Queue Gate Escape Hatch

The first queue-aware implementation blocked new commands whenever:

```text
RunQueuedCmd == 1
and max(abs(QTarget[:4] - QActual[:4])) > QUEUE_BACKLOG_GATE_RAD
```

That is the right default behavior for limiting queue depth, but it had one
dangerous edge case: if real feedback left `RunQueuedCmd` high while the robot
was stalled or paused, the controller could return `QueueBusy` forever and
never reach stuck recovery.

The controller now tracks `queue_busy_start_time` and uses:

```text
QUEUE_BUSY_ESCAPE_SEC = 0.30
```

Behavior after the change:

- queue busy and robot still moving: keep blocking new commands
- queue busy but robot velocity stays below `STUCK_VELOCITY_THRESHOLD` longer
  than `QUEUE_BUSY_ESCAPE_SEC`: allow the normal stuck-detection logic to run
- after a command is accepted by the motion socket, reset both stuck and queue
  busy timers

Regression coverage:

- queue backlog blocks sends while the queue is still active
- queue busy can escape into stuck recovery when the robot is stopped
- queue busy does not escape while the robot is still moving
- timing path still uses one monotonic clock source

## Next Work

1. Decide whether `force_send` should still exist in
   `format_command_string()`. It is currently passed by the caller but not used.
2. Decide whether speed should remain fixed at 100 percent or use
   `MotionPlanner.calculate_speed()` again.
3. Re-check experimental modes after timing cleanup because they also depend on
   high-precision loop timing.
4. When real hardware is available, validate `RunQueuedCmd`-based gating on the
   actual MG400.

## Mock Benchmark On 2026-04-24

To compare `default`-path behavior before trying the real robot, a small
benchmark harness was added at:

- `tools/queue_logic_mock_benchmark.py`

This script runs against the upstream Dockerized `MG400_Mock` submodule and
drives a shared target trajectory for three strategy families:

- `old24`: approximation of the `2026-02-24` default logic
- `oldcurrent`: approximation of the pre-change current default
- `new`: current queue-aware default logic using `RunQueuedCmd` and backlog

It also includes a mock-only `newproxy` mode that ignores `RunQueuedCmd` and
gates only on backlog. This mode does not represent the real code path; it is
only useful to estimate what the queue gate would do in a mock that does not
publish queue state correctly.

### Results

Measured over one 8-second scripted target sequence:

| Strategy | Commands Sent | Mean Latest Error (deg) | P95 Latest Error (deg) | Max Latest Error (deg) | Mean Backlog (deg) | Max Backlog (deg) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `old24` | 12 | 20.25 | 43.07 | 54.65 | 12.38 | 43.14 |
| `oldcurrent` | 4 | 18.93 | 45.10 | 52.17 | 7.35 | 53.06 |
| `new` | 9 | 18.93 | 44.17 | 51.14 | 9.40 | 53.28 |
| `newproxy` | 9 | 24.72 | 46.93 | 49.28 | 10.70 | 46.93 |

### Interpretation

- `old24` performed worst for latest-target tracking in this mock run.
- `oldcurrent` and `new` were very close on latest-target error.
- The apparent `new` result is not a trustworthy validation of the queue-aware
  gate on its own, because the upstream mock never asserts queue-running state.

### Critical Mock Limitations

The upstream mock is useful for quick socket-level checks, but it is not a
faithful validator for MG400 queue behavior:

1. `RunQueuedCmd` is not populated in feedback packets.
   - `MG400_Mock/app/src/dobot_command/dobot_hardware.py` writes many status
     fields in `__pack_status()` but does not write `RunQueuedCmd`.
   - As a result, the real queue-aware condition
     `if run_queued_cmd and backlog > gate:` never activates in the benchmark.

2. `JointMovJ` ignores command options such as `CP`.
   - `MG400_Mock/app/src/dobot_command/motion_command.py` still contains:
     `TODO: Support options, e.g., user, tool, speed_j, acc_j, cp.`
   - This means the mock cannot validate differences such as `CP=80` vs
     `CP=100`.

### Practical Conclusion

The benchmark is still useful for one thing:

- it supports the earlier reasoning that the `2026-02-24` default, with
  low-speed micro-step batching, is not a good fit when the goal is "follow the
  latest hand target as much as possible."

But it does **not** prove that the new queue-aware gate helps on the real robot.
That still requires testing against real MG400 feedback where `RunQueuedCmd`
and queue execution state are present.
