# Report Writing Guideline

This guide is written for turning the project into a final report/thesis
chapter structure.

## Suggested Report Title

```text
Development of a ROS2-Based VR Teleoperation System for Dobot MG400
with Queue-Aware Motion Command Control
```

## Main Message

The project is not only a robot-control demo. The engineering contribution is a
teleoperation pipeline that handles the Dobot MG400's queued-command limitation.

The final system receives hand/VR target positions, keeps the newest useful
target, and sends commands only when the robot queue is ready enough. This
reduces the chance that the robot follows old operator positions.

## Recommended Chapter Structure

### Chapter 1: Introduction

Explain:

- teleoperation lets a user control a robot from another interface
- VR/GUI control feels direct to humans but the robot controller has queue and
  motion-planning limitations
- the Dobot MG400 does not instantly replace queued motion commands
- the project goal is to make MG400 follow the latest operator target as much
  as practical within those constraints

Example research problem:

```text
How can a ROS2 teleoperation system send target positions to the Dobot MG400
without overfilling the robot's queued motion command buffer?
```

### Chapter 2: Related Work And Background

Cover:

- ROS2 node/topic architecture
- `sensor_msgs/JointState`
- Dobot MG400 TCP/IP command ports
- queued motion commands on port `30003`
- Unity/ROS communication concept
- mock/simulation role in development

Important Dobot point:

```text
Motion commands are queued by the robot controller. Therefore, sending commands
too frequently can increase delay because the robot executes older targets
before newer targets.
```

### Chapter 3: System Architecture

Use this high-level diagram:

```text
Unity-like GUI / VR input
        |
        v
/unity/joint_cmd
        |
        v
ROS2 Teleop Node
        |
        +-- Joint validation and joint limit clamp
        |
        +-- Latency compensation
        |
        +-- Queue-aware command gate
        |
        +-- Motion command formatter
        |
        v
TCP/IP port 30003
        |
        v
Dobot MG400 controller queue
        |
        v
Robot motion
```

Mention the main folders:

- `src/dobot_mg400/mg400_controller`: ROS2 teleop control logic
- `src/dobot_mg400/mg400_description`: robot model and meshes
- `tools/mg400_simulator`: Unity-like Python GUI simulator
- `MG400_Mock`: Docker-based external mock server
- `Dobot_TCP_IP_Python_V4`: Dobot reference TCP/IP files and 4-axis guide

### Chapter 4: Teleoperation Command Logic

Focus only on the final production mode:

```text
CONTROL_MODE = jointmovj
runtime logic = default queue-aware production path
```

Explain the default path:

```text
latest target from GUI/VR
    -> validate and clamp
    -> compensate communication latency
    -> store as latest_target
    -> read robot feedback
    -> check queue backlog
    -> decide whether to send command
    -> send JointMovJ command to port 30003
```

Do not present `M11`, `M14`, `M15`, `M8_raw`, `M16`, or `M17` as final modes.
They are archived development experiments.

### Chapter 5: Queue-Aware Control Design

Explain the problem:

```text
If every input target is sent to the robot, the MG400 may execute a long queue
of old positions. This makes the robot lag behind the user's latest hand pose.
```

Explain the solution:

```text
The controller stores only the latest target and sends a new command when the
robot feedback indicates the queue is shallow enough.
```

Main feedback values:

- `QActual`: actual joint position
- `QTarget`: robot controller target joint position
- `RunQueuedCmd`: whether queued command execution is running

Main backlog metric:

```text
queue_backlog = max(abs(QTarget[:4] - QActual[:4]))
```

Gate condition:

```text
if RunQueuedCmd == 1 and queue_backlog > QUEUE_BACKLOG_GATE_RAD:
    wait
else:
    allow command decision
```

Safety escape:

```text
if queue remains busy but robot velocity is near zero for more than
QUEUE_BUSY_ESCAPE_SEC:
    allow stuck recovery to run
```

### Chapter 6: Implementation Details

Key files to cite:

- `vr_teleop_node.py`: ROS2 node, callback, control loop, command send path
- `feedback_handler.py`: binary feedback parsing from port `30004`
- `teleop_controller.py`: queue-aware decision logic
- `motion_config.py`: thresholds and control constants
- `motion_planner.py`: command string formatting
- `tools/queue_logic_mock_benchmark.py`: mock benchmark script

Important implementation decisions:

- use radians internally in ROS
- convert to degrees only when formatting Dobot TCP/IP commands
- keep `JointMovJ` as the final command mode
- update controller state only after socket send succeeds
- use monotonic loop time for velocity and stuck detection

### Chapter 7: Testing And Validation

Use these validation categories:

1. Unit tests

```text
python3 -m unittest src/dobot_mg400/mg400_controller/test/test_queue_aware_logic.py
```

Validated:

- queue backlog blocks sends
- queue-busy escape reaches stuck recovery
- queue-busy escape does not open while robot is moving
- feedback parser reads queue-related fields
- controller timing does not mix wall-clock and monotonic time

2. Syntax validation

```text
python3 -m py_compile <edited files>
```

3. Mock benchmark

Use `MG400_Mock` only as a smoke/integration test because it does not fully
simulate real queue state.

Known mock limitations:

- `RunQueuedCmd` remains `0`
- `CP` option is not implemented for `JointMovJ`

### Chapter 8: Results And Discussion

Recommended discussion:

- the final design is better aligned with MG400 queue behavior than older
  batch/micro-step approaches
- sending fewer, more useful commands is better than sending every intermediate
  hand target
- the queue-aware design improves engineering correctness even before hardware
  validation because it follows the robot controller's documented behavior
- final tuning still requires the real MG400

### Chapter 9: Limitations And Future Work

Limitations:

- real MG400 hardware validation is still required
- mock server does not expose `RunQueuedCmd` correctly
- CP behavior cannot be evaluated with the current mock
- threshold values may need tuning on the real robot

Future work:

- record real robot telemetry for queue backlog and target tracking error
- tune `QUEUE_BACKLOG_GATE_RAD`
- tune `QUEUE_BUSY_ESCAPE_SEC`
- compare `JointMovJ`, `MovJ`, and `MovL` on real hardware
- improve GUI visualization for queue backlog and command send events

## Experimental Modes Section

If the report needs to mention old modes, place them in a subsection such as:

```text
Development Experiments Not Used In Final Runtime
```

Write:

```text
Several experimental strategies were implemented during development to study
smoothness, command rate, filtering, feedforward, and CP behavior. These modes
were useful for comparison, but the final submitted system uses the default
queue-aware controller because it directly addresses the MG400 queued-command
limitation.
```

Then list:

- `M8_RawData`: raw periodic sending
- `M11_Stable`: velocity-clamped integrator
- `M14_Smooth`: adaptive rate with curvature gate
- `M15_Sharp`: stricter response to sharp motion
- `M16_AdaptCP`: adaptive CP experiment
- `M17_CleanFF`: LPF and feedforward experiment

Do not describe them as user-selectable final modes.

## Final Claim To Use Carefully

Safe wording:

```text
The final implementation is designed to reduce queued-command lag by gating
motion commands based on robot feedback. Local unit tests and mock integration
tests validate the software behavior. Full performance validation requires the
physical Dobot MG400 because the available mock does not fully implement queue
state feedback.
```

Avoid claiming:

```text
The system has been proven to improve real MG400 tracking performance.
```

That claim needs real hardware logs.
