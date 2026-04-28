# Project Objective

This file is the working compass for this repository.

It exists to keep maintainers, teammates, and AI agents aligned on the real
goal of the project so the implementation does not drift toward the wrong
optimization target.

## Primary Objective

The primary objective of this project is:

```text
to transform user hand motion from VR/Unity into a reusable robot program that
can be taught, adapted, and replayed on real robots.
```

In plain language:

- the user demonstrates a task with hand motion;
- the system interprets that demonstration as a program or task description;
- the program can then be adapted to the target robot's capabilities;
- the robot executes the taught task as well as that robot allows.

This means the core of the project is **robot teaching**, not raw online
teleoperation by itself.

## Correct Framing

The right framing for the system is:

```text
VR teaching input
    -> robot-neutral task/program representation
    -> robot capability profile
    -> robot-specific translation/execution
```

The project should move toward a system that can support more than one robot by
asking what each robot needs in order to interpret and execute the taught task.

## Core Engineering Challenge

The real engineering challenge is not just making one robot move.

It is:

```text
designing a middle layer that can connect VR-based teaching to different robot
families with different capabilities, limitations, and execution models.
```

This is the hardest and most important part of the project because different
robots may differ in:

- axis count and kinematic structure
- joint ordering and limits
- command vocabulary
- queue semantics
- motion blending behavior
- IO and tool control model
- timing guarantees
- whether they support offline programs, supervised playback, streaming, or a
  mix of these

So the long-term value of the system comes from how well it separates:

- what the user is trying to teach
- what the target robot is capable of doing
- how the taught program must be translated for that robot

MG400 should therefore be treated as:

- the first serious robot adapter
- a case study in queue-aware execution
- not the final definition of the whole system

Examples of robot-specific requirements:

- number of joints / active axes
- joint order and limits
- supported motion types
- queue behavior
- streaming ability
- blending / CP support
- Cartesian vs joint-space motion support
- gripper / IO / tool configuration
- tolerances, speed, acceleration, timing constraints
- offline upload format vs supervised playback vs near-real-time execution

## What Real-Time Means In This Project

Real-time or near-real-time control is **not** the main objective.

It is better described as:

```text
system extension / advanced mode
```

Why:

- some robots, especially MG400, execute commands through a queue and do not
  truly follow the newest target instantly;
- optimizing the whole architecture around immediate streaming can distort the
  main teaching objective;
- for many robots, offline playback or supervised replay is a better fit than
  aggressive live streaming.

So:

- **offline-first teaching** is the core direction;
- **robot-specific execution strategy** is the correct abstraction;
- **near-real-time behavior** is optional and depends on the target robot.

## The MG400 as the Validation Target

The MG400 is not the end goal; it is the first real-world validation of the
middle layer.

Building for the MG400 first provides concrete constraints and prevents the
middle layer from being overly abstract. The system must successfully translate
VR input into a form that the MG400 can execute well, without hard-coupling
the entire project to the MG400.

**Recent Milestone:** The system now has an experimental `SegmentClassifier`
that can translate recorded VR frames into sparse high-level geometric
primitives (`MovL`, `Arc`, fallback `JointMovJ`).  This is an important step
toward interpreting a teach session as a reusable program, but it is not yet
proof of high-fidelity repeat on dense tasks.  Local Mock testing shows a small
trajectory can pass geometrically, while a longer Unity capture still times out
when executed as host-streamed TCP commands.  The project direction therefore
remains offline/controller-side program execution where possible, with
host-streaming used only as a guarded fallback.

## Architectural Direction

The repository should evolve toward these layers:

1. `Teaching Input Layer`
   - Unity / VR hand motion
   - user gestures
   - task capture

2. `Canonical Program Layer`
   - waypoints
   - actions
   - timing
   - tool state
   - constraints

3. `Robot Capability Profile`
   - user-provided or configured robot requirements
   - what the robot can and cannot do

4. `Robot Adapter / Translator`
   - converts canonical program data into robot-specific commands
   - MG400 is one adapter, not the whole system definition

5. `Execution Layer`
   - offline export
   - supervised playback
   - queue-aware execution
   - near-real-time mode where supported

## Design Guardrails

When making decisions in this repo, prefer the choice that:

- improves task teaching and replay;
- keeps the representation reusable across robots;
- separates robot-neutral logic from robot-specific execution;
- respects the physical constraints of the real robot;
- makes the system easier to extend to other robots later.

Be careful of changes that:

- optimize only for MG400 live streaming;
- hardcode assumptions that only fit one robot;
- bury task/program meaning inside ROS node glue;
- treat near-real-time control as the main definition of success.

When in doubt, prefer the decision that makes it easier to add a different
robot later.

## Short Reminder

If you feel the project drifting, come back to this sentence:

```text
This project is for teaching robots to perform tasks from VR hand motion, with
execution style chosen according to each robot's capabilities.
```

## For Future Contributors And AI Agents

Before refactoring or adding features, ask:

1. Does this help the system represent and replay taught tasks better?
2. Is this robot-neutral logic or robot-specific behavior?
3. Am I improving the teaching pipeline, or only chasing lower latency?
4. Would this still make sense if the target robot changed?

If the answer is mostly "this only helps MG400 feel more real-time right now,"
it probably belongs in an execution-specific extension, not in the center of
the architecture.
