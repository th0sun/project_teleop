# Architecture Phase Research Log

This file is for report/thesis writing.

It records the current status of the architecture phase, the research path that
was followed, the major design options that were considered, the reasons behind
the current direction, and the reasons some alternatives were rejected or
deferred.

Unlike the AI handoff files, this document is written as project evidence for a
future report chapter.

## 1. Current Phase Status

The project has already completed the legacy cleanup/refactor phase of the
original MG400-first ROS2 codebase.

That closed phase covered:

- teleop runtime cleanup
- monitor/runtime cleanup
- playback helper extraction
- ROS2 boundary extraction
- submodule/manual cleanup
- supporting documentation cleanup

The current phase is different:

```text
design the next architecture step for multi-robot teaching
```

As of the current architecture phase:

- the new architecture is still at the **proposal / research-backed design**
  stage;
- the design direction is documented in
  `docs/ai_handoffs/next_phase_architecture_proposal.md`;
- research-backed corrections and audit notes are documented in
  `docs/ai_handoffs/next_phase_architecture_revision.md`;
- full implementation of the new multi-robot core has **not** been completed
  yet.

So the current status is:

```text
legacy MG400-first refactor phase = closed
multi-robot architecture phase = designed in detail, not yet fully implemented
```

## 2. Why The Project Direction Was Reframed

The project originally spent significant effort on making the MG400 follow the
latest teleoperation target as closely as possible.

That work was still useful, especially because the MG400 queue semantics are a
real engineering constraint. However, the deeper project objective became
clearer over time:

```text
the real goal is not only real-time control of one robot,
but teaching robots from VR/Unity hand motion in a way that can be adapted
across different robot families.
```

This reframing matters because:

- MG400 is queue-based and not ideal as the definition of a general real-time
  teleoperation system;
- the more valuable contribution is a system that can capture a taught task,
  represent it in a robot-neutral form, and translate it according to each
  robot's capability;
- this makes the project more aligned with long-term extensibility and more
  suitable as a serious engineering/research contribution.

The central project statement therefore became:

```text
VR teaching input
    -> robot-neutral task/program representation
    -> robot capability profile
    -> robot-specific translation/execution
```

## 3. Research Workflow Used In This Phase

The architecture phase was not built from intuition alone. The process was
deliberately staged.

### Step 1: Close the old refactor phase first

Before starting a new architecture, the existing MG400-first codebase was
cleaned up enough that future work would not be buried under unrelated
technical debt.

Reason:

- it is easier to reason about a new architecture when the old runtime has
  clearer boundaries;
- otherwise the next phase risks turning into endless MG400-specific cleanup.

### Step 2: Re-state the true project objective

The project objective was rewritten so contributors and AI agents would stop
drifting toward "make MG400 more real-time" as the only success signal.

Reason:

- without this correction, design choices would continue to overfit the first
  robot case;
- the final report needs a correct statement of the engineering problem.

### Step 3: Define the architecture questions explicitly

The following questions were made explicit before proposing a design:

1. What is the canonical taught-program representation?
2. What minimum robot capability profile is required?
3. What belongs in the robot-neutral core, and what belongs in robot-specific
   adapters?
4. How should execution modes vary across different robot families?
5. How can the system be validated without immediate access to real hardware?

Reason:

- writing these questions first prevented the architecture from becoming a vague
  abstraction exercise.

### Step 4: Research prior art and ecosystem reality

The architecture work then looked outward instead of only inward. The research
covered:

- Learning from Demonstration / Programming by Demonstration
- robot task and skill representations
- behavior trees and state-machine tradeoffs
- ROS2 ecosystem tools
- capability modeling / hardware abstraction
- retargeting between different morphologies and DOF counts
- execution semantics of queued robots, streaming robots, trajectory-action
  robots, and offline-program robots
- validation strategies using mocks, simulators, and fake controllers

Reason:

- the project needed to avoid inventing a framework that sounded broad but
  ignored how robotics systems are actually built and tested.

### Step 5: Compare multiple architecture options

At least three architecture families were compared before choosing one:

- raw trajectory replay
- canonical waypoint program with capability-driven adapters
- behavior-tree / skill-graph layer over primitives

Reason:

- a real architecture decision should include rejected alternatives, not only a
  single preferred story.

### Step 6: Pressure-test the chosen direction

After the first proposal was written, it was reviewed again and challenged on
specific issues such as:

- orientation semantics
- raw capture format
- URDF requirements
- MG400 bias in the first lifter milestone

Reason:

- this second pass reduced the chance that early design assumptions would harden
  into the wrong contract.

## 4. Architecture Options That Were Considered

### Option A: Raw trajectory replay

Definition:

- store demonstrations as time-indexed samples and replay them directly.

Why it was attractive:

- smallest migration from the current MG400-oriented trajectory recorder path;
- easy to understand;
- already close to what the existing code can do.

Why it was not chosen as the core architecture:

- weak retargeting across robots with different DOF;
- poor representation of tool actions, wait steps, and higher-level task
  structure;
- difficult to compile into offline programs for different vendors;
- captures motion traces, but not the task meaning well enough.

Report-friendly interpretation:

```text
raw trajectory replay is useful as a capture artifact or fallback path,
but it is too low-level to act as the long-term canonical system core.
```

### Option B: Canonical waypoint program + capability-driven adapters

Definition:

- represent a taught task as a robot-neutral ordered program of `move`, `tool`,
  `wait`, and `set_frame` steps;
- let each robot adapter translate and execute that program according to its own
  capabilities and execution model.

Why it was chosen:

- it balances practicality and generality;
- it matches how real industrial robot workflows often think in terms of
  waypoints + actions;
- it supports queued, streamed, trajectory-based, and offline execution modes;
- it is understandable, storable, diffable, and testable;
- it keeps MG400 as the first adapter instead of making MG400 the definition of
  the whole system.

### Option C: Behavior-tree / skill-graph layer over primitives

Definition:

- put a reactive task-composition layer above lower-level motion/action
  primitives.

Why it was not chosen as the first core:

- it is more expressive than the project currently needs for the first working
  cross-robot architecture;
- it adds more design and implementation complexity before the canonical motion
  layer has been proven;
- segmentation of free-form demonstrations into reusable skills remains harder
  than representing them first as waypoint/action programs.

Why it was not rejected completely:

- research and ROS2 precedent support behavior trees strongly;
- it was therefore moved to a later, explicit follow-on phase rather than being
  discarded.

Report-friendly interpretation:

```text
behavior trees were treated as a planned extension layer,
not as the first architecture milestone.
```

## 5. Why The Current Direction Was Chosen

The chosen direction is:

```text
canonical waypoint-style robot-neutral program
+ robot capability profile
+ robot-specific adapter/executor
```

This direction was chosen because it best satisfied all of the following at the
same time:

- compatible with the real project objective
- implementable in the current repository
- respectful of MG400 queue semantics
- extendable to ROS2-native arms and offline-program workflows
- testable without immediate access to real hardware

In short:

- Option A was too narrow;
- Option C was too early;
- Option B was the best engineering center of gravity.

## 6. Key Design Decisions And Why They Changed

This section records the important architecture decisions that were revised
during the research and review process.

### 6.1 Cartesian pose as the main contract

Decision:

- the canonical program uses Cartesian pose as the primary move contract;
- joint values are treated as hints, not as the main cross-robot contract.

Why:

- cross-embodiment teleoperation and LfD practice strongly converged on
  end-effector/task-space representations;
- pure joint-to-joint retargeting does not generalize well across robots with
  different DOF or morphology.

### 6.2 Add object/task frames

Earlier weakness:

- base-frame-only motion representation makes the program brittle to workspace
  changes.

Decision:

- canonical programs should include named object/task frames.

Why:

- task-parameterized trajectory literature shows that object-relative framing is
  central to replaying the same taught task when object positions change.

### 6.3 Add per-step orientation intent

Earlier weakness:

- knowing what a robot can rotate is not enough;
- the system also needs to know how strict the user's orientation intention is
  for each move.

Decision:

- each move step should carry orientation intent/tolerance information.

Why:

- the same task can mix strict orientation steps and relaxed approach steps;
- a SCARA-like robot cannot infer from a quaternion alone whether flattening
  roll/pitch is acceptable.

### 6.4 Replace rigid URDF requirement with a kinematics contract

Earlier weakness:

- requiring every adapter to have a URDF was too strict and excluded some
  offline or vendor-specific workflows.

Decision:

- require an explicit kinematics contract/strategy instead of blindly requiring
  URDF for every case.

Why:

- some robots can rely on URDF-based pipelines;
- some can rely on named provider implementations;
- some offline-export-only adapters do not need local IK at all.

### 6.5 Separate raw capture from canonical program

Earlier weakness:

- raw capture and canonical program risked collapsing into one artifact.

Decision:

- keep a two-tier model:
  - raw session capture
  - canonical program

Why:

- raw data and interpreted task program serve different engineering purposes;
- this also makes the report story cleaner.

### 6.6 Move the kinematics seam earlier

Earlier weakness:

- one proposal version still allowed the first lifter milestone to depend on
  MG400 FK directly and abstract later.

Decision:

- the kinematics seam should exist from the first milestone of the new core.

Why:

- otherwise the architecture claims robot neutrality while growing from a
  single-robot worldview at the most foundational layer.

## 7. Reasons Some Ideas Were Explicitly Deferred

The following ideas were not rejected forever, but were intentionally deferred.

### Behavior-tree task composition

Deferred because:

- the canonical motion/task layer should be proven first;
- a second real or simulated adapter should exist before the project takes on a
  larger reactive-task framework.

### Machine-learning dataset integration as a core requirement

Deferred because:

- interoperability with research datasets is useful;
- but the project's immediate engineering milestone is still a working
  cross-robot teaching architecture, not a full learning pipeline.

### Hard commitment to one kinematics library

Deferred because:

- the important architecture choice is the `KinematicsProvider` seam;
- the backend library can still be selected later without changing the design
  center.

## 8. Current Open Questions

As of the current architecture phase, the following are still open enough to be
treated carefully in the report:

- the exact orientation representation format in stored programs
- the exact message schema used inside raw captured session bags
- the concrete kinematics backend library used for URDF-backed providers
- the exact BT framework choice if/when the follow-on reactive layer is built

These are not architecture failures. They are controlled open questions after
the main design direction has already stabilized.

## 9. Current Implementation Readiness

The architecture phase is now in a state where the first implementation work
can be scoped more cleanly.

The main items that should be treated as load-bearing before implementation are:

- canonical program field names and semantics
- orientation intent/tolerance semantics
- kinematics contract field names and validation rules
- separation of raw capture artifacts from canonical program artifacts
- adapter contract boundaries

The main items that should **not** be over-frozen too early are:

- the exact kinematics backend library
- the exact bag message schema
- the future BT framework

## 10. How To Use This File In The Final Report

This document is especially useful when writing:

- problem framing / motivation
- architecture methodology
- design alternatives
- why one architecture was selected over others
- reasons for rejecting or deferring alternative approaches
- future work and limitations

Suggested chapter use:

- Chapter "System Architecture": use Sections 2, 4, 5
- Chapter "Design Process / Methodology": use Sections 3 and 6
- Chapter "Discussion": use Sections 7 and 8
- Chapter "Future Work": use Sections 7, 8, 9

## 11. Source Files To Cite Alongside This Log

Use this file together with:

- `docs/project_objective.md`
- `docs/current_refactor_status.md`
- `docs/ai_handoffs/next_phase_architecture_brief.md`
- `docs/ai_handoffs/next_phase_architecture_proposal.md`
- `docs/ai_handoffs/next_phase_architecture_revision.md`
- `docs/report_materials/report_writing_guideline.md`

## 12. Short Summary

If a short summary is needed for a report draft, use:

```text
After completing the cleanup and refactor of the original MG400-first runtime,
the project entered a new architecture phase focused on multi-robot teaching.
Several architecture families were compared. Raw trajectory replay was judged
too narrow, while a behavior-tree-first design was considered too early for the
current milestone. The selected direction was a robot-neutral waypoint-style
program representation combined with robot capability profiles and per-robot
adapters. This direction was reinforced by a dedicated research pass, which led
to several important refinements, including object-relative task frames,
per-step orientation intent, a kinematics contract instead of a blanket URDF
requirement, separation between raw session capture and canonical task program,
and introduction of a kinematics-provider seam from the first implementation
milestone.
```

## 13. Demo-Critical Teach-Repeat Timing Update

Date: 2026-04-26

The project has an immediate demonstration constraint: in the next project
review, the system must convincingly show the MG400 responding to VR/Unity
input and performing pick-and-place style work with suction.  This does not
replace the multi-robot teaching objective, but it changes the implementation
priority: the MG400 path must be robust enough for a live demo while still
moving toward the reusable architecture.

Important clarification:

- Real-time MG400 control and teach-repeat playback are different execution
  modes.
- Real-time mode should follow the latest Unity/VR target as responsively as
  possible.
- Teach-repeat mode should preserve the demonstrated path and timing when the
  robot can physically execute it.
- If the demonstrated hand motion is faster than the robot can execute, the
  correct behavior is not to discard waypoints or apply an arbitrary fixed
  speed cap.  The correct behavior is to preserve all taught waypoints and
  stretch the timeline only as much as needed by the robot limits.

Implementation added:

- `src/robot_teaching_core/teaching_core/trajectory/retiming.py`
  - robot-neutral timed joint trajectory retimer
  - preserves original timestamps when feasible
  - stretches only segments that exceed configured joint velocity limits
  - keeps the original geometric path unchanged
- `src/dobot_mg400/mg400_controller/mg400_controller/common/trajectory/trajectory_recorder.py`
  - uses the retimer during preview/playback
  - computes MG400 `SpeedJ` from segment distance and segment duration
  - emits playback metadata showing original duration, retimed duration, and
    whether original timing was feasible
  - can load already-materialized frames from Unity or other upstream
    trajectory sources
  - tightens final settle tolerance from 2.0 deg to 0.5 deg for better
    pick/place accuracy
- `src/dobot_mg400/mg400_controller/mg400_controller/vr_teleop_node.py`
  - subscribes to Unity's `trajectory_msgs/JointTrajectory` topic
    `/mg400/joint_trajectory_controller/command`
  - converts ROS radians + `time_from_start` points into internal degree
    frames
  - starts MG400 TCP playback through the same retimed `TrajectoryRecorder`
    path used by saved JSON playback
- `tools/demo_lift/retiming_demo.py`
  - software-only demonstration showing both feasible and too-fast hand motion

Current software demo result:

```text
A. demo pick-and-place timing is preserved
waypoints in/out: 7 / 7
original duration: 3.000s
retimed duration:  3.000s
time scale:        x1.000
original feasible: True

B. too-fast hand motion is stretched, not decimated
waypoints in/out: 3 / 3
original duration: 0.100s
retimed duration:  0.389s
time scale:        x3.889
original feasible: False
```

Current live MG400 Mock replay probe after final-settle tightening:

```text
planned_duration_s:   3.0
measured_duration_s:  3.332
max_error_deg:        6.7433
mean_error_deg:       3.0829
final_error_deg:      [0.0, 0.4444, 0.0, 0.0]
final waypoint arrival: target 3.0s, arrival 3.281s, timing error +0.281s
```

Interpretation for the report:

- The taught path can be preserved without waypoint decimation.
- MG400 TCP playback cannot guarantee exact timestamp tracking in the same way
  as a ROS2 `JointTrajectoryController`.
- However, a retiming layer can make the behavior explainable: if the motion is
  feasible, preserve timing; if not, report the minimum required slowdown.
- For pick/place, final-settle tolerance matters more than ending exactly at
  the nominal schedule.  The system now waits for a tighter final error before
  declaring playback complete.

Unity repository status:

- Repository cloned as a sibling workspace:
  `/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/TeleOp`
- Remote: `https://github.com/Bazedo/TeleOp.git`
- Current checked-out branch: `master`
- Latest observed commit:
  `8b29b9c Merge pull request #2 from Bazedo/prevent-project-bom`
  at `2026-04-26 00:18:05 +0700`
- Relevant Unity scripts:
  - `Assets/Scripts/ContinuousTeachAndRepeat.cs`
  - `Assets/Scripts/ROSPathPublisher.cs`
  - `Assets/Scripts/VacummRosControl.cs`
- Unity already publishes:
  - live joint commands on `/unity/joint_cmd`
  - suction command on `/vr/suction_cmd`
  - saved teach-repeat trajectory as `trajectory_msgs/JointTrajectory` on
    `/mg400/joint_trajectory_controller/command`

Next implementation implication:

The ROS/MG400 side should add or adapt a bridge that consumes Unity's
`trajectory_msgs/JointTrajectory`, converts it into the same retimed playback
path used by `TrajectoryRecorder`, and then emits MG400 TCP commands.  This
keeps the Unity-facing contract close to standard ROS practice while still
handling MG400's queued TCP limitations.

Status after this update: the first version of this bridge exists in
`vr_teleop_node.py`.  It still needs a real Unity-to-ROS smoke test with the
Quest/Unity scene, but unit tests now cover the conversion from Unity-style
`JointTrajectory` points into internal degree frames.
