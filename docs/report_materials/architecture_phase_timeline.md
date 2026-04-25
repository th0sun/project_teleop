# Architecture Phase Timeline

This file records the chronological development story of the current
architecture phase.

Use it when writing report/thesis sections such as:

- development timeline
- methodology timeline
- iterative design process
- why the project direction changed
- why the current architecture proposal did not appear all at once

This file is intentionally chronological.

## 1. Timeline Purpose

The purpose of this file is to preserve the step-by-step history of how the
project moved from:

```text
MG400-first teleoperation cleanup
```

to:

```text
multi-robot teaching architecture design
```

This is useful because the final architecture proposal was not created in one
step. It emerged through:

- cleanup of the old codebase
- reframing of the project objective
- creation of architecture prompts and handoff material
- first-pass architecture proposal
- research-backed revision and pressure-testing

## 2. Phase 0 — Legacy Runtime And MG400-First Focus

Before the current architecture phase, the project mainly behaved like a
MG400-first teleoperation system.

The engineering center of gravity at that stage was:

- sending motion commands from VR/GUI input to the real MG400
- dealing with queue semantics of the MG400 controller
- making the final runtime more robust and understandable

This phase was still important because it revealed the real robot constraints
clearly, especially:

- queued execution behavior
- the difference between mock behavior and real-hardware expectations
- the importance of queue-aware pacing instead of blindly chasing low latency

## 3. Phase 1 — Cleanup / Refactor Of The Existing System

The first major step before any new architecture work was to clean up the
existing codebase enough that a new design would not be buried under legacy
confusion.

Representative commits from this phase:

- `bbba64a` `refactor(teleop): extract queue-aware runtime helpers`
- `641bbef` `refactor(monitor): extract ros and monitor-domain layers`
- `865e7b5` `refactor(playback): make trajectory recorder testable`
- `55fb434` `docs(objective): center project on multi-robot teaching`
- `4e58bb0` `docs(status): close legacy refactor phase`

Main outcomes of this phase:

- teleop runtime boundaries became clearer
- monitor/runtime concerns were separated more cleanly
- playback logic became more testable
- repo structure and documentation became easier to hand off

Why this phase matters in the report:

- it shows that the project did not jump directly into grand architecture work;
- the old system was stabilized first so the next phase would rest on clearer
  engineering ground.

## 4. Phase 2 — Project Objective Was Corrected

After the old cleanup phase, a critical realization was written down explicitly:

```text
the project is fundamentally about teaching robots from VR/Unity hand motion,
not only about making MG400 feel more real-time.
```

This was an important turning point.

What changed conceptually:

- MG400 was reinterpreted as the first adapter / case study
- real-time control was reframed as an execution mode or system extension
- the main problem became:

```text
how to capture a taught task once and adapt it across robots with different
capabilities
```

Why this matters in the report:

- this is the moment where the engineering contribution became broader and more
  research-worthy than MG400-only teleoperation tuning

## 5. Phase 3 — Architecture Brief And Skill Preparation

Once the project objective was corrected, dedicated AI handoff material was
created so the next architecture phase would not drift back into MG400-only
cleanup.

Key artifacts created for that purpose:

- `docs/ai_handoffs/next_phase_architecture_brief.md`
- `docs/skills/multi_robot_architecture/SKILL.md`
- `docs/ai_handoffs/architecture_prompt_th.md`

Purpose of these files:

- define the problem clearly
- stop future contributors from optimizing only for MG400 latency
- require research before architecture decisions
- set expectations for deliverables such as:
  - canonical program schema
  - capability profile
  - adapter boundary
  - validation ladder
  - migration plan

Why this matters in the report:

- it shows that the architecture phase was not improvised;
- there was a deliberate setup step to structure the design work.

## 6. Phase 4 — First-Pass Architecture Proposal

After the brief and skill files existed, the first substantial architecture
proposal was produced.

Main content of the first-pass proposal:

- compare architecture options
- choose a preferred architecture
- define a canonical program idea
- define a capability profile idea
- define an adapter contract
- define execution modes
- define a migration plan
- define a validation strategy

The architecture options considered were:

1. raw trajectory replay
2. canonical waypoint program + capability-driven adapters
3. behavior-tree / skill-graph layer over primitives

The chosen direction was:

```text
canonical waypoint-style robot-neutral program
+ robot capability profile
+ robot-specific adapters
```

Why this phase matters in the report:

- it is the first point where the project stopped being only a local runtime
  cleanup effort and became an actual system-design effort.

## 7. Phase 5 — Research Pass And External Validation

The first-pass proposal was then checked against external evidence instead of
being accepted only because it "sounded good."

Research topics included:

- Learning from Demonstration / Programming by Demonstration
- cross-embodiment teleoperation and dataset practice
- task and skill representations
- behavior trees in robotics and ROS2
- capability modeling and ROS2 control stacks
- SCARA / 4-axis orientation limits
- execution semantics across robot families
- validation tooling available in ROS2 and simulators

This phase produced:

- stronger evidence for the chosen architecture core
- clearer relation to ROS2 ecosystem practice
- clearer understanding of what should be deferred

Why this phase matters in the report:

- it demonstrates that the design direction was evidence-seeking, not just
  opinion-driven

## 8. Phase 6 — Pressure Pass And Correction Of Weak Points

After the research pass, several specific weaknesses were challenged and then
patched into the proposal.

The important pressure points were:

1. orientation semantics were too weak if modeled only at the robot-profile
   level
2. raw capture format story was not yet fully consistent
3. blanket URDF requirement was too restrictive
4. an early migration step still risked baking MG400 kinematics into the core

These challenges led to important design corrections such as:

- per-step orientation intent and tolerance
- a two-tier artifact model for raw sessions vs canonical programs
- `KinematicsContract` instead of a blanket URDF requirement
- moving the kinematics seam earlier in the migration plan

Why this phase matters in the report:

- it shows real iterative engineering decision-making
- it provides strong material for a "design refinement" or "architecture
  revision" subsection

## 9. Current Document State

At the current point in time, the key files of this phase are:

- `docs/project_objective.md`
- `docs/current_refactor_status.md`
- `docs/ai_handoffs/next_phase_architecture_brief.md`
- `docs/ai_handoffs/next_phase_architecture_proposal.md`
- `docs/ai_handoffs/next_phase_architecture_revision.md`
- `docs/report_materials/architecture_phase_research_log.md`

The intended source-of-truth logic is:

- `next_phase_architecture_proposal.md` = main architecture proposal
- `next_phase_architecture_revision.md` = audit trail / research-backed review
- `architecture_phase_research_log.md` = report-facing explanation of the
  process and rationale

## 10. Current Status In Practical Terms

The current architecture phase should be described honestly as:

```text
proposal-ready, research-backed, and pressure-tested,
but not yet fully implemented in the runtime
```

This means:

- the design direction is already much clearer than before
- the reasoning behind the design has been documented
- the project now has enough structure to begin implementation more safely
- but the architecture is still a plan, not yet a completed multi-robot core

## 11. Suggested Use In The Final Report

Use this file when writing sections such as:

### Development Timeline

Describe the transition:

```text
MG400-first teleoperation runtime
    -> runtime cleanup/refactor
    -> objective reframing
    -> architecture brief
    -> first architecture proposal
    -> research-backed revision
    -> implementation planning
```

### Design Iteration

Use the pressure-pass phases to show:

- why first-pass designs were not accepted uncritically
- how evidence changed the proposal
- why the final proposal is stronger than the first one

### Discussion

Use the chronology to explain:

- why some alternatives were deferred rather than rejected
- why the system is now better aligned with the real project objective

## 12. Short Chronology Summary

If you need a short paragraph for a report draft, use:

```text
The current architecture phase began only after the legacy MG400-first runtime
was cleaned up and documented. Once the project objective was clarified as
multi-robot teaching rather than MG400-only real-time control, dedicated
handoff and architecture-brief material was prepared. A first-pass architecture
proposal was then produced by comparing several design alternatives, with a
canonical waypoint-style robot-neutral program plus capability-driven adapters
selected as the best balance between practicality and extensibility. That
proposal was subsequently pressure-tested through an additional research and
review pass, which led to important refinements such as object-relative task
frames, per-step orientation intent, a more flexible kinematics contract, and a
clearer separation between raw captured sessions and canonical task programs.
```
