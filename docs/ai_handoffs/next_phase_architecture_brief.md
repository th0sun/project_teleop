# Next-Phase Architecture Brief

Use this brief when handing the repository to a new AI agent or teammate for
the next major phase.

This file is intentionally direct. It should reduce drift and stop the next
worker from falling back into MG400-only cleanup.

## Mission

Design the next architecture phase of this project around the real objective:

```text
turn VR/Unity hand teaching into a robot-neutral task/program representation
that can be adapted to many robots with different capabilities.
```

The new worker should treat this as a **multi-robot teaching system design**
problem, not as a narrow MG400 teleop tuning task.

## Read This First

In this order:

1. `docs/project_objective.md`
2. `docs/current_refactor_status.md`
3. `docs/controller_continuation_guide.md`
4. `docs/teleop_command_logic.md`
5. `docs/reference_manuals/dobot/README.md`

## Current State

What already exists and should be respected:

- the legacy MG400-first runtime has already been cleaned up;
- teleop, monitor, and playback code are much more modular than before;
- the project now has a stable documentation backbone;
- MG400-specific queue-aware execution remains useful as the first robot
  adapter case;
- real-time behavior should now be treated as an execution mode, not as the
  center of the architecture.

## What Not To Do

Do not spend the new phase mostly on:

- squeezing a bit more real-time behavior out of MG400;
- reorganizing already-clean files without moving the architecture forward;
- baking MG400 command assumptions into the new system core;
- writing a generic abstraction layer with no validation path;
- proposing a framework that sounds broad but cannot be tested without hardware.

## What To Design

The next phase should answer these questions:

1. What is the canonical taught-program representation?
2. What minimum robot capability profile is needed to adapt one taught task to
   many robots?
3. What belongs in robot-neutral teaching logic versus robot-specific adapters?
4. How should execution modes vary between:
   - offline program generation
   - supervised playback
   - queued execution
   - near-real-time streaming
5. How can the system be validated repeatedly with mocks, simulators, or
   open-source robot stacks before real hardware is available?

## Expected Deliverables

The next worker should ideally produce:

1. an architecture proposal with at least 2-3 options and a chosen direction;
2. a canonical program schema proposal;
3. a robot capability profile schema;
4. an adapter interface proposal;
5. an execution-mode model;
6. a migration plan from the current MG400-first codebase;
7. a practical validation plan using mocks/simulators;
8. a risk register with weak spots and mitigations.

## Practical Constraints

The design must be:

- implementable in this repository;
- compatible with a ROS2-centered workflow;
- realistic for robots with queued TCP command models like MG400;
- extendable to robots that support different execution semantics;
- testable without requiring immediate access to real hardware.

## Quality Bar

The new phase should feel convincing at both levels:

- **research level**: grounded in real robotics abstractions and prior art;
- **engineering level**: specific enough that we can start implementing it.

If a proposal cannot show how it will be tested with available mocks or
simulators, it is incomplete.

## Success Signal

The next phase is successful if, after reading the output, a maintainer can say:

```text
I understand what the robot-neutral core is, what the robot-specific adapter is,
and how a new robot would be integrated without rewriting the whole system.
```
