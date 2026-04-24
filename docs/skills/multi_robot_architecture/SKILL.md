# Multi-Robot Architecture Skill

Use this skill when the task is to design the next architecture phase of this
repository around robot teaching across multiple robot families.

## Mission

You are not here to merely improve MG400 teleop.

You are here to design a framework that can:

- capture task teaching from VR/Unity hand motion,
- represent that teaching in a robot-neutral way,
- adapt it to robots with different capabilities,
- execute it through the most appropriate strategy for each robot.

The architecture must be broad, practical, research-grounded, and implementable
inside this repository.

## First Principles

Keep these truths in view:

1. The project objective is **robot teaching**, not raw low-latency control by
   itself.
2. MG400 is the first adapter and an important case study, but not the
   definition of the whole system.
3. Different robots may require different execution modes:
   - offline program upload
   - queued playback
   - supervised execution
   - near-real-time streaming
4. The architecture must separate:
   - user intent
   - canonical program/task meaning
   - robot capability description
   - robot-specific translation/execution

## Required Reading Inside The Repo

Read these before proposing architecture:

1. `docs/project_objective.md`
2. `docs/current_refactor_status.md`
3. `docs/ai_handoffs/next_phase_architecture_brief.md`
4. `docs/controller_continuation_guide.md`
5. `docs/teleop_command_logic.md`

## Required Research Areas

Before choosing an architecture, research the following areas using primary
sources where possible:

1. Learning from Demonstration / Programming by Demonstration
2. Robot skill and task representations
   - waypoint programs
   - skill graphs
   - behavior trees
   - state machines
   - parameterized task models
3. Robot capability modeling / hardware abstraction
4. Retargeting between different robot morphologies and DOF counts
5. Execution-mode differences
   - offline programming
   - queued command execution
   - servo/streaming control
   - hybrid supervised playback
6. Simulation, mocking, and validation strategies for robot software
7. ROS2 ecosystem tools that may help with adapters, simulation, or execution

## Research Method

Do not just collect facts. Synthesize.

Your workflow should be:

1. research broadly;
2. cluster the design space into a few viable architecture families;
3. compare them against this repository's objective and constraints;
4. choose one direction with clear reasons;
5. identify weaknesses early and propose mitigations.

Prefer recent, credible, implementation-relevant sources.

## Mandatory Design Outputs

Produce all of the following:

1. **Problem framing**
   - define the actual architecture problem in this repo

2. **Option set**
   - propose at least 2-3 architecture approaches
   - compare tradeoffs honestly

3. **Chosen architecture**
   - explain why it best fits the objective

4. **Canonical program schema**
   - what a taught task looks like independent of robot brand/model

5. **Robot capability profile schema**
   - minimum information needed to adapt the task to a robot

6. **Adapter boundary**
   - define what every robot adapter must implement

7. **Execution mode model**
   - define how the system chooses between offline, playback, queue-aware, or
     near-real-time execution

8. **Validation strategy**
   - propose how to test the architecture without real hardware first
   - include mocks/simulators and contract tests

9. **Migration plan**
   - map current MG400-first code into the new architecture

10. **Weakness audit**
    - list where the chosen architecture could fail or become too abstract
    - give concrete mitigations

## Practical Validation Requirement

The proposal is not complete unless it includes a real testing path.

You must think about:

- contract tests for canonical program -> adapter translation;
- mock-backed tests for queued/streamed execution logic;
- simulator-backed tests for basic motion semantics;
- regression tests that do not require a physical robot every time.

Actively look for usable mock/simulation options across the robotics ecosystem.
Examples to consider include ROS2-friendly simulators, vendor simulators, TCP
mock servers, and generic physics/simulation environments.

Do not assume one mock is enough. Build a validation ladder.

## Anti-Patterns

Avoid these traps:

- overfitting architecture to MG400 queue semantics;
- inventing abstractions with no clear implementation path;
- treating the canonical representation as just a copy of MG400 commands;
- optimizing only for latency;
- ignoring tool/IO/task semantics;
- choosing a design that cannot be validated until real hardware appears.

## Decision Rule

If a design choice mostly helps one robot stream faster, it is probably too
narrow for the system core.

If a design choice makes it easier to teach once and adapt across robots, it is
probably closer to the right center of gravity.

## Deliverable Style

Your output should be:

- structured;
- specific;
- implementation-oriented;
- explicit about uncertainty;
- honest about tradeoffs;
- strong enough that another engineer can start building from it.
