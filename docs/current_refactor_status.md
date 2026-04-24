# Current Refactor Status

This note marks the end of the legacy refactor / cleanup phase for the current
MG400-first codebase.

It exists so future contributors do not reopen the same cleanup work by
mistake.

## Phase Outcome

The refactor phase for the original system shape is considered **complete
enough to close**.

That means:

- the old teleop runtime has been split into clearer reusable layers;
- the monitor runtime has been split into ROS-facing and monitor-domain layers;
- playback logic is no longer a single opaque loop and now has targeted tests;
- project structure, monitor tools, manuals, and submodules were cleaned up;
- continuation docs and project objective were aligned with the real direction
  of the project.

## What Was Closed In This Phase

Closed areas:

- teleop runtime cleanup
- queue-aware MG400 execution path cleanup
- monitor GUI/runtime cleanup
- ROS2 boundary extraction
- monitor-domain extraction
- trajectory playback helper extraction
- project structure cleanup
- submodule/manual cleanup
- continuation and report-facing documentation cleanup

Representative commits from this phase:

- `bbba64a` `refactor(teleop): extract queue-aware runtime helpers`
- `641bbef` `refactor(monitor): extract ros and monitor-domain layers`
- `865e7b5` `refactor(playback): make trajectory recorder testable`
- `55fb434` `docs(objective): center project on multi-robot teaching`

## What This Phase Did Not Solve

This phase did **not** finish the next architecture step.

Still intentionally open:

- canonical robot-neutral program/task representation
- robot capability profile schema
- adapter layer for multiple robot families
- execution-mode selection based on robot capability
- true multi-robot teaching pipeline

Those belong to the next phase, not to legacy cleanup.

## Guidance For The Next Phase

From this point onward, avoid spending major effort on more MG400-only cleanup
unless it blocks the new architecture direction.

The next phase should focus on:

1. defining the canonical taught-program representation
2. defining the robot capability profile
3. separating robot-neutral teaching logic from robot-specific adapters
4. treating MG400 as the first adapter, not the whole system definition

Useful kickoff files for that phase:

- `docs/ai_handoffs/next_phase_architecture_brief.md`
- `docs/skills/multi_robot_architecture/SKILL.md`

## Short Decision Rule

If a proposed change mostly improves tidiness inside the current MG400 runtime,
the legacy refactor phase is probably already good enough.

If a proposed change improves how the system can teach and adapt tasks across
different robots, it belongs to the next phase and is worth prioritizing.
