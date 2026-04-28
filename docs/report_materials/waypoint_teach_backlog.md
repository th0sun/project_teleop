# Waypoint Teach Backlog

Status: deferred idea, not current demo scope.

This note records a future teach-and-repeat mode discussed during the MG400
demo-preparation phase.  It is intentionally kept as backlog material so the
current continuous teach-and-repeat path can be stabilized first.

## Problem

The current teach-and-repeat mode records a continuous path while the user moves
the robot in Unity/VR.  This is useful for showing hand-following behavior, but
it creates many frames and makes replay depend on filtering, retiming, and robot
queue behavior.

For pick-and-place style tasks, the operator often wants a simpler industrial
teach-pendant workflow:

1. Move the robot to an important pose.
2. Capture that pose.
3. Repeat for approach, pick, lift, place, and retreat poses.
4. Attach digital-output events such as vacuum on/off.
5. Execute the resulting sequence with adjustable speed, acceleration, and CP.

## Deferred Concept

Add a separate `Waypoint Teach` mode/page in Unity instead of adding more state
to the existing continuous recorder.

The mode would capture only intentional waypoints.  It would not preserve hand
movement timing.  Execution timing should be generated later by the ROS/MG400
compiler from operator-selected parameters such as:

- speed
- acceleration
- CP/blend
- motion type per segment

This keeps the recorded program editable and easier to reason about than a dense
timestamped path.

## Why It Is Deferred

This should not be implemented before the current demo path is stable.

Reasons:

- The existing continuous teach-and-repeat path is already connected to Unity,
  ROS, Mock, and the MG400 adapter.
- Adding another page and controller-button workflow now would increase demo
  risk.
- The current priority is to make the existing path reliable, visible, and
  testable on Mock/Ubuntu.
- Motion primitive selection (`JointMovJ`, `MovL`, `Arc`) still needs more real
  hardware validation before it becomes a default user-facing choice.

## Recommended Future Shape

Use a separate Unity component/page, for example:

```text
WaypointTeachAndRepeat
```

Keep the backend shared:

```text
Unity waypoint UI
    -> TeachJobPublisher
    -> /teach/job_request
    -> ROS teach job handler
    -> MG400 compiler/executor
```

Do not duplicate the ROS job pipeline.

## Suggested UI

- `Start Waypoint Teach`
- `Add Point`
- `Undo Last Point`
- `Clear`
- `Vacuum On`
- `Vacuum Off`
- `Compile / Preview`
- `Send To Robot`

Optional later controls:

- default speed
- default acceleration
- default CP
- motion type for the next segment: `joint`, `linear`, `arc`

## Data Model Direction

The saved artifact should explicitly identify the teach mode:

```json
{
  "teach_mode": "waypoint",
  "frames": [
    {
      "j1": 0.0,
      "j2": 0.0,
      "j3": 0.0,
      "j4": 0.0,
      "motion": "joint"
    }
  ],
  "events": [
    {
      "after_frame": 0,
      "kind": "digital_output",
      "channel": "vacuum",
      "value": true,
      "port": 0
    }
  ],
  "default_motion": {
    "speed": 20,
    "acc": 20,
    "cp": 0
  }
}
```

The important distinction from continuous recording is that waypoint mode should
not treat `timeStamp` as replay authority.  If timestamps are kept for debugging,
they should be metadata only.

## Timing Policy

Do not use the time gap between button presses as robot playback timing.

The operator may pause to think, inspect the scene, or talk during teaching.
Those pauses should not become robot dwell time unless an explicit wait step is
added.

Instead:

- use ordered waypoints as the source of truth
- use explicit wait/events for intentional pauses
- let the ROS/MG400 compiler retime motion from speed/acc/CP settings

## Initial Motion Policy

For the first implementation, default to `JointMovJ`.

`MovL` and `Arc` should be added only after:

- sampled path preflight is reliable
- workspace/fixture collision checks are integrated
- real MG400 behavior is validated for the chosen primitive

This avoids repeating the demo-phase issue where visually plausible arc/line
fits produced controller alarms on real hardware.

## Open Questions

- Should waypoint capture be UI-only first, or should a controller shortcut be
  added later?
- Which controller buttons are actually free in the target Unity scene?
- How should the UI show digital-output events attached to a waypoint?
- Should arc mode use exactly three captured points, or a user-marked segment?
- What safety preflight is required before enabling real robot execution?

## Acceptance Criteria For Future Work

- Continuous teach-and-repeat remains unchanged and stable.
- Waypoint mode is a separate UI path, not mixed into the continuous recorder.
- Saving a waypoint program produces an artifact with `teach_mode: waypoint`.
- Execute ignores button-press timing and uses explicit motion parameters.
- Vacuum/DO events survive save, load, compile, and execute.
- Mock/ROS integration can run the whole waypoint job without requiring a real
  robot.
