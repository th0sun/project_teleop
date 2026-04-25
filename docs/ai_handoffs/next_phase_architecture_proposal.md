# Next-Phase Architecture Proposal

> **This file is the single source of truth for the next-phase
> architecture.** Implementation must follow this document.
> `docs/ai_handoffs/next_phase_architecture_revision.md` is **audit
> log only** — it records *why* sections of this file read the way
> they do, not *what* the architecture is. If those two files ever
> disagree, this one wins; the revision file must be patched to
> match.

Status: **proposal / draft v1.2** (second pressure-pass folded in:
per-step orientation intent now required, KinematicsContract replaces
URDF-required, two-tier capture model `*.session.mcap`/`*.program.json`,
M4/M5 collapse). Not yet implemented.

Scope: answer the design questions in
`docs/ai_handoffs/next_phase_architecture_brief.md` with a concrete, testable
plan for turning VR/Unity hand teaching into a robot-neutral program that can
be adapted to robots other than MG400.

This document is the intended handoff artifact for the next phase. It is
opinionated. Tradeoffs are stated where the choice is not obvious.

Companion file `docs/ai_handoffs/next_phase_architecture_revision.md`
records the research findings (primary-source citations) behind every
v1.1 change. This file is the canonical proposal; the revision file is
the audit trail.

### Prior art and why we proceed anyway

A robot-neutral motion vocabulary is not a new idea. ROS-Industrial
shipped three attempts that are now in `ros-industrial-attic`:
`simple_message` + `industrial_robot_client` (joint-trajectory
streaming), CRCL ("Canonical Robot Command Language", NIST-led
vendor-neutral motion language), and `robot_movement_interface`
(Cartesian/joint move primitives). All three stalled. The lesson: a
neutral surface without a real first adapter, kept thin, anchored in
the ecosystem the users already live in, drifts into ambition without
delivery. We proceed because (a) MG400 is a real first adapter, (b)
the IR is deliberately v0.1-narrow (move/tool/wait/set_frame), and (c)
the profile leans on URDF/SRDF/ros2_control rather than redefining
them. See R11 in §7.

---

## 1. Problem Framing

The real design problem is not "how do we make MG400 stream better." It is:

```text
How do we capture a VR teaching session once, store it as a robot-neutral
program, and replay it on any robot whose capability profile we can describe?
```

This splits cleanly into five concerns that are currently entangled inside
`vr_teleop_node.py` and `trajectory_recorder.py`:

1. **Capture** — raw, high-rate hand/joint stream from Unity (demonstration
   signal).
2. **Segmentation / lifting** — turning the raw stream into meaningful
   program units (move, wait, tool action, constraint).
3. **Canonical representation** — a robot-neutral IR that can be stored,
   edited, diffed, and replayed.
4. **Adaptation** — converting that IR into commands a specific robot
   accepts, honoring its capability profile and execution mode.
5. **Execution** — running the adapted plan offline, under supervision,
   queued, or streamed.

Today, (1)+(2)+(3)+(4)+(5) all live implicitly inside the MG400 runtime.
The canonical layer does not exist as a first-class artifact.

### Non-goals this phase

- making MG400 "feel" more real-time
- building a universal DSL that covers every robot ever shipped
- replacing MoveIt or ROS2 controllers for robots that already have a
  mature stack — we wrap, not replace

---

## 2. Research Synthesis

Condensed synthesis of the research areas required by the skill brief.
Sources listed with enough specificity that the next worker can re-derive
them, not as padding.

### 2.1 Learning / Programming by Demonstration (LfD / PbD)

Two dominant lineages:

- **Trajectory-level LfD** — store demonstrations as time-indexed state
  sequences; replay directly or via DMPs / ProMPs / GMM-GMR for
  generalization. Good when the task is about *motion shape*. Ijspeert's
  DMP line and Paraschos' ProMP line are the canonical references.
- **Symbolic / skill-level PbD** — segment demonstrations into named
  primitives ("pick", "place", "insert", "approach") parameterized by
  pose/object. Good when the task is about *what to do*, not *exactly
  how*. Kroemer et al. survey (A Review of Robot Learning for
  Manipulation, JMLR 2021) is the clean reference.

Practical rule for this project: we need **both**, but at different
moments. The capture signal is trajectory-level. The stored program is
most useful if it is segmented waypoints + discrete actions, optionally
annotated with a skill name.

### 2.2 Task / skill representations

Five families, ordered by abstraction:

1. **Raw trajectories** (JSON/CSV of joint or Cartesian samples) — what
   `trajectory_recorder.py` does today.
2. **Waypoint programs with actions** — ordered list of targets
   (joint or Cartesian), motion type, blending, tool/IO events. Matches
   how teach pendants (Dobot, UR, Fanuc, ABB) think.
3. **Parameterized skills / primitives** — `MoveL`, `MoveJ`,
   `LinearApproach(pose, offset)`, `Grasp(object_frame)`. Typical of
   skill libraries (KUKA sunrise, RLBench, ROBOSUITE).
4. **Behavior trees / state machines** — compose skills with control
   flow. ROS2 `py_trees_ros`, Nav2 BT. Needed when tasks branch or
   react to sensing.
5. **Policy / learned model** — a neural net that outputs actions. Out
   of scope for this repo's first two phases.

This project should stop at (2) as its canonical core, with (3) as
optional metadata. (4) can be layered later without changing the IR.

### 2.3 Capability modeling / HAL

There is no single industry standard. Most production systems roll their
own capability descriptor. The useful ingredients, distilled:

- **Kinematic identity** — DOF count, joint order, limits, base frame.
  URDF already covers this. Do not reinvent.
- **Motion vocabulary** — which motion types the controller accepts
  (joint, linear, circular, spline, servo), and blending / CP support.
- **Execution semantics** — queued vs streaming vs offline; sync
  primitives; speed/acc units; immediate vs queued commands.
- **Tool / IO model** — DO ports, vacuum/gripper abstraction.
- **Tolerances & limits** — max speed/acc, payload, workspace.
- **Timing contract** — how close to "now" the robot actually follows
  commands.

The correct shape is a profile struct the **adapter owns**, not a core
truth. Core only reads it to decide what it can ask for.

### 2.4 Retargeting

Across morphology (4-DOF MG400 vs 6-DOF UR/xArm, 7-DOF Panda), the
only robust approach is **Cartesian-anchored retargeting**:

- store the taught program in task space (tool/TCP pose) as the
  primary representation;
- keep the source joint sample as a hint, not as the contract;
- let each adapter run its own IK to realize the Cartesian waypoint
  within its own joint limits and redundancy resolution.

Pure joint-space retargeting (J1..J4 → J1..J6) does not generalize and
should be rejected as the canonical form.

Reference: Rakita et al. *RelaxedIK* (RSS 2018) — retargeting for
dissimilar arms, and the retargeting literature around teleop (Handa
et al. *DexPilot*, 2020).

### 2.5 Execution semantics by robot family

Concrete examples, for designing the exec-mode abstraction:

- **Queued-TCP robots** — Dobot MG400, Dobot Nova series. Commands
  enter a firmware queue; host owns pacing; blending via `CP`. This
  repo already knows this family well.
- **Streaming / servo robots** — UR (servoj / RTDE at 500 Hz), Franka
  Panda (1 kHz via libfranka), xArm servo_j. Host provides a fresh
  setpoint every control tick; drops = safety stop.
- **Offline-program robots** — older ABB/Fanuc flows, Dobot Studio
  offline programs. Whole program uploaded then executed.
- **Trajectory-action robots** — anything behind a
  `control_msgs/action/FollowJointTrajectory` ROS2 action. Host sends a
  discretized trajectory; controller executes. This is the **default
  for ROS2-native arms** (UR, Franka, xArm, Kinova) via
  `joint_trajectory_controller` /
  `scaled_joint_trajectory_controller`. Distinct from `QUEUED` (no
  controller-side trajectory object) and from `STREAMING` (continuous
  setpoints). Gets its own `ExecutionMode` value in §4.6.

A single canonical program must be convertible into each of these.
That is exactly what the adapter boundary is for.

### 2.6 Validation tooling realistically available

- **Unit-level** — plain `unittest`, no ROS, no hardware. Already in
  the repo.
- **Contract tests** — golden canonical programs + assertion that each
  adapter emits a known command sequence. Cheap, high-value.
- **TCP mock** — `MG400_Mock` submodule, usable today. Known to
  under-report `RunQueuedCmd` and ignore `CP`. Good for socket plumbing
  only.
- **Kinematics/physics sim** — PyBullet + URDF (MG400 URDF exists in
  the Dobot community; UR/Panda ship with theirs) is the lowest
  friction option and does not require ROS build. Good for motion
  semantics without firmware quirks.
- **ROS2 + fake controllers** — `ros2_control` with `fake_components`
  plugin gives a `FollowJointTrajectory` that responds deterministically.
  Good for validating the trajectory adapter without Gazebo.
- **Gazebo Harmonic / Ignition** — heaviest. Only for end-to-end scene
  validation. Avoid as a first-line test.

These will be stacked into the validation ladder in §8.

---

## 3. Architecture Options

Three honest options, compared against the project objective. Each is
described at the level where the tradeoff actually lives, not as a
brochure.

### Option A — "Raw trajectory replay" (status-quo extended)

Keep `trajectory_recorder.py`'s JSON-of-samples as the canonical form.
Add a thin adapter layer that knows how to feed samples to a target
robot via its native motion command.

- **Pros** — tiny migration; already works for MG400; easy to reason
  about; no IK required at replay time on same-morphology robots.
- **Cons** — does not retarget across DOF counts; no notion of
  actions, IO, or constraints; any edit of the program is at the
  sample level; impossible to support offline-program robots well
  because there is no structure to compile.
- **Kills the objective?** Yes. This is basically what we have. It
  cannot teach "the task," only "the hand wiggle."

### Option B — "Canonical waypoint program + capability-driven adapters"

Canonical IR is a structured list of steps: `MoveJ` / `MoveL` /
`ToolAction` / `Wait` / `SetFrame`, each parameterized by Cartesian
pose (primary) and joint hint (secondary), plus speed/acc/blend
intent. A lifter segments the raw hand stream into this list.

Each adapter is a pair of:

- a **capability profile** (static struct describing the robot);
- a **translator** (canonical step → native commands);
- plus an **executor** that honors the robot's execution mode.

- **Pros** — matches how real teach pendants think, so it maps onto
  most industrial robots cleanly; retargets across DOF via Cartesian
  primary + per-adapter IK; supports offline/queued/streaming equally;
  canonical programs are small JSON, human-readable, diffable; works
  today with MG400 as first adapter.
- **Cons** — requires a real segmentation/lifter step from the raw
  capture stream; needs a URDF or kinematics model per adapter for
  Cartesian replay; waypoints alone cannot express branching or
  sensing-dependent behavior.
- **Kills the objective?** No. This is the minimum viable robot-neutral
  core. It is also the smallest step that does not bake in MG400
  assumptions.

### Option C — "Behavior-tree / skill graph over primitives"

Canonical IR is a tree of named skills (`pick(frame)`,
`linear_approach(pose, offset)`, `wait_until(...)`), leaves expand
into Option-B-style waypoints at compile time.

- **Pros** — most expressive; scales to tasks with branching and
  sensing; aligns with published skill-library research.
- **Cons** — segmentation of raw hand motion into skills is an
  unsolved research problem without priors; most teaching sessions
  today will have no recognizable skill labels, so the tree degenerates
  into a single `replay` skill; adds a whole new compilation layer we
  are not ready to test.
- **Kills the objective?** Over-reaches as a ground-floor choice, but
  the **right direction** as a v0.2 layer on top of B. ROS2 production
  precedent (Nav2 BT navigator) and the survey evidence (Iovino et al.
  2022; Biggar & Zamani 2024 showing BT edits are O(1) vs FSM O(n))
  make BT-over-skills the current ROS2 idiom for composing reactive
  skill sequences. Scheduled explicitly as Phase **M7** in §6, built
  on top of the v0.1 IR without breaking it.

### Decision matrix

| Criterion | A | B | C |
|---|---|---|---|
| Fits stated objective | no | yes | yes but premature |
| Works for MG400 today | yes | yes | no (not without B first) |
| Retargets across DOF | no | yes | yes |
| Testable without hardware | yes | yes | partial |
| Adds next robot cheaply | no | yes | yes |
| Implementation cost this phase | tiny | medium | large |

**Chosen: Option B.** Option C is scheduled as v0.2 (Phase M7) on top
of B. Option A is rejected as insufficient.

---

## 4. Chosen Architecture

### 4.1 Layer map

```text
[ Teaching Input ]       Unity / VR hand stream      (raw demonstration)
        │
        ▼
[ Capture ]              common/capture/             (Unity stream → *.session.mcap)
        │
        ▼
[ Lifter / Segmenter ]   common/lifter/              (session → canonical IR)
        │
        ▼
[ Canonical Program ]    data artifact (JSON)        — robot-neutral
        │
        ▼
[ Adapter ]              adapters/<robot>/           (canonical → native plan)
        │                 ├─ capability profile
        │                 ├─ translator
        │                 └─ executor (mode-aware)
        ▼
[ Robot ]                MG400 / UR / Panda / ...
```

Everything above the canonical program is **robot-neutral** and lives
in a new package (proposed: `robot_teaching_core`). Everything below is
**robot-specific** and lives under `adapters/<robot_name>/`. MG400
becomes `adapters/mg400/` — the first adapter, not the center.

### 4.2 Package layout (proposed)

```text
src/
  robot_teaching_core/           # new, robot-neutral
    teaching_core/
      capture/                   # raw session IO (Unity stream → *.session.mcap)
      lifter/                    # segmentation + Cartesian lifting
      program/                   # canonical IR types + schema + IO
      capability/                # RobotCapabilityProfile dataclass + validation
      adapter_api/               # abstract Adapter / Translator / Executor
      validation/                # contract-test harness
    test/
  adapters/
    mg400/                       # = today's mg400_controller, trimmed
      mg400_adapter/
        profile.py               # returns the capability profile
        translator.py            # canonical step → Dobot TCP string
        executor.py              # queue-aware runner (existing logic)
        ros_teleop_node.py       # the live-teaching entry point
    ur/                          # placeholder, proves the seam exists
    fake/                        # in-memory executor for CI
```

`vr_teleop_node.py` does **not** move wholesale. Only the robot-neutral
parts of it (target validation transforms, latency compensator, the
program-capture path) move into `robot_teaching_core`. The queue-aware
controller stays in `adapters/mg400/` because it is MG400-specific.

### 4.3 Canonical program schema (v0.1)

Primary key insight: **Cartesian pose is the contract; joint values
are a hint, not the source of truth.** This is what makes retargeting
possible. Adapters without Cartesian capability (pure joint-replay
robots) can fall back to the hint, but the canonical form does not
lose information.

```json
{
  "schema_version": "0.1",
  "program_id": "pick_demo_2026-04-25_01",
  "action_layout_compat": "open_x_embodiment_7dof",
  "source": {
    "capture_id": "unity_session_abc123",
    "captured_at": "2026-04-25T10:12:03Z",
    "demonstrator": "hand_vr"
  },
  "frames": {
    "world": "robot_base",
    "objects": {
      "target_part": { "parent": "world", "pose": null }
    },
    "tool_offset_m": [0.0, 0.0, 0.0]
  },
  "defaults": {
    "speed_pct": 50,
    "accel_pct": 50,
    "blend": "smooth"
  },
  "steps": [
    {
      "kind": "move",
      "motion": "linear",
      "pose_frame": "world",
      "pose": {
        "position_m": [0.30, 0.00, 0.20],
        "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]
      },
      "orientation_intent": "yaw_only",
      "orientation_tolerance_rad": [3.14159, 3.14159, 0.05],
      "joint_hint_rad": [0.0, 0.4, -0.6, 0.0],
      "speed_pct": 40,
      "blend": "smooth",
      "tolerance_m": 0.002
    },
    {
      "kind": "tool",
      "tool": "vacuum",
      "action": "on",
      "settle_ms": 150
    },
    {
      "kind": "move",
      "motion": "linear",
      "pose_frame": "target_part",
      "pose": { "position_m": [0.0, 0.0, 0.05], "orientation_quat_xyzw": [0,0,0,1] },
      "orientation_intent": "exact",
      "joint_hint_rad": [0.0, 0.3, -0.5, 0.0]
    },
    {
      "kind": "wait",
      "duration_ms": 200
    }
  ],
  "annotations": {
    "skill_hint": null,
    "task_label": "pick_from_bin"
  }
}
```

**Frame model.** `frames.world` names the IR's reference frame
(default `robot_base`). `frames.objects.<name>` declares a named
object/task frame whose pose is `null` at capture time and gets bound
at replay (operator pick, perception, fixture probe). Each `move` step
specifies `pose_frame`; default is `"world"`. Object-frame support is
the TP-GMM-shaped lever for "teach once, replay when the part moves"
without re-teaching. Adapters resolve object frames before IK.

**`action_layout_compat`** is a non-binding annotation declaring that
`move` step pose + adjacent `tool` step gripper state align with the
Open X-Embodiment / RT-X 7-DoF EE action layout (Δx, Δy, Δz, roll,
pitch, yaw, gripper). Free interop upside for future LeRobot/OXE
ingestion; no runtime cost.

**Orientation intent (per step) — required.** A `move` step's `pose`
always carries a full 6-DoF target (so the IR is morphology-neutral),
but the operator's *intent* about orientation is rarely "exactly that
quaternion." Modeled directly on MoveIt's `OrientationConstraint`
(per-axis absolute tolerance, with `~pi` meaning "free axis"; see
`moveit_msgs/Constraints`):

```text
orientation_intent: REQUIRED ∈ {
  "exact",            # tight tolerance on all 3 axes
  "yaw_only",         # tool-Z yaw matters; roll/pitch free (SCARA-friendly)
  "tool_axis_align",  # only the tool-axis direction matters; roll about tool free
  "free",             # any reachable orientation OK
}
orientation_tolerance_rad: OPTIONAL [rx, ry, rz]  # absolute per-axis tol; overrides intent default
```

`orientation_intent` is **required on every `move` step**. Schema
validation rejects any `move` step that omits it. Reason: silent
defaulting is exactly the SCARA-projection failure mode R12 is meant
to prevent. The author (lifter or human) always knows which intent
to write.

`orientation_tolerance_rad` stays optional; if omitted the adapter
uses the per-intent defaults below.

Default tolerance per intent (override only when needed):

| intent | rx | ry | rz |
|---|---|---|---|
| `exact` | 0.05 | 0.05 | 0.05 |
| `yaw_only` | π | π | 0.05 |
| `tool_axis_align` | 0.05 | 0.05 | π |
| `free` | π | π | π |

Why per-step, not just per-profile: the same teaching session can
contain a precise insertion step (intent=`exact`) followed by a
linear-approach step (intent=`tool_axis_align`); collapsing to a
single profile-level flag would force the adapter to be conservative
on everything. Per-step intent is what MoveIt and CRCL both expose,
and it is how the SCARA roll/pitch question stops being silent.

This field also resolves the R12 mitigation cleanly: `YAW_ONLY_SCARA`
adapter accepts steps where the per-step intent (after merging
defaults + profile authority) leaves rx/ry tolerances >= roughly π/2;
otherwise raises `UnsupportedStep`.

Lifter intent-defaulting rule:

- if the source kinematics provider declares
  `orientation_authority = YAW_ONLY_SCARA` → write `yaw_only`;
- if `FULL_6DOF` and the captured roll/pitch was within the wrist's
  natural rest band (operator did not actively rotate the tool) →
  write `tool_axis_align`;
- otherwise → write `exact`.

The lifter never writes "I don't know"; the field is always present.

Step kinds in v0.1: `move`, `tool`, `wait`, `set_frame`. Everything
else is deferred. Unknown kinds MUST cause the adapter to refuse the
program, not silently skip.

### 4.4 Robot capability profile schema (v0.1)

Only the fields that actually gate an adaptation decision. No
vaporware.

The profile is **additive** over URDF, SRDF, and `ros2_control`. It
references those rather than duplicating them. URDF supplies the
kinematic tree and limits; SRDF supplies the planning group and
collision pairs; `ros2_control` supplies the per-joint command/state
interface vocabulary. The profile only adds what those layers do not
cover: execution-mode support, queue semantics, tool model,
orientation authority, timing contract.

```python
class OrientationAuthority(StrEnum):
    FULL_6DOF       = "full_6dof"        # any reachable orientation
    YAW_ONLY_SCARA  = "yaw_only_scara"   # SCARA: tool yaw free, roll/pitch fixed
    PLANAR_XY_RZ    = "planar_xy_rz"     # planar manipulators
    CUSTOM          = "custom"           # see orientation_axis_mask

@dataclass(frozen=True)
class MotionSupport:
    joint: bool
    linear: bool
    circular: bool
    spline: bool
    servo_stream: bool          # continuous setpoints at >= 100 Hz
    blending: Literal["none", "radius", "cp_percent"]

@dataclass(frozen=True)
class ExecutionSupport:
    offline_program: bool       # upload full program, run standalone
    queued: bool                # host-paced queue, e.g. MG400
    supervised_playback: bool   # host watches, steps through
    trajectory_action: bool     # ROS2 control_msgs/FollowJointTrajectory
    streaming: bool             # realtime servo

@dataclass(frozen=True)
class ToolModel:
    kind: Literal["none", "vacuum", "gripper_binary", "gripper_analog"]
    io_port: Optional[int]      # adapter-specific meaning

@dataclass(frozen=True)
class RobotCapabilityProfile:
    robot_id: str                       # "mg400", "ur5e", ...
    dof: int
    joint_names: list[str]
    joint_limits_rad: list[tuple[float, float]]

    # Kinematics contract. Required for every adapter that ever runs IK
    # (i.e., everything except OFFLINE_EXPORT-only adapters that emit
    # vendor language directly). Reference URDF/SRDF/ros2_control where
    # available; do not duplicate them.
    kinematics: "KinematicsContract"
    ros2_control_command_interfaces: list[Literal["position", "velocity", "effort"]]
    ros2_control_state_interfaces:   list[Literal["position", "velocity", "effort"]]

    base_frame: str
    tcp_frame: str

    motion: MotionSupport
    execution: ExecutionSupport
    tool: ToolModel

    orientation_authority: OrientationAuthority
    orientation_axis_mask: Optional[tuple[bool, bool, bool]]  # required iff CUSTOM, (roll, pitch, yaw)

    max_tcp_speed_m_s: float
    max_joint_speed_rad_s: list[float]
    timing_contract: Literal["queued", "hard_realtime", "best_effort"]
```

**KinematicsContract.** Bluntly requiring `urdf_path: str` was wrong:
`ros2_control` itself accepts a minimal `<ros2_control>`-only URDF
with no geometry, and offline-program adapters (ABB RAPID, KUKA KRL,
Fanuc TP) often work without a ROS-shaped URDF at all. The contract
is what the adapter needs, not which file format ships it:

```python
class KinematicsKind(StrEnum):
    URDF        = "urdf"          # standard ROS2 description; preferred
    NAMED       = "named"         # built-in provider (e.g., "mg400_4axis_fk")
    EXTERNAL    = "external"      # pluggable IK service (RelaxedIK, TracIK, vendor)
    NONE        = "none"          # OFFLINE_EXPORT-only adapter; no IK ever runs

@dataclass(frozen=True)
class KinematicsContract:
    kind: KinematicsKind
    urdf_path: Optional[str]            # required iff kind == URDF
    srdf_path: Optional[str]            # optional even when URDF is present
    planning_group: Optional[str]       # SRDF group name if applicable
    provider_id: Optional[str]          # e.g. "mg400_4axis_fk" iff kind == NAMED
    fk_only: bool = False               # provider supplies FK but not IK (lifter-side use)
```

Validation rule (lives in `teaching_core.capability.profile`):

- `kind == URDF` requires `urdf_path` to exist.
- `kind == NAMED` requires `provider_id` to resolve in
  `teaching_core.kinematics.registry`.
- `kind == NONE` is only valid if the profile's `ExecutionSupport` is
  `offline_program=True` and every other mode is False.
- `kind == EXTERNAL` carries vendor-specific config in a separate
  `external_config: dict` (kept opaque to core).

This change is the **direct fix for finding C**. It keeps the IR
Cartesian-anchored, but stops blocking adapters whose kinematics live
outside a ROS URDF file. The contract test suite still requires a
runnable kinematics provider for any adapter declaring any non-offline
mode.

Adapters in this repo today:

| Adapter | Kinematics kind | Notes |
|---|---|---|
| MG400 (now) | NAMED, `provider_id="mg400_4axis_fk"` | Stock 4-axis FK from `Dobot_TCP_IP_Python_V4`; community URDF can be added later as URDF kind. |
| MG400 (later) | URDF + `mg400.urdf` | Once a vetted URDF lands; same profile, swap kind. |
| UR / Franka | URDF, with SRDF + planning_group | MoveIt path. |
| ABB RAPID export-only | NONE | Adapter only emits `.mod` files; no IK ever runs in this repo. |
| Fake / sim | URDF (UR5, MG400) | CI runs Cartesian programs through real URDFs. |

**Orientation authority** is a correctness gate. SCARA / 4-DOF arms
(MG400, M1 Pro) declare `YAW_ONLY_SCARA`; the translator projects the
IR's full-6-DoF target onto the feasible subspace (preserve yaw,
discard roll/pitch) **only when this field permits it**. A
`FULL_6DOF` adapter that receives a yaw-only program is fine; a
`YAW_ONLY_SCARA` adapter that receives roll/pitch intent the operator
genuinely meant must raise `UnsupportedStep` with a clear reason
rather than silently flatten it. See R12.

The profile is **loaded once at adapter init**. It is read-only to the
core. Any runtime state (current queue depth, error codes) belongs to
the executor, not the profile.

### 4.5 Adapter contract (v0.1)

```python
class RobotAdapter(Protocol):
    profile: RobotCapabilityProfile

    def plan(
        self,
        program: CanonicalProgram,
        mode: ExecutionMode,
    ) -> AdaptedPlan:
        """Translate + validate. Raises UnsupportedStep if capability mismatch."""

    def execute(
        self,
        plan: AdaptedPlan,
        *,
        cancel: CancelToken,
        progress: Optional[ProgressSink] = None,
    ) -> ExecutionResult:
        """Run the plan under the given execution mode. Must be cancellable."""
```

`AdaptedPlan` is whatever the adapter wants internally — opaque to
the core. For MG400 it is the pre-formatted TCP command strings plus
pacing metadata. For `FollowJointTrajectory` robots it is a
`trajectory_msgs/JointTrajectory`. For offline robots it is a file
bundle.

The protocol deliberately does not expose *how* motion is paced.
That keeps MG400's queue-awareness out of the robot-neutral API.

**Reference adapter implementations.** The Protocol is intentionally
opaque, but the implementation pattern for each robot family is not.
Pick the closest reference when adding an adapter:

| Robot family | Reference plan stack | Reference exec stack |
|---|---|---|
| Dobot MG400 / M1 Pro (queued TCP) | this repo's `MotionPlanner.format_command` | this repo's queue-aware `TeleopController` + `CommandSender` |
| UR (ROS2-native) | MoveIt Task Constructor | `controller_manager` switching between `scaled_joint_trajectory_controller` (default) and `forward_position_controller` (+ MoveIt Servo for `STREAMING`) |
| Franka Panda (ROS2-native) | MoveIt Task Constructor | `joint_trajectory_controller` (+ libfranka RT loop for `STREAMING`) |
| Simulated / CI | any (or none) | `ros2_control` `mock_components/GenericSystem` with `calculate_dynamics` |
| Offline-program (ABB / KUKA / Fanuc) | postprocessor → vendor language (RAPID / KRL / TP) | vendor IDE upload; `OFFLINE_EXPORT` mode only |

Where MoveIt Task Constructor is named, the IR's `move`/`tool`/`wait`
steps map onto MTC `Generator` / `Connector` / `Propagator` stages
inside a `SerialContainer`. That is an implementation choice inside
the adapter; the IR does not depend on MTC.

### 4.6 Execution mode model

```python
class ExecutionMode(StrEnum):
    OFFLINE_EXPORT       = "offline_export"        # emit file, do not run
    SUPERVISED_PLAYBACK  = "supervised_playback"   # step + confirm
    QUEUED               = "queued"                # host-paced queue (MG400)
    TRAJECTORY_ACTION    = "trajectory_action"     # ROS2 FollowJointTrajectory
    STREAMING            = "streaming"             # realtime servo (Servo / RTDE / libfranka)
```

`TRAJECTORY_ACTION` is the ROS2 industry default and is semantically
distinct from both `QUEUED` (host-paced, no controller-side trajectory
object) and `STREAMING` (continuous setpoints, no terminal goal).
ROS2-native arms (UR, Franka, xArm, Kinova) declare it and treat it
as their default whole-program execution mode.

Selection rule (deterministic, lives in `teaching_core.planner`):

```text
requested_mode
  └─ if profile.execution does not support it → error out early.
  └─ else adapter returns a plan compiled for that mode.

if caller requests "default":
  └─ if profile supports TRAJECTORY_ACTION  → pick TRAJECTORY_ACTION
  └─ elif profile supports QUEUED           → pick QUEUED
  └─ elif profile supports OFFLINE_EXPORT   → pick OFFLINE_EXPORT
  └─ else                                   → error.
STREAMING is opt-in only, never the default.
```

This mirrors the UR ROS2 driver convention:
`scaled_joint_trajectory_controller` is the default,
`forward_position_controller` (streaming) is opt-in. Live teach paths
explicitly request `STREAMING`; whole-program replay does not.

No "best guess" beyond that table. The caller picks the mode (or
"default"); the adapter either runs it or refuses with a structured
reason. That is how different robots diverge cleanly.

MG400's current realtime path maps to `STREAMING`-on-top-of-`QUEUED`:
the adapter declares `queued=True, streaming=True,
trajectory_action=False` with `timing_contract="queued"`. UI and CLI
labels for MG400's `STREAMING` mode read "streaming-through-queue
(best-effort)" rather than "realtime." That is the honest description;
do not pretend otherwise. See R13.

---

## 5. Validation Strategy (the ladder)

No rung is optional. Each runs in CI without hardware.

**Rung 0 — Two-tier artifact model.** Two distinct file kinds; one
is *not* a replacement for the other:

| Tier | File kind | Purpose | Format |
|---|---|---|---|
| Raw demo | `*.session.mcap` | the high-rate Unity stream as-recorded; ROS-ecosystem default since Iron (`ros2 bag` writes MCAP by default; `mcap.dev`) | **MCAP** |
| Canonical program | `*.program.json` | the lifted, robot-neutral taught program — IR v0.1 from §4.3 | **JSON** (schema-validated) |

The lifter is the *only* component that reads `*.session.mcap` and
writes `*.program.json`. Adapters never see raw streams; they see
canonical programs only. This keeps R8 ("capture and canonical
formats merge") tractable.

Migration of today's `TrajectoryRecorder` JSON: that artifact is
neither a true raw stream (it is sample-decimated at ~8–10 Hz with
delta gating) nor a true canonical program (it has no step kinds, no
frames, no orientation intent). It belongs in the **adapter-internal
playback cache** for MG400, not at either tier of the new ladder.
Phase M3 keeps the existing `.json` trajectory format working as an
MG400-only artifact behind the adapter; it does not become the IR.

LeRobot Parquet+MP4 is a third tier (ML-training format) and is
deliberately out of scope for v0.1. A small `mcap → lerobot` exporter
can be added later without touching the IR.

**Rung 1 — IR schema tests.** JSON Schema for the canonical program;
round-trip a fixture set; reject malformed programs. Pure data.
Standard `unittest`.

**Rung 2 — Lifter contract tests.** Given a fixed synthetic MCAP
session, assert the lifter produces a known canonical program.
Segmentation stays testable.

**Rung 3 — Adapter translator golden tests.** For each adapter, a
set of `(canonical_program, expected_native_commands)` pairs. For
MG400 this is `(program, list_of_TCP_strings)`. Pure function, no
sockets. This is where most adapter bugs get caught cheaply, and the
real correctness gate the `MG400_Mock` cannot give us.

**Rung 4 — Adapter executor against a fake backend.** Pure-Python
`FakeBackend` implementing the `RobotAdapter` contract (no TCP, no
firmware) with deterministic Euler-forward state propagation
mirroring `ros2_control` `mock_components/GenericSystem`'s
`calculate_dynamics` behavior. Records commands and simulates queue
drain / feedback. Generalizes today's `test_queue_aware_logic.py`.

**Rung 5 — Adapter executor against vendor TCP mock.** For MG400 use
the `MG400_Mock` submodule (limited — no `RunQueuedCmd`, ignores
`CP`; only trustworthy for socket-level plumbing as
`docs/teleop_command_logic.md` warns). The analogous tool for UR is
**URSim** (Universal Robots' Docker image), which is far higher
fidelity and runs in CI. Per-vendor — name the tool in the adapter's
test config.

**Rung 6 — Kinematics/physics sim.** **PyBullet** + URDF as the
first-line option (lightweight, no ROS build). Validate that the
canonical program, when adapted to a given URDF, produces reachable
motion with no joint limit violations. Robot-agnostic; the same
harness drives MG400 URDF, UR5 URDF, Panda URDF. **The test matrix
must include at least one URDF with a different DOF count from MG400
from day one** (UR5 is the pragmatic choice — community URDF stable,
no license key). See R1.

**Rung 7 — ROS2 `ros2_control` mock_components.** For any adapter
that goes via `TRAJECTORY_ACTION` or `STREAMING`, run through
`mock_components/GenericSystem` (with `calculate_dynamics=true`)
wired up via `launch_testing`. Assert that
`FollowJointTrajectory` goals complete and that command/state
interface vocabularies match the profile. This is how UR / Panda
adapters get validated without hardware.

**Rung 8 — Gazebo Harmonic / hardware.** Only after 1–7 pass. Keep
manual.

The chosen architecture is not considered complete until rungs 0–4
are wired into CI and rung 6 works for MG400 URDF with at least one
non-MG400 URDF in the test matrix.

---

## 6. Migration Plan

The constraint is "don't rewrite everything." So: grow the new core
next to the old code, move files in, then retire the duplicate.

**Migration revision history.** v1.1 had a separate "M4 lifter v0
with temporary MG400 FK" + "M5 decouple FK." That was wrong: it baked
MG400 worldview into the first lifter milestone and then asked a
later phase to undo it. v1.2 collapses those into a single milestone
where the **`KinematicsProvider` seam exists from PR 1**, with MG400
as the first concrete provider. No "temporary then fix" step.

### Phase M1 — Introduce the new package + kinematics seam

- create `src/robot_teaching_core/` as a ROS2 ament_python package;
- define `program/schema.py`, `program/types.py`, `capability/profile.py`,
  `capability/kinematics.py` (the `KinematicsContract` + `KinematicsProvider`
  Protocol from §4.4), `adapter_api/base.py`, `validation/golden.py`,
  `kinematics/registry.py`;
- ship `kinematics/providers/null.py` (FK/IK both raise) and
  `kinematics/providers/urdf.py` (URDF-backed, uses `pinocchio` or
  `KDL` — pick at scoping) so the seam is real on day one;
- no behavior change to MG400 runtime yet.

Deliverables: package builds, schema round-trip tests pass, registry
resolves a stub provider. 1 PR.

### Phase M2 — Extract robot-neutral pieces from MG400

Move out of `mg400_controller/common/` into `teaching_core/`:

- `logic/target_compensator.py` → `teaching_core/capture/latency.py`
- `utils/clock_calibrator.py` → `teaching_core/capture/clock.py`
- `logic/joint_validator.py` → split: generic clamp math into
  `teaching_core/program/validation.py`; MG400 limits stay as a
  profile constant in `adapters/mg400/`.
- existing `TrajectoryRecorder` JSON IO (the ~8–10 Hz delta-gated
  artifact today's MG400 runtime writes) → **does not** move to core.
  Reclassified as MG400-internal playback cache (see §5 Rung 0);
  stays in `adapters/mg400/cache/`. New raw captures use
  `*.session.mcap` instead.

Behavior change: zero (wrappers keep old imports working for one
release).

### Phase M3 — First adapter: wrap MG400 as `adapters/mg400/`

- rename package dir, not contents initially;
- implement `profile.py` returning the MG400 `RobotCapabilityProfile`
  with `kinematics.kind = NAMED, provider_id = "mg400_4axis_fk"`;
- implement `kinematics/providers/mg400_4axis.py` and register it —
  this is MG400's concrete `KinematicsProvider`;
- implement `translator.py` backed by the existing
  `MotionPlanner.format_command`;
- implement `executor.py` backed by the existing `TeleopController`
  + `CommandSender` pair — the queue-aware logic stays exactly where
  it is, just behind the adapter contract.

Deliverables: `MG400Adapter().execute(program, mode=QUEUED)` works
end-to-end. Rung 3 + Rung 4 tests pass.

### Phase M4 — Lifter v0 (no MG400 FK bias)

- the lifter consumes `*.session.mcap` and emits canonical
  `*.program.json`;
- segmentation uses dwell/velocity thresholds — robot-neutral math;
- Cartesian poses come from an **injected** `KinematicsProvider`,
  resolved by the caller (CLI flag `--kinematics mg400_4axis_fk` or
  `--kinematics urdf:path/to/ur5.urdf`);
- the lifter package is forbidden from importing
  `adapters.*` (enforced by a small import-linter test);
- ship CLI: `python -m teaching_core.lifter --kinematics ... session.mcap > program.json`.

The lifter's first concrete run uses the MG400 provider to make a
real demo work, but **the lifter never references MG400 directly**.
Swapping `--kinematics ur5.urdf` (with synthetic UR5 session data)
must produce a valid program from day one — that is the M4 acceptance
test.

### Phase M5 — Proof of portability (was M6)

Add `adapters/fake/` (in-memory) and either:

- `adapters/ur/` via `ros2_control` + UR ROS2 driver
  (`TRAJECTORY_ACTION` mode), or
- `adapters/pybullet/` for a sim-first demonstration.

Run the same canonical program through MG400 adapter + fake adapter +
second adapter. If it replays on a second robot (even a sim one)
without editing the program, **the objective is met**. Until then,
the next phase is not done.

### Phase M6 — BT-over-skills v0.2 (was M7)

After M5 lands and a second adapter is real, layer Behavior Trees
above the canonical IR.

- compose named skills (`pick`, `place`, `linear_approach`, etc.)
  whose leaves are v0.1 `move` / `tool` / `wait` / `set_frame` steps;
- BT engine: **BehaviorTree.CPP** (Nav2 precedent, mature, Groot2
  introspection); `py_trees_ros` is the Python alternative if
  build-tooling pressure pushes that way — defer the call to M7
  scoping;
- BT files are XML; they reference skills by name and bind step ids
  / object frames as blackboard entries;
- **the v0.1 IR is not modified.** BT composition lives **above**
  `steps`, by grouping step ids into named skills that the BT engine
  invokes through a thin "skill leaf" adapter. Existing canonical
  programs replay unchanged.

This phase exists in writing so that the v0.1 design's forward
compatibility is testable: if a v0.1 program cannot be wrapped as a
single trivial BT skill leaf, M1–M6 are wrong somewhere.

### What is explicitly not moved

- `monitor_gui.py` and everything under `common/monitor/` — this is
  MG400 operator telemetry. Stays in `adapters/mg400/`.
- queue-aware gating logic — stays in `adapters/mg400/executor.py`.
- Dobot TCP packet parsing — stays in `adapters/mg400/` (it is the
  definition of the MG400 adapter).

---

## 7. Weakness Audit / Risk Register

Honest list. If these are not addressed, the architecture will rot.

| # | Risk | Why it hurts | Mitigation |
|---|---|---|---|
| R1 | Canonical IR turns into "MG400 commands in JSON" | kills retargeting, we built nothing | Cartesian pose is primary; `joint_hint` is hint only; **rung 6 test matrix MUST include at least one URDF with different DOF count from MG400 from day one** (UR5 community URDF). Lifter package may not import from `adapters/*`. |
| R2 | Lifter depends on MG400 FK | ties "robot-neutral" core to one robot | M5 decouples FK via pluggable kinematics provider; enforce by making the lifter package not import from `adapters/*` |
| R3 | Capability profile grows into a god-struct | every new robot adds fields, nothing gets removed | profile is versioned; fields must gate a real adaptation decision in code or they do not land; review checklist in `docs/skills/multi_robot_architecture/SKILL.md` |
| R4 | `ExecutionMode` values are aspirational on MG400 | users think `STREAMING` means hard-realtime; it does not | `timing_contract` on the profile is displayed in all user-facing mode selection; docs spell out MG400's "streaming-through-queue" honestly |
| R5 | The mock is wrong in known ways | false-green CI | rung 3 (golden translator tests) is the real correctness gate; the mock is only trusted for rung 5 (socket plumbing) |
| R6 | Second-adapter work keeps getting deferred | architecture never proven | M6 gate: no further work on MG400 optimization until a second adapter (even `fake` + URDF-only sim) replays a real captured program |
| R7 | Retargeting fails silently on workspace mismatch | robot A reaches, robot B cannot, adapter quietly clips | planner runs an IK/reachability pass at adaptation time and surfaces an `UnreachableStep` error; partial execution is never the default |
| R8 | Capture format and canonical format merge over time | muddies the "raw demo vs taught program" distinction | keep `*.session.mcap` (raw, MCAP) and `*.program.json` (canonical, JSON) as separate file kinds with separate schemas; lifter is the only thing that reads the first and writes the second; today's `TrajectoryRecorder` JSON is **neither** tier and lives only as MG400-internal playback cache (see §5 Rung 0) |
| R9 | Over-investment in behavior trees / Option C early | eats calendar with no objective payoff | explicitly out of scope for this phase; revisit after M6 lands |
| R10 | ROS2 build env not available locally | M3–M6 pass unit tests but break the ROS package | every PR that touches package layout runs in a containerized `colcon build` before merge; the `controller_continuation_guide.md` already flags this environment risk |
| R11 | We re-invent CRCL / `robot_movement_interface` and stall the same way | a grand neutral motion vocabulary without a real first user is exactly how those projects ended in `ros-industrial-attic` | stay thin: every v0.1 IR field and every `RobotCapabilityProfile` field must gate a real adaptation decision in code, or it does not land. MG400 is the anchoring real first adapter. Review checklist enforces this. |
| R12 | SCARA / 4-DOF orientation projection silently loses tool roll/pitch intent | operator teaches a 6-DoF pose, MG400 flattens to yaw-only, replay looks subtly wrong | Two layers cooperate: (a) **per-step `orientation_intent`** in IR (§4.3) declares whether the step needs `exact` / `yaw_only` / `tool_axis_align` / `free`; (b) profile's `orientation_authority` (§4.4) declares what the robot can actually deliver. Translator merges the two: if a step requests tighter tolerance than the profile can satisfy, `UnsupportedStep` is raised. Lifter writes `yaw_only` by default for SCARA-captured sessions, `exact` for 6-DoF captures. |
| R13 | `STREAMING` mode semantics diverge silently between ROS2-native arms and MG400 | one user's `STREAMING` is hard-realtime servo, another's is queue-best-effort | `timing_contract` on the profile is surfaced verbatim in any UI/CLI mode selector; MG400's `STREAMING` label reads "streaming-through-queue (best-effort)"; profile-driven label generation is contract-tested. |
| R14 | Lifter quietly imports MG400 FK and re-bakes the bias M4 was supposed to fix | "robot-neutral" core in name only; M5/portability proof becomes theatre | M1 ships `KinematicsProvider` Protocol + URDF + Null implementations on day one; M4 lifter consumes provider via DI; **import-linter test** in CI fails the build if `teaching_core/lifter/` imports from `adapters.*` or from `mg400_*`. M4 acceptance includes running the lifter against a UR5 URDF synthetic session. |
| R15 | Adapters silently ignore `orientation_intent` and use the raw quaternion | per-step intent looks like docs theatre; SCARA still projects without warning | Two layers: (1) **schema requires `orientation_intent` on every `move` step** — JSON Schema validation rejects programs that omit it, no silent defaulting; (2) translator contract: every `move` step's effective `[orientation_intent, orientation_tolerance_rad]` must be either honored exactly or rejected with `UnsupportedStep`. Golden tests for MG400 include yaw-only steps, full-6DoF steps, and one over-tight step that the adapter must refuse. |

---

## 8. How a New Robot Gets Added (acceptance test for the design)

If this answer is not concrete, the design has failed. So:

To add robot `X`:

1. Write `adapters/X/profile.py` returning a `RobotCapabilityProfile`.
2. Write `adapters/X/translator.py`: one function per supported
   canonical step kind. Unsupported kinds raise `UnsupportedStep`.
3. Write `adapters/X/executor.py` implementing the `RobotAdapter`
   protocol. Pick one execution mode to start. **If `X` is
   ROS2-native, the normal implementation is MoveIt Task Constructor
   for planning + `controller_manager` switching for execution-mode
   selection** (see §4.5 reference table); custom TCP code is the
   exception, not the rule.
4. Add a golden test file under `adapters/X/test/golden/` pairing
   the repo's shared canonical programs with expected native output
   for that robot.
5. Run the shared rung 6 sim harness against robot X's URDF to
   confirm reachability on the canonical test set.

No core changes required. If step 1–5 cannot be done without editing
`robot_teaching_core/`, that is a design bug and goes in the risk
register.

---

## 9. Immediate Next Actions (if going to implementation)

In order, each is a small PR:

1. **Freeze the canonical IR schema** (JSON Schema file + Python
   dataclasses + round-trip tests). `orientation_intent` and
   `pose_frame` are **required** fields on every `move` step; schema
   validation rejects programs that omit them. No adapter changes.
   ~1 day.
2. **Land `KinematicsContract` + `KinematicsProvider` Protocol +
   stub URDF/Null providers + registry.** Required before profiles
   compile. ~1 day.
3. **Land `RobotCapabilityProfile` dataclass + MG400 profile instance**
   (with `kinematics.kind = NAMED, provider_id = "mg400_4axis_fk"`).
   Read-only, no behavior change. ~0.5 day.
4. **Define `RobotAdapter` protocol + write a `FakeAdapter` + 3
   golden tests** including a yaw-only step and an over-tight step
   the adapter must refuse. ~1 day.
5. **Introduce `adapters/mg400/` and re-export existing modules
   from it** (leave legacy imports as wrappers). Land
   `mg400_4axis_fk` provider here. Zero behavior change. ~1 day.
6. **Lifter v0 with provider injection**: capture → lifter (via
   injected provider) → canonical program → MG400 adapter →
   playback. Same lifter binary must produce a valid program from a
   synthetic UR5 URDF session. ~3–4 days.
7. **Second adapter scaffold** (`fake` or `ur`), same canonical
   program, run in sim. ~2–3 days.

After (7), the next phase's stated goal — "teach once, replay
elsewhere" — exists in the repo. Before (7), it does not.

---

## 10. Open Questions (deliberately unresolved)

These need a call, not a guess.

- **Pose orientation representation**: quaternion (as in the schema
  above) or axis-angle? Quaternion is more standard but slightly
  less readable in JSON. Low stakes; pick one and stick with it.
- **Tool model generality**: vacuum + binary gripper + analog gripper
  is enough for v0.1. Force/torque finger control, suction-group
  patterns, and dual-arm tools are deferred.
- **Kinematics library**: `pinocchio` vs `KDL` vs `python-fcl` for
  the URDF-backed `KinematicsProvider`. `pinocchio` is the modern
  choice (Drake-grade, used by MoveIt 2 internally for several
  components); `KDL` is the legacy default. Decide at M1 scoping;
  the `KinematicsProvider` Protocol does not depend on the choice.
- **MCAP schema for raw sessions**: which message types go into the
  bag — raw `sensor_msgs/JointState`, custom `unity_hand_pose` msg,
  or both. Pick when the first lifter PR lands; non-breaking either
  way as long as the lifter knows what to read.
- **Program editing UI**: out of scope. The canonical JSON is the
  program; any UI is a separate tool that reads/writes the schema.

**Closed (no longer open as of v1.2):**
- ~~Frame handling for multi-part workspaces~~ — resolved in §4.3
  (`frames.objects.*` + per-step `pose_frame`).
- ~~Per-step orientation tolerance vs profile-only~~ — resolved in
  §4.3 (`orientation_intent` + `orientation_tolerance_rad`).
- ~~URDF required vs optional~~ — resolved in §4.4
  (`KinematicsContract` with four kinds).
- ~~Whether lifter should own MG400 FK temporarily~~ — resolved in
  §6 M1+M4 (provider seam from PR 1; lifter never imports adapters).

---

## 11. Pointer Index (where things live / will live)

- project objective: `docs/project_objective.md`
- refactor phase closed: `docs/current_refactor_status.md`
- phase brief: `docs/ai_handoffs/next_phase_architecture_brief.md`
- this proposal: `docs/ai_handoffs/next_phase_architecture_proposal.md`
- research-pass audit trail: `docs/ai_handoffs/next_phase_architecture_revision.md`
- skill spec: `docs/skills/multi_robot_architecture/SKILL.md`
- MG400 runtime continuation: `docs/controller_continuation_guide.md`
- MG400 command logic detail: `docs/teleop_command_logic.md`

Update rule: any decision that changes §4 (chosen architecture) or
§6 (migration plan) must edit this file in the same PR. Do not let
it rot.
