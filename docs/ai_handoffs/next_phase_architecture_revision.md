# Next-Phase Architecture Revision (Audit Log)

> **NOT THE SOURCE OF TRUTH.** The canonical architecture is
> `docs/ai_handoffs/next_phase_architecture_proposal.md`. This file is
> the **audit log only** — it records research findings, pressure-pass
> deltas, and the reasoning behind specific edits to the proposal.
> If this file ever conflicts with the proposal, the proposal wins.
> Never implement from this file; implement from the proposal.

Status: audit log accompanying the proposal. Currently covers three
pressure passes: v1.0 → v1.1 (research findings, Parts 1–5),
v1.1 → v1.2 (second pressure-pass, Part 6), and v1.2 → v1.3
(VR-to-robot calibration + delta-robot coverage, Part 7).

Purpose: pressure-test every significant claim in the proposal against
published work, vendor docs, and the ROS2 ecosystem. Say what holds,
what should change, what should be cut, what must be added — then
edit the proposal in place.

Method note: research performed in this session via live web access
(WebSearch + WebFetch) against primary sources. URL citations are
inline. Background research sub-agents without web access returned a
recall-based report that is merged where it cites known canonical
sources; everything load-bearing here is either freshly fetched or
cross-referenced against a live result from this session.

---

## Part 1 — Research Findings

### 1.1 Learning from Demonstration / Programming by Demonstration

- Argall et al. (2009) *A survey of robot learning from demonstration*
  introduces the **correspondence problem** (teacher embodiment ≠
  learner embodiment), which is the formal name for the MG400 ↔ UR ↔
  Panda portability question. DOI 10.1016/j.robot.2008.10.024.
- Ravichandar, Polydoros, Chernova, Billard (2020) *Recent Advances in
  Robot Learning from Demonstration*, Annual Review of Control,
  Robotics, and Autonomous Systems — splits LfD into
  **trajectory-level** (DMP, ProMP, GMM-GMR, KMP) vs **task/skill-level**
  (options, skill trees, symbolic). The survey is explicit: trajectory
  reps are data-efficient and safe but weak at compositional
  generalization; skill-level reps generalize but need priors.
  <https://www.annualreviews.org/doi/10.1146/annurev-control-100819-063206>
- Kroemer, Niekum, Konidaris (2021) *A Review of Robot Learning for
  Manipulation*, JMLR — defines a skill as a closed-loop policy with
  **preconditions, termination conditions, effects**; argues manipulation
  needs hybrid continuous+discrete representations because contact
  events segment the world.
  <https://www.jmlr.org/papers/v22/19-804.html>
- Task-parameterized representations (Calinon 2016, TP-GMM) generalize
  demonstrations by **attaching reference frames to objects**, then
  conditioning the trajectory on those frames. This is the mechanism
  that makes "teach once, replay when the object moves" work.
  DOI 10.1007/s11370-015-0187-9.

**Implication.** Canonical IR must carry **object/task frames**, not
just a base-frame pose list. Without that, re-teaching is required
every time the object moves, independently of the robot question.
The original proposal v0.1 schema only had `frames.world = robot_base`
and `frames.tool_offset_m`. This is a gap.

### 1.2 Cross-embodiment capture — what the community actually stores

Strong convergence in 2023–2026 open-source practice:

- **Open X-Embodiment / RT-X** (Google DeepMind et al., 2023+). 1M+
  trajectories across **22 embodiments**. The dataset converts every
  action into a **7-DoF end-effector action space (Δx, Δy, Δz, roll,
  pitch, yaw, gripper)**. arXiv 2310.08864 / paper site
  <https://robotics-transformer-x.github.io/>.
- **DROID** (Khazatsky et al., 2024). 76k trajectories across 13
  institutions, stored as EE pose + gripper + RGB.
  <https://droid-dataset.github.io/>.
- **LeRobot dataset** (HuggingFace 2024–25). Canonical fields:
  `observation.state` (1D concatenated state), `action` (1D
  concatenated action), `observation.images.*`, timestamps. Low-rate
  signals in Parquet, video in MP4. ROS bridging: `/joint_states`
  maps to `observation.state`, `/joint_cmd`-style topics map to
  `action`. <https://huggingface.co/docs/lerobot/en/lerobot-dataset-v3>.
- **AnyTeleop** (Qin et al., RSS 2023). General vision-based
  teleoperation system "to support multiple different arms, hands,
  realities, and camera configurations within a single system",
  explicitly built around **EE-pose retargeting** without per-robot
  learned models. <https://arxiv.org/abs/2307.04577>.

**Implication.** The proposal's choice to make Cartesian pose primary
and joint values a hint is **directly validated by the field**. The
specific 7-DoF (pose + gripper) action layout used by OXE/LeRobot is
the community interop vector. Aligning the canonical IR's `move` step
and `tool` step payloads with that layout is free upside: it means a
future learning pipeline can ingest our captured demos without
reformatting.

### 1.3 Task / skill representation (waypoint vs BT vs HSM vs MTC vs PDDL)

- **Behavior trees** — Colledanchise & Ögren (2018, arXiv 1709.00084);
  Iovino et al. (2022) *A survey of behavior trees in robotics and
  AI*, RAS 154, DOI 10.1016/j.robot.2022.104096. Key empirical result
  from a follow-up formal comparison (Biggar & Zamani, arXiv
  2405.16137, 2024): **adding a node to a BT is O(1); editing an FSM
  is O(n)**; BTs dominate on modularity and reactivity, FSMs only win
  on simplicity for very short tasks.
  <https://arxiv.org/html/2405.16137v1>.
- **Nav2 BT navigator** is the in-field precedent for BT-over-skills
  in the ROS 2 ecosystem. <https://docs.nav2.org/behavior_trees/overview/>.
- **MoveIt Task Constructor** (Görner et al. ICRA 2019) — hierarchical
  stage-based planner built on MoveIt. Stage types are Generator,
  Propagator, Connector; containers are SerialContainer and
  ParallelContainer; stages communicate via MoveIt's PlanningScene.
  Not a program-storage format — it is a **per-robot planning and
  execution pipeline**. <https://github.com/moveit/moveit_task_constructor>.
- **SMACH** / **FlexBE** — hierarchical state machine family; ROS 1
  heritage, still usable on ROS 2 but seeing less new adoption than
  BTs.
- **MoveIt Servo** — accepts `TwistStamped`, `JointJog`, or
  `PoseStamped` and streams velocity commands through the
  Jacobian pseudo-inverse with singularity / joint-limit / collision
  safeties. Exact target for a streaming execution mode in ROS2-native
  adapters.
  <https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html>.

**Implication.** The original proposal treated Option C (BT over
skills) as "next-next phase." The evidence is stronger than that: BTs
are the current ROS2 idiom for composing reactive skill sequences,
and a BT layer above the v0.1 waypoint program can be layered without
breaking IR compatibility. BT should move from "maybe-someday" to
"explicit v0.2, scheduled after M6." MTC is the right **adapter
implementation** pattern for any ROS2-native arm, not the program
format itself.

### 1.4 Capability modeling / hardware abstraction

- **URDF** covers kinematic tree, inertials, visual/collision meshes.
  Does not cover motion command vocabulary, queue semantics, IO
  groups, or tool types. URDF spec: <https://wiki.ros.org/urdf/XML>.
- **SRDF** adds planning groups, disable_collisions pairs, and named
  end-effectors — MoveIt's view of what is controllable together.
  <https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html>.
- **ros2_control hardware_interface** standardizes command/state
  interfaces per joint (`position`, `velocity`, `effort`). Controllers
  like `joint_trajectory_controller`,
  `scaled_joint_trajectory_controller`, `forward_position_controller`,
  and `forward_velocity_controller` consume those interfaces.
  <https://control.ros.org/>.
- **Universal Robots ROS2 driver**: ships both
  `scaled_joint_trajectory_controller` (safety-aware queued trajectory
  execution with speed-scaling) and `forward_position_controller`
  (streams target positions directly to `servoj`, the preferred mode
  for MoveIt Servo). Mode selection is by **runtime controller
  switching** via `controller_manager`.
  <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html>.
- **ROS-Industrial prior art** for cross-vendor neutral commands:
  `simple_message` protocol + `industrial_robot_client` for
  `JointTrajectoryPt`; CRCL ("Canonical Robot Command Language") for
  vendor-neutral motion; `robot_movement_interface` for Cartesian/joint
  move primitives. **All three are now attic** at ros-industrial-attic
  on GitHub. <https://github.com/ros-industrial-attic/robot_movement_interface>,
  <https://github.com/ros-industrial-attic/crcl>.

**Implication.** The capability-profile idea is not new; ROS-Industrial
tried it and the attic status means their specific schemas did not
reach critical mass. That is a cautionary signal, not a
disqualification: modern ROS2 practice has settled on **URDF + SRDF
+ ros2_control interfaces + controller switching** as the de-facto
capability layer for ROS2-native arms. Our capability profile should
**lean on these rather than duplicate them** — treat the profile as
"what the adapter needs that URDF/SRDF/ros2_control does not already
cover." Mostly: execution-mode support, queue semantics, tool model,
orientation authority (see 1.5).

### 1.5 SCARA / 4-DOF orientation authority

- SCARA (Selective Compliance Assembly Robot Arm) is RRPR or PRRR with
  a vertical prismatic axis and a yaw-only wrist. Axis 4 is wrist
  rotation around the tool Z. Roll and pitch of the tool are
  kinematically fixed by the SCARA geometry. Wikipedia: SCARA;
  industry consensus, see Hitbot Robot SCARA guide
  <https://www.hitbotrobot.com/scara-robots-for-automatic-pick-and-place/>.
- MG400 is a 4-DOF arm with MG400-specific TCP/IP command set
  (`JointMovJ`, `MovJ`, `MovL`) acting on `(J1, J2, J3, J4)` where J4
  is tool yaw. Dobot TCP/IP 4-Axis protocol:
  <https://github.com/Dobot-Arm/TCP-IP-Protocol-4AXis>,
  <https://github.com/Dobot-Arm/TCP-IP-4Axis-Python>.

**Implication.** A canonical IR that stores full 6-DoF pose will not
always be reachable on MG400. The adapter must either project the
target orientation onto the feasible subspace (preserve yaw, ignore
roll/pitch) or refuse the step. The profile must declare this
explicitly. The original proposal had a generic `MotionSupport` struct
but no **orientation-authority field**. This is a real gap and a
direct correctness risk for cross-robot replay.

### 1.6 Execution semantics across robot families

- **Queued TCP robots** (Dobot MG400, M1 Pro). Motion commands enter a
  firmware queue on port 30003. `Sync` blocks until the queue drains.
  `CP` sets blend aggressiveness. `RunQueuedCmd` and queue state are
  read from the 30004 feedback packet. Already well-understood in
  this repo.
- **Trajectory-action robots.** `control_msgs/FollowJointTrajectory`
  is the ROS 2 canonical action; virtually every ROS2-native driver
  (UR, Franka, xArm, Kinova) exposes some variant of
  `joint_trajectory_controller`.
- **Streaming / servo robots.** UR RTDE at up to 500 Hz (`servoj`,
  `speedj`), Franka libfranka at 1 kHz. In ROS 2, the de-facto
  streaming pipeline is **MoveIt Servo → forward_position_controller
  → driver RT loop**.
- **Offline program robots.** ABB RAPID (uploaded via RobotStudio or
  RWS), KUKA KRL (via KUKAVARPROXY or RSI), Fanuc TP (via
  Karel/FANUCAPI). Vendor-locked; the portable story is to emit a
  vendor-specific postprocessor from the canonical program, which is
  exactly what RoboDK, Delfoi, and OCTOPUZ do commercially.

**Implication.** The `ExecutionMode` enum in the original proposal
(`OFFLINE_EXPORT`, `SUPERVISED_PLAYBACK`, `QUEUED`, `STREAMING`) maps
cleanly onto what each family really supports. Keep it. Add
`TRAJECTORY_ACTION` as an explicit mode because it is the ROS 2
industry default, and it is semantically distinct from both `QUEUED`
(host-paced, no controller-side trajectory object) and `STREAMING`
(continuous setpoints). The UR driver model of **per-mode controller
selected via `controller_manager`** is a precedent the adapter
interface should mirror.

### 1.7 Validation tooling realistically available

- **ros2_control mock_components / GenericSystem**: in-process
  hardware mock that implements `SystemInterface`, mirrors commands
  to state, supports `calculate_dynamics` for Euler-forward state
  simulation, mimics URDF mimic joints. "You can test your
  controllers, broadcaster, launch files, and even integrations with,
  e.g., MoveIt" without hardware.
  <https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html>.
- **launch_testing** (ROS 2 test framework) is the standard wrapper
  for integration tests that bring up a ROS graph.
- **URSim** (UR vendor simulator, Docker image), **MG400_Mock** (in
  this repo, limited — does not emit `RunQueuedCmd`, ignores `CP`).
- **PyBullet + URDF** — the lingua franca of LfD research sims.
- **Drake / MuJoCo / Isaac Sim / Gazebo Harmonic** — heavier; only for
  end-to-end scene validation.
- **MCAP** is the default bag format from ROS 2 Iron onward
  (<https://mcap.dev/>). Storing raw demo captures as MCAP buys us
  zero-cost compatibility with every ROS 2 tool.

**Implication.** The original proposal's validation ladder is
directionally correct but understates how much of it is ready-made:
mock_components is effectively rung-7 out of the box; MCAP is
effectively rung-1 storage out of the box; URSim is effectively
rung-5 for UR out of the box. The ladder should name these
specifically.

### 1.8 Direct pressure points on the original proposal

| Claim in original | Evidence from research | Verdict |
|---|---|---|
| Option B (canonical waypoint program + capability-driven adapters) is the right core | OXE/DROID/LeRobot all adopt EE-pose IR; MoveIt Servo + UR driver model fit the adapter shape | **Confirmed** |
| Cartesian pose primary, joint hint secondary | 7-DoF EE action space is the cross-embodiment consensus | **Confirmed, strengthen alignment with OXE/LeRobot vocab** |
| Option C (BT over skills) belongs in "next-next phase" | Nav2 production precedent + Iovino 2022 survey show BTs are the current ROS2 idiom | **Partly wrong — promote to explicit v0.2 roadmap** |
| Capability profile is a new thing we define | URDF/SRDF/ros2_control already define most of it | **Partly wrong — make profile additive, not replacement** |
| Adapter is "opaque internally" | MoveIt Task Constructor + controller-switching is the in-ecosystem pattern for ROS2-native arms | **Keep opaque, but name MTC as reference implementation for ROS2-native adapters** |
| `MotionSupport` captures what adapters need | Missing **orientation authority** field — SCARA is the counter-example | **Gap — add** |
| `ExecutionMode` covers queued/streaming/offline/supervised | Missing `TRAJECTORY_ACTION` which is the ROS2 default | **Gap — add** |
| Validation ladder is "write it yourself" | mock_components, launch_testing, URSim, MCAP are ready | **Underspecified — name tools** |
| Canonical IR is a new invention | ROS-Industrial's CRCL / simple_message / robot_movement_interface already tried this | **Acknowledge prior art; explain how we avoid their fate** |
| Object-frame handling not in v0.1 schema | TP-GMM literature shows object frames are required for generalization | **Gap — add minimal support** |

---

## Part 2 — Revised Architecture Judgment

### 2.1 What holds (keep)

- **Option B as the chosen architecture.** Multiple independent lines
  of 2023–2026 evidence converge on the same core shape (EE-pose IR +
  per-robot adapter). No superior alternative surfaced. The shape
  itself is right.
- **Layer map** (capture → lifter → canonical → adapter → robot).
  Correct factoring; directly mirrors the ALOHA/OXE/DROID/AnyTeleop
  capture-and-replay pipelines.
- **Cartesian-primary, joint-hint representation.** Confirmed as
  cross-embodiment consensus.
- **Adapter as an opaque plan+execute contract.** Matches how MoveIt
  Task Constructor, UR ROS2 driver, and ros2_control expose their
  per-robot surfaces.
- **Validation ladder structure** (unit → contract → fake → TCP mock
  → sim → ros2_control fake → hardware). Rungs are right; we just
  need to name the specific tooling on each rung.
- **Migration plan shape** (grow the new package next to old, move
  files in, retire duplicate). Low-risk and matches how Nav2 and
  MoveIt 2 evolved from their ROS 1 predecessors.
- **MG400 as first adapter, not center.** Prior art on failed neutral
  command standards (CRCL) reinforces this: trying to design a grand
  universal vocabulary without a real first adapter is how they
  stalled.

### 2.2 What changes (revise)

1. **Canonical program IR alignment with OXE/LeRobot.** The schema
   retains its structured step list, but `move` and `tool` step
   payloads are stated in the same 7-DoF (pose + gripper) vocabulary
   used by Open X-Embodiment and LeRobot. Zero cost, interop upside.
2. **Capability profile made additive over URDF/SRDF/ros2_control.**
   The profile **references** a URDF path and a SRDF planning group
   and only defines fields that URDF/SRDF/ros2_control do not cover:
   execution-mode support, orientation authority, queue semantics,
   tool model, timing contract.
3. **Adapter pattern clarified.** For ROS2-native arms (UR, Franka,
   xArm, Kinova), the reference adapter implementation is
   **MoveIt Task Constructor for planning + controller-switching via
   `controller_manager` for execution-mode selection**. For TCP-queued
   vendor arms (Dobot MG400, M1 Pro), the reference is the existing
   queue-aware executor in this repo.
4. **Execution-mode enum extended** to include `TRAJECTORY_ACTION`
   (the ROS2 `FollowJointTrajectory` path), making the mode set:
   `OFFLINE_EXPORT`, `SUPERVISED_PLAYBACK`, `QUEUED`,
   `TRAJECTORY_ACTION`, `STREAMING`.
5. **BT-over-skills layer explicitly scheduled as v0.2.** Not "maybe
   someday." Moved into the migration plan as phase M7, built on top
   of the v0.1 IR without breaking it.
6. **Object/task frames in the v0.1 schema.** Even if the lifter
   initially writes `frames.object = null`, the field exists and the
   translator must route object-frame poses through the adapter's
   world-frame reference.
7. **Validation ladder tool names made concrete.** Each rung cites
   the specific tool (mock_components, launch_testing, PyBullet,
   URSim, MCAP, etc.).

### 2.3 What to cut

- **Treating the canonical IR as a pure-invention.** Replace with a
  short "prior art and lessons" subsection citing CRCL, simple_message,
  and robot_movement_interface, and explicitly stating why those
  projects stalled (too ambitious surface, no first-adapter anchor, no
  community momentum) and how we avoid their fate (stay thin, MG400
  is the first real user).
- **Joint-only retargeting as an adapter fallback path.** Do not
  describe "pure joint-replay robots can fall back to the hint"
  except as an internal adapter-implementation option, never as an
  IR-level design. Encouraging this invites joint-count brittleness.

### 2.4 What to add

- **§ Prior art and why we proceed anyway** (short, honest).
- **Orientation authority** field in `RobotCapabilityProfile` with
  concrete values: `full_6dof`, `yaw_only_scara`, `planar_xy_rz`,
  `custom` (with explicit axis mask).
- **ExecutionMode `TRAJECTORY_ACTION`** and its selection rule.
- **Adapter reference implementations** — a short table of
  (robot family, reference exec stack, reference plan stack).
- **Phase M7: BT-over-skills v0.2** in the migration plan.
- **Concrete tool names** on each validation-ladder rung.
- **Object/task frames** in the canonical IR.
- **MCAP** as the raw capture storage format.
- **Risk R11**: "We re-invent CRCL." Mitigation: stay thin, require
  every profile field to gate a real adapter decision.
- **Risk R12**: "Orientation projection on SCARA silently loses tool
  roll/pitch intent." Mitigation: the profile's orientation authority
  must be honored by the translator; mismatch raises
  `UnsupportedStep` with a clear reason.
- **Risk R13**: "Streaming mode semantics diverge silently between
  ROS2-native arms and MG400." Mitigation: profile's
  `timing_contract` is surfaced in the UI for any mode selection,
  and `STREAMING` on MG400 is labeled "streaming-through-queue" not
  "hard realtime."

---

## Part 3 — Concrete Changes to the Proposal

The following are the specific edits to apply to
`docs/ai_handoffs/next_phase_architecture_proposal.md`. Each is
prefixed with a §-number matching the target section in the original.

### §2.5 (Research Synthesis, execution semantics)
- Add explicit mention of **trajectory-action robots** as a distinct
  family, with `FollowJointTrajectory` as the ROS2 canonical action.

### §3 (Architecture Options)
- Under Option C tradeoffs, change "wrong time" to "right direction,
  v0.2 layer on top of v0.1." Cite Nav2 BT precedent and Iovino 2022.

### §4.3 (Canonical program schema v0.1)
- Replace `frames` object with:
  ```json
  "frames": {
    "world": "robot_base",
    "objects": {
      "target_part": { "parent": "world", "pose": null }
    },
    "tool_offset_m": [0.0, 0.0, 0.0]
  }
  ```
  with `objects.*.pose == null` meaning "unresolved at capture time;
  executor or operator must bind before replay."
- In each `move` step, add optional `pose_frame: "world"` (default)
  or `pose_frame: "target_part"` (one of the declared object frames).
- Add top-level `action_layout_compat: "open_x_embodiment_7dof"` as
  a non-binding annotation for future ML interop.

### §4.4 (Robot capability profile v0.1)
- Add to dataclass:
  ```python
  class OrientationAuthority(StrEnum):
      FULL_6DOF        = "full_6dof"
      YAW_ONLY_SCARA   = "yaw_only_scara"
      PLANAR_XY_RZ     = "planar_xy_rz"
      CUSTOM           = "custom"

  @dataclass(frozen=True)
  class RobotCapabilityProfile:
      ...
      urdf_path: str                  # required, not Optional
      srdf_path: Optional[str]
      planning_group: Optional[str]   # SRDF group name if applicable
      orientation_authority: OrientationAuthority
      orientation_axis_mask: Optional[tuple[bool, bool, bool]]  # only if CUSTOM
      ros2_control_command_interfaces: list[Literal["position", "velocity", "effort"]]
      ros2_control_state_interfaces:   list[Literal["position", "velocity", "effort"]]
  ```
- Remove the justification that "profile is a new thing"; replace
  with a note that URDF/SRDF/ros2_control interfaces are referenced
  from the profile, not duplicated.

### §4.5 (Adapter contract)
- Keep the `RobotAdapter` Protocol unchanged.
- Add a short table of reference implementations:

  | Robot family | Reference plan stack | Reference exec stack |
  |---|---|---|
  | Dobot MG400 / M1Pro (queued TCP) | this repo's current MotionPlanner | this repo's queue-aware TeleopController |
  | UR (ROS2-native) | MoveIt Task Constructor | `controller_manager` switching between `scaled_joint_trajectory_controller` and `forward_position_controller` (+ MoveIt Servo for streaming) |
  | Franka Panda (ROS2-native) | MoveIt Task Constructor | `joint_trajectory_controller` (+ libfranka RT loop for streaming) |
  | Simulated / CI | any | `ros2_control` mock_components GenericSystem |

### §4.6 (Execution mode model)
- Extend enum:
  ```python
  class ExecutionMode(StrEnum):
      OFFLINE_EXPORT       = "offline_export"
      SUPERVISED_PLAYBACK  = "supervised_playback"
      QUEUED               = "queued"
      TRAJECTORY_ACTION    = "trajectory_action"   # ROS2 FollowJointTrajectory
      STREAMING            = "streaming"
  ```
- Add a rule: when a profile declares both `TRAJECTORY_ACTION` and
  `STREAMING`, the default adapter behavior is to pick
  `TRAJECTORY_ACTION` for whole-program execution and `STREAMING`
  only when the caller requests it (e.g., live teach). This matches
  the UR driver convention: `scaled_joint_trajectory_controller` is
  the default, `forward_position_controller` is opt-in.

### §5 (Validation strategy)
- Rung 4 (fake backend): use the `RobotAdapter` protocol with a
  pure-Python `FakeBackend` + deterministic `calculate_dynamics`
  model mirroring `ros2_control` mock_components behavior.
- Rung 5 (TCP mock): keep current `MG400_Mock` warnings; add that
  **URSim** (Docker) is the analogous tool for UR.
- Rung 6 (kinematics sim): use PyBullet as the first-line sim.
  Add a second URDF to the test matrix (UR5 community URDF).
- Rung 7 (ROS2 fake): use `ros2_control` mock_components GenericSystem
  with `calculate_dynamics=true`; wire via `launch_testing`.
- Raw captures stored as **MCAP** bags, not ad-hoc JSON, for ecosystem
  compatibility.

### §6 (Migration plan)
- Insert **Phase M7: BT-over-skills v0.2**, after M6, as:
  - add a BT layer above the canonical IR that composes named skills
    whose leaves are v0.1 `move`/`tool`/`wait`/`set_frame` steps;
  - use BehaviorTree.CPP; XML trees + Groot2 introspection;
  - do not modify v0.1 IR; BT reads/writes the same schema through a
    skill-leaf adapter.
- Note: M1–M6 do not need to anticipate M7. The IR is forward-
  compatible as long as `steps` remains a linear list; BT composition
  happens **above** `steps`, by grouping step ids into named skills.

### §7 (Risk register)
- Add R11–R13 as listed in §2.4 above.
- R1 updated: explicit requirement that one of the rung-6 URDFs in
  the test matrix has a different DOF count than MG400 from day one.

### §8 (How a new robot gets added)
- Add a sentence: if the robot is ROS2-native, step 3 (`executor.py`)
  should normally be implemented as MoveIt Task Constructor +
  controller_manager switching, not custom TCP code.

### §11 (Pointer index)
- Add this revision file as a sibling pointer.

---

## Part 4 — Residual Uncertainty

Honest list of things still not settled after this research pass.

- **BT framework choice** (BehaviorTree.CPP + Groot2 vs py_trees_ros).
  Both are viable; BT.CPP has stronger industry adoption (Nav2,
  BehaviorTree.dev), py_trees_ros has lighter barrier to entry and
  native Python. Defer decision to M7 scoping.
- **Whether to store captures in MCAP or LeRobot Parquet/MP4
  primarily.** MCAP is the ROS2 ecosystem default; LeRobot is the
  ML-training default. A small translator utility between the two is
  likely cheaper than picking one and regretting it.
- **Whether the canonical program storage format should be JSON or
  YAML.** Irrelevant to architecture; pick when the first schema
  lands.
- **Whether to depend on MoveIt 2 at all for ROS2-native adapters.**
  MoveIt 2 brings heavy dependencies. If the project only ever uses
  MG400 plus one simple UR use case, a thinner stack (direct
  `joint_trajectory_controller` + an IK library like TracIK) may
  suffice. Revisit at M6 scoping.

---

## Part 5 — Bottom-line Change Summary

Original architecture holds. Option B is still the right core.
Evidence is strong enough that no alternative option surfaced.

Seven real improvements land on top:

1. IR step payloads aligned with OXE/LeRobot 7-DoF EE-action vocab.
2. Object/task frames in IR v0.1 (TP-GMM legacy).
3. Capability profile additive over URDF/SRDF/ros2_control, not
   replacement.
4. Orientation-authority field for SCARA honesty.
5. `TRAJECTORY_ACTION` execution mode for ROS2-native arms.
6. Adapter reference-implementation table (MTC, mock_components, UR
   controller-switching).
7. BT-over-skills explicitly scheduled as v0.2 (M7), not "maybe
   someday."

Three risks added: CRCL-repeat, SCARA orientation-loss,
streaming-mode divergence.

No part of the migration plan is invalidated. All proposed additions
are forward-compatible with the original phased plan.

---

## Part 6 — Second Pressure-Pass (proposal v1.2)

After v1.1 landed, four review findings were raised as hypotheses and
re-evaluated against repo + primary sources. The proposal was patched
in place; this section is the audit log so the v1.2 changes are not
ahistorical.

### 6.1 Finding A — "Canonical IR has no per-step orientation intent"

**Verdict: confirmed.**

Evidence:
- MoveIt's production `OrientationConstraint` carries per-link
  `absolute_x/y/z_axis_tolerance` (rad) — values like `~pi` mean
  "free axis" — and it is the canonical way ROS2 expresses partial
  orientation constraints. Profile-only flag would be too coarse: a
  single demo session can mix precise inserts with free approaches.
  See `moveit_msgs/Constraints` and OMPL constrained-planning docs.
- CRCL specifies pose orientation via X+Z direction vectors, with
  motion success defined "within tolerance"; per-command tolerance is
  built in, not a profile-level flag.
- AnyTeleop / DexPilot lineage retargets a full 6-DoF wrist pose but
  resolves SCARA-like dimensional mismatches at the optimization
  layer; per-step intent is the right place to declare what the
  optimizer should preserve.

Fix applied:
- §4.3 adds `orientation_intent` ∈ {`exact`, `yaw_only`,
  `tool_axis_align`, `free`} per `move` step, with optional
  `orientation_tolerance_rad` override (rpy triple), defaults table.
- R12 mitigation rewritten to merge per-step intent + profile
  authority.
- New R15 covers "adapter silently ignores intent."

### 6.2 Finding B — "Raw capture format inconsistent: MCAP vs session.json"

**Verdict: partially confirmed; the framing was wrong.**

Evidence:
- MCAP is the ROS 2 default bag format from Iron onward (Foxglove
  blog, ROS Discourse PSA, `ros2/rosbag2#1160`). Adopted in Jazzy
  LTS too.
- LeRobot uses Parquet+MP4, not MCAP, but it serves a *third* purpose
  (ML training set), not raw capture.
- Today's `TrajectoryRecorder` JSON is sample-decimated at ~8–10 Hz
  with delta gating. It is neither raw stream nor canonical program.

The two-format tension is not real if MCAP and `*.program.json`
serve different tiers. Treating them as competing was the framing
error.

Fix applied:
- §5 Rung 0 explicitly defines a **two-tier artifact model**:
  `*.session.mcap` (raw) and `*.program.json` (canonical IR).
- Existing `TrajectoryRecorder` JSON is reclassified as MG400
  adapter-internal playback cache (lives in `adapters/mg400/cache/`).
- LeRobot is named as a third tier (out of scope for v0.1).

### 6.3 Finding C — "URDF required is too narrow"

**Verdict: confirmed.**

Evidence:
- `ros2_control` itself accepts a minimal URDF with only the
  `<ros2_control>` tag and no geometric description; "you don't need
  any geometric description… you can implement the kinematics in the
  hardware_interface." (control.ros.org joints_userdoc)
- Offline-program robot families (ABB RAPID, KUKA KRL, Fanuc TP) are
  routinely programmed without a ROS-shaped URDF; vendor-native
  postprocessors (RoboDK supports >50 controllers) are the reality.
- A blunt `urdf_path: str` requirement excludes those workflows for
  no architectural gain.

Fix applied:
- §4.4 replaces `urdf_path: str` with `KinematicsContract` carrying
  `kind ∈ {URDF, NAMED, EXTERNAL, NONE}`. URDF is the preferred kind
  but not the only kind. NONE is allowed iff the profile is offline-
  export-only.
- New "adapters in this repo" table shows how MG400 today maps to
  `NAMED` (4-axis FK) and can later move to `URDF` without breaking
  the profile.

### 6.4 Finding D — "M4 buries MG400 kinematics bias"

**Verdict: confirmed.**

Evidence:
- v1.1 §6 M4 explicitly said "emit Cartesian poses via MG400 FK
  (temporary — fix in M5)." This is the exact anti-pattern the
  brief warns against: a robot-neutral core that imports MG400.
- The cleaner shape was always available: introduce the
  `KinematicsProvider` seam at the same milestone where the package
  is first created, with MG400 as the first concrete implementation.
- M4 acceptance can include running the lifter against a UR5 URDF
  synthetic session — that catches bias regressions for free.

Fix applied:
- §6 M1 now ships `KinematicsContract` + `KinematicsProvider` +
  URDF + Null providers from the first PR.
- §6 M3 lands the `mg400_4axis_fk` provider as the MG400 adapter's
  first concrete provider, not as core code.
- §6 M4 lifter consumes provider via DI; lifter package forbidden
  from importing `adapters.*` (CI-enforced). M4 acceptance includes
  a UR5-URDF synthetic-session run.
- Old M5 ("decouple FK") is deleted (the bias is never introduced in
  the first place).
- §9 next-actions reordered accordingly.
- New R14 covers "lifter quietly imports adapters."

### 6.5 What did **not** change

- Option B is still the chosen architecture.
- The `RobotAdapter` Protocol is unchanged.
- `ExecutionMode` enum is unchanged from v1.1 (already includes
  `TRAJECTORY_ACTION`).
- Validation ladder rungs 1–8 are unchanged (rung 0 was clarified,
  not invented).
- M6 (BT-over-skills, was M7) is unchanged in content; only the
  number shifted.

### 6.6 Bottom-line v1.2 delta

Four targeted edits land:

1. Per-step `orientation_intent` + tolerance triple (§4.3).
2. `KinematicsContract` replaces blunt `urdf_path: str` (§4.4).
3. Two-tier capture model (`*.session.mcap` + `*.program.json`) made
   explicit; today's `TrajectoryRecorder` JSON reclassified as MG400
   internal cache (§5 Rung 0).
4. Migration plan: kinematics seam from M1; M4 lifter never bakes
   MG400; old M5 deleted; M6→M5; M7→M6 (§6).

Three new risks added: R14 (lifter import discipline), R15
(adapter-ignores-intent). R12 mitigation rewritten to use per-step
intent + profile authority.

Open questions reduced; closed list now includes orientation, frames,
URDF, and lifter-FK. Two new open questions opened: kinematics
library choice, MCAP message-type for raw sessions.

---

## Part 7 — Third Pressure-Pass: VR Grounding + Delta Coverage (proposal v1.3)

### 7.1 Question

The user raised a real architecture gap: a VR hand pose is not
automatically a robot-accurate pose. Bare VR teaching may not know the
real table height, object pose, tool offset, robot base, or reachable
workspace. If we ignore this, the canonical program can look clean but
fail on the robot.

The user also requested explicit delta-robot coverage, because a
multi-robot teaching architecture must not assume only serial arms or
SCARA-like manipulators.

### 7.2 Evidence

- ROS `tf2` frames are the right mental model: robotic systems contain
  many frames, and meaningful pose use requires transforming data
  between source and target frames over time.
  <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html>
- OpenXR `STAGE` space provides a floor-referenced room-scale origin
  and bounds, but it remains a VR/operator-space reference, not the
  robot base or task fixture.
  <https://registry.khronos.org/OpenXR/specs/1.1/man/html/XR_REFERENCE_SPACE_TYPE_STAGE.html>
- MoveIt models the world around the robot in a planning scene,
  handles world geometry, collision objects, kinematics plugins, and
  joint-limit-aware trajectory processing. This supports keeping final
  feasibility checks on the robot/planning side, not only in VR.
  <https://moveit.ai/documentation/concepts/>
- Teleoperation virtual-fixture studies show that haptic/visual
  constraints can improve awareness and collision avoidance, but they
  are interface assistance; they are not a substitute for final robot
  feasibility validation.
  <https://link.springer.com/article/10.1007/s11370-019-00283-w>
- OpenXR core haptics are vibration primitives with amplitude,
  frequency, and duration, so v0.1 should use haptics as boundary
  awareness, not as precise force feedback.
  <https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrHapticVibration.html>
- Delta/parallel robots have workspace and singularity issues that
  are not well represented by a simple rectangular box. Their possible
  orientation can depend on position, and singular zones must be
  excluded.
  <https://www.mdpi.com/2673-4591/70/1/5>

### 7.3 Decision

Add a small but explicit calibration/retargeting layer to the
proposal, without turning VR into the source of truth.

Final shape:

- VR-captured programs carry a `calibration` block that binds
  `openxr_stage` / `unity_world` into a task frame.
- Task/object frames remain the long-term way to make the same taught
  program replay when the table/object location changes.
- `TeachingFeedbackContract` lets robot-side capability/workspace data
  flow back to Unity/VR for visual overlays or haptic warnings.
- That feedback is advisory only. The adapter still performs final
  reachability, orientation, limit, and workspace checks.
- `WorkspaceModel` is added to `RobotCapabilityProfile` so a robot can
  expose conservative workspace hints before expensive IK/planning.
- Delta robots are represented explicitly with
  `TRANSLATION_ONLY_DELTA` and `WorkspaceKind.ANALYTIC_DELTA`; they
  must not be squeezed into SCARA or serial-arm assumptions.

### 7.4 Proposal Changes Applied

- §4.2: added `calibration/` package in the proposed robot-neutral
  core layout.
- §4.3: added `calibration` block to the canonical program example.
- §4.3: added "Teaching calibration / retargeting contract" and
  "Retargeting policy" subsections.
- §4.4: added `WorkspaceModel`, `WorkspaceKind`, and
  `TRANSLATION_ONLY_DELTA`.
- §4.4: added delta adapter row and orientation-authority explanation.
- §5: added calibration/retargeting contract tests and delta/fake
  provider expectation in the validation ladder.
- §6: updated M1/M4/M5 migration steps to carry calibration and
  delta/fake coverage.
- §7: added R16, R17, R18.
- §9: updated PR1 next-actions to include calibration, workspace
  feedback, and a translation-only delta/fake fixture.
- §10: closed generic frame handling; opened calibration UI and first
  concrete delta-model choices.

### 7.5 What Remains Open

- How the user will perform calibration in the UI: three-point table
  fixture, robot probe, manual transform, or perception-assisted
  object binding.
- Which concrete delta robot or delta simulator becomes the first
  non-serial morphology target.
- Exact MCAP message types for raw Unity sessions.

These are not blockers for PR1 as long as the data contracts and tests
exist.
