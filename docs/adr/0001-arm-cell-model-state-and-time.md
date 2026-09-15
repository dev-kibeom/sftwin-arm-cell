# ADR 0001: ARM Cell model, measured state, TF, and simulation time authority

- **Document ID:** `ADR-ARM-CELL-0001`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Isaac development planning profile`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `Historical Milestone 1A decision; exact date not recorded`
- **Related Design:** `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`
- **Supersedes:** `None`
- **Superseded By:** `None`

## 1. Context

The ARM Cell development profile needs one reusable robot model, measured simulator state,
one TF authority, and one simulation-time authority while preserving the existing
user-verified Isaac Sim M0609/Robotiq baseline.

Directly exposing Isaac articulation topology to upper ROS layers would couple planning to
simulator-specific joints. Maintaining independent editable ROS and Isaac robot models would
allow the two representations to drift. Allowing both Isaac and ROS to publish robot-model TF
would create competing authorities. Hiding simulation reset by rebasing time would make
simulation epochs difficult to reason about and would require a broader lifecycle policy.

Milestone 1A is a development planning profile. It does not establish production execution,
safety, mission orchestration, or permanent reset/recovery semantics.

## 2. Decision

We will use one shared editable robot model, project Isaac measurements into the canonical
model state through a narrow adapter, keep robot-model TF under ROS authority, use Isaac as
the sole simulation-clock authority, and require explicit ROS-profile restart after a
backward simulation-time epoch change.

Specifically:

- `arm_cell_description` owns the shared editable Xacro and model resources used by both ROS
  and the Isaac construction workflow.
- Isaac owns raw measured articulation state.
- `arm_cell_sim_adapter` projects only the canonical model's independent measured joints to
  `/joint_states`; it does not invent missing measurements or expose simulator-only joints.
- `robot_state_publisher` is the sole robot-model TF publisher.
- ARM Cell bringup owns the configured static `world -> base_link` edge for this profile.
- Isaac is the sole `/clock` publisher and participating ROS nodes use simulation time.
- Backward clock/state timestamp movement causes the adapter to stop canonical publication
  until the complete ROS profile is restarted.
- `arm_cell_moveit_config` owns planning semantics/configuration; ROS bringup composes ROS
  processes, while simulator construction remains infrastructure responsibility.

Current topic, frame, state, and profile semantics are owned by the relevant design/interface
documents rather than by this ADR.

### 2.1 Considered Alternatives

#### Option A — Independent ROS and Isaac editable robot models

- **Advantages:** each environment could evolve independently.
- **Trade-offs:** model drift and duplicated maintenance.
- **Why not selected:** the development profile needs shared kinematic identity more than
  independent model ownership.

#### Option B — Consume raw Isaac joint state directly

- **Advantages:** fewer ROS components.
- **Trade-offs:** simulator topology and gripper implementation leak above the adapter boundary.
- **Why not selected:** upper-layer planning should depend on canonical model semantics, not
  simulator-specific articulation details.

#### Option C — Publish robot TF from both Isaac and ROS

- **Advantages:** fewer transformations may need to be reconstructed in one environment.
- **Trade-offs:** competing TF authorities and difficult-to-diagnose inconsistency.
- **Why not selected:** each TF edge must have one authority.

#### Option D — Automatically rebase backward simulation time

- **Advantages:** fewer operator restarts.
- **Trade-offs:** hides simulation epochs and introduces implicit lifecycle semantics.
- **Why not selected:** Milestone 1A did not yet justify a permanent reset/recovery design.

## 3. Consequences and Trade-offs

### Benefits

- ROS and Isaac share one editable robot-model source.
- Planning consumes canonical measured state rather than simulator topology.
- TF and simulation-time authority are unambiguous.
- Reset behavior remains explicit and locally understandable.

### Trade-offs / Costs

- A narrow simulation-state adapter must be maintained.
- Reset/stage replacement is operationally heavier because the complete ROS profile must be
  restarted.
- This decision intentionally does not solve production recovery, execution freshness, or
  safety lifecycle behavior.

## 4. Mitigation Strategy

- Keep the adapter limited to state projection and validation.
- Keep current state/time/TF contracts in CDS/ICD-level design rather than expanding this ADR
  into a current specification.
- Treat explicit restart as a bounded development-profile policy until a broader lifecycle
  design is approved.
- Verify loaded-stage placement before claiming spatial alignment; configuration values alone
  are not proof of stage agreement.

## 5. References

- `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`
- `docs/guides/arm-cell-isaac-sim.md`
- `docs/reports/acceptance/arm-cell-milestone-1a.md`
