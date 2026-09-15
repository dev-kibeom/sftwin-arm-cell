# ADR 0002: ARM Cell bounded static planning-scene ownership and lifecycle

- **Document ID:** `ADR-ARM-CELL-0002`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Milestone 1B static planning scene`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `Historical Milestone 1B decision; exact date not recorded`
- **Related Design:** `docs/components/arm-cell/motion/cds-static-scene.md`
- **Supersedes:** `None`
- **Superseded By:** `None`

## 1. Context

Milestone 1A intentionally plans in a robot-only world. Milestone 1B needs bounded static
collision awareness for selected ARM Cell geometry without making a saved Isaac USD stage,
a generated MoveIt artifact, or a runtime repository checkout the canonical geometry source.

The same physical geometry is consumed by two environments with different responsibilities:
Isaac constructs the simulation scene, while MoveIt consumes a planning projection. A single
monolithic scene representation would either expose simulator-specific details to planning or
duplicate geometry and allow drift.

The planning scene also has existing runtime state. Loading the bounded static projection
must not clear unrelated PlanningScene state or create a second long-lived scene authority.

## 2. Decision

We will keep a simulator-neutral canonical nominal static-environment manifest, keep MoveIt
selection policy separate from geometry, generate a self-contained MoveIt artifact, and use a
bounded loader that validates before mutation and verifies readback.

Specifically:

- the ARM Cell physical-layout manifest owns the nominal static geometry;
- Isaac construction consumes the selected canonical definitions while retaining Isaac-only
  construction properties in simulator code;
- MoveIt owns a selection policy containing identifiers/policy rather than duplicated geometry;
- the selected planning artifact is generated and installed with the MoveIt package so runtime
  does not depend on the repository root or Isaac;
- the loader validates the complete artifact before applying selected `ADD` objects through
  PlanningScene APIs and verifies IDs, geometry, frames, and poses by readback;
- reapplication preserves unrelated PlanningScene world state rather than globally clearing it;
- the robot-only profile remains the default and the bounded static-scene profile is explicit;
- MoveIt's PlanningSceneMonitor remains the runtime planning-scene authority.

Current file paths, selected object set, launch profile, and runtime contract are owned by the
relevant design documents.

### 2.1 Considered Alternatives

#### Option A — Treat the saved Isaac USD stage as canonical geometry

- **Advantages:** direct visual/simulation correspondence.
- **Trade-offs:** saved stages can contain session/editor state and simulator-specific details.
- **Why not selected:** planning needs a stable simulator-neutral nominal definition.

#### Option B — Maintain separate hand-authored Isaac and MoveIt geometry

- **Advantages:** simple local configuration in each environment.
- **Trade-offs:** duplicated geometry and silent drift.
- **Why not selected:** the same physical layout should have one nominal source.

#### Option C — Clear and rebuild the entire PlanningScene

- **Advantages:** simple loader semantics.
- **Trade-offs:** destroys unrelated world state and creates unnecessary coupling.
- **Why not selected:** the static projection is intentionally bounded and should mutate only
  its owned objects.

#### Option D — Enable the static scene in every planning profile

- **Advantages:** fewer launch choices.
- **Trade-offs:** changes the established robot-only baseline and makes isolation harder.
- **Why not selected:** Milestone 1B is an explicit extension of the Milestone 1A profile.

## 3. Consequences and Trade-offs

### Benefits

- Isaac and MoveIt consume one nominal static-geometry source without sharing simulator detail.
- Runtime planning is self-contained.
- Static-scene mutation is bounded and preserves unrelated PlanningScene state.
- Robot-only and static-scene profiles remain independently testable.

### Trade-offs / Costs

- Generated planning artifacts must be regenerated when canonical geometry or selection changes.
- The selected static scene is intentionally partial and is not a complete world model.
- A loader and selection layer add maintenance compared with directly reading simulator state.

## 4. Mitigation Strategy

- Validate generated artifacts before runtime mutation and verify applied state by readback.
- Keep geometry ownership, selection policy, and runtime scene authority separate.
- Keep the static profile explicitly selected until a broader product/runtime requirement says
  otherwise.
- Keep dynamic/perception-derived obstacles outside this static-scene decision.

## 5. References

- `docs/components/arm-cell/motion/cds-static-scene.md`
- `docs/guides/arm-cell-isaac-sim.md`
- `docs/reports/acceptance/arm-cell-milestone-1b-static-scene.md`
