# [SF-Twin] ARM Cell Motion MoveIt Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-MOTION-MOVEIT_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / MoveIt Planning Responsibility`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Motion`

## 1. Responsibilities

The Motion/MoveIt concern SHALL own or consume, as appropriate:

- ARM planning-group semantics;
- kinematics/planning configuration;
- canonical current-state consumption;
- collision-aware planning;
- Motion-owned target offsets;
- PlanningScene interaction;
- controller-execution dependency when enabled by the selected profile.

`arm_cell_moveit_config` owns planning configuration semantics.

## 2. Canonical State

MoveIt SHALL consume canonical robot state from the integration boundary and SHALL NOT depend directly on simulator-specific articulation state.

## 3. Planning Profiles

A robot-only development planning profile does not establish cell-environment collision clearance.

Static-cell collision claims require the explicit static-scene profile.

Planning-only validation SHALL NOT be represented as production trajectory-execution or safety acceptance.

## 4. Verification Requirements

### VR-MOT-MOVEIT-01 — Canonical-state consumption
MoveIt SHALL use canonical robot state.

### VR-MOT-MOVEIT-02 — Profile-scope integrity
Robot-only planning SHALL NOT be represented as static-cell collision or execution acceptance.

### VR-MOT-MOVEIT-03 — Motion-owned target conversion
Planning targets SHALL use Motion-owned tool/TCP transformation semantics.
