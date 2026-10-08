# [SF-Twin] ARM Cell Static Planning Scene Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-MOTION-STATIC-SCENE_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / MoveIt Static Planning Scene`
- **Version:** `1.0.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Motion`
- **Related ADR:** `ADR-ARM-CELL-0002`

## 1. Canonical Geometry and Selection

Canonical nominal environment geometry and MoveIt inclusion policy are
separate concerns. Cross-consumer environment relationships are owned by
[ARM Cell Simulation Environment Semantics](../simulation-environment-semantics.md).

```text
canonical nominal environment
+ explicit MoveIt selection
→ generated self-contained runtime artifact
→ bounded loader
→ MoveIt PlanningScene
```

Generated runtime artifacts SHALL NOT become canonical geometry authority.

## 2. Loader Responsibility

The loader SHALL:

- validate generated artifact before applying it;
- mutate only its selected static-scene responsibility;
- preserve unrelated PlanningScene state;
- perform sufficient readback/confirmation;
- fail explicitly on invalid load/readback.

## 3. Runtime Authority

MoveIt PlanningScene/PlanningSceneMonitor remains runtime planning-scene authority.

## 4. Profile Boundary

The static-scene profile is explicit and opt-in.

## 5. Verification Requirements

### VR-MOT-STATIC-01 — Canonical projection
Selected canonical static geometry SHALL project into the generated planning artifact.

### VR-MOT-STATIC-02 — Bounded mutation
Reapplication SHALL preserve unrelated PlanningScene state.

### VR-MOT-STATIC-03 — Readback
Failed load/readback SHALL be detectable.

### VR-MOT-STATIC-04 — Explicit profile
Static-scene behavior SHALL not silently redefine robot-only planning.
