# ARM Cell Simulation Environment Semantics

- **Document ID:** `[SF-Twin]_DOMAIN-SEMANTICS-ARM-CELL-SIMULATION-ENVIRONMENT_v0.1.0`
- **Document Type:** Domain Semantics
- **Domain / Semantic Concern:** ARM Cell simulation environment
- **Version:** `0.1.0`
- **Status:** `Review`
- **Owner:** ARM Cell Product / Architecture Authority
- **Related HLD / CDS / FDS / ICD:** [System Overview HLD](../../architecture/system-overview.md), [Vision CDS](vision/cds.md), [Motion Static Planning Scene CDS](motion/cds-static-scene.md), [Material Handoff FDS](integration/fds-material-handoff.md), [UI Hub FDS](integration/fds-operator-ui-hub.md), [Vision ↔ Orchestration ICD](../../interfaces/arm-cell/icd-vision-orchestration.md), [Material Handoff ICD](../../interfaces/arm-cell/icd-integration-vda-material.md), [Final Demo Definition](../../product/demos/arm-cell/definition.md)

## 1. Purpose

This document canonically owns the cross-cutting semantic invariants that an
ARM Cell simulation environment must preserve so Vision, Motion, Integration,
VDA, Safety, Orchestration, and the operator surface interpret one simulated
cell consistently. These invariants have no single component, behavior, or
interface owner: they span environment realization and remain relevant when
that realization is refactored. The environment is a shared semantic input,
not a new runtime authority.

This model defines environment validity and meaning only. Component
responsibility remains in CDS documents, runtime behavior in FDS documents,
shared contracts in ICD documents, and the integrated demo outcome in the
[Final Demo Definition](../../product/demos/arm-cell/definition.md).

## 2. Scope

### In Scope

- Cross-consumer meaning of cell/workspace spatial relationships.
- Environment preconditions for production perception, material handoff,
  collision planning, external-process indication, and acceptance claims.
- Observable invariants that remain true across simulation-environment
  refactors.

### Out of Scope

- Environment layout, exact poses, dimensions, prim paths, camera coordinates,
  ArUco IDs, or AMR path lengths.
- Isaac/USD construction, simulator APIs, scene graph organization, or runtime
  implementation choices.
- Component algorithms, message fields, interface endpoints, Safety policy,
  mission sequencing, or admission decisions.
- Demo-specific calibration values and physics settings.

## 3. Domain Context

The environment supplies sensor observations and spatial context used by
multiple runtime owners. It must preserve the same intended cell relationships
whether a consumer is interpreting production RGB-D, placing material,
planning collision-aware robot motion, observing CNC state, or injecting a
scenario stimulus. Passing an environment invariant does not by itself prove a
component algorithm or authorize runtime motion or mission admission.

## 4. Semantic Model / Concepts

- **Canonical cell/workspace relationship:** the shared spatial relationship
  used to interpret the cell, target workspace, material region, robot
  workspace, and sensor observations. Frame names and transform contracts
  remain owned by the applicable ICDs.
- **Sensor-observable evidence:** support/work-surface geometry and
  target-relevant evidence that production sensor observations expose to a
  declared Vision profile, without authored simulator ground truth. Vision
  FDS documents own actual target selection and detection outcome semantics.
- **Material handoff continuity:** the environment represents the delivery
  from AMR/VDA unload through transfer realization to the material workspace
  without losing delivery or spatial continuity. Integration remains the
  authority for transfer completion and `MATERIAL_READY`.
- **Acceptance-scoped physical profile:** the physics, gravity, and collision
  configuration is appropriate to the physical behavior claimed by a given
  acceptance scenario. A profile does not imply claims beyond its evidence.
- **Fault stimulus:** a scene element may create an observable condition or
  request a scenario through its owning boundary. Its presence or visual state
  does not change another component's canonical authority.

## 5. Ownership / Authority

This document owns only the cross-cutting environment invariants below. It does
not transfer authority from existing owners:

- Vision FDS/VR owns profile execution, target selection, detection success,
  and perception result semantics; this document does not assert those outcomes.
- Integration/VDA own delivery facts and transfer completion at their existing
  boundaries.
- Orchestration owns batch admission and mission progression.
- Safety owns capability, severity, and permitted motion envelope.
- Motion owns collision-aware planning and execution.
- The UI Hub observes canonical state and issues requests/stimuli only.
- ICDs continue to own frame, sensor, material, and external-process contracts.

## 6. Cross-Component Usage

| Consumer | Consistent interpretation required |
|---|---|
| Vision | Production RGB-D observations expose usable support and target evidence/geometry to the declared profiles; Vision FDS owns profile execution and detection results. |
| Integration / VDA | Unload and transfer realization preserve material continuity into the material workspace; only Integration establishes transfer completion. |
| Motion | Robot workspace, material region, and collision-planning geometry refer to the same canonical cell relationships. |
| Safety / UI Hub | CNC/PackML indication reflects canonical external-process state without creating or changing that state. |
| Fault-scenario consumers | Environment elements produce stimuli through their owner paths and do not bypass canonical state authority. |

## 7. Dependencies / Boundaries

Spatial frame names, sensor data semantics, material delivery facts, external
PackML state, collision-scene ownership, and component runtime behavior are
specified by their respective ICD/CDS/FDS owners. This document defines only
the environment-level invariants those contracts consume together.

Realization and calibration values belong to configuration and implementation
authority. No exact pose, dimension, prim path, marker identifier, camera
coordinate, motion distance, or simulator metadata is a semantic requirement
here.

## 8. Domain Invariants

1. The canonical cell/workspace frame relationship SHALL remain semantically
   consistent across sensor observations, target references, material regions,
   robot workspaces, and collision-planning representations.
2. The production Vision sensor SHALL observe the nominal target workspace
   sufficiently to support the declared production target profiles.
3. The support/work-surface reference SHALL be establishable from production
   sensor observations without requiring authored simulator surface identity.
4. Target material placed on the support surface SHALL provide sufficient
   sensor-observable evidence and geometry for its declared production Vision
   profile to evaluate the observation. This invariant does not assert target
   selection or detection success.
5. Unrelated foreground geometry SHALL NOT make the target workspace incapable
   of isolating a requested target under its configured production profile.
6. The environment SHALL preserve material handoff continuity from AMR/VDA
   unload, through transfer realization, into the material workspace.
7. Conveyor or transfer visualization/realization SHALL NOT be treated as
   canonical transfer completion, `MATERIAL_READY`, or batch-admission
   authority.
8. Any CNC/PackML indicator in the environment SHALL be a read-only projection
   of canonical external-process state and SHALL NOT publish, select, or
   overwrite that state.
9. The robot workspace, material region, and collision-planning representation
   SHALL remain semantically aligned within the canonical cell/workspace
   relationships.
10. The production sensor-facing environment SHALL expose target evidence
    through sensor observations without requiring simulator prim identity, exact
    world coordinates, simulator metadata, or simulator API ground truth.
11. Physics, gravity, and collision configuration SHALL remain appropriate to
    the physical behavior claimed by each acceptance scenario; a configured
    profile SHALL NOT invalidate its own stated claim.
12. Fault-injection scene elements SHALL NOT bypass or directly overwrite the
    canonical Safety, Orchestration, Vision, Motion, VDA, or Integration
    authority they are intended to stimulate.

## 9. Verification Requirements

| ID | Semantic Claim / Invariant | Required Observable Result |
|---|---|---|
| `VR-ARM-SIM-ENV-01` | Canonical cell/workspace frame relationships remain consistent. | Sensor observations, target references, workspace regions, and planning geometry resolve to mutually consistent spatial relationships. |
| `VR-ARM-SIM-ENV-02` | Production Vision observes the nominal target workspace. | The declared target workspace is represented in valid production sensor observations and supports the configured production profiles. |
| `VR-ARM-SIM-ENV-03` | Support/work-surface reference is sensor-establishable. | Production sensor observations contain sufficient support evidence to establish the support reference without simulator-authored identity. |
| `VR-ARM-SIM-ENV-04` | Supported target material provides profile-usable sensor evidence and geometry. | Production RGB-D observations of the target on its support contain evidence/geometry sufficient for the declared production profile's sensor-facing preconditions; the oracle does not assert Vision target-selection or detection success. |
| `VR-ARM-SIM-ENV-05` | Target-profile isolation remains possible with unrelated foreground. | With target and unrelated foreground both present, production sensor observations preserve distinguishable target evidence under the requested profile's declared sensor-facing constraints; the target-selection result remains owned by the Vision FDS. |
| `VR-ARM-SIM-ENV-06` | Scene realization preserves the physical continuity of the declared material handoff. | Scene observation shows material continuity from the VDA unload realization into the material workspace for the same delivery; cross-boundary fact identity/order is owned by the Integration↔VDA ICD and Integration FDS. |
| `VR-ARM-SIM-ENV-07` | Scene realization cannot bypass canonical material authority. | Activating or observing conveyor/transfer scene elements alone does not mutate canonical Integration readiness or Orchestration admission; those state facts remain observable only through their owning interfaces, as specified by `VR-ICD-INT-VDA-MAT-03` and `VR-ICD-MATERIAL-01`. |
| `VR-ARM-SIM-ENV-08` | CNC/PackML indication is read-only. | The indicator follows canonical external-process state; changing the indicator alone cannot change that state. |
| `VR-ARM-SIM-ENV-09` | Robot, material, and collision-planning regions are semantically aligned. | Observed cell relationships and planning collision representation agree for the regions used by the acceptance claim. |
| `VR-ARM-SIM-ENV-10` | Production target evidence is available from sensor observations without simulator ground truth. | Sensor observations and declared profile inputs contain the target evidence needed for semantic interpretation without requiring simulator-only identity, metadata, exact coordinates, or API access; the oracle does not assert a Vision result. |
| `VR-ARM-SIM-ENV-11` | Physical configuration supports the acceptance claim. | The active physics/gravity/collision profile does not contradict the physical behavior asserted by the scenario evidence. |
| `VR-ARM-SIM-ENV-12` | Fault elements cannot bypass canonical authority. | A scene stimulus changes canonical state only through the owning component/interface; direct scene/UI mutation of canonical state is absent. |

## 10. References

- [Final Demo Definition](../../product/demos/arm-cell/definition.md)
- [Vision CDS](vision/cds.md), [Vision DetectTarget FDS](vision/fds-detect-target.md), and [Vision ↔ Orchestration ICD](../../interfaces/arm-cell/icd-vision-orchestration.md)
- [Simulated RGB-D ICD](../../interfaces/arm-cell/icd-sim-rgbd.md) and [Camera Overlay CDS](integration/cds-camera-overlay.md)
- [Static Planning Scene CDS](motion/cds-static-scene.md) and [MoveIt CDS](motion/cds-moveit.md)
- [Material Handoff FDS](integration/fds-material-handoff.md), [VDA Material Delivery FDS](vda/fds-material-delivery.md), and [Integration ↔ VDA Material ICD](../../interfaces/arm-cell/icd-integration-vda-material.md)
- [External State ↔ Safety ICD](../../interfaces/arm-cell/icd-external-safety.md), [Safety Supervision FDS](safety/fds-supervision.md), and [Simulation UI Hub FDS](integration/fds-operator-ui-hub.md)
- [Fault Injection FDS](vda/fds-fault-injection.md)
