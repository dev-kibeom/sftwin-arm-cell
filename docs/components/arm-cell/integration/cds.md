# [SF-Twin] ARM Cell Integration Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-INTEGRATION_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / ROS–Simulator Integration`
- **Version:** `1.1.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Integration`
- **Related ADRs:** `ADR-ARM-CELL-0001`, `ADR-ARM-CELL-0003`

## 1. Purpose

This document defines the static responsibility and ownership boundary of the ARM Cell integration family.

The integration family owns ROS/simulator composition and adaptation required to expose simulator-backed ARM Cell behavior through canonical ROS contracts.

It does not own mission behavior, motion-task semantics, perception algorithms, or safety policy.

## 2. Responsibilities

The integration family SHALL own:

- composition of supported ARM Cell ROS development profiles;
- separation between simulator infrastructure and ROS-side composition;
- adaptation from simulator-specific measured state into canonical robot state;
- composition of shared robot-model, TF, and simulation-time authorities;
- simulated-camera overlay derivation/composition needed to expose the RGB-D source through canonical ROS frames;
- explicit simulation-session lifecycle behavior when simulator time moves backward;
- validation-only Isaac fixture lifecycle and seeded scene material creation;
- simulator-specific realization of backend-neutral gripper commands,
  capture-local snap attachment, detachment, and holding observation.

The integration family SHALL preserve:

```text
simulator-internal representation
→ adaptation / composition boundary
→ canonical ROS contracts
→ Motion / Vision / other consumers
```

## 3. Non-Responsibilities

The integration family SHALL NOT own:

- mission sequencing or retry policy;
- Safety motion-capability authority;
- Motion task semantics;
- Vision perception processing;
- safety-rated E-Stop/STO behavior;
- product-level acceptance policy;
- production perception or calibration authority;
- Motion-owned PICK/PLACE ordering and TCP/tool conversion.

## 4. Authority Boundaries

### 4.1 Robot model

`arm_cell_description` is the editable source of the shared robot model used by ROS and the Isaac construction workflow.

Generated URDF/USD artifacts are derived artifacts and SHALL NOT become independent editable model authorities.

### 4.2 Simulator construction

Simulator scene construction remains simulator-infrastructure responsibility.

ROS bringup composes ROS processes and configuration but does not take ownership of simulator scene construction.

The supported simulator-backed profile assumes an externally started and valid Isaac Sim
session. ROS bringup does not own Isaac application startup or simulator scene construction.

### 4.3 Validation fixture and gripper adaptation

The deterministic PnP fixture and Isaac gripper adapter are validation/runtime
integration infrastructure. The fixture may create eligible test material and
register its known object reference with a validation-only Vision adapter. The
Isaac gripper adapter may create/remove simulator joints only through the
backend-neutral `GripperPort` behavior.

Neither component may identify a grasp candidate from `target_id` or leak
simulator prim identity into Vision, Motion, or Orchestration. Detailed
behavior is defined by
[`fds-deterministic-pnp-fixture.md`](fds-deterministic-pnp-fixture.md).

### 4.4 Shared interface semantics

Exact topic, TF, time, frame, unit, QoS, and invalid/stale semantics are owned by the relevant ICDs.

## 5. Verification Requirements

### VR-INT-OWN-01 — Single editable robot model authority
The supported profile SHALL use one editable robot-model source for shared ROS/Isaac kinematic identity.

### VR-INT-OWN-02 — Simulator infrastructure separation
ROS bringup SHALL NOT become the owner of Isaac scene construction or simulator-internal topology.

### VR-INT-OWN-03 — No cross-domain responsibility leakage
Integration SHALL NOT absorb Motion, Vision, Orchestration, or Safety semantic ownership.

### VR-INT-OWN-04 — Validation infrastructure isolation
Fixture identity, Isaac prim paths, capture volumes, and fixed-joint details
SHALL remain below the integration boundary and SHALL NOT redefine canonical
Vision, Motion, or Orchestration contracts.


## 6. References

- `docs/adr/0001-arm-cell-model-state-and-time.md`
- `docs/adr/0003-arm-cell-rgbd-sensor-contract.md`
- `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`
- `docs/interfaces/arm-cell/icd-sim-rgbd.md`
