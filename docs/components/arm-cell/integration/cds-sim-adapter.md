# [SF-Twin] ARM Cell Simulator State Adapter Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-INTEGRATION-SIM-ADAPTER_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Simulator Measured-State Adaptation`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Integration`
- **Related ADR:** `ADR-ARM-CELL-0001`

## 1. Purpose

The simulator-state adapter prevents simulator-specific articulation topology from becoming a canonical ROS robot-state contract.

## 2. Responsibilities

The adapter SHALL:

- consume raw measured articulation state from the simulator boundary;
- derive required independent movable joints from the canonical robot model;
- project only canonical independent measured joints into canonical robot state;
- preserve valid source measurement values and timestamps;
- preserve optional velocity/effort when valid and available;
- reject malformed or incomplete required measurements;
- isolate simulator-only and dependent mimic representation from upper ROS layers.

## 3. Non-Responsibilities

The adapter SHALL NOT:

- synthesize missing joint positions;
- substitute missing required measurements with zero;
- refresh stale timestamps;
- reinterpret units without an explicit shared contract;
- expose simulator-only articulation entries;
- command arm or gripper motion;
- publish robot-model TF;
- own mission or safety semantics.

## 4. Canonical Projection

```text
Isaac raw measured articulation state
            ↓
    simulator-state adapter
            ↓
canonical independent robot measurements
            ↓
       /joint_states
       ├─ robot_state_publisher
       └─ MoveIt current-state monitor
```

## 5. Verification Requirements

### VR-INT-STATE-01 — Canonical-joint projection
Canonical output SHALL contain required independent canonical joints and SHALL NOT expose simulator-only/dependent articulation entries.

### VR-INT-STATE-02 — No fabricated measurements
Malformed or incomplete required measurements SHALL be rejected rather than synthesized.

### VR-INT-STATE-03 — Source semantic preservation
Valid source values and timestamps SHALL be preserved unless an explicit ICD defines a conversion.

### VR-INT-STATE-04 — Optional field preservation
Absence of optional velocity/effort SHALL NOT be represented as fabricated values.
