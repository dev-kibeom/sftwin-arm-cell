# [SF-Twin] ARM Cell Simulator State, Time, and TF Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-SIM-STATE-TIME-TF_v1.0.0`
- **Document Type:** `ICD`
- **Scope:** `Isaac / ARM Cell Integration / ROS State & TF Consumers`
- **Version:** `1.0.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Integration`
- **Related ADR:** `ADR-ARM-CELL-0001`

## 1. State

Simulator raw articulation state is published on:

```text
/isaac/joint_states
```

The canonical ROS state is published on:

```text
/joint_states
```

Canonical output exposes required independent canonical joints only.

Malformed/incomplete required state is rejected rather than fabricated.

## 2. Time

Isaac is the sole `/clock` authority for the supported simulator profile.

Participating ROS nodes configured for simulation time use that clock.

No competing `/clock` publisher is permitted.

## 3. Epoch

Backward simulation-time movement invalidates the current supported ROS session.

```text
backward epoch
→ integration session invalid
→ canonical publication refused
→ complete ROS profile restart
```

Transparent timestamp rebasing is prohibited.

## 4. TF

- `robot_state_publisher` is sole authority for robot-model TF edges.
- ARM Cell bringup owns the configured `world → base_link` edge.
- MoveIt virtual-joint semantics do not create a competing TF publisher.

## 5. Verification Requirements

### VR-ICD-SIM-STATE-01
One canonical `/joint_states` publication path SHALL exist.

### VR-ICD-SIM-STATE-02
Simulator-only/dependent articulation SHALL not leak into canonical state.

### VR-ICD-SIM-TIME-01
Isaac SHALL be the sole `/clock` authority in the supported profile.

### VR-ICD-SIM-TIME-02
Backward time SHALL invalidate the active session rather than be hidden.

### VR-ICD-SIM-TF-01
Robot-model TF SHALL have one publisher authority.

### VR-ICD-SIM-TF-02
`world → base_link` SHALL have one configured publisher authority.
