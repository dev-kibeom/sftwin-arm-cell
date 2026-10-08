# M09-W01 Interface Realization Inventory

This inventory records executable IDL mapping and delegated realization
choices for the approved M09 shared/public contracts. Contract meaning remains
owned by the linked ICDs.

| Endpoint | ROS interface | Realization |
|---|---|---|
| `/vda/request_material` | `srv/RequestMaterial` | `delivery_id` is supplied by Integration; response acceptance is distinct from handoff completion. |
| `/vda/material_handoff_state` | `msg/MaterialHandoffState` | VDA source timestamp, delivery UUID, canonical phase, and validity. |
| `/integration/material_readiness` | `msg/MaterialReadiness` | Integration source timestamp, delivery UUID, readiness fact, and validity. |
| `/safety/reset` | `srv/ResetSafety` | UUID-correlated explicit operator acknowledgement; only Safety reports whether policy applied. |
| `/orchestration/execute_cycle` | `action/ExecuteCycle` | Goal correlates admitted `delivery_id`; feedback adds batch, iteration, target, and first-interruption observation. |
| `/vision/detect_target` | `srv/DetectTarget` | Adds the approved temporary-invalid-target result constant while preserving the existing response shape. |
| `/safety/state` | `msg/SafetyState` | Adds Safety-owned envelope validity and velocity/acceleration scale fields. |
| `/motion/state` | `msg/MotionStatus` | Adds holding disposition and backend-attested applied-envelope fields. |

The two new state topics use the repository's existing external-state shape:
`std_msgs/Header` timestamps identify producer samples, `valid` gates use of a
sample, and consumers must evaluate freshness with local receipt time. Existing
ROS state endpoints use reliable, volatile `rclcpp::QoS(10)`; these topics
follow that convention. No freshness threshold is introduced by IDL. The
service and action endpoints follow their existing ROS service/action
transport conventions, and IDL does not prescribe endpoint implementation or
runtime behavior.

Contract references:

- [Integration ↔ VDA Material](icd-integration-vda-material.md)
- [Material Readiness ↔ Orchestration](icd-integration-orchestration-material.md)
- [Operator Acknowledge and Safety Reset](icd-safety-operator.md)
- [ExecuteCycle](icd-orchestration-execute-cycle.md)
- [Vision ↔ Orchestration](icd-vision-orchestration.md)
- [Safety ↔ Motion](icd-safety-motion.md)
