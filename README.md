# SF-Twin ARM Cell

SF-Twin ARM Cell is a simulator-backed robotics workcell for developing and
verifying a pick-and-place cell with ROS 2, MoveIt, Isaac Sim, a Doosan M0609
robot model, and a Robotiq 2F-85 gripper model.

It demonstrates a contract-first integration baseline: a shared robot model,
canonical measured joint state, single simulation-time and TF authorities,
bounded static-scene planning, RGB-D ingress, and backend-neutral public ROS
interfaces for the runtime components that follow.

## Scope and maturity

The supported profile is simulator-backed and targets ROS 2 Humble with Isaac
Sim 5.1. Current source implements the robot-model, state/time/TF,
generated-USD provenance, static-scene, and Vision-ingress foundations.
Selected acceptance reports identify the available runtime evidence and its
traceability limits.

The public interfaces and designs also describe planned runtime work: task
execution, perception-driven target detection, Safety supervision, mission
orchestration, and external adapters. Those components are not yet a complete
production workcell runtime.

This project does not implement or validate safety-rated E-stop or STO
hardware. Software supervision must not be treated as a substitute for
safety-rated equipment.

## Architecture and contracts

- Public overview: `docs/product/arm-cell-overview.md`
- System architecture: `docs/architecture/`
- ARM Cell components: `docs/components/arm-cell/`
- Shared ROS contracts: `docs/interfaces/arm-cell/`
- Architecture decisions: `docs/adr/`
- Isaac Sim procedure: `docs/guides/arm-cell-isaac-sim.md`
- Historical validation evidence and limitations:
  `docs/reports/acceptance/`

## Build the supported ROS subset

Install ROS 2 Humble and the dependencies named by the package manifests,
including Xacro, `robot_state_publisher`, TF2, and the relevant MoveIt
packages. Then build from the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to arm_cell_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

The Isaac procedure documents simulator-side prerequisites, generated-artifact
regeneration, and the supported planning profiles. Isaac-generated USD and
full runtime captures are intentionally not distributed; regenerate or capture
them in your local environment.

## Validation evidence

The selected acceptance reports record historical ARM Cell Milestone 1A
(robot-only planning profile) and 1B (bounded static planning scene) evidence.
They state their limits, including missing exact historical baseline identifiers.
Runtime geometry evidence remains simulator/operator dependent.

## License and notices

Project-owned code is available under the [MIT License](LICENSE). Third-party
models and assets retain their own terms; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
