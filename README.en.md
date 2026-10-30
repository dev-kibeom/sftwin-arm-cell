# SF-Twin ARM Cell

[한국어](README.md) | [English](README.en.md)

A simulator-backed ROS 2 workcell for developing and evaluating a Doosan M0609 robot with a Robotiq 2F-85 gripper. The project brings perception, motion planning, task orchestration, software safety supervision, material handoff, and an operator UI together around shared ROS interfaces.

The v0.3.0 release candidate demonstrates how to build a pick-and-place cell whose robot state, simulation time, transforms, camera observations, motion permission, and mission progress have explicit owners. It is an engineering and simulation project; it is not a validated production or safety-rated robot cell.

## Project Overview

The ARM Cell uses Isaac Sim as its simulated robot and sensor environment and ROS 2 as the integration and runtime layer. MoveIt plans and executes robot motion. The software is organized so that Vision reports observations, Orchestration sequences mission work, Motion performs robot tasks, Safety controls software motion capability, and Integration owns material-transfer readiness.

The current profile includes a simulated material-delivery path and an Isaac Sim UI Hub for operator observation and requests. External AMR/VDA, PLC, and safety-hardware state are represented by a mock adapter. The project does not implement AMR navigation or safety-rated E-stop/STO hardware.

## Key Features

- Shared Doosan M0609 and Robotiq 2F-85 robot description for ROS planning and Isaac Sim construction.
- ROS integration for canonical joint state, simulation time, TF, and RGB-D camera input.
- MoveIt 2 robot planning with explicitly scoped planning-scene profiles.
- A Vision/Motion boundary where Vision reports observed geometry and Motion builds request-specific planning occupancy excluding the selected candidate.
- Geometry-first RGB-D RawPart detection, partial-observation shape hypotheses, detector profiles, and source-frame diagnostics.
- PICK, PLACE, GO_HOME, and recovery motion tasks behind backend-neutral ROS interfaces.
- Reusable Behavior Tree task actions plus PICK candidate ordering and occupancy-exclusion heuristics.
- Software Safety supervision with fail-closed startup, motion capability, and a direct stop path to Motion.
- Acquisition-linked diagnostic snapshots and evidence records, simulated material handoff, and an Isaac Sim UI Hub.

## System Architecture

The runtime keeps perception, mission decisions, robot execution, and motion permission in separate ROS components. Isaac Sim supplies simulated time, robot state, and camera data; the integration layer adapts those inputs to the shared ROS contracts and provides simulator-specific gripper behavior.

```mermaid
flowchart LR
    Isaac[Isaac Sim: robot, time, RGB-D] --> Integration[Simulation Integration]
    Integration --> Vision[Vision]
    Vision --> Orchestration[Orchestration]
    Orchestration --> Motion[Motion / MoveIt 2]
    Motion --> Isaac
    VDA[VDA / external-state mock] --> Material[Material Integration]
    Hub[Isaac UI Hub] --> Material
    Material --> Orchestration
    Safety[Safety supervisor] -->|motion capability| Motion
    Safety -->|direct stop request| Motion
    Orchestration --> Hub
    Vision --> Hub
    Motion --> Hub
    Safety --> Hub
    Material --> Hub
```

| Component | Responsibility and interaction |
|---|---|
| **Vision** | Acquires fresh RGB-D observations on `DetectTarget` requests, applies the selected detector profile, and returns a target observation with source-frame diagnostics. It does not grant motion permission. |
| **Motion** | Uses MoveIt 2 to plan and execute PICK, PLACE, GO_HOME, and related tasks. It applies robot/tool geometry and consumes Safety's motion capability. |
| **Orchestration** | Owns the mission cycle: requests targets from Vision, submits tasks to Motion, tracks outcomes, and coordinates retry or authorized recovery. It admits a material batch only when current readiness and Safety conditions allow it. |
| **Safety** | Evaluates required input freshness and operational conditions, publishes the sole software motion capability, and sends stop requests directly to Motion. It does not replace safety-rated hardware. |
| **Material Integration** | Connects operator supply requests to the external-state mock and publishes `MATERIAL_READY` only after the simulated handoff completes. Orchestration, not Integration, admits the mission. |
| **UI Hub** | Runs as an Isaac Sim/Kit extension. It presents camera and component state and sends requests to their owning components; it does not author canonical runtime state. |

## Design Records and Sim-to-Real Direction

The [Engineering Stories](docs/record/arm-cell/engineering-story/) and [architecture/algorithm decision records](docs/record/arm-cell/) cover geometry-first Vision, Vision/Motion occupancy ownership, reusable Motion BT actions, Orchestration and Safety recovery, and the Sim-to-Real boundary. These historical records do not replace current contracts or establish physical-robot support or performance superiority.

## Technology Stack

- **ROS 2 Humble** — component communication, shared messages/actions/services, and workspace build.
- **Isaac Sim 5.1** — simulated M0609/2F-85 cell, robot state, simulation time, and RGB-D source.
- **MoveIt 2** — robot kinematics, planning-scene integration, motion planning, and execution.
- **C++ / `rclcpp`** — ROS runtime components, including Vision, Motion, Orchestration, Safety, and Integration.
- **Python** — Isaac Sim scene/runtime entrypoints, validation tooling, and the UI Hub extension.
- **Xacro, TF2, OpenCV, and OMPL** — robot description, transforms, image processing, and planning support.

## Getting Started

Clone the public repository:

```bash
git clone https://github.com/dev-kibeom/sftwin-arm-cell.git
cd sftwin-arm-cell
```

### Build the ROS workspace on a Humble host

With ROS 2 Humble and the package dependencies installed, build the supported workspace subset:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to arm_cell_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

### Build with Docker

The root Dockerfile builds the same ROS subset in a Humble container. It verifies image construction and compilation; it does not provide Isaac Sim or run ROS nodes.

Run the following from the repository root:

```bash
docker build --progress=plain -t sftwin-arm-cell:humble .
```

See the [Docker build guide](docs/guides/docker-humble-build.md) for prerequisites and build output details.

### Run with Isaac Sim

Live operation requires ROS 2 Humble and Isaac Sim 5.1 on the host. The ROS build alone does not start the simulator or establish a live runtime. Follow the [ARM Cell Isaac Sim guide](docs/guides/arm-cell-isaac-sim.md) for prerequisites, the supported launch profiles, and the Isaac Script Editor entrypoints (`1_before_play.py` → Play → `2_after_play.py`).

## Documentation

- [v0.3.0 release notes (candidate)](docs/releases/v0.3.0.en.md) / [한국어](docs/releases/v0.3.0.ko.md)

- [ARM Cell demo definition](docs/product/arm-cell/definition.md)
- [System architecture](docs/design/arm-cell/architecture/) and [ARM Cell component designs](docs/design/arm-cell/components/)
- [ROS and simulator interface contracts](docs/design/arm-cell/interfaces/)
- [Architecture Decision Records (ADRs)](docs/record/arm-cell/architecture-decision/)
- [Engineering Stories](docs/record/arm-cell/engineering-story/)
- [Technical records](docs/record/arm-cell/)
- [Acceptance reports](docs/reports/acceptance/)
- [Operator guides](docs/guides/)

## Current Status & Limitations

The public source contains the Vision, Motion, Orchestration, Safety, Material Integration, VDA mock, and UI Hub implementations described above. Their presence in source does not mean the complete final demonstration has passed live acceptance.

- **Build:** Public PR #2 recorded a successful Docker image build and a ROS 2 Humble `colcon build` of 11 packages through `arm_cell_bringup`. The public v0.2.0 export also recorded a successful ROS build. These results verify compilation, not a running Isaac Sim session. The ROS test suite still has failures that need resolution before release publication. The Humble Docker compile image does not include the unavailable TRAC-IK runtime plugin.
- **Planning evidence:** Historical Milestone 1A robot-only planning and Milestone 1B bounded static-scene acceptance are documented in the [acceptance reports](docs/reports/acceptance/). The reports preserve their evidence limits: exact historical commit/tag identifiers are unavailable, and Milestone 1B does not establish general reachability or full-cell collision coverage.
- **Live runtime:** This v0.3.0 candidate work did not run Isaac Sim live. Use the Isaac guide to run the supported profile in a configured environment; a build or historical planning result is not full mission acceptance.
- **RGB-D known issue:** Vision can intermittently fail to acquire a usable RGB-D pair before detector processing starts. Distinct depth-reception gaps and missing synchronization callbacks are recorded; a common cause and resolution remain unconfirmed. See the [open acquisition issue record](docs/record/arm-cell/engineering-story/vision-rgbd-intermittent-acquisition-failure.md).
- **E-Stop recovery evidence:** The latest clean live record from 2026-10-30 shows reset, SafetyRecovery, and GO_HOME succeeding, after which the Mission resumed PnP. A later approach ended with `BACKEND_UNAVAILABLE`, so the complete Mission did not succeed. This supports only that execution condition, not a general success rate.
- **Simulation and safety boundary:** The VDA/external-state adapter is a mock subset; it is not full VDA 5050 or AMR navigation. Software Safety is not safety-rated E-stop/STO hardware and is not a substitute for it. Isaac Sim is the supported development environment; physical robot operation is not validated here.

## License & Third-Party Attribution

Project-owned code is available under the [MIT License](LICENSE). Robot models and other third-party assets retain their upstream terms. See [Third-Party Notices](THIRD_PARTY_NOTICES.md) and the retained upstream license/provenance files before redistributing those materials.
