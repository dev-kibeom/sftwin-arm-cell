# SF-Twin ARM Cell

[한국어](README.md) | [English](README.en.md)

A simulator-backed ROS 2 workcell for developing and evaluating a Doosan M0609 robot with a Robotiq 2F-85 gripper. The project brings perception, motion planning, task orchestration, software safety supervision, material handoff, and an operator UI together around shared ROS interfaces.

The current official release is v0.3.4. It improves Vision/PICK candidate selection, PLACE recovery, per-run diagnostic records, local D455 assets, and runtime logging. This is an engineering and simulation project, not a validated production or safety-rated robot cell.

## Key Features

- Doosan M0609, Robotiq 2F-85, and RGB-D camera simulation in Isaac Sim
- Depth-image cuboid detection and MoveIt 2 PICK, PLACE, and return motions
- Material handoff, quantity-based task flow, and software Safety supervision
- GUI launcher for starting and stopping simulator and ROS processes
- Camera/Vision views and operator requests through the Isaac Sim UI Hub

External AMR/VDA, PLC, and safety-hardware state use mock adapters. Physical-robot
operation, AMR navigation, and safety-rated E-stop/STO hardware are outside the supported scope.

[Watch the demo](https://youtu.be/6eOcrNFdf2g)

## Prerequisites

- Linux host; the commands below target Ubuntu 22.04
- ROS 2 Humble, Isaac Sim 5.1, and a GPU/driver suitable for Isaac Sim
- GUI launcher: Python 3.10 or later and an X11 or XWayland display
- Host build: colcon, initialized rosdep, and `nlohmann-json3-dev`
- Docker option: Docker Compose v2

```bash
git clone https://github.com/dev-kibeom/sftwin-arm-cell.git
cd sftwin-arm-cell
```

Run commands from the repository root unless noted otherwise.

## Run ARM Cell

Choose one path. The GUI launcher uses a ROS workspace built on the host.
To run ROS in Docker, use [manual startup](#manual-startup-host-or-docker).
Do not start the same cell through both the GUI and manual commands.

### GUI Launcher

**1. Build the ROS workspace** in a fresh terminal.

```bash
bash ros2_ws/build-local.bash
```

A successful build creates `ros2_ws/install/setup.bash`.

**2. Install launcher dependencies.** Python dependencies are PySide6, PyYAML,
and xacro. Use the same Python environment for installation and startup.

```bash
sudo apt-get install python3-pip libegl1 libxcb-cursor0 xterm
```

A virtual environment is optional; choose its location yourself. If using one,
activate it before installing Python dependencies. For example:

```bash
sudo apt-get install python3-venv
python3 -m venv .venv
source .venv/bin/activate
```

```bash
python3 -m pip install -r infra/launcher/requirements.txt
```

Use the same requirements command when updating an existing installation.

**3. Start the launcher.**

```bash
python3 launcher.py
```

The default Isaac Sim installation is `~/isaac_sim`. Override the location or
ROS domain when starting the GUI if needed:

```bash
ISAAC_SIM_ROOT=/path/to/isaac_sim ROS_DOMAIN_ID=0 python3 launcher.py
```

**4. Start the cell.** Button labels currently appear in Korean.

1. Select Robot, Tool, Cell, and Backend. The supported combination is M0609 +
   Robotiq 2F-85 + the `isaac_m0609` Cell.
2. Click `Isaac 시작` (Start Isaac). Wait for the scene, Timeline, and After Playing initialization.
3. Click `ROS 2 시작` (Start ROS 2). For detailed logs, select `ROS 2 디버그 모드` before starting.
4. Check the camera/Vision views in Isaac Sim's UI Hub and errors in both run terminals.
   The launcher reports process execution; this does not establish system readiness or PnP success.

Isaac and ROS each open an `xterm` window for output and errors. Use `모두 종료`
(Stop all) or close the launcher to stop. `Ctrl+C` or closing either run terminal
also initiates shutdown. Allow shutdown to finish.

### Manual Startup: Host or Docker

**1. Prepare the ROS environment.** For host execution, build first. Docker does
not require a host workspace build.

```bash
bash ros2_ws/build-local.bash
```

**2. Start Isaac Sim in the first terminal.**

```bash
source /opt/ros/humble/setup.bash
./infra/isaac_sim/scripts/isaac_arm_cell.sh
```

For a different installation, prefix the script with `ISAAC_SIM_ROOT=/path/to/isaac_sim`.
Wait for scene, Timeline, and After Playing initialization before continuing.

**3. Start ROS in a second terminal.** Choose one of the following.

Host:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch arm_cell_bringup arm_cell.launch.py log_level:=info
```

Docker:

```bash
export ARM_CELL_UID="$(id -u)" ARM_CELL_GID="$(id -g)"
docker compose up --build
```

Isaac and ROS must use matching `ROS_DOMAIN_ID` and RMW settings. Compose defaults
to `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
Use `docker compose logs -f arm-cell` to follow Docker logs.

Stop ROS with `Ctrl+C` in its terminal. For Docker, use `Ctrl+C` in the Compose
terminal or `docker compose down`. Close Isaac after ROS has stopped.

## Basic Use

The **UI Hub** inside Isaac Sim provides camera/Vision views and operator controls.
The separate launcher manages process startup and shutdown.

- In `External Handoff Trigger`, select `Material type` and `Quantity`, then use
  `Random Drop` to publish a supply stimulus. After supply completes, Safety and readiness conditions
  govern task admission. Successful publication does not establish supply or task success.
- Inspect camera images and Vision Snapshots. Use ROS terminal output and diagnostic
  records for system state and task errors. The Hub does not provide an aggregate state dashboard.
- Use the standard Isaac Timeline Pause to pause, and Play to resume the same experiment.
- For a new experiment, the simplest path is Stop all in the launcher, then start again.
  To keep Isaac running, stop ROS → Timeline Stop → `1_before_play.py` → Play →
  `2_after_play.py` → start fresh ROS. Stop resets simulation state; it does not replace
  Safety Reset or a physical emergency stop.

See [UI Hub behavior](docs/design/components/ui-hub/fds.md) and
[simulation sessions](docs/design/components/simulation/fds-sim-session.md) for details.

## Configuration and Logs

- The GUI applies the selected configuration to both Isaac and ROS. Stop and restart after changing configuration files.
- Manual startup examples use the default operational profile. The Isaac startup script
  exports its configuration to `.run_records/runtime_configuration.json` by default.
- Material definitions are in `ros2_ws/src/bringup/config/materials/`. The UI Hub uses the catalog loaded at startup.
- Diagnostic records default to `.run_records/`. Launcher terminal output disappears when the terminal closes.
- For host ROS, use `log_level:=debug` for detailed logs. Default `info` disables optional
  diagnostic collection; `debug` enables all ROS diagnostic collection. Select records with
  `diagnostics:=none`, `all`, or a comma-separated subset of `motion,vision,safety,orchestration`.

For Docker, select logs and diagnostics with environment variables at startup.
Stop the existing run before changing settings:

```bash
ARM_CELL_LOG_LEVEL=debug docker compose up --build
ARM_CELL_LOG_LEVEL=info ARM_CELL_DIAGNOSTICS=vision,motion docker compose up --build
```

See [component architecture](docs/design/architecture/components.md) for configuration
responsibilities and [Observability Records ICD](docs/design/interfaces/icd-observability-records.md)
for record interpretation.

## Troubleshooting

| Symptom | Check |
|---|---|
| Python module missing in launcher | Install requirements with the same Python used to launch. Activate your virtual environment if applicable. |
| Missing `ros2_ws/install/setup.bash` | Complete the host build. The GUI does not use the workspace built inside a Docker image. |
| Isaac executable not found | Set `ISAAC_SIM_ROOT` to the directory containing `isaac-sim.sh`. |
| Run terminal does not open | Check `xterm` installation and an X11/XWayland display. |
| ROS startup blocked or already running | Stop the previous GUI, CLI, or Docker run. Processes from another checkout are not cleaned up automatically. |
| Images or robot state do not update | Check Timeline Play, completed After Playing initialization, and matching ROS domain/RMW settings. |
| Code changes do not take effect | Rebuild and source the overlay on the host; replace Docker with `docker compose up --build`. Reload the Isaac runtime or restart for Isaac Python changes. |
| Docker cannot write records | Stop Isaac and Compose, then check `.run_records/` and `.motion_data/` ownership and `ARM_CELL_UID`/`ARM_CELL_GID`. |

## Documentation

- [v0.3.4 release notes](docs/releases/v0.3.4.en.md) / [한국어](docs/releases/v0.3.4.ko.md): changes, upgrades, and verification scope
- [Project definition](docs/product/definition.md) and [live acceptance contract](docs/product/acceptance.md)
- [System architecture](docs/design/architecture/), [component designs](docs/design/components/), and [interfaces](docs/design/interfaces/)
- [Algorithm designs](docs/design/algorithms/)
- [Design decisions and Engineering Stories](docs/record/)
- [Acceptance reports](docs/reports/acceptance/)

## License and Third-Party Assets

Project-owned code uses the [MIT License](LICENSE). Robot models and third-party
assets retain their upstream terms. Check [Third-Party Notices](THIRD_PARTY_NOTICES.md)
and retained licenses and source information before redistribution.
