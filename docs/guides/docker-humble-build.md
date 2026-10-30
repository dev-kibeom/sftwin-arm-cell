# ROS 2 Humble Docker Build Guide

This guide builds the current ARM Cell ROS 2 workspace with the repository's Dockerfile. It produces a build image; it does not start ROS nodes or Isaac Sim.

## Requirements

- A Docker installation whose daemon is running and can build Linux containers. The image is based on `ros:humble-ros-base-jammy` (ROS 2 Humble on Ubuntu 22.04 Jammy).
- Docker BuildKit support for the `--progress=plain` option.
- An internet connection from the Docker build environment for Ubuntu packages, ROS dependencies, and rosdep data.

ROS 2 and Isaac Sim do not need to be installed on the host for this image build. The recorded v0.3.0 build completed in the documented Docker environment; other Docker host setups were not separately validated.

## Build

Clone the public repository and build from its root directory:

```bash
git clone https://github.com/dev-kibeom/sftwin-arm-cell.git
cd sftwin-arm-cell
docker build --progress=plain -t sftwin-arm-cell:humble .
```

The Dockerfile installs build tools and dependencies with `rosdep`. It explicitly installs `nlohmann-json3-dev` because Humble rosdep has no definition for the `nlohmann_json` key. Humble's configured apt repositories do not publish `trac_ik_kinematics_plugin`, so the Dockerfile fetches the upstream TRACLabs ROS 2 source at pinned revision `210f767d1d4e89ab30aa0a951aaa62716b4ec7a7` (TRAC-IK 2.0.1), builds `trac_ik_lib` and `trac_ik_kinematics_plugin`, and installs them in `/opt/trac_ik`. The workspace rosdep step skips the `trac_ik_kinematics_plugin` key because that package is supplied by this source overlay. This overlay build is separate from the ARM Cell build, which remains:

```bash
colcon build --packages-up-to arm_cell_bringup
```

## Confirm success

A successful build exits with status 0 and BuildKit reports the resulting image as `sftwin-arm-cell:humble`. The build log reports the ARM Cell `colcon` summary; the Docker verification completed 12 packages through `arm_cell_bringup`. The separate upstream overlay build compiles two TRAC-IK packages.

## Verify TRAC-IK at runtime

Check that ROS can find the package, then run the MoveIt plugin load and initialization probe against the installed ARM Cell robot model:

```bash
docker run --rm \
  -v "$PWD/infra/docker/verify-trac-ik-plugin.sh:/tmp/verify-trac-ik-plugin.sh:ro" \
  sftwin-arm-cell:humble \
  bash -lc 'source /workspace/ros2_ws/install/setup.bash && ros2 pkg prefix trac_ik_kinematics_plugin && /tmp/verify-trac-ik-plugin.sh'
```

The script checks the installed shared library and plugin description, creates the plugin through MoveIt's `pluginlib::ClassLoader`, and calls `initialize()` with the ARM Cell URDF/SRDF, the `arm` group, `base_link`, `sf_grasp_tcp`, and `solve_type=Distance`. This tests plugin loading and IK initialization; it does not request or validate an IK solution for a target pose.

## Scope of this check

The Docker verification completed image construction and the 12-package `arm_cell_bringup` build closure, TRAC-IK package installation, and MoveIt plugin loading and initialization. It also attempted the MoveIt `planning_only.launch.py`; that launch did not remain running because its execution configuration has no `moveit_controller_manager` parameter. The successful plugin initialization probe is independent of that launch limitation. No ROS operational launch, IK target solution, simulator connection, Isaac Sim live run, or PnP success was validated. The checked-in `kinematics.yaml` remains KDL for the general MoveIt group, while the default operational profile selects TRAC-IK for PICK and Motion initializes that plugin separately. For Isaac Sim setup and live operation, see the [ARM Cell Isaac Sim guide](arm-cell-isaac-sim.md).

## Common environment issues

- **Cannot connect to the Docker daemon / permission denied on `docker.sock`:** start the Docker service or Docker Desktop and ensure your user can access the daemon. On Linux, use an authorized Docker group membership or run the Docker command with `sudo`.
- **BuildKit progress option is unsupported:** enable BuildKit in the Docker installation or update it to a version that supports `docker build --progress=plain`.
- **`apt-get`, `rosdep update`, or dependency downloads fail:** check that the Docker daemon's build environment has network access to Ubuntu package repositories and ROS/rosdep sources, then retry the same build command.
