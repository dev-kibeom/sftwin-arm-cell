# ROS 2 Humble Docker Build Guide

This guide builds the public v0.2.0 ARM Cell ROS 2 workspace with the repository's Dockerfile. It produces a build image; it does not start ROS nodes or Isaac Sim.

## Requirements

- A Docker installation whose daemon is running and can build Linux containers. The image is based on `ros:humble-ros-base-jammy` (ROS 2 Humble on Ubuntu 22.04 Jammy).
- Docker BuildKit support for the `--progress=plain` option.
- An internet connection from the Docker build environment for Ubuntu packages, ROS dependencies, and rosdep data.

ROS 2 and Isaac Sim do not need to be installed on the host for this image build. The public PR #2 build completed on its recorded environment; other Docker host setups were not separately validated.

## Build

Clone the public repository and build from its root directory:

```bash
git clone https://github.com/dev-kibeom/sftwin-arm-cell.git
cd sftwin-arm-cell
docker build --progress=plain -t sftwin-arm-cell:humble .
```

The Dockerfile installs build tools and dependencies with `rosdep`, then runs:

```bash
colcon build --packages-up-to arm_cell_bringup
```

## Confirm success

A successful build exits with status 0 and BuildKit reports the resulting image as `sftwin-arm-cell:humble`. The build log also reports the `colcon` package summary; the PR #2 verification completed 11 packages.

## Scope of this check

This verifies image construction and compilation of the ROS workspace through `arm_cell_bringup`. It does not run the resulting ROS applications, connect to ROS middleware, or launch Isaac Sim. For Isaac Sim setup and live operation, see the [ARM Cell Isaac Sim guide](arm-cell-isaac-sim.md).

## Common environment issues

- **Cannot connect to the Docker daemon / permission denied on `docker.sock`:** start the Docker service or Docker Desktop and ensure your user can access the daemon. On Linux, use an authorized Docker group membership or run the Docker command with `sudo`.
- **BuildKit progress option is unsupported:** enable BuildKit in the Docker installation or update it to a version that supports `docker build --progress=plain`.
- **`apt-get`, `rosdep update`, or dependency downloads fail:** check that the Docker daemon's build environment has network access to Ubuntu package repositories and ROS/rosdep sources, then retry the same build command.
