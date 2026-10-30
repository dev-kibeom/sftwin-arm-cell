FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace
COPY ros2_ws/src/ ros2_ws/src/

# Humble has no packaged TRAC-IK plugin; it is a launch-time runtime dependency, not needed to compile this image.
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        build-essential \
        python3-colcon-common-extensions \
        python3-rosdep \
        nlohmann-json3-dev \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble --skip-keys=trac_ik_kinematics_plugin -y \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/ros2_ws
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-up-to arm_cell_bringup
