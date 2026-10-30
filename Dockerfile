FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace
COPY ros2_ws/src/ ros2_ws/src/

# ROS 2 Humble has no binary TRAC-IK plugin package in the configured apt repositories.
# Build the upstream MoveIt plugin into a separate overlay so ARM Cell's 12-package build stays isolated.
# The nlohmann_json rosdep key is unavailable; nlohmann-json3-dev is installed explicitly below.
RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        nlohmann-json3-dev \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && git init /workspace/trac_ik_ws/src/trac_ik \
    && git -C /workspace/trac_ik_ws/src/trac_ik remote add origin https://github.com/traclabs/trac_ik.git \
    && git -C /workspace/trac_ik_ws/src/trac_ik fetch --depth=1 origin 210f767d1d4e89ab30aa0a951aaa62716b4ec7a7 \
    && git -C /workspace/trac_ik_ws/src/trac_ik checkout --detach FETCH_HEAD \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths ros2_ws/src trac_ik_ws/src/trac_ik/trac_ik_lib trac_ik_ws/src/trac_ik/trac_ik_kinematics_plugin --ignore-src --rosdistro humble --skip-keys='nlohmann_json' -y \
    && colcon --log-base /tmp/trac_ik_log build --base-paths trac_ik_ws/src/trac_ik/trac_ik_lib trac_ik_ws/src/trac_ik/trac_ik_kinematics_plugin --packages-up-to trac_ik_kinematics_plugin --merge-install --install-base /opt/trac_ik \
    && . /opt/trac_ik/setup.sh \
    && ros2 pkg prefix trac_ik_kinematics_plugin \
    && . /opt/ros/humble/setup.sh \
    && . /opt/trac_ik/setup.sh \
    && rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble --skip-keys='nlohmann_json trac_ik_kinematics_plugin' -y \
    && apt-get purge -y git \
    && apt-get autoremove --purge -y \
    && rm -rf /workspace/trac_ik_ws /tmp/trac_ik_log \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/ros2_ws
RUN . /opt/ros/humble/setup.sh \
    && . /opt/trac_ik/setup.sh \
    && colcon build --packages-up-to arm_cell_bringup
