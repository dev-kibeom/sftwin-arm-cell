#!/usr/bin/env bash
set -eo pipefail

workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
trac_workspace="$workspace/build/trac_ik_underlay"
trac_source="$trac_workspace/src/trac_ik"
trac_revision=210f767d1d4e89ab30aa0a951aaa62716b4ec7a7

source /opt/ros/humble/setup.bash

if [[ ! -d "$trac_source/.git" ]]; then
  git init "$trac_source"
fi
if ! git -C "$trac_source" cat-file -e "$trac_revision^{commit}" 2>/dev/null; then
  git -C "$trac_source" fetch --depth=1 https://github.com/traclabs/trac_ik.git "$trac_revision"
fi
git -C "$trac_source" checkout --detach "$trac_revision"

trac_packages=("$trac_source/trac_ik_lib" "$trac_source/trac_ik_kinematics_plugin")
rosdep install --from-paths "$workspace/src" "${trac_packages[@]}" \
  --ignore-src --rosdistro humble --skip-keys=nlohmann_json -y

cd "$trac_workspace"
colcon build --base-paths "${trac_packages[@]}" \
  --packages-up-to trac_ik_kinematics_plugin --merge-install
source install/local_setup.bash
ros2 pkg prefix trac_ik_kinematics_plugin

cd "$workspace"
colcon build --base-paths src --packages-up-to arm_cell_bringup \
  --cmake-args -DBUILD_TESTING=ON
