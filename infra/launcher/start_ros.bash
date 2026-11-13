#!/usr/bin/env bash
set -eo pipefail
# Use the existing host overlay and production CLI; no alternate bringup.
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source /opt/ros/humble/setup.bash
source "$project_root/ros2_ws/install/setup.bash"
exec /usr/bin/python3 "$project_root/infra/launcher/ros_guard.py" ros2 launch arm_cell_bringup arm_cell.launch.py "profile:=$1" "log_level:=${2:-info}"
