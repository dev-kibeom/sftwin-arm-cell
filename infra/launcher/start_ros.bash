#!/usr/bin/env bash
set -eo pipefail
# Use the existing host overlay and production CLI; no alternate bringup.
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source /opt/ros/humble/setup.bash
source "$project_root/ros2_ws/install/setup.bash"
if [[ -n "${3:-}" ]]; then
    export ARM_CELL_ROS_PREFLIGHT_FILE="$3"
fi
exec /usr/bin/python3 "$project_root/infra/launcher/ros_guard.py" ros2 launch arm_cell_bringup arm_cell.launch.py "profile:=$1" "log_level:=${2:-info}" "diagnostics:=${ARM_CELL_DIAGNOSTICS:-none}"
