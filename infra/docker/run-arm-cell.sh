#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/trac_ik/setup.bash
source /workspace/ros2_ws/install/setup.bash

export PYTHONPATH="/arm_cell/infra/isaac_sim/extensions/arm_cell.simulation_ui_hub/python${PYTHONPATH:+:$PYTHONPATH}"
export SFTWIN_PROJECT_ROOT=/arm_cell
export SFTWIN_MATERIAL_SIDECAR_HOST=127.0.0.1
export SFTWIN_MATERIAL_SIDECAR_PORT="${SFTWIN_MATERIAL_SIDECAR_PORT:-47631}"
export SFTWIN_HUB_SIDECAR_HOST=127.0.0.1
export SFTWIN_HUB_SIDECAR_PORT="${SFTWIN_HUB_SIDECAR_PORT:-47632}"
log_level="${ARM_CELL_LOG_LEVEL:-info}"
diagnostics="${ARM_CELL_DIAGNOSTICS:-__log_level_default__}"

python3.10 /arm_cell/infra/isaac_sim/scripts/m0609_cell/material_supply_ros_sidecar.py --listen &
material_pid=$!
python3.10 -m arm_cell_simulation_ui_hub.sidecar --listen &
hub_pid=$!

cleanup() {
    trap - EXIT INT TERM
    kill "$ros_pid" "$material_pid" "$hub_pid" 2>/dev/null || true
    wait "$ros_pid" "$material_pid" "$hub_pid" 2>/dev/null || true
}
ros2 launch arm_cell_bringup arm_cell.launch.py \
    "log_level:=${log_level}" \
    "diagnostics:=${diagnostics}" &
ros_pid=$!
trap cleanup EXIT INT TERM

wait "$ros_pid"
