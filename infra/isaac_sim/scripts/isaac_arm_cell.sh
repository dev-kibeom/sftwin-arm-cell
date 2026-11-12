#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd -P)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaac_sim}"
ISAAC_LAUNCHER="$ISAAC_SIM_ROOT/isaac-sim.sh"
STARTUP_SCRIPT="$SCRIPT_DIR/isaac_arm_cell_startup.py"

if [[ ! -x "$ISAAC_LAUNCHER" ]]; then
    printf 'Isaac Sim launcher is missing or not executable: %s\n' "$ISAAC_LAUNCHER" >&2
    printf 'Set ISAAC_SIM_ROOT to the directory containing isaac-sim.sh.\n' >&2
    exit 127
fi

if [[ ! -f "$STARTUP_SCRIPT" ]]; then
    printf 'ARM Cell startup script is missing: %s\n' "$STARTUP_SCRIPT" >&2
    exit 2
fi

export SFTWIN_PROJECT_ROOT="$PROJECT_ROOT"
# Run the public producer CLI in a separate process. Isaac receives only JSON;
# the temporary Python search path is not inherited by the simulator.
export ARM_CELL_RUNTIME_CONFIG="${ARM_CELL_RUNTIME_CONFIG:-$PROJECT_ROOT/.run_records/runtime_configuration.json}"
PYTHONPATH="$PROJECT_ROOT/ros2_ws/src/bringup${PYTHONPATH:+:$PYTHONPATH}" \
    python3 "$PROJECT_ROOT/ros2_ws/src/bringup/scripts/export_runtime_configuration" \
    --profile "${ARM_CELL_PROFILE:-$PROJECT_ROOT/ros2_ws/src/bringup/config/operational_profile.yaml}" \
    --output "$ARM_CELL_RUNTIME_CONFIG"
exec "$ISAAC_LAUNCHER" "$@" --exec "$STARTUP_SCRIPT"
