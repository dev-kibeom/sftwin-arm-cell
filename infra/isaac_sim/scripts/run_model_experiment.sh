#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd -P)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaac_sim}"
if [[ $# -lt 1 ]]; then
    echo 'Usage: run_model_experiment.sh PROFILE [--headless] [--cycles N] [--internal-grasp]' >&2
    exit 2
fi
profile="$1"
shift
mkdir -p "$PROJECT_ROOT/.run_records"
record_dir="$(mktemp -d "$PROJECT_ROOT/.run_records/model-experiment-XXXXXXXX")"
export SFTWIN_PROJECT_ROOT="$PROJECT_ROOT"
export ARM_CELL_EXPERIMENT_REVISION="$(git -C "$PROJECT_ROOT" rev-parse HEAD)"
PYTHONPATH="$PROJECT_ROOT/ros2_ws/src/bringup${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m arm_cell_bringup.model_experiment --profile "$profile" --output "$record_dir/configuration.json"
"$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/model_joint_experiment.py" \
    --configuration "$record_dir/configuration.json" --report "$record_dir/result.json" "$@"
# Some Kit shutdown paths exit without propagating the Python exception status.
# The result, rather than Kit's exit alone, owns the experiment conclusion.
python3 - "$record_dir/result.json" <<'PYRESULT'
import json
from pathlib import Path
import sys
report = json.loads(Path(sys.argv[1]).read_text())
print('Model experiment result:', report['status'])
if report['status'] != 'PASS':
    raise SystemExit(1)
PYRESULT
