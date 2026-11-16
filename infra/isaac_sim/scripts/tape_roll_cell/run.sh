#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
project_root="$(cd -- "$script_dir/../../../.." && pwd -P)"
if [[ $# -lt 2 ]]; then
    echo 'Usage: tape_roll_cell/run.sh CELL_EXECUTION_OR_MODEL_EXPERIMENT_PROFILE TASK_FIXTURE_LIBRARY [--headless] [--cycles N]' >&2
    exit 2
fi
# Prepare source-export dependencies without discarding a caller's native test overlay.
set +u
source /opt/ros/humble/local_setup.bash
if [[ -f "$project_root/ros2_ws/install/local_setup.bash" ]]; then
    source "$project_root/ros2_ws/install/local_setup.bash"
fi
set -u
profile="$1"
library="$2"
shift 2
mkdir -p "$project_root/.run_records"
record_dir="$(mktemp -d "$project_root/.run_records/tape-roll-XXXXXXXX")"
export SFTWIN_PROJECT_ROOT="$project_root"
export ARM_CELL_EXPERIMENT_REVISION="$(git -C "$project_root" rev-parse HEAD)"
export PYTHONPATH="$project_root/ros2_ws/src/bringup:$project_root/infra/isaac_sim/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -m tape_roll_cell.configuration --profile "$profile" --output "$record_dir/configuration.json"
"${ISAAC_SIM_ROOT:-$HOME/isaac_sim}/python.sh" "$script_dir/../model_joint_experiment.py" \
    --configuration "$record_dir/configuration.json" --report "$record_dir/result.json" \
    --experiment tape_roll_cell.experiment:run --task-execution-library "$library" "$@"
python3 - "$record_dir/result.json" <<'PYRESULT'
import json,sys
report=json.load(open(sys.argv[1]))
print('Tape roll result:',report['status'])
raise SystemExit(0 if report['status']=='PASS' else 1)
PYRESULT
