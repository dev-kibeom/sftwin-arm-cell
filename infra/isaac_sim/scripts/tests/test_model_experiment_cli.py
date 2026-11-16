"""Kit's exit code alone cannot declare successful model verification."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('wrapper', ['model', 'tape_roll'])
@pytest.mark.parametrize('status,exit_code', [('PASS', 0), ('FAIL', 1), ('ABORTED', 1)])
def test_wrapper_checks_measurement_result_after_kit_exit_zero(tmp_path, status, exit_code, wrapper):
    runner = tmp_path / 'python.sh'
    runner.write_text('''#!/usr/bin/env python3
import json
from pathlib import Path
import sys
report = Path(sys.argv[sys.argv.index('--report') + 1])
report.write_text(json.dumps({'status': STATUS}))
print('FAKE_KIT_REPORT=' + str(report), flush=True)
'''.replace('STATUS', repr(status)))
    runner.chmod(0o755)
    script = 'run_model_experiment.sh' if wrapper == 'model' else 'tape_roll_cell/run.sh'
    arguments = [str(ROOT / 'ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml')]
    if wrapper == 'tape_roll':
        arguments.append(str(tmp_path / 'fixture.so'))
    result = subprocess.run([
        'bash', str(ROOT / 'infra/isaac_sim/scripts' / script),
        *arguments, '--headless', '--cycles', '1',
    ], env=dict(os.environ, ISAAC_SIM_ROOT=str(tmp_path)), text=True, capture_output=True)
    report = next((Path(line.split('=', 1)[1]) for line in result.stdout.splitlines()
                   if line.startswith('FAKE_KIT_REPORT=')), None)
    try:
        assert result.returncode == exit_code, result.stdout + result.stderr
        label = 'Model experiment' if wrapper == 'model' else 'Tape roll'
        assert f'{label} result: {status}' in result.stdout
        assert json.loads(report.read_text())['status'] == status
    finally:
        if report is not None:
            shutil.rmtree(report.parent)
