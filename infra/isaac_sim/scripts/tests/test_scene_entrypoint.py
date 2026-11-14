"""Scene selection works with the scripts root, without a Cell sys.path entry."""
from pathlib import Path
import subprocess
import sys


SCRIPTS = Path(__file__).resolve().parents[1]


def _isolated(code, *args):
    result = subprocess.run([sys.executable, '-I', '-c', code, *map(str, args)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_alternate_builder_runs_in_checkout_without_m0609(tmp_path):
    (tmp_path / 'scene_entrypoint.py').write_text((SCRIPTS / 'scene_entrypoint.py').read_text())
    (tmp_path / 'other_cell.py').write_text(
        'def build(*, configuration, stage):\n'
        '    assert configuration["cell"]["isaac"]["scene_builder"] == "other_cell:build"\n'
        '    return stage\n')
    _isolated('''
import sys
sys.path.insert(0, sys.argv[1])
from scene_entrypoint import run_scene_build
config = {'cell': {'isaac': {'scene_builder': 'other_cell:build'}}}
stage = object()
assert run_scene_build(configuration=config, stage=stage) is stage
assert not any(name.startswith(('pnp_cnc_cell', 'scene_builder', 'shared')) for name in sys.modules)
''', tmp_path)


def test_default_builder_import_and_dispatch_need_only_scripts_root():
    _isolated('''
import sys
sys.path.insert(0, sys.argv[1])
from scene_entrypoint import run_scene_build
from pnp_cnc_cell.scene_builder import composition
config = {'cell': {'isaac': {'scene_builder': 'pnp_cnc_cell.scene_builder.composition:run_scene_build'}}}
stage = object()
composition.run_scene_build = lambda **kwargs: (kwargs['configuration'], kwargs['stage'])
assert run_scene_build(configuration=config, stage=stage) == (config, stage)
assert not any(name in sys.modules for name in ('scene_builder', 'shared', 'runtime_configuration', 'scene_selection'))
assert not any(path.endswith('/pnp_cnc_cell') for path in sys.path)
''', SCRIPTS)


def test_launcher_preserves_common_startup_and_cell_selection(tmp_path):
    import json
    import os
    import shlex

    snapshot = tmp_path / 'runtime.json'
    snapshot.write_text(json.dumps({'configuration': {
        'cell': {'isaac': {'scene_builder': 'pnp_cnc_cell.scene_builder.composition:run_scene_build'}}}}))
    isaac = tmp_path / 'isaac'
    isaac.mkdir()
    code = '''
import json, os, sys
from pathlib import Path
startup = Path(sys.argv[sys.argv.index('--exec') + 1])
assert startup == Path(os.environ['SFTWIN_PROJECT_ROOT']) / 'infra/isaac_sim/scripts/isaac_arm_cell_startup.py'
sys.path.insert(0, str(startup.parent))
from scene_entrypoint import run_scene_build
from pnp_cnc_cell.scene_builder import composition
config = json.loads(Path(os.environ['ARM_CELL_RUNTIME_CONFIG']).read_text())['configuration']
composition.run_scene_build = lambda **kwargs: kwargs['configuration']
assert run_scene_build(configuration=config) == config
assert not any(path.endswith('/pnp_cnc_cell') for path in sys.path)
'''
    launcher = isaac / 'isaac-sim.sh'
    launcher.write_text('#!/bin/sh\nexec ' + shlex.quote(sys.executable) + ' -I -c ' + shlex.quote(code) + ' "$@"\n')
    launcher.chmod(0o755)
    result = subprocess.run(['bash', str(SCRIPTS / 'isaac_arm_cell.sh'), '--runtime-config', str(snapshot)],
                            env={**os.environ, 'ISAAC_SIM_ROOT': str(isaac)},
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
