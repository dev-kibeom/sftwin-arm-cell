"""Read the exported runtime data contract without importing ROS or Bringup code."""
import hashlib
import json
import os
from pathlib import Path


def selected_configuration(project_root=None):
    explicit = os.environ.get('ARM_CELL_RUNTIME_CONFIG')
    if explicit:
        return Path(explicit).expanduser().resolve()
    root = project_root or os.environ.get('SFTWIN_PROJECT_ROOT')
    if not root:
        raise ValueError('ARM_CELL_RUNTIME_CONFIG or SFTWIN_PROJECT_ROOT is required')
    return Path(root).expanduser().resolve() / '.run_records/runtime_configuration.json'


def load_runtime_configuration(path=None):
    path = Path(path) if path is not None else selected_configuration()
    if not path.is_file():
        raise ValueError(f'Export runtime configuration with arm_cell_bringup export_runtime_configuration first: {path}')
    document = json.loads(path.read_text())
    if document.get('kind') != 'arm-cell-runtime-configuration' or document.get('schema_version') != 1:
        raise ValueError('Unsupported runtime configuration contract')
    configuration = document['configuration']
    canonical = json.dumps(configuration, sort_keys=True, allow_nan=False).encode()
    if hashlib.sha256(canonical).hexdigest() != document.get('configuration_sha256'):
        raise ValueError('Runtime configuration checksum mismatch')
    if not isinstance(configuration.get('_mission_recipe'), dict):
        raise ValueError('Runtime configuration requires an embedded mission recipe')
    if configuration['_mission_recipe']['target_id'] != configuration['orchestration']['mission_target_id']:
        raise ValueError('Runtime configuration mission target mismatch')
    return configuration
