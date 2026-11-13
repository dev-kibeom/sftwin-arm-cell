"""Selected Cell inputs survive composition and export with one sensor owner."""
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from arm_cell_bringup.runtime import export_runtime_configuration, load_profile

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def test_export_preserves_selected_cell_and_sensor_consumers(tmp_path):
    profile = yaml.safe_load((CONFIG / 'operational_profile.yaml').read_text())
    cell = yaml.safe_load((CONFIG / 'cells/isaac_m0609.yaml').read_text())
    cell['camera']['prim_path'] = '/World/Selected/Camera'
    cell['camera']['mount_prim_path'] = '/World/Selected/Mount'
    cell['camera']['rgb_topic'] = '/selected/rgb'
    cell['material_supply']['delivery_root'] = '/World/Selected/Deliveries'
    cell['camera']['snapshot'] = str(tmp_path / 'snapshot.json')
    cell_path = tmp_path / 'cell.yaml'
    cell_path.write_text(yaml.safe_dump(cell))
    profile['profiles']['cell'] = str(cell_path)
    source = tmp_path / 'profile.yaml'
    source.write_text(yaml.safe_dump(profile))
    path = export_runtime_configuration(source, tmp_path / 'runtime.json')
    config = load_profile(path)
    assert config['cell']['camera']['prim_path'] == cell['camera']['prim_path']
    assert config['vision']['rgb_topic'] == cell['camera']['rgb_topic']
    assert config['cell']['material_supply'] == cell['material_supply']
    assert config['cell']['camera']['snapshot'] == cell['camera']['snapshot']
    assert json.loads(path.read_text())['configuration']['cell'] == config['cell']
    # Exercise external consumers in an isolated interpreter against this export.
    code = """
import os, sys
sys.path.insert(0, sys.argv[1])
os.environ['ARM_CELL_RUNTIME_CONFIG'] = sys.argv[2]
from runtime_configuration import selected_camera, selected_delivery_root
from graph_builder.ros_action_graph import graph_declaration
from camera_tooling.stage_inspector import script_editor_output_path, optical_transform_snapshot
camera = selected_camera()
values = dict(graph_declaration(str)[1])
assert values['CreateRenderProduct.inputs:cameraPrim'] == [camera['prim_path']]
assert values['CameraHelperColor.inputs:topicName'] == camera['rgb_topic']
assert script_editor_output_path(os.environ) == camera['snapshot']
assert optical_transform_snapshot()['child_frame'] == camera['frame_id']
assert selected_delivery_root() == '/World/Selected/Deliveries'
"""
    cell_modules = CONFIG.parents[3] / 'infra/isaac_sim/scripts/m0609_cell'
    result = subprocess.run([sys.executable, '-I', '-c', code, str(cell_modules), str(path)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_composition_rejects_camera_calibration_resolution_mismatch(tmp_path):
    profile = yaml.safe_load((CONFIG / 'operational_profile.yaml').read_text())
    cell = yaml.safe_load((CONFIG / 'cells/isaac_m0609.yaml').read_text())
    cell['camera']['resolution'] = [640, 360]
    cell_path = tmp_path / 'cell.yaml'
    cell_path.write_text(yaml.safe_dump(cell))
    profile['profiles']['cell'] = str(cell_path)
    source = tmp_path / 'profile.yaml'
    source.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='Calibration.*resolution'):
        load_profile(source)
