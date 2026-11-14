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
    cell = yaml.safe_load((CONFIG / 'cells/isaac_pnp_cnc.yaml').read_text())
    cell['camera']['prim_path'] = '/World/Selected/Camera'
    cell['camera']['mount_prim_path'] = '/World/Selected/Mount'
    cell['camera']['rgb_topic'] = '/selected/rgb'
    cell['material_supply']['delivery_root'] = '/World/Selected/Deliveries'
    supply_profile = yaml.safe_load((CONFIG.parents[3] / 'infra/isaac_sim/config/material_supply.yaml').read_text())
    supply_profile['material_supply']['spawn_height_above_shelf_m'] = 0.17
    supply_path = tmp_path / 'supply.yaml'
    supply_path.write_text(yaml.safe_dump(supply_profile))
    cell['material_supply']['profile'] = str(supply_path)
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
    assert config['cell']['material_supply']['delivery_root'] == cell['material_supply']['delivery_root']
    assert config['cell']['camera']['snapshot'] == cell['camera']['snapshot']
    assert config['cell']['material_supply']['profile'] == supply_profile
    supply_path.unlink()  # External consumers must not reopen the source selection.
    assert json.loads(path.read_text())['configuration']['cell'] == config['cell']
    # Exercise external consumers in an isolated interpreter against this export.
    code = """
import os, sys
sys.path.insert(0, sys.argv[1])
os.environ['ARM_CELL_RUNTIME_CONFIG'] = sys.argv[2]
from runtime_configuration import selected_camera, selected_delivery_root, load_runtime_configuration
from graph_builder.ros_action_graph import graph_declaration
from camera_tooling.stage_inspector import script_editor_output_path, optical_transform_snapshot
camera = selected_camera()
values = dict(graph_declaration(str)[1])
assert values['CreateRenderProduct.inputs:cameraPrim'] == [camera['prim_path']]
assert values['CameraHelperColor.inputs:topicName'] == camera['rgb_topic']
assert script_editor_output_path(os.environ) == camera['snapshot']
assert optical_transform_snapshot()['child_frame'] == camera['frame_id']
assert selected_delivery_root() == '/World/Selected/Deliveries'
from scene_builder.material_supply import MaterialSupplyProfile
config = load_runtime_configuration()
profile = MaterialSupplyProfile.from_mapping(config['cell']['material_supply']['profile'], config['vision'], config['material_definitions'])
assert profile.spawn_height_above_shelf_m == 0.17
"""
    cell_modules = CONFIG.parents[3] / 'infra/isaac_sim/scripts/pnp_cnc_cell'
    result = subprocess.run([sys.executable, '-I', '-c', code, str(cell_modules), str(path)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_composition_rejects_camera_calibration_resolution_mismatch(tmp_path):
    profile = yaml.safe_load((CONFIG / 'operational_profile.yaml').read_text())
    cell = yaml.safe_load((CONFIG / 'cells/isaac_pnp_cnc.yaml').read_text())
    cell['camera']['resolution'] = [640, 360]
    cell_path = tmp_path / 'cell.yaml'
    cell_path.write_text(yaml.safe_dump(cell))
    profile['profiles']['cell'] = str(cell_path)
    source = tmp_path / 'profile.yaml'
    source.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='Calibration.*resolution'):
        load_profile(source)


def test_supply_profile_and_shared_support_dimensions_are_exported():
    config = load_profile(CONFIG / 'operational_profile.yaml')
    supply = config['cell']['material_supply']
    geometry = json.loads(Path(config['cell']['geometry']).read_text())
    support = next(item for item in geometry['objects'] if item['id'] == supply['support_geometry_object'])
    for detector in config['vision']['detector_profiles'].values():
        assert detector['support_region_dimensions_m'] == support['geometry']['dimensions_m'][:2]
    expected = yaml.safe_load((CONFIG.parents[3] / 'infra/isaac_sim/config/material_supply.yaml').read_text())
    assert supply['profile'] == expected


@pytest.mark.parametrize('profile_name', ['operational_profile.yaml', 'rb5_850_profile.yaml'])
def test_supported_robots_share_role_based_stage_installation(tmp_path, profile_name):
    path = export_runtime_configuration(CONFIG / profile_name, tmp_path / 'runtime.json')
    config = load_profile(path)
    assert config['cell']['isaac']['robot_prim_path'] == '/World/SF_Twin_Cell/ARM'
    assert config['cell']['display_name'] == 'PnP_CNC'
    assert config['cell']['world_to_base'] == {
        'x': 0.15, 'y': -0.15, 'z': 0.68, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    }
