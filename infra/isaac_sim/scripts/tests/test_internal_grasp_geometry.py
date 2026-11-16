"""Geometry qualification uses the selected model, not nominal finger lengths."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'infra/isaac_sim/scripts'))
from internal_grasp_geometry import ToolGeometry, qualify_capture
from arm_cell_bringup.model_experiment import load_model_experiment


@pytest.mark.parametrize('suffix,depth', [('', .004), ('_long', .020)])
def test_selected_geometry_has_insertion_clearance_and_expansion_contact(suffix, depth):
    config = load_model_experiment(ROOT / 'ros2_ws/src/bringup/config/model_experiments' / f'm0609_hand_e_c10{suffix}.yaml')
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root': model['mesh_root']}).toxml())
    geometry = ToolGeometry(urdf, config['tool']['runtime'])
    fixture = geometry.fixture(depth, .002)
    assert fixture['contracted_swept_radius_m'] + .001 < fixture['inner_radius_m']
    assert fixture['expanded_radius_m'] > fixture['inner_radius_m']
    assert fixture['tcp_z_m'] > depth
    qualify_capture([0, 0, -depth], np.eye(3), fixture['grasp_position_m'], fixture, .0002)
    with pytest.raises(ValueError, match='alignment'):
        qualify_capture([.01, 0, -depth], np.eye(3), fixture['grasp_position_m'], fixture, .0002)
    with pytest.raises(ValueError, match='insertion'):
        qualify_capture([0, 0, 0], np.eye(3), fixture['grasp_position_m'], fixture, .0002)
    with pytest.raises(ValueError, match='expansion'):
        qualify_capture([0, 0, -depth], np.eye(3), fixture['contract_position_m'], fixture, .0002)
    with pytest.raises(ValueError):
        geometry.fixture(.3, .002)


def test_slab_includes_triangle_edge_intersections():
    from internal_grasp_geometry import slab_radius
    triangle = np.array([[[0., 0, -1], [2, 0, 1], [0, 0, 1]]])
    # No vertex lies on z=0; the true section reaches x=1.
    assert slab_radius(triangle, 0., 0.) == pytest.approx(1.)


@pytest.mark.parametrize('rotation,position,primary', [
    (np.diag([-1., -1, 1]), [0, 0, -.004], .01),
    (np.eye(3), [0, 0, float('nan')], .01),
    (np.eye(3), [0, 0, -.004], float('nan')),
])
def test_capture_rejects_orientation_and_nonfinite_observations(rotation, position, primary):
    with pytest.raises(ValueError):
        qualify_capture(position, rotation, primary, {'insertion_depth_m': .004, 'grasp_position_m': .01}, .0002)


def test_joint_and_finger_link_names_are_not_algorithm_inputs():
    config = load_model_experiment(ROOT / 'ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10_long.yaml')
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root': model['mesh_root']}).toxml())
    settings = config['tool']['runtime']
    original = ToolGeometry(urdf, settings).fixture(.02, .002)
    renamed = {}
    for element in urdf.iter():
        for key, value in list(element.attrib.items()):
            if key in ('name', 'link', 'joint') and value.startswith('gripper_'):
                renamed[value] = value.replace('gripper_', 'selected_')
                element.set(key, renamed[value])
    settings['primary_joint'] = renamed[settings['primary_joint']]
    settings['joint_model'] = {renamed[name]: joint for name, joint in settings['joint_model'].items()}
    settings['finger_tip_links'] = [renamed[name] for name in settings['finger_tip_links']]
    actual = ToolGeometry(urdf, settings).fixture(.02, .002)
    assert actual == pytest.approx(original)
