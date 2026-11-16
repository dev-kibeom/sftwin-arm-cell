"""User-requested 100 mm finger extensions preserve the C10 mechanism."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro

from arm_cell_bringup.model_experiment import load_model_experiment

ROOT = Path(__file__).resolve().parents[4]


@pytest.mark.parametrize('robot_id', ['m0609', 'hcr14'])
def test_long_fingers_preserve_mechanism_and_move_tcp(robot_id):
    directory = ROOT / 'ros2_ws/src/bringup/config/model_experiments'
    stock = load_model_experiment(directory / f'{robot_id}_hand_e_c10.yaml')
    long = load_model_experiment(directory / f'{robot_id}_hand_e_c10_long.yaml')
    assert long['robot'] == stock['robot']
    assert long['tool']['runtime']['joint_model'] == stock['tool']['runtime']['joint_model']
    assert long['tool']['runtime']['supported_combinations'] == []
    def expand(config):
        model = config['model_configuration']
        return ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root': model['mesh_root']}).toxml())
    original, extended = expand(stock), expand(long)
    for i in range(1, 4):
        name = f'gripper_hande_c10_finger_{i}'
        assert ET.tostring(original.find(f"link[@name='{name}']")) == ET.tostring(extended.find(f"link[@name='{name}']"))
        link = extended.find(f"link[@name='gripper_hande_c10_extension_{i}']")
        assert link is not None
        dimensions = list(map(float, link.find('collision/geometry/box').get('size').split()))
        assert dimensions == pytest.approx([.003, .012, .100])
        assert float(link.find('inertial/mass').get('value')) == pytest.approx(.00972)
        joint = extended.find(f"joint[@name='gripper_hande_c10_extension_{i}_mount']")
        assert joint.get('type') == 'fixed'
        assert joint.find('parent').get('link') == name
        origin = list(map(float, joint.find('origin').get('xyz').split()))
        assert origin == pytest.approx([.06271698, 0., .15476571])
    offset = extended.find("joint[@name='gripper_to_sf_grasp_tcp']/origin")
    assert list(map(float, offset.get('xyz').split())) == pytest.approx([0, 0, .09999571])
