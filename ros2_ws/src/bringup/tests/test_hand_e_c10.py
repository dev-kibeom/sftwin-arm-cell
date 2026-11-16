"""C10 stage-one geometry, assembly and unsupported grasp boundary."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[4]
EXPERIMENT = ROOT / 'ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml'


def test_c10_assembly_and_radial_motion():
    from arm_cell_bringup.model_experiment import load_model_experiment, joint_targets
    config = load_model_experiment(EXPERIMENT)
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root': model['mesh_root']}).toxml())
    assert urdf.get('name') == 'm0609_hand_e_c10'
    assert len(config['tool']['runtime']['joint_model']) == 3
    assert config['tool']['runtime']['supported_combinations'] == []
    assert config['tool']['runtime']['capabilities'] == ['joint_position']
    assert all(Path(mesh.get('filename')).is_file() for mesh in urdf.findall('.//mesh'))
    tcp = urdf.find("joint[@name='gripper_tcp_link_joint']/origin")
    assert list(map(float, tcp.get('xyz').split())) == pytest.approx([0, 0, .15477])
    canonical = urdf.find("joint[@name='gripper_to_sf_grasp_tcp']")
    assert canonical.find('parent').get('link') == 'gripper_tcp_link'
    assert canonical.find('child').get('link') == 'sf_grasp_tcp'
    directions = []
    for index in range(1, 4):
        name = f'gripper_hande_c10_finger_{index}_joint'
        joint = urdf.find(f"joint[@name='{name}']")
        assert joint.get('type') == 'prismatic'
        assert float(joint.find('limit').get('upper')) == pytest.approx(.0192)
        angle = float(joint.find('origin').get('rpy').split()[2])
        axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
        rotation = np.array([[math.cos(angle), -math.sin(angle), 0],
                             [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
        direction = rotation @ axis
        radial = np.array([math.cos(angle), math.sin(angle), 0])
        assert direction @ radial == pytest.approx(-1)  # positive q contracts for OD grasp
        directions.append(direction)
        if index > 1:
            assert joint.find('mimic').get('joint') == 'gripper_hande_c10_finger_1_joint'
        link = urdf.find(f"link[@name='gripper_hande_c10_finger_{index}']")
        assert len(link.findall('collision')) == 2
    assert np.sum(directions, axis=0) == pytest.approx([0, 0, 0], abs=1e-12)
    assert set(joint_targets(config['tool']['runtime']['joint_model'], .0192).values()) == {.0192}
    for position in (-.001, .02, float('nan')):
        with pytest.raises(ValueError):
            joint_targets(config['tool']['runtime']['joint_model'], position)


def test_experiment_is_not_a_production_profile():
    from arm_cell_bringup.runtime import load_profile
    with pytest.raises(ValueError, match='model experiment'):
        load_profile(EXPERIMENT)


@pytest.mark.parametrize('change', ['identity', 'binding', 'grasp', 'primary', 'gain'])
def test_invalid_experiment_selection_is_rejected(tmp_path, change):
    import yaml
    from arm_cell_bringup.model_experiment import load_model_experiment
    from arm_cell_bringup.runtime import read, resolve
    source = read(EXPERIMENT)
    for key in ('robot', 'motion_configuration'):
        source[key] = str(resolve(source[key], EXPERIMENT.parent))
    model = read(resolve(source['model_configuration'], EXPERIMENT.parent))
    tool = read(resolve(source['tool'], EXPERIMENT.parent))
    if change == 'identity':
        model['tool_id'] = 'robotiq_2f85'
    elif change == 'binding':
        model['isaac']['prismatic_joints'].pop()
    elif change == 'grasp':
        tool['runtime']['supported_combinations'] = [['logical', 'fixed_attach']]
    elif change == 'primary':
        tool['runtime']['primary_joint'] = 'joint_1'
    else:
        tool['runtime']['drive_stiffness'] = float('nan')
    (tmp_path / 'model.yaml').write_text(yaml.safe_dump(model))
    (tmp_path / 'tool.yaml').write_text(yaml.safe_dump(tool))
    source.update(model_configuration=str(tmp_path / 'model.yaml'), tool=str(tmp_path / 'tool.yaml'))
    profile = tmp_path / 'experiment.yaml'
    profile.write_text(yaml.safe_dump(source))
    with pytest.raises(ValueError):
        load_model_experiment(profile)
