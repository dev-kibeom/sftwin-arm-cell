"""Production C10 geometry, radial Physical Real contract and static assemblies."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[4]



@pytest.mark.parametrize('robot_id', ['m0609', 'hcr14'])
def test_c10_assembly_and_radial_motion(robot_id):
    from arm_cell_bringup.assembly_model import load_assembly_model
    config = load_assembly_model(robot_id, 'hand_e_c10')
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root': model['mesh_root']}).toxml())
    assert urdf.get('name') == f'{robot_id}_hand_e_c10'
    assert len(config['tool']['runtime']['joint_model']) == 3
    assert config['tool']['runtime']['supported_combinations'] == [['physics', 'physical']]
    assert config['tool']['runtime']['mode_id'] == 'physical_real'
    assert config['tool']['runtime']['grasp_strategies'] == ['internal_radial']
    assert 'actuator_observation' in config['tool']['runtime']['capabilities']
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
    linkage = config['tool']['runtime']['joint_model']
    for joint in linkage.values():
        assert joint['lower'] == 0.
        assert joint['upper'] == pytest.approx(.0192)
        assert joint['velocity'] > 0
        assert joint['effort'] > 0
        mimic = joint.get('mimic')
        if mimic:
            assert mimic['multiplier'] == 1.
            assert mimic['offset'] == 0.
