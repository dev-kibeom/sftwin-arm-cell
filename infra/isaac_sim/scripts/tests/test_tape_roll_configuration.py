"""The two finger lengths use one material and one horizontal fixture sequence."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import xacro
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'infra/isaac_sim/scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/bringup'))
from tape_roll_cell.configuration import compose
from internal_grasp_geometry import ToolGeometry, link_transforms


@pytest.mark.parametrize('suffix',['','_long'])
def test_horizontal_roll_is_compatible_with_selected_tool_and_taught_endpoints(suffix):
    config=compose(ROOT/f'ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10{suffix}.yaml',
        ROOT/'infra/isaac_sim/scripts/tape_roll_cell/experiment.yaml')
    model=config['model_configuration']
    urdf=ET.fromstring(xacro.process_file(model['urdf'],mappings={'mesh_root':model['mesh_root']}).toxml())
    geometry=ToolGeometry(urdf,config['tool']['runtime'])
    experiment=config['cell']['tape_roll']
    material=experiment['material']
    solution=geometry.solve_bore(experiment['insertion_depth_m'],material['width_m'],material['inner_radius_m'])
    assert solution['contracted_swept_radius_m']+.001 < material['inner_radius_m']
    for name,center in [('insert','supply_center_m'),('transfer','receiver_center_m')]:
        pose=experiment['teaching'][name]
        transform=link_transforms(urdf,dict(zip(config['robot']['motion']['arm_joint_names'],pose)))['sf_grasp_tcp']
        assert transform[:3,2] == pytest.approx([1,0,0],abs=1e-6)
        roll_center=transform@np.array([0,0,-experiment['insertion_depth_m'],1])
        expected=np.array(experiment[center],dtype=float)
        if name == 'transfer': expected[2] += experiment['release_clearance_m']
        assert roll_center[:3] == pytest.approx(expected,abs=1e-6)
