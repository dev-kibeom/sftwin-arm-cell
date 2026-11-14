"""Robot owner data reaches USD drives and both ROS articulation endpoints."""
import copy
import math

import pytest

from runtime_configuration import load_runtime_configuration
from robot_binding import RobotBinding, author_home_targets, apply_joint_drive_profile
from graph_builder.ros_action_graph import graph_declaration


def test_default_binding_preserves_m0609(exported_configuration):
    config = load_runtime_configuration(exported_configuration)
    binding = RobotBinding.from_runtime(config)
    assert binding.robot_path == '/World/SF_Twin_Cell/m0609'
    assert binding.articulation_root_path == binding.robot_path + '/root_joint'
    assert binding.base_link_path == binding.robot_path + '/base_link'
    assert binding.joint_names == tuple(f'joint_{i}' for i in range(1, 7))
    assert binding.home_positions == (0.0, 0.0, math.pi / 2, 0.0, math.pi / 2, 0.0)
    assert binding.stiffness_scales == {'joint_2': 2.75}


def alternate(config):
    config = copy.deepcopy(config)
    config['cell']['isaac']['robot_prim_path'] = '/World/Other/Arm'
    config['model_configuration']['isaac'].update(articulation_root='joints/root', articulation_base='links/base')
    config['motion']['arm_joint_names'] = ['shoulder', 'elbow']
    config['home_joint_positions'] = [math.pi / 2, -math.pi / 4]
    config['robot']['isaac'] = {'joint_drive_stiffness_scales': {'elbow': 3.0}}
    return RobotBinding.from_runtime(config)


class Attribute:
    def __init__(self, value):
        self.value = value
    def IsValid(self):
        return True
    def Get(self):
        return self.value
    def Set(self, value):
        self.value = value


class Joint:
    def __init__(self, path):
        self.path = path
        self.target = Attribute(0)
        self.stiffness = Attribute(100)
        self.data = {}
    def IsValid(self):
        return True
    def GetPath(self):
        return self.path
    def GetName(self):
        return self.path.rsplit('/', 1)[-1]
    def SetCustomDataByKey(self, key, value):
        self.data[key] = value
    def GetTargetPositionAttr(self):
        return self.target
    def GetStiffnessAttr(self):
        return self.stiffness


class Stage:
    def __init__(self, joints):
        self.joints = joints
    def Traverse(self):
        return iter(self.joints)


class Physics:
    class DriveAPI:
        @staticmethod
        def Get(prim, kind):
            assert kind == 'angular'
            return prim


def test_selected_names_paths_home_drive_and_reset(exported_configuration):
    binding = alternate(load_runtime_configuration(exported_configuration))
    shoulder = Joint('/World/Other/Arm/joints/shoulder')
    elbow = Joint('/World/Other/Arm/joints/elbow')
    outside = Joint('/World/Other/ArmExtra/elbow')
    stage = Stage([shoulder, elbow, outside])
    apply_joint_drive_profile(stage, binding, Physics)
    author_home_targets(stage, binding, Physics)
    assert (shoulder.target.value, elbow.target.value) == pytest.approx((90, -45))
    assert (elbow.stiffness.value, outside.stiffness.value) == (300, 100)
    elbow.target.Set(12)
    author_home_targets(stage, binding, Physics)  # same operation used by Stop
    assert elbow.target.value == pytest.approx(-45)
    assert elbow.stiffness.value == 300  # reset must not multiply again
    assert outside.target.value == 0
    _, values, links, _ = graph_declaration(str, binding=binding)
    values = dict(values)
    assert values['PublishJointState.inputs:targetPrim'] == ['/World/Other/Arm/joints/root']
    assert values['ArticulationController.inputs:targetPrim'] == ['/World/Other/Arm/joints/root']
    assert binding.base_link_path == '/World/Other/Arm/links/base'
    assert values['PublishJointState.inputs:topicName'] == '/isaac/joint_states'
    assert values['SubscribeJointState.inputs:topicName'] == '/joint_commands'
    assert ('SubscribeJointState.outputs:jointNames', 'ArticulationController.inputs:jointNames') in links


def test_same_name_outside_subtree_cannot_supply_missing_joint(exported_configuration):
    binding = alternate(load_runtime_configuration(exported_configuration))
    with pytest.raises(RuntimeError, match='elbow'):
        author_home_targets(Stage([Joint('/World/Other/Arm/shoulder'), Joint('/World/elbow')]), binding, Physics)


def test_duplicate_joint_inside_selected_robot_is_ambiguous(exported_configuration):
    binding = alternate(load_runtime_configuration(exported_configuration))
    stage = Stage([Joint('/World/Other/Arm/shoulder'),
                   Joint('/World/Other/Arm/a/elbow'),
                   Joint('/World/Other/Arm/b/elbow')])
    with pytest.raises(RuntimeError, match='ambiguous.*elbow'):
        author_home_targets(stage, binding, Physics)


def test_registered_rb5_assembly_reaches_graph_and_common_tool(tmp_path, monkeypatch):
    from pathlib import Path
    from arm_cell_bringup.runtime import export_runtime_configuration
    root = Path(__file__).resolve().parents[6]
    snapshot = export_runtime_configuration(
        root / 'ros2_ws/src/bringup/config/rb5_850_profile.yaml', tmp_path / 'rb5.json')
    monkeypatch.setenv('ARM_CELL_RUNTIME_CONFIG', str(snapshot))
    config = load_runtime_configuration(snapshot)
    binding = RobotBinding.from_runtime(config)
    assert binding.joint_names == tuple(config['robot']['motion']['arm_joint_names'])
    assert binding.home_positions == tuple(config['home_joint_positions'])
    _, values, connections, _ = graph_declaration(str, binding=binding, camera=config['cell']['camera'])
    values = dict(values)
    for node in ('PublishJointState', 'ArticulationController'):
        assert values[f'{node}.inputs:targetPrim'] == [binding.articulation_root_path]
    assert ('SubscribeJointState.outputs:jointNames', 'ArticulationController.inputs:jointNames') in connections
    from isaac_runtime.runtime_workflow import call_selected
    import m0609_cell.runtime_entrypoints as entries
    observed = []
    monkeypatch.setattr(entries, '_run', lambda step, configuration, namespace: observed.append((step, configuration)))
    call_selected(config['tool']['runtime']['isaac_bootstrap'], configuration=config, namespace={})
    assert observed == [('bootstrap_gripper.py', config)]
