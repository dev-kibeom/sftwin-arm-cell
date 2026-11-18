"""Selected assembly can request a Robot-owned precision drive policy."""
from copy import deepcopy
from isaac_runtime.robot_binding import RobotBinding


def test_precision_policy_is_selected_without_mutating_default_robot_gains():
    config = dict(cell={'isaac':{'robot_prim_path':'/World/Robot'}},
        model_configuration={'isaac':{'articulation_root':'root_joint','articulation_base':'base_link'}},
        motion={'arm_joint_names':['joint_1','joint_2']},home_joint_positions=[0.,0.],
        robot={'isaac':{'joint_drive_stiffness_scales':{'joint_2':2.75},
            'drive_profiles':{'radial_precision':{'stiffness_scale':100.,'damping_scale':10.}}}})
    original = deepcopy(config['robot'])
    ordinary = RobotBinding.from_runtime(config)
    assert ordinary.stiffness_scales == {'joint_2':2.75}
    assert ordinary.damping_scales == {}
    config['model_configuration']['isaac']['robot_drive_profile']='radial_precision'
    precision = RobotBinding.from_runtime(config)
    assert precision.stiffness_scales == {'joint_1':100.,'joint_2':275.}
    assert precision.damping_scales == {'joint_1':10.,'joint_2':10.}
    assert config['robot'] == original
