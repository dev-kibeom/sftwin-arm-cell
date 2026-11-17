"""Operational composition for configured supply, reusing Motion/Safety and simulator boundaries."""
import importlib.util
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def flatten(values,prefix=''):
    output={}
    for key,value in values.items():
        name=prefix+key
        if isinstance(value,dict):output.update(flatten(value,name+'.'))
        else:output[name]=value
    return output


def compose(config,profile_path,log_level='info'):
    bringup=Path(get_package_share_directory('arm_cell_bringup'))
    model=config['model_configuration']
    description_spec=importlib.util.spec_from_file_location(
        'motion_robot_description',
        Path(get_package_share_directory('arm_cell_motion'))/'launch/robot_description.py')
    description_module=importlib.util.module_from_spec(description_spec)
    description_spec.loader.exec_module(description_module)
    description=description_module.load_robot_description(model['urdf'],model['mesh_root'])
    parameters=flatten(config['motion'])
    parameters.update(robot_description=description,robot_description_semantic=Path(model['semantic']).read_text(),
        home_joint_positions=config['home_joint_positions'],use_sim_time=config['use_sim_time'])
    kinematics=yaml.safe_load(Path(config['motion_configuration']['kinematics']).read_text())
    for key,value in kinematics[model['planning_group']].items():
        parameters['robot_description_kinematics.'+model['planning_group']+'.'+key]=value
    return [
        Node(package='arm_cell_vda',executable='external_state_mock_node',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='arm_cell_safety',executable='safety_node',output='screen',parameters=[{**config['safety'],'use_sim_time':True}]),
        Node(package='arm_cell_task_execution',executable='task_execution_node',name='motion_node',output='screen',parameters=[parameters],arguments=['--ros-args','--log-level',log_level]),
        Node(package='arm_cell_orchestration',executable='configured_supply_mission.py',name='orchestration_node',output='screen',parameters=[{**flatten(config['orchestration']),'use_sim_time':True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/isaac_planning.launch.py')),
            launch_arguments={'profile':str(bringup/'config/isaac_planning.yaml'),'runtime_profile':str(profile_path),'static_scene':'true','use_rviz':'false','log_level':log_level}.items())]
