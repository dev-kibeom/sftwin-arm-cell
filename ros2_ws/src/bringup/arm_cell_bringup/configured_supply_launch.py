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


def compose(config,profile_path,log_level='info', *, node_options=None, diagnostics=None, include_simulation=True):
    diagnostics = diagnostics or {}
    node_options = node_options or (lambda package: {'ros_arguments':['--log-level',log_level]})
    def node(package, executable, component=None, logger_package=None, **kwargs):
        values = kwargs.pop('parameters', {})
        values.update(diagnostics.get(component, {}))
        return Node(package=package, executable=executable, output='screen',
                    parameters=[values], **node_options(logger_package or package), **kwargs)
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
    actions = [
        node('arm_cell_vda','external_state_mock_node',parameters={'use_sim_time':config['use_sim_time']}),
        node('arm_cell_safety','safety_node','safety',parameters={**flatten(config['safety']),'use_sim_time':config['use_sim_time']}),
        node('arm_cell_task_execution','task_execution_node','motion',logger_package='arm_cell_motion',name='motion_node',parameters=parameters),
        node('arm_cell_orchestration','configured_supply_mission.py','orchestration',name='orchestration_node',
             parameters={**flatten(config['orchestration']),'use_sim_time':config['use_sim_time']})]
    if include_simulation:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/isaac_planning.launch.py')),
            launch_arguments={'profile':str(bringup/'config/isaac_planning.yaml'),'runtime_profile':str(profile_path),
                              'static_scene':'true','use_rviz':'false','log_level':log_level}.items()))
    return actions
