"""Cell selections flow to existing camera and material consumers."""
from copy import deepcopy

from runtime_configuration import load_runtime_configuration
from graph_builder.ros_action_graph import graph_declaration


def test_graph_uses_selected_camera_instead_of_default_cell():
    config = load_runtime_configuration()
    camera = deepcopy(config['cell'].get('camera', {}))
    camera.update(prim_path='/World/Selected/Sensor', resolution=[640, 360],
                  frame_id='selected_optical', rgb_topic='/selected/rgb',
                  depth_topic='/selected/depth', camera_info_topic='/selected/info')
    _, values, _, _ = graph_declaration(str, camera=camera)
    values = dict(values)
    assert values['CreateRenderProduct.inputs:cameraPrim'] == [camera['prim_path']]
    assert (values['CreateRenderProduct.inputs:width'], values['CreateRenderProduct.inputs:height']) == tuple(camera['resolution'])
    for node, topic in [('CameraHelperColor', 'rgb_topic'), ('CameraHelperDepth', 'depth_topic'), ('CameraInfoHelper', 'camera_info_topic')]:
        assert values[f'{node}.inputs:topicName'] == camera[topic]
        assert values[f'{node}.inputs:frameId'] == camera['frame_id']


def test_capture_registry_uses_selected_delivery_subtree():
    from gripper_runtime.operational_config import production_capture_configuration
    config = deepcopy(load_runtime_configuration())
    config['cell']['material_supply'] = {'delivery_root': '/World/Selected/Stock'}
    _, _, registry = production_capture_configuration(config, config['_mission_recipe'])
    from types import SimpleNamespace
    def prim(path):
        return SimpleNamespace(GetPath=lambda: path,
                               GetAttribute=lambda _: SimpleNamespace(Get=lambda: 'TestCube'))
    selected = '/World/Selected/Stock/Delivery_a/Material_0000'
    stage = SimpleNamespace(GetPrimAtPath=lambda _: SimpleNamespace(IsValid=lambda: True),
                            Traverse=lambda: [prim(selected), prim('/World/Other/Material_0000')])
    assert registry.candidate_paths(stage) == (selected,)


def test_default_mount_and_support_preserve_cell_geometry():
    from scene_builder.static_environment import load_canonical_static_geometry, camera_mount_pose, supply_support_origin
    config = load_runtime_configuration()
    cell = config['cell']
    geometry = load_canonical_static_geometry(manifest_path=cell['geometry'])
    position, rotation = camera_mount_pose(geometry, cell['camera'])
    assert position == [0.2, -0.83, 1.735]
    assert rotation == [0.0, 90.0, 0.0]
    assert supply_support_origin(geometry, cell['material_supply']) == (0.2, -0.85, 0.6625)


def test_mount_and_supply_follow_selected_existing_geometry():
    from scene_builder.static_environment import camera_mount_pose, supply_support_origin
    geometry = {'selected': {'pose': {'position_m': [2., 3., 4.]},
                             'geometry': {'dimensions_m': [1., 2., 0.4]}}}
    camera = {'mount': {'geometry_object': 'selected', 'translation_offset_m': [0.1, -0.2, 0.3],
                        'rotation_xyz_deg': [1., 2., 3.]}}
    assert camera_mount_pose(geometry, camera) == ([2.1, 2.8, 4.3], [1., 2., 3.])
    assert supply_support_origin(geometry, {'support_geometry_object': 'selected'}) == (2., 3., 4.2)
