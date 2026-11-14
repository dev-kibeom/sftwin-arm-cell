import importlib.util
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
import pytest


def load_module():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    spec = importlib.util.spec_from_file_location(
        "camera_launch", Path(__file__).resolve().parents[1] / "launch/isaac_camera.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_camera_overlay_requires_snapshot_when_no_packaged_snapshot_exists(
    tmp_path, monkeypatch
):
    profile = tmp_path / "profile.yaml"
    profile.write_text("camera_snapshot: null\n")
    context = LaunchContext()
    context.launch_configurations.update({"profile": str(profile), "snapshot": ""})
    module = load_module()
    bringup_share = tmp_path / "checkout/ros2_ws/install/share/arm_cell_bringup"
    vision_share = tmp_path / "checkout/ros2_ws/install/share/arm_cell_sim_adapter"
    bringup_share.mkdir(parents=True)
    vision_share.mkdir(parents=True)
    module.get_package_share_directory = lambda package: str(
        vision_share if package == "arm_cell_sim_adapter" else bringup_share
    )
    monkeypatch.delenv("SFTWIN_PROJECT_ROOT", raising=False)
    with pytest.raises(RuntimeError, match="inspector-generated"):
        module.compose(context)


def test_camera_overlay_has_exactly_two_approved_tf_edges(tmp_path):
    snapshot = tmp_path / "snapshot.json"
    snapshot.write_text(
        json.dumps(
            {
                "world_to_camera_link": {
                    "parent_frame": "world",
                    "child_frame": "camera_link",
                    "translation_m": [1, 2, 3],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "camera_link_to_optical": {
                    "parent_frame": "camera_link",
                    "child_frame": "camera_color_optical_frame",
                    "translation_m": [0, 0, 0],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
            }
        )
    )
    profile = tmp_path / "profile.yaml"
    profile.write_text("camera_snapshot: " + str(snapshot) + "\n")
    context = LaunchContext()
    context.launch_configurations.update({"profile": str(profile), "snapshot": ""})
    nodes = load_module().compose(context)
    assert len(nodes) == 2 and all(
        isinstance(node, Node) and node.node_package == "tf2_ros" for node in nodes
    )


def test_camera_overlay_snapshot_argument_overrides_profile(tmp_path):
    snapshot = tmp_path / "snapshot.json"
    snapshot.write_text(
        json.dumps(
            {
                "world_to_camera_link": {
                    "parent_frame": "world",
                    "child_frame": "camera_link",
                    "translation_m": [1, 2, 3],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "camera_link_to_optical": {
                    "parent_frame": "camera_link",
                    "child_frame": "camera_color_optical_frame",
                    "translation_m": [0, 0, 0],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
            }
        )
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {"profile": str(tmp_path / "missing-profile.yaml"), "snapshot": str(snapshot)}
    )
    nodes = load_module().compose(context)
    assert len(nodes) == 2


def test_camera_overlay_uses_simulation_adapter_snapshot_by_default(
    tmp_path, monkeypatch
):
    bringup_share = tmp_path / "checkout/ros2_ws/install/share/arm_cell_bringup"
    vision_share = tmp_path / "checkout/ros2_ws/install/share/arm_cell_sim_adapter"
    bringup_share.mkdir(parents=True)
    snapshot = vision_share / "config/isaac_camera_stage_snapshot.json"
    snapshot.parent.mkdir(parents=True)
    snapshot.write_text(
        json.dumps(
            {
                "world_to_camera_link": {
                    "parent_frame": "world",
                    "child_frame": "camera_link",
                    "translation_m": [0, 0, 0],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "camera_link_to_optical": {
                    "parent_frame": "camera_link",
                    "child_frame": "camera_color_optical_frame",
                    "translation_m": [0, 0, 0],
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
            }
        )
    )
    module = load_module()
    module.get_package_share_directory = lambda package: str(
        vision_share if package == "arm_cell_sim_adapter" else bringup_share
    )
    monkeypatch.delenv("SFTWIN_PROJECT_ROOT", raising=False)
    context = LaunchContext()
    profile = tmp_path / "profile.yaml"
    profile.write_text("camera_snapshot: null\n")
    context.launch_configurations.update({"profile": str(profile), "snapshot": ""})
    assert len(module.compose(context)) == 2


def test_runtime_selection_precedes_standalone_profile_and_snapshot(tmp_path):
    from arm_cell_bringup.runtime import export_runtime_configuration
    import yaml
    source = Path(__file__).resolve().parents[1] / 'config/operational_profile.yaml'
    runtime = export_runtime_configuration(source, tmp_path / 'runtime.json')
    document = json.loads(runtime.read_text())
    camera = document['configuration']['cell']['camera']
    camera['prim_path'] = '/World/Selected/Camera'
    camera['snapshot'] = str(tmp_path / 'selected.json')
    # Re-export an owner profile so the runtime checksum remains authoritative.
    profile = yaml.safe_load(source.read_text())
    cell_path = tmp_path / 'cell.yaml'
    cell = yaml.safe_load((source.parent / 'cells/isaac_pnp_cnc.yaml').read_text())
    cell['camera'] = camera
    cell_path.write_text(yaml.safe_dump(cell))
    profile['profiles']['cell'] = str(cell_path)
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    export_runtime_configuration(profile_path, runtime)
    snapshot = {
        'camera_prim': camera['prim_path'], 'camera_binding': {k: v for k, v in camera.items() if k != 'snapshot'},
        'world_to_camera_link': {'parent_frame': 'world', 'child_frame': camera['link_frame_id'],
                                 'translation_m': [1, 2, 3], 'quaternion_xyzw': [0, 0, 0, 1]},
        'camera_link_to_optical': {'parent_frame': camera['link_frame_id'], 'child_frame': camera['frame_id'],
                                   'translation_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
    }
    Path(camera['snapshot']).write_text(json.dumps(snapshot))
    context = LaunchContext()
    context.launch_configurations.update(runtime_profile=str(runtime), profile='missing.yaml', snapshot='missing.json')
    nodes = load_module().compose(context)
    assert len(nodes) == 2
    snapshot['camera_binding']['mount']['translation_offset_m'][0] += 1.0
    Path(camera['snapshot']).write_text(json.dumps(snapshot))
    with pytest.raises(RuntimeError, match='selected Cell camera configuration'):
        load_module().compose(context)
    snapshot['camera_prim'] = '/World/Other/Camera'
    Path(camera['snapshot']).write_text(json.dumps(snapshot))
    with pytest.raises(RuntimeError, match='selected Cell camera prim'):
        load_module().compose(context)
