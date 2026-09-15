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
        "camera_launch", share / "launch/isaac_camera.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_camera_overlay_requires_snapshot(tmp_path):
    profile = tmp_path / "profile.yaml"
    profile.write_text("camera_snapshot: null\n")
    context = LaunchContext()
    context.launch_configurations.update({"profile": str(profile), "snapshot": ""})
    with pytest.raises(RuntimeError, match="inspector-generated"):
        load_module().compose(context)


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
