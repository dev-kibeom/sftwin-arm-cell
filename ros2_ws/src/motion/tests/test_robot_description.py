"""Preserve the user-verified model through relocation and installation."""

import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def canonical(element):
    return [
        element.tag,
        sorted(map(list, element.attrib.items())),
        [canonical(child) for child in element],
    ]


def test_installed_model_matches_baseline():
    share = (
        Path(get_package_share_directory("arm_cell_motion"))
        / "assets/robots/m0609_robotiq_2f85"
    )
    model = ET.fromstring(
        subprocess.check_output(
            ["xacro", str(share / "urdf/m0609_robotiq_2f85.urdf.xacro")]
        )
    )
    expected = json.loads((Path(__file__).parent / "baseline_model.json").read_text())
    for mesh in model.findall(".//mesh"):
        uri = mesh.get("filename")
        resource = Path(uri)
        assert resource.is_absolute()
        key = resource.resolve().relative_to((share / "meshes").resolve()).as_posix()
        assert (
            hashlib.sha256(resource.read_bytes()).hexdigest() == expected["meshes"][key]
        )
        mesh.set("filename", key)
    assert canonical(model) == expected["model"]


def test_isaac_entry_point_preserves_model():
    repository = Path(__file__).resolve().parents[4]
    entry = (
        repository / "infra/isaac_sim/assets/urdf/assemblies/m0609_robotiq_2f85.xacro"
    )
    model = ET.fromstring(subprocess.check_output(["xacro", str(entry)]))
    mesh_root = repository / "assets/robots/m0609_robotiq_2f85/meshes"
    expected = json.loads((Path(__file__).parent / "baseline_model.json").read_text())
    for mesh in model.findall(".//mesh"):
        resource = Path(mesh.get("filename"))
        assert resource.is_absolute()
        key = resource.resolve().relative_to(mesh_root).as_posix()
        assert (
            hashlib.sha256(resource.read_bytes()).hexdigest() == expected["meshes"][key]
        )
        mesh.set("filename", key)
    assert canonical(model) == expected["model"]


def test_installed_rb5_assembly_relocates_with_shared_tool():
    import xacro
    repository = Path(__file__).resolve().parents[4]
    installed = Path(get_package_share_directory('arm_cell_motion')) / 'assets'
    source = repository / 'assets'

    def expanded(assets):
        model = ET.fromstring(xacro.process_file(str(
            assets / 'robots/rb5_850/urdf/rb5_850_robotiq_2f85.urdf.xacro')).toxml())
        digests = {}
        for mesh in model.findall('.//mesh'):
            resource = Path(mesh.get('filename')).resolve()
            # URI spelling may differ between source and installed trees.
            root = source.resolve() if source.resolve() in resource.parents else assets.resolve()
            key = resource.relative_to(root).as_posix()
            assert resource.is_file()
            digests[key] = hashlib.sha256(resource.read_bytes()).hexdigest()
            mesh.set('filename', key)
        return canonical(model), digests

    assert expanded(installed) == expanded(source)


def test_ros_model_preserves_base_frame_and_rigid_body_inertia():
    import xacro
    import importlib.util

    source_root = Path(__file__).resolve().parents[4]
    spec = importlib.util.spec_from_file_location(
        'motion_description', source_root / 'ros2_ws/src/motion/launch/robot_description.py')
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    robots = source_root / 'assets/robots'
    source = robots / 'm0609_suction/urdf/m0609_suction.urdf.xacro'
    original = ET.fromstring(xacro.process_file(str(source)).toxml())
    ros = ET.fromstring(loader.load_robot_description(source, robots / 'm0609_robotiq_2f85/meshes'))
    base = ros.find("link[@name='base_link']")
    assert base.find('inertial') is None
    assert base.find('collision') is not None
    assert ros.find("joint/child[@link='base_link']") is None
    joint = ros.find("joint[@name='base_link_to_base_inertia']")
    assert joint.get('type') == 'fixed' and joint.find('origin') is None
    assert joint.find('parent').get('link') == 'base_link'
    body = ros.find(f"link[@name='{joint.find('child').get('link')}']")
    assert canonical(body.find('inertial')) == canonical(original.find("link[@name='base_link']/inertial"))
