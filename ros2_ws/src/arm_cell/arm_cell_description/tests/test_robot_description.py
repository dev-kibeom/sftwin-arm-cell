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
    share = Path(get_package_share_directory("arm_cell_description"))
    model = ET.fromstring(
        subprocess.check_output(
            ["xacro", str(share / "urdf/m0609_robotiq_2f85.urdf.xacro")]
        )
    )
    expected = json.loads((Path(__file__).parent / "baseline_model.json").read_text())
    for mesh in model.findall(".//mesh"):
        uri = mesh.get("filename")
        assert uri.startswith("package://arm_cell_description/")
        resource = share / uri.removeprefix("package://arm_cell_description/")
        key = resource.relative_to(share / "meshes").as_posix()
        assert (
            hashlib.sha256(resource.read_bytes()).hexdigest() == expected["meshes"][key]
        )
        mesh.set("filename", resource.relative_to(share / "meshes").as_posix())
    assert canonical(model) == expected["model"]


def test_isaac_entry_point_preserves_model():
    repository = Path(__file__).resolve().parents[5]
    entry = (
        repository / "infra/isaac_sim/assets/urdf/assemblies/m0609_robotiq_2f85.xacro"
    )
    model = ET.fromstring(subprocess.check_output(["xacro", str(entry)]))
    mesh_root = repository / "ros2_ws/src/arm_cell/arm_cell_description/meshes"
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
