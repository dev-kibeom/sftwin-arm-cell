"""Validate the selected assembly against the pinned vendor model, not invented geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET

import hashlib

import pytest
import xacro

from arm_cell_bringup.runtime import load_profile

ROOT = Path(__file__).resolve().parents[4]
ASSET = ROOT / 'assets/robots/rb5_850'


def canonical(element):
    return (element.tag, sorted(element.attrib.items()), [canonical(child) for child in element])


def test_rb5_preserves_vendor_kinematics_inertias_and_meshes():
    config = load_profile(ROOT / 'ros2_ws/src/bringup/config/rb5_850_profile.yaml')
    actual = ET.fromstring(xacro.process_file(config['model_configuration']['urdf']).toxml())
    vendor = ET.parse(ASSET / 'third_party/rbpodo_description/rb5_850e.urdf').getroot()
    for mesh in actual.findall('.//mesh'):
        assert Path(mesh.get('filename')).is_file()
    for tag in ('joint', 'link'):
        by_name = {element.get('name'): element for element in actual.findall(tag)}
        for expected in vendor.findall(tag):
            selected = by_name[expected.get('name')]
            for element, actual_mesh in zip(
                    expected.findall('.//mesh'), selected.findall('.//mesh')):
                relative = element.get('filename').split('/rb5_850e/')[1]
                original = ASSET / 'meshes' / relative
                resource = Path(actual_mesh.get('filename'))
                assert hashlib.sha256(resource.read_bytes()).digest() == hashlib.sha256(
                    original.read_bytes()).digest()
                element.set('filename', relative)
                actual_mesh.set('filename', relative)
            assert canonical(selected) == canonical(expected)
    assert actual.find("joint[@name='base_link_to_link0']/origin").get('xyz') == '0 0 0'
    # Outward Tool +Z follows the manufacturer's flange normal (-Y).
    import math
    mount = actual.find("joint[@name='tcp_to_tool_mount']/origin")
    assert [float(v) for v in mount.get('xyz').split()] == [0, 0, 0]
    assert math.isclose(float(mount.get('rpy').split()[0]), math.pi/2)
    assert {link.get('name') for link in actual.findall('link')} >= {
        'tcp', 'tool_mount', 'sf_grasp_tcp'}


@pytest.mark.parametrize('profile', ['operational_profile.yaml', 'rb5_850_profile.yaml'])
def test_selected_semantic_links_and_tool_tcp_are_valid(profile):
    config = load_profile(ROOT / 'ros2_ws/src/bringup/config' / profile)
    model = config['model_configuration']
    actual = ET.fromstring(xacro.process_file(model['urdf']).toxml())
    semantic = ET.parse(model['semantic']).getroot()
    links = {link.get('name') for link in actual.findall('link')}
    for exclusion in semantic.findall('disable_collisions'):
        assert {exclusion.get('link1'), exclusion.get('link2')} <= links
    tcp = actual.find("joint[@name='gripper_to_sf_grasp_tcp']")
    baseline_path = ROOT / 'assets/robots/m0609_robotiq_2f85/urdf/m0609_robotiq_2f85.urdf.xacro'
    baseline = ET.fromstring(xacro.process_file(str(baseline_path)).toxml())
    assert canonical(tcp) == canonical(baseline.find("joint[@name='gripper_to_sf_grasp_tcp']"))


def test_pinned_vendor_inputs_match_recorded_checksums():
    import json
    manifest = json.loads((ASSET / 'third_party/rbpodo_description/SHA256SUMS.json').read_text())
    for relative, expected in manifest.items():
        assert hashlib.sha256((ASSET / relative).read_bytes()).hexdigest() == expected
