"""Validate the selected assembly against the pinned vendor model, not invented geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET

import hashlib

import pytest
import xacro

from arm_cell_bringup.runtime import load_profile
from arm_cell_bringup.configured_supply import configured_tool_model

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


def test_shared_suction_assembly_binds_to_rb5_carrier(tmp_path):
    """Exercise an alternate mounting frame without registering an RB5 deployment."""
    bringup = ROOT / 'ros2_ws/src/bringup/config'
    config = load_profile(bringup / 'rb5_850_profile.yaml')
    conveyor = load_profile(bringup / 'conveyor_pnp_profile.yaml')
    config['tool'] = conveyor['tool']
    robot_macro = ASSET / 'urdf/rb5_850_macro.xacro'
    tool_macro = ROOT / 'assets/tools/suction/urdf/assembly.xacro'
    urdf = tmp_path / 'rb5_suction.urdf.xacro'
    urdf.write_text(f"""<robot name="rb5_suction" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{robot_macro}"/>
      <xacro:include filename="{tool_macro}"/>
      <xacro:rb5_850_robot mesh_dir="{ASSET / 'meshes'}"/>
      <link name="base_link"/>
      <joint name="base_to_link0" type="fixed"><parent link="base_link"/><child link="link0"/></joint>
      <link name="rb5_cup_adapter"/>
      <joint name="tcp_to_adapter" type="fixed">
        <parent link="tcp"/><child link="rb5_cup_adapter"/><origin rpy="1.57079632679 0 0"/>
      </joint>
      <xacro:suction_assembly parent="rb5_cup_adapter"/>
    </robot>""")
    semantic = tmp_path / 'rb5_suction.srdf'
    semantic.write_text("""<robot name="rb5_suction"><group name="arm">
      <chain base_link="base_link" tip_link="sf_grasp_tcp"/></group></robot>""")
    config['model_configuration'].update(model_id='rb5_suction', tool_id='suction',
                                         urdf=str(urdf), semantic=str(semantic))
    configured_tool_model(config)
    runtime = config['tool']['runtime']
    assert runtime['carrier_link'] == 'rb5_cup_adapter'
    expanded = ET.fromstring(xacro.process_file(str(urdf)).toxml())
    cup = expanded.find(f"link[@name='{runtime['cup_link']}']")
    cylinder = cup.find('collision/geometry/cylinder')
    tcp = next(joint for joint in expanded.findall('joint')
               if joint.find('child').get('link') == 'sf_grasp_tcp')
    assert tcp.find('parent').get('link') == runtime['cup_link']
    assert [float(v) for v in tcp.find('origin').get('xyz').split()] == [0, 0, float(cylinder.get('length'))]
