"""Geometric invariants of the experimental CAD-pose convention, not hardware calibration."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from arm_cell_bringup.runtime import load_profile

ROOT = Path(__file__).resolve().parents[4]
PROFILE = ROOT / 'ros2_ws/src/bringup/config/hcr14_profile.yaml'


def model():
    config = load_profile(PROFILE)
    owner = config['model_configuration']
    return config, ET.fromstring(xacro.process_file(owner['urdf'], mappings={
        'mesh_root': owner['mesh_root']}).toxml())


def test_cad_reference_pose_preserves_axis_lines_and_flange():
    _, urdf = model()
    # Independent STEP ring centre/axis measurements; CAD mm -> ROS metres:
    # (x,y,z)_ROS = (x,-z,y)_CAD / 1000. Signs below are simulation choices.
    points = [(0, 0, 0), (0, .207, 0), (0, .937, 0),
              (-.5388, .937, 0), (0, .937, .1847), (-.690, .937, 0)]
    axes = [(0, 1, 0), (0, 0, 1), (0, 0, 1), (0, 0, 1), (1, 0, 0), (0, 0, 1)]
    transform = lambda p: np.array([p[0], -p[2], p[1]])
    position = np.zeros(3)
    for i, (point, axis) in enumerate(zip(points, axes), 1):
        joint = urdf.find(f"joint[@name='joint{i}']")
        origin = joint.find('origin')
        assert np.allclose([float(v) for v in origin.get('rpy').split()], 0)
        position += np.array([float(v) for v in origin.get('xyz').split()])
        actual_axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
        assert np.allclose(np.cross(position - transform(point), transform(axis)), 0, atol=1e-9)
        assert np.allclose(actual_axis, transform(axis))
    flange = urdf.find("joint[@name='link6_to_flange']/origin")
    position += np.array([float(v) for v in flange.get('xyz').split()])
    assert position == pytest.approx([-.690, -.3259, .937])
    assert [float(v) for v in flange.get('rpy').split()] == pytest.approx([math.pi/2, 0, 0])


def test_limits_tcp_semantic_and_home_use_common_assembly_contract():
    config, urdf = model()
    semantic = ET.parse(config['model_configuration']['semantic']).getroot()
    links = {link.get('name') for link in urdf.findall('link')}
    for item in semantic.findall('disable_collisions'):
        assert {item.get('link1'), item.get('link2')} <= links
    for i in range(1, 7):
        limit = urdf.find(f"joint[@name='joint{i}']/limit")
        bound = math.radians(165 if i == 3 else 360)
        assert float(limit.get('lower')) == pytest.approx(-bound)
        assert float(limit.get('upper')) == pytest.approx(bound)
        assert float(limit.get('velocity')) == pytest.approx(math.radians([155,155,230,270,270,270][i-1]))
    baseline = ET.fromstring(xacro.process_file(str(ROOT / 'assets/robots/rb5_850/urdf/rb5_850_robotiq_2f85.urdf.xacro')).toxml())
    assert ET.tostring(urdf.find("joint[@name='gripper_to_sf_grasp_tcp']")) == ET.tostring(baseline.find("joint[@name='gripper_to_sf_grasp_tcp']"))
    assert config['launcher']['validation_status'] == 'experimental'


def test_private_mesh_override_keeps_the_same_chain_and_tool(tmp_path):
    config, basic = model()
    mesh_root = tmp_path / 'private_meshes'
    detailed = ET.fromstring(xacro.process_file(config['model_configuration']['urdf'],
        mappings={'mesh_root': str(mesh_root)}).toxml())
    # Private meshes are external inputs; the public source retains only URDF.
    for i in range(7):
        link = detailed.find(f"link[@name='link{i}']")
        assert {m.get('filename') for m in link.findall('.//mesh')} == {str(mesh_root / f'link{i}.stl')}
    assert [ET.tostring(j) for j in detailed.findall('joint')] == [
        ET.tostring(j) for j in basic.findall('joint')]


def test_requested_home_and_existing_return_are_independent():
    config, _ = model()
    assert config['home_joint_positions'] == pytest.approx([
        math.pi, 0, 0, 0, math.pi/2, 0])
    assert config['robot']['motion']['place_tray_return_joint_positions'] == pytest.approx([
        0, 0, -math.pi/2, 0, math.pi/2, 0])


def test_authored_visual_meshes_are_available_in_metres():
    _, urdf = model()
    for i in range(7):
        visual = urdf.find(f"link[@name='link{i}']/visual/geometry/mesh")
        assert visual is not None
        path = Path(visual.get('filename'))
        document = ET.parse(path)
        ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
        assert float(document.find('c:asset/c:unit', ns).get('meter')) == 1
        geometries = document.findall('.//c:geometry', ns)
        assert geometries
        for geometry in geometries:
            positions = geometry.find('c:mesh/c:source/c:float_array', ns)
            assert all(math.isfinite(float(v)) for v in positions.text.split())


def test_regenerated_visual_digest_invalidates_expanded_urdf(tmp_path):
    import importlib.util
    import shutil
    script = ROOT / 'assets/robots/hcr14/scripts/generate_visuals.py'
    spec = importlib.util.spec_from_file_location('hcr14_visuals', script)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    for index in range(7):
        shutil.copyfile(script.parents[1] / 'visual' / f'link{index}.dae',
                        tmp_path / f'link{index}.dae')
    wrapper = tmp_path / 'test.xacro'
    wrapper.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
                       '<xacro:include filename="revision.xacro"/>'
                       '<xacro:hcr14_visual_revision/></robot>')
    generator.write_revision(tmp_path)
    before = xacro.process_file(str(wrapper)).toxml()
    changed = tmp_path / 'link0.dae'
    changed.write_bytes(changed.read_bytes() + b'\n')
    generator.write_revision(tmp_path)
    assert xacro.process_file(str(wrapper)).toxml() != before
