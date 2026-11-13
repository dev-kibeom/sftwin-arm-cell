"""Assembly selection must reach the importer without changing legacy output paths."""
import copy
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

from runtime_configuration import load_runtime_configuration


ROOT = Path(__file__).resolve().parents[6]


def test_default_assembly_preserves_wrapper_meshes_and_scene_paths(exported_configuration):
    from scene_builder.model_build_configuration import ModelBuildConfiguration

    config = load_runtime_configuration(exported_configuration)
    selected = ModelBuildConfiguration.from_runtime(config, ROOT)
    assets = ROOT / 'infra/isaac_sim/assets'
    assert selected.source == assets / 'urdf/assemblies/m0609_robotiq_2f85.xacro'
    assert selected.generated_urdf == assets / 'urdf/generated/m0609_robotiq_2f85.urdf'
    assert selected.usd == assets / 'usd/m0609_robotiq_2f85_generated.usd'
    legacy = subprocess.check_output(['xacro', str(selected.source)])
    actual = subprocess.check_output(['xacro', str(selected.source), *selected.xacro_arguments])
    assert actual == legacy
    assert len(selected.revolute_joints) == 6
    assert 'gripper_robotiq_85_right_finger_tip_joint' in selected.revolute_joints


def test_another_assembly_uses_selected_input_and_separate_outputs(exported_configuration, tmp_path):
    from scene_builder.model_build_configuration import ModelBuildConfiguration

    config = load_runtime_configuration(exported_configuration)
    original = ModelBuildConfiguration.from_runtime(config, ROOT)
    config = copy.deepcopy(config)
    model = config['model_configuration']
    source = model['urdf']
    model.update(model_id='selected_assembly', urdf=str(tmp_path / 'selected.xacro'),
                 mesh_root=str(tmp_path / 'meshes'))
    (tmp_path / 'meshes').symlink_to(ROOT / 'assets/robots/m0609_robotiq_2f85/meshes', target_is_directory=True)
    (tmp_path / 'selected.xacro').write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="selected_assembly">'
        f'<xacro:include filename="{source}"/>'
        '</robot>')
    model['isaac'] = dict(articulation_root='root_joint', articulation_base='base_link',
                          revolute_joints=['joint_1'])
    semantic = tmp_path / 'selected.srdf'
    semantic.write_text(Path(model['semantic']).read_text().replace(
        'name="m0609_robotiq_2f85"', 'name="selected_assembly"'))
    model['semantic'] = str(semantic)
    model_path = tmp_path / 'model.yaml'
    model_path.write_text(yaml.safe_dump(model))
    from arm_cell_bringup.runtime import export_runtime_configuration
    profile = yaml.safe_load((ROOT / 'ros2_ws/src/bringup/config/operational_profile.yaml').read_text())
    profile['model_configuration'] = str(model_path)
    profile_path = tmp_path / 'deployment.yaml'
    profile_path.write_text(yaml.safe_dump(profile))
    exported = export_runtime_configuration(profile_path, tmp_path / 'runtime.json')
    selected = ModelBuildConfiguration.from_runtime(load_runtime_configuration(exported), ROOT)
    assert selected.source == tmp_path / 'selected.xacro'
    assert selected.xacro_arguments == (f'mesh_root:={tmp_path / "meshes"}',)
    assert selected.revolute_joints == ('joint_1',)
    assert selected.generated_urdf.name == 'selected_assembly.urdf'
    assert selected.usd.name == 'selected_assembly_generated.usd'
    assert selected.generated_urdf != original.generated_urdf
    assert selected.usd != original.usd
    assert selected.provenance != original.provenance
    assert selected.base_layer != original.base_layer
    expanded = subprocess.check_output(['xacro', str(selected.source), *selected.xacro_arguments])
    urdf = ET.fromstring(expanded)
    assert urdf.get('name') == 'selected_assembly'
    assert all(Path(mesh.get('filename')).is_file() for mesh in urdf.findall('.//mesh'))


def test_selected_meshes_are_part_of_provenance(exported_configuration, tmp_path):
    from scene_builder.model_build_configuration import ModelBuildConfiguration
    from scene_builder.model_provenance import build_manifest, verify_current_artifact, write_manifest

    config = load_runtime_configuration(exported_configuration)
    selected = ModelBuildConfiguration.from_runtime(config, ROOT)
    generated = tmp_path / 'model.urdf'
    generated.write_bytes(subprocess.check_output(
        ['xacro', str(selected.source), *selected.xacro_arguments]))
    usd = tmp_path / 'model.usd'
    usd.write_bytes(b'importer-output')
    manifest = tmp_path / 'model.provenance.json'
    write_manifest(manifest, build_manifest(
        selected.source, generated, usd, xacro_arguments=selected.xacro_arguments))
    # The unchanged Scene consumer can reuse the default Assembly artifact.
    verify_current_artifact(selected.source, generated, usd, manifest)
    verify_current_artifact(selected.source, generated, usd, manifest,
                            xacro_arguments=selected.xacro_arguments)
    with pytest.raises(ValueError, match='canonical Xacro digest'):
        verify_current_artifact(selected.source, generated, usd, manifest,
                                xacro_arguments=(f'mesh_root:={tmp_path / "other-meshes"}',))


def test_assembly_identity_cannot_escape_generated_directory(exported_configuration):
    from scene_builder.model_build_configuration import ModelBuildConfiguration

    config = load_runtime_configuration(exported_configuration)
    config['model_configuration']['model_id'] = '../other-assembly'
    with pytest.raises(ValueError, match='filename-safe'):
        ModelBuildConfiguration.from_runtime(config, ROOT)
