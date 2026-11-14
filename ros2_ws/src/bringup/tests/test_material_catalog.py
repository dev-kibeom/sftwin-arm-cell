from pathlib import Path

import pytest
import yaml

from arm_cell_bringup.material_catalog import load_material_catalog

MATERIALS = Path(__file__).resolve().parents[1] / 'config/materials'


def test_cnc_material_dimensions_and_mass():
    catalog = load_material_catalog(MATERIALS)
    for identity, dimensions in (
        ('AL_BILLET_060_040_030', [0.06, 0.04, 0.03]),
        ('STEEL_BLANK_070_050_040', [0.07, 0.05, 0.04]),
        ('AL_PLATE_080_060_020', [0.08, 0.06, 0.02]),
    ):
        assert catalog[identity]['geometry']['nominal_dimensions_m'] == dimensions
        assert catalog[identity]['mass_kg'] > 0


@pytest.mark.parametrize('field,value', [
    ('dimensions', [0.06, 0.04, -0.03]),
    ('quaternion', [0, 0, 0, 0]),
    ('mass', True),
])
def test_rejects_unusable_material(tmp_path, field, value):
    material = yaml.safe_load((MATERIALS / 'AL_BILLET_060_040_030.yaml').read_text())
    if field == 'dimensions':
        material['geometry']['nominal_dimensions_m'] = value
    elif field == 'quaternion':
        material['pick_offset']['quaternion_xyzw'] = value
    elif field == 'mass':
        material['mass_kg'] = value
    (tmp_path / 'AL_BILLET_060_040_030.yaml').write_text(yaml.safe_dump(material))
    with pytest.raises(ValueError):
        load_material_catalog(tmp_path)


def test_validation_uses_material_offset_without_recipe_width_policy(tmp_path):
    import json
    from arm_cell_bringup.runtime import export_runtime_configuration
    profile = MATERIALS.parent / 'pnp_validation_profile.yaml'
    output = export_runtime_configuration(
        profile, tmp_path / 'plate.json', material_id='AL_PLATE_080_060_020',
    )
    config = json.loads(output.read_text())['configuration']
    assert config['fixture']['dimensions_m'] == [0.08, 0.06, 0.02]
    assert config['motion']['object_to_grasp_tcp_translation'] == [0, 0, 0]
    recipe = config['_mission_recipe']
    assert 'min_expected_contact_width_mm' not in recipe['grasp']
    assert 'max_expected_contact_width_mm' not in recipe['grasp']
    assert recipe['object_to_grasp_tcp']['translation_m'] == [0, 0, 0]
    assert recipe['object_to_grasp_tcp']['quaternion_xyzw'] == [1, 0, 0, 0]
    # Production recipes also leave contact width to Motion.
    from arm_cell_bringup.runtime import compile_mission
    ordinary = compile_mission(MATERIALS.parent / 'missions/AL_PLATE_080_060_020.yaml')[0]
    assert 'max_expected_contact_width_mm' not in ordinary['grasp']


def test_calibration_trial_does_not_modify_catalog(tmp_path):
    import json
    from arm_cell_bringup.runtime import export_runtime_configuration
    original = (MATERIALS / 'AL_PLATE_080_060_020.yaml').read_bytes()
    result = export_runtime_configuration(
        MATERIALS.parent / 'pnp_validation_profile.yaml', tmp_path / 'trial.json',
        material_id='AL_PLATE_080_060_020', pick_offset_z_m=0.005,
    )
    assert json.loads(result.read_text())['configuration']['motion']['object_to_grasp_tcp_translation'][2] == 0.005
    assert (MATERIALS / 'AL_PLATE_080_060_020.yaml').read_bytes() == original


@pytest.mark.parametrize('material_id', [
    'TestCube', 'AL_BILLET_060_040_030',
    'STEEL_BLANK_070_050_040', 'AL_PLATE_080_060_020',
])
def test_compiled_material_grasp_reference_is_volume_center(material_id):
    from arm_cell_bringup.runtime import compile_mission
    recipe, material = compile_mission(MATERIALS.parent / f'missions/{material_id}.yaml')
    assert material['pick_offset']['translation_m'] == [0.0, 0.0, 0.0]
    assert recipe['object_to_grasp_tcp']['translation_m'] == [0.0, 0.0, 0.0]
