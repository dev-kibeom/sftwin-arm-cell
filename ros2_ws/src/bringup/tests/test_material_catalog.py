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
    ('width', 90),
])
def test_rejects_unusable_material(tmp_path, field, value):
    material = yaml.safe_load((MATERIALS / 'AL_BILLET_060_040_030.yaml').read_text())
    if field == 'dimensions':
        material['geometry']['nominal_dimensions_m'] = value
    elif field == 'quaternion':
        material['pick_offset']['quaternion_xyzw'] = value
    elif field == 'mass':
        material['mass_kg'] = value
    else:
        material['grasp']['min_expected_contact_width_mm'] = value
    (tmp_path / 'AL_BILLET_060_040_030.yaml').write_text(yaml.safe_dump(material))
    with pytest.raises(ValueError):
        load_material_catalog(tmp_path)


def test_validation_uses_material_offset_and_explicit_80mm_trial(tmp_path):
    import json
    from arm_cell_bringup.runtime import export_runtime_configuration
    profile = MATERIALS.parent / 'pnp_validation_profile.yaml'
    output = export_runtime_configuration(
        profile, tmp_path / 'plate.json', material_id='AL_PLATE_080_060_020',
        grasp_width_mm=80.0,
    )
    config = json.loads(output.read_text())['configuration']
    assert config['fixture']['dimensions_m'] == [0.08, 0.06, 0.02]
    assert config['motion']['object_to_grasp_tcp_translation'] == [0, 0, 0.0075]
    recipe = config['_mission_recipe']
    assert recipe['grasp']['min_expected_contact_width_mm'] == 79
    assert recipe['grasp']['max_expected_contact_width_mm'] == 81
    assert recipe['object_to_grasp_tcp']['translation_m'] == [0, 0, 0.0075]
    assert recipe['object_to_grasp_tcp']['quaternion_xyzw'] == [1, 0, 0, 0]
    # Ordinary production still excludes the 80 mm direction.
    from arm_cell_bringup.runtime import compile_mission
    ordinary = compile_mission(MATERIALS.parent / 'missions/AL_PLATE_080_060_020.yaml')[0]
    assert ordinary['grasp']['max_expected_contact_width_mm'] < 80


@pytest.mark.parametrize('width', [85.1, 70.0, float('nan'), 0.0])
def test_validation_rejects_width_without_matching_contact_faces(tmp_path, width):
    from arm_cell_bringup.runtime import export_runtime_configuration
    with pytest.raises(ValueError, match='grasp width'):
        export_runtime_configuration(
            MATERIALS.parent / 'pnp_validation_profile.yaml', tmp_path / 'trial.json',
            material_id='AL_PLATE_080_060_020', grasp_width_mm=width,
        )


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


def test_explicit_aperture_trial_tolerates_float_observation_rounding(tmp_path):
    import json
    import struct
    from arm_cell_bringup.runtime import export_runtime_configuration
    result = export_runtime_configuration(
        MATERIALS.parent / 'pnp_validation_profile.yaml', tmp_path / 'trial.json',
        material_id='AL_PLATE_080_060_020', grasp_width_mm=80.0,
    )
    grasp = json.loads(result.read_text())['configuration']['_mission_recipe']['grasp']
    observed_width_mm = struct.unpack('f', struct.pack('f', 0.08))[0] * 1000.0
    assert observed_width_mm < 80.0
    assert grasp['min_expected_contact_width_mm'] <= observed_width_mm <= grasp['max_expected_contact_width_mm']
    assert not grasp['min_expected_contact_width_mm'] <= 60.0 <= grasp['max_expected_contact_width_mm']
