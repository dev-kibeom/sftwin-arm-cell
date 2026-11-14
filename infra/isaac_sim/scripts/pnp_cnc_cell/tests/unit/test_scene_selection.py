"""Cell selection and Assembly placement at the Isaac scene boundary."""
import copy
import json
import sys
from types import SimpleNamespace

import pytest

from runtime_configuration import load_runtime_configuration


def test_selected_cell_builder_receives_runtime_and_scene_dependencies(exported_configuration, monkeypatch):
    from scene_entrypoint import run_scene_build
    config = load_runtime_configuration(exported_configuration)
    config['cell']['isaac']['scene_builder'] = 'alternate_cell:build'
    expected = object()
    calls = []
    monkeypatch.setitem(sys.modules, 'alternate_cell', SimpleNamespace(
        build=lambda **kwargs: calls.append(kwargs) or expected))
    assert run_scene_build(configuration=config, stage='stage') is expected
    assert calls == [dict(configuration=config, stage='stage')]


def test_default_scene_inputs_and_alternate_assembly(exported_configuration, tmp_path):
    from scene_builder.scene_configuration import SceneConfiguration
    config = load_runtime_configuration(exported_configuration)
    selected = SceneConfiguration.from_runtime(config, tmp_path)
    assert selected.robot_prim_path == '/World/SF_Twin_Cell/ARM'
    assert selected.position_m == (0.15, -0.15, 0.68)
    assert selected.rotation_deg == (0.0, 0.0, 0.0)
    assert selected.model.usd == tmp_path / 'infra/isaac_sim/assets/usd/m0609_robotiq_2f85_generated.usd'
    assert selected.geometry == config['cell']['geometry']
    other = copy.deepcopy(config)
    other['model_configuration']['model_id'] = 'another_assembly'
    other['cell']['isaac']['robot_prim_path'] = '/World/Selected/Robot'
    other['cell']['world_to_base']['yaw'] = 1.5707963267948966
    selected = SceneConfiguration.from_runtime(other, tmp_path)
    assert selected.model.usd.name == 'another_assembly_generated.usd'
    assert selected.robot_prim_path == '/World/Selected/Robot'
    assert selected.rotation_deg == (0.0, 0.0, 90.0)


def test_selected_static_geometry_is_consumed(exported_configuration, tmp_path):
    from scene_builder.static_environment import load_canonical_static_geometry
    from pathlib import Path
    config = load_runtime_configuration(exported_configuration)
    geometry = tmp_path / 'selected_geometry.json'
    manifest = json.loads(Path(config['cell']['geometry']).read_text())
    manifest['objects'][0]['geometry']['dimensions_m'][0] += 1.0
    geometry.write_text(json.dumps(manifest))
    actual = load_canonical_static_geometry(manifest_path=geometry)
    first = manifest['objects'][0]
    assert actual[first['id']]['geometry'] == first['geometry']
    assert actual != load_canonical_static_geometry(manifest_path=config['cell']['geometry'])


@pytest.mark.parametrize("alternate", [False, True])
def test_robot_station_authors_selected_reference_and_cell_transform(exported_configuration, tmp_path, monkeypatch, alternate):
    from scene_builder.scene_configuration import SceneConfiguration
    from scene_builder import robot_station
    config = load_runtime_configuration(exported_configuration)
    if alternate:
        config['model_configuration']['model_id'] = 'selected_assembly'
        config['cell']['isaac']['robot_prim_path'] = '/World/Chosen/Robot'
        config['cell']['world_to_base'].update(x=0.3, z=0.8, yaw=1.5707963267948966)
    selected = SceneConfiguration.from_runtime(config, tmp_path)
    selected.model.usd.parent.mkdir(parents=True)
    selected.model.usd.touch()
    verified, references, transforms, boxes = [], [], [], []
    monkeypatch.setattr(robot_station, 'verify_current_artifact',
                        lambda *args, **kwargs: verified.append((args, kwargs)))
    prim = SimpleNamespace(IsValid=lambda: True, SetCustomDataByKey=lambda *args: None)
    xform = SimpleNamespace(
        ClearXformOpOrder=lambda: None,
        AddTranslateOp=lambda **kwargs: SimpleNamespace(Set=lambda value: transforms.append(value)),
        AddRotateXYZOp=lambda: SimpleNamespace(Set=lambda value: transforms.append(value)))
    context = SimpleNamespace(
        scene_configuration=selected,
        stage=SimpleNamespace(GetPrimAtPath=lambda path: prim),
        cell_root='/World/SF_Twin_Cell',
        colors=dict(charcoal=0, steel=0, black=0),
        materials=dict(mat_frame=0, mat_steel=0, mat_black=0),
        gf=SimpleNamespace(Vec3d=lambda *v: v, Vec3f=lambda *v: v),
        usd_geom=SimpleNamespace(Xformable=lambda p: xform,
                                 XformOp=SimpleNamespace(PrecisionDouble='double')),
        static_box=lambda *args: boxes.append(args[0]),
        create_cylinder=lambda *args: None,
        add_reference_to_stage=lambda *args, **kwargs: references.append(args))
    robot_station.build_robot_station(context)
    expected_prim = '/World/Chosen/Robot' if alternate else '/World/SF_Twin_Cell/ARM'
    expected_usd = ('selected_assembly_generated.usd' if alternate
                    else 'm0609_robotiq_2f85_generated.usd')
    assert references == [(str(tmp_path / 'infra/isaac_sim/assets/usd' / expected_usd), expected_prim)]
    assert transforms == ([(0.3, -0.15, 0.8), (0.0, 0.0, 90.0)] if alternate
                          else [(0.15, -0.15, 0.68), (0.0, 0.0, 0.0)])
    assert verified == [((selected.model.source, selected.model.generated_urdf,
                          selected.model.usd, selected.model.provenance),
                         dict(xacro_arguments=selected.model.xacro_arguments))]
    assert boxes == ['pedestal', 'pedestal_top', 'robot_riser']


def test_unavailable_cell_builder_does_not_fall_back(exported_configuration):
    from scene_entrypoint import run_scene_build
    config = load_runtime_configuration(exported_configuration)
    config['cell']['isaac']['scene_builder'] = 'unavailable_cell_scene:build'
    with pytest.raises(ModuleNotFoundError, match='unavailable_cell_scene'):
        run_scene_build(configuration=config)


def test_script_editor_temporary_scene_step_uses_common_entrypoint(exported_configuration, tmp_path, monkeypatch):
    import runpy
    from pathlib import Path
    from pnp_cnc_cell.scene_builder import composition

    root = Path(__file__).resolve().parents[6]
    scripts = root / 'infra/isaac_sim/scripts'
    config = load_runtime_configuration(exported_configuration)
    received = []

    class SceneReached(Exception):
        pass

    def build(**kwargs):
        received.append(kwargs['configuration'])
        raise SceneReached

    monkeypatch.setattr(composition, 'run_scene_build', build)
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(root))
    monkeypatch.setattr(sys, 'path', [path for path in sys.path if path != str(scripts)])
    monkeypatch.delitem(sys.modules, 'scene_entrypoint', raising=False)
    monkeypatch.setitem(sys.modules, 'pxr', SimpleNamespace(
        Gf=object(), UsdGeom=object(), UsdLux=object(), UsdPhysics=object()))
    monkeypatch.setitem(sys.modules, 'isaacsim.core.api', SimpleNamespace(World=object()))
    monkeypatch.setitem(sys.modules, 'isaacsim.core.utils.extensions', SimpleNamespace(enable_extension=object()))
    monkeypatch.setitem(sys.modules, 'isaacsim.core.utils.prims', SimpleNamespace(delete_prim=object()))
    monkeypatch.setitem(sys.modules, 'isaacsim.core.utils.stage', SimpleNamespace(add_reference_to_stage=object()))
    script = tmp_path / 'carb.script_editor' / 'script_123.py'
    script.parent.mkdir()
    script.write_text((scripts / 'pnp_cnc_cell/runtime_steps/setup_scene.py').read_text())
    with pytest.raises(SceneReached):
        runpy.run_path(str(script))
    assert received == [config]
    assert Path(sys.modules['scene_entrypoint'].__file__) == scripts / 'scene_entrypoint.py'
