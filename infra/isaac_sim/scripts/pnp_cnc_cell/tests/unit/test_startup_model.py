"""A deployment can explicitly prepare unpublished assets before scene construction."""
import pytest

from isaac_runtime import runtime_workflow as workflow


@pytest.mark.parametrize('enabled,complete,expected', [
    (False, False, []), (True, False, ['build_robot_usd.py']), (True, True, [])])
def test_startup_preparation_is_opt_in_and_reuses_common_builder(tmp_path, monkeypatch,
                                                               enabled, complete, expected):
    config = {'model_configuration': {'model_id': 'selected', 'urdf': '/source.xacro',
              'mesh_root': '/mesh', 'isaac': {'build_missing_on_startup': enabled}}}
    monkeypatch.setattr(workflow, 'load_runtime_configuration', lambda: config)
    if complete:
        from isaac_runtime.model_build_configuration import ModelBuildConfiguration
        model = ModelBuildConfiguration.from_runtime(config, tmp_path)
        for path in (model.generated_urdf, model.usd, model.provenance):
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
    monkeypatch.setattr('isaac_runtime.model_provenance.verify_current_artifact', lambda *a, **kw: None)
    observed = []
    monkeypatch.setattr(workflow, 'run_step', lambda filename, ns: observed.append(filename))
    namespace = {}
    workflow.prepare_startup_model(namespace, tmp_path)
    assert observed == expected
    assert namespace['runtime_configuration'] is config


def test_asset_build_failure_stops_startup(tmp_path, monkeypatch):
    config = {'model_configuration': {'model_id': 'selected', 'urdf': '/source.xacro',
              'mesh_root': '/mesh', 'isaac': {'build_missing_on_startup': True}}}
    monkeypatch.setattr(workflow, 'load_runtime_configuration', lambda: config)
    def fail(*args):
        raise RuntimeError('import failed')
    monkeypatch.setattr(workflow, 'run_step', fail)
    with pytest.raises(RuntimeError, match='import failed'):
        workflow.prepare_startup_model({}, tmp_path)


def test_changed_model_rebuilds_existing_derived_assets(tmp_path, monkeypatch):
    from isaac_runtime.model_build_configuration import ModelBuildConfiguration
    config = {'model_configuration': {'model_id': 'selected', 'urdf': '/source.xacro',
              'mesh_root': '/mesh', 'isaac': {'build_missing_on_startup': True}}}
    monkeypatch.setattr(workflow, 'load_runtime_configuration', lambda: config)
    model = ModelBuildConfiguration.from_runtime(config, tmp_path)
    for path in (model.generated_urdf, model.usd, model.provenance):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    def stale(*args, **kwargs):
        raise ValueError('canonical Xacro digest does not match robot artifact provenance')
    monkeypatch.setattr('isaac_runtime.model_provenance.verify_current_artifact', stale)
    observed = []
    monkeypatch.setattr(workflow, 'run_step', lambda filename, ns: observed.append(filename))
    workflow.prepare_startup_model({}, tmp_path)
    assert observed == ['build_robot_usd.py']
