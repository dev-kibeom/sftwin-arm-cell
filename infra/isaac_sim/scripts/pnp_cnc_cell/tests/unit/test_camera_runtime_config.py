"""Generated camera inputs preserve bytes when observations have not changed."""
import json
from camera_tooling import stage_inspector


def test_same_camera_values_preserve_existing_bytes_and_mtime(tmp_path):
    path = tmp_path / 'camera.json'
    before = '{"camera_binding": {"prim": "/Camera"}, "translation": [0.0, 1.0, 2.0]}\n'
    path.write_text(before)
    stamp = path.stat().st_mtime_ns
    # Producer order and tuple/list representations must not cause a rewrite.
    data = {'translation': (0., 1., 2.), 'camera_binding': {'prim': '/Camera'}}
    assert not stage_inspector.write_camera_runtime_config(path, data)
    assert path.read_text() == before
    assert path.stat().st_mtime_ns == stamp


def test_actual_camera_change_is_saved_without_rounding(tmp_path):
    path = tmp_path / 'camera.json'
    assert stage_inspector.write_camera_runtime_config(path, {'translation': [0., 1., 2.]})
    changed = {'translation': [0., 1., 2.000000001]}
    assert stage_inspector.write_camera_runtime_config(path, changed)
    assert json.loads(path.read_text()) == changed


def test_invalid_previous_camera_file_is_replaced_with_valid_json(tmp_path):
    path = tmp_path / 'camera.json'
    path.write_text('{incomplete')
    assert stage_inspector.write_camera_runtime_config(path, {'camera': '/Camera'})
    assert json.loads(path.read_text()) == {'camera': '/Camera'}
