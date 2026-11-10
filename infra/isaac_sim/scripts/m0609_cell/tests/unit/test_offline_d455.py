import pytest
from scene_builder.d455_camera import local_d455_asset


def test_local_asset_does_not_require_server(tmp_path, monkeypatch):
    asset = tmp_path / 'rsd455.usd'
    asset.touch()
    monkeypatch.setenv('SFTWIN_D455_ASSET', str(asset))
    assert local_d455_asset() == str(asset)


def test_missing_asset_explains_preparation(tmp_path, monkeypatch):
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(tmp_path))
    monkeypatch.delenv('SFTWIN_D455_ASSET', raising=False)
    with pytest.raises(FileNotFoundError, match='prepare_d455_assets.py'):
        local_d455_asset()


def test_remote_override_rejected(monkeypatch):
    monkeypatch.setenv('SFTWIN_D455_ASSET', 'https://example.org/camera.usd')
    with pytest.raises(ValueError, match='local'):
        local_d455_asset()
