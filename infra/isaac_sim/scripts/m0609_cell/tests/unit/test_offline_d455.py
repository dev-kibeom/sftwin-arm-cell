import pytest
from scene_builder.d455_camera import local_d455_asset


def test_local_asset_does_not_require_server(tmp_path, monkeypatch):
    asset = tmp_path / 'rsd455.usd'
    asset.touch()
    monkeypatch.setenv('SFTWIN_D455_ASSET', str(asset))
    assert local_d455_asset() == str(asset)


def test_missing_asset_explains_checkout(tmp_path, monkeypatch):
    monkeypatch.setenv('SFTWIN_PROJECT_ROOT', str(tmp_path))
    monkeypatch.delenv('SFTWIN_D455_ASSET', raising=False)
    with pytest.raises(FileNotFoundError, match='checkout'):
        local_d455_asset()


def test_remote_override_rejected(monkeypatch):
    monkeypatch.setenv('SFTWIN_D455_ASSET', 'https://example.org/camera.usd')
    with pytest.raises(ValueError, match='local'):
        local_d455_asset()


def test_checkout_contains_default_asset(monkeypatch):
    from pathlib import Path
    root = Path(__file__).resolve().parents[6]
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(root))
    monkeypatch.delenv("SFTWIN_D455_ASSET", raising=False)
    assert local_d455_asset() == str(root / "infra/isaac_sim/assets/sensors/d455/rsd455.usd")


def test_checkout_includes_collected_dependencies(monkeypatch):
    import json
    from pathlib import Path

    monkeypatch.delenv("SFTWIN_PROJECT_ROOT", raising=False)
    monkeypatch.delenv("SFTWIN_D455_ASSET", raising=False)
    asset = Path(local_d455_asset())
    manifest = json.loads((asset.parent / ".collect.mapping.json").read_text())
    assert len(manifest["file_records"]) > 1
    for record in manifest["file_records"]:
        relative = record["target_url"]
        assert "://" not in relative
        assert (asset.parent / relative).is_file(), relative
