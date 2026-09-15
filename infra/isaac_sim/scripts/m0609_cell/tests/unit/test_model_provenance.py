import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from model_provenance import build_manifest, verify_manifest, xacro_output_hash
from scene_builder import robot_station


def test_manifest_binds_generated_usd_to_source_and_generated_urdf(tmp_path):
    source = tmp_path / "source.urdf"
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    source.write_text("canonical-model\n")
    generated.write_text("generated-model\n")
    usd.write_bytes(b"usd-artifact")

    manifest = build_manifest(source, generated, usd)
    verify_manifest(manifest, source, generated, usd)

    usd.write_bytes(b"stale-usd-artifact")
    with pytest.raises(ValueError, match="USD digest"):
        verify_manifest(manifest, source, generated, usd)


def test_xacro_output_hash_uses_expanded_source_bytes(tmp_path):
    xacro = tmp_path / "xacro"
    xacro.write_text("#!/bin/sh\nprintf 'expanded canonical model\\n'\n")
    xacro.chmod(0o755)
    expected = hashlib.sha256(b"expanded canonical model\n").hexdigest()

    assert xacro_output_hash(tmp_path / "model.xacro", str(xacro)) == expected


def test_scene_builder_refuses_stale_robot_artifact_before_referencing_it(
    tmp_path, monkeypatch
):
    (tmp_path / "infra/isaac_sim/assets/usd").mkdir(parents=True)
    (tmp_path / "infra/isaac_sim/assets/usd/m0609_robotiq_2f85_generated.usd").write_bytes(
        b"stale"
    )
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(tmp_path))
    monkeypatch.setattr(
        robot_station,
        "verify_current_artifact",
        lambda *args: (_ for _ in ()).throw(ValueError("stale robot artifact")),
    )
    references = []
    context = SimpleNamespace(
        stage=SimpleNamespace(),
        cell_root="/World/SF_Twin_Cell",
        colors={"charcoal": "charcoal", "steel": "steel", "black": "black"},
        gf=SimpleNamespace(),
        usd_geom=SimpleNamespace(),
        static_box=lambda *args: None,
        create_cylinder=lambda *args: None,
        add_reference_to_stage=lambda *args, **kwargs: references.append(args),
        materials={"mat_frame": object(), "mat_steel": object(), "mat_black": object()},
    )

    with pytest.raises(ValueError, match="stale robot artifact"):
        robot_station.build_robot_station(context)
    assert references == []
