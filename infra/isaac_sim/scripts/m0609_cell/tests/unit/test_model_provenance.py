import hashlib
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from scene_builder.model_provenance import (
    build_manifest,
    verify_current_artifact,
    verify_manifest,
    write_manifest,
    xacro_output_hash,
    xacro_subprocess_environment,
)
from scene_builder import robot_station


def fake_xacro(tmp_path):
    xacro = tmp_path / "model.xacro"
    xacro.write_text("canonical-model\n")
    executable = tmp_path / "xacro"
    executable.write_text('#!/bin/sh\ncat "$1"\n')
    executable.chmod(0o755)
    return xacro, str(executable)


def test_fresh_pipeline_identity_uses_distinct_source_and_artifact_hashes(tmp_path):
    source, executable = fake_xacro(tmp_path)
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    generated.write_text("generated-model\n")
    usd.write_bytes(b"usd-artifact")
    provenance = tmp_path / "robot.provenance.json"

    manifest = build_manifest(source, generated, usd, executable)
    write_manifest(provenance, manifest)

    assert manifest["schema_version"] == 2
    assert manifest["canonical_xacro_output_sha256"] == xacro_output_hash(
        source, executable
    )
    assert (
        manifest["canonical_xacro_output_sha256"] != manifest["generated_urdf_sha256"]
    )
    verify_current_artifact(source, generated, usd, provenance, executable)


def test_producer_and_verifier_share_canonical_xacro_digest(tmp_path):
    source, executable = fake_xacro(tmp_path)
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    generated.write_text("derived-urdf\n")
    usd.write_bytes(b"usd-artifact")

    manifest = build_manifest(source, generated, usd, executable)

    assert manifest["canonical_xacro_output_sha256"] == xacro_output_hash(
        source, executable
    )


def test_canonical_source_change_fails_current_artifact_verification(tmp_path):
    source, executable = fake_xacro(tmp_path)
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    provenance = tmp_path / "robot.provenance.json"
    generated.write_text("derived-urdf\n")
    usd.write_bytes(b"usd-artifact")
    write_manifest(provenance, build_manifest(source, generated, usd, executable))

    source.write_text("canonical-model-changed\n")

    with pytest.raises(ValueError, match="canonical Xacro digest"):
        verify_current_artifact(source, generated, usd, provenance, executable)


def test_generated_urdf_change_fails_current_artifact_verification(tmp_path):
    source, executable = fake_xacro(tmp_path)
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    provenance = tmp_path / "robot.provenance.json"
    generated.write_text("derived-urdf\n")
    usd.write_bytes(b"usd-artifact")
    write_manifest(provenance, build_manifest(source, generated, usd, executable))

    generated.write_text("derived-urdf-tampered\n")

    with pytest.raises(ValueError, match="generated URDF digest"):
        verify_current_artifact(source, generated, usd, provenance, executable)


def test_usd_change_fails_current_artifact_verification(tmp_path):
    source, executable = fake_xacro(tmp_path)
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    provenance = tmp_path / "robot.provenance.json"
    generated.write_text("derived-urdf\n")
    usd.write_bytes(b"usd-artifact")
    write_manifest(provenance, build_manifest(source, generated, usd, executable))

    usd.write_bytes(b"stale-usd-artifact")
    with pytest.raises(ValueError, match="USD digest"):
        verify_current_artifact(source, generated, usd, provenance, executable)


def test_unsupported_provenance_schema_fails_closed(tmp_path):
    source, executable = fake_xacro(tmp_path)
    generated = tmp_path / "generated.urdf"
    usd = tmp_path / "robot.usd"
    generated.write_text("derived-urdf\n")
    usd.write_bytes(b"usd-artifact")

    manifest = build_manifest(source, generated, usd, executable)
    manifest["schema_version"] = 1

    with pytest.raises(ValueError, match="schema is unsupported"):
        verify_manifest(manifest, source, generated, usd, executable)


def test_xacro_output_hash_uses_expanded_source_bytes(tmp_path):
    xacro = tmp_path / "xacro"
    xacro.write_text("#!/bin/sh\nprintf 'expanded canonical model\\n'\n")
    xacro.chmod(0o755)
    expected = hashlib.sha256(b"expanded canonical model\n").hexdigest()

    assert xacro_output_hash(tmp_path / "model.xacro", str(xacro)) == expected


def test_xacro_environment_excludes_kit_python_and_library_paths(tmp_path, monkeypatch):
    isaac_root = tmp_path / "isaac"
    ros_python = "/opt/ros/humble/lib/python3.10/site-packages"
    ros_library = "/opt/ros/humble/lib"
    monkeypatch.setenv("ISAAC_PATH", str(isaac_root))
    monkeypatch.setenv(
        "PYTHONPATH",
        f"{ros_python}{os.pathsep}{isaac_root / 'kit/python/lib/python3.11'}",
    )
    monkeypatch.setenv(
        "LD_LIBRARY_PATH",
        f"{ros_library}{os.pathsep}{isaac_root / 'kit/lib'}",
    )

    environment = xacro_subprocess_environment()

    assert environment["PYTHONPATH"] == ros_python
    assert environment["LD_LIBRARY_PATH"] == ros_library


def test_scene_builder_refuses_stale_robot_artifact_before_referencing_it(
    tmp_path, monkeypatch
):
    (tmp_path / "infra/isaac_sim/assets/usd").mkdir(parents=True)
    (
        tmp_path / "infra/isaac_sim/assets/usd/m0609_robotiq_2f85_generated.usd"
    ).write_bytes(b"stale")
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
