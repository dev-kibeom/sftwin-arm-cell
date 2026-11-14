"""Deterministic provenance for the derived Isaac robot artifact."""

import hashlib
import json
import subprocess
from pathlib import Path


SCHEMA_VERSION = 1


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xacro_output_hash(xacro_path: Path, executable: str = "xacro") -> str:
    try:
        expanded = subprocess.check_output([executable, str(xacro_path)])
    except FileNotFoundError as error:
        raise RuntimeError("xacro is required to verify the derived Isaac robot artifact") from error
    except subprocess.CalledProcessError as error:
        raise RuntimeError("canonical robot Xacro expansion failed") from error
    return hashlib.sha256(expanded).hexdigest()


def build_manifest(canonical_urdf: Path, generated_urdf: Path, usd: Path) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "canonical_urdf_sha256": sha256_file(canonical_urdf),
        "generated_urdf_sha256": sha256_file(generated_urdf),
        "usd_sha256": sha256_file(usd),
    }


def write_manifest(path: Path, manifest: dict) -> None:
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def load_manifest(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as error:
        raise RuntimeError(f"robot artifact provenance is missing: {path}") from error
    except json.JSONDecodeError as error:
        raise RuntimeError(f"robot artifact provenance is malformed: {path}") from error


def verify_manifest(manifest: dict, canonical_urdf: Path, generated_urdf: Path, usd: Path) -> None:
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("robot artifact provenance schema is unsupported")
    expected = {
        "canonical URDF": sha256_file(canonical_urdf),
        "generated URDF": sha256_file(generated_urdf),
        "USD": sha256_file(usd),
    }
    recorded = {
        "canonical URDF": manifest.get("canonical_urdf_sha256"),
        "generated URDF": manifest.get("generated_urdf_sha256"),
        "USD": manifest.get("usd_sha256"),
    }
    for label, digest in expected.items():
        if recorded[label] != digest:
            raise ValueError(f"{label} digest does not match robot artifact provenance")


def verify_current_artifact(
    xacro_path: Path, generated_urdf: Path, usd: Path, provenance: Path
) -> None:
    manifest = load_manifest(provenance)
    if manifest.get("canonical_urdf_sha256") != xacro_output_hash(xacro_path):
        raise ValueError("canonical Xacro digest does not match robot artifact provenance")
    verify_manifest(manifest, generated_urdf, generated_urdf, usd)
