"""Deterministic provenance for the derived Isaac robot artifact."""

import hashlib
import json
import os
import subprocess
from pathlib import Path


SCHEMA_VERSION = 2


def xacro_subprocess_environment():
    """Keep Kit's Python 3.11 paths out of ROS tools using system Python."""
    environment = os.environ.copy()
    isaac_root_value = environment.get("ISAAC_PATH")
    if not isaac_root_value:
        return environment
    isaac_root = Path(isaac_root_value).resolve()
    for variable in ("PYTHONPATH", "LD_LIBRARY_PATH"):
        entries = environment.get(variable, "").split(os.pathsep)
        retained = []
        for entry in filter(None, entries):
            lexical_path = Path(entry).absolute()
            resolved = Path(entry).resolve()
            under_isaac_install = (
                lexical_path == isaac_root
                or isaac_root in lexical_path.parents
                or resolved == isaac_root
                or isaac_root in resolved.parents
            )
            if not under_isaac_install:
                retained.append(entry)
        environment[variable] = os.pathsep.join(retained)
    return environment


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xacro_output_hash(xacro_path: Path, executable: str = "xacro") -> str:
    try:
        expanded = subprocess.check_output(
            [executable, str(xacro_path)], env=xacro_subprocess_environment()
        )
    except FileNotFoundError as error:
        raise RuntimeError(
            "xacro is required to verify the derived Isaac robot artifact"
        ) from error
    except subprocess.CalledProcessError as error:
        raise RuntimeError("canonical robot Xacro expansion failed") from error
    return hashlib.sha256(expanded).hexdigest()


def build_manifest(
    canonical_xacro: Path,
    generated_urdf: Path,
    usd: Path,
    executable: str = "xacro",
) -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "canonical_xacro_output_sha256": xacro_output_hash(canonical_xacro, executable),
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


def verify_manifest(
    manifest: dict,
    canonical_xacro: Path,
    generated_urdf: Path,
    usd: Path,
    executable: str = "xacro",
) -> None:
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("robot artifact provenance schema is unsupported")
    expected = {
        "canonical Xacro": xacro_output_hash(canonical_xacro, executable),
        "generated URDF": sha256_file(generated_urdf),
        "USD": sha256_file(usd),
    }
    recorded = {
        "canonical Xacro": manifest.get("canonical_xacro_output_sha256"),
        "generated URDF": manifest.get("generated_urdf_sha256"),
        "USD": manifest.get("usd_sha256"),
    }
    for label, digest in expected.items():
        if recorded[label] != digest:
            if label == "canonical Xacro":
                raise ValueError(
                    "canonical Xacro digest does not match robot artifact provenance"
                )
            raise ValueError(f"{label} digest does not match robot artifact provenance")


def verify_current_artifact(
    xacro_path: Path,
    generated_urdf: Path,
    usd: Path,
    provenance: Path,
    executable: str = "xacro",
) -> None:
    manifest = load_manifest(provenance)
    verify_manifest(manifest, xacro_path, generated_urdf, usd, executable)
