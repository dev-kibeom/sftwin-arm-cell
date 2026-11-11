"""Repository-local, ignored output locations for M0609 operator tooling."""

from pathlib import Path
from datetime import datetime
import json
import os

import yaml

from shared.script_editor_bootstrap import discover_module_root

def run_records_root(checkout):
    """Resolve the configured record root relative to the checkout."""
    config_path = checkout / "config/run_records.yaml"
    config = (yaml.safe_load(config_path.read_text()) or {}) if config_path.is_file() else {}
    return (checkout / Path(config.get("root", ".run_records")).expanduser()).resolve()


def run_records_config(checkout):
    config_path = Path(checkout) / "config/run_records.yaml"
    return (yaml.safe_load(config_path.read_text()) or {}) if config_path.is_file() else {}


_RUN_DIRECTORIES = {}


def standalone_run_directory(artifact_root):
    """Reserve one output directory per process; mkdir arbitrates collisions."""
    root = Path(artifact_root).resolve()
    key = (os.getpid(), root)
    if key not in _RUN_DIRECTORIES:
        now = datetime.now().astimezone()
        parent = root / now.strftime("%Y%m%d")
        parent.mkdir(parents=True, exist_ok=True)
        suffix = 0
        while True:
            candidate = parent / (now.strftime("%H%M%S") + (f"-{suffix:02d}" if suffix else ""))
            try:
                candidate.mkdir()
                break
            except FileExistsError:
                suffix += 1
        _RUN_DIRECTORIES[key] = candidate
    return _RUN_DIRECTORIES[key]


def diagnostic_component_directory(checkout, component):
    config = run_records_config(checkout)
    diagnostic = config.get("directories", {}).get("diagnostic", "diagnostic")
    directory = standalone_run_directory(run_records_root(checkout)) / diagnostic
    directory.mkdir(parents=True, exist_ok=True)
    descriptor = directory / "session.json"
    if not descriptor.exists():
        descriptor.write_text(json.dumps({
            "schema_version": 1, "session_id": None,
            "session_association": "unassociated", "owner_pid": os.getpid(),
        }, indent=2) + "\n", encoding="utf-8")
    return directory / component


def project_root(script_path, environ=None):
    """Resolve the repository root from the validated module root, never cwd."""
    module_root = discover_module_root(script_path, environ)
    for candidate in (module_root, *module_root.parents):
        if (candidate / "infra/isaac_sim/scripts/m0609_cell").is_dir() and (
            candidate / "ros2_ws/src"
        ).is_dir():
            return candidate
    raise RuntimeError(
        "Validated M0609 module root is not contained by an SFTwin repository: "
        f"{module_root}"
    )


def project_relative_path(value, script_path, environ=None):
    """Resolve a relative operator path from the validated project root."""
    path = Path(value).expanduser()
    return path if path.is_absolute() else project_root(script_path, environ) / path


def default_run_record_path(tool, filename, script_path, environ=None):
    """Return and create the parent for a stable tool-owned output file."""
    root = project_root(script_path, environ)
    path = diagnostic_component_directory(root, tool) / filename
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def default_run_record_directory(tool, script_path, environ=None):
    """Return and create a stable tool-owned output directory."""
    directory = diagnostic_component_directory(project_root(script_path, environ), tool)
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def output_path(override, tool, filename, script_path, environ=None):
    """Use an explicit operator path or the default run record path."""
    path = (
        Path(override).expanduser()
        if override
        else default_run_record_path(tool, filename, script_path, environ)
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    return path
