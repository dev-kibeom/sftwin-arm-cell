"""Repository-local, ignored output locations for M0609 operator tooling."""

from pathlib import Path

from shared.script_editor_bootstrap import discover_module_root

ARTIFACT_ROOT_RELATIVE_PATH = Path(".local_artifacts/m0609_cell")


def project_root(script_path, environ=None):
    """Resolve the repository root from the validated module root, never cwd."""
    module_root = discover_module_root(script_path, environ)
    for candidate in (module_root, *module_root.parents):
        if (candidate / "infra/isaac_sim/scripts/m0609_cell").is_dir() and (
            candidate / "ros2_ws/src/arm_cell"
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


def default_artifact_path(tool, filename, script_path, environ=None):
    """Return and create the parent for a stable tool-owned output file."""
    path = (
        project_root(script_path, environ)
        / ARTIFACT_ROOT_RELATIVE_PATH
        / tool
        / filename
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def default_artifact_directory(tool, script_path, environ=None):
    """Return and create a stable tool-owned output directory."""
    directory = project_root(script_path, environ) / ARTIFACT_ROOT_RELATIVE_PATH / tool
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def output_path(override, tool, filename, script_path, environ=None):
    """Use an explicit operator path or the stable local artifact path."""
    path = (
        Path(override).expanduser()
        if override
        else default_artifact_path(tool, filename, script_path, environ)
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    return path
