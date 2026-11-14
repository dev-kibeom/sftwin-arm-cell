"""Explicit project-root bootstrap for PnP_CNC Script Editor entrypoints.

Isaac Script Editor executes a temporary ``/tmp/carb.../script_*.py`` copy.
Those copies have no trustworthy repository-relative ``__file__`` location, so
``SFTWIN_PROJECT_ROOT`` is mandatory there.  Ordinary source execution may use
its source path; this resolver intentionally never treats the current directory
as repository authority.
"""

import os
from pathlib import Path
import sys

CELL_RELATIVE_PATH = Path("infra/isaac_sim/scripts/pnp_cnc_cell")


def is_script_editor_path(script_path):
    path = Path(script_path)
    return path.name.startswith("script_") and any(
        parent.name.startswith("carb") for parent in path.parents
    )


def module_root_from_project_root(project_root):
    root = Path(project_root).expanduser()
    if not root.is_dir():
        raise RuntimeError(f"SFTWIN_PROJECT_ROOT is not a directory: {root}")
    module_root = root / CELL_RELATIVE_PATH
    if not (module_root / "shared").is_dir():
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT does not contain PnP_CNC Script Editor support: "
            f"{module_root}"
        )
    return module_root


def discover_module_root(script_path, environ=None):
    """Resolve the PnP_CNC module root without consulting the current directory."""
    environment = os.environ if environ is None else environ
    configured_root = environment.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        return module_root_from_project_root(configured_root)

    source = Path(script_path).resolve()
    if is_script_editor_path(source):
        raise RuntimeError(
            "Cannot locate PnP_CNC modules from an Isaac Script Editor temporary "
            "script. Set SFTWIN_PROJECT_ROOT to the repository root."
        )
    for ancestor in (source.parent, *source.parents):
        if ancestor.name == "pnp_cnc_cell" and (ancestor / "shared").is_dir():
            return ancestor
        nested = ancestor / "pnp_cnc_cell"
        if (nested / "shared").is_dir():
            return nested
    raise RuntimeError(
        "Cannot locate PnP_CNC modules from this source path. Set "
        "SFTWIN_PROJECT_ROOT to the repository root."
    )


def add_module_root(script_path, environ=None):
    """Add the validated PnP_CNC module root to ``sys.path`` and return it."""
    module_root = discover_module_root(script_path, environ)
    as_string = str(module_root)
    if as_string not in sys.path:
        sys.path.insert(0, as_string)
    return module_root


def clear_imported_package(package_name, module_cache=None):
    """Remove one package and its children from a persistent Script Editor."""
    cache = sys.modules if module_cache is None else module_cache
    prefix = f"{package_name}."
    for name in tuple(cache):
        if name == package_name or name.startswith(prefix):
            del cache[name]


def clear_package_bytecode(package_path):
    """Remove cached bytecode before reloading a live Script Editor package."""
    cache_path = Path(package_path) / "__pycache__"
    if not cache_path.is_dir():
        return 0
    removed = 0
    for bytecode_path in cache_path.glob("*.pyc"):
        bytecode_path.unlink(missing_ok=True)
        removed += 1
    return removed
