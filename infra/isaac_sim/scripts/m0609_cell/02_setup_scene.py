"""Script Editor-safe entrypoint for the M0609 cell scene composition."""

import os
import sys
from pathlib import Path

from pxr import Gf, UsdGeom, UsdLux

try:
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.prims import delete_prim
    from isaacsim.core.utils.stage import add_reference_to_stage
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.utils.extensions import enable_extension
    from omni.isaac.core.utils.prims import delete_prim
    from omni.isaac.core.utils.stage import add_reference_to_stage

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    try:
        from isaacsim.core.utils.nucleus import get_assets_root_path
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path


def _add_cell_module_root():
    """Make sibling builders importable from a temporary Script Editor copy."""
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        module_root = (
            Path(configured_root).expanduser()
            / "infra"
            / "isaac_sim"
            / "scripts"
            / "m0609_cell"
        )
    else:
        module_root = Path(__file__).resolve().parent
    if not (module_root / "scene_builder").is_dir():
        raise RuntimeError(
            "Cannot locate M0609 scene builders. When running from Isaac Script "
            "Editor, set SFTWIN_PROJECT_ROOT to the repository root."
        )
    if str(module_root) not in sys.path:
        sys.path.insert(0, str(module_root))


_add_cell_module_root()

from scene_builder.composition import run_scene_build

run_scene_build(
    world_cls=World,
    enable_extension=enable_extension,
    delete_prim=delete_prim,
    get_assets_root_path=get_assets_root_path,
    add_reference_to_stage=add_reference_to_stage,
    gf=Gf,
    usd_geom=UsdGeom,
    usd_lux=UsdLux,
    script_path=__file__,
)
