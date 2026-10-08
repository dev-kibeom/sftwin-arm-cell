"""Script Editor-safe entrypoint for the M0609 ROS 2 Action Graph builder."""

import os
import importlib
import sys
from pathlib import Path

from omni.isaac.core.utils.extensions import enable_extension

import omni.graph.core as og
import omni.usd
import usdrt.Sdf

from pxr import UsdGeom, UsdPhysics


def _add_cell_module_root():
    """Locate sibling graph builders for source and Script Editor execution."""
    candidates = []
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        candidates.append(
            Path(configured_root).expanduser()
            / "infra"
            / "isaac_sim"
            / "scripts"
            / "m0609_cell"
        )
    candidates.extend(
        [
            Path(__file__).resolve().parent,
            Path.cwd() / "infra" / "isaac_sim" / "scripts" / "m0609_cell",
        ]
    )
    for module_root in candidates:
        if (module_root / "graph_builder").is_dir():
            if str(module_root) not in sys.path:
                sys.path.insert(0, str(module_root))
            return
    raise RuntimeError(
        "Cannot locate M0609 Action Graph contracts. Set SFTWIN_PROJECT_ROOT "
        "to the repository root when running from Isaac Script Editor."
    )


_add_cell_module_root()

import shared.script_editor_bootstrap as script_editor_bootstrap

importlib.reload(script_editor_bootstrap)
script_editor_bootstrap.clear_imported_package("graph_builder")

from graph_builder.ros_action_graph import build_ros_action_graph

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.core.nodes")

build_ros_action_graph(
    stage=omni.usd.get_context().get_stage(),
    controller=og.Controller,
    sdf_path=usdrt.Sdf.Path,
    usd_geom=UsdGeom,
    usd_physics=UsdPhysics,
)
