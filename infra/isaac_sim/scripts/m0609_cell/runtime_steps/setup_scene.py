"""Script Editor-safe entrypoint for the M0609 cell scene composition."""

import os
import sys
import math
from pathlib import Path

import yaml
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

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


ARM_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)


def _load_home_joint_positions():
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not configured_root:
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py from "
            "the intended checkout."
        )
    home_path = (
        Path(configured_root).expanduser().resolve()
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/home_joint_positions.yaml"
    )
    config = yaml.safe_load(home_path.read_text()) or {}
    positions = tuple(float(value) for value in config.get("home_joint_positions", ()))
    if len(positions) != len(ARM_JOINT_NAMES) or not all(
        math.isfinite(value) for value in positions
    ):
        raise RuntimeError(
            "canonical HOME pose must contain six finite joint positions"
        )
    return positions


def _load_operational_profile():
    configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
    if not configured_root:
        raise RuntimeError(
            "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py from "
            "the intended checkout."
        )
    profile_path = (
        Path(configured_root).expanduser().resolve()
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/operational_profile.yaml"
    )
    return yaml.safe_load(profile_path.read_text()) or {}


def _author_initial_home_targets(stage):
    """Set Play-time angular drive targets from the canonical HOME pose.

    USD angular drive target positions are authored in degrees.  The canonical
    repository HOME contract remains radians, so conversion is kept explicit
    at this Isaac scene boundary.
    """
    home_positions = _load_home_joint_positions()
    joints = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.IsValid() and prim.GetName() in ARM_JOINT_NAMES
    }
    missing = [name for name in ARM_JOINT_NAMES if name not in joints]
    if missing:
        raise RuntimeError(f"HOME joints not found in robot stage: {missing}")

    for name, position_rad in zip(ARM_JOINT_NAMES, home_positions):
        drive = UsdPhysics.DriveAPI.Get(joints[name], "angular")
        if not drive:
            raise RuntimeError(f"angular drive not found for HOME joint: {name}")
        target = drive.GetTargetPositionAttr()
        if not target.IsValid():
            raise RuntimeError(
                f"angular drive target unavailable for HOME joint: {name}"
            )
        target.Set(math.degrees(position_rad))
        joints[name].SetCustomDataByKey(
            "sf_twin:initial_home_position_rad", position_rad
        )
        print(
            f">>> [HOME] initial drive target {name}: "
            f"{position_rad:.9f} rad ({math.degrees(position_rad):.3f} deg)"
        )


def _apply_joint_2_stiffness_profile(stage):
    """Apply the external W06 production scale to the imported joint_2 drive."""
    simulation = _load_operational_profile().get("simulation", {})
    scale = float(simulation.get("joint_2_stiffness_scale", 1.0))
    if not math.isfinite(scale) or scale <= 0.0:
        raise RuntimeError(
            "simulation.joint_2_stiffness_scale must be a finite positive value"
        )

    joints = [
        prim
        for prim in stage.Traverse()
        if prim.IsValid() and prim.GetName() == "joint_2"
    ]
    if len(joints) != 1:
        raise RuntimeError(
            f"expected exactly one joint_2 in robot stage, found {len(joints)}"
        )
    drive = UsdPhysics.DriveAPI.Get(joints[0], "angular")
    if not drive:
        raise RuntimeError("angular drive not found for joint_2")
    stiffness = drive.GetStiffnessAttr()
    if not stiffness.IsValid():
        raise RuntimeError("angular drive stiffness unavailable for joint_2")
    base_stiffness = float(stiffness.Get())
    if not math.isfinite(base_stiffness) or base_stiffness <= 0.0:
        raise RuntimeError("joint_2 imported angular stiffness must be positive")
    stiffness.Set(base_stiffness * scale)
    print(
        f">>> [DRIVE] joint_2 stiffness scale={scale:.6f} "
        f"imported={base_stiffness:.6f} authored={base_stiffness * scale:.6f}; "
        "damping, max effort, velocity, gravity, and other joints unchanged"
    )


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

_author_initial_home_targets(World.instance().stage)
_apply_joint_2_stiffness_profile(World.instance().stage)
