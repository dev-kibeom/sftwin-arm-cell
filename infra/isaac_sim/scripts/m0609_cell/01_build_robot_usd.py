"""
SF-Twin Robot Asset Builder

Pipeline:
    Xacro
      -> URDF
      -> Isaac Sim URDF Importer (mimic disabled)
      -> USD validation

Target:
    Doosan M0609 + Robotiq 2F-85
    Isaac Sim 5.1

Design note:
    The source URDF keeps Robotiq mimic joints for ROS / MoveIt.
    Isaac Sim intentionally ignores mimic constraints. SF-Twin will control
    the gripper as a logical 1-DOF tool in a separate controller layer.
"""

import os
import shutil
import subprocess
from pathlib import Path

import omni.kit.commands

from omni.isaac.core.utils.extensions import enable_extension
from pxr import Usd, UsdPhysics

from model_provenance import build_manifest, write_manifest
from usd_visual_cleanup import cleanup_generated_base_layer

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

PROJECT_ROOT = Path(os.path.expanduser("~/sftwin_project"))
ASSET_ROOT = PROJECT_ROOT / "infra" / "isaac_sim" / "assets"

XACRO_PATH = ASSET_ROOT / "urdf" / "assemblies" / "m0609_robotiq_2f85.xacro"
GENERATED_URDF = ASSET_ROOT / "urdf" / "generated" / "m0609_robotiq_2f85.urdf"
OUTPUT_USD = ASSET_ROOT / "usd" / "m0609_robotiq_2f85_generated.usd"
PROVENANCE = OUTPUT_USD.with_suffix(".provenance.json")
GENERATED_BASE_LAYER = (
    OUTPUT_USD.parent / "configuration" / f"{OUTPUT_USD.stem}_base.usd"
)

ROBOTIQ_JOINTS = [
    "gripper_robotiq_85_left_knuckle_joint",
    "gripper_robotiq_85_right_knuckle_joint",
    "gripper_robotiq_85_left_inner_knuckle_joint",
    "gripper_robotiq_85_right_inner_knuckle_joint",
    "gripper_robotiq_85_left_finger_tip_joint",
    "gripper_robotiq_85_right_finger_tip_joint",
]


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------


def check_file(path: Path, label: str):
    if not path.exists():
        raise FileNotFoundError(f"[ERROR] {label} not found: {path}")


def validate_urdf():
    print("\n" + "=" * 70)
    print(">>> Validate URDF")

    check_file(GENERATED_URDF, "Generated URDF")
    executable = shutil.which("check_urdf")

    if executable is None:
        print(">>> [WARNING] check_urdf not found. Validation skipped.")
        return

    result = subprocess.run(
        [executable, str(GENERATED_URDF)],
        capture_output=True,
        text=True,
    )

    if result.stdout:
        print(result.stdout)

    if result.returncode != 0:
        if result.stderr:
            print(result.stderr)
        raise RuntimeError("[ERROR] URDF validation failed.")

    print(">>> [SUCCESS] URDF validation passed.")


def find_xacro():
    """Prefer the xacro executable available in the ROS environment."""
    executable = shutil.which("xacro")

    if executable is None:
        raise RuntimeError(
            "[ERROR] xacro executable not found.\n"
            "Source your ROS environment before launching Isaac Sim.\n"
            "Example:\n"
            "  source /opt/ros/humble/setup.bash"
        )

    return executable


def find_joint_prim(stage: Usd.Stage, joint_name: str):
    """Find exactly one joint by prim name without assuming a root prim path."""
    matches = [prim for prim in stage.Traverse() if prim.GetName() == joint_name]

    if not matches:
        raise RuntimeError(f"[USD Validate] Joint not found: {joint_name}")

    if len(matches) > 1:
        paths = ", ".join(str(prim.GetPath()) for prim in matches)
        raise RuntimeError(
            f"[USD Validate] Multiple joints named '{joint_name}' found: {paths}"
        )

    return matches[0]


# ---------------------------------------------------------
# 1. Xacro -> URDF
# ---------------------------------------------------------


def build_urdf_from_xacro():
    print("\n" + "=" * 70)
    print(">>> [1/3] Xacro -> URDF")

    check_file(XACRO_PATH, "Assembly Xacro")
    GENERATED_URDF.parent.mkdir(parents=True, exist_ok=True)

    xacro_exe = find_xacro()
    command = [
        xacro_exe,
        str(XACRO_PATH),
        "-o",
        str(GENERATED_URDF),
    ]

    print(">>> Command:", " ".join(command))

    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
    )

    if result.stdout:
        print(result.stdout)

    if result.returncode != 0:
        print(result.stderr)
        raise RuntimeError("[ERROR] Xacro generation failed.")

    check_file(GENERATED_URDF, "Generated URDF")
    print(f">>> [SUCCESS] URDF generated:\n    {GENERATED_URDF}")


# ---------------------------------------------------------
# 2. URDF -> USD
# ---------------------------------------------------------


def build_usd_from_urdf():
    print("\n" + "=" * 70)
    print(">>> [2/3] URDF -> USD")

    enable_extension("isaacsim.asset.importer.urdf")
    check_file(GENERATED_URDF, "Generated URDF")
    OUTPUT_USD.parent.mkdir(parents=True, exist_ok=True)

    # Isaac Sim 5.1 import namespace compatibility
    try:
        import _urdf

        target_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    except Exception:
        from isaacsim.asset.importer.urdf import _urdf

        target_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

    if not status:
        raise RuntimeError("[ERROR] URDFCreateImportConfig failed.")

    # Preserve semantic/tool frames such as tool0, tool_mount and sf_grasp_tcp.
    import_config.merge_fixed_joints = False

    # M0609 is mounted on a fixed pedestal.
    import_config.fix_base = True
    import_config.make_default_prim = True

    # Internal robot/gripper self-collision is not required for SF-Twin PnP.
    import_config.self_collision = False
    import_config.create_physics_scene = False

    import_config.default_drive_type = target_drive_type
    import_config.distance_scale = 1.0

    # IMPORTANT:
    # Keep <mimic> in the source URDF for ROS/MoveIt, but do not generate
    # PhysX Mimic API constraints in Isaac. Gripper motion will be handled by
    # the SF-Twin logical 1-DOF controller instead.
    import_config.parse_mimic = False

    print(f">>> Input URDF:\n    {GENERATED_URDF}")
    print(f">>> Output USD:\n    {OUTPUT_USD}")
    print(">>> Robotiq PhysX mimic import: DISABLED")

    status, result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(GENERATED_URDF),
        import_config=import_config,
        dest_path=str(OUTPUT_USD),
    )

    if not status:
        raise RuntimeError("[ERROR] URDF import failed.")

    check_file(OUTPUT_USD, "Generated USD")
    cleaned = cleanup_generated_base_layer(GENERATED_URDF, GENERATED_BASE_LAYER)
    if cleaned:
        print(">>> [SUCCESS] Removed dangling visual references: " + ", ".join(cleaned))
    write_manifest(
        PROVENANCE, build_manifest(GENERATED_URDF, GENERATED_URDF, OUTPUT_USD)
    )
    print(f">>> [SUCCESS] Robot USD generated:\n    {OUTPUT_USD}")
    print(f">>> [SUCCESS] Provenance  : {PROVENANCE}")


# ---------------------------------------------------------
# 3. Validate Isaac-specific gripper import
# ---------------------------------------------------------


def validate_isaac_gripper_import():
    """
    Verify the assumptions required by the simplified SF-Twin gripper model:
      1) all six Robotiq revolute joints still exist;
      2) no PhysX mimic attributes were authored on those joints;
      3) each joint remains a revolute joint for the next controller step.

    This function intentionally does not alter the generated USD.
    """
    print("\n" + "=" * 70)
    print(">>> [3/3] Validate simplified Robotiq import")

    check_file(OUTPUT_USD, "Generated USD")
    stage = Usd.Stage.Open(str(OUTPUT_USD))

    if stage is None:
        raise RuntimeError(f"[ERROR] Failed to open USD: {OUTPUT_USD}")

    mimic_attrs_found = []

    for joint_name in ROBOTIQ_JOINTS:
        prim = find_joint_prim(stage, joint_name)
        joint = UsdPhysics.RevoluteJoint(prim)

        if not joint:
            raise RuntimeError(f"[USD Validate] Not a revolute joint: {prim.GetPath()}")

        mimic_attrs = [
            attr.GetName()
            for attr in prim.GetAttributes()
            if "physxMimicJoint" in attr.GetName()
        ]

        if mimic_attrs:
            mimic_attrs_found.append((prim.GetPath(), mimic_attrs))

        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        drive_state = "present" if drive else "none"

        print(
            f"    {joint_name}\n"
            f"      path  = {prim.GetPath()}\n"
            f"      mimic = {'present' if mimic_attrs else 'none'}\n"
            f"      drive = {drive_state}"
        )

    if mimic_attrs_found:
        details = "\n".join(f"  {path}: {attrs}" for path, attrs in mimic_attrs_found)
        raise RuntimeError(
            "[USD Validate] PhysX mimic attributes still exist although "
            "parse_mimic=False:\n" + details
        )

    print(
        ">>> [SUCCESS] Simplified Robotiq import validated: "
        "6 revolute joints present, PhysX mimic absent."
    )


# ---------------------------------------------------------
# Main
# ---------------------------------------------------------

try:
    print("\n" + "=" * 70)
    print(">>> SF-Twin Robot Builder : M0609 + Robotiq 2F-85")

    # Keep the current workflow: URDF is generated/validated outside Isaac Sim.
    check_file(GENERATED_URDF, "Generated URDF")

    build_usd_from_urdf()
    validate_isaac_gripper_import()

    print("\n" + "=" * 70)
    print(">>> [SUCCESS] Complete robot asset build finished.")
    print("=" * 70)

except Exception as exc:
    print("\n" + "=" * 70)
    print(f">>> [FAILED] {exc}")
    print("=" * 70)
    raise
