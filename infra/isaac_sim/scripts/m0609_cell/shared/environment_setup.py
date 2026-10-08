"""Environment configuration helpers used by M0609 Script Editor entrypoints."""

import os
from pathlib import Path
import sys


CELL_RELATIVE_PATH = Path("infra/isaac_sim/scripts/m0609_cell")
VALIDATION_PROFILE_RELATIVE_PATH = Path(
    "ros2_ws/src/arm_cell/arm_cell_bringup/config/pnp_validation_profile.yaml"
)
DEFAULT_ROS_DOMAIN_ID = "0"


def validate_project_root(project_root):
    """Validate an explicitly supplied SFTwin repository root."""
    root = Path(project_root).expanduser()
    if (
        not root.is_dir()
        or not (root / CELL_RELATIVE_PATH / "shared").is_dir()
        or not (root / "ros2_ws/src/arm_cell").is_dir()
    ):
        raise RuntimeError(f"project root is not a validated SFTwin repository: {root}")
    return root.resolve()


def resolve_project_root(project_root=None, environ=None):
    """Require an explicit root argument or an existing configured root."""
    if project_root:
        return validate_project_root(project_root), "explicit project root"
    environment = os.environ if environ is None else environ
    configured_root = environment.get("SFTWIN_PROJECT_ROOT")
    if configured_root:
        return validate_project_root(configured_root), "existing SFTWIN_PROJECT_ROOT"
    raise RuntimeError(
        "No project root was supplied. Pass an explicit PROJECT_ROOT or configure "
        "SFTWIN_PROJECT_ROOT before running an Isaac entrypoint."
    )


def resolve_ros_domain_id(environ=None):
    """Return the effective DDS domain, defaulting to the canonical domain 0."""
    environment = os.environ if environ is None else environ
    value = environment.get("ROS_DOMAIN_ID", DEFAULT_ROS_DOMAIN_ID).strip()
    if not value.isdigit() or not 0 <= int(value) <= 232:
        raise RuntimeError("ROS_DOMAIN_ID must be an integer in the range 0..232")
    effective = str(int(value))
    environment["ROS_DOMAIN_ID"] = effective
    return effective


def resolve_validation_profile(project_root, environ=None):
    environment = os.environ if environ is None else environ
    configured = environment.get("SFTWIN_PNP_PROFILE")
    profile = (
        Path(configured).expanduser()
        if configured
        else (project_root / VALIDATION_PROFILE_RELATIVE_PATH)
    )
    if not profile.is_absolute():
        profile = project_root / profile
    profile = profile.resolve()
    if not profile.is_file():
        raise RuntimeError(f"SFTWIN_PNP_PROFILE does not resolve to a file: {profile}")
    environment["SFTWIN_PNP_PROFILE"] = str(profile)
    return profile


def configure_environment(project_root=None, environ=None):
    """Configure the existing full diagnostic environment setup."""
    environment = os.environ if environ is None else environ
    result = configure_live_environment(project_root, environment)
    root = Path(result["SFTWIN_PROJECT_ROOT"])
    validation_profile = resolve_validation_profile(root, environment)
    return {**result, "SFTWIN_PNP_PROFILE": str(validation_profile)}


def configure_live_environment(project_root=None, environ=None):
    """Configure only the explicit project root and production live runtime."""
    environment = os.environ if environ is None else environ
    root, source = resolve_project_root(project_root, environment)
    ros_domain_id = resolve_ros_domain_id(environment)
    module_root = root / CELL_RELATIVE_PATH
    environment["SFTWIN_PROJECT_ROOT"] = str(root)
    environment["SFTWIN_RUNTIME_MODE"] = "production"
    if str(module_root) not in sys.path:
        sys.path.insert(0, str(module_root))
    return {
        "status": "VERIFIED",
        "SFTWIN_PROJECT_ROOT": str(root),
        "module_root": str(module_root),
        "resolution_source": source,
        "ROS_DOMAIN_ID": ros_domain_id,
    }
