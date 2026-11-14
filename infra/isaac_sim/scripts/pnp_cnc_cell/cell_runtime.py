"""Isaac PnP_CNC Cell device realization; Timeline ownership stays in the backend."""
import os
from pathlib import Path
import sys
cell_root = str(Path(__file__).resolve().parent)
if cell_root not in sys.path:
    sys.path.insert(0, cell_root)
from scene_builder.material_supply import MaterialSupplyProfile
from scene_builder.material_supply_adapter import IsaacMaterialSupplyAdapter
from scene_builder.static_environment import supply_support_origin
from scene_builder.material_supply_sidecar import MaterialSupplySidecarClient
try:
    from isaacsim.core.api import World
except ImportError:
    from omni.isaac.core import World

def start_material_supply(*, configuration, context):
    """Connect the Isaac supply realization to Integration's accepted event."""
    from isaac_runtime.run_records import diagnostic_component_directory
    project_root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
    ros_install = project_root / "ros2_ws/install/setup.bash"
    if not ros_install.is_file():
        print(
            "ROS workspace is not built locally; waiting for the Docker "
            "material supply sidecar."
        )
    operational_profile = configuration
    supply_config = operational_profile['cell']['material_supply']['profile']
    material_handoff = operational_profile.get("material_handoff", {})
    profile = MaterialSupplyProfile.from_mapping(
        supply_config,
        operational_profile.get("vision", {}),
        operational_profile.get("material_definitions", {}),
    )
    from copy import deepcopy
    from dataclasses import replace
    material_profiles = {}
    for identity, material in operational_profile["material_catalog"].items():
        selected_supply = deepcopy(supply_config)
        selected_supply["material_supply"]["material_type"] = identity
        selected_supply["material_supply"]["mass_kg"] = material.get("mass_kg", profile.mass_kg)
        selected_profile = MaterialSupplyProfile.from_mapping(
            selected_supply, operational_profile["vision"],
            operational_profile["material_definitions"],
        )
        material_profiles[identity] = replace(selected_profile, display_color_rgb=tuple(material["display_color_rgb"]))
    sidecar = MaterialSupplySidecarClient.launch(project_root)
    try:
        import omni.timeline

        adapter = IsaacMaterialSupplyAdapter(
            stage=World.instance().stage,
            sidecar=sidecar,
            profile=profile,
            material_profiles=material_profiles,
            delivery_deadline_ms=material_handoff.get("delivery_deadline_ms"),
            delivery_root=operational_profile['cell']['material_supply']['delivery_root'],
            shelf_origin_m=supply_support_origin(context.static_geometry, operational_profile['cell']['material_supply']),
            artifact_directory=(
                diagnostic_component_directory(project_root, "simulation")
            ),
            timeline=omni.timeline.get_timeline_interface(),
        )
    except Exception:
        sidecar.close()
        raise
    return adapter
