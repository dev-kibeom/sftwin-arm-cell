import importlib
import inspect
import logging
import os
import sys
from pathlib import Path

import omni.physx
from omni.physx.bindings._physx import SimulationEvent

LOGGER = logging.getLogger(__name__)

configured_root = os.environ.get("SFTWIN_PROJECT_ROOT")
if not configured_root:
    raise RuntimeError(
        "SFTWIN_PROJECT_ROOT is not configured. Run 1_before_play.py before "
        "bootstrapping the gripper runtime."
    )
project_root = Path(configured_root).expanduser().resolve()
cell_root = project_root / "infra/isaac_sim/scripts/m0609_cell"
shared_bytecode_cache = cell_root / "shared" / "__pycache__"
if shared_bytecode_cache.is_dir():
    for bytecode_path in shared_bytecode_cache.glob("*.pyc"):
        bytecode_path.unlink(missing_ok=True)
sys.modules.pop("shared.script_editor_bootstrap", None)
importlib.invalidate_caches()

from shared.script_editor_bootstrap import (
    clear_package_bytecode,
    module_root_from_project_root,
)

SCRIPT_DIR = str(module_root_from_project_root(project_root))

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Isaac Script Editor keeps Python modules alive across executions.  Ensure a
# rerun cannot reuse gripper/validation classes from an older checkout or
# earlier script snapshot.
else:
    sys.path.remove(SCRIPT_DIR)
    sys.path.insert(0, SCRIPT_DIR)

previous_session = globals().get("runtime_session")
previous_status_sequence = int(getattr(previous_session, "_status_sequence", 0) or 0)
if previous_session is not None:
    previous_grasp_manager = getattr(previous_session, "grasp_manager", None)
    previous_observation = getattr(previous_grasp_manager, "holding_observation", None)
    if callable(previous_observation):
        try:
            previous_status_sequence = max(
                previous_status_sequence,
                int(getattr(previous_observation(), "sequence", 0) or 0),
            )
        except Exception:
            pass

old_owner = globals().get("pnp_validation_session_owner")
if old_owner is not None:
    old_owner.prepare_for_rerun()
elif previous_session is not None:
    previous_session.shutdown()

for module_name in list(sys.modules):
    if module_name == "gripper_runtime" or module_name.startswith("gripper_runtime."):
        del sys.modules[module_name]
    elif module_name == "pnp_validation" or module_name.startswith("pnp_validation."):
        del sys.modules[module_name]
    elif module_name == "graph_builder" or module_name.startswith("graph_builder."):
        del sys.modules[module_name]

removed_gripper_bytecode = clear_package_bytecode(Path(SCRIPT_DIR) / "gripper_runtime")
importlib.invalidate_caches()

import gripper_runtime.action_graph_command_adapter as adapter_module
import gripper_runtime.grasp_attachment as grasp_module
import gripper_runtime.grasp_policy as grasp_policy_module
import gripper_runtime.robotiq_actuator as gripper_module
import gripper_runtime.session as session_module
from graph_builder.graph_contract import GRAPH_PATH


capture_config = None
capture_registry = None
validation_profile = (
    os.environ.get("SFTWIN_PNP_PROFILE")
    if os.environ.get("SFTWIN_RUNTIME_MODE") == "diagnostic"
    else None
)
if validation_profile:
    from pnp_validation.run_validation import (
        capture_config_from_profile,
        capture_reference_config_from_profile,
        load_validation_profile,
    )

    loaded_profile = load_validation_profile(validation_profile)
    capture_config = capture_config_from_profile(loaded_profile)
    capture_reference_config = capture_reference_config_from_profile(loaded_profile)
else:
    import yaml

    from gripper_runtime.operational_config import (
        load_production_mission_recipe,
        production_capture_configuration,
    )

    operational_profile_path = (
        project_root
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/operational_profile.yaml"
    )
    with operational_profile_path.open("r", encoding="utf-8") as profile_file:
        operational_profile = yaml.safe_load(profile_file)
    mission_recipe = load_production_mission_recipe(
        operational_profile, operational_profile_path
    )
    (
        capture_config,
        capture_reference_config,
        capture_registry,
    ) = production_capture_configuration(operational_profile, mission_recipe)

importlib.reload(gripper_module)
importlib.reload(adapter_module)
importlib.reload(grasp_policy_module)
importlib.reload(grasp_module)
importlib.reload(session_module)

ACTION_GRAPH_PATH = GRAPH_PATH

runtime_session = session_module.create_gripper_runtime_session(
    omni.physx.get_physx_interface(),
    capture_config=capture_config,
    capture_reference_config=capture_reference_config,
    registry=capture_registry,
    initial_status_sequence=session_module.initial_status_sequence(
        previous_status_sequence
    ),
)
from pnp_validation.editor_lifecycle import ValidationSessionOwner

_validation_session_owner = ValidationSessionOwner(
    omni.physx.get_physx_interface().get_simulation_event_stream_v2(),
    SimulationEvent.STOPPED,
)
_validation_session_owner.bind_namespace(globals())
_validation_session_owner.register_session(runtime_session)
_validation_session_owner.start()
globals()["pnp_validation_session_owner"] = _validation_session_owner
LOGGER.debug(
    "gripper runtime modules loaded root=%s session_module=%s actuator_module=%s "
    "factory_signature=%s init_source=%s:%d holding_sequence=%s "
    "gripper_type=%s.%s feedback_generation_present=%s removed_bytecode=%s",
    SCRIPT_DIR,
    session_module.__file__,
    gripper_module.__file__,
    inspect.signature(session_module.create_gripper_runtime_session),
    session_module.GripperRuntimeSession.__init__.__code__.co_filename,
    session_module.GripperRuntimeSession.__init__.__code__.co_firstlineno,
    runtime_session._status_sequence,
    type(runtime_session.gripper).__module__,
    type(runtime_session.gripper).__name__,
    hasattr(runtime_session.gripper, "feedback_generation"),
    removed_gripper_bytecode,
)
if not hasattr(runtime_session.gripper, "feedback_generation"):
    runtime_session._feedback_generation = 0
    runtime_session.gripper.feedback_generation = 0
    LOGGER.debug("gripper feedback generation baseline repaired at bootstrap")
if getattr(runtime_session.gripper, "feedback_generation", None) is None:
    raise RuntimeError(
        "gripper runtime did not expose feedback generation; "
        "rerun 2_after_play.py to rebuild the gripper runtime, or restart Isaac Kit"
    )

# A Script Editor session may retain an older bound physics callback even after
# the source modules are reloaded.  Preserve the callback's behavior, but
# guarantee the validation feedback-generation contract at this live boundary.
_runtime_on_physics_step = runtime_session.on_physics_step


def _validation_on_physics_step(dt):
    before = runtime_session.gripper.feedback_generation
    _runtime_on_physics_step(dt)
    after = runtime_session.gripper.feedback_generation
    if not isinstance(after, int) or after <= before:
        runtime_session._feedback_generation = int(before) + 1
        runtime_session.gripper.feedback_generation = (
            runtime_session._feedback_generation
        )


runtime_session._unsubscribe()
runtime_session.on_physics_step = _validation_on_physics_step
runtime_session.subscribe()
GRASP_CONFIG = runtime_session.grasp_config
grasp_manager = runtime_session.grasp_manager
gripper = runtime_session.gripper
sim_adapter = runtime_session.sim_adapter
on_physics_step = runtime_session.on_physics_step
physics_sub = runtime_session.physics_sub

# Establish the physical open posture before the runtime begins processing
# newly received graph commands. The retained graph generation is primed when
# the session creates its adapter, so old data is not replayed as a command.
gripper.open()


LOGGER.debug(
    "gripper validation bootstrap complete feedback_generation=%s "
    "action_graph=%s command_topic=/gripper/command command_type=std_msgs/msg/Float64",
    runtime_session.gripper.feedback_generation,
    ACTION_GRAPH_PATH,
)
