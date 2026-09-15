"""Play-time lifecycle for the scripted Robotiq actuator, adapter, and grasp attachment."""

from graph_builder.graph_contract import GRAPH_PATH
from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.grasp_attachment import GraspManager
from gripper_runtime.grasp_policy import GraspConfig
from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator


class GripperRuntimeSession:
    """Own runtime objects and the retained physics-step subscription."""

    def __init__(
        self,
        physics_interface,
        actuator_cls=Robotiq2F85Actuator,
        adapter_cls=SimGripperAdapter,
        grasp_manager_cls=GraspManager,
        config_cls=GraspConfig,
    ):
        self.grasp_config = config_cls(
            position_tolerance_m=0.015,
            orientation_tolerance_deg=12.0,
            contact_width_tolerance_mm=2.0,
        )
        self.gripper = actuator_cls(verbose=True)
        self.sim_adapter = adapter_cls(
            gripper=self.gripper,
            graph_path=GRAPH_PATH,
            move_duration=0.40,
            verbose=True,
        )
        self.grasp_manager = grasp_manager_cls(
            gripper=self.gripper,
            target_path=None,
            config=self.grasp_config,
            verbose=True,
        )
        self._physics_interface = physics_interface
        self.physics_sub = None

    def on_physics_step(self, dt):
        self.sim_adapter.update()
        self.gripper.update(dt)
        self.grasp_manager.update(dt)

    def subscribe(self):
        self.physics_sub = self._physics_interface.subscribe_physics_step_events(
            self.on_physics_step
        )
        return self.physics_sub


def create_gripper_runtime_session(physics_interface):
    """Construct and subscribe the stable Script Editor runtime session."""
    session = GripperRuntimeSession(physics_interface)
    session.subscribe()
    return session
