"""Equipment permit producer for the launcher's simulated Conveyor."""
import time

from arm_cell_interfaces.msg import MotionCapability, SafetyState
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool


class SimulationEquipmentPermit:
    # Same local receipt window as the configured Mission's Safety consumer.
    FRESHNESS_SECONDS = 0.5

    def __init__(self, node, topic):
        self.normal_safety_at = None
        qos = QoSProfile(depth=1, lifespan=Duration(seconds=self.FRESHNESS_SECONDS))
        self.publisher = node.create_publisher(Bool, topic, qos)
        node.create_subscription(SafetyState, '/safety/state', self.on_safety, 1)
        node.get_logger().info(f'Simulated Conveyor equipment permit publisher active: {topic}')

    def on_safety(self, state):
        normal = (state.valid and state.required_inputs_fresh and state.motion_envelope_valid
                  and state.safety_state == SafetyState.SAFETY_STATE_SAFE
                  and state.motion_capability.value == MotionCapability.MOTION_NORMAL
                  and 0 < state.max_velocity_scale <= 1 and 0 < state.max_acceleration_scale <= 1)
        self.normal_safety_at = time.monotonic() if normal else None

    def publish(self, plc_connected):
        allowed = (plc_connected and self.normal_safety_at is not None
                   and time.monotonic() - self.normal_safety_at <= self.FRESHNESS_SECONDS)
        self.publisher.publish(Bool(data=allowed))
