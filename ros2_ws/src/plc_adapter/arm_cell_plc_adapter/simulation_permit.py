"""Equipment permit producer for the launcher's simulated Conveyor."""
import time
from threading import Lock

from arm_cell_interfaces.msg import MotionCapability, SafetyState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool


class SimulationEquipmentPermit:
    # Same local receipt window as the configured Mission's Safety consumer.
    SAFETY_MAX_AGE = 0.5
    # Match the Conveyor equipment's local observation window.
    PLC_MAX_AGE = 0.9

    def __init__(self, node, topic):
        self.normal_safety_at = None
        self.last_exchange_at = None
        self.link_lock = Lock()
        qos = QoSProfile(depth=1, lifespan=Duration(seconds=self.SAFETY_MAX_AGE))
        self.publisher = node.create_publisher(Bool, topic, qos)
        self.group = MutuallyExclusiveCallbackGroup()
        node.create_subscription(SafetyState, '/safety/state', self.on_safety, 1,
                                 callback_group=self.group)
        node.create_timer(0.1, self.publish, callback_group=self.group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        node.get_logger().info(f'Simulated Conveyor equipment permit publisher active: {topic}')

    def on_safety(self, state):
        normal = (state.valid and state.required_inputs_fresh and state.motion_envelope_valid
                  and state.safety_state == SafetyState.SAFETY_STATE_SAFE
                  and state.motion_capability.value == MotionCapability.MOTION_NORMAL
                  and 0 < state.max_velocity_scale <= 1 and 0 < state.max_acceleration_scale <= 1)
        self.normal_safety_at = time.monotonic() if normal else None

    def observe_exchange(self, connected):
        with self.link_lock:
            self.last_exchange_at = time.monotonic() if connected else None

    def publish(self):
        with self.link_lock:
            now = time.monotonic()
            allowed = (self.last_exchange_at is not None
                       and now - self.last_exchange_at < self.PLC_MAX_AGE
                       and self.normal_safety_at is not None
                       and now - self.normal_safety_at <= self.SAFETY_MAX_AGE)
            self.publisher.publish(Bool(data=allowed))
