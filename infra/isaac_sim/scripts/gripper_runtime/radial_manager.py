"""Correlate device possession estimates with commands, without identifying bodies."""
import math
from gripper_runtime.configured_manager import ConfiguredGraspManager, configured_reference
from gripper_runtime.grasp_strategy import select_grasp_strategy



class RadialCapture:
    name = 'internal_radial'

    def __init__(self, settings, geometry):
        self.settings, self.geometry = settings, geometry
        self.strategy = None

    @staticmethod
    def decode_reference(packet):
        try:
            reference = configured_reference(packet, 'internal_radial')
            if reference is None:
                return None
            for key in ('insertion_depth_m', 'thickness_m', 'inner_radius_m'):
                value = packet[key]
                if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0:
                    return None
            return {key: packet[key] for key in ('motion_goal_uuid', 'strategy', 'material_id',
                'supply_id', 'slot_id', 'supply_epoch', 'reservation_id',
                'insertion_depth_m', 'thickness_m', 'inner_radius_m')}
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def select(self, packet):
        self.strategy = select_grasp_strategy(self.settings, packet, geometry=self.geometry)
        return self.strategy


class RadialGraspManager(ConfiguredGraspManager):
    diagnostic_prefix = 'radial'

    def diagnostics(self):
        return dict(holding_state=self._holding.state.value, attach_joint_path=None,
                    holding_inference={'source':'primary_encoder_obstruction'},
                    accepted_relation=self.accepted_relation, actuator=self.gripper.observation())
