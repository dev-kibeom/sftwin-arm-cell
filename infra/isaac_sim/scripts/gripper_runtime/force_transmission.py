"""Quasistatic parallel-linkage virtual work, not measured contact calibration.

The counter-rotating fingertip mimic keeps each pad parallel. Its centre's
primary-angle derivative equals that of the knuckle-to-tip lever. With symmetric
contacts each pad carries half the requested total opposing normal force. Both
pads' virtual work reaches the single driven joint through the mimic coupling.
Gravity, inertia, solver impulses, and structural compliance are not estimated.
"""
import math


def validate_grip_force_n(value):
    value = float(value)
    # Manufacturer 20..235 N per finger, normalized to the sum of both fingers.
    if not math.isfinite(value) or not 40. <= value <= 470.:
        raise ValueError('grip_force_n must be finite inside Tool range [40, 470] N total')
    return value


class RobotiqForceTransmission:
    def __init__(self, model):
        self.x = float(model['finger_lever_x_m'])
        self.z = float(model['finger_lever_z_m'])
        if not all(math.isfinite(v) and v > 0 for v in (self.x, self.z)):
            raise ValueError('Invalid URDF Tool force transmission lever')

    def drive_torque_nm(self, total_force_n, primary_angle_rad):
        if not all(math.isfinite(v) for v in (total_force_n, primary_angle_rad)) or total_force_n <= 0:
            raise ValueError('Force and observed linkage angle must be finite; force positive')
        # w(q)=2*(base_x+x*cos(q)-z*sin(q))+constant pad offset.
        # tau=(F_total/2)*abs(dw/dq).
        return total_force_n * abs(self.x * math.sin(primary_angle_rad)
                                   + self.z * math.cos(primary_angle_rad))

    def maximum_drive_torque_nm(self, total_force_n, lower, upper):
        endpoints = [self.drive_torque_nm(total_force_n, q) for q in (lower, upper)]
        # All absolute sine extrema are pi apart.
        first = math.atan2(self.x, self.z)
        critical = first + math.ceil((lower - first) / math.pi) * math.pi
        if critical <= upper:
            endpoints.append(total_force_n * math.hypot(self.x, self.z))
        return max(endpoints)
