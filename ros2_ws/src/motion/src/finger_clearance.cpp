#include "arm_cell_motion/finger_clearance.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace arm_cell_motion
{
void validate_finger_clearance_parameters(const FingerClearanceParameters & parameters)
{
  for (double value : {parameters.radius_m, parameters.contact_span_m, parameters.depth_m}) {
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
      throw std::invalid_argument("finger clearance dimensions must be finite metres in (0, 1]");
    }
  }
}

std::array<FingerProtectionVolume, 2> make_finger_protection_volumes(
  const Eigen::Isometry3d & tcp, const Eigen::Vector3d & object_center,
  const Eigen::Quaterniond & object_orientation, const BoxGraspContact & contact,
  const Eigen::Vector3d & insertion_axis_tcp, const FingerClearanceParameters & parameters)
{
  validate_finger_clearance_parameters(parameters);
  if (contact.closing_axis > 2U || !std::isfinite(contact.width_mm) ||
    contact.width_mm <= 0.0 || !tcp.matrix().allFinite() || !object_center.allFinite() ||
    !object_orientation.coeffs().allFinite() || object_orientation.norm() < 1e-9 ||
    !insertion_axis_tcp.allFinite() || insertion_axis_tcp.norm() < 1e-9)
  {
    throw std::invalid_argument("finger clearance contact geometry unavailable");
  }
  const Eigen::Vector3d object_axis = object_orientation.normalized() *
    Eigen::Vector3d::Unit(contact.closing_axis);
  // Finger indices follow TCP -X/+X even when the candidate closes along
  // the negative direction of the same object axis.
  const Eigen::Vector3d axis = object_axis.dot(tcp.linear().col(0)) < 0.0 ?
    -object_axis : object_axis;
  const Eigen::Vector3d insertion = tcp.linear() * insertion_axis_tcp.normalized();
  // Contact validation already establishes closing-axis alignment. Project out
  // only its numerical residual, preserving an orthonormal extrusion frame.
  const Eigen::Vector3d tangent_insertion = insertion - axis * axis.dot(insertion);
  if (tangent_insertion.norm() < 1e-6) {
    throw std::invalid_argument("finger insertion axis parallel to closing axis");
  }
  const Eigen::Vector3d z = tangent_insertion.normalized();
  const Eigen::Vector3d center = tcp.translation() -
    axis * axis.dot(tcp.translation() - object_center);
  std::array<FingerProtectionVolume, 2> volumes;
  for (std::size_t side = 0; side < volumes.size(); ++side) {
    const Eigen::Vector3d outward = (side == 0U ? -1.0 : 1.0) * axis;
    auto & volume = volumes[side];
    volume.dimensions = parameters;
    volume.frame.translation() = center + outward * contact.width_mm * 0.0005;
    volume.frame.linear().col(0) = outward;
    volume.frame.linear().col(1) = z.cross(outward);
    volume.frame.linear().col(2) = z;
  }
  return volumes;
}

}  // namespace arm_cell_motion
