#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace arm_cell_motion
{
// Request-local payload: the inverse accepted object->TCP relation locates the
// object's center in TCP coordinates. Only the physical grasp contacts may touch.
inline moveit_msgs::msg::AttachedCollisionObject make_attached_payload(
  const geometry_msgs::msg::Transform & object_to_tcp,
  const std::vector<double> & dimensions, const std::vector<std::string> & touch_links = {},
  const std::string & link = "sf_grasp_tcp")
{
  if (dimensions.size() != 3 || !std::all_of(
      dimensions.begin(), dimensions.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument("accepted payload requires three positive dimensions");
  }
  Eigen::Quaterniond rotation(object_to_tcp.rotation.w, object_to_tcp.rotation.x,
    object_to_tcp.rotation.y, object_to_tcp.rotation.z);
  Eigen::Vector3d translation(object_to_tcp.translation.x, object_to_tcp.translation.y,
    object_to_tcp.translation.z);
  if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-3 ||
    !translation.allFinite()) {throw std::invalid_argument("invalid payload grasp transform");}
  rotation.normalize();
  const auto inverse_rotation = rotation.conjugate();
  const Eigen::Vector3d position = -(inverse_rotation * translation);
  moveit_msgs::msg::AttachedCollisionObject payload;
  payload.link_name = link;
  payload.touch_links = {link};
  for (const auto & contact : touch_links) {
    if (contact != link) {payload.touch_links.push_back(contact);}
  }
  payload.object.id = "accepted_pick_payload";
  payload.object.header.frame_id = link;
  payload.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.assign(dimensions.begin(), dimensions.end());
  payload.object.primitives = {box};
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x(); pose.position.y = position.y(); pose.position.z = position.z();
  pose.orientation.x = inverse_rotation.x(); pose.orientation.y = inverse_rotation.y();
  pose.orientation.z = inverse_rotation.z(); pose.orientation.w = inverse_rotation.w();
  payload.object.primitive_poses = {pose};
  return payload;
}
}  // namespace arm_cell_motion
