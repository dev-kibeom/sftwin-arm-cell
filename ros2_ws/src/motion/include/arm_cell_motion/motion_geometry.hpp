#pragma once

#include <string>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace arm_cell_motion
{

struct MotionGeometry
{
  std::string observation_reference;
  geometry_msgs::msg::Transform observation_to_object = geometry_msgs::msg::Transform();
  geometry_msgs::msg::Transform object_to_grasp_tcp = geometry_msgs::msg::Transform();
  geometry_msgs::msg::Vector3 insertion_axis_tcp = geometry_msgs::msg::Vector3();
  double pick_retract_distance_m{0.0};
  double place_waypoint_clearance_y_boundary_m{0.0};
  double place_waypoint_clearance_y_margin_m{0.05};
  double place_waypoint_approach_distance_m{0.0};
  geometry_msgs::msg::Vector3 place_waypoint_approach_direction_object;
  bool place_waypoint_clearance_toward_positive_y{false};
  geometry_msgs::msg::PoseStamped place_waypoint_target_pose;
  bool place_waypoint_configured{false};
  bool configured{false};

  bool valid() const;
};

}  // namespace arm_cell_motion
