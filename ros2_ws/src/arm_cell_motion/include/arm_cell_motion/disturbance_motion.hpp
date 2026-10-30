#pragma once

#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace arm_cell_motion
{

enum class DisturbanceDirection
{
  POSITIVE_X,
  NEGATIVE_X,
  POSITIVE_Y,
  NEGATIVE_Y
};

inline constexpr std::array<DisturbanceDirection, 4> disturbance_directions()
{
  return {DisturbanceDirection::POSITIVE_X, DisturbanceDirection::NEGATIVE_X,
    DisturbanceDirection::POSITIVE_Y, DisturbanceDirection::NEGATIVE_Y};
}

struct DisturbanceRoute
{
  geometry_msgs::msg::PoseStamped approach;
  geometry_msgs::msg::PoseStamped push_end;
  geometry_msgs::msg::PoseStamped retract_end;
};

inline DisturbanceRoute make_disturbance_route(
  const geometry_msgs::msg::PoseStamped & representative,
  DisturbanceDirection direction, double push_distance_m, double approach_clearance_m)
{
  double dx = 0.0;
  double dy = 0.0;
  switch (direction) {
    case DisturbanceDirection::POSITIVE_X: dx = 1.0; break;
    case DisturbanceDirection::NEGATIVE_X: dx = -1.0; break;
    case DisturbanceDirection::POSITIVE_Y: dy = 1.0; break;
    case DisturbanceDirection::NEGATIVE_Y: dy = -1.0; break;
  }

  DisturbanceRoute route;
  route.approach = representative;
  route.push_end = representative;
  route.retract_end = representative;
  route.approach.pose.orientation.w = 1.0;
  route.push_end.pose.orientation.w = 1.0;
  route.retract_end.pose.orientation.w = 1.0;
  route.approach.pose.position.x -= dx * approach_clearance_m;
  route.approach.pose.position.y -= dy * approach_clearance_m;
  route.push_end.pose.position.x += dx * push_distance_m;
  route.push_end.pose.position.y += dy * push_distance_m;
  route.retract_end.pose.position.x += dx * (push_distance_m + approach_clearance_m);
  route.retract_end.pose.position.y += dy * (push_distance_m + approach_clearance_m);
  return route;
}

}  // namespace arm_cell_motion
