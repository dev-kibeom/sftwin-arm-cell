#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <arm_cell_interfaces/msg/collision_primitive.hpp>
#include <arm_cell_interfaces/msg/pick_candidate.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include "arm_cell_motion/occupancy_collision_object.hpp"

namespace arm_cell_motion
{

inline std::vector<arm_cell_interfaces::msg::CollisionPrimitive>
candidate_occupancy_from_candidates(
  const std::vector<arm_cell_interfaces::msg::PickCandidate> & candidates,
  const std::string & acquisition_id, const std::string & selected_candidate_id)
{
  using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
  std::vector<Primitive> occupancy;
  for (const auto & candidate : candidates) {
    if (candidate.acquisition_id != acquisition_id || candidate.candidate_id.empty() ||
      candidate.candidate_id == selected_candidate_id || !candidate.has_estimated_height ||
      candidate.target_pose.header.frame_id != "base_link")
    {
      continue;
    }
    const auto length = static_cast<double>(candidate.rectangle_long_side_m);
    const auto width = static_cast<double>(candidate.rectangle_short_side_m);
    const auto height = static_cast<double>(candidate.estimated_height_m);
    const auto & pose = candidate.target_pose.pose;
    const auto & orientation = pose.orientation;
    Primitive box;
    box.occupancy_class = Primitive::OBSERVED_OCCUPANCY;
    box.shape_type = Primitive::SOLID_PRIMITIVE;
    box.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.shape.dimensions = {length, width, height};
    box.pose = pose;
    const double half_height = height * 0.5;
    box.pose.position.x -= 2.0 *
      (orientation.x * orientation.z + orientation.w * orientation.y) * half_height;
    box.pose.position.y -= 2.0 *
      (orientation.y * orientation.z - orientation.w * orientation.x) * half_height;
    box.pose.position.z -= (1.0 - 2.0 *
      (orientation.x * orientation.x + orientation.y * orientation.y)) * half_height;
    occupancy.push_back(std::move(box));
  }
  return occupancy;
}

enum class VisionOccupancyPolicy
{
  EXCLUDE,
  INCLUDE
};

inline std::vector<std::string> candidate_occupancy_ids(
  const moveit_msgs::msg::PlanningScene & scene)
{
  std::vector<std::string> ids;
  for (const auto & object : scene.world.collision_objects) {
    if (object.id.compare(
        0, sizeof(kCandidateOccupancyIdPrefix) - 1,
        kCandidateOccupancyIdPrefix) == 0)
    {
      ids.push_back(object.id);
    }
  }
  return ids;
}

inline bool valid_candidate_occupancy(
  bool available,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & primitives,
  const geometry_msgs::msg::PoseStamped & observation)
{
  if (!available && !primitives.empty()) {return false;}
  if (available && (observation.header.frame_id != "base_link" ||
    observation.header.stamp.sec < 0 || observation.header.stamp.nanosec >= 1000000000))
  {
    return false;
  }
  for (const auto & p : primitives) {
    using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
    if (p.occupancy_class != Primitive::OBSERVED_OCCUPANCY &&
      p.occupancy_class != Primitive::UNKNOWN_OCCUPANCY) {return false;}
    if (p.shape_type == Primitive::SOLID_PRIMITIVE) {
      const auto type = p.shape.type;
      const size_t count = type == shape_msgs::msg::SolidPrimitive::BOX ? 3 :
        type == shape_msgs::msg::SolidPrimitive::CYLINDER ? 2 : 0;
      if (count == 0 || p.shape.dimensions.size() != count) {return false;}
      for (double d : p.shape.dimensions) {
        if (!std::isfinite(d) || d <= 0) {return false;}
      }
    } else if (p.shape_type == Primitive::CONVEX_TOP) {
      const auto & v = p.top_vertices;
      if (v.size() < 3 || !std::isfinite(p.bottom_height_m)) {return false;}
      double sign = 0, area = 0, max_height = 0;
      for (size_t i = 0; i < v.size(); ++i) {
        const auto & a = v[i]; const auto & b = v[(i + 1) % v.size()];
        const auto & c = v[(i + 2) % v.size()];
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
          a.z < p.bottom_height_m) {return false;}
        max_height = std::max(max_height, a.z - p.bottom_height_m);
        if (std::hypot(b.x - a.x, b.y - a.y) <= 1e-10) {return false;}
        const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (std::abs(cross) > 1e-12) {
          if (sign * cross < 0) {return false;}
          sign = cross;
        }
        area += a.x * b.y - a.y * b.x;
      }
      if (std::abs(area) <= 1e-12 || max_height <= 1e-10) {return false;}
      for (size_t i = 0; i < v.size(); ++i) {
        const auto & a = v[i]; const auto & b = v[(i + 1) % v.size()];
        for (size_t j = 0; j < v.size(); ++j) {
          if (j != i && std::hypot(v[j].x - a.x, v[j].y - a.y) <= 1e-10) {return false;}
          const double side = (b.x - a.x) * (v[j].y - a.y) -
            (b.y - a.y) * (v[j].x - a.x);
          if (side * (area > 0 ? 1 : -1) < -1e-12) {return false;}
        }
      }
      // All vertices must lie in the same top plane.
      const auto & a = v[0];
      size_t j = 1, k = 2;
      while (k < v.size() && std::abs(
          (v[j].x - a.x) * (v[k].y - a.y) -
          (v[j].y - a.y) * (v[k].x - a.x)) <= 1e-12) {++j; ++k;}
      if (k == v.size()) {return false;}
      const double nx = (v[j].y - a.y) * (v[k].z - a.z) - (v[j].z - a.z) * (v[k].y - a.y);
      const double ny = (v[j].z - a.z) * (v[k].x - a.x) - (v[j].x - a.x) * (v[k].z - a.z);
      const double nz = (v[j].x - a.x) * (v[k].y - a.y) - (v[j].y - a.y) * (v[k].x - a.x);
      for (const auto & t : v) {
        if (std::abs(nx * (t.x - a.x) + ny * (t.y - a.y) + nz * (t.z - a.z)) >
          1e-8 * std::sqrt(nx * nx + ny * ny + nz * nz)) {return false;}
      }
    } else {return false;}
    const auto & q = p.pose.orientation;
    const auto & t = p.pose.position;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-6 ||
      !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) {return false;}
  }
  return true;
}

inline moveit_msgs::msg::PlanningScene candidate_scene_diff(
  const std::vector<std::string> & previous,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & primitives,
  const std_msgs::msg::Header & header, std::vector<std::string> & next_ids)
{
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  for (const auto & id : previous) {
    moveit_msgs::msg::CollisionObject object;
    object.id = id; object.header = header;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    diff.world.collision_objects.push_back(object);
  }
  next_ids.clear();
  for (size_t i = 0; i < primitives.size(); ++i) {
    auto object = candidate_occupancy_object(primitives[i], header, i);
    next_ids.push_back(object.id);
    diff.world.collision_objects.push_back(std::move(object));
  }
  return diff;
}

// Compose request-local occupancy changes. EXCLUDE removes only candidate
// occupancy objects from the request; fixed geometry, attachments and the
// shared Planning Scene remain untouched.
inline moveit_msgs::msg::PlanningScene candidate_scene_diff(
  const std::vector<std::string> & previous,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & primitives,
  const std_msgs::msg::Header & header, VisionOccupancyPolicy policy,
  std::vector<std::string> & next_ids)
{
  return candidate_scene_diff(
    previous,
    policy == VisionOccupancyPolicy::INCLUDE ? primitives :
    std::vector<arm_cell_interfaces::msg::CollisionPrimitive>{},
    header, next_ids);
}
}  // namespace arm_cell_motion
