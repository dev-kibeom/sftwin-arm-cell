#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

namespace arm_cell_motion
{

// Validation fixtures already publish their actual BOX geometry to MoveIt.
// Use that geometry without turning it into a Vision Rectangle observation.
inline std::optional<double> fixture_contact_width_mm(
  const std::vector<moveit_msgs::msg::CollisionObject> & objects,
  const std::string & fixture_id, const geometry_msgs::msg::PoseStamped & selected_tcp,
  double policy_min_mm, double policy_max_mm, double gripper_max_mm)
{
  const moveit_msgs::msg::CollisionObject * fixture = nullptr;
  for (const auto & object : objects) {
    if (!fixture_id.empty() && object.id == fixture_id) {
      if (fixture) {return std::nullopt;}
      fixture = &object;
    }
  }
  if (!fixture || selected_tcp.header.frame_id.empty() ||
    fixture->header.frame_id != selected_tcp.header.frame_id ||
    fixture->primitives.size() != 1U || fixture->primitive_poses.size() != 1U ||
    !fixture->meshes.empty() || !fixture->planes.empty() ||
    fixture->primitives[0].type != shape_msgs::msg::SolidPrimitive::BOX ||
    fixture->primitives[0].dimensions.size() != 3U ||
    !std::isfinite(policy_min_mm) || !std::isfinite(policy_max_mm) ||
    !std::isfinite(gripper_max_mm) || policy_min_mm <= 0.0 ||
    policy_max_mm < policy_min_mm || gripper_max_mm <= 0.0)
  {
    return std::nullopt;
  }
  const auto valid_pose = [](const geometry_msgs::msg::Pose & pose) {
      const auto & q = pose.orientation;
      return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
             std::isfinite(pose.position.z) && std::isfinite(q.x) &&
             std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
             std::abs(Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm() - 1.0) <= 1e-6;
    };
  if (!valid_pose(fixture->pose) || !valid_pose(fixture->primitive_poses[0]) ||
    !valid_pose(selected_tcp.pose))
  {
    return std::nullopt;
  }
  const auto rotation = [](const geometry_msgs::msg::Pose & pose) {
      const auto & q = pose.orientation;
      return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
    };
  const Eigen::Vector3d closing =
    (rotation(fixture->pose) * rotation(fixture->primitive_poses[0])).conjugate() *
    (rotation(selected_tcp.pose) * Eigen::Vector3d::UnitX());
  const auto & dimensions = fixture->primitives[0].dimensions;
  if (std::any_of(dimensions.begin(), dimensions.end(), [](double length) {
      return !std::isfinite(length) || length <= 0.0;
    }))
  {
    return std::nullopt;
  }
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    // Same numerical axis residual as box_grasp_contact; no process tolerance.
    if (closing.cross(Eigen::Vector3d::Unit(axis)).norm() <= 1e-4) {
      const double width = dimensions[axis] * 1000.0;
      if (width >= policy_min_mm && width <= policy_max_mm && width <= gripper_max_mm) {
        return width;
      }
    }
  }
  return std::nullopt;
}

enum class GraspContactPolicyReason
{
  ACCEPTED,
  TARGET_NOT_FOUND,
  TARGET_AMBIGUOUS,
  INVALID_CONFIG,
  SCENE_SERVICE_UNAVAILABLE,
  SCENE_READ_TIMEOUT,
  ACM_APPLY_FAILED
};

inline uint32_t grasp_contact_policy_planning_scene_components()
{
  return moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
         moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
}

struct GraspContactPolicyResolution
{
  GraspContactPolicyReason reason{GraspContactPolicyReason::INVALID_CONFIG};
  std::string target_identity;
  std::string resolved_collision_object_id;
  std::vector<std::string> prefix_matches;
  std::vector<std::string> authorized_links;
};

inline GraspContactPolicyReason planning_scene_failure_reason(
  bool scene_services_available, bool scene_read_ready, bool acm_apply_succeeded)
{
  if (!scene_services_available) {
    return GraspContactPolicyReason::SCENE_SERVICE_UNAVAILABLE;
  }
  if (!scene_read_ready) {
    return GraspContactPolicyReason::SCENE_READ_TIMEOUT;
  }
  if (!acm_apply_succeeded) {
    return GraspContactPolicyReason::ACM_APPLY_FAILED;
  }
  return GraspContactPolicyReason::ACCEPTED;
}

enum class GraspContactClass
{
  EXPECTED_TARGET_FINGERTIP,
  TARGET_UNAUTHORIZED_ROBOT_LINK,
  ROBOT_SELF_CONTACT,
  UNRELATED_WORLD_OBJECT,
  UNRELATED_FIXTURE,
  STALE_OR_DUPLICATE_COLLISION_OBJECT
};

inline bool valid_grasp_contact_config(
  const std::string & prefix, const std::string & target_identity,
  const std::vector<std::string> & authorized_links)
{
  return !prefix.empty() && !target_identity.empty() && authorized_links.size() == 2U &&
         std::all_of(
    authorized_links.begin(), authorized_links.end(),
    [](const std::string & link) {return !link.empty();}) &&
         authorized_links[0] != authorized_links[1];
}

inline GraspContactPolicyResolution resolve_current_target(
  const std::vector<moveit_msgs::msg::CollisionObject> & objects,
  const std::string & prefix, const std::string & current_fixture_object_id,
  const std::vector<std::string> & authorized_links)
{
  GraspContactPolicyResolution result;
  result.target_identity = current_fixture_object_id;
  result.authorized_links = authorized_links;
  if (!valid_grasp_contact_config(prefix, current_fixture_object_id, authorized_links)) {
    result.reason = GraspContactPolicyReason::INVALID_CONFIG;
    return result;
  }

  std::size_t target_count = 0U;
  for (const auto & object : objects) {
    if (object.id.rfind(prefix, 0) == 0) {
      result.prefix_matches.push_back(object.id);
    }
    if (object.id == current_fixture_object_id) {
      ++target_count;
    }
  }
  if (target_count == 0U) {
    result.reason = GraspContactPolicyReason::TARGET_NOT_FOUND;
  } else if (target_count != 1U) {
    result.reason = GraspContactPolicyReason::TARGET_AMBIGUOUS;
  } else {
    result.resolved_collision_object_id = current_fixture_object_id;
    result.reason = GraspContactPolicyReason::ACCEPTED;
  }
  return result;
}

inline std::optional<moveit_msgs::msg::AllowedCollisionMatrix> allow_current_target_contacts(
  const moveit_msgs::msg::AllowedCollisionMatrix & original,
  const std::string & target_object_id, const std::vector<std::string> & authorized_links)
{
  if (target_object_id.empty() || authorized_links.size() != 2U ||
    authorized_links[0].empty() || authorized_links[1].empty() ||
    authorized_links[0] == authorized_links[1])
  {
    return std::nullopt;
  }

  moveit_msgs::msg::AllowedCollisionMatrix result = original;
  std::vector<std::string> names = original.entry_names;
  const auto append_name = [&names](const std::string & name) {
      if (std::find(names.begin(), names.end(), name) == names.end()) {
        names.push_back(name);
      }
    };
  append_name(target_object_id);
  for (const auto & link : authorized_links) {
    append_name(link);
  }

  std::vector<std::vector<bool>> matrix(names.size(), std::vector<bool>(names.size(), false));
  for (std::size_t row = 0; row < original.entry_names.size(); ++row) {
    for (std::size_t column = 0; column < original.entry_names.size(); ++column) {
      if (row >= original.entry_values.size() ||
        column >= original.entry_values[row].enabled.size())
      {
        continue;
      }
      const auto new_row = static_cast<std::size_t>(std::distance(
          names.begin(), std::find(names.begin(), names.end(), original.entry_names[row])));
      const auto new_column = static_cast<std::size_t>(std::distance(
          names.begin(), std::find(names.begin(), names.end(), original.entry_names[column])));
      matrix[new_row][new_column] = original.entry_values[row].enabled[column];
    }
  }

  const auto target_index = static_cast<std::size_t>(std::distance(
      names.begin(), std::find(names.begin(), names.end(), target_object_id)));
  for (const auto & link : authorized_links) {
    const auto link_index = static_cast<std::size_t>(std::distance(
        names.begin(), std::find(names.begin(), names.end(), link)));
    matrix[target_index][link_index] = true;
    matrix[link_index][target_index] = true;
  }

  result.entry_names = names;
  result.entry_values.clear();
  for (const auto & values : matrix) {
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled = values;
    result.entry_values.push_back(entry);
  }
  return result;
}

inline GraspContactClass classify_grasp_contact(
  const std::string & body1, uint8_t type1, const std::string & body2, uint8_t type2,
  const std::string & target_object_id, const std::vector<std::string> & authorized_links)
{
  const auto is_target = [&target_object_id](const std::string & body) {
      return body == target_object_id;
    };
  const auto is_link = [&authorized_links](const std::string & body) {
      return std::find(authorized_links.begin(), authorized_links.end(), body) !=
             authorized_links.end();
    };
  if (type1 == 1U && type2 == 1U) {
    return GraspContactClass::ROBOT_SELF_CONTACT;
  }
  if ((is_target(body1) && is_link(body2)) || (is_target(body2) && is_link(body1))) {
    return GraspContactClass::EXPECTED_TARGET_FINGERTIP;
  }
  if ((is_target(body1) && type2 == 1U) || (is_target(body2) && type1 == 1U)) {
    return GraspContactClass::TARGET_UNAUTHORIZED_ROBOT_LINK;
  }
  if ((type1 == 2U && body1.rfind("wu14_fixture_", 0) == 0 && !is_target(body1)) ||
    (type2 == 2U && body2.rfind("wu14_fixture_", 0) == 0 && !is_target(body2)))
  {
    return GraspContactClass::UNRELATED_FIXTURE;
  }
  if (type1 == 2U || type2 == 2U) {
    return GraspContactClass::UNRELATED_WORLD_OBJECT;
  }
  return GraspContactClass::STALE_OR_DUPLICATE_COLLISION_OBJECT;
}

}  // namespace arm_cell_motion
