#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

#include "arm_cell_motion/grasp_contact_policy.hpp"

namespace arm_cell_motion
{

inline constexpr char kPickRetractTrayCollisionObject[] = "amr_tray";
inline constexpr std::array<const char *, 2> kPickRetractTrayAllowedLinks{
  "gripper_robotiq_85_left_finger_tip_link",
  "gripper_robotiq_85_right_finger_tip_link"};

inline uint32_t pick_retract_planning_scene_components()
{
  return moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
         moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
         moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
         moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
         moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
         moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
         moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
         moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
         moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;
}

inline bool is_pick_retract_collision_exception_scope(bool is_pick, bool is_retract)
{
  return is_pick && is_retract;
}

inline std::optional<moveit_msgs::msg::AllowedCollisionMatrix>
make_pick_retract_allowed_collision_matrix(
  const moveit_msgs::msg::AllowedCollisionMatrix & original)
{
  return allow_current_target_contacts(
    original, kPickRetractTrayCollisionObject,
    std::vector<std::string>{kPickRetractTrayAllowedLinks[0], kPickRetractTrayAllowedLinks[1]});
}

inline moveit_msgs::msg::PlanningScene make_pick_retract_planning_scene_diff(
  const moveit_msgs::msg::AllowedCollisionMatrix & original)
{
  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  const auto acm = make_pick_retract_allowed_collision_matrix(original);
  if (acm) {
    diff.allowed_collision_matrix = *acm;
  }
  return diff;
}

}  // namespace arm_cell_motion
