#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <functional>
#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "arm_cell_motion/pick_approach_ik.hpp"
#include "arm_cell_motion/trajectory_continuity.hpp"
#include "arm_cell_motion/pick_collision_seed_policy.hpp"
#include "arm_cell_motion/pick_approach_axis.hpp"
#include "arm_cell_motion/executable_trajectory_finalize.hpp"
#include "arm_cell_motion/grasp_collision_validator.hpp"
#include "arm_cell_motion/pick_grasp_contact.hpp"
#include "arm_cell_motion/pick_grasp_ik.hpp"
#include "arm_cell_motion/pick_grasp_kinematics.hpp"
#include "arm_cell_motion/pick_grasp_pose_candidates.hpp"
#include "arm_cell_motion/pick_search_budget.hpp"
#include "arm_cell_motion/pick_search_controller.hpp"
#include "arm_cell_motion/planning_scene_builder.hpp"
#include "arm_cell_motion/planning_scene_policy.hpp"

namespace arm_cell_motion
{

struct PickPlanningCandidate
{
  geometry_msgs::msg::PoseStamped object_pose;
  geometry_msgs::msg::PoseStamped target_pose;
  geometry_msgs::msg::PoseStamped approach_pose;
  std::vector<double> joint_goal;
  std::vector<double> grasp_joint_goal;
  double joint_distance{0.0};
  double expected_contact_width_mm{0.0};
  std::size_t closing_axis_index{0U};
  std::size_t sample_index{0U};
};


// State produced by exactly one Pose/Seed attempt. Replaced before every retry.
struct PickPlanningAttempt
{
  std::size_t current_pose_index{0U};
  std::map<std::string, double> stage_elapsed_s;
  std::vector<std::string> executed_stages;
  std::vector<double> current_seed;
  geometry_msgs::msg::PoseStamped current_grasp_goal;
  geometry_msgs::msg::PoseStamped current_approach_goal;
  PickGraspIkResult grasp_ik_result;
  std::optional<PickGraspKinematics> grasp_validation_result;
  PickGraspContactValidation grasp_contact_result;
  PickCollisionValidationResult grasp_collision_result;
  GraspCollisionValidationResult grasp_occupancy_result;
  PickApproachIkResult approach_ik_result;
  Eigen::Quaterniond grasp_fk_orientation{Eigen::Quaterniond::Identity()};
  std::vector<double> grasp_joint_positions;
  std::vector<double> approach_joint_positions;
  std::vector<double> selected_joint_goal;
  std::optional<std::size_t> expected_closing_axis;
  double approach_ik_elapsed_s{0.0};
  double grasp_timeout_s{0.0};
  double approach_timeout_s{0.0};
  std::size_t trajectory_points{0U};
  bool candidate_registered{false};
  double joint_distance{0.0};
  std::size_t seed_index{0U};
  PickSearchSeed seed_source{PickSearchSeed::MEASURED};
  std::vector<PickCollisionContact> collision_feedback;
  bool collision_seed_diversification_used{false};
  moveit_msgs::msg::MoveItErrorCodes grasp_ik_error;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan;
  PickSearchEvaluation attempt_evaluation{PickSearchEvaluation::GRASP_VALIDATION_FAILED};
  std::string failure_stage;
  std::string failure_reason;
  double approach_planning_elapsed_s{0.0};
  double effective_planning_timeout_s{0.0};
  int planning_error_code{0};
  std::string finalize_reason{"not_attempted"};
  std::string planning_status{"NOT_RUN"};
  std::vector<std::string> planning_request_evidence;
};

// Final selection survives attempt replacement and is consumed by the transport.
struct PickPlanningResult
{
  moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
  bool selected{false};
  PickSearchStatus search_status{PickSearchStatus::SEARCHING};
  PickSearchEvaluation evaluation{PickSearchEvaluation::GRASP_VALIDATION_FAILED};
  std::vector<double> selected_joint_goal;
  std::size_t pose_index{0U};
  std::size_t seed_index{0U};
  std::string failure_stage;
  std::string failure_reason;
};

// Request-owned state shared by the planning BT's leaves and search control.
// MoveIt runtime handles are borrowed from the transport for this invocation.
struct PlanningContext
{
  PickPlanningAttempt attempt;
  geometry_msgs::msg::PoseStamped object_pose;
  std::vector<PickPlanningCandidate> valid_candidates;
  std::vector<std::size_t> visited_poses;
  PickPlanningResult result;
  std::string operation_identity;
  geometry_msgs::msg::PoseStamped reference_grasp_goal;
  geometry_msgs::msg::Vector3 insertion_axis_tcp;
  PickApproachAxis approach_axis{PickApproachAxis::PLANNING_FRAME_VERTICAL};
  double approach_distance_m{0.0};
  std::vector<double> measured_start;
  std::vector<JointContinuityLimit> continuity_limits;
  std::vector<PickCollisionContact> latest_collision_contacts;
  std::vector<std::vector<std::vector<double>>> attempted_solutions;
  std::shared_ptr<const moveit::core::RobotState> robot_state;
  moveit::core::RobotModelConstPtr robot_model;
  const moveit::core::JointModelGroup * joint_group{nullptr};
  kinematics::KinematicsBasePtr grasp_ik_solver;
  moveit::planning_interface::MoveGroupInterface * move_group{nullptr};
  moveit_msgs::msg::PlanningScene grasp_scene_diff;
  moveit_msgs::msg::PlanningScene approach_scene_diff;
  moveit_msgs::msg::PlanningScene path_scene_diff;
  moveit_msgs::msg::PlanningScene scene_snapshot;
  std::unique_ptr<PlanningSceneBuilder> scene_builder;
  planning_scene::PlanningScenePtr grasp_scene;
  planning_scene::PlanningScenePtr common_grasp_scene;
  GraspOccupancyMode grasp_occupancy_mode{GraspOccupancyMode::FULL_COLLISION};
  FingerClearanceParameters finger_clearance;
  std::vector<arm_cell_interfaces::msg::CollisionPrimitive> non_target_occupancy;
  planning_scene::PlanningScenePtr approach_scene;
  planning_scene::PlanningScenePtr path_scene;
  std::unique_ptr<PickSearchBudget> planning_budget;
  std::vector<PickGraspPoseCandidate> grasp_candidates;
  std::unique_ptr<PickSearchController> search_controller;
  Eigen::Quaterniond object_orientation{Eigen::Quaterniond::Identity()};
  double rectangle_long_side_m{0.0};
  double rectangle_short_side_m{0.0};
  double object_height_m{0.0};
  double grasp_width_min_mm{0.0};
  double grasp_width_max_mm{0.0};
  double gripper_max_width_mm{0.0};
  double grasp_ik_timeout_s{0.0};
  double approach_ik_timeout_s{0.0};
  double approach_planning_timeout_s{0.0};
  std::size_t planning_max_requests{3U};
  unsigned int planning_num_attempts{1U};
  std::function<ExecutableTrajectoryFinalizeResult(moveit_msgs::msg::RobotTrajectory &)>
  finalize_trajectory;
  std::size_t expected_joint_count{0U};
  bool validate_grasp_orientation{true};
  bool apply_pick_retract_tray_exception{false};
  PlanningScenePolicy active_scene_policy;
  std::vector<std::string> diagnostic_evidence;
  std::function<std::uint8_t()> interruption_result;
  std::function<void(std::size_t, std::size_t, PickSearchEvaluation,
    const std::string &, const std::string &)> record_attempt_outcome;
};

// Shared BT mechanics; operation selection is virtual C++ dispatch, never a string lookup.
class PlanningLeaf : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() final;

protected:
  virtual const char * stage_name() const = 0;
  virtual BT::NodeStatus execute(PlanningContext & context) = 0;
};

class PrepareAttempt final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "PrepareAttempt";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class PrepareSeed final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "PrepareSeed";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class SetGraspScenePolicy final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "SetGraspScenePolicy";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class SolveGraspIK final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "SolveGraspIK";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class ValidateGraspKinematics final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "ValidateGraspKinematics";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class ValidateGraspContact final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "ValidateGraspContact";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class ValidateGraspCollision final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "ValidateGraspCollision";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class SetApproachScenePolicy final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "SetApproachScenePolicy";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class SolveApproachIK final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "SolveApproachIK";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class ValidateApproachState final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "ValidateApproachState";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class RegisterValidCandidate final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "RegisterValidCandidate";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class SetPathScenePolicy final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "SetPathScenePolicy";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class PlanApproachPath final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "PlanApproachPath";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class FinalizeTrajectory final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "FinalizeTrajectory";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class RecordAttemptOutcome final : public PlanningLeaf
{
public:
  using PlanningLeaf::PlanningLeaf;

private:
  const char * stage_name() const override {return "RecordAttemptOutcome";}
  BT::NodeStatus execute(PlanningContext & context) override;
};

class SearchPoseSeed final : public BT::ControlNode
{
public:
  SearchPoseSeed(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}
  void halt() override;

private:
  BT::NodeStatus tick() override;
  bool attempt_started_{false};
};

const char * pick_approach_planning_tree_xml();

void register_pick_approach_planning_nodes(BT::BehaviorTreeFactory & factory);

}  // namespace arm_cell_motion
