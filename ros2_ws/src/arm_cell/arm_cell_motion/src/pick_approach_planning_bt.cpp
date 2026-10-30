#include "arm_cell_motion/pick_approach_planning_bt.hpp"
#include "pick_approach_tree_xml.hpp"
#include "arm_cell_motion/motion_path_planning.hpp"
#include "arm_cell_motion/pick_approach_geometry.hpp"

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

#include <cmath>
#include <nlohmann/json.hpp>
#include <limits>

namespace arm_cell_motion
{

namespace
{
bool planning_dependency_error(int code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Codes::UNABLE_TO_AQUIRE_SENSOR_DATA:
    case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
    case Codes::INVALID_GROUP_NAME:
    case Codes::INVALID_ROBOT_STATE:
    case Codes::INVALID_LINK_NAME:
    case Codes::INVALID_OBJECT_NAME:
    case Codes::FRAME_TRANSFORM_FAILURE:
    case Codes::COLLISION_CHECKING_UNAVAILABLE:
    case Codes::ROBOT_STATE_STALE:
    case Codes::SENSOR_INFO_STALE:
    case Codes::COMMUNICATION_FAILURE:
    case Codes::CRASH:
    case Codes::ABORT: return true;
    default: return false;
  }
}
}  // namespace

BT::NodeStatus PlanningLeaf::tick()
{
  const auto context = config().blackboard->get<std::shared_ptr<PlanningContext>>("context");
  if (!context) {throw BT::BehaviorTreeException("PlanningLeaf requires request context");}
  if ((context->interruption_result && context->interruption_result() != 0) ||
    (context->planning_budget && context->planning_budget->remaining_s() <= 0.0))
  {
    context->attempt.failure_stage = "INTERRUPTED_OR_BUDGET";
    context->attempt.failure_reason = "RESULT_DISCARDED";
    return BT::NodeStatus::FAILURE;
  }
  const std::string stage = stage_name();
  context->diagnostic_evidence.push_back(name() + ":" + stage);
  context->attempt.executed_stages.push_back(stage);
  const auto started = std::chrono::steady_clock::now();
  const auto status = execute(*context);
  context->attempt.stage_elapsed_s[stage] += std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  return status;
}

void SearchPoseSeed::halt()
{
  if (attempt_started_) {
    const auto context = config().blackboard->get<std::shared_ptr<PlanningContext>>("context");
    if (context && context->search_controller) {
      (void)context->search_controller->next(0.0, true);
      context->result = PickPlanningResult();
      context->result.search_status = PickSearchStatus::INTERRUPTED;
      context->result.failure_stage = "INTERRUPTED";
      context->result.failure_reason = "RESULT_DISCARDED";
    }
  }
  attempt_started_ = false;
  BT::ControlNode::halt();
}

BT::NodeStatus SearchPoseSeed::tick()
{
  if (childrenCount() != 1) {
    throw BT::BehaviorTreeException("SearchPoseSeed requires one attempt");
  }
  const auto context = config().blackboard->get<std::shared_ptr<PlanningContext>>("context");
  if (!context || !context->search_controller || !context->planning_budget) {
    throw BT::BehaviorTreeException("SearchPoseSeed requires initialized search context");
  }
  auto & controller = *context->search_controller;
  const auto interrupted = [&]() {
      return context->interruption_result && context->interruption_result() != 0;
    };
  const auto terminal = [&]() {
      auto & result = context->result;
      result.search_status = controller.status();
      result.evaluation = context->attempt.attempt_evaluation;
      result.failure_stage = context->attempt.failure_stage;
      result.failure_reason = context->attempt.failure_reason;
      if (result.search_status != PickSearchStatus::SELECTED) {
        result.selected = false;
        result.selected_plan = moveit::planning_interface::MoveGroupInterface::Plan();
        result.selected_joint_goal.clear();
      }
      haltChildren();
      return result.selected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    };
  for (;; ) {
    if (!attempt_started_) {
      const auto status = controller.next(context->planning_budget->remaining_s(), interrupted());
      if (status != PickSearchStatus::SEARCHING) {
        context->attempt.failure_stage = status == PickSearchStatus::INTERRUPTED ? "INTERRUPTED" :
          status == PickSearchStatus::SEARCH_BUDGET_EXHAUSTED ? "SEARCH_BUDGET" : "SEARCH";
        context->attempt.failure_reason = status ==
          PickSearchStatus::INTERRUPTED ? "RESULT_DISCARDED" :
          status ==
          PickSearchStatus::SEARCH_BUDGET_EXHAUSTED ? "SEARCH_BUDGET_EXHAUSTED" :
          "ALL_CANDIDATES_EXHAUSTED";
        return terminal();
      }
      context->attempt = PickPlanningAttempt();
      context->attempt.current_pose_index = controller.pose_index();
      if (std::find(
          context->visited_poses.begin(), context->visited_poses.end(),
          controller.pose_index()) ==
        context->visited_poses.end()) {context->visited_poses.push_back(controller.pose_index());}
      context->attempt.seed_index = controller.seed_attempt();
      context->attempt.seed_source = controller.seed_source();
      context->result = PickPlanningResult();
      attempt_started_ = true;
    }
    const auto status = children_nodes_[0]->executeTick();
    if (status == BT::NodeStatus::RUNNING) {return status;}
    attempt_started_ = false;
    const bool cancelled = interrupted();
    const bool exhausted = context->planning_budget->remaining_s() <= 0.0 &&
      context->attempt.attempt_evaluation != PickSearchEvaluation::BACKEND_ERROR;
    if (cancelled || exhausted) {
      context->attempt.failure_stage = cancelled ? "INTERRUPTED" : "SEARCH_BUDGET";
      context->attempt.failure_reason = cancelled ? "RESULT_DISCARDED" : "SEARCH_BUDGET_EXHAUSTED";
    } else if (context->result.selected && status == BT::NodeStatus::SUCCESS) {
      context->attempt.attempt_evaluation = PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED;
    }
    // Diagnostics consume the completed attempt before controller transition.
    if (context->record_attempt_outcome) {
      context->record_attempt_outcome(
        context->attempt.current_pose_index,
        controller.seed_attempt(),
        context->attempt.attempt_evaluation, context->attempt.failure_stage,
        context->attempt.failure_reason);
    }
    context->diagnostic_evidence.push_back(
      "SearchPoseSeed:evaluate:" +
      std::string(pick_search_evaluation_name(context->attempt.attempt_evaluation)));
    if (cancelled || exhausted) {
      (void)controller.next(0.0, cancelled);
      return terminal();
    }
    controller.evaluate(context->attempt.attempt_evaluation);
    if (controller.status() != PickSearchStatus::SEARCHING) {return terminal();}
    resetChildren();
  }
}

const char * pick_approach_planning_tree_xml()
{
  return kPickApproachPlanningTreeXml;
}

void register_pick_approach_planning_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<SearchPoseSeed>("SearchPoseSeed");
  factory.registerNodeType<PrepareAttempt>("PrepareAttempt");
  factory.registerNodeType<PrepareSeed>("PrepareSeed");
  factory.registerNodeType<SetGraspScenePolicy>("SetGraspScenePolicy");
  factory.registerNodeType<SolveGraspIK>("SolveGraspIK");
  factory.registerNodeType<ValidateGraspKinematics>("ValidateGraspKinematics");
  factory.registerNodeType<ValidateGraspContact>("ValidateGraspContact");
  factory.registerNodeType<ValidateGraspCollision>("ValidateGraspCollision");
  factory.registerNodeType<SetApproachScenePolicy>("SetApproachScenePolicy");
  factory.registerNodeType<SolveApproachIK>("SolveApproachIK");
  factory.registerNodeType<ValidateApproachState>("ValidateApproachState");
  factory.registerNodeType<RegisterValidCandidate>("RegisterValidCandidate");
  factory.registerNodeType<SetPathScenePolicy>("SetPathScenePolicy");
  factory.registerNodeType<PlanApproachPath>("PlanApproachPath");
  factory.registerNodeType<FinalizeTrajectory>("FinalizeTrajectory");
  factory.registerNodeType<RecordAttemptOutcome>("RecordAttemptOutcome");
}

BT::NodeStatus PrepareAttempt::execute(PlanningContext & context)
{
  auto & attempt = context.attempt;
  attempt.grasp_occupancy_result.mode = context.grasp_occupancy_mode;
  const auto & candidate = context.grasp_candidates.at(attempt.current_pose_index);
  attempt.seed_index = context.search_controller->seed_attempt();
  attempt.seed_source = context.search_controller->seed_source();
  attempt.current_grasp_goal = context.reference_grasp_goal;
  if (candidate.contact) {
    const auto & q = candidate.tcp_orientation;
    auto & orientation = attempt.current_grasp_goal.pose.orientation;
    orientation.x = q.x(); orientation.y = q.y(); orientation.z = q.z(); orientation.w = q.w();
    const Eigen::Vector3d offset = context.object_orientation.normalized() *
      candidate.grasp_position_offset_object;
    attempt.current_grasp_goal.pose.position.x += offset.x();
    attempt.current_grasp_goal.pose.position.y += offset.y();
    attempt.current_grasp_goal.pose.position.z += offset.z();
    attempt.expected_closing_axis = candidate.contact->closing_axis;
  }
  attempt.current_approach_goal = pick_approach_pose(
    attempt.current_grasp_goal,
    context.approach_axis, context.insertion_axis_tcp, context.approach_distance_m);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PrepareSeed::execute(PlanningContext & context)
{
  auto & attempt = context.attempt;
  attempt.current_seed = context.measured_start;
  if (attempt.seed_source == PickSearchSeed::DIVERSIFIED) {
    for (const auto & contact : context.latest_collision_contacts) {
      if (contact.pose_candidate == attempt.current_pose_index + 1U &&
        contact.seed_attempt + 1U == attempt.seed_index)
      {
        attempt.collision_feedback.push_back(contact);
      }
    }
    moveit::core::RobotState seed(*context.robot_state);
    seed.setToRandomPositions(context.joint_group);
    seed.update();
    std::vector<double> randomized;
    seed.copyJointGroupPositions(context.joint_group, randomized);
    const auto variables = context.joint_group->getVariableNames();
    const auto influenced = collision_influenced_variables(
      attempt.collision_feedback, variables,
      [&context](const std::string & body) {
        std::vector<std::string> result;
        const auto & links = context.robot_model->getLinkModelNames();
        if (std::find(links.begin(), links.end(), body) == links.end()) {return result;}
        auto * link = context.robot_model->getLinkModel(body);
        while (link) {
          const auto * joint = link->getParentJointModel();
          if (joint && context.joint_group->hasJointModel(joint->getName())) {
            const auto & names = joint->getVariableNames();
            result.insert(result.end(), names.begin(), names.end());
          }
          link = link->getParentLinkModel();
        }
        return result;
      });
    attempt.current_seed = diversify_pick_ik_seed(
      context.measured_start, randomized, variables,
      influenced);
    attempt.collision_seed_diversification_used = !influenced.empty();
    seed.setJointGroupPositions(context.joint_group, attempt.current_seed);
    seed.update();
    if (!seed.satisfiesBounds(context.joint_group)) {
      attempt.failure_stage = "IK";
      attempt.failure_reason = "NO_BOUNDED_VALID_SEED";
      return BT::NodeStatus::FAILURE;
    }
  }
  context.latest_collision_contacts.clear();
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetGraspScenePolicy::execute(PlanningContext & context)
{
  context.active_scene_policy.vision_occupancy = VisionOccupancyPolicy::INCLUDE;
  context.active_scene_policy.apply_pick_retract_tray_exception = false;
  if (!context.scene_builder) {
    context.attempt.failure_stage = "GRASP_SCENE";
    context.attempt.failure_reason = "SCENE_BUILDER_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto scene = context.scene_builder->build(context.active_scene_policy);
  if (!scene.valid) {
    context.attempt.failure_stage = "GRASP_SCENE";
    context.attempt.failure_reason = scene.failure_reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  context.grasp_scene_diff = scene.diff;
  if (!context.robot_model) {
    context.attempt.failure_stage = "GRASP_SCENE";
    context.attempt.failure_reason = "ROBOT_MODEL_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  context.grasp_scene = std::make_shared<planning_scene::PlanningScene>(context.robot_model);
  if (!context.grasp_scene->setPlanningSceneMsg(context.scene_builder->snapshot()) ||
    !context.grasp_scene->setPlanningSceneDiffMsg(scene.diff))
  {
    context.grasp_scene.reset();
    context.attempt.failure_stage = "GRASP_SCENE";
    context.attempt.failure_reason = "GRASP_SCENE_DIFF_INVALID";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto common = context.scene_builder->build(PlanningScenePolicy{});
  context.common_grasp_scene = std::make_shared<planning_scene::PlanningScene>(context.robot_model);
  if (!common.valid ||
    !context.common_grasp_scene->setPlanningSceneMsg(context.scene_builder->snapshot()) ||
    !context.common_grasp_scene->setPlanningSceneDiffMsg(common.diff))
  {
    context.common_grasp_scene.reset();
    context.attempt.failure_stage = "GRASP_SCENE";
    context.attempt.failure_reason = "COMMON_GRASP_SCENE_INVALID";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SolveGraspIK::execute(PlanningContext & context)
{
  if (!context.grasp_ik_solver || !context.planning_budget) {
    context.attempt.failure_stage = "GRASP_IK";
    context.attempt.failure_reason = "IK_SOLVER_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const double timeout_s = context.planning_budget->call_timeout_s(
    context.grasp_ik_timeout_s);
  if (timeout_s <= 0.0) {
    context.attempt.failure_stage = "GRASP_IK";
    context.attempt.failure_reason = "PICK_SEARCH_BUDGET_EXHAUSTED";
    return BT::NodeStatus::FAILURE;
  }
  context.attempt.grasp_timeout_s = timeout_s;
  moveit_msgs::msg::MoveItErrorCodes error;
  const auto default_callback = [&context](const geometry_msgs::msg::Pose &,
      const std::vector<double> & positions,
      moveit_msgs::msg::MoveItErrorCodes & callback_error) {
      if (context.interruption_result && context.interruption_result() !=
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
      {
        callback_error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      } else if (positions.size() == context.expected_joint_count &&
        std::all_of(
          positions.begin(), positions.end(),
          [](double value) {return std::isfinite(value);}))
      {
        context.attempt.grasp_joint_positions = positions;
        callback_error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      } else {
        context.attempt.failure_reason = "nonfinite_or_joint_count";
        callback_error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      }
    };
  const auto callback = default_callback;
  context.attempt.grasp_ik_result = solve_pick_grasp_ik(
    context.grasp_ik_solver, context.attempt.current_grasp_goal.pose,
    context.attempt.current_seed,
    timeout_s, callback, moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  error = context.attempt.grasp_ik_result.error;
  context.attempt.grasp_ik_error = error;
  if (!context.attempt.grasp_ik_result.solver_returned ||
    context.attempt.grasp_joint_positions.empty())
  {
    context.attempt.failure_stage = "GRASP_IK";
    context.attempt.failure_reason =
      context.attempt.failure_reason.empty() ? "TRAC_IK_NO_SOLUTION" :
      context.attempt.failure_reason;
    context.attempt.attempt_evaluation = context.attempt.grasp_ik_result.solver_returned ?
      PickSearchEvaluation::GRASP_VALIDATION_FAILED : planning_dependency_error(error.val) ?
      PickSearchEvaluation::BACKEND_ERROR : error.val ==
      moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION ?
      PickSearchEvaluation::GRASP_IK_NO_SOLUTION : PickSearchEvaluation::GRASP_IK_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ValidateGraspKinematics::execute(PlanningContext & context)
{
  auto & previous = context.attempted_solutions.at(context.attempt.current_pose_index);
  const auto & solution = context.attempt.grasp_joint_positions;
  const bool duplicate = std::any_of(
    previous.begin(), previous.end(), [&solution](const auto & existing) {
      if (existing.size() != solution.size()) {return false;}
      double distance = 0.0;
      for (std::size_t joint = 0; joint < solution.size(); ++joint) {
        const double delta =
        std::remainder(solution[joint] - existing[joint], 6.28318530717958647692);
        distance += delta * delta;
      }
      return distance < 1e-4;
    });
  if (duplicate) {
    context.attempt.failure_stage = "GRASP_KINEMATICS";
    context.attempt.failure_reason = "DUPLICATE_IK_RESULT";
    context.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_VALIDATION_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  previous.push_back(solution);
  if (!context.robot_state || !context.joint_group) {
    context.attempt.failure_stage = "GRASP_KINEMATICS";
    context.attempt.failure_reason = "ROBOT_STATE_OR_GROUP_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  context.attempt.grasp_validation_result = validate_pick_grasp_kinematics(
    *context.robot_state, context.joint_group, context.attempt.grasp_joint_positions,
    context.expected_joint_count, context.attempt.current_grasp_goal,
    context.validate_grasp_orientation);
  if (context.attempt.grasp_validation_result->check != PickGraspKinematicCheck::VALID) {
    context.attempt.failure_stage = "GRASP_KINEMATICS";
    context.attempt.failure_reason = context.attempt.grasp_validation_result->reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_VALIDATION_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  context.attempt.grasp_fk_orientation =
    context.attempt.grasp_validation_result->actual_orientation;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ValidateGraspContact::execute(PlanningContext & context)
{
  context.attempt.grasp_contact_result = validate_pick_grasp_contact(
    context.attempt.grasp_fk_orientation, context.attempt.expected_closing_axis,
    context.object_orientation, context.rectangle_long_side_m,
    context.rectangle_short_side_m, context.object_height_m,
    context.grasp_width_min_mm, context.grasp_width_max_mm,
    context.gripper_max_width_mm);
  if (context.attempt.grasp_contact_result.check == PickGraspContactCheck::MISMATCH) {
    context.attempt.failure_stage = "GRASP_CONTACT";
    context.attempt.failure_reason = context.attempt.grasp_contact_result.reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_CONTACT_MISMATCH;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ValidateGraspCollision::execute(PlanningContext & context)
{
  if (!context.grasp_scene || !context.common_grasp_scene ||
    !context.attempt.grasp_validation_result) {
    context.attempt.failure_stage = "GRASP_COLLISION";
    context.attempt.failure_reason = "GRASP_SCENE_OR_KINEMATICS_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto & goal = context.attempt.current_grasp_goal.pose;
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  tcp.translation() = Eigen::Vector3d(goal.position.x, goal.position.y, goal.position.z);
  tcp.linear() = Eigen::Quaterniond(goal.orientation.w, goal.orientation.x,
    goal.orientation.y, goal.orientation.z).normalized().toRotationMatrix();
  const auto & center = context.object_pose.pose.position;
  Eigen::Vector3d insertion(context.insertion_axis_tcp.x,
    context.insertion_axis_tcp.y, context.insertion_axis_tcp.z);
  if (context.approach_axis == PickApproachAxis::PLANNING_FRAME_VERTICAL) {
    insertion = tcp.linear().transpose() * -Eigen::Vector3d::UnitZ();
  }
  auto & validation = context.attempt.grasp_occupancy_result;
  validation = validate_grasp_collision(
    context.grasp_occupancy_mode, *context.common_grasp_scene, *context.grasp_scene,
    context.attempt.grasp_validation_result->state, tcp,
    Eigen::Vector3d(center.x, center.y, center.z), context.object_orientation,
    context.attempt.grasp_contact_result.contact, insertion,
    context.non_target_occupancy, context.finger_clearance);
  if (!validation.valid) {
    context.attempt.failure_stage = "GRASP_COLLISION";
    context.attempt.failure_reason = validation.reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  // Preserve the legacy full-scene contact set for collision seed feedback,
  // including attempts where static/self and Vision collisions coexist.
  context.attempt.grasp_collision_result =
    validation.mode == GraspOccupancyMode::FULL_COLLISION ? validation.occupancy :
    validation.common.collision ? validation.common : validation.occupancy;
  if (!validation.allowed) {
    context.latest_collision_contacts.clear();
    for (const auto & contact : context.attempt.grasp_collision_result.contacts) {
      context.latest_collision_contacts.push_back(
        PickCollisionContact{
          contact.first, contact.second, "grasp_target_state", false, 0U,
          context.attempt.current_pose_index + 1U, context.attempt.seed_index});
    }
    context.attempt.failure_stage = "GRASP_COLLISION";
    context.attempt.failure_reason = validation.reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::GRASP_COLLISION;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetApproachScenePolicy::execute(PlanningContext & context)
{
  context.active_scene_policy.vision_occupancy = VisionOccupancyPolicy::EXCLUDE;
  context.active_scene_policy.apply_pick_retract_tray_exception = false;
  if (!context.scene_builder) {
    context.attempt.failure_stage = "APPROACH_SCENE";
    context.attempt.failure_reason = "SCENE_BUILDER_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto scene = context.scene_builder->build(context.active_scene_policy);
  if (!scene.valid) {
    context.attempt.failure_stage = "APPROACH_SCENE";
    context.attempt.failure_reason = scene.failure_reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  context.approach_scene_diff = scene.diff;
  if (!context.robot_model) {
    context.attempt.failure_stage = "APPROACH_SCENE";
    context.attempt.failure_reason = "ROBOT_MODEL_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  context.approach_scene = std::make_shared<planning_scene::PlanningScene>(context.robot_model);
  if (!context.approach_scene->setPlanningSceneMsg(context.scene_builder->snapshot()) ||
    !context.approach_scene->setPlanningSceneDiffMsg(scene.diff))
  {
    context.approach_scene.reset();
    context.attempt.failure_stage = "APPROACH_SCENE";
    context.attempt.failure_reason = "APPROACH_SCENE_DIFF_INVALID";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SolveApproachIK::execute(PlanningContext & context)
{
  if (!context.attempt.grasp_validation_result || !context.joint_group ||
    !context.planning_budget)
  {
    context.attempt.failure_stage = "APPROACH_IK";
    context.attempt.failure_reason = "GRASP_STATE_OR_GROUP_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const double timeout_s = context.planning_budget->call_timeout_s(
    context.approach_ik_timeout_s);
  context.attempt.approach_timeout_s = timeout_s;
  const auto started = std::chrono::steady_clock::now();
  context.attempt.approach_ik_result = solve_pick_approach_ik(
    context.attempt.grasp_validation_result->state, context.joint_group,
    context.attempt.current_approach_goal, context.attempt.grasp_fk_orientation, timeout_s);
  context.attempt.approach_ik_elapsed_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  if (context.attempt.approach_ik_result.check != PickApproachIkCheck::VALID) {
    context.attempt.failure_stage = "APPROACH_IK";
    context.attempt.failure_reason = context.attempt.approach_ik_result.reason;
    context.attempt.attempt_evaluation = context.attempt.approach_ik_result.check ==
      PickApproachIkCheck::APPROACH_IK_FAILED ? PickSearchEvaluation::APPROACH_IK_FAILED :
      PickSearchEvaluation::APPROACH_VALIDATION_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  context.attempt.approach_joint_positions = context.attempt.approach_ik_result.joint_positions;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ValidateApproachState::execute(PlanningContext & context)
{
  auto & attempt = context.attempt;
  if (!context.robot_state || !context.joint_group || !context.approach_scene) {
    attempt.failure_stage = "APPROACH_STATE";
    attempt.failure_reason = "APPROACH_SCENE_OR_STATE_UNAVAILABLE";
    attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto selected = select_nearest_joint_goal(
    attempt.approach_joint_positions,
    context.measured_start, context.continuity_limits,
    [&context](const std::vector<double> & positions, std::string & reason) {
      moveit::core::RobotState state(*context.robot_state);
      state.setJointGroupPositions(context.joint_group, positions);
      state.update();
      const auto collision = validate_pick_state_collision(*context.approach_scene, state);
      if (collision.collision) {
        context.latest_collision_contacts.clear();
        for (const auto & contact : collision.contacts) {
          context.latest_collision_contacts.push_back(
            PickCollisionContact{
            contact.first, contact.second, "approach_target_state", false, 0U,
            context.attempt.current_pose_index + 1U, context.attempt.seed_index});
        }
      }
      reason = collision.collision ? "collision/contact" : "";
      return !collision.collision;
    });
  if (!selected.valid) {
    attempt.failure_stage = "APPROACH_STATE";
    attempt.failure_reason = selected.reason;
    attempt.attempt_evaluation = PickSearchEvaluation::APPROACH_VALIDATION_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  const auto validation = validate_pick_grasp_kinematics(
    *context.robot_state, context.joint_group,
    selected.selected_goal, context.expected_joint_count, attempt.current_approach_goal, true);
  if (validation.check != PickGraspKinematicCheck::VALID) {
    attempt.failure_stage = "APPROACH_STATE";
    attempt.failure_reason = "selected_approach_fk_target_mismatch";
    attempt.attempt_evaluation = PickSearchEvaluation::APPROACH_VALIDATION_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  attempt.selected_joint_goal = selected.selected_goal;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RegisterValidCandidate::execute(PlanningContext & context)
{
  auto & attempt = context.attempt;
  if (attempt.selected_joint_goal.size() != context.measured_start.size()) {
    attempt.failure_stage = "CANDIDATE_REGISTRATION";
    attempt.failure_reason = "INVALID_SELECTED_JOINT_GOAL";
    attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  double distance_squared = 0.0;
  for (std::size_t joint = 0; joint < context.measured_start.size(); ++joint) {
    const double delta = std::remainder(
      attempt.selected_joint_goal[joint] - context.measured_start[joint],
      6.28318530717958647692);
    distance_squared += delta * delta;
  }
  attempt.joint_distance = std::sqrt(distance_squared);
  attempt.candidate_registered = true;
  const auto contact = attempt.grasp_contact_result.contact;
  context.valid_candidates.push_back(
    PickPlanningCandidate{context.object_pose,
      attempt.current_grasp_goal, attempt.current_approach_goal, attempt.selected_joint_goal,
      attempt.grasp_joint_positions, attempt.joint_distance, contact ? contact->width_mm : 0.0,
      contact ? contact->closing_axis : 0U, attempt.current_pose_index});
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetPathScenePolicy::execute(PlanningContext & context)
{
  context.active_scene_policy.vision_occupancy = VisionOccupancyPolicy::INCLUDE;
  context.active_scene_policy.apply_pick_retract_tray_exception =
    context.apply_pick_retract_tray_exception;
  if (!context.scene_builder) {
    context.attempt.failure_stage = "APPROACH_PATH_SCENE";
    context.attempt.failure_reason = "SCENE_BUILDER_UNAVAILABLE";
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto scene = context.scene_builder->build(context.active_scene_policy);
  if (!scene.valid) {
    context.attempt.failure_stage = "APPROACH_PATH_SCENE";
    context.attempt.failure_reason = scene.failure_reason;
    context.attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  context.path_scene_diff = scene.diff;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PlanApproachPath::execute(PlanningContext & context)
{
  auto & attempt = context.attempt;
  if (!context.move_group || !context.robot_state || !context.joint_group ||
    !context.planning_budget)
  {
    attempt.failure_stage = "GLOBAL_PLANNING";
    attempt.failure_reason = "MOVE_GROUP_OR_STATE_UNAVAILABLE";
    attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  auto & group = *context.move_group;
  moveit::core::RobotState target(*context.robot_state);
  target.setJointGroupPositions(context.joint_group, attempt.selected_joint_goal);
  target.update();
  const bool target_set = group.setJointValueTarget(target);
  const auto started = std::chrono::steady_clock::now();
  moveit_msgs::msg::MotionPlanRequest original;
  group.constructMotionPlanRequest(original);
  struct Restore
  {
    moveit::planning_interface::MoveGroupInterface & group;
    unsigned int count;
    ~Restore() {group.setNumPlanningAttempts(count);}
  } restore{group, static_cast<unsigned int>(std::max(1, original.num_planning_attempts))};
  group.setNumPlanningAttempts(context.planning_num_attempts);
  attempt.planning_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  for (std::size_t index = 0; target_set && index < context.planning_max_requests; ++index) {
    if (context.interruption_result && context.interruption_result() != 0) {break;}
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    const double remaining = context.planning_budget->remaining_s();
    const double timeout = std::min(context.approach_planning_timeout_s - elapsed, remaining);
    if (timeout <= 0.0) {break;}
    attempt.effective_planning_timeout_s = timeout;
    group.setPlanningTime(timeout);
    const auto call_started = std::chrono::steady_clock::now();
    const auto result = plan_pick_approach(
      group, attempt.current_plan, &context.path_scene_diff,
      [&context]() {return context.interruption_result && context.interruption_result() != 0;});
    attempt.planning_error_code = result.val;
    const bool valid = result == moveit::core::MoveItErrorCode::SUCCESS &&
      !attempt.current_plan.trajectory_.joint_trajectory.points.empty();
    nlohmann::json evidence = {{"request_index", index + 1U},
      {"measured_start_joints", context.measured_start},
      {"target_joints", attempt.selected_joint_goal},
      {"moveit_error_code", result.val},
      {"elapsed_s", std::chrono::duration<double>(
          std::chrono::steady_clock::now() - call_started).count()},
      {"planning_timeout_limit_s", timeout}, {"planning_time_remaining_s", remaining},
      {"trajectory_points", attempt.current_plan.trajectory_.joint_trajectory.points.size()},
      {"result", valid ? "SUCCESS" : "PLANNING_FAILED"}};
    if (!valid) {evidence["collision_feedback"] = "NOT_RETURNED_BY_PLANNER";}
    attempt.planning_request_evidence.push_back(evidence.dump());
    if (planning_dependency_error(result.val)) {break;}
    if (valid) {
      attempt.approach_planning_elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      attempt.planning_status = "SUCCESS";
      return BT::NodeStatus::SUCCESS;
    }
  }
  attempt.approach_planning_elapsed_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  attempt.failure_stage = "GLOBAL_PLANNING";
  attempt.failure_reason = target_set ? "planning_failed" : "target_rejected";
  attempt.planning_status = "FAILED";
  attempt.attempt_evaluation = planning_dependency_error(attempt.planning_error_code) ?
    PickSearchEvaluation::BACKEND_ERROR : PickSearchEvaluation::GLOBAL_PLANNING_FAILED;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FinalizeTrajectory::execute(PlanningContext & context)
{
  auto & attempt = context.attempt;
  if (!context.finalize_trajectory) {
    attempt.failure_stage = "TRAJECTORY_FINALIZATION";
    attempt.failure_reason = "FINALIZER_UNAVAILABLE";
    attempt.attempt_evaluation = PickSearchEvaluation::BACKEND_ERROR;
    return BT::NodeStatus::FAILURE;
  }
  const auto finalized = context.finalize_trajectory(attempt.current_plan.trajectory_);
  attempt.finalize_reason = finalized.reason;
  if (!finalized.success) {
    attempt.failure_stage = "TRAJECTORY_FINALIZATION";
    attempt.failure_reason = finalized.reason;
    attempt.attempt_evaluation = PickSearchEvaluation::GLOBAL_PLANNING_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RecordAttemptOutcome::execute(PlanningContext & context)
{
  if (context.result.selected) {return BT::NodeStatus::SUCCESS;}
  if (context.attempt.current_plan.trajectory_.joint_trajectory.points.empty()) {
    context.attempt.failure_stage = "GLOBAL_PLANNING";
    context.attempt.failure_reason = "empty_approach_plan";
    context.attempt.attempt_evaluation = PickSearchEvaluation::GLOBAL_PLANNING_FAILED;
    return BT::NodeStatus::FAILURE;
  }
  context.attempt.trajectory_points =
    context.attempt.current_plan.trajectory_.joint_trajectory.points.size();
  context.result.selected_plan = std::move(context.attempt.current_plan);
  context.result.selected = true;
  context.result.selected_joint_goal = context.attempt.selected_joint_goal;
  context.result.pose_index = context.attempt.current_pose_index;
  context.result.seed_index = context.attempt.seed_index;
  context.attempt.attempt_evaluation = PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED;
  context.attempt.failure_stage = "SUCCESS";
  context.attempt.failure_reason.clear();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace arm_cell_motion
