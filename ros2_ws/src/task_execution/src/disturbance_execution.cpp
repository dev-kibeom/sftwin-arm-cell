#include "arm_cell_task_execution/disturbance_execution.hpp"

#include <cmath>
#include <array>
#include <utility>

#include <arm_cell_interfaces/msg/disturbance_failure_reason.hpp>
#include <arm_cell_interfaces/msg/disturbance_outcome.hpp>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

#include "arm_cell_motion/candidate_search.hpp"
#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/motion_plan_execute_action.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
using Outcome = arm_cell_interfaces::msg::DisturbanceOutcome;
using Failure = arm_cell_interfaces::msg::DisturbanceFailureReason;

bool is_planning_failure(std::uint8_t code)
{
  return code == Code::TASK_RESULT_IK_UNREACHABLE ||
         code == Code::TASK_RESULT_PATH_OBSTRUCTED ||
         code == Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED;
}

BT::NodeStatus status_for(std::uint8_t code)
{
  return code == Code::TASK_RESULT_SUCCESS ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace

DisturbanceExecution::DisturbanceExecution(
  TaskExecutionSupport & support, MotionDisturbRequest request)
: support_(support), request_(std::move(request))
{}

void DisturbanceExecution::register_nodes(
  BT::BehaviorTreeFactory & factory)
{
  register_callback_action(
    factory, "PrepareDisturbanceSearch", [this]() {
      return prepare_search() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "SelectDisturbanceCandidate", [this]() {
      return select_candidate() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "SelectDisturbanceDirection", [this]() {
      return select_direction() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "PrepareDisturbanceRoute", [this]() {
      return prepare_route() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  const std::array<std::string, 3> plan_names{
    "PlanDisturbanceApproach", "PlanDisturbancePush", "PlanDisturbanceRetract"};
  const std::array<std::string, 3> execute_names{
    "ExecuteDisturbanceApproach", "ExecuteDisturbancePush", "ExecuteDisturbanceRetract"};
  for (std::size_t index = 0; index < plan_names.size(); ++index) {
    register_plan_motion_action(
      factory, plan_names[index], support_, [this, index]() {return route_request(index);},
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_PLANNING,
      [this](const MotionPlanningResult & result) {
        last_result_ = result.result_code;
        if (result.result_code != Code::TASK_RESULT_SUCCESS) {return status_for(last_result_);}
        planned_route_.push_back(result.plan);
        return BT::NodeStatus::SUCCESS;
      });
    register_execute_planned_motion_action(
      factory, execute_names[index], support_, [this, index]() {
        if (index == 0) {execution_started_ = true;}
        return planned_route_.at(index);
      }, index + 1 == execute_names.size(), [this](const MotionPortResult & result) {
        last_result_ = result.result_code;
        return status_for(last_result_);
      });
  }
  register_callback_action(
    factory, "AdvanceDisturbanceDirection", [this]() {
      return allow_direction_transition() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  register_callback_action(
    factory, "AdvanceDisturbanceCandidate", [this]() {
      return allow_candidate_transition() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
}

bool DisturbanceExecution::prepare_search()
{
  std::vector<CandidatePriority> priorities;
  priorities.reserve(request_.disturbance_candidates.size());
  for (std::size_t index = 0; index < request_.disturbance_candidates.size(); ++index) {
    priorities.push_back(
      {index, 0.0,
        request_.disturbance_candidates[index].observed_height_m, {}});
  }
  candidate_order_ = order_candidate_indices(priorities, CandidatePriorityPolicy::HEIGHT_FIRST);
  candidate_cursor_ = 0;
  direction_cursor_ = 0;
  last_result_ = Code::TASK_RESULT_PATH_OBSTRUCTED;
  execution_started_ = false;
  return !candidate_order_.empty();
}

bool DisturbanceExecution::select_candidate()
{
  if (candidate_cursor_ >= candidate_order_.size() || execution_started_) {return false;}
  selected_candidate_ = candidate_order_[candidate_cursor_++];
  direction_cursor_ = 0;
  return true;
}

bool DisturbanceExecution::select_direction()
{
  const auto directions = disturbance_directions();
  if (direction_cursor_ >= directions.size() || execution_started_) {return false;}
  selected_direction_ = directions[direction_cursor_++];
  return true;
}

bool DisturbanceExecution::prepare_route()
{
  route_requests_.clear();
  planned_route_.clear();
  const auto & candidate = request_.disturbance_candidates.at(selected_candidate_);
  if (!std::isfinite(candidate.observed_height_m) ||
    candidate.center.header.frame_id != "base_link" ||
    !std::isfinite(candidate.center.pose.position.x) ||
    !std::isfinite(candidate.center.pose.position.y) ||
    !std::isfinite(candidate.center.pose.position.z))
  {
    last_result_ = Code::TASK_RESULT_INVALID_GOAL;
    return false;
  }
  const auto parameters = support_.motion_port()->disturbanceParameters();
  const auto route = make_disturbance_route(
    candidate.center, selected_direction_, parameters.push_distance_m,
    parameters.approach_clearance_m);
  const auto make_request = [&](const geometry_msgs::msg::PoseStamped & target,
      MotionSegmentKind segment) {
      MotionPortRequest motion;
      motion.execution_id = request_.execution_id;
      motion.movement_id = request_.movement_id;
      motion.segment = segment;
      motion.target_pose = target;
      motion.constraints.motion_speed_fraction = parameters.speed_fraction;
      motion.vision_acquisition_id = request_.acquisition_id;
      motion.vision_candidate_id = candidate.candidate_id;
      return motion;
    };
  route_requests_ = {
    make_request(route.approach, MotionSegmentKind::DISTURB_APPROACH),
    make_request(route.push_end, MotionSegmentKind::DISTURB_PUSH),
    make_request(route.retract_end, MotionSegmentKind::DISTURB_RETRACT)};
  last_result_ = Code::TASK_RESULT_SUCCESS;
  return true;
}

MotionPortRequest DisturbanceExecution::route_request(std::size_t index) const
{
  auto request = route_requests_.at(index);
  if (index > 0) {
    request.has_start_joint_state = true;
    request.start_joint_state = planned_route_.at(index - 1).end_state;
  }
  return request;
}

bool DisturbanceExecution::allow_direction_transition() const
{
  return !execution_started_ && is_planning_failure(last_result_) &&
         direction_cursor_ < disturbance_directions().size();
}

bool DisturbanceExecution::allow_candidate_transition() const
{
  return !execution_started_ && is_planning_failure(last_result_) &&
         direction_cursor_ >= disturbance_directions().size() &&
         candidate_cursor_ < candidate_order_.size();
}

MotionDisturbResult DisturbanceExecution::result() const
{
  MotionDisturbResult result;
  result.execution_id = request_.execution_id;
  result.movement_id = request_.movement_id;
  result.acquisition_id = request_.acquisition_id;
  const auto observed = support_.motion_port()->observeMotionState();
  result.backend_inactivity_confirmed = observed.motion_status.backend_inactivity_confirmed;
  if (last_result_ == Code::TASK_RESULT_SUCCESS) {
    result.outcome.value = Outcome::OUTCOME_COMPLETED;
    result.failure_reason.value = Failure::FAILURE_REASON_NONE;
    result.diagnostic_detail = "Disturbance trajectory completed by measured state";
  } else if (last_result_ == Code::TASK_RESULT_CANCELED) {
    result.outcome.value = Outcome::OUTCOME_CANCELED;
    result.failure_reason.value = Failure::FAILURE_REASON_EXECUTION_FAILURE;
    result.diagnostic_detail = "Disturbance operation canceled";
  } else if (last_result_ == Code::TASK_RESULT_SAFETY_PREEMPTED) {
    result.outcome.value = Outcome::OUTCOME_SAFETY_PREEMPTED;
    result.failure_reason.value = Failure::FAILURE_REASON_EXECUTION_FAILURE;
    result.diagnostic_detail = "Disturbance operation preempted by Safety";
  } else {
    result.outcome.value = Outcome::OUTCOME_FAILED;
    result.failure_reason.value = is_planning_failure(last_result_) ?
      Failure::FAILURE_REASON_PLAN_EXHAUSTED :
      Failure::FAILURE_REASON_EXECUTION_FAILURE;
    result.diagnostic_detail = is_planning_failure(last_result_) ?
      "No candidate direction has a complete feasible plan" :
      "Disturbance motion failed before measured completion";
  }
  return result;
}

}  // namespace arm_cell_task_execution
