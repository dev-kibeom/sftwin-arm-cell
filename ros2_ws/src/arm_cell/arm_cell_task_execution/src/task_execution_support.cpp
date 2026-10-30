#include "arm_cell_task_execution/task_execution_support.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
namespace
{
using Result = arm_cell_interfaces::msg::MotionTaskResultCode;
using Phase = arm_cell_interfaces::msg::MotionTaskPhase;

unique_identifier_msgs::msg::UUID execution_uuid(const std::string & id)
{
  unique_identifier_msgs::msg::UUID uuid;
  std::size_t byte = 0;
  int high = -1;
  for (const char ch : id) {
    int value = -1;
    if (ch >= '0' && ch <= '9') {value = ch - '0';}
    else if (ch >= 'a' && ch <= 'f') {value = ch - 'a' + 10;}
    else if (ch >= 'A' && ch <= 'F') {value = ch - 'A' + 10;}
    if (value < 0) {continue;}
    if (high < 0) {high = value; continue;}
    if (byte < uuid.uuid.size()) {uuid.uuid[byte++] = static_cast<std::uint8_t>((high << 4) | value);}
    high = -1;
  }
  if (byte == 0) {
    for (std::size_t i = 0; i < std::min(id.size(), uuid.uuid.size()); ++i) {
      uuid.uuid[i] = static_cast<std::uint8_t>(id[i]);
    }
    if (!uuid.uuid.empty() && uuid.uuid[0] == 0) {uuid.uuid[0] = 1;}
  }
  return uuid;
}

MotionOrientationSymmetry orientation_symmetry(std::uint8_t order)
{
  switch (order) {
    case 0: return MotionOrientationSymmetry::UNOBSERVABLE;
    case 2: return MotionOrientationSymmetry::TWOFOLD;
    case 4: return MotionOrientationSymmetry::FOURFOLD;
    default: return MotionOrientationSymmetry::UNIQUE;
  }
}
}  // namespace

TaskExecutionSupport::TaskExecutionSupport(
  std::shared_ptr<MotionPort> motion_port, std::shared_ptr<GripperPort> gripper_port,
  MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout)
: motion_port_(std::move(motion_port)), gripper_port_(std::move(gripper_port)),
  geometry_(std::move(geometry)), holding_confirmation_timeout_(holding_confirmation_timeout)
{}

void TaskExecutionSupport::set_interruption_checker(InterruptionChecker checker)
{interruption_checker_ = std::move(checker);}

void TaskExecutionSupport::set_phase_callback(PhaseCallback callback)
{phase_callback_ = std::move(callback);}

void TaskExecutionSupport::set_runtime_geometry(
  MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout)
{
  geometry_ = std::move(geometry);
  holding_confirmation_timeout_ = holding_confirmation_timeout;
}

MotionPortRequest TaskExecutionSupport::make_motion_request(
  const ExecuteTask::Goal & goal, std::uint8_t phase, const std::string & execution_id,
  const std::string & movement_id)
{
  const bool pick = goal.task_type.value == MotionTaskType::TASK_TYPE_PICK;
  MotionPortRequest request;
  request.execution_id = execution_uuid(execution_id);
  request.movement_id = movement_id.empty() ? next_movement_id("segment") : movement_id;
  request.target_pose = goal.target_pose;
  request.segment = pick ?
    (phase == Phase::TASK_PHASE_APPROACHING ? MotionSegmentKind::PICK_APPROACH :
    phase == Phase::TASK_PHASE_RETRACTING ? MotionSegmentKind::PICK_RETRACT :
    MotionSegmentKind::PICK_DESCENT) :
    (phase == Phase::TASK_PHASE_APPROACHING ? MotionSegmentKind::PLACE_APPROACH :
    phase == Phase::TASK_PHASE_RETRACTING ? MotionSegmentKind::PLACE_RETRACT :
    MotionSegmentKind::PLACE_DESCENT);
  request.constraints.approach_distance_m = pick ? goal.pick_approach_distance_m :
    goal.place_approach_distance_m;
  request.constraints.retract_distance_m = pick ? geometry_.pick_retract_distance_m :
    goal.place_retract_distance_m;
  request.constraints.position_tolerance_m = pick ? goal.pick_grasp_position_tolerance_m :
    goal.place_position_tolerance_m;
  request.constraints.orientation_tolerance_rad = pick ?
    goal.pick_grasp_orientation_tolerance_rad : goal.place_orientation_tolerance_rad;
  request.constraints.approach_direction_object = goal.place_approach_direction_object;
  request.constraints.has_estimated_object_height = goal.has_estimated_object_height;
  request.constraints.estimated_object_height_m = goal.estimated_object_height_m;
  request.constraints.has_observed_rectangle = goal.has_observed_rectangle;
  request.constraints.rectangle_long_side_m = goal.rectangle_long_side_m;
  request.constraints.rectangle_short_side_m = goal.rectangle_short_side_m;
  request.constraints.grasp_width_min_mm = goal.grasp_width_min_mm;
  request.constraints.grasp_width_max_mm = goal.grasp_width_max_mm;
  request.constraints.orientation_symmetry_order = pick ?
    orientation_symmetry(goal.orientation_symmetry_order) : MotionOrientationSymmetry::UNIQUE;
  request.constraints.pick_approach_axis = goal.pick_approach_translation_direction ==
    ExecuteTask::Goal::PICK_APPROACH_PLANNING_FRAME_VERTICAL ?
    PickApproachAxis::PLANNING_FRAME_VERTICAL : PickApproachAxis::TCP_INSERTION_AXIS;
  request.constraints.place_orientation_constraint = static_cast<PlaceOrientationConstraint>(
    goal.place_orientation_constraint);
  request.vision_acquisition_id = goal.vision_acquisition_id;
  request.vision_candidate_id = goal.vision_candidate_id;
  request.material_candidates = goal.material_candidates;
  request.has_non_target_occupancy = goal.has_non_target_occupancy;
  request.non_target_occupancy = goal.non_target_occupancy;
  request.has_unobserved_yaw_fallback = goal.has_unobserved_yaw_fallback;
  request.unobserved_yaw_fallback_rad = goal.unobserved_yaw_fallback_rad;
  request.has_place_tool_orientation_preference = goal.has_place_tool_orientation_preference;
  request.place_tool_orientation_preference = goal.place_tool_orientation_preference;
  return request;
}

MotionPlanningResult TaskExecutionSupport::plan_motion_request(
  const MotionPortRequest & request, std::uint8_t phase)
{
  MotionPlanningResult result;
  result.plan.execution_id = request.execution_id;
  result.plan.movement_id = request.movement_id;
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {
      result.result_code = interrupted;
      return result;
    }
  }
  if (!motion_port_) {
    result.result_code = Result::TASK_RESULT_BACKEND_UNAVAILABLE;
    return result;
  }
  if (phase_callback_) {phase_callback_(phase, 0, 0, 0);}
  {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    if (!active_movement_id_.empty() &&
      (active_execution_id_.uuid != request.execution_id.uuid ||
      active_movement_id_ != request.movement_id))
    {
      result.result_code = Result::TASK_RESULT_BUSY;
      return result;
    }
    active_execution_id_ = request.execution_id;
    active_movement_id_ = request.movement_id;
  }
  try {
    result = motion_port_->planMotion(request);
  } catch (...) {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_movement_id_.clear();
    active_execution_id_.uuid.fill(0U);
    throw;
  }
  if (result.result_code != Result::TASK_RESULT_SUCCESS) {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_movement_id_.clear();
    active_execution_id_.uuid.fill(0U);
  }
  return result;
}

MotionPortResult TaskExecutionSupport::execute_planned_motion(
  const MotionPlan & plan, bool completes_operation)
{
  MotionPortResult result;
  result.execution_id = plan.execution_id;
  result.movement_id = plan.movement_id;
  result.result_code = Result::TASK_RESULT_SUCCESS;
  if (!motion_port_) {
    result.result_code = Result::TASK_RESULT_BACKEND_UNAVAILABLE;
    return result;
  }
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {
      result.result_code = interrupted;
      try {
        (void)motion_port_->cancelMotion(plan.execution_id, plan.movement_id);
      } catch (...) {
        // Preserve the interruption result; cancellation cleanup is best-effort.
      }
      completes_operation = true;
    }
  }
  if (result.result_code == Result::TASK_RESULT_SUCCESS) {
    try {
      result = motion_port_->executePlannedMotion(plan);
    } catch (...) {
      std::lock_guard<std::mutex> lock(movement_mutex_);
      active_movement_id_.clear();
      active_execution_id_.uuid.fill(0U);
      throw;
    }
  }
  if (completes_operation || result.result_code != Result::TASK_RESULT_SUCCESS) {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_execution_id_.uuid.fill(0U);
    active_movement_id_.clear();
  }
  return result;
}

std::uint8_t TaskExecutionSupport::execute_legacy_movement(std::uint8_t task_type)
{
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {return interrupted;}
  }
  if (!motion_port_) {return Result::TASK_RESULT_BACKEND_UNAVAILABLE;}
  const auto movement_id = next_movement_id("task");
  const auto execution_id = execution_uuid(execution_id_);
  {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_execution_id_ = execution_id;
    active_movement_id_ = movement_id;
  }
  std::uint8_t result = Result::TASK_RESULT_BACKEND_ERROR;
  try {result = motion_port_->executeLegacyMovement(task_type, execution_id_, movement_id);}
  catch (...) {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_movement_id_.clear();
    active_execution_id_.uuid.fill(0U);
    throw;
  }
  {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_movement_id_.clear();
    active_execution_id_.uuid.fill(0U);
  }
  return result;
}

std::uint8_t TaskExecutionSupport::execute_place_recovery_joint_target()
{
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {return interrupted;}
  }
  if (!motion_port_) {return Result::TASK_RESULT_BACKEND_UNAVAILABLE;}
  const auto movement_id = next_movement_id("joint");
  const auto execution_id = execution_uuid(execution_id_);
  {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_execution_id_ = execution_id;
    active_movement_id_ = movement_id;
  }
  std::uint8_t result = Result::TASK_RESULT_BACKEND_ERROR;
  try {result = motion_port_->executePlaceRecoveryJointTarget(execution_id, movement_id);}
  catch (...) {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_movement_id_.clear();
    active_execution_id_.uuid.fill(0U);
    throw;
  }
  {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    active_movement_id_.clear();
    active_execution_id_.uuid.fill(0U);
  }
  return result;
}

MotionCancelResult TaskExecutionSupport::cancel_active_motion(
  const unique_identifier_msgs::msg::UUID & requested_execution_id)
{
  unique_identifier_msgs::msg::UUID execution_id;
  std::string movement_id;
  {
    std::lock_guard<std::mutex> lock(movement_mutex_);
    execution_id = active_execution_id_;
    movement_id = active_movement_id_;
    if (execution_id.uuid != requested_execution_id.uuid) {
      return {false, false, arm_cell_interfaces::msg::MotionStatus()};
    }
  }
  if (!motion_port_ || movement_id.empty()) {
    return {false, false, arm_cell_interfaces::msg::MotionStatus()};
  }
  return motion_port_->cancelMotion(execution_id, movement_id);
}

std::string TaskExecutionSupport::next_movement_id(const char * kind)
{
  std::lock_guard<std::mutex> lock(movement_mutex_);
  return execution_id_ + ":" + kind + ":" + std::to_string(++movement_sequence_);
}

std::uint8_t TaskExecutionSupport::execute_gripper_close(std::uint64_t & baseline_sequence)
{
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {return interrupted;}
  }
  if (phase_callback_) {phase_callback_(Phase::TASK_PHASE_GRIPPER, 0, 0, 0);}
  if (!gripper_port_) {return Result::TASK_RESULT_GRASP_FAILURE;}
  baseline_sequence = holding_sequence();
  const auto result = gripper_port_->close();
  RCLCPP_DEBUG(rclcpp::get_logger("task_executor"), "gripper close command issued: result=%u", result);
  if (result != Result::TASK_RESULT_SUCCESS && gripper_port_->active()) {gripper_port_->stop();}
  return map_object_operation_result(result);
}

std::uint8_t TaskExecutionSupport::execute_gripper_open(
  std::uint64_t & baseline_sequence, std::uint8_t release_attempt,
  std::uint8_t retry_budget, std::uint8_t last_failure)
{
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {return interrupted;}
  }
  if (phase_callback_) {
    phase_callback_(Phase::TASK_PHASE_GRIPPER, release_attempt, retry_budget, last_failure);
  }
  if (!gripper_port_) {return Result::TASK_RESULT_GRASP_FAILURE;}
  baseline_sequence = holding_sequence();
  const auto result = gripper_port_->open();
  RCLCPP_DEBUG(rclcpp::get_logger("task_executor"), "gripper open command issued: result=%u", result);
  if (result != Result::TASK_RESULT_SUCCESS && gripper_port_->active()) {gripper_port_->stop();}
  return map_object_operation_result(result);
}

std::uint8_t TaskExecutionSupport::wait_interruptibly(std::chrono::milliseconds duration) const
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  constexpr auto poll_interval = std::chrono::milliseconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    if (interruption_checker_) {
      const auto interrupted = interruption_checker_();
      if (interrupted != Result::TASK_RESULT_SUCCESS) {return interrupted;}
    }
    const auto wake_at = std::min(deadline, std::chrono::steady_clock::now() + poll_interval);
    std::this_thread::sleep_until(wake_at);
  }
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted != Result::TASK_RESULT_SUCCESS) {return interrupted;}
  }
  return Result::TASK_RESULT_SUCCESS;
}

std::uint64_t TaskExecutionSupport::holding_sequence() const
{return gripper_port_ ? gripper_port_->holding().sequence : 0;}

HoldingWaitResult TaskExecutionSupport::fresh_holding(
  HoldingState expected, std::uint64_t baseline_sequence) const
{
  if (!gripper_port_) {return HoldingWaitResult::BACKEND_UNAVAILABLE;}
  bool saw_unknown = false;
  const auto deadline = HoldingObservation::Clock::now() + holding_confirmation_timeout_;
  while (HoldingObservation::Clock::now() < deadline) {
    if (interruption_checker_) {
      const auto interrupted = interruption_checker_();
      if (interrupted == Result::TASK_RESULT_CANCELED) {return HoldingWaitResult::CANCELED;}
      if (interrupted == Result::TASK_RESULT_SAFETY_PREEMPTED) {
        return HoldingWaitResult::SAFETY_PREEMPTED;
      }
      if (interrupted == Result::TASK_RESULT_BACKEND_UNAVAILABLE) {
        return HoldingWaitResult::BACKEND_UNAVAILABLE;
      }
    }
    const auto observation = gripper_port_->holding();
    if (observation.sequence > baseline_sequence && observation.state == expected &&
      observation.fresh())
    {
      return HoldingWaitResult::CONFIRMED;
    }
    saw_unknown = saw_unknown || (observation.sequence > baseline_sequence &&
      observation.state == HoldingState::UNKNOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (interruption_checker_) {
    const auto interrupted = interruption_checker_();
    if (interrupted == Result::TASK_RESULT_CANCELED) {return HoldingWaitResult::CANCELED;}
    if (interrupted == Result::TASK_RESULT_SAFETY_PREEMPTED) {
      return HoldingWaitResult::SAFETY_PREEMPTED;
    }
    if (interrupted == Result::TASK_RESULT_BACKEND_UNAVAILABLE) {
      return HoldingWaitResult::BACKEND_UNAVAILABLE;
    }
  }
  return saw_unknown ? HoldingWaitResult::UNKNOWN : HoldingWaitResult::TIMEOUT;
}

std::uint8_t TaskExecutionSupport::map_object_operation_result(std::uint8_t result)
{
  if (result == Result::TASK_RESULT_SUCCESS || result == Result::TASK_RESULT_BACKEND_UNAVAILABLE ||
    result == Result::TASK_RESULT_CANCELED || result == Result::TASK_RESULT_SAFETY_PREEMPTED)
  {
    return result;
  }
  return Result::TASK_RESULT_GRASP_FAILURE;
}

}  // namespace arm_cell_task_execution
