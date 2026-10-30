#include "arm_cell_orchestration_bt/cycle_coordinator.hpp"

#include <cmath>
#include <utility>

namespace arm_cell_orchestration_bt
{

namespace
{
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using DetectResult = arm_cell_interfaces::msg::DetectTargetResultCode;
using MotionType = arm_cell_interfaces::msg::MotionTaskType;
using MotionPhase = arm_cell_interfaces::msg::MotionTaskPhase;

CycleCoordinator::MissionExitReason exit_reason(uint8_t value)
{
  CycleCoordinator::MissionExitReason result;
  result.value = value;
  return result;
}

CycleCoordinator::MotionTaskResultCode motion_result_code(uint8_t value)
{
  CycleCoordinator::MotionTaskResultCode result;
  result.value = value;
  return result;
}

bool valid_place_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & position = pose.pose.position;
  const auto & orientation = pose.pose.orientation;
  const auto orientation_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return pose.header.frame_id == "base_link" &&
         std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
         std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
         std::isfinite(orientation_norm_squared) && orientation_norm_squared > 0.0;
}

bool valid_orientation_constraint(PlaceOrientationConstraint constraint)
{
  switch (constraint) {
    case PlaceOrientationConstraint::FIXED:
    case PlaceOrientationConstraint::BOUNDED:
    case PlaceOrientationConstraint::FREE:
      return true;
  }
  return false;
}

bool valid_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const auto norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
         std::isfinite(norm_squared) && norm_squared > 1e-12;
}

bool valid_unit_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const auto norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
         std::isfinite(norm_squared) && std::abs(norm_squared - 1.0) <= 1e-6;
}

bool valid_unit_direction(const geometry_msgs::msg::Vector3 & direction)
{
  const auto norm_squared = direction.x * direction.x + direction.y * direction.y +
    direction.z * direction.z;
  return std::isfinite(direction.x) && std::isfinite(direction.y) &&
         std::isfinite(direction.z) && std::isfinite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= 1e-6;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  const auto norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  const auto x = orientation.x / norm;
  const auto y = orientation.y / norm;
  const auto z = orientation.z / norm;
  const auto w = orientation.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw / 2.0);
  orientation.w = std::cos(yaw / 2.0);
  return orientation;
}
}

CycleCoordinator::CycleCoordinator(Callbacks callbacks)
: callbacks_(std::move(callbacks))
{
}

void CycleCoordinator::set_cancel_requested(bool requested)
{
  cancel_requested_.store(requested);
}

bool CycleCoordinator::normal_motion_allowed() const
{
  if (!callbacks_.read_safety) {
    return false;
  }
  return normal_motion_allowed(callbacks_.read_safety());
}

bool CycleCoordinator::normal_motion_allowed(const SafetyState & state) const
{
  return state.valid && state.required_inputs_fresh &&
         state.motion_capability.value == MotionCapability::MOTION_NORMAL;
}

CycleCoordinator::Result CycleCoordinator::interrupted(
  MissionExitReason reason, const std::string & detail)
{
  bool send_cancel = false;
  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    if (!cancellation_sent_) {
      cancellation_sent_ = true;
      send_cancel = true;
    }
  }
  if (send_cancel && callbacks_.cancel_motion) {
    callbacks_.cancel_motion();
  }
  return Result{reason, detail};
}

CycleCoordinator::Result CycleCoordinator::safety_preempted(
  const SafetyState & entry_safety,
  std::optional<MotionFailureTrace> motion_failure)
{
  auto result = interrupted(
    exit_reason(MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED),
    "Safety preempted the mission");
  const auto recovery_entry = callbacks_.read_recovery_entry ? callbacks_.read_recovery_entry() :
    std::nullopt;
  coordinate_recovery(recovery_entry.value_or(entry_safety));
  result.motion_failure = std::move(motion_failure);
  return result;
}

void CycleCoordinator::coordinate_recovery(const SafetyState & entry_safety)
{
  if (!callbacks_.wait_for_recovery) {
    return;
  }
  if (callbacks_.publish_phase) {
    callbacks_.publish_phase(MissionPhase::MISSION_PHASE_WAITING_RECOVERY);
  }

  bool recovery_authorization_seen = false;
  std::optional<uint8_t> episode_stop_severity;
  if (entry_safety.valid && entry_safety.required_inputs_fresh &&
    entry_safety.selected_stop_mode.value <= StopMode::STOP_MODE_EMERGENCY)
  {
    episode_stop_severity = entry_safety.selected_stop_mode.value;
  }
  while (true) {
    const auto observation = callbacks_.wait_for_recovery();
    if (!observation.safety.valid || !observation.safety.required_inputs_fresh) {
      continue;
    }

    const auto selected_stop_severity = observation.safety.selected_stop_mode.value;
    if (selected_stop_severity > StopMode::STOP_MODE_EMERGENCY) {
      continue;
    }
    if (!episode_stop_severity) {
      episode_stop_severity = selected_stop_severity;
    } else if (selected_stop_severity > *episode_stop_severity) {
      return;
    }
    if (selected_stop_severity == StopMode::STOP_MODE_EMERGENCY) {
      return;
    }

    const auto recovery_authorized =
      observation.safety.motion_capability.value == MotionCapability::MOTION_RECOVERY_ONLY;
    if (recovery_authorization_seen && !recovery_authorized) {
      return;
    }
    if (!recovery_authorized) {
      continue;
    }

    recovery_authorization_seen = true;
    if (observation.motion.execution_state != MotionStatus::MOTION_STATE_STOPPED ||
      observation.motion.execution_active || !observation.motion.backend_inactivity_confirmed)
    {
      continue;
    }
    if (observation.motion.holding_state != MotionStatus::HOLDING_HELD &&
      observation.motion.holding_state != MotionStatus::HOLDING_RELEASED)
    {
      return;
    }

    if (callbacks_.publish_phase) {
      callbacks_.publish_phase(MissionPhase::MISSION_PHASE_RETRACTING);
    }
    auto goal = motion_goal(MotionType::TASK_TYPE_RETRACT);
    const auto result = callbacks_.execute_task(goal);
    if (callbacks_.record_recovery_result) {
      callbacks_.record_recovery_result(result);
    }
    return;
  }
}

CycleCoordinator::Result CycleCoordinator::commit_non_safety_terminal(
  MissionExitReason reason, const std::string & detail,
  std::optional<MotionFailureTrace> motion_failure)
{
  const auto safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
  if (!normal_motion_allowed(safety)) {
    return safety_preempted(safety, std::move(motion_failure));
  }
  if (reason.value != MissionExitReason::MISSION_EXIT_MOTION_ERROR) {
    motion_failure.reset();
  }
  return Result{reason, detail, std::move(motion_failure)};
}

CycleCoordinator::MotionResult CycleCoordinator::execute_motion(
  const ExecuteTask::Goal & goal, uint8_t phase)
{
  const auto with_task_type = [&goal](MotionResult result) {
      result.task_type = goal.task_type.value;
      return result;
    };
  if (callbacks_.publish_phase) {
    callbacks_.publish_phase(phase);
  }
  if (cancel_requested_.load()) {
    return with_task_type(
      MotionResult{
        motion_result_code(MotionTaskResultCode::TASK_RESULT_CANCELED),
        "Caller cancellation requested"});
  }
  if (!normal_motion_allowed()) {
    return with_task_type(
      MotionResult{
        motion_result_code(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED),
        "Safety capability no longer permits normal motion"});
  }
  if (!callbacks_.execute_task) {
    return with_task_type(
      MotionResult{
        motion_result_code(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE),
        "Motion client unavailable"});
  }
  return with_task_type(callbacks_.execute_task(goal));
}

CycleCoordinator::MotionFailureTrace CycleCoordinator::motion_failure_trace(
  const MotionResult & result)
{
  return MotionFailureTrace{
    result.task_type, result.code, result.diagnostic_detail, result.motion_execution_id};
}

CycleCoordinator::MissionExitReason CycleCoordinator::motion_failure_exit_reason(
  const MotionResult & result) const
{
  return result.code.value == MotionTaskResultCode::TASK_RESULT_CANCELED &&
         cancel_requested_.load() ?
         exit_reason(MissionExitReason::MISSION_EXIT_CANCELED) :
         exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR);
}

CycleCoordinator::ExecuteTask::Goal CycleCoordinator::motion_goal(uint8_t task_type)
{
  ExecuteTask::Goal goal;
  goal.task_type.value = task_type;
  return goal;
}

CycleCoordinator::ExecuteTask::Goal CycleCoordinator::pick_goal(
  const DetectTarget::Response & response, const MissionRecipe & recipe)
{
  auto goal = motion_goal(arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK);
  goal.target_pose = response.target_pose;
  goal.has_target_pose = response.has_target_pose;
  const auto yaw = response.has_target_yaw ?
    yaw_from_quaternion(response.target_pose.pose.orientation) : recipe.grasp_yaw_rad;
  goal.target_pose.pose.orientation = quaternion_from_yaw(yaw);
  goal.estimated_object_height_m = response.estimated_height_m;
  goal.has_estimated_object_height = response.has_estimated_height;
  goal.grasp_width_mm = recipe.grasp_width_mm;
  goal.has_grasp_width = recipe.has_grasp_width;
  return goal;
}

CycleCoordinator::ExecuteTask::Goal CycleCoordinator::place_goal(
  const MissionRecipe & recipe)
{
  auto goal = motion_goal(arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE);
  goal.target_pose = recipe.place_pose;
  goal.has_target_pose = recipe.has_place_pose;
  goal.place_position_tolerance_m = recipe.place_position_tolerance_m;
  goal.place_orientation_tolerance_rad = recipe.place_orientation_tolerance_rad;
  switch (recipe.place_orientation_constraint) {
    case PlaceOrientationConstraint::FIXED:
      goal.place_orientation_constraint = ExecuteTask::Goal::PLACE_ORIENTATION_FIXED;
      break;
    case PlaceOrientationConstraint::BOUNDED:
      goal.place_orientation_constraint = ExecuteTask::Goal::PLACE_ORIENTATION_BOUNDED;
      break;
    case PlaceOrientationConstraint::FREE:
      goal.place_orientation_constraint = ExecuteTask::Goal::PLACE_ORIENTATION_FREE;
      break;
  }
  goal.has_place_tool_orientation_preference =
    recipe.has_place_tool_orientation_preference;
  goal.place_tool_orientation_preference = recipe.place_tool_orientation_preference;
  goal.place_approach_direction_object = recipe.place_approach_direction_object;
  goal.place_approach_distance_m = recipe.place_approach_distance_m;
  goal.place_retract_distance_m = recipe.retract_distance_m;
  return goal;
}

bool CycleCoordinator::successful(const MotionResult & result)
{
  return result.code.value == MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

CycleCoordinator::Result CycleCoordinator::execute(const std::string & target_id)
{
  {
    std::lock_guard<std::mutex> lock(cancel_mutex_);
    cancellation_sent_ = false;
  }

  const auto recipe = callbacks_.load_recipe ? callbacks_.load_recipe(target_id) :
    std::optional<MissionRecipe>{};
  if (!recipe || !recipe->has_grasp_width || !recipe->has_grasp_yaw || !recipe->has_place_pose ||
    !std::isfinite(recipe->grasp_width_mm) || !valid_place_pose(recipe->place_pose) ||
    !std::isfinite(recipe->place_position_tolerance_m) ||
    recipe->place_position_tolerance_m < 0.0 ||
    !std::isfinite(recipe->place_orientation_tolerance_rad) ||
    recipe->place_orientation_tolerance_rad < 0.0 ||
    !valid_orientation_constraint(recipe->place_orientation_constraint) ||
    (recipe->place_orientation_constraint == PlaceOrientationConstraint::FREE &&
    recipe->has_place_tool_orientation_preference) ||
    (recipe->has_place_tool_orientation_preference &&
    !valid_unit_quaternion(recipe->place_tool_orientation_preference)) ||
    !valid_unit_direction(recipe->place_approach_direction_object) ||
    !std::isfinite(recipe->place_approach_distance_m) ||
    recipe->place_approach_distance_m <= 0.0 ||
    !std::isfinite(recipe->retract_distance_m) || recipe->retract_distance_m <= 0.0 ||
    !std::isfinite(recipe->grasp_yaw_rad) ||
    recipe->grasp_width_mm < 0.0F || recipe->grasp_width_mm > 85.0F)
  {
    return commit_non_safety_terminal(
      exit_reason(MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR),
      "Mission recipe is missing a valid grasp width, grasp yaw, or PLACE pose");
  }

  while (true) {
    if (cancel_requested_.load()) {
      const auto result = commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
        "Caller canceled the mission");
      if (result.exit_reason.value == MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED) {
        return result;
      }
      return interrupted(result.exit_reason, result.diagnostic_detail);
    }
    const auto entry_safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
    if (!normal_motion_allowed(entry_safety)) {
      return safety_preempted(entry_safety);
    }

    auto home = execute_motion(
      motion_goal(MotionType::TASK_TYPE_GO_HOME),
      MissionPhase::MISSION_PHASE_ENSURING_HOME);
    if (!successful(home)) {
      const auto safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
      const auto failure = motion_failure_trace(home);
      if (home.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
        return safety_preempted(safety);
      }
      if (!normal_motion_allowed(safety)) {
        return safety_preempted(safety, failure);
      }
      return commit_non_safety_terminal(
        motion_failure_exit_reason(home),
        home.diagnostic_detail,
        failure);
    }
    const auto post_home_safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
    if (!normal_motion_allowed(post_home_safety)) {
      return safety_preempted(post_home_safety);
    }

    if (callbacks_.publish_phase) {
      callbacks_.publish_phase(MissionPhase::MISSION_PHASE_DETECTING_TARGET);
    }
    DetectTarget::Request request;
    request.target_id = target_id;
    const auto detected = callbacks_.detect_target ? callbacks_.detect_target(request) :
      DetectTarget::Response{};
    if (detected.result_code.value == DetectResult::DETECT_RESULT_OBJECT_NOT_FOUND) {
      const auto result = commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_DEPLETED),
        detected.diagnostic_detail);
      if (result.exit_reason.value == MissionExitReason::MISSION_EXIT_DEPLETED &&
        callbacks_.publish_phase)
      {
        callbacks_.publish_phase(MissionPhase::MISSION_PHASE_FINISHED);
      }
      return result;
    }
    if (detected.result_code.value != DetectResult::DETECT_RESULT_SUCCESS) {
      return commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
        detected.diagnostic_detail);
    }
    if (detected.has_target_yaw && !valid_quaternion(detected.target_pose.pose.orientation)) {
      return commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
        "Vision reported an invalid present yaw");
    }

    auto pick = execute_motion(
      pick_goal(detected, *recipe), MissionPhase::MISSION_PHASE_PICKING);
    if (!successful(pick)) {
      const auto safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
      const auto failure = motion_failure_trace(pick);
      if (pick.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
        return safety_preempted(safety);
      }
      if (!normal_motion_allowed(safety)) {
        return safety_preempted(safety, failure);
      }
      return commit_non_safety_terminal(
        motion_failure_exit_reason(pick),
        pick.diagnostic_detail,
        failure);
    }

    auto place = execute_motion(
      place_goal(*recipe), MissionPhase::MISSION_PHASE_PLACING);
    if (!successful(place)) {
      const auto safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
      const auto failure = motion_failure_trace(place);
      if (place.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
        return safety_preempted(safety);
      }
      if (!normal_motion_allowed(safety)) {
        return safety_preempted(safety, failure);
      }
      return commit_non_safety_terminal(
        motion_failure_exit_reason(place),
        place.diagnostic_detail,
        failure);
    }

    auto return_home = execute_motion(
      motion_goal(MotionType::TASK_TYPE_GO_HOME),
      MissionPhase::MISSION_PHASE_RETURNING_HOME);
    if (!successful(return_home)) {
      const auto safety = callbacks_.read_safety ? callbacks_.read_safety() : SafetyState{};
      const auto failure = motion_failure_trace(return_home);
      if (return_home.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
        return safety_preempted(safety);
      }
      if (!normal_motion_allowed(safety)) {
        return safety_preempted(safety, failure);
      }
      return commit_non_safety_terminal(
        motion_failure_exit_reason(return_home),
        return_home.diagnostic_detail,
        failure);
    }
  }
}

}  // namespace arm_cell_orchestration_bt

namespace arm_cell_orchestration_bt
{

void CancellationRegistry::register_execution(
  const std::string & key, CancelFn cancel_execution)
{
  CancelFn execution_callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & entry = entries_[key];
    entry.cancel_execution = std::move(cancel_execution);
    if (pending_caller_cancels_.count(key) != 0U) {
      execution_callback = entry.cancel_execution;
    }
  }
  if (execution_callback) {
    execution_callback();
  }
}

void CancellationRegistry::register_active_motion(
  const std::string & key, CancelFn cancel_motion)
{
  CancelFn motion_callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & entry = entries_[key];
    entry.cancel_motion = std::move(cancel_motion);
    if (pending_caller_cancels_.count(key) != 0U ||
      pending_motion_cancels_.erase(key) > 0U)
    {
      motion_callback = entry.cancel_motion;
    }
  }
  if (motion_callback) {
    motion_callback();
  }
}

void CancellationRegistry::request_caller_cancel(const std::string & key)
{
  CancelFn execution_callback;
  CancelFn motion_callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      pending_caller_cancels_.insert(key);
      return;
    }
    execution_callback = it->second.cancel_execution;
    motion_callback = it->second.cancel_motion;
    if (!execution_callback) {
      pending_caller_cancels_.insert(key);
    }
    if (!motion_callback) {
      pending_motion_cancels_.insert(key);
    }
  }
  if (execution_callback) {
    execution_callback();
  }
  if (motion_callback) {
    motion_callback();
  }
}

void CancellationRegistry::request_secondary_motion_cancel(const std::string & key)
{
  CancelFn motion_callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it == entries_.end() || !it->second.cancel_motion) {
      pending_motion_cancels_.insert(key);
      return;
    }
    motion_callback = it->second.cancel_motion;
  }
  motion_callback();
}

void CancellationRegistry::clear_active_motion(const std::string & key)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = entries_.find(key);
  if (it != entries_.end()) {
    it->second.cancel_motion = nullptr;
  }
}

void CancellationRegistry::erase_execution(const std::string & key)
{
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.erase(key);
  pending_caller_cancels_.erase(key);
  pending_motion_cancels_.erase(key);
}

}  // namespace arm_cell_orchestration_bt
