#include "arm_cell_motion/motion_core.hpp"

#include <iomanip>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_motion/motion_result_code_name.hpp"

namespace arm_cell_motion
{

namespace
{
std::string uuid_string(const UUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid.uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

UUID parse_uuid(const std::string & value)
{
  UUID uuid;
  std::size_t index = 0;
  int high = -1;
  for (const char ch : value) {
    int nibble = -1;
    if (ch >= '0' && ch <= '9') {nibble = ch - '0';} else if (ch >= 'a' && ch <= 'f') {
      nibble = ch - 'a' + 10;
    } else if (ch >= 'A' && ch <= 'F') {nibble = ch - 'A' + 10;}
    if (nibble < 0) {continue;}
    if (high < 0) {high = nibble; continue;}
    if (index < uuid.uuid.size()) {
      uuid.uuid[index++] = static_cast<std::uint8_t>((high << 4) | nibble);
    }
    high = -1;
  }
  return uuid;
}

void set_diagnostic(std::string * diagnostic_detail, const std::string & value)
{
  if (diagnostic_detail) {
    *diagnostic_detail = value;
  }
}

bool valid_motion_port_pose(const MotionPortRequest & request)
{
  if (!std::isfinite(request.constraints.motion_speed_fraction) ||
    request.constraints.motion_speed_fraction <= 0.0 ||
    request.constraints.motion_speed_fraction > 1.0)
  {
    return false;
  }
  const auto order = request.constraints.orientation_symmetry_order;
  if (order != MotionOrientationSymmetry::UNOBSERVABLE &&
    order != MotionOrientationSymmetry::UNIQUE &&
    order != MotionOrientationSymmetry::TWOFOLD &&
    order != MotionOrientationSymmetry::FOURFOLD)
  {
    return false;
  }
  const auto & pose = request.target_pose;
  if (pose.header.frame_id != "base_link" || !std::isfinite(pose.pose.position.x) ||
    !std::isfinite(pose.pose.position.y) || !std::isfinite(pose.pose.position.z))
  {
    return false;
  }
  if (order == MotionOrientationSymmetry::UNOBSERVABLE) {
    return request.has_unobserved_yaw_fallback &&
           std::isfinite(request.unobserved_yaw_fallback_rad);
  }
  const auto & q = pose.pose.orientation;
  const auto norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return !request.has_unobserved_yaw_fallback && request.unobserved_yaw_fallback_rad == 0.0 &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && std::isfinite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= 1e-6;
}

bool same_trajectory(
  const trajectory_msgs::msg::JointTrajectory & lhs,
  const trajectory_msgs::msg::JointTrajectory & rhs)
{
  if (lhs.joint_names != rhs.joint_names || lhs.points.size() != rhs.points.size()) {
    return false;
  }
  for (std::size_t index = 0; index < lhs.points.size(); ++index) {
    const auto & left = lhs.points[index];
    const auto & right = rhs.points[index];
    if (left.positions != right.positions || left.velocities != right.velocities ||
      left.accelerations != right.accelerations || left.effort != right.effort ||
      left.time_from_start.sec != right.time_from_start.sec ||
      left.time_from_start.nanosec != right.time_from_start.nanosec)
    {
      return false;
    }
  }
  return true;
}

uint8_t task_type_for_motion_segment(MotionSegmentKind segment)
{
  switch (segment) {
    case MotionSegmentKind::PICK_APPROACH:
    case MotionSegmentKind::PICK_DESCENT:
    case MotionSegmentKind::PICK_RETRACT:
      return MotionTaskType::TASK_TYPE_PICK;
    case MotionSegmentKind::PLACE_APPROACH:
    case MotionSegmentKind::PLACE_DESCENT:
    case MotionSegmentKind::PLACE_CLEARANCE:
    case MotionSegmentKind::PLACE_RETRACT:
    case MotionSegmentKind::DISTURB_APPROACH:
    case MotionSegmentKind::DISTURB_PUSH:
    case MotionSegmentKind::DISTURB_RETRACT:
      return MotionTaskType::TASK_TYPE_PLACE;
  }
  return MotionTaskType::TASK_TYPE_UNSPECIFIED;
}
}  // namespace

MotionCore::MotionCore(
  std::shared_ptr<MotionBackend> backend,
  std::chrono::milliseconds safety_state_timeout, MotionGeometry geometry,
  std::chrono::milliseconds holding_confirmation_timeout,
  std::function<Clock::time_point()> now)
: backend_(std::move(backend)),
  motion_execution_support_(std::make_shared<MotionExecutionSupport>(backend_, geometry)),
  safety_state_timeout_(safety_state_timeout), now_(std::move(now)), runtime_geometry_(geometry),
  holding_confirmation_timeout_(holding_confirmation_timeout)
{
  status_.execution_state = MotionStatus::MOTION_STATE_IDLE;
  status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
  status_.execution_active = false;
  status_.backend_inactivity_confirmed = true;
  motion_execution_support_->set_interruption_checker(
    [this]() {return current_interruption_result();});
}

void MotionCore::update_safety_state(
  const SafetyState & state, Clock::time_point receipt_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  safety_state_ = state;
  safety_state_receipt_time_ = receipt_time;
  has_safety_state_ = true;
}

MotionStatus MotionCore::status() const
{
  MotionStatus current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current = status_;
  }

  HoldingObservation observation;
  try {
    const auto gripper = backend_ ? backend_->gripper_port() : nullptr;
    if (gripper) {
      observation = gripper->holding();
    }
  } catch (...) {
    observation = HoldingObservation{};
  }
  if (!observation.fresh()) {
    current.holding_state = MotionStatus::HOLDING_UNKNOWN;
  } else if (observation.state == HoldingState::HELD) {
    current.holding_state = MotionStatus::HOLDING_HELD;
  } else if (observation.state == HoldingState::RELEASED) {
    current.holding_state = MotionStatus::HOLDING_RELEASED;
  } else {
    current.holding_state = MotionStatus::HOLDING_UNKNOWN;
  }
  if (current.execution_state == MotionStatus::MOTION_STATE_STOPPING && backend_) {
    current.diagnostic_detail = backend_->inactivity_diagnostic();
  }
  return current;
}

uint8_t MotionCore::accept_goal_impl(
  const UUID & goal_uuid, uint8_t task_type, Clock::time_point acceptance_time,
  bool refresh_pre_submit_time, bool sample_initial_time,
  std::string * diagnostic_detail, const MotionPortRequest * motion_request,
  const ExecuteTask::Goal * named_task, bool joint_target, const std::string & movement_id,
  NormalizedTrajectoryPlan * planning_output, bool retain_planned_operation,
  std::uint64_t * retained_plan_id_output)
{
  const auto goal_id = uuid_string(goal_uuid);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string validation_diagnostic;
    // Existing Motion ownership takes priority over new request validity: a
    // conflicting request must not reveal a different admission reason on
    // each resend, and it must leave the owner untouched.
    const bool backend_active = backend_->execution_active();
    const bool inactivity_confirmed = backend_->backend_inactivity_confirmed();
    if (backend_active ||
      (!inactivity_confirmed && status_.execution_state != MotionStatus::MOTION_STATE_IDLE) ||
      (goal_submission_pending_ && pending_goal_uuid_.uuid != goal_uuid.uuid) ||
      motion_execution_in_progress_ || operator_gripper_open_in_progress_ ||
      place_waypoint_generation_in_progress_ ||
      status_.has_active_execution ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING)
    {
      status_.execution_active = backend_active;
      status_.backend_inactivity_confirmed = inactivity_confirmed;
      if ((backend_active ||
        (!inactivity_confirmed && status_.execution_state != MotionStatus::MOTION_STATE_IDLE)) &&
        status_.execution_state != MotionStatus::MOTION_STATE_STOPPING)
      {
        status_.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
      }
      set_diagnostic(
        diagnostic_detail,
        "Motion execution reservation or backend shutdown is still active");
      return MotionTaskResultCode::TASK_RESULT_BUSY;
    }
    const auto initial_validation_time = sample_initial_time ? now_() : acceptance_time;
    const auto validation_result = validate_goal_locked(
      task_type, initial_validation_time, &validation_diagnostic);
    if (validation_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      set_diagnostic(
        diagnostic_detail,
        validation_diagnostic.empty() ?
        "validation/permission rejection: result_code=" + std::to_string(validation_result) :
        validation_diagnostic);
      return validation_result;
    }
    const bool is_reserved_goal = goal_submission_pending_ &&
      pending_goal_uuid_.uuid == goal_uuid.uuid;
    if (!is_reserved_goal) {
      goal_submission_pending_ = true;
      pending_goal_uuid_ = goal_uuid;
      active_movement_id_ = motion_request ? motion_request->movement_id : movement_id;
      active_tuning_snapshot_ = runtime_tuning_;
      active_geometry_snapshot_ = runtime_geometry_;
      active_holding_confirmation_timeout_ = holding_confirmation_timeout_;
      stop_requested_ = false;
      interruption_reason_ = InterruptionReason::NONE;
    }
  }

  const bool backend_available = backend_->available();
  if (!backend_available) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = interruption_result_locked(
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
    goal_submission_pending_ = false;
    pending_goal_uuid_ = UUID{};
    active_movement_id_.clear();
    if (stop_requested_) {
      status_.execution_state = MotionStatus::MOTION_STATE_STOPPING;
      stop_requested_ = false;
    }
    clear_execution_interruption_locked();
    if (result == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      set_diagnostic(diagnostic_detail, "caller cancellation");
    } else if (result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      set_diagnostic(diagnostic_detail, "Safety preemption");
    } else {
      set_diagnostic(diagnostic_detail, "backend unavailable");
    }
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Capability authorization is not a lease. Re-evaluate immediately before
    // handing the trajectory-producing request to the backend.
    const auto pre_submit_time = refresh_pre_submit_time ? Clock::now() : acceptance_time;
    const auto pre_submit_validation = validate_goal_locked(
      task_type, pre_submit_time, nullptr);
    if (pre_submit_validation != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      active_movement_id_.clear();
      if (stop_requested_) {
        status_.execution_state = MotionStatus::MOTION_STATE_STOPPING;
        stop_requested_ = false;
      }
      clear_execution_interruption_locked();
      set_diagnostic(diagnostic_detail, "permission rejection before backend submit");
      return MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED;
    }
    const auto interruption_result = interruption_result_locked(
      MotionTaskResultCode::TASK_RESULT_SUCCESS);
    if (interruption_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      active_movement_id_.clear();
      clear_execution_interruption_locked();
      set_diagnostic(
        diagnostic_detail,
        interruption_result == MotionTaskResultCode::TASK_RESULT_CANCELED ?
        "caller cancellation" : "Safety preemption");
      return interruption_result;
    }
    const MotionEnvelope requested_envelope{
      safety_state_.max_velocity_scale, safety_state_.max_acceleration_scale, true};
    if (!backend_->set_motion_envelope(requested_envelope)) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      active_movement_id_.clear();
      set_diagnostic(diagnostic_detail, "backend could not apply current Safety envelope");
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    // Parameter updates serialize on this mutex. Once the goal becomes active,
    // the executor and backend retain this one immutable tuning snapshot.
    if (!backend_->apply_runtime_tuning(active_tuning_snapshot_)) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      active_movement_id_.clear();
      set_diagnostic(diagnostic_detail, "backend rejected the Motion tuning snapshot");
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    motion_execution_support_->set_runtime_geometry(active_geometry_snapshot_);
    const auto applied_envelope = backend_->applied_motion_envelope();
    if (!applied_envelope.valid ||
      !std::isfinite(applied_envelope.max_velocity_scale) ||
      !std::isfinite(applied_envelope.max_acceleration_scale) ||
      applied_envelope.max_velocity_scale > requested_envelope.max_velocity_scale ||
      applied_envelope.max_acceleration_scale > requested_envelope.max_acceleration_scale)
    {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      active_movement_id_.clear();
      set_diagnostic(diagnostic_detail, "backend did not attest the current Safety envelope");
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    status_.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
    status_.active_execution_id = goal_uuid;
    status_.has_active_execution = true;
    status_.active_task_type.value = task_type;
    active_movement_id_ = motion_request ? motion_request->movement_id : movement_id;
    // The backend has not received the task yet; the accepted active execution
    // reservation is what Safety observes while this single movement runs.
    status_.execution_active = false;
    status_.backend_inactivity_confirmed = true;
    status_.applied_velocity_scale = applied_envelope.max_velocity_scale;
    status_.applied_acceleration_scale = applied_envelope.max_acceleration_scale;
    status_.applied_envelope_valid = true;
    motion_execution_in_progress_ = true;
  }

  uint8_t execution_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  {
    try {
      execution_result = motion_request ? (planning_output ?
        motion_execution_support_->plan_motion(*motion_request, *planning_output) :
        motion_execution_support_->execute_motion(*motion_request)) :
        named_task ? motion_execution_support_->execute_task(*named_task, goal_id) :
        joint_target ? motion_execution_support_->execute_joint_target() :
        MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    } catch (...) {
      throw;
    }
  }
  if (execution_result == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
    planning_output && retain_planned_operation)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto interrupted = interruption_result_locked(execution_result);
    if (interrupted == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      const auto plan_id = next_plan_id_++;
      retained_planned_operations_[plan_id] = *planning_output;
      if (retained_plan_id_output) {*retained_plan_id_output = plan_id;}
      motion_execution_in_progress_ = false;
      return execution_result;
    }
    execution_result = interrupted;
  }
  if (execution_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
                 "%s returned terminal failure result_code=%s",
                 "MotionPort",
                 motion_result_code_name(execution_result));
    std::lock_guard<std::mutex> lock(mutex_);
    const auto final_result = interruption_result_locked(execution_result);
    const auto failed_backend_envelope = backend_->applied_motion_envelope();
    status_.execution_active = backend_->execution_active();
    status_.backend_inactivity_confirmed = backend_->backend_inactivity_confirmed();
    const bool planning_failed_without_execution = planning_output != nullptr &&
      !status_.execution_active &&
      (final_result == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
      final_result == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
      final_result == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED);
    const bool backend_still_active = status_.execution_active ||
      (!status_.backend_inactivity_confirmed && !planning_failed_without_execution);
    const bool joint_progress_stalled = execution_result ==
      MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED;
    const bool stop_pending = stop_requested_ || joint_progress_stalled ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING;
    status_.execution_state = stop_pending ?
      (!backend_still_active ? MotionStatus::MOTION_STATE_STOPPED :
      MotionStatus::MOTION_STATE_STOPPING) :
      (backend_still_active ? MotionStatus::MOTION_STATE_EXECUTING :
      MotionStatus::MOTION_STATE_IDLE);
    // The Action is terminal once its task executor returns. Keep STOPPING
    // observable while physical feedback is unresolved, without claiming that
    // the old Action or trajectory producer still owns Motion.
    status_.has_active_execution = stop_pending ? status_.execution_active :
      backend_still_active;
    status_.applied_envelope_valid = failed_backend_envelope.valid && status_.execution_active;
    status_.applied_velocity_scale = failed_backend_envelope.max_velocity_scale;
    status_.applied_acceleration_scale = failed_backend_envelope.max_acceleration_scale;
    if (!status_.has_active_execution) {
      status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
    }
    goal_submission_pending_ = false;
    pending_goal_uuid_ = UUID{};
    motion_execution_in_progress_ = false;
    active_movement_id_.clear();
    if (stop_requested_) {
      status_.execution_state = MotionStatus::MOTION_STATE_STOPPING;
      stop_requested_ = false;
    }
    clear_execution_interruption_locked();
    if (final_result == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      set_diagnostic(diagnostic_detail, "caller cancellation");
    } else if (final_result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      set_diagnostic(diagnostic_detail, "Safety preemption");
    } else if (final_result == MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE) {
      set_diagnostic(diagnostic_detail, "backend unavailable");
    } else {
      set_diagnostic(
        diagnostic_detail,
        std::string("backend operation failed: ") + motion_result_code_name(execution_result));
    }
    return final_result;
  }

  const bool execution_active = backend_->execution_active();
  const bool inactivity_confirmed = backend_->backend_inactivity_confirmed();
  uint8_t final_result = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    final_result = interruption_result_locked(final_result);
    const bool stop_active = stop_requested_ ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING;
    status_.execution_state = stop_active ?
      (!execution_active && inactivity_confirmed ? MotionStatus::MOTION_STATE_STOPPED :
      MotionStatus::MOTION_STATE_STOPPING) :
      (execution_active ? MotionStatus::MOTION_STATE_EXECUTING : MotionStatus::MOTION_STATE_IDLE);
    status_.active_execution_id = goal_uuid;
    status_.has_active_execution = execution_active;
    status_.active_task_type.value = task_type;
    status_.execution_active = execution_active;
    status_.backend_inactivity_confirmed = inactivity_confirmed;
    const auto applied_envelope = backend_->applied_motion_envelope();
    status_.applied_envelope_valid = applied_envelope.valid;
    status_.applied_velocity_scale = applied_envelope.max_velocity_scale;
    status_.applied_acceleration_scale = applied_envelope.max_acceleration_scale;
    goal_submission_pending_ = false;
    pending_goal_uuid_ = UUID{};
    motion_execution_in_progress_ = false;
    active_movement_id_.clear();
    if (!status_.has_active_execution) {
      status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
    }
    stop_requested_ = false;
    clear_execution_interruption_locked();
  }
  return final_result;
}

bool MotionCore::request_cancel(const UUID & goal_uuid)
{
  bool should_cancel_backend = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool matches_pending = goal_submission_pending_ &&
      pending_goal_uuid_.uuid == goal_uuid.uuid;
    const bool matches_active = status_.has_active_execution &&
      status_.active_execution_id.uuid == goal_uuid.uuid;
    if (!matches_pending && !matches_active) {
      return false;
    }
    if (interruption_reason_ != InterruptionReason::SAFETY_PREEMPTED) {
      interruption_reason_ = InterruptionReason::CALLER_CANCELED;
      should_cancel_backend = true;
    }
  }
  if (should_cancel_backend) {
    const auto gripper_port = backend_->gripper_port();
    if (gripper_port && gripper_port->active()) {
      gripper_port->stop();
    }
    backend_->cancel();
  }
  return true;
}

uint8_t MotionCore::current_interruption_result() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return interruption_result_locked(MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

uint8_t MotionCore::interruption_result_locked(uint8_t execution_result) const
{
  if (interruption_reason_ == InterruptionReason::SAFETY_PREEMPTED) {
    return MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
  }
  if (interruption_reason_ == InterruptionReason::CALLER_CANCELED) {
    return MotionTaskResultCode::TASK_RESULT_CANCELED;
  }
  return execution_result;
}

void MotionCore::clear_execution_interruption_locked()
{
  interruption_reason_ = InterruptionReason::NONE;
}

bool MotionCore::request_stop(const UUID & request_uuid)
{
  return request_stop(
    request_uuid, arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
}

bool MotionCore::request_stop(const UUID & request_uuid, uint8_t stop_mode)
{
  std::lock_guard<std::mutex> dispatch_lock(stop_dispatch_mutex_);
  bool dispatch_required = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.has_active_execution && !goal_submission_pending_ &&
      !operator_gripper_open_in_progress_ &&
      status_.execution_state != MotionStatus::MOTION_STATE_STOPPING)
    {
      return false;
    }
    if (stop_mode != arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED &&
      stop_mode != arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE &&
      stop_mode != arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY)
    {
      return false;
    }
    const bool stop_already_in_progress =
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING;
    status_.last_stop_request_id = request_uuid;
    status_.has_last_stop_request = true;
    if (stop_already_in_progress && stop_mode <= requested_stop_mode_) {
      dispatch_required = false;
    } else {
      status_.execution_state = MotionStatus::MOTION_STATE_STOPPING;
      stop_requested_ = true;
      requested_stop_mode_ = stop_mode;
      interruption_reason_ = InterruptionReason::SAFETY_PREEMPTED;
      dispatch_required = true;
    }
  }
  if (!dispatch_required) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "Motion retained the active stop procedure for a duplicate or lower-severity request");
    return true;
  }
  const auto gripper_port = backend_->gripper_port();
  if (gripper_port && gripper_port->active()) {
    gripper_port->stop();
  }
  backend_->safety_preempt(stop_mode);
  std::optional<NormalizedTrajectoryPlan> canceled_plan;
  MotionPlan public_plan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_execution_in_progress_ && !retained_planned_operations_.empty()) {
      // Claim terminalization under the same lock used by executePlannedMotion.
      motion_execution_in_progress_ = true;
      const auto & entry = *retained_planned_operations_.begin();
      canceled_plan = entry.second;
      public_plan.plan_id = entry.first;
      public_plan.execution_id = status_.active_execution_id;
      public_plan.movement_id = entry.second.request.movement_id;
      public_plan.trajectory = entry.second.trajectory;
      public_plan.start_state = entry.second.start_state;
      public_plan.end_state = entry.second.end_state;
    }
  }
  if (canceled_plan) {
    (void)finish_planned_operation(
      public_plan, *canceled_plan, MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  }
  return true;
}

void MotionCore::refresh_stop_state()
{
  const bool execution_active = backend_->execution_active();
  const bool inactivity_confirmed = backend_->backend_inactivity_confirmed();
  const auto applied_envelope = backend_->applied_motion_envelope();
  std::lock_guard<std::mutex> lock(mutex_);
  status_.execution_active = execution_active;
  status_.backend_inactivity_confirmed = inactivity_confirmed;
  status_.applied_envelope_valid = applied_envelope.valid;
  status_.applied_velocity_scale = applied_envelope.max_velocity_scale;
  status_.applied_acceleration_scale = applied_envelope.max_acceleration_scale;
  if (
    status_.execution_state == MotionStatus::MOTION_STATE_EXECUTING &&
    !execution_active && inactivity_confirmed && !motion_execution_in_progress_)
  {
    status_.execution_state = MotionStatus::MOTION_STATE_IDLE;
    status_.has_active_execution = false;
    status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
  }
  if (
    status_.execution_state == MotionStatus::MOTION_STATE_STOPPING &&
    !execution_active && inactivity_confirmed && !motion_execution_in_progress_)
  {
    status_.execution_state = MotionStatus::MOTION_STATE_STOPPED;
    status_.has_active_execution = false;
    status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
    RCLCPP_INFO(
      rclcpp::get_logger("motion_core"),
      "Motion backend execution stopped and inactivity was confirmed");
  }
  if (status_.execution_state == MotionStatus::MOTION_STATE_STOPPING &&
    !execution_active && !motion_execution_in_progress_)
  {
    // Physical stop observation may still be pending, but the terminal Action
    // and its command producer no longer own Motion.
    status_.has_active_execution = false;
    status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
  }
}

bool MotionCore::set_runtime_tuning(
  const MotionTuning & tuning, const MotionGeometry & geometry,
  std::chrono::milliseconds holding_confirmation_timeout, bool idle_mutable)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (place_waypoint_generation_in_progress_ ||
    (idle_mutable && (goal_submission_pending_ || motion_execution_in_progress_ ||
    status_.has_active_execution ||
    status_.execution_state == MotionStatus::MOTION_STATE_STOPPING)))
  {
    return false;
  }
  runtime_tuning_ = tuning;
  runtime_geometry_ = geometry;
  holding_confirmation_timeout_ = holding_confirmation_timeout;
  return true;
}

MotionCore::GripperOpenResult MotionCore::request_operator_gripper_open(
  bool object_supported_or_restrained, bool release_area_clear)
{
  GripperOpenResult result;
  std::shared_ptr<GripperPort> gripper;
  std::chrono::milliseconds confirmation_timeout;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = now_();
    if (!object_supported_or_restrained || !release_area_clear) {
      result.diagnostic_detail = "operator safety confirmation is required";
      return result;
    }
    if (!has_safety_state_ || now < safety_state_receipt_time_ ||
      now - safety_state_receipt_time_ > safety_state_timeout_ || !safety_state_.valid ||
      !safety_state_.required_inputs_fresh ||
      safety_state_.motion_capability.value !=
      arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL)
    {
      result.diagnostic_detail = "fresh Safety MOTION_NORMAL permission is required";
      return result;
    }
    if (goal_submission_pending_ || motion_execution_in_progress_ ||
      place_waypoint_generation_in_progress_ ||
      status_.has_active_execution || status_.execution_active ||
      (status_.execution_state != MotionStatus::MOTION_STATE_IDLE &&
      status_.execution_state != MotionStatus::MOTION_STATE_STOPPED))
    {
      result.diagnostic_detail = "Motion has an active or stopping task";
      return result;
    }
    if (!backend_ || backend_->execution_active() ||
      !backend_->backend_inactivity_confirmed())
    {
      result.diagnostic_detail = "backend inactivity is not confirmed";
      return result;
    }
    gripper = backend_->gripper_port();
    if (!gripper || gripper->active()) {
      result.diagnostic_detail = "Gripper is unavailable or active";
      return result;
    }
    if (operator_gripper_open_in_progress_) {
      result.diagnostic_detail = "another operator Gripper Open is active";
      return result;
    }
    operator_gripper_open_in_progress_ = true;
    confirmation_timeout = holding_confirmation_timeout_;
    result.accepted = true;
  }

  struct OpenReservationGuard
  {
    MotionCore & core;
    ~OpenReservationGuard()
    {
      std::lock_guard<std::mutex> lock(core.mutex_);
      core.operator_gripper_open_in_progress_ = false;
    }
  } reservation{*this};

  const auto safety_permission_current = [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = now_();
      return !stop_requested_ &&
             status_.execution_state != MotionStatus::MOTION_STATE_STOPPING &&
             has_safety_state_ && now >= safety_state_receipt_time_ &&
             now - safety_state_receipt_time_ <= safety_state_timeout_ &&
             safety_state_.valid && safety_state_.required_inputs_fresh &&
             safety_state_.motion_capability.value ==
             arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
    };

  try {
    if (!safety_permission_current()) {
      result.diagnostic_detail = "Safety permission was withdrawn before Gripper Open";
      return result;
    }
    result.command_issued = gripper->open() ==
      MotionTaskResultCode::TASK_RESULT_SUCCESS;
    if (!result.command_issued) {
      result.diagnostic_detail = "Gripper Open command was rejected by the backend";
      return result;
    }
    const auto deadline = Clock::now() + confirmation_timeout;
    while (Clock::now() < deadline) {
      if (!safety_permission_current()) {
        if (gripper->active()) {
          gripper->stop();
        }
        result.diagnostic_detail = "Safety permission was withdrawn during Gripper Open";
        return result;
      }
      const auto observation = gripper->holding();
      if (observation.state == HoldingState::RELEASED && observation.fresh()) {
        if (safety_permission_current()) {
          result.released_confirmed = true;
          result.diagnostic_detail = "fresh Gripper RELEASED observation confirmed";
          return result;
        }
        if (gripper->active()) {
          gripper->stop();
        }
        result.diagnostic_detail = "Safety permission was withdrawn during Gripper Open";
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    result.diagnostic_detail = "Open command issued; fresh RELEASED was not confirmed";
    return result;
  } catch (const std::exception & error) {
    result.diagnostic_detail = std::string("Gripper Open failed: ") + error.what();
  } catch (...) {
    result.diagnostic_detail = "Gripper Open failed with an unknown backend error";
  }
  return result;
}

MotionPortResult MotionCore::planAndExecuteMotion(const MotionPortRequest & request)
{
  MotionPortResult result;
  result.execution_id = request.execution_id;
  result.movement_id = request.movement_id;
  const bool known_segment =
    request.segment == MotionSegmentKind::PICK_APPROACH ||
    request.segment == MotionSegmentKind::PICK_DESCENT ||
    request.segment == MotionSegmentKind::PICK_RETRACT ||
    request.segment == MotionSegmentKind::PLACE_APPROACH ||
    request.segment == MotionSegmentKind::PLACE_DESCENT ||
    request.segment == MotionSegmentKind::PLACE_CLEARANCE ||
    request.segment == MotionSegmentKind::PLACE_RETRACT ||
    request.segment == MotionSegmentKind::DISTURB_APPROACH ||
    request.segment == MotionSegmentKind::DISTURB_PUSH ||
    request.segment == MotionSegmentKind::DISTURB_RETRACT;
  const bool has_execution_id = std::any_of(
    request.execution_id.uuid.begin(), request.execution_id.uuid.end(),
    [](std::uint8_t byte) {return byte != 0U;});
  if (request.movement_id.empty() || !known_segment || !has_execution_id) {
    result.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    result.failure_kind = MotionFailureKind::OTHER;
    result.diagnostic_detail = "MotionPort requires a valid execution_id, movement_id, and segment";
    result.motion_status = status();
    return result;
  }
  if (!valid_motion_port_pose(request)) {
    result.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    result.failure_kind = MotionFailureKind::OTHER;
    result.diagnostic_detail = "MotionPort requires a valid base_link target pose and orientation";
    result.motion_status = status();
    return result;
  }

  result.result_code = accept_goal_impl(
    request.execution_id, task_type_for_motion_segment(request.segment),
    Clock::now(), true, true, &result.diagnostic_detail, &request);
  switch (result.result_code) {
    case MotionTaskResultCode::TASK_RESULT_SUCCESS:
      result.failure_kind = MotionFailureKind::NONE;
      result.motion_execution_completed = true;
      break;
    case MotionTaskResultCode::TASK_RESULT_CANCELED:
      result.failure_kind = MotionFailureKind::CANCELED;
      break;
    case MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED:
      result.failure_kind = MotionFailureKind::SAFETY_PREEMPTED;
      break;
    case MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE:
    case MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED:
    case MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED:
      result.failure_kind = MotionFailureKind::PLANNING;
      break;
    default:
      result.failure_kind = result.result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR ?
        MotionFailureKind::EXECUTION : MotionFailureKind::OTHER;
      break;
  }
  switch (result.failure_kind) {
    case MotionFailureKind::PLANNING: result.failure_stage = "motion_port:planning"; break;
    case MotionFailureKind::EXECUTION: result.failure_stage = "motion_port:execution"; break;
    case MotionFailureKind::CANCELED: result.failure_stage = "motion_port:canceled"; break;
    case MotionFailureKind::SAFETY_PREEMPTED:
      result.failure_stage = "motion_port:safety_preempted"; break;
    case MotionFailureKind::NONE: result.failure_stage.clear(); break;
    case MotionFailureKind::OTHER: result.failure_stage = "motion_port:admission"; break;
  }
  refresh_stop_state();
  result.motion_status = status();
  return result;
}

MotionPlanningResult MotionCore::planMotion(const MotionPortRequest & request)
{
  MotionPlanningResult result;
  result.plan.execution_id = request.execution_id;
  result.plan.movement_id = request.movement_id;
  const bool known_segment =
    request.segment == MotionSegmentKind::PICK_APPROACH ||
    request.segment == MotionSegmentKind::PICK_DESCENT ||
    request.segment == MotionSegmentKind::PICK_RETRACT ||
    request.segment == MotionSegmentKind::PLACE_APPROACH ||
    request.segment == MotionSegmentKind::PLACE_DESCENT ||
    request.segment == MotionSegmentKind::PLACE_CLEARANCE ||
    request.segment == MotionSegmentKind::PLACE_RETRACT ||
    request.segment == MotionSegmentKind::DISTURB_APPROACH ||
    request.segment == MotionSegmentKind::DISTURB_PUSH ||
    request.segment == MotionSegmentKind::DISTURB_RETRACT;
  const bool has_execution_id = std::any_of(
    request.execution_id.uuid.begin(), request.execution_id.uuid.end(),
    [](std::uint8_t byte) {return byte != 0U;});
  if (request.movement_id.empty() || !known_segment || !has_execution_id) {
    result.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    result.failure_kind = MotionFailureKind::OTHER;
    result.diagnostic_detail = "MotionPort requires a valid execution_id, movement_id, and segment";
    result.motion_status = status();
    return result;
  }
  if (!valid_motion_port_pose(request)) {
    result.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    result.failure_kind = MotionFailureKind::OTHER;
    result.diagnostic_detail = "MotionPort requires a valid base_link target pose and orientation";
    result.motion_status = status();
    return result;
  }

  bool extend_reserved_operation = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    extend_reserved_operation = !retained_planned_operations_.empty() &&
      goal_submission_pending_ && pending_goal_uuid_.uuid == request.execution_id.uuid &&
      active_movement_id_ == request.movement_id && !motion_execution_in_progress_ &&
      interruption_reason_ == InterruptionReason::NONE;
    if (extend_reserved_operation) {
      const auto validation = validate_goal_locked(
        task_type_for_motion_segment(request.segment), Clock::now(), nullptr);
      if (validation != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        result.result_code = validation;
        result.failure_kind = MotionFailureKind::OTHER;
        result.failure_stage = "motion_port:admission";
        result.diagnostic_detail = "Safety permission was withdrawn before planning";
        result.motion_status = status_;
        return result;
      }
      motion_execution_in_progress_ = true;
    }
  }
  if (extend_reserved_operation) {
    NormalizedTrajectoryPlan backend_plan;
    const bool backend_available = backend_->available();
    const auto backend_plan_result = backend_available ?
      motion_execution_support_->plan_motion(request, backend_plan) :
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    auto plan_result = backend_plan_result;
    MotionPlan public_plan;
    MotionPlan interrupted_plan;
    std::optional<NormalizedTrajectoryPlan> interrupted_backend_plan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      plan_result = interruption_result_locked(plan_result);
      motion_execution_in_progress_ = false;
      if (plan_result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        public_plan.plan_id = next_plan_id_++;
        public_plan.execution_id = request.execution_id;
        public_plan.movement_id = request.movement_id;
        public_plan.trajectory = backend_plan.trajectory;
        public_plan.start_state = backend_plan.start_state;
        public_plan.end_state = backend_plan.end_state;
        retained_planned_operations_[public_plan.plan_id] = backend_plan;
      } else if (!retained_planned_operations_.empty()) {
        motion_execution_in_progress_ = true;
        const auto & entry = *retained_planned_operations_.begin();
        interrupted_backend_plan = entry.second;
        interrupted_plan.plan_id = entry.first;
        interrupted_plan.execution_id = request.execution_id;
        interrupted_plan.movement_id = request.movement_id;
        interrupted_plan.trajectory = entry.second.trajectory;
        interrupted_plan.start_state = entry.second.start_state;
        interrupted_plan.end_state = entry.second.end_state;
      }
      result.motion_status = status_;
    }
    if (interrupted_backend_plan) {
      const auto terminal = finish_planned_operation(
        interrupted_plan, *interrupted_backend_plan, plan_result, 0.0,
        !(backend_available && backend_->supports_split_planning() &&
        backend_plan_result != MotionTaskResultCode::TASK_RESULT_SUCCESS));
      result.motion_status = terminal.motion_status;
    }
    result.result_code = plan_result;
    result.plan = std::move(public_plan);
    if (plan_result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      result.failure_kind = MotionFailureKind::NONE;
    } else if (plan_result == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      result.failure_kind = MotionFailureKind::CANCELED;
    } else if (plan_result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      result.failure_kind = MotionFailureKind::SAFETY_PREEMPTED;
    } else if (plan_result == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
      plan_result == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
      plan_result == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED)
    {
      result.failure_kind = MotionFailureKind::PLANNING;
    } else {
      result.failure_kind = MotionFailureKind::OTHER;
    }
    result.failure_stage = result.failure_kind == MotionFailureKind::PLANNING ?
      "motion_port:planning" : result.failure_kind == MotionFailureKind::CANCELED ?
      "motion_port:canceled" : result.failure_kind == MotionFailureKind::SAFETY_PREEMPTED ?
      "motion_port:safety_preempted" : result.failure_kind == MotionFailureKind::NONE ?
      "" : "motion_port:admission";
    return result;
  }

  NormalizedTrajectoryPlan backend_plan;
  std::uint64_t plan_id = 0U;
  result.result_code = accept_goal_impl(
    request.execution_id, task_type_for_motion_segment(request.segment),
    Clock::now(), true, true, &result.diagnostic_detail, &request, nullptr, false, {},
    &backend_plan, true, &plan_id);
  switch (result.result_code) {
    case MotionTaskResultCode::TASK_RESULT_SUCCESS:
      result.failure_kind = MotionFailureKind::NONE;
      result.plan.plan_id = plan_id;
      result.plan.execution_id = request.execution_id;
      result.plan.movement_id = request.movement_id;
      result.plan.trajectory = backend_plan.trajectory;
      result.plan.start_state = backend_plan.start_state;
      result.plan.end_state = backend_plan.end_state;
      break;
    case MotionTaskResultCode::TASK_RESULT_CANCELED:
      result.failure_kind = MotionFailureKind::CANCELED;
      break;
    case MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED:
      result.failure_kind = MotionFailureKind::SAFETY_PREEMPTED;
      break;
    case MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE:
    case MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED:
    case MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED:
      result.failure_kind = MotionFailureKind::PLANNING;
      break;
    default:
      result.failure_kind = result.result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR ?
        MotionFailureKind::EXECUTION : MotionFailureKind::OTHER;
      break;
  }
  switch (result.failure_kind) {
    case MotionFailureKind::PLANNING:
      result.failure_stage = "motion_port:planning";
      break;
    case MotionFailureKind::EXECUTION:
      result.failure_stage = "motion_port:execution";
      break;
    case MotionFailureKind::CANCELED:
      result.failure_stage = "motion_port:canceled";
      break;
    case MotionFailureKind::SAFETY_PREEMPTED:
      result.failure_stage = "motion_port:safety_preempted";
      break;
    case MotionFailureKind::NONE:
      result.failure_stage.clear();
      break;
    case MotionFailureKind::OTHER:
      result.failure_stage = "motion_port:admission";
      break;
  }
  result.motion_status = status();
  return result;
}

MotionPortResult MotionCore::executePlannedMotion(const MotionPlan & plan)
{
  MotionPortResult result;
  result.execution_id = plan.execution_id;
  result.movement_id = plan.movement_id;
  NormalizedTrajectoryPlan backend_plan;
  std::uint8_t preflight_result = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stored_plan = retained_planned_operations_.find(plan.plan_id);
    const bool matching_plan = stored_plan != retained_planned_operations_.end() &&
      stored_plan->second.request.motion_goal_uuid == uuid_string(plan.execution_id) &&
      stored_plan->second.request.movement_id == plan.movement_id &&
      goal_submission_pending_ && pending_goal_uuid_.uuid == plan.execution_id.uuid &&
      active_movement_id_ == plan.movement_id;
    if (!matching_plan) {
      MotionPortResult invalid;
      invalid.execution_id = plan.execution_id;
      invalid.movement_id = plan.movement_id;
      invalid.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      invalid.failure_kind = MotionFailureKind::OTHER;
      invalid.failure_stage = "motion_port:plan_identity";
      invalid.diagnostic_detail = "Trajectory plan is stale or belongs to another operation";
      invalid.motion_status = status_;
      return invalid;
    }
    if (motion_execution_in_progress_) {
      result.result_code = MotionTaskResultCode::TASK_RESULT_BUSY;
      result.failure_kind = MotionFailureKind::OTHER;
      result.failure_stage = "motion_port:admission";
      result.diagnostic_detail = "Another plan or trajectory execution owns Motion";
      result.motion_status = status_;
      return result;
    }
    const bool earlier_plan_pending = std::any_of(
      retained_planned_operations_.begin(), retained_planned_operations_.end(),
      [&plan](const auto & entry) {return entry.first < plan.plan_id;});
    if (earlier_plan_pending) {
      result.result_code = MotionTaskResultCode::TASK_RESULT_BUSY;
      result.failure_kind = MotionFailureKind::OTHER;
      result.failure_stage = "motion_port:admission";
      result.diagnostic_detail = "An earlier planned trajectory must complete first";
      result.motion_status = status_;
      return result;
    }
    backend_plan = stored_plan->second;
    if (plan.trajectory.points.empty() ||
      !same_trajectory(plan.trajectory, stored_plan->second.trajectory))
    {
      result.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      result.failure_kind = MotionFailureKind::OTHER;
      result.failure_stage = "motion_port:plan_trajectory";
      result.diagnostic_detail = "Trajectory does not match the retained plan";
      result.motion_status = status_;
      return result;
    }
    preflight_result = validate_goal_locked(
      backend_plan.request.task_type.value, Clock::now(), nullptr);
    preflight_result = interruption_result_locked(preflight_result);
    motion_execution_in_progress_ = true;
    status_.backend_inactivity_confirmed = false;
  }

  const auto execution_started = std::chrono::steady_clock::now();
  auto execution_result = preflight_result;
  if (execution_result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    execution_result = motion_execution_support_->execute_planned_motion(backend_plan, false);
  }
  return finish_planned_operation(
    plan, backend_plan, execution_result,
    std::chrono::duration<double>(std::chrono::steady_clock::now() - execution_started).count());
}

MotionPortResult MotionCore::finish_planned_operation(
  const MotionPlan & plan, const NormalizedTrajectoryPlan &,
  std::uint8_t execution_result, double elapsed_s, bool finalize_diagnostic)
{
  MotionPortResult result;
  result.execution_id = plan.execution_id;
  result.movement_id = plan.movement_id;
  result.result_code = execution_result;
  const bool succeeded = execution_result == MotionTaskResultCode::TASK_RESULT_SUCCESS;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto retained_plan = retained_planned_operations_.find(plan.plan_id);
    if (retained_plan == retained_planned_operations_.end()) {
      result.result_code = interruption_result_locked(execution_result);
      result.motion_status = status_;
      return result;
    }
    result.result_code = interruption_result_locked(execution_result);
    if (result.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      result.failure_kind = MotionFailureKind::NONE;
    } else if (result.result_code == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      result.failure_kind = MotionFailureKind::CANCELED;
    } else if (result.result_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      result.failure_kind = MotionFailureKind::SAFETY_PREEMPTED;
    } else if (result.result_code == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
      result.result_code == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
      result.result_code == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED)
    {
      result.failure_kind = MotionFailureKind::PLANNING;
    } else if (result.result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR) {
      result.failure_kind = MotionFailureKind::EXECUTION;
    } else {
      result.failure_kind = MotionFailureKind::OTHER;
    }
    switch (result.failure_kind) {
      case MotionFailureKind::NONE: result.failure_stage.clear(); break;
      case MotionFailureKind::PLANNING: result.failure_stage = "motion_port:planning"; break;
      case MotionFailureKind::EXECUTION: result.failure_stage = "motion_port:execution"; break;
      case MotionFailureKind::CANCELED: result.failure_stage = "motion_port:canceled"; break;
      case MotionFailureKind::SAFETY_PREEMPTED:
        result.failure_stage = "motion_port:safety_preempted"; break;
      case MotionFailureKind::OTHER: result.failure_stage = "motion_port:admission"; break;
    }
    retained_planned_operations_.erase(plan.plan_id);
    const bool retain_owner = result.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
      !retained_planned_operations_.empty() && !stop_requested_;
    if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      retained_planned_operations_.clear();
    }
    const bool active = backend_->execution_active();
    const bool inactive = backend_->backend_inactivity_confirmed();
    status_.execution_active = active;
    status_.backend_inactivity_confirmed = inactive;
    if (retain_owner) {
      status_.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
      status_.has_active_execution = true;
    } else if (stop_requested_ ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING)
    {
      status_.execution_state = !active && inactive ? MotionStatus::MOTION_STATE_STOPPED :
        MotionStatus::MOTION_STATE_STOPPING;
      status_.has_active_execution = active;
    } else {
      const bool completed = result.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS;
      status_.execution_state = active || (!completed && !inactive) ?
        MotionStatus::MOTION_STATE_EXECUTING : MotionStatus::MOTION_STATE_IDLE;
      status_.has_active_execution = active || (!completed && !inactive);
    }
    const auto envelope = backend_->applied_motion_envelope();
    status_.applied_envelope_valid = envelope.valid;
    status_.applied_velocity_scale = envelope.max_velocity_scale;
    status_.applied_acceleration_scale = envelope.max_acceleration_scale;
    if (!status_.has_active_execution) {
      status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
    }
    motion_execution_in_progress_ = false;
    if (!retain_owner) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      active_movement_id_.clear();
      retained_planned_operations_.clear();
      stop_requested_ = false;
      clear_execution_interruption_locked();
    }
    result.motion_status = status_;
  }
  result.motion_execution_completed = succeeded &&
    result.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS;
  if (result.result_code != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    result.diagnostic_detail = result.result_code == MotionTaskResultCode::TASK_RESULT_CANCELED ?
      "caller cancellation" :
      result.result_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ?
      "Safety preemption" :
      result.result_code == MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE ?
      "backend unavailable" :
      std::string("backend operation failed: ") + motion_result_code_name(result.result_code);
  }
  const auto diagnostic_stage = result.failure_stage;
  const auto diagnostic_detail = result.diagnostic_detail;
  if (finalize_diagnostic) {
    try {
      backend_->finalize_motion_planning_diagnostic(
        result.result_code, elapsed_s, diagnostic_stage, diagnostic_detail);
    } catch (...) {
      // Diagnostic capture is best-effort and cannot replace the movement result.
    }
  }
  return result;
}

MotionDisturbResult MotionCore::executeDisturb(const MotionDisturbRequest & request)
{
  MotionDisturbResult result;
  result.execution_id = request.execution_id;
  result.movement_id = request.movement_id;
  result.acquisition_id = request.acquisition_id;
  return result;
}

MotionDisturbanceParameters MotionCore::disturbanceParameters() const
{
  return backend_->disturbance_parameters();
}

MotionCancelResult MotionCore::cancelMotion(
  const UUID & execution_id, const std::string & movement_id)
{
  MotionCancelResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool matching_pending = goal_submission_pending_ &&
      pending_goal_uuid_.uuid == execution_id.uuid;
    const bool matching_active = status_.has_active_execution &&
      status_.active_execution_id.uuid == execution_id.uuid;
    if (movement_id.empty() || active_movement_id_ != movement_id ||
      (!matching_pending && !matching_active))
    {
      result.motion_status = status_;
      result.backend_inactivity_confirmed = backend_->backend_inactivity_confirmed();
      return result;
    }
  }
  result.accepted = request_cancel(execution_id);
  std::optional<NormalizedTrajectoryPlan> canceled_plan;
  MotionPlan public_plan;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_execution_in_progress_ &&
      interruption_reason_ != InterruptionReason::SAFETY_PREEMPTED &&
      !retained_planned_operations_.empty() &&
      pending_goal_uuid_.uuid == execution_id.uuid &&
      active_movement_id_ == movement_id)
    {
      // Claim terminalization while holding the operation lock so an executor
      // cannot start or finalize the same retained plan concurrently.
      motion_execution_in_progress_ = true;
      const auto & entry = *retained_planned_operations_.begin();
      canceled_plan = entry.second;
      public_plan.plan_id = entry.first;
      public_plan.execution_id = execution_id;
      public_plan.movement_id = movement_id;
      public_plan.trajectory = entry.second.trajectory;
      public_plan.start_state = entry.second.start_state;
      public_plan.end_state = entry.second.end_state;
    }
  }
  if (canceled_plan) {
    (void)finish_planned_operation(
      public_plan, *canceled_plan, MotionTaskResultCode::TASK_RESULT_CANCELED);
  }
  result.backend_inactivity_confirmed = backend_->backend_inactivity_confirmed();
  result.motion_status = status();
  return result;
}

MotionObservation MotionCore::observeMotionState() const
{
  MotionObservation observation;
  observation.motion_status = status();
  observation.measured_joint_state_available =
    backend_->measured_joint_state(observation.measured_joint_state);
  return observation;
}

bool MotionCore::measuredTcpPose(geometry_msgs::msg::PoseStamped & pose) const
{
  return backend_ && backend_->measured_tcp_pose(pose);
}

bool MotionCore::generatePlaceWaypointApproach(const std::string & path, std::string & detail)
{
  using Code = MotionTaskResultCode;
  MotionGeometry geometry;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path.empty() || place_waypoint_generation_in_progress_ || goal_submission_pending_ ||
      motion_execution_in_progress_ || operator_gripper_open_in_progress_ ||
      status_.has_active_execution || status_.execution_active ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING ||
      !retained_planned_operations_.empty() || !backend_ || backend_->execution_active() ||
      !backend_->backend_inactivity_confirmed())
    {
      detail = "PLACE WAYPOINT generation requires an idle Motion runtime and output path";
      return false;
    }
    place_waypoint_generation_in_progress_ = true;
    geometry = runtime_geometry_;
  }
  struct GenerationGuard
  {
    MotionCore & core;
    ~GenerationGuard()
    {
      std::lock_guard<std::mutex> lock(core.mutex_);
      core.place_waypoint_generation_in_progress_ = false;
    }
  } guard{*this};
  if (!geometry.place_waypoint_configured || !backend_->available()) {
    detail = "PLACE WAYPOINT profile or Planning Backend is unavailable";
    return false;
  }
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  goal.target_pose = geometry.place_waypoint_target_pose;
  goal.place_approach_distance_m = geometry.place_waypoint_approach_distance_m;
  goal.place_retract_distance_m = geometry.place_waypoint_approach_distance_m;
  goal.place_approach_direction_object = geometry.place_waypoint_approach_direction_object;
  goal.place_orientation_constraint = ExecuteTask::Goal::PLACE_ORIENTATION_FIXED;
  auto request = motion_execution_support_->normalized_request(
    goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
    "explicit-place-waypoint-generation");
  request.place_waypoint_approach = true;
  const auto result = backend_->generate_place_waypoint_approach(request, path);
  detail = result == Code::TASK_RESULT_SUCCESS ?
    "optimized PLACE WAYPOINT trajectory saved to " + path :
    "PLACE WAYPOINT trajectory generation failed with result " + std::to_string(result);
  return result == Code::TASK_RESULT_SUCCESS;
}

void MotionCore::beginPickSearch(const std::string & execution_id)
{backend_->begin_pick_search(execution_id);}

void MotionCore::beginPickCandidateSearch()
{backend_->begin_pick_candidate_search();}

void MotionCore::recordMaterialCandidateSearch(
  const std::vector<MaterialCandidateSearchAttempt> & attempts,
  const std::string & selected_candidate_id)
{backend_->record_material_candidate_search(attempts, selected_candidate_id);}

std::string MotionCore::lastPickCandidateEvaluation() const
{return backend_->last_pick_candidate_evaluation();}

void MotionCore::resetPickGraspInsertionStarted()
{backend_->reset_pick_grasp_insertion_started();}

bool MotionCore::pickGraspInsertionStarted() const
{return backend_->pick_grasp_insertion_started();}

std::string MotionCore::lastPlaceFailureStage() const
{return backend_->last_place_failure_stage();}

bool MotionCore::advanceToNextPlanningCandidate()
{return backend_->advance_to_next_planning_candidate();}

bool MotionCore::retryPlaceRetreat()
{return backend_->retry_place_retreat();}

void MotionCore::finalizeMotionPlanningDiagnostic(
  std::uint8_t result_code, double elapsed_s, const std::string & failure_stage,
  const std::string & failure_detail)
{
  backend_->finalize_motion_planning_diagnostic(
    result_code, elapsed_s, failure_stage,
    failure_detail);
}

void MotionCore::beginPickGraspRelation()
{backend_->begin_pick_grasp_relation();}

void MotionCore::stageSelectedPickGraspRelation(const geometry_msgs::msg::Transform & transform)
{backend_->stage_selected_pick_grasp_relation(transform);}

bool MotionCore::stagePickCaptureReference(
  const geometry_msgs::msg::PoseStamped & pose, double height, std::uint8_t symmetry,
  const std::string & source)
{return backend_->stage_pick_capture_reference(pose, height, symmetry, source);}

void MotionCore::discardPendingGraspRelation()
{backend_->discard_pending_grasp_relation();}

bool MotionCore::commitPendingGraspRelation()
{return backend_->commit_pending_grasp_relation();}

void MotionCore::clearGraspRelationAfterRelease()
{backend_->clear_grasp_relation_after_release();}

std::uint8_t MotionCore::executeLegacyMovement(
  std::uint8_t task_type, const std::string & execution_id,
  const std::string & movement_id)
{
  if (task_type != MotionTaskType::TASK_TYPE_GO_HOME &&
    task_type != MotionTaskType::TASK_TYPE_RETRACT)
  {
    return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  ExecuteTask::Goal legacy_movement;
  legacy_movement.task_type.value = task_type;
  return accept_goal_impl(
    parse_uuid(execution_id), task_type, Clock::now(), true, true,
    nullptr, nullptr, &legacy_movement, false, movement_id);
}

std::uint8_t MotionCore::executePlaceRecoveryJointTarget(
  const UUID & execution_id, const std::string & movement_id)
{
  return accept_goal_impl(
    execution_id, MotionTaskType::TASK_TYPE_PLACE, Clock::now(), true, true,
    nullptr, nullptr, nullptr, true, movement_id);
}

bool MotionCore::is_supported_task(uint8_t task_type)
{
  return task_type >= MotionTaskType::TASK_TYPE_PICK &&
         task_type <= MotionTaskType::TASK_TYPE_RETRACT;
}

bool MotionCore::is_known_capability(uint8_t capability)
{
  return capability == arm_cell_interfaces::msg::MotionCapability::MOTION_NONE ||
         capability == arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY ||
         capability == arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
}

bool MotionCore::is_known_safety_state(uint8_t safety_state)
{
  return safety_state == SafetyState::SAFETY_STATE_UNKNOWN ||
         safety_state == SafetyState::SAFETY_STATE_SAFE ||
         safety_state == SafetyState::SAFETY_STATE_INTERLOCKED ||
         safety_state == SafetyState::SAFETY_STATE_STOPPING ||
         safety_state == SafetyState::SAFETY_STATE_STOPPED ||
         safety_state == SafetyState::SAFETY_STATE_RECOVERY_REQUIRED ||
         safety_state == SafetyState::SAFETY_STATE_EMERGENCY_STOPPED;
}

bool MotionCore::capability_allows_task(uint8_t capability, uint8_t task_type)
{
  if (capability == arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL) {
    return true;
  }
  return capability == arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY &&
         task_type == MotionTaskType::TASK_TYPE_RETRACT;
}

uint8_t MotionCore::validate_goal_locked(
  uint8_t task_type, Clock::time_point now, std::string * diagnostic_detail) const
{
  if (!is_supported_task(task_type)) {
    return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  std::string failed_checks;
  const auto add_failed_check = [&failed_checks](const char * check) {
      if (!failed_checks.empty()) {
        failed_checks += ",";
      }
      failed_checks += check;
    };
  if (!has_safety_state_) {add_failed_check("safety_state_unavailable");}
  if (!safety_state_.valid) {add_failed_check("safety_state_invalid");}
  if (!safety_state_.required_inputs_fresh) {add_failed_check("required_inputs_stale");}
  if (!is_known_safety_state(safety_state_.safety_state)) {
    add_failed_check("unknown_safety_state");
  }
  if (safety_state_.safety_state == SafetyState::SAFETY_STATE_UNKNOWN) {
    add_failed_check("safety_state_unknown");
  }
  if (now < safety_state_receipt_time_) {add_failed_check("safety_state_receipt_in_future");}
  if (now >= safety_state_receipt_time_ &&
    now - safety_state_receipt_time_ > safety_state_timeout_)
  {
    add_failed_check("safety_state_stale");
  }
  if (!is_known_capability(safety_state_.motion_capability.value)) {
    add_failed_check("unknown_motion_capability");
  }
  if (!safety_state_.motion_envelope_valid) {add_failed_check("motion_envelope_invalid");}
  if (!std::isfinite(safety_state_.max_velocity_scale)) {
    add_failed_check("max_velocity_scale_non_finite");
  } else if (safety_state_.max_velocity_scale <= 0.0F ||
    safety_state_.max_velocity_scale > 1.0F)
  {
    add_failed_check("max_velocity_scale_out_of_range");
  }
  if (!std::isfinite(safety_state_.max_acceleration_scale)) {
    add_failed_check("max_acceleration_scale_non_finite");
  } else if (safety_state_.max_acceleration_scale <= 0.0F ||
    safety_state_.max_acceleration_scale > 1.0F)
  {
    add_failed_check("max_acceleration_scale_out_of_range");
  }
  if (is_known_capability(safety_state_.motion_capability.value) &&
    !capability_allows_task(safety_state_.motion_capability.value, task_type))
  {
    add_failed_check("capability_does_not_allow_task");
  }
  if (!failed_checks.empty()) {
    set_diagnostic(
      diagnostic_detail,
      "Safety validation denied task_type=" + std::to_string(task_type) +
      " failed_checks=" + failed_checks +
      " safety_state=" + std::to_string(safety_state_.safety_state) +
      " motion_capability=" + std::to_string(safety_state_.motion_capability.value) +
      " motion_envelope_valid=" + (safety_state_.motion_envelope_valid ? "true" : "false") +
      " max_velocity_scale=" + std::to_string(safety_state_.max_velocity_scale) +
      " max_acceleration_scale=" + std::to_string(safety_state_.max_acceleration_scale));
    return MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED;
  }
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

}  // namespace arm_cell_motion
