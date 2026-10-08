#include "arm_cell_motion_moveit2/motion_core.hpp"

#include <iomanip>
#include <cmath>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

namespace arm_cell_motion_moveit2
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

void set_diagnostic(std::string * diagnostic_detail, const std::string & value)
{
  if (diagnostic_detail) {
    *diagnostic_detail = value;
  }
}
}  // namespace

MotionCore::MotionCore(
  std::shared_ptr<MotionBackend> backend,
  std::chrono::milliseconds safety_state_timeout, MotionGeometry geometry,
  std::chrono::milliseconds holding_confirmation_timeout)
: backend_(std::move(backend)),
  task_executor_(backend_, geometry, holding_confirmation_timeout),
  safety_state_timeout_(safety_state_timeout), runtime_geometry_(geometry),
  holding_confirmation_timeout_(holding_confirmation_timeout)
{
  status_.execution_state = MotionStatus::MOTION_STATE_IDLE;
  status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
  status_.execution_active = false;
  status_.backend_inactivity_confirmed = true;
  task_executor_.set_interruption_checker(
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

void MotionCore::update_object_state(MotionObjectState object_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  object_state_ = object_state;
  task_executor_.set_object_state(object_state);
}

uint8_t MotionCore::validate_goal(
  const ExecuteTask::Goal & goal, Clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return validate_goal_locked(goal, now);
}

uint8_t MotionCore::reserve_goal(const UUID & goal_uuid, const ExecuteTask::Goal & goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto validation_result = validate_goal_locked(goal, Clock::now());
  if (validation_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return validation_result;
  }
  if (goal_submission_pending_ || status_.has_active_execution ||
    status_.execution_state == MotionStatus::MOTION_STATE_STOPPING)
  {
    return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  goal_submission_pending_ = true;
  pending_goal_uuid_ = goal_uuid;
  active_tuning_snapshot_ = runtime_tuning_;
  active_geometry_snapshot_ = runtime_geometry_;
  active_holding_confirmation_timeout_ = holding_confirmation_timeout_;
  stop_requested_ = false;
  interruption_reason_ = InterruptionReason::NONE;
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
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
  return current;
}

uint8_t MotionCore::accept_goal(const UUID & goal_uuid, const ExecuteTask::Goal & goal)
{
  const auto acceptance_time = Clock::now();
  return accept_goal_impl(
    goal_uuid, goal, acceptance_time, true);
}

uint8_t MotionCore::accept_goal_with_diagnostic(
  const UUID & goal_uuid, const ExecuteTask::Goal & goal, std::string & diagnostic_detail)
{
  diagnostic_detail.clear();
  return accept_goal_impl(
    goal_uuid, goal, Clock::now(), true, &diagnostic_detail);
}

uint8_t MotionCore::accept_goal(
  const UUID & goal_uuid, const ExecuteTask::Goal & goal, Clock::time_point now)
{
  return accept_goal_impl(goal_uuid, goal, now, false);
}

uint8_t MotionCore::accept_goal_impl(
  const UUID & goal_uuid, const ExecuteTask::Goal & goal,
  Clock::time_point acceptance_time, bool refresh_pre_submit_time,
  std::string * diagnostic_detail)
{
  const auto goal_id = uuid_string(goal_uuid);
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=entry goal_uuid=%s", goal_id.c_str());
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=initial_mutex before goal_uuid=%s", goal_id.c_str());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=initial_mutex acquired goal_uuid=%s", goal_id.c_str());
    std::string validation_diagnostic;
    const auto validation_result = validate_goal_locked(
      goal, acceptance_time, &validation_diagnostic);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=initial_safety_validation complete goal_uuid=%s result_code=%u",
      goal_id.c_str(), validation_result);
    if (validation_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      set_diagnostic(
        diagnostic_detail,
        validation_diagnostic.empty() ?
        "validation/permission rejection: result_code=" + std::to_string(validation_result) :
        validation_diagnostic);
      return validation_result;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=reservation_lookup before goal_uuid=%s pending=%s",
      goal_id.c_str(), goal_submission_pending_ ? "true" : "false");
    const bool is_reserved_goal = goal_submission_pending_ &&
      pending_goal_uuid_.uuid == goal_uuid.uuid;
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=reservation_lookup complete goal_uuid=%s reserved_for_goal=%s",
      goal_id.c_str(), is_reserved_goal ? "true" : "false");
    if ((goal_submission_pending_ && !is_reserved_goal) || status_.has_active_execution ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("motion_core"),
        "accept_goal transient rejection goal_uuid=%s goal_submission_pending=%s "
        "has_active_execution=%s execution_state=%u",
        uuid_string(goal_uuid).c_str(), goal_submission_pending_ ? "true" : "false",
        status_.has_active_execution ? "true" : "false", status_.execution_state);
      set_diagnostic(
        diagnostic_detail,
        "duplicate/pending execution rejection: goal_submission_pending=" +
        std::string(goal_submission_pending_ ? "true" : "false") +
        ", has_active_execution=" +
        std::string(status_.has_active_execution ? "true" : "false") +
        ", execution_state=" + std::to_string(status_.execution_state));
      return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=initial_state complete goal_uuid=%s goal_submission_pending=%s "
      "has_active_execution=%s execution_state=%u",
      goal_id.c_str(), goal_submission_pending_ ? "true" : "false",
      status_.has_active_execution ? "true" : "false", status_.execution_state);
    if (!is_reserved_goal) {
      goal_submission_pending_ = true;
      pending_goal_uuid_ = goal_uuid;
      active_tuning_snapshot_ = runtime_tuning_;
      active_geometry_snapshot_ = runtime_geometry_;
      active_holding_confirmation_timeout_ = holding_confirmation_timeout_;
      stop_requested_ = false;
      interruption_reason_ = InterruptionReason::NONE;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=reservation_consume complete goal_uuid=%s mode=%s",
      goal_id.c_str(), is_reserved_goal ? "accepted-reservation" : "new-reservation");
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=initial_mutex released goal_uuid=%s", goal_id.c_str());

  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=backend_available before goal_uuid=%s", goal_id.c_str());
  const bool backend_available = backend_->available();
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=backend_available complete goal_uuid=%s available=%s",
    goal_id.c_str(), backend_available ? "true" : "false");
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=backend_readiness_snapshot goal_uuid=%s %s",
    goal_id.c_str(), backend_->availability_diagnostic().c_str());
  if (!backend_available) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=backend_unavailable_cleanup before goal_uuid=%s", goal_id.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = interruption_result_locked(
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
    goal_submission_pending_ = false;
    pending_goal_uuid_ = UUID{};
    if (stop_requested_) {
      status_.execution_state = MotionStatus::MOTION_STATE_STOPPING;
      stop_requested_ = false;
    }
    clear_execution_interruption_locked();
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=backend_unavailable_cleanup complete goal_uuid=%s",
      goal_id.c_str());
    RCLCPP_WARN(
      rclcpp::get_logger("motion_core"),
      "backend unavailable goal_uuid=%s", uuid_string(goal_uuid).c_str());
    if (result == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      set_diagnostic(diagnostic_detail, "caller cancellation");
    } else if (result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      set_diagnostic(diagnostic_detail, "Safety preemption");
    } else {
      set_diagnostic(diagnostic_detail, "backend unavailable");
    }
    return result;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=pre_submit_mutex before goal_uuid=%s", goal_id.c_str());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=pre_submit_mutex acquired goal_uuid=%s", goal_id.c_str());
    // Capability authorization is not a lease. Re-evaluate immediately before
    // handing the trajectory-producing request to the backend.
    const auto pre_submit_time = refresh_pre_submit_time ? Clock::now() : acceptance_time;
    const auto pre_submit_validation = validate_goal_locked(goal, pre_submit_time);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=pre_submit_safety_validation complete goal_uuid=%s result_code=%u",
      goal_id.c_str(), pre_submit_validation);
    if (pre_submit_validation != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
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
      set_diagnostic(diagnostic_detail, "backend could not apply current Safety envelope");
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    // Parameter updates serialize on this mutex. Once the goal becomes active,
    // the executor and backend retain this one immutable tuning snapshot.
    if (!backend_->apply_runtime_tuning(active_tuning_snapshot_)) {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      set_diagnostic(diagnostic_detail, "backend rejected the Motion tuning snapshot");
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    task_executor_.set_runtime_geometry(
      active_geometry_snapshot_, active_holding_confirmation_timeout_);
    const auto applied_envelope = backend_->applied_motion_envelope();
    if (!applied_envelope.valid ||
      !std::isfinite(applied_envelope.max_velocity_scale) ||
      !std::isfinite(applied_envelope.max_acceleration_scale) ||
      applied_envelope.max_velocity_scale > requested_envelope.max_velocity_scale ||
      applied_envelope.max_acceleration_scale > requested_envelope.max_acceleration_scale)
    {
      goal_submission_pending_ = false;
      pending_goal_uuid_ = UUID{};
      set_diagnostic(diagnostic_detail, "backend did not attest the current Safety envelope");
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    status_.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
    status_.active_execution_id = goal_uuid;
    status_.has_active_execution = true;
    status_.active_task_type = goal.task_type;
    // The backend has not received the task yet; the accepted active execution
    // reservation is what Safety observes while TaskExecutor plans it.
    status_.execution_active = false;
    status_.backend_inactivity_confirmed = true;
    status_.applied_velocity_scale = applied_envelope.max_velocity_scale;
    status_.applied_acceleration_scale = applied_envelope.max_acceleration_scale;
    status_.applied_envelope_valid = true;
    task_execution_in_progress_ = true;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=pre_submit_mutex released goal_uuid=%s", goal_id.c_str());

  MotionObjectState object_state;
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=object_state_mutex before goal_uuid=%s", goal_id.c_str());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=object_state_mutex acquired goal_uuid=%s", goal_id.c_str());
    object_state = object_state_;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=object_state_mutex released goal_uuid=%s state=%u",
    goal_id.c_str(), static_cast<unsigned int>(object_state));
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=task_executor before goal_uuid=%s task_type=%u",
    goal_id.c_str(), goal.task_type.value);
  const auto execution_result = task_executor_.execute(goal, object_state, goal_id);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    task_execution_in_progress_ = false;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=task_executor complete goal_uuid=%s result_code=%u",
    goal_id.c_str(), execution_result);
  if (execution_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "task executor returned terminal failure goal_uuid=%s result_code=%u",
      uuid_string(goal_uuid).c_str(), execution_result);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto final_result = interruption_result_locked(execution_result);
    object_state_ = task_executor_.object_state();
    const auto failed_backend_envelope = backend_->applied_motion_envelope();
    status_.execution_active = backend_->execution_active();
    status_.backend_inactivity_confirmed = backend_->backend_inactivity_confirmed();
    status_.execution_state = stop_requested_ ? MotionStatus::MOTION_STATE_STOPPING :
      (status_.execution_active ? MotionStatus::MOTION_STATE_EXECUTING :
      MotionStatus::MOTION_STATE_IDLE);
    status_.has_active_execution = status_.execution_active;
    status_.applied_envelope_valid = failed_backend_envelope.valid && status_.execution_active;
    status_.applied_velocity_scale = failed_backend_envelope.max_velocity_scale;
    status_.applied_acceleration_scale = failed_backend_envelope.max_acceleration_scale;
    if (!status_.has_active_execution) {
      status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
    }
    goal_submission_pending_ = false;
    pending_goal_uuid_ = UUID{};
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
        "backend error: result_code=" + std::to_string(execution_result));
    }
    return final_result;
  }

  bool stop_was_requested = false;
  uint8_t stop_mode = arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED;
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=stop_state_mutex before goal_uuid=%s", goal_id.c_str());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=stop_state_mutex acquired goal_uuid=%s", goal_id.c_str());
    stop_was_requested = stop_requested_ ||
      status_.execution_state == MotionStatus::MOTION_STATE_STOPPING;
    stop_mode = requested_stop_mode_;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=stop_state_mutex released goal_uuid=%s stop_requested=%s stop_mode=%u",
    goal_id.c_str(), stop_was_requested ? "true" : "false", stop_mode);
  if (stop_was_requested) {
    if (stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_hold before goal_uuid=%s", goal_id.c_str());
      backend_->hold();
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_hold complete goal_uuid=%s", goal_id.c_str());
    } else if (stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_cancel before goal_uuid=%s", goal_id.c_str());
      backend_->cancel();
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_cancel complete goal_uuid=%s", goal_id.c_str());
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_hold before goal_uuid=%s", goal_id.c_str());
      backend_->hold();
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_hold complete goal_uuid=%s", goal_id.c_str());
    } else {
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_stop before goal_uuid=%s", goal_id.c_str());
      backend_->stop();
      RCLCPP_DEBUG(
        rclcpp::get_logger("motion_core"),
        "accept_goal step=backend_stop complete goal_uuid=%s", goal_id.c_str());
    }
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=backend_state before goal_uuid=%s", goal_id.c_str());
  const bool execution_active = backend_->execution_active();
  const bool inactivity_confirmed = backend_->backend_inactivity_confirmed();
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=backend_state complete goal_uuid=%s execution_active=%s "
    "inactivity_confirmed=%s",
    goal_id.c_str(), execution_active ? "true" : "false",
    inactivity_confirmed ? "true" : "false");
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=active_state_mutex before goal_uuid=%s", goal_id.c_str());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=active_state_mutex acquired goal_uuid=%s", goal_id.c_str());
    const bool stop_active = stop_requested_ || stop_was_requested;
    status_.execution_state = stop_active ?
      (!execution_active && inactivity_confirmed ? MotionStatus::MOTION_STATE_STOPPED :
      MotionStatus::MOTION_STATE_STOPPING) : MotionStatus::MOTION_STATE_EXECUTING;
    status_.active_execution_id = goal_uuid;
    status_.has_active_execution = !(
      stop_active && !execution_active && inactivity_confirmed);
    status_.active_task_type = goal.task_type;
    status_.execution_active = execution_active;
    status_.backend_inactivity_confirmed = inactivity_confirmed;
    const auto applied_envelope = backend_->applied_motion_envelope();
    status_.applied_envelope_valid = applied_envelope.valid;
    status_.applied_velocity_scale = applied_envelope.max_velocity_scale;
    status_.applied_acceleration_scale = applied_envelope.max_acceleration_scale;
    object_state_ = task_executor_.object_state();
    goal_submission_pending_ = false;
    pending_goal_uuid_ = UUID{};
    if (!status_.has_active_execution) {
      status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
    }
    stop_requested_ = false;
    clear_execution_interruption_locked();
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_core"),
      "accept_goal step=active_state_mutation complete goal_uuid=%s state=%u "
      "has_active_execution=%s execution_active=%s",
      goal_id.c_str(), status_.execution_state,
      status_.has_active_execution ? "true" : "false",
      status_.execution_active ? "true" : "false");
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_core"),
    "accept_goal step=active_state_mutex released goal_uuid=%s result_code=%u",
    goal_id.c_str(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  return MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

bool MotionCore::request_cancel()
{
  UUID goal_uuid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.has_active_execution && !goal_submission_pending_) {
      return false;
    }
    goal_uuid = goal_submission_pending_ ? pending_goal_uuid_ : status_.active_execution_id;
  }
  return request_cancel(goal_uuid);
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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!status_.has_active_execution && !goal_submission_pending_) {
      return false;
    }
    if (stop_mode != arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED &&
      stop_mode != arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE &&
      stop_mode != arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY)
    {
      return false;
    }
    status_.execution_state = MotionStatus::MOTION_STATE_STOPPING;
    status_.last_stop_request_id = request_uuid;
    status_.has_last_stop_request = true;
    stop_requested_ = true;
    requested_stop_mode_ = stop_mode;
    interruption_reason_ = InterruptionReason::SAFETY_PREEMPTED;
  }
  const auto gripper_port = backend_->gripper_port();
  if (gripper_port && gripper_port->active()) {
    gripper_port->stop();
  }
  backend_->safety_preempt(stop_mode);
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
    !execution_active && inactivity_confirmed && !task_execution_in_progress_)
  {
    status_.execution_state = MotionStatus::MOTION_STATE_IDLE;
    status_.has_active_execution = false;
    status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
  }
  if (
    status_.execution_state == MotionStatus::MOTION_STATE_STOPPING &&
    !execution_active && inactivity_confirmed)
  {
    status_.execution_state = MotionStatus::MOTION_STATE_STOPPED;
    status_.has_active_execution = false;
    status_.active_task_type.value = MotionTaskType::TASK_TYPE_UNSPECIFIED;
  }
}

bool MotionCore::set_runtime_tuning(
  const MotionTuning & tuning, const MotionGeometry & geometry,
  std::chrono::milliseconds holding_confirmation_timeout, bool idle_mutable)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (idle_mutable && (goal_submission_pending_ || task_execution_in_progress_ ||
    status_.has_active_execution || status_.execution_state == MotionStatus::MOTION_STATE_STOPPING)
  ) {
    return false;
  }
  runtime_tuning_ = tuning;
  runtime_geometry_ = geometry;
  holding_confirmation_timeout_ = holding_confirmation_timeout;
  return true;
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
  const ExecuteTask::Goal & goal, Clock::time_point now,
  std::string * diagnostic_detail) const
{
  if (!is_supported_task(goal.task_type.value)) {
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
    !capability_allows_task(safety_state_.motion_capability.value, goal.task_type.value))
  {
    add_failed_check("capability_does_not_allow_task");
  }
  if (!failed_checks.empty()) {
    set_diagnostic(
      diagnostic_detail,
      "Safety validation denied task_type=" + std::to_string(goal.task_type.value) +
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

}  // namespace arm_cell_motion_moveit2
