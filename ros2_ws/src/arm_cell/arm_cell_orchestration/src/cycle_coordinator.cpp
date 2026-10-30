#include "arm_cell_orchestration/cycle_coordinator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "arm_cell_orchestration/failure_trace_record.hpp"
#include "arm_cell_orchestration/recovery_wait_predicate.hpp"

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/decorator_node.h>
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <memory>
#include <sstream>
#include <mutex>
#include <string_view>
#include <stdexcept>
#include <thread>
#include <utility>

namespace arm_cell_orchestration
{

namespace
{
constexpr unsigned kGrootPublisherPort = 1666;
constexpr unsigned kGrootServerPort = 1667;

class AsyncTaskNode : public BT::AsyncActionNode
{
public:
  using Task = std::function<BT::NodeStatus()>;
  using Cancel = std::function<void ()>;
  using Submission = CycleCoordinator::Submission;
  using ScopedTask = std::function<BT::NodeStatus(const Submission &)>;

  AsyncTaskNode(
    const std::string & name, const BT::NodeConfiguration & config, Task task, Cancel cancel)
  : BT::AsyncActionNode(name, config), task_(std::move(task)), cancel_(std::move(cancel))
  {
  }

  AsyncTaskNode(const std::string & name, const BT::NodeConfiguration & config, ScopedTask task)
  : BT::AsyncActionNode(name, config), scoped_task_(std::move(task)) {}

  void halt() override
  {
    Submission submission;
    if (status() == BT::NodeStatus::RUNNING) {
      std::lock_guard<std::mutex> lock(submission_mutex_);
      halt_pending_ = true;
      submission = submission_;
    }
    if (submission) {submission->request_cancel();}
    if (status() == BT::NodeStatus::RUNNING && cancel_) {
      cancel_();
    }
    BT::AsyncActionNode::halt();
    std::lock_guard<std::mutex> lock(submission_mutex_);
    submission_.reset();
    halt_pending_ = false;
  }

protected:
  BT::NodeStatus tick() override
  {
    try {
      if (!scoped_task_) {return task_();}
      auto submission = std::make_shared<CancellationRegistry::ActionSubmission>();
      bool canceled;
      {
        std::lock_guard<std::mutex> lock(submission_mutex_);
        submission_ = submission;
        canceled = halt_pending_;
      }
      if (canceled) {submission->request_cancel();}
      const auto result = scoped_task_(submission);
      {
        std::lock_guard<std::mutex> lock(submission_mutex_);
        if (submission_ == submission) {submission_.reset();}
      }
      return result;
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  Task task_;
  Cancel cancel_;
  ScopedTask scoped_task_;
  std::mutex submission_mutex_;
  Submission submission_;
  bool halt_pending_{false};
};

class ApprovedAcquisitionNode : public BT::ControlNode
{
public:
  using ConsumeRetry = std::function<bool ()>;

  ApprovedAcquisitionNode(
    const std::string & name, const BT::NodeConfiguration & config, ConsumeRetry consume_retry)
  : BT::ControlNode(name, config), consume_retry_(std::move(consume_retry))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  void halt() override
  {
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 1U) {
      return BT::NodeStatus::FAILURE;
    }
    while (true) {
      const auto status = children_nodes_[0]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {
        return status;
      }
      resetChildren();
      if (status == BT::NodeStatus::SUCCESS) {
        if (consume_retry_) {consume_retry_();}
        return status;
      }
      if (!consume_retry_ || !consume_retry_()) {
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  ConsumeRetry consume_retry_;
};

class DisturbanceReacquisitionNode final : public BT::ControlNode
{
public:
  using ConsumeRetry = ApprovedAcquisitionNode::ConsumeRetry;
  using ClearRetry = std::function<void ()>;

  DisturbanceReacquisitionNode(
    const std::string & name, const BT::NodeConfiguration & config,
    ConsumeRetry consume_retry, ClearRetry clear_retry)
  : BT::ControlNode(name, config), consume_retry_(std::move(consume_retry)),
    clear_retry_(std::move(clear_retry)) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    started_ = false;
    if (clear_retry_) {clear_retry_();}
    BT::ControlNode::halt();
  }

private:
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 1U) {return BT::NodeStatus::FAILURE;}
    if (!started_) {
      if (clear_retry_) {clear_retry_();}
      started_ = true;
    }
    while (true) {
      const auto status = children_nodes_[0]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {return status;}
      resetChildren();
      if (status == BT::NodeStatus::SUCCESS) {
        if (clear_retry_) {clear_retry_();}
        started_ = false;
        return status;
      }
      if (!consume_retry_ || !consume_retry_()) {
        started_ = false;
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  ConsumeRetry consume_retry_;
  ClearRetry clear_retry_;
  bool started_{false};
};

class PnpWithWorkRecoveryNode final : public BT::ControlNode
{
public:
  enum class Decision : std::uint8_t {COMPLETED, RETRY, DISTURB, TERMINAL};
  using DecideOutcome = std::function<Decision()>;
  using IsDisturbance = std::function<bool ()>;

  PnpWithWorkRecoveryNode(
    const std::string & name, const BT::NodeConfiguration & config, DecideOutcome decide,
    DecideOutcome decide_disturbance, IsDisturbance is_disturbance)
  : BT::ControlNode(name, config), decide_(std::move(decide)),
    decide_disturbance_(std::move(decide_disturbance)),
    is_disturbance_(std::move(is_disturbance)) {}

  static BT::PortsList providedPorts() {return {};}

  void halt() override
  {
    stage_ = Stage::PNP;
    BT::ControlNode::halt();
  }

private:
  enum class Stage : std::uint8_t {PNP, HOME, ACQUISITION, DISTURBANCE_ACQUISITION};

  BT::NodeStatus tick() override
  {
    if (childrenCount() != 5U) {return BT::NodeStatus::FAILURE;}
    while (true) {
      const bool disturbance = stage_ == Stage::PNP && is_disturbance_ && is_disturbance_();
      const auto index = stage_ == Stage::PNP ? (disturbance ? 3U : 0U) :
        stage_ == Stage::HOME ? 1U : stage_ == Stage::ACQUISITION ? 2U : 4U;
      const auto status = children_nodes_[index]->executeTick();
      if (status == BT::NodeStatus::RUNNING) {return status;}
      haltChild(index);
      if (stage_ == Stage::PNP) {
        const auto decision = disturbance ?
          (decide_disturbance_ ? decide_disturbance_() : Decision::TERMINAL) :
          (decide_ ? decide_() : Decision::TERMINAL);
        if (disturbance && decision == Decision::RETRY &&
          status == BT::NodeStatus::SUCCESS)
        {
          stage_ = Stage::DISTURBANCE_ACQUISITION;
          continue;
        }
        if (decision == Decision::DISTURB) {continue;}
        if (decision == Decision::COMPLETED) {stage_ = Stage::PNP; return BT::NodeStatus::SUCCESS;}
        if (decision != Decision::RETRY || status == BT::NodeStatus::SUCCESS) {
          stage_ = Stage::PNP;
          return BT::NodeStatus::FAILURE;
        }
        stage_ = Stage::HOME;
        continue;
      }
      if (status != BT::NodeStatus::SUCCESS) {
        stage_ = Stage::PNP;
        return BT::NodeStatus::FAILURE;
      }
      stage_ = stage_ == Stage::HOME ? Stage::ACQUISITION : Stage::PNP;
    }
  }

  DecideOutcome decide_;
  DecideOutcome decide_disturbance_;
  IsDisturbance is_disturbance_;
  Stage stage_{Stage::PNP};
};

class TreeStatusObserver : public BT::StatusChangeLogger
{
public:
  TreeStatusObserver(BT::TreeNode * root, CycleCoordinator::TreeStatusFn observer)
  : BT::StatusChangeLogger(root), observer_(std::move(observer))
  {
  }

  void callback(
    BT::Duration, const BT::TreeNode & node, BT::NodeStatus previous,
    BT::NodeStatus status) override
  {
    static const std::array<std::string_view, 7> kExecutionBoundaries{
      "ValidateRecipe", "InitialHome", "DetectTarget", "ExecutePnp", "ExecuteDisturb",
      "RetryHome",
      "SafetyRecovery"};
    if (!observer_ || std::find(kExecutionBoundaries.begin(), kExecutionBoundaries.end(),
      node.name()) == kExecutionBoundaries.end())
    {
      return;
    }
    observer_(node.name(), BT::toStr(previous), BT::toStr(status));
  }

  void flush() override
  {
  }

private:
  CycleCoordinator::TreeStatusFn observer_;
};

class RepeatQuantityNode : public BT::DecoratorNode
{
public:
  using ProgressReader = std::function<std::uint64_t()>;
  RepeatQuantityNode(const std::string & name, const BT::NodeConfiguration & config,
    ProgressReader progress_reader)
  : BT::DecoratorNode(name, config), progress_reader_(std::move(progress_reader))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::uint32_t>("quantity")};
  }

  void halt() override
  {
    completed_ = 0U;
    BT::DecoratorNode::halt();
  }

protected:
  BT::NodeStatus tick() override
  {
    const auto quantity = getInput<std::uint32_t>("quantity");
    if (!quantity || !child_node_) {
      return BT::NodeStatus::FAILURE;
    }
    while (completed_ < quantity.value()) {
      const auto progress_before = progress_reader_ ? progress_reader_() : completed_;
      const auto child_status = child_node_->executeTick();
      if (child_status == BT::NodeStatus::SUCCESS) {
        const auto progress_after = progress_reader_ ? progress_reader_() : completed_ + 1U;
        if (progress_after > progress_before) {
          completed_ += static_cast<std::uint32_t>(progress_after - progress_before);
        }
        resetChild();
        continue;
      }
      if (child_status == BT::NodeStatus::FAILURE) {
        resetChild();
        return BT::NodeStatus::FAILURE;
      }
      return child_status;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::uint32_t completed_{0U};
  ProgressReader progress_reader_;
};

using DetectResult = arm_cell_interfaces::msg::DetectTargetResultCode;
using MotionType = arm_cell_interfaces::msg::MotionTaskType;
using MotionPhase = arm_cell_interfaces::msg::MotionTaskPhase;
constexpr double kPi = 3.14159265358979323846;
// Only transient observation failures consume this Target Episode budget.
constexpr uint32_t kMaximumVisionRetries = 3U;
// The Vision ICD assigns this additive result code; its generated enum is owned by WP E.
constexpr uint8_t kVisionBusyResultCode = 8U;

bool vision_slot_busy(uint8_t result_code)
{
  return result_code == kVisionBusyResultCode;
}

bool retryable_vision_failure(uint8_t result_code)
{
  return result_code == DetectResult::DETECT_RESULT_TEMPORARY_INVALID_TARGET ||
         result_code == DetectResult::DETECT_RESULT_TIMEOUT;
}

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

bool valid_unit_quaternion(const geometry_msgs::msg::Quaternion & orientation);

bool valid_material_candidate(
  const arm_cell_interfaces::msg::PickCandidate & response, bool require_rectangle)
{
  const auto & pose = response.target_pose;
  const bool valid_order = response.orientation_symmetry_order == 0 ||
    response.orientation_symmetry_order == 1 || response.orientation_symmetry_order == 2 ||
    response.orientation_symmetry_order == 4;
  const bool valid_observable_orientation = response.orientation_symmetry_order == 0 ||
    valid_unit_quaternion(pose.pose.orientation);
  const bool rectangle_required = require_rectangle || response.has_observed_rectangle ||
    response.rectangle_long_side_m != 0.0F || response.rectangle_short_side_m != 0.0F ||
    response.orientation_symmetry_order == 2 || response.orientation_symmetry_order == 4;
  const bool valid_rectangle = !rectangle_required ||
    (std::isfinite(response.rectangle_long_side_m) &&
    std::isfinite(response.rectangle_short_side_m) &&
    response.rectangle_short_side_m > 0.0F &&
    response.rectangle_long_side_m >= response.rectangle_short_side_m &&
    (response.orientation_symmetry_order != 2 ||
    response.rectangle_long_side_m > response.rectangle_short_side_m) &&
    response.has_estimated_height && std::isfinite(response.estimated_height_m) &&
    response.estimated_height_m > 0.0F);
  return !response.acquisition_id.empty() &&
         !response.candidate_id.empty() && valid_order &&
         valid_observable_orientation && valid_rectangle &&
         pose.header.frame_id == "base_link" &&
         std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y) &&
         std::isfinite(pose.pose.position.z);
}

bool valid_material_candidate_list(
  const CycleCoordinator::DetectTarget::Response & response, bool require_rectangle)
{
  // Validate the complete list; Task Execution owns execution priority.
  const auto & candidates = response.material_candidates;
  if (candidates.empty()) {return false;}
  std::vector<std::string> ids;
  for (const auto & candidate : candidates) {
    if (candidate.acquisition_id != response.acquisition_id ||
      candidate.candidate_id.empty() ||
      std::find(ids.begin(), ids.end(), candidate.candidate_id) != ids.end() ||
      !valid_material_candidate(candidate, require_rectangle))
    {return false;}
    ids.push_back(candidate.candidate_id);
  }
  return true;
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

}

CycleCoordinator::CycleCoordinator(Callbacks callbacks)
: callbacks_(std::move(callbacks))
{}

void CycleCoordinator::record_diagnostic(const DiagnosticRecord & record) const noexcept
{
  if (callbacks_.record_structured_diagnostic) {
    try {
      callbacks_.record_structured_diagnostic(record);
    } catch (...) {
      // Diagnostics are observational and cannot change Mission behavior.
    }
  }
  if (record.event == DiagnosticEvent::BT_NODE_STATUS)
  {
    return;
  }
  if (callbacks_.record_diagnostic) {
    const char * event = record.event ==
      DiagnosticEvent::CANCELLATION ? "cancellation_propagation" :
      record.event == DiagnosticEvent::RECOVERY ? "recovery_observation" :
      record.event == DiagnosticEvent::RETRACT ? "retract_decision" : "retry_decision";
    const char * reason = "observation_changed";
    switch (record.reason) {
      case DiagnosticReason::MISSION_INTERRUPTION: reason = "mission_interruption"; break;
      case DiagnosticReason::HIGHER_STOP_SEVERITY: reason = "higher_stop_severity"; break;
      case DiagnosticReason::EMERGENCY_STOP: reason = "emergency_stop"; break;
      case DiagnosticReason::RECOVERY_CAPABILITY_REMOVED: reason = "recovery_capability_removed";
        break;
      case DiagnosticReason::HOLDING_INVALID: reason = "holding_invalid"; break;
      case DiagnosticReason::BUDGET_AVAILABLE: reason = "budget_available"; break;
      case DiagnosticReason::RETRY_BUDGET_EXHAUSTED: reason = "retry_budget_exhausted"; break;
      case DiagnosticReason::NON_RETRYABLE: reason = "non_retryable"; break;
      case DiagnosticReason::RESULT_UNAVAILABLE: reason = "result_unavailable"; break;
      case DiagnosticReason::RETRY_ADMISSION: reason = "retry_admission"; break;
      case DiagnosticReason::MOTION_STATUS_UNAVAILABLE: reason = "motion_status_unavailable"; break;
      case DiagnosticReason::MOTION_NOT_IDLE: reason = "motion_not_idle"; break;
      case DiagnosticReason::MOTION_ACTIVE: reason = "motion_active"; break;
      case DiagnosticReason::HOLDING_NOT_RELEASED: reason = "holding_not_released"; break;
      case DiagnosticReason::RETRY_ADMISSION_FAILED: reason = "retry_admission_failed"; break;
      case DiagnosticReason::RETRY_SAFETY_UNAVAILABLE: reason = "retry_safety_unavailable"; break;
      case DiagnosticReason::RETRY_SAFETY_NOT_NORMAL: reason = "retry_safety_not_normal"; break;
      case DiagnosticReason::RETRY_STOP_NOT_CONFIRMED: reason = "retry_stop_not_confirmed"; break;
      case DiagnosticReason::RETRY_CANCEL_REQUESTED: reason = "retry_cancel_requested"; break;
      case DiagnosticReason::RECOVERY_AUTHORIZED_AND_STOPPED: reason =
          "recovery_authorized_and_stopped"; break;
      case DiagnosticReason::MISSION_CANCELLED: reason = "mission_cancelled"; break;
      case DiagnosticReason::SAFETY_CAPABILITY_WITHDRAWN: reason =
          "safety_capability_withdrawn"; break;
      case DiagnosticReason::UNEXPECTED_BUSY: reason = "unexpected_busy"; break;
      case DiagnosticReason::FEEDBACK_MISSING: reason = "feedback_missing"; break;
      case DiagnosticReason::FEEDBACK_RECOVERED: reason = "feedback_recovered"; break;
      case DiagnosticReason::ACTION_COMMUNICATION_UNKNOWN: reason = "action_communication_unknown";
        break;
      case DiagnosticReason::RELEASE_CONFIRMATION_TIMEOUT:
        reason = "release_confirmation_timeout"; break;
      case DiagnosticReason::NONE: break;
    }
    std::ostringstream message;
    message << "event=" << event << " reason=" << reason;
    if (record.event == DiagnosticEvent::RETRY) {
      message << " phase=" << (record.phase == DiagnosticPhase::VISION ? "Vision" : "PICK")
              << " decision=" << (record.decision == DiagnosticDecision::RETRY ? "retry" : "stop")
              << " attempt=" << record.attempt << " retry_budget=" << record.retry_budget
              << " failure=";
      if (!record.result_available) {
        message << "unavailable";
      } else if (record.failure_domain == DiagnosticFailureDomain::VISION) {
        message << detect_target_result_code_name(record.failure_code);
      } else {
        message << motion_result_code_name(record.failure_code);
      }
    } else if (record.event == DiagnosticEvent::CANCELLATION) {
      message << " mission_exit=" << mission_exit_reason_name(record.mission_exit_reason);
    } else if (record.event == DiagnosticEvent::RETRACT) {
      message << " decision=" <<
        (record.decision == DiagnosticDecision::ALLOWED ? "allowed" : "denied")
              << " wait=ended";
    } else if (record.event == DiagnosticEvent::RECOVERY) {
      const char * block = !record.safety_valid ? "Safety permission unavailable" :
        record.motion_active || record.has_active_execution ||
        !record.backend_inactivity_confirmed ?
        "awaiting confirmed Motion inactivity" :
        record.capability != arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY ?
        "awaiting Safety recovery authorization" : "evaluating stopped Motion and holding evidence";
      message << " condition=" << block;
    }
    try {
      callbacks_.record_diagnostic(message.str());
    } catch (...) {
      // Diagnostics are observational and cannot change Mission behavior.
    }
  }
}

bool CycleCoordinator::diagnostics_observed() const noexcept
{
  return static_cast<bool>(callbacks_.record_diagnostic) ||
         static_cast<bool>(callbacks_.record_structured_diagnostic);
}

void CycleCoordinator::set_cancel_requested(bool requested)
{
  cancel_requested_.store(requested);
}

SafetyObservation CycleCoordinator::read_safety() const
{
  return callbacks_.read_safety ? callbacks_.read_safety() : SafetyObservation{};
}

bool CycleCoordinator::normal_motion_allowed() const
{
  return normal_motion_allowed(read_safety());
}

bool CycleCoordinator::normal_motion_allowed(const SafetyObservation & observation) const
{
  return observation.normal_motion_allowed();
}

CycleCoordinator::Result CycleCoordinator::interrupted(
  MissionExitReason reason, const std::string & detail,
  std::optional<MotionFailureTrace> motion_failure)
{
  if (diagnostics_observed()) {
    DiagnosticRecord record;
    record.event = DiagnosticEvent::CANCELLATION;
    record.mission_exit_reason = reason.value;
    record.final_result = reason.value;
    record.reason = DiagnosticReason::MISSION_INTERRUPTION;
    if (motion_failure) {
      record.failure_domain = DiagnosticFailureDomain::MOTION;
      record.failure_code = motion_failure->code.value;
      record.result_available = motion_failure->result_code_available;
      record.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
        motion_failure->motion_execution_id.size(), record.motion_execution_id.size()));
      std::copy_n(
        motion_failure->motion_execution_id.begin(), record.motion_execution_id_size,
        record.motion_execution_id.begin());
      record.result_detail_size = static_cast<std::uint8_t>(std::min(
        motion_failure->diagnostic_detail.size(), record.result_detail.size()));
      std::copy_n(
        motion_failure->diagnostic_detail.begin(), record.result_detail_size,
        record.result_detail.begin());
      record.motion_action_terminal_size = static_cast<std::uint8_t>(std::min(
        motion_failure->action_terminal.size(), record.motion_action_terminal.size()));
      std::copy_n(
        motion_failure->action_terminal.begin(), record.motion_action_terminal_size,
        record.motion_action_terminal.begin());
    }
    record_diagnostic(record);
  }
  return Result{reason, detail, std::move(motion_failure), processed_count_};
}

CycleCoordinator::Result CycleCoordinator::safety_preempted(
  const SafetyObservation & entry_safety,
  std::optional<MotionFailureTrace> motion_failure)
{
  const bool motion_reported_safety_preemption = motion_failure &&
    motion_failure->code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
  if (!entry_safety.delivery_valid && !motion_reported_safety_preemption) {
    return Result{
      exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR),
      "Safety permission unavailable; mission progression withheld",
      std::move(motion_failure), processed_count_};
  }
  auto result = interrupted(
    exit_reason(MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED),
    "Safety preempted the mission", motion_failure);
  const auto recovery_entry = callbacks_.read_recovery_entry ? callbacks_.read_recovery_entry() :
    std::nullopt;
  recovery_entry_ = recovery_entry.value_or(entry_safety);
  return result;
}

void CycleCoordinator::coordinate_recovery(const SafetyObservation & entry_safety)
{
  if (!callbacks_.wait_for_recovery && !callbacks_.wait_for_recovery_cancellable) {
    return;
  }
  if (callbacks_.publish_phase) {
    callbacks_.publish_phase(MissionPhase::MISSION_PHASE_WAITING_RECOVERY);
  }

  bool recovery_authorization_seen = false;
  bool episode_retract_allowance_available = true;
  bool episode_held_return_attempted = false;
  const auto entry_causes = entry_safety.state.active_causes.value |
    entry_safety.state.latched_causes.value;
  const bool episode_was_estop = (entry_causes &
    arm_cell_interfaces::msg::SafetyCauseSet::E_STOP) != 0U;
  std::optional<uint8_t> episode_stop_severity;
  std::array<std::uint64_t, 7> previous_observation{};
  bool has_previous_observation = false;
  if (entry_safety.delivery_valid &&
    entry_safety.state.selected_stop_mode.value <= StopMode::STOP_MODE_EMERGENCY)
  {
    episode_stop_severity = entry_safety.state.selected_stop_mode.value;
  }
  while (true) {
    if (cancel_requested_.load()) {
      return;
    }
    RecoveryObservation observation;
    const bool observation_received = callbacks_.wait_for_recovery_cancellable ?
      callbacks_.wait_for_recovery_cancellable(
      [this]() {return cancel_requested_.load();}, observation) :
      (observation = callbacks_.wait_for_recovery(), true);
    if (!observation_received || cancel_requested_.load()) {
      return;
    }
    if (diagnostics_observed()) {
      const std::array<std::uint64_t, 7> signature{
        observation.safety.delivery_valid,
        observation.safety.state.motion_capability.value,
        observation.safety.state.selected_stop_mode.value,
        observation.motion.execution_state,
        observation.motion.execution_active,
        observation.motion.backend_inactivity_confirmed,
        observation.motion.holding_state};
      if (!has_previous_observation || signature != previous_observation) {
        previous_observation = signature;
        has_previous_observation = true;
        DiagnosticRecord record;
        record.event = DiagnosticEvent::RECOVERY;
        record.safety_valid = observation.safety.delivery_valid;
        record.capability = observation.safety.state.motion_capability.value;
        record.selected_stop_mode = observation.safety.state.selected_stop_mode.value;
        record.motion_state = observation.motion.execution_state;
        record.motion_active = observation.motion.execution_active;
        record.has_active_execution = observation.motion.has_active_execution;
        record.backend_inactivity_confirmed = observation.motion.backend_inactivity_confirmed;
        record.holding_state = observation.motion.holding_state;
        record.episode_retract_allowance = episode_retract_allowance_available;
        record.cancellation_requested = cancel_requested_.load();
        record_diagnostic(record);
      }
    }
    if (!observation.safety.delivery_valid) {
      continue;
    }

    const auto selected_stop_severity = observation.safety.state.selected_stop_mode.value;
    if (!episode_stop_severity) {
      episode_stop_severity = selected_stop_severity;
    } else if (selected_stop_severity > *episode_stop_severity) {
      if (diagnostics_observed()) {
        DiagnosticRecord record;
        record.event = DiagnosticEvent::RETRACT;
        record.decision = DiagnosticDecision::DENIED;
        record.reason = DiagnosticReason::HIGHER_STOP_SEVERITY;
        record.episode_retract_allowance = true;
        record.safety_valid = observation.safety.delivery_valid;
        record.capability = observation.safety.state.motion_capability.value;
        record.selected_stop_mode = selected_stop_severity;
        record.motion_state = observation.motion.execution_state;
        record.motion_active = observation.motion.execution_active;
        record.has_active_execution = observation.motion.has_active_execution;
        record.backend_inactivity_confirmed = observation.motion.backend_inactivity_confirmed;
        record.holding_state = observation.motion.holding_state;
        record_diagnostic(record);
      }
      return;
    }
    if (selected_stop_severity == StopMode::STOP_MODE_EMERGENCY && !episode_was_estop) {
      if (diagnostics_observed()) {
        DiagnosticRecord record;
        record.event = DiagnosticEvent::RETRACT;
        record.decision = DiagnosticDecision::DENIED;
        record.reason = DiagnosticReason::EMERGENCY_STOP;
        record.episode_retract_allowance = true;
        record.safety_valid = observation.safety.delivery_valid;
        record.capability = observation.safety.state.motion_capability.value;
        record.selected_stop_mode = selected_stop_severity;
        record.motion_state = observation.motion.execution_state;
        record.motion_active = observation.motion.execution_active;
        record.has_active_execution = observation.motion.has_active_execution;
        record.backend_inactivity_confirmed = observation.motion.backend_inactivity_confirmed;
        record.holding_state = observation.motion.holding_state;
        record_diagnostic(record);
      }
      return;
    }

    if (episode_was_estop &&
      observation.safety.normal_motion_allowed() && observation.motion_status_fresh &&
      observation.motion.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
      !observation.motion.execution_active && !observation.motion.has_active_execution &&
      observation.motion.backend_inactivity_confirmed &&
      observation.motion.holding_state == MotionStatus::HOLDING_HELD)
    {
      if (episode_held_return_attempted) {return;}
      episode_held_return_attempted = true;
      if (callbacks_.publish_phase) {
        callbacks_.publish_phase(MissionPhase::MISSION_PHASE_RETRACTING);
      }
      const auto goal = motion_goal(MotionType::TASK_TYPE_RETURN_HELD);
      const auto result = callbacks_.execute_task(goal);
      recovery_held_return_result_ = result;
      if (callbacks_.record_recovery_result) {
        callbacks_.record_recovery_result(result);
      }
      return;
    }

    if (recovery_ready_for_batch_continuation(
        observation.safety, observation.motion, observation.motion_status_fresh))
    {
      return;
    }
    if (!observation.motion_status_fresh) {
      continue;
    }

    const auto recovery_authorized = observation.safety.recovery_motion_allowed();
    if (recovery_authorization_seen && !recovery_authorized) {
      if (diagnostics_observed()) {
        DiagnosticRecord record;
        record.event = DiagnosticEvent::RETRACT;
        record.decision = DiagnosticDecision::DENIED;
        record.reason = DiagnosticReason::RECOVERY_CAPABILITY_REMOVED;
        record.episode_retract_allowance = true;
        record.safety_valid = observation.safety.delivery_valid;
        record.capability = observation.safety.state.motion_capability.value;
        record.selected_stop_mode = selected_stop_severity;
        record.motion_state = observation.motion.execution_state;
        record.motion_active = observation.motion.execution_active;
        record.has_active_execution = observation.motion.has_active_execution;
        record.backend_inactivity_confirmed = observation.motion.backend_inactivity_confirmed;
        record.holding_state = observation.motion.holding_state;
        record_diagnostic(record);
      }
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
      if (diagnostics_observed()) {
        DiagnosticRecord record;
        record.event = DiagnosticEvent::RETRACT;
        record.decision = DiagnosticDecision::DENIED;
        record.reason = DiagnosticReason::HOLDING_INVALID;
        record.episode_retract_allowance = true;
        record.safety_valid = observation.safety.delivery_valid;
        record.capability = observation.safety.state.motion_capability.value;
        record.selected_stop_mode = selected_stop_severity;
        record.motion_state = observation.motion.execution_state;
        record.motion_active = observation.motion.execution_active;
        record.has_active_execution = observation.motion.has_active_execution;
        record.backend_inactivity_confirmed = observation.motion.backend_inactivity_confirmed;
        record.holding_state = observation.motion.holding_state;
        record_diagnostic(record);
      }
      return;
    }

    episode_retract_allowance_available = false;
    if (diagnostics_observed()) {
      DiagnosticRecord record;
      record.event = DiagnosticEvent::RETRACT;
      record.decision = DiagnosticDecision::ALLOWED;
      record.reason = DiagnosticReason::RECOVERY_AUTHORIZED_AND_STOPPED;
      record.episode_retract_allowance = false;
      record.safety_valid = true;
      record.capability = observation.safety.state.motion_capability.value;
      record.selected_stop_mode = observation.safety.state.selected_stop_mode.value;
      record.motion_state = observation.motion.execution_state;
      record.motion_active = observation.motion.execution_active;
      record.backend_inactivity_confirmed = observation.motion.backend_inactivity_confirmed;
      record.holding_state = observation.motion.holding_state;
      record_diagnostic(record);
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
  const auto safety = read_safety();
  if (!normal_motion_allowed(safety)) {
    return safety_preempted(safety, std::move(motion_failure));
  }
  if (reason.value != MissionExitReason::MISSION_EXIT_MOTION_ERROR &&
    reason.value != MissionExitReason::MISSION_EXIT_CANCELED)
  {
    motion_failure.reset();
  }
  return Result{reason, detail, std::move(motion_failure), processed_count_};
}

CycleCoordinator::MotionResult CycleCoordinator::execute_motion(
  const ExecuteTask::Goal & goal, uint8_t phase, const MotionFeedbackFn & feedback,
  const Submission & submission)
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
        "Caller cancellation requested", 0U, {}});
  }
  if (!normal_motion_allowed()) {
    return with_task_type(
      MotionResult{
        motion_result_code(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED),
        "Safety capability no longer permits normal motion", 0U, {}});
  }
  if (!callbacks_.execute_task) {
    return with_task_type(
      MotionResult{
        motion_result_code(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE),
        "Motion client unavailable", 0U, {}});
  }
  if (callbacks_.execute_task_scoped) {
    return with_task_type(callbacks_.execute_task_scoped(goal, feedback, submission));
  }
  if (callbacks_.execute_task_with_feedback) {
    return with_task_type(callbacks_.execute_task_with_feedback(goal, feedback));
  }
  return with_task_type(callbacks_.execute_task(goal));
}

CycleCoordinator::PnpResult CycleCoordinator::execute_pnp(
  const ExecutePnp::Goal & goal, uint8_t phase, const PnpFeedbackFn & feedback,
  const Submission & submission)
{
  if (callbacks_.publish_phase) {callbacks_.publish_phase(phase);}
  if (cancel_requested_.load()) {
    return PnpResult{ExecutePnp::Result::PNP_OUTCOME_FAILED, false,
      motion_result_code(MotionTaskResultCode::TASK_RESULT_CANCELED),
      "Caller cancellation requested", {}, "CANCELED"};
  }
  if (!normal_motion_allowed()) {
    return PnpResult{ExecutePnp::Result::PNP_OUTCOME_SAFETY_PREEMPTED, false,
      motion_result_code(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED),
      "Safety capability no longer permits normal motion", {}, "ABORTED"};
  }
  if (callbacks_.execute_pnp_scoped) {return callbacks_.execute_pnp_scoped(goal, feedback, submission);}
  if (callbacks_.execute_pnp_with_feedback) {
    return callbacks_.execute_pnp_with_feedback(goal, feedback);
  }
  if (callbacks_.execute_pnp) {return callbacks_.execute_pnp(goal);}
  return PnpResult{ExecutePnp::Result::PNP_OUTCOME_FAILED, false,
    motion_result_code(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE),
    "ExecutePnp client unavailable", {}, "UNKNOWN"};
}

CycleCoordinator::DisturbResult CycleCoordinator::execute_disturb(
  const ExecuteDisturb::Goal & goal, const Submission & submission)
{
  if (cancel_requested_.load()) {
    return DisturbResult{arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_CANCELED,
      "Caller cancellation requested", "CANCELED"};
  }
  if (!normal_motion_allowed()) {
    return DisturbResult{arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_SAFETY_PREEMPTED,
      "Safety capability no longer permits normal motion", "ABORTED"};
  }
  if (callbacks_.execute_disturb_scoped) {return callbacks_.execute_disturb_scoped(goal, submission);}
  return callbacks_.execute_disturb ? callbacks_.execute_disturb(goal) :
         DisturbResult{arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED,
           "ExecuteDisturb client unavailable", "UNKNOWN"};
}

CycleCoordinator::TargetAcquisition CycleCoordinator::acquire_target(
  const std::string & target_id, const std::string & algorithm,
  const CancelPredicateFn & extra_cancel)
{
  if (cancel_requested_.load() || (extra_cancel && extra_cancel())) {
    return {};
  }
  DetectTarget::Request request;
  request.target_id = target_id;
  request.algorithm = algorithm;
  auto detected = callbacks_.detect_target_cancellable ?
    callbacks_.detect_target_cancellable(request, extra_cancel) :
    callbacks_.detect_target ? callbacks_.detect_target(request) : DetectTarget::Response{};
  if (cancel_requested_.load() || (extra_cancel && extra_cancel())) {
    return {};
  }
  if (!extra_cancel && callbacks_.read_safety && !normal_motion_allowed()) {
    return {};
  }
  if (callbacks_.record_vision_result && !vision_slot_busy(detected.result_code.value)) {
    callbacks_.record_vision_result(detected.result_code.value);
  }
  return TargetAcquisition{detected};
}

CycleCoordinator::MotionFailureTrace CycleCoordinator::motion_failure_trace(
  const MotionResult & result)
{
  return MotionFailureTrace{
    result.task_type, result.code, result.diagnostic_detail, result.motion_execution_id,
    result.result_code_available, result.action_terminal, result.object_disposition};
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

CycleCoordinator::ExecutePnp::Goal CycleCoordinator::pnp_goal(
  const DetectTarget::Response & response, const MissionRecipe & recipe)
{
  ExecutePnp::Goal goal;
  goal.vision_acquisition_id = response.acquisition_id;
  goal.material_candidates = response.material_candidates;
  goal.place_target_pose = recipe.place_pose;
  goal.has_place_target_pose = recipe.has_place_pose;
  goal.pick_approach_translation_direction =
    recipe.pick_approach_axis == PickApproachAxis::PLANNING_FRAME_VERTICAL ?
    ExecutePnp::Goal::PICK_APPROACH_PLANNING_FRAME_VERTICAL :
    ExecutePnp::Goal::PICK_APPROACH_TCP_INSERTION_AXIS;
  goal.pick_approach_distance_m = recipe.pick_approach_distance_m;
  goal.pick_grasp_position_tolerance_m = recipe.pick_grasp_position_tolerance_m;
  goal.pick_grasp_orientation_tolerance_rad = recipe.pick_grasp_orientation_tolerance_rad;
  goal.grasp_width_min_mm = recipe.grasp_width_min_mm;
  goal.grasp_width_max_mm = recipe.grasp_width_max_mm;
  if (recipe.unobserved_yaw_policy == UnobservedYawPolicy::ALLOWED &&
    std::any_of(response.material_candidates.begin(), response.material_candidates.end(),
      [](const auto & candidate) {return candidate.orientation_symmetry_order == 0;}))
  {
    goal.has_unobserved_yaw_fallback = true;
    goal.unobserved_yaw_fallback_rad = recipe.grasp_yaw_rad;
  }
  goal.place_position_tolerance_m = recipe.place_position_tolerance_m;
  goal.place_orientation_tolerance_rad = recipe.place_orientation_tolerance_rad;
  switch (recipe.place_orientation_constraint) {
    case PlaceOrientationConstraint::FIXED:
      goal.place_orientation_constraint = ExecutePnp::Goal::PLACE_ORIENTATION_FIXED;
      break;
    case PlaceOrientationConstraint::BOUNDED:
      goal.place_orientation_constraint = ExecutePnp::Goal::PLACE_ORIENTATION_BOUNDED;
      break;
    case PlaceOrientationConstraint::FREE:
      goal.place_orientation_constraint = ExecutePnp::Goal::PLACE_ORIENTATION_FREE;
      break;
  }
  goal.place_approach_direction_object = recipe.place_approach_direction_object;
  goal.place_approach_distance_m = recipe.place_approach_distance_m;
  goal.place_retract_distance_m = recipe.retract_distance_m;
  goal.has_place_tool_orientation_preference = recipe.has_place_tool_orientation_preference;
  goal.place_tool_orientation_preference = recipe.place_tool_orientation_preference;
  return goal;
}

bool CycleCoordinator::successful(const MotionResult & result)
{
  return result.result_code_available &&
         result.code.value == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
         (result.action_terminal.empty() || result.action_terminal == "SUCCEEDED");
}

CycleCoordinator::Result CycleCoordinator::execute(
  const std::string & target_id, uint32_t quantity)
{
  if (quantity == 0U) {
    return commit_non_safety_terminal(
      exit_reason(MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR),
      "Batch quantity must be positive");
  }

  processed_count_ = 0U;
  recovery_entry_.reset();
  recovery_held_return_result_.reset();
  struct MissionState
  {
    std::optional<DetectTarget::Response> acquisition;
    std::optional<DetectTarget::Response> disturbance_acquisition;
    std::optional<DisturbResult> disturbance_result;
    std::optional<DetectTarget::Response> first_vision_failure;
    std::optional<Result> terminal;
    std::optional<PnpResult> pnp_result;
    std::unordered_set<std::string> accounted_pnp_goals;
    bool retry_acquisition{false};
    bool retry_request_pending{false};
    bool disturbance_pending{false};
  } state;
  std::optional<MissionRecipe> recipe;
  std::optional<Result> terminal_result;
  auto blackboard = BT::Blackboard::create();
  blackboard->set("quantity", quantity);

  const auto recipe_is_valid = [](const std::optional<MissionRecipe> & candidate) {
      const bool valid_pick_approach_axis = candidate &&
        (candidate->pick_approach_axis == PickApproachAxis::TCP_INSERTION_AXIS ||
        candidate->pick_approach_axis == PickApproachAxis::PLANNING_FRAME_VERTICAL);
      const bool valid_pick_grasp_envelope = candidate &&
        std::isfinite(candidate->pick_approach_distance_m) &&
        candidate->pick_approach_distance_m > 0.0 &&
        std::isfinite(candidate->pick_grasp_position_tolerance_m) &&
        candidate->pick_grasp_position_tolerance_m > 0.0 &&
        std::isfinite(candidate->pick_grasp_orientation_tolerance_rad) &&
        candidate->pick_grasp_orientation_tolerance_rad > 0.0 &&
        candidate->pick_grasp_orientation_tolerance_rad <= kPi;
      return candidate &&
             candidate->vision_algorithm == "cuboid_top_face" &&
             candidate->has_grasp_width_policy && candidate->has_grasp_yaw &&
             candidate->has_unobserved_yaw_policy && candidate->has_place_pose &&
             std::isfinite(candidate->grasp_width_min_mm) &&
             std::isfinite(candidate->grasp_width_max_mm) &&
             valid_place_pose(candidate->place_pose) &&
             std::isfinite(candidate->place_position_tolerance_m) &&
             candidate->place_position_tolerance_m >= 0.0 &&
             std::isfinite(candidate->place_orientation_tolerance_rad) &&
             candidate->place_orientation_tolerance_rad >= 0.0 &&
             valid_orientation_constraint(candidate->place_orientation_constraint) &&
             !(candidate->place_orientation_constraint == PlaceOrientationConstraint::FREE &&
             candidate->has_place_tool_orientation_preference) &&
             (!candidate->has_place_tool_orientation_preference ||
             valid_unit_quaternion(candidate->place_tool_orientation_preference)) &&
             valid_unit_direction(candidate->place_approach_direction_object) &&
             std::isfinite(candidate->place_approach_distance_m) &&
             candidate->place_approach_distance_m > 0.0 &&
             std::isfinite(candidate->retract_distance_m) && candidate->retract_distance_m > 0.0 &&
             std::isfinite(candidate->grasp_yaw_rad) && valid_pick_approach_axis &&
             valid_pick_grasp_envelope && candidate->grasp_width_min_mm > 0.0F &&
             candidate->grasp_width_max_mm >= candidate->grasp_width_min_mm &&
             candidate->grasp_width_max_mm <= 85.0F;
    };

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<RepeatQuantityNode>("RepeatQuantity",
    [this](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<RepeatQuantityNode>(name, config,
        [this]() {return processed_count_;});
    });
  factory.registerBuilder<ApprovedAcquisitionNode>(
    "ApprovedAcquisition",
    [this, &state](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<ApprovedAcquisitionNode>(
        name, config, [&state]() {
          const bool approved = state.retry_acquisition;
          state.retry_acquisition = false;
          return approved;
        });
    });
  factory.registerBuilder<DisturbanceReacquisitionNode>(
    "DisturbanceReacquisition",
    [this, &state](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<DisturbanceReacquisitionNode>(
        name, config, [&state]() {
          const bool approved = state.retry_acquisition;
          state.retry_acquisition = false;
          return approved;
        }, [&state]() {state.retry_acquisition = false;});
    });
  factory.registerBuilder<PnpWithWorkRecoveryNode>(
    "PnpWithWorkRecovery",
    [this, &state, quantity](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<PnpWithWorkRecoveryNode>(
        name, config, [this, &state, quantity]() {
          if (!state.pnp_result) {
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR),
              "ExecutePnp returned no terminal result");
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          const auto & pnp = *state.pnp_result;
          MotionResult detail{pnp.detail_code, pnp.diagnostic_detail, 0U, pnp.goal_uuid,
            pnp.result_code_available, pnp.action_terminal,
            pnp.material_placed ? ObjectDisposition::OBJECT_DISPOSITION_PLACED :
            ObjectDisposition::OBJECT_DISPOSITION_UNCHANGED};
          if (pnp.material_placed) {
            if (pnp.goal_uuid.empty() || state.accounted_pnp_goals.insert(pnp.goal_uuid).second) {
              ++processed_count_;
              if (callbacks_.publish_processed_count) {
                callbacks_.publish_processed_count(processed_count_);
              }
            }
          }
          if (pnp.safety_handoff || pnp.safety_preempted ||
          pnp.outcome == ExecutePnp::Result::PNP_OUTCOME_SAFETY_PREEMPTED ||
          !normal_motion_allowed())
          {
            const auto safety_entry = callbacks_.read_recovery_entry ?
              callbacks_.read_recovery_entry() : std::nullopt;
            state.terminal = safety_preempted(
              safety_entry.value_or(read_safety()), motion_failure_trace(detail));
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          if (cancel_requested_.load() || pnp.action_terminal == "CANCELED") {
            state.terminal = interrupted(
              exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
              "Caller canceled while awaiting ExecutePnp terminal result",
              motion_failure_trace(detail));
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          if (pnp.outcome == ExecutePnp::Result::PNP_OUTCOME_COMPLETED &&
            pnp.material_placed)
          {
            state.acquisition.reset();
            return PnpWithWorkRecoveryNode::Decision::COMPLETED;
          }
          if (pnp.outcome == ExecutePnp::Result::PNP_OUTCOME_CANDIDATES_EXHAUSTED &&
          !pnp.material_placed && state.disturbance_acquisition &&
          !state.disturbance_acquisition->disturbance_candidates.empty() &&
          processed_count_ <= quantity && quantity - processed_count_ >= 3U)
          {
            state.acquisition.reset();
            state.disturbance_pending = true;
            return PnpWithWorkRecoveryNode::Decision::DISTURB;
          }
          if ((pnp.outcome == ExecutePnp::Result::PNP_OUTCOME_RETRYABLE ||
          pnp.outcome == ExecutePnp::Result::PNP_OUTCOME_CANDIDATES_EXHAUSTED) &&
            !pnp.material_placed)
          {
            state.acquisition.reset();
            state.retry_acquisition = true;
            return PnpWithWorkRecoveryNode::Decision::RETRY;
          }
          state.terminal = commit_non_safety_terminal(
            exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR), pnp.diagnostic_detail.empty() ?
            "ExecutePnp reported a non-retryable terminal result" : pnp.diagnostic_detail,
            motion_failure_trace(detail));
          return PnpWithWorkRecoveryNode::Decision::TERMINAL;
        }, [this, &state]() {
          if (!state.disturbance_pending || !state.disturbance_acquisition) {
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
              "ExecuteDisturb requires a valid unconfirmed Acquisition");
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          if (!state.disturbance_result) {
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR),
              "ExecuteDisturb returned no terminal result");
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          const auto & result = *state.disturbance_result;
          if (result.safety_handoff || result.safety_preempted || result.outcome ==
            arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_SAFETY_PREEMPTED ||
            !normal_motion_allowed())
          {
            const auto safety_entry = callbacks_.read_recovery_entry ?
              callbacks_.read_recovery_entry() : std::nullopt;
            state.terminal = safety_preempted(safety_entry.value_or(read_safety()));
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          if (cancel_requested_.load() ||
          result.outcome == arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_CANCELED ||
          result.action_terminal == "CANCELED")
          {
            state.terminal = interrupted(
              exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
              result.diagnostic_detail.empty() ?
              "Caller canceled while awaiting ExecuteDisturb terminal result" :
              result.diagnostic_detail);
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          if (result.outcome != arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_COMPLETED ||
          result.action_terminal != "SUCCEEDED")
          {
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR),
              result.diagnostic_detail.empty() ?
              "ExecuteDisturb reported a non-completed terminal result" :
              result.diagnostic_detail);
            return PnpWithWorkRecoveryNode::Decision::TERMINAL;
          }
          state.disturbance_pending = false;
          state.disturbance_acquisition.reset();
          state.disturbance_result.reset();
          state.acquisition.reset();
          return PnpWithWorkRecoveryNode::Decision::RETRY;
        }, [&state]() {return state.disturbance_pending;});
    });
  factory.registerSimpleAction(
    "ValidateRecipe", [&](BT::TreeNode &) {
      recipe = callbacks_.load_recipe ? callbacks_.load_recipe(target_id) :
      std::optional<MissionRecipe>{};
      if (!recipe_is_valid(recipe)) {
        state.terminal = commit_non_safety_terminal(
          exit_reason(MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR),
          "Mission recipe has invalid Vision algorithm, grasp policy, grasp yaw, or PLACE pose");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    });
  const auto finish_motion = [this, &state](const MotionResult & result) {
      const auto safety = read_safety();
      const bool capability_withheld =
        result.diagnostic_detail == "Safety capability no longer permits normal motion";
      if (capability_withheld) {
        state.terminal = safety_preempted(safety);
        return;
      }
      if (result.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ||
        !normal_motion_allowed(safety))
      {
        state.terminal = safety_preempted(safety, motion_failure_trace(result));
      } else if (cancel_requested_.load()) {
        state.terminal = Result{
          exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
          "Caller canceled while awaiting Motion result", motion_failure_trace(result),
          processed_count_};
      } else {
        state.terminal = commit_non_safety_terminal(
          motion_failure_exit_reason(result), result.diagnostic_detail,
          motion_failure_trace(result));
      }
    };
  const auto motion_leaf = [this, &state, finish_motion, &factory](
    const std::string & name, uint8_t task_type, uint8_t phase,
    std::function<bool()> should_run = {}) {
      factory.registerBuilder<AsyncTaskNode>(
        name,
        [this, &state, finish_motion, task_type, phase, should_run](const std::string & instance,
        const BT::NodeConfiguration & config) {
          return std::make_unique<AsyncTaskNode>(
            instance, config,
            [this, &state, finish_motion, task_type, phase, should_run](const Submission & submission) {
              if (cancel_requested_.load()) {
                state.terminal = interrupted(
                  exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
                  "Caller canceled before Motion action");
                return BT::NodeStatus::FAILURE;
              }
              if (should_run && !should_run()) {
                return BT::NodeStatus::SUCCESS;
              }
              const auto result = execute_motion(motion_goal(task_type), phase, {}, submission);
              if (!successful(result)) {
                finish_motion(result);
                return BT::NodeStatus::FAILURE;
              }
              return BT::NodeStatus::SUCCESS;
            });
        });
    };
  motion_leaf(
    "InitialHome", MotionType::TASK_TYPE_GO_HOME,
    MissionPhase::MISSION_PHASE_ENSURING_HOME);
  motion_leaf(
    "RetryHome", MotionType::TASK_TYPE_GO_HOME,
    MissionPhase::MISSION_PHASE_ENSURING_HOME);
  factory.registerBuilder<AsyncTaskNode>(
    "DetectTarget",
    [this, &state, &target_id, quantity, &recipe](const std::string & name,
    const BT::NodeConfiguration & config) {
      return std::make_unique<AsyncTaskNode>(
        name, config, [this, &state, &target_id, quantity, &recipe]() {
          if (cancel_requested_.load()) {
            state.terminal = interrupted(
              exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
              "Caller canceled before Vision acquisition");
            return BT::NodeStatus::FAILURE;
          }
          if (state.retry_request_pending) {
            const auto safety = read_safety();
            if (!normal_motion_allowed(safety)) {
              state.terminal = safety_preempted(safety);
              return BT::NodeStatus::FAILURE;
            }
            state.retry_request_pending = false;
          }
          if (callbacks_.publish_phase) {
            callbacks_.publish_phase(MissionPhase::MISSION_PHASE_DETECTING_TARGET);
          }
          auto acquired = acquire_target(
            target_id, recipe->vision_algorithm, [this]() {
              return cancel_requested_.load() || !normal_motion_allowed();
            });
          if (!acquired.response) {
            if (cancel_requested_.load()) {
              state.terminal = interrupted(
                exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
                "Caller canceled during Vision acquisition");
            } else if (!normal_motion_allowed()) {
              state.terminal = safety_preempted(read_safety());
            } else {
              state.terminal = commit_non_safety_terminal(
                exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
                "Vision acquisition ended without a terminal result");
            }
            return BT::NodeStatus::FAILURE;
          }
          const auto & detected = *acquired.response;
          if (vision_slot_busy(detected.result_code.value)) {
            const auto generation = callbacks_.read_vision_diagnostic_generation ?
            callbacks_.read_vision_diagnostic_generation() : 0U;
            const bool idle = callbacks_.wait_for_vision_idle && callbacks_.wait_for_vision_idle(
              generation, [this]() {
                return cancel_requested_.load() || !normal_motion_allowed();
              });
            if (cancel_requested_.load()) {
              state.terminal = interrupted(
                exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
                "Caller canceled while waiting for Vision IDLE");
              return BT::NodeStatus::FAILURE;
            }
            if (!normal_motion_allowed()) {
              state.terminal = safety_preempted(read_safety());
              return BT::NodeStatus::FAILURE;
            }
            if (!idle) {
              state.terminal = commit_non_safety_terminal(
                exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
                "Vision IDLE was not observed before the bounded wait expired");
              return BT::NodeStatus::FAILURE;
            }
            state.retry_request_pending = true;
            state.retry_acquisition = true;
            return BT::NodeStatus::FAILURE;
          }
          if (detected.result_code.value == DetectResult::DETECT_RESULT_OBJECT_NOT_FOUND) {
            state.terminal = commit_non_safety_terminal(
              exit_reason(
                processed_count_ < quantity ?
                MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL :
                MissionExitReason::MISSION_EXIT_VISION_ERROR), detected.diagnostic_detail);
            if (callbacks_.publish_phase) {
              callbacks_.publish_phase(MissionPhase::MISSION_PHASE_FINISHED);
            }
            return BT::NodeStatus::FAILURE;
          }
          const bool sufficient_disturbance_remainder = processed_count_ <= quantity &&
          static_cast<std::uint64_t>(quantity) - processed_count_ >= 3U;
          if (detected.result_code.value ==
          DetectResult::DETECT_RESULT_GEOMETRY_ERROR &&
          detected.material_candidates.empty() &&
          !detected.disturbance_candidates.empty() &&
          sufficient_disturbance_remainder)
          {
            state.acquisition.reset();
            state.disturbance_acquisition = detected;
            state.disturbance_pending = true;
            return BT::NodeStatus::SUCCESS;
          }
          if (detected.result_code.value != DetectResult::DETECT_RESULT_SUCCESS) {
            const bool retryable = retryable_vision_failure(detected.result_code.value);
            if (retryable && !state.first_vision_failure) {
              state.first_vision_failure = detected;
            }
            const bool approved = retryable && episode_retry_count_ < kMaximumVisionRetries &&
            !cancel_requested_.load() && normal_motion_allowed();
            if (diagnostics_observed()) {
              DiagnosticRecord record;
              record.event = DiagnosticEvent::RETRY;
              record.phase = DiagnosticPhase::VISION;
              record.failure_domain = DiagnosticFailureDomain::VISION;
              record.failure_code = detected.result_code.value;
              if (retryable && !approved && state.first_vision_failure) {
                record.original_failure_code = state.first_vision_failure->result_code.value;
              }
              record.attempt = episode_retry_count_ + 1U;
              record.retry_budget = kMaximumVisionRetries;
              record.retryable = retryable;
              record.decision = approved ? DiagnosticDecision::RETRY : DiagnosticDecision::STOP;
              record.reason = approved ? DiagnosticReason::BUDGET_AVAILABLE : retryable ?
              DiagnosticReason::RETRY_BUDGET_EXHAUSTED : DiagnosticReason::NON_RETRYABLE;
              record_diagnostic(record);
            }
            if (approved) {
              ++episode_retry_count_;
              if (callbacks_.publish_retry_count) {
                callbacks_.publish_retry_count(static_cast<uint8_t>(episode_retry_count_));
              }
              state.retry_request_pending = true;
              state.retry_acquisition = true;
              return BT::NodeStatus::FAILURE;
            }
            if (retryable && state.first_vision_failure && callbacks_.record_vision_result) {
              callbacks_.record_vision_result(state.first_vision_failure->result_code.value);
            }
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
              retryable && state.first_vision_failure ?
              state.first_vision_failure->diagnostic_detail : detected.diagnostic_detail);
            return BT::NodeStatus::FAILURE;
          }
          if (!valid_material_candidate_list(detected, target_id == "RawPart")) {
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
              "Vision returned an invalid Material Candidate list");
            return BT::NodeStatus::FAILURE;
          }
          state.disturbance_pending = false;
          state.disturbance_acquisition = detected;
          state.acquisition = detected;
          return BT::NodeStatus::SUCCESS;
        }, []() {});
    });
  factory.registerSimpleAction(
    "EvaluateAcquisition", [this, &state, &target_id, &recipe](BT::TreeNode &) {
      if (state.disturbance_pending) {
        return BT::NodeStatus::SUCCESS;
      }
      if (!state.acquisition || !recipe) {
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    });
  factory.registerSimpleAction(
    "BeginTargetEpisode", [this, &state](BT::TreeNode &) {
      episode_retry_count_ = 0U;
      state.first_vision_failure.reset();
      state.retry_acquisition = false;
      state.retry_request_pending = false;
      if (callbacks_.publish_retry_count) {
        callbacks_.publish_retry_count(0U);
      }
      return BT::NodeStatus::SUCCESS;
    });
  factory.registerBuilder<AsyncTaskNode>(
    "ExecuteDisturb",
    [this, &state](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<AsyncTaskNode>(
        name, config, [this, &state](const Submission & submission) {
          if (!state.disturbance_pending || !state.disturbance_acquisition) {
            return BT::NodeStatus::FAILURE;
          }
          const auto & acquisition = *state.disturbance_acquisition;
          ExecuteDisturb::Goal goal;
          goal.acquisition_id = acquisition.acquisition_id;
          goal.scene_generation = acquisition.scene_generation;
          goal.disturbance_candidates = acquisition.disturbance_candidates;
          state.disturbance_result = execute_disturb(goal, submission);
          const auto & result = *state.disturbance_result;
          if (result.outcome == arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_COMPLETED &&
          result.action_terminal == "SUCCEEDED")
          {
            return BT::NodeStatus::SUCCESS;
          }
          return BT::NodeStatus::FAILURE;
        });
    });
  factory.registerBuilder<AsyncTaskNode>(
    "ExecutePnp",
    [this, &state, &target_id, &recipe, quantity](const std::string & name,
    const BT::NodeConfiguration & config) {
      return std::make_unique<AsyncTaskNode>(
        name, config, [this, &state, &target_id, &recipe, quantity](const Submission & submission) {
          if (!state.acquisition || !recipe) {
            state.terminal = commit_non_safety_terminal(
              exit_reason(MissionExitReason::MISSION_EXIT_VISION_ERROR),
              "ExecutePnp requires a validated Acquisition and Recipe");
            return BT::NodeStatus::FAILURE;
          }
          state.pnp_result = execute_pnp(
            pnp_goal(*state.acquisition, *recipe), MissionPhase::MISSION_PHASE_PICKING, {}, submission);
          return state.pnp_result->outcome ==
            ExecutePnp::Result::PNP_OUTCOME_COMPLETED && state.pnp_result->material_placed ?
            BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        });
    });
  factory.registerSimpleAction(
    "CommitMission", [&](BT::TreeNode &) {
      terminal_result = commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_QUANTITY_REACHED),
        "Batch quantity reached after successful PnP final GO_HOME");
      terminal_result->processed_count = processed_count_;
      if (terminal_result->exit_reason.value == MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED) {
        state.terminal = terminal_result;
      }
      if (terminal_result->exit_reason.value == MissionExitReason::MISSION_EXIT_QUANTITY_REACHED &&
      callbacks_.publish_phase)
      {
        callbacks_.publish_phase(
          MissionPhase::MISSION_PHASE_FINISHED);
      }
      return terminal_result->exit_reason.value == MissionExitReason::MISSION_EXIT_QUANTITY_REACHED ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "SafetyRecoveryRequired", [&state](BT::TreeNode &) {
      return state.terminal && state.terminal->exit_reason.value ==
      MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction(
    "SafetyRecovery", [this, &state](BT::TreeNode &) {
      if (recovery_entry_) {
        coordinate_recovery(*recovery_entry_);
      }
      if (recovery_held_return_result_ &&
      (!successful(*recovery_held_return_result_) ||
      recovery_held_return_result_->object_disposition !=
      ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED))
      {
        const auto & result = *recovery_held_return_result_;
        if (!state.terminal) {
          state.terminal = safety_preempted(read_safety());
        }
        state.terminal->diagnostic_detail +=
        "; E-Stop held-material return failed; recovery will not retry";
        if (!state.terminal->motion_failure) {
          state.terminal->motion_failure = motion_failure_trace(result);
        }
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    });
  factory.registerSimpleAction(
    "ContinueBatch", [this, &state](BT::TreeNode &) {
      if (!callbacks_.wait_for_batch_continuation ||
        !callbacks_.wait_for_batch_continuation([this]() {return cancel_requested_.load();}))
      {
        return BT::NodeStatus::FAILURE;
      }
      if (!normal_motion_allowed()) {
        return BT::NodeStatus::FAILURE;
      }
      const auto home = execute_motion(
        motion_goal(MotionType::TASK_TYPE_GO_HOME), MissionPhase::MISSION_PHASE_ENSURING_HOME);
      if (!successful(home)) {
        if (home.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
          state.terminal = safety_preempted(read_safety(), motion_failure_trace(home));
        } else {
          state.terminal = interrupted(
            exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR),
            "Continuation GO_HOME failed", motion_failure_trace(home));
        }
        return BT::NodeStatus::FAILURE;
      }
      state.terminal.reset();
      state.acquisition.reset();
      return BT::NodeStatus::SUCCESS;
    });

  try {
    const auto tree_path = ament_index_cpp::get_package_share_directory("arm_cell_orchestration") +
      "/config/execute_cycle.xml";
    auto tree = factory.createTreeFromFile(tree_path, blackboard);
    std::unique_ptr<BT::PublisherZMQ> groot_publisher;
    try {
      groot_publisher = std::make_unique<BT::PublisherZMQ>(
        tree, 25, kGrootPublisherPort, kGrootServerPort);
    } catch (const std::exception & error) {
      RCLCPP_WARN_ONCE(
        rclcpp::get_logger("arm_cell_orchestration"),
        "Groot ExecuteCycle monitor unavailable; mission will continue: %s", error.what());
    }
    TreeStatusObserver tree_observer(
      tree.rootNode(), [this](
        const std::string & node, const std::string & previous, const std::string & current) {
        if (callbacks_.observe_tree_status) {
          try {
            callbacks_.observe_tree_status(node, previous, current);
          } catch (...) {
            // Tree observation is best-effort and cannot change Mission behavior.
          }
        }
        if (callbacks_.record_structured_diagnostic) {
          DiagnosticRecord record;
          record.event = DiagnosticEvent::BT_NODE_STATUS;
          record.tree_node_size = static_cast<std::uint8_t>(std::min(
              node.size(), record.tree_node.size()));
          record.tree_previous_status_size = static_cast<std::uint8_t>(std::min(
              previous.size(), record.tree_previous_status.size()));
          record.tree_current_status_size = static_cast<std::uint8_t>(std::min(
              current.size(), record.tree_current_status.size()));
          std::copy_n(node.begin(), record.tree_node_size, record.tree_node.begin());
          std::copy_n(
            previous.begin(), record.tree_previous_status_size,
            record.tree_previous_status.begin());
          std::copy_n(
            current.begin(), record.tree_current_status_size, record.tree_current_status.begin());
          record_diagnostic(record);
        }
      });
    auto status = tree.tickRoot();
    while (status == BT::NodeStatus::RUNNING) {
      if (cancel_requested_.load()) {
        tree.haltTree();
        break;
      }
      tree.sleep(std::chrono::milliseconds(10));
      status = tree.tickRoot();
    }
    if (state.terminal) {
      terminal_result = state.terminal;
    }
    if (cancel_requested_.load()) {
      const auto motion_failure = state.terminal ? state.terminal->motion_failure :
        std::optional<MotionFailureTrace>{};
      const auto canceled = commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_CANCELED),
        "Caller canceled the mission", motion_failure);
      if (canceled.exit_reason.value == MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED) {
        return canceled;
      }
      return interrupted(canceled.exit_reason, canceled.diagnostic_detail, canceled.motion_failure);
    }
    if (terminal_result) {
      return *terminal_result;
    }
    if (status != BT::NodeStatus::SUCCESS) {
      return commit_non_safety_terminal(
        exit_reason(MissionExitReason::MISSION_EXIT_MOTION_ERROR),
        "Behavior Tree terminated without a mission result");
    }
    return Result{
      exit_reason(MissionExitReason::MISSION_EXIT_QUANTITY_REACHED),
      "Batch quantity reached after successful PnP final GO_HOME", std::nullopt, processed_count_};
  } catch (const std::exception & error) {
    return commit_non_safety_terminal(
      exit_reason(MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR),
      std::string("Behavior Tree construction or execution failed: ") + error.what());
  }
}

}  // namespace arm_cell_orchestration


namespace arm_cell_orchestration
{

bool CancellationRegistry::ActionSubmission::set_submission_id(std::string submission_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (submission_id.empty() || !submission_id_.empty()) {return false;}
  submission_id_ = std::move(submission_id);
  return true;
}

std::string CancellationRegistry::ActionSubmission::submission_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return submission_id_;
}

void CancellationRegistry::ActionSubmission::set_cancel_observer(CancelFn observer)
{
  CancelFn callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_observer_ = std::move(observer);
    if (cancel_requested_ && !cancel_observed_ && cancel_observer_) {
      cancel_observed_ = true;
      callback = cancel_observer_;
    }
  }
  if (callback) {callback();}
}

void CancellationRegistry::ActionSubmission::request_cancel()
{
  CancelFn cancel_callback;
  CancelFn observer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_requested_ = true;
    if (!cancel_observed_ && cancel_observer_) {
      cancel_observed_ = true;
      observer = cancel_observer_;
    }
    if (!cancel_sent_ && cancel_action_) {
      cancel_sent_ = true;
      cancel_callback = cancel_action_;
    }
  }
  if (observer) {observer();}
  if (cancel_callback) {
    cancel_callback();
  }
}

void CancellationRegistry::ActionSubmission::set_cancel_fn(CancelFn cancel_action)
{
  CancelFn cancel_callback;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_action_ = std::move(cancel_action);
    if (cancel_requested_ && !cancel_sent_ && cancel_action_) {
      cancel_sent_ = true;
      cancel_callback = cancel_action_;
    }
  }
  if (cancel_callback) {
    cancel_callback();
  }
}

bool CancellationRegistry::ActionSubmission::set_goal_uuid(std::string goal_uuid)
{
  if (goal_uuid.empty()) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_uuid_.empty()) {
    goal_uuid_ = goal_uuid;
  }
  return goal_uuid_ == goal_uuid;
}

std::string CancellationRegistry::ActionSubmission::goal_uuid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return goal_uuid_;
}

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

bool CancellationRegistry::register_action_submission(
  const std::string & key, const std::shared_ptr<ActionSubmission> & submission)
{
  bool cancel = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it == entries_.end() || it->second.active_submission || !submission ||
      it->second.next_action_sequence == std::numeric_limits<std::uint64_t>::max())
    {return false;}
    std::ostringstream id;
    id << std::hex << std::setfill('0');
    for (const unsigned char byte : key) {id << std::setw(2) << static_cast<unsigned>(byte);}
    id << ':' << std::dec << it->second.next_action_sequence++;
    if (!submission->set_submission_id(id.str())) {return false;}
    it->second.active_submission = submission;
    cancel = pending_caller_cancels_.count(key) != 0U;
  }
  if (cancel) {submission->request_cancel();}
  return true;
}

void CancellationRegistry::request_caller_cancel(const std::string & key)
{
  CancelFn execution_callback;
  std::shared_ptr<ActionSubmission> active_submission;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      pending_caller_cancels_.insert(key);
      return;
    }
    pending_caller_cancels_.insert(key);
    execution_callback = it->second.cancel_execution;
    active_submission = it->second.active_submission;
  }
  if (execution_callback) {
    execution_callback();
  }
  if (active_submission) {active_submission->request_cancel();}
}

void CancellationRegistry::clear_active_action(
  const std::string & key, const std::shared_ptr<ActionSubmission> & submission)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = entries_.find(key);
  if (it != entries_.end() && it->second.active_submission == submission) {
    it->second.active_submission.reset();
  }
}

void CancellationRegistry::erase_execution(const std::string & key)
{
  std::shared_ptr<ActionSubmission> active_submission;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key);
    if (it != entries_.end()) {
      active_submission = std::move(it->second.active_submission);
      entries_.erase(it);
    }
    pending_caller_cancels_.erase(key);
  }
  if (active_submission) {active_submission->request_cancel();}
}

}  // namespace arm_cell_orchestration
