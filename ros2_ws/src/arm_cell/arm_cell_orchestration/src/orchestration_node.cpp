#include "arm_cell_orchestration/cycle_coordinator.hpp"
#include "arm_cell_orchestration/vision_request_timeout.hpp"
#include "arm_cell_orchestration/execute_cycle_terminal.hpp"
#include "arm_cell_orchestration/failure_trace_record.hpp"
#include "arm_cell_orchestration/material_admission_gate.hpp"
#include "arm_cell_orchestration/recovery_entry_latch.hpp"
#include "arm_cell_orchestration/recovery_wait_predicate.hpp"
#include "arm_cell_orchestration/recipe_repository.hpp"
#include "arm_cell_orchestration/motion_feedback_supervision.hpp"
#include "orchestration_diagnostic_record.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>
#include <arm_cell_interfaces/action/execute_cycle.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/vision_diagnostic.hpp>
#include <arm_cell_interfaces/msg/vision_calibration.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace arm_cell_orchestration
{

namespace
{
constexpr auto kMotionStatusFreshnessTimeout = std::chrono::milliseconds(1000);
constexpr std::size_t kDiagnosticQueueCapacity = 128;
constexpr std::size_t kDiagnosticRecordLimit = 4096;
constexpr std::uint64_t kDiagnosticByteLimit = 2 * 1024 * 1024;
constexpr auto kVisionCalibrationStatusFreshnessTimeout = std::chrono::milliseconds(1000);

bool batch_continuation_ready(
  const SafetyObservation & safety, bool delivery_admitted, bool calibration_fresh,
  std::uint8_t calibration_state,
  const arm_cell_interfaces::msg::MotionStatus & motion,
  bool motion_fresh)
{
  return safety.normal_motion_allowed() && delivery_admitted && calibration_fresh &&
         calibration_state == arm_cell_interfaces::msg::VisionCalibration::STATE_READY &&
         motion_fresh &&
         (motion.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_IDLE ||
         motion.execution_state == arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED) &&
         !motion.execution_active && !motion.has_active_execution &&
         motion.backend_inactivity_confirmed &&
         motion.holding_state == arm_cell_interfaces::msg::MotionStatus::HOLDING_RELEASED;
}

using detail::QueuedOrchestrationRecord;
using detail::release_attempt_diagnostic;
using detail::serialize_diagnostic;

class OrchestrationDiagnosticCapture final
{
public:
  OrchestrationDiagnosticCapture(std::string directory, rclcpp::Logger logger)
  : directory_(std::move(directory)), logger_(std::move(logger)), writer_([this]() {write_loop();})
  {}

  ~OrchestrationDiagnosticCapture()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (writer_.joinable()) {writer_.join();}
  }

  bool enqueue(const QueuedOrchestrationRecord & record)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || failed_ || truncated_ || queue_.size() >= kDiagnosticQueueCapacity) {
        ++dropped_count_;
        return false;
      }
      auto bounded_record = record;
      bounded_record.enqueue_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      bounded_record.sequence = ++sequence_;
      queue_.push_back(std::move(bounded_record));
    }
    condition_.notify_one();
    return true;
  }

private:
  void write_loop() noexcept
  {
    bool record_in_flight = false;
    try {
      std::filesystem::create_directories(directory_);
      std::uint64_t retained_bytes = 0;
      for (const auto & entry : std::filesystem::directory_iterator(directory_)) {
        if (entry.is_regular_file()) {retained_bytes += entry.file_size();}
      }
      if (retained_bytes >= kDiagnosticByteLimit - 512) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          truncated_ = true;
        }
        RCLCPP_WARN(
          logger_,
          "Orchestration diagnostic capture total storage limit is already full");
        return;
      }
      const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      std::ofstream output(
        std::filesystem::path(directory_) /
        ("orchestration-" + std::to_string(stamp) + ".jsonl"), std::ios::out | std::ios::binary);
      if (!output) {throw std::runtime_error("open failed");}
      std::uint64_t bytes = 0;
      for (;; ) {
        QueuedOrchestrationRecord record;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
          if (queue_.empty() && stopping_) {break;}
          record = std::move(queue_.front());
          queue_.pop_front();
          record_in_flight = true;
        }
        std::string line;
        try {
          line = serialize_diagnostic(record, record.sequence) + '\n';
          if (line.size() > kDiagnosticRecordLimit) {
            std::lock_guard<std::mutex> lock(mutex_);
            ++dropped_count_;
            continue;
          }
        } catch (...) {
          std::lock_guard<std::mutex> lock(mutex_);
          ++dropped_count_;
          continue;
        }
        if (retained_bytes + bytes + line.size() > kDiagnosticByteLimit - 512) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            truncated_ = true;
            dropped_count_ += queue_.size() + 1;
            queue_.clear();
          }
          RCLCPP_WARN(
            logger_,
            "Orchestration diagnostic capture reached its byte limit; further records are dropped");
          break;
        }
        output.write(line.data(), static_cast<std::streamsize>(line.size()));
        if (!output) {throw std::runtime_error("write failed");}
        bytes += line.size();
        record_in_flight = false;
      }
      std::uint64_t dropped = 0;
      bool truncated = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        dropped = dropped_count_;
        truncated = truncated_;
      }
      const auto summary = std::string(
        "{\"schema_version\":1,\"event\":\"capture_summary\",\"dropped_records\":") +
        std::to_string(dropped) + ",\"complete\":" +
        (truncated || dropped != 0 ? "false}" : "true}") + '\n';
      if (retained_bytes + bytes + summary.size() <= kDiagnosticByteLimit) {
        output.write(summary.data(), static_cast<std::streamsize>(summary.size()));
        output.flush();
        if (!output) {throw std::runtime_error("summary write failed");}
      }
      output.close();
      if (output.fail()) {throw std::runtime_error("close failed");}
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        failed_ = true;
        dropped_count_ += queue_.size() + (record_in_flight ? 1U : 0U);
        queue_.clear();
      }
      RCLCPP_WARN(
        logger_,
        "Orchestration diagnostic capture failed; mission results are unchanged: %s",
        error.what());
    }
  }

  std::string directory_;
  rclcpp::Logger logger_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<QueuedOrchestrationRecord> queue_;
  bool stopping_{false};
  bool failed_{false};
  bool truncated_{false};
  std::uint64_t sequence_{0};
  std::uint64_t dropped_count_{0};
  std::thread writer_;
};

}  // namespace

class OrchestrationNode : public rclcpp::Node
{
#ifdef ARM_CELL_ORCHESTRATION_TEST
  friend struct OrchestrationNodeTestAccess;
  std::function<void(const detail::QueuedOrchestrationRecord &)> test_action_record_;
  std::function<void()> test_late_feedback_;
#endif
public:
  using ExecuteCycle = arm_cell_interfaces::action::ExecuteCycle;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCycle>;
  using MotionAction = arm_cell_interfaces::action::ExecuteTask;
  using PnpAction = arm_cell_interfaces::action::ExecutePnp;
  using DisturbAction = arm_cell_interfaces::action::ExecuteDisturb;
  using DetectTarget = arm_cell_interfaces::srv::DetectTarget;
  using SafetyState = arm_cell_interfaces::msg::SafetyState;
  using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
  using MaterialReadiness = arm_cell_interfaces::msg::MaterialReadiness;
  using VisionDiagnostic = arm_cell_interfaces::msg::VisionDiagnostic;
  using VisionCalibration = arm_cell_interfaces::msg::VisionCalibration;

  explicit OrchestrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("orchestration_node", options)
  {
    mission_target_id_ = declare_parameter<std::string>("mission_target_id", "");
    const auto recipe_directory = declare_parameter<std::string>("recipe_directory", "");
    if (mission_target_id_.empty() || recipe_directory.empty()) {
      throw std::invalid_argument("mission target_id and recipe_directory are required");
    }
    recipe_repository_ = std::make_unique<RecipeRepository>(recipe_directory);
    diagnostic_capture_enabled_ = declare_parameter<bool>(
      "diagnostics.capture_enabled", false);
    const auto diagnostics_directory = declare_parameter<std::string>(
      "diagnostics.output_directory", ".local_artifacts/observability/orchestration");
    diagnostic_capture_ = std::make_unique<OrchestrationDiagnosticCapture>(
      diagnostics_directory, get_logger());
    RCLCPP_DEBUG(get_logger(), "recipe registry ready source=%s", recipe_directory.c_str());
    vision_request_timeout_ms_ = declare_parameter<int64_t>(
      "vision_request_timeout_ms", 30000);
    if (vision_request_timeout_ms_ <= 0) {
      throw std::invalid_argument("vision_request_timeout_ms must be positive");
    }
    safety_state_freshness_timeout_ms_ = declare_parameter<int>(
      "safety_state_freshness_timeout_ms", 500);
    if (safety_state_freshness_timeout_ms_ <= 0) {
      throw std::invalid_argument("safety_state_freshness_timeout_ms must be positive");
    }
    detect_client_ = create_client<DetectTarget>("/vision/detect_target");
    vision_calibration_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    vision_calibration_client_ = create_client<std_srvs::srv::Trigger>(
      "/vision/calibrate", rmw_qos_profile_services_default,
      vision_calibration_client_callback_group_);
    vision_calibration_service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    vision_calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "/orchestration/calibrate_vision",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_vision_calibration_request(*response);
      }, rmw_qos_profile_services_default, vision_calibration_service_callback_group_);
    motion_client_ = rclcpp_action::create_client<MotionAction>(
      this, "/motion/execute_task");
    pnp_client_ = rclcpp_action::create_client<PnpAction>(
      this, "/task_execution/execute_pnp");
    disturb_client_ = rclcpp_action::create_client<DisturbAction>(
      this, "/task_execution/execute_disturb");
    material_cycle_client_ = rclcpp_action::create_client<ExecuteCycle>(
      this, "/orchestration/execute_cycle");
    safety_subscription_ = create_subscription<SafetyState>(
      "/safety/state", rclcpp::QoS(10),
      [this](const SafetyState::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          latest_safety_ = *message;
          latest_safety_receipt_ = MaterialAdmissionGate::Clock::now();
          safety_state_received_ = true;
          for (const auto & entry : recovery_entries_) {
            observe_recovery_entry(
              entry.first, safety_observation_locked(latest_safety_receipt_), recovery_entries_);
          }
          ++recovery_generation_;
          if (!safety_observation_locked(latest_safety_receipt_).normal_motion_allowed()) {
            ++safety_preempt_generation_;
          }
          recovery_cv_.notify_all();
        }
        attempt_material_admission();
      });
    material_readiness_subscription_ = create_subscription<MaterialReadiness>(
      "/integration/material_readiness",
      rclcpp::QoS(64).reliable().transient_local(),
      [this](const MaterialReadiness::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          material_admission_gate_.observe(*message);
        }
        recovery_cv_.notify_all();
        attempt_material_admission();
      });
    motion_subscription_ = create_subscription<MotionStatus>(
      "/motion/state", rclcpp::QoS(10),
      [this](const MotionStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        latest_motion_ = *message;
        latest_motion_receipt_ = std::chrono::steady_clock::now();
        ++motion_status_generation_;
        ++recovery_generation_;
        recovery_cv_.notify_all();
      });
    vision_diagnostic_subscription_ = create_subscription<VisionDiagnostic>(
      "/vision/diagnostics", rclcpp::QoS(1).reliable().transient_local(),
      [this](const VisionDiagnostic::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          latest_vision_diagnostic_ = *message;
          latest_vision_diagnostic_receipt_ = std::chrono::steady_clock::now();
          vision_diagnostic_received_ = true;
          ++vision_diagnostic_generation_;
        }
        recovery_cv_.notify_all();
      });
    vision_calibration_subscription_ = create_subscription<VisionCalibration>(
      "/vision/calibration_status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const VisionCalibration::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          latest_vision_calibration_ = *message;
          latest_vision_calibration_receipt_ = std::chrono::steady_clock::now();
          vision_calibration_received_ = true;
        }
        attempt_material_admission();
      });
    action_server_ = rclcpp_action::create_server<ExecuteCycle>(
        this, "/orchestration/execute_cycle",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const ExecuteCycle::Goal> goal) {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          const auto delivery_key = delivery_identity_key(goal->delivery_id);
          const auto authorized =
              authorized_material_deliveries_.find(delivery_key);
          if (goal->target_id.empty() || goal->quantity == 0U ||
              delivery_key.empty() ||
              authorized == authorized_material_deliveries_.end() ||
              goal->target_id != authorized->second.material_type ||
              goal->quantity != authorized->second.quantity) {
            RCLCPP_DEBUG(get_logger(),
                         "mission rejected reason=material_not_authorized");
            return rclcpp_action::GoalResponse::REJECT;
          }
          authorized_material_deliveries_.erase(authorized);
          RCLCPP_DEBUG(get_logger(), "mission accepted");
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          const auto key = uuid_key(handle->get_goal_id());
          const auto cancel_requested_at = std::chrono::steady_clock::now();
          {
            std::lock_guard<std::mutex> lock(recovery_mutex_);
            cancellation_started_.emplace(key, cancel_requested_at);
          }
          RCLCPP_INFO(get_logger(), "mission cancellation requested");
          RCLCPP_DEBUG(get_logger(),
                       "mission cancellation propagation started");
          cancellation_registry_.request_caller_cancel(key);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          const auto key = uuid_key(handle->get_goal_id());
          {
            std::lock_guard<std::mutex> lock(recovery_mutex_);
            begin_recovery_entry(key, recovery_entries_);
          }
          auto vision_result_code = std::make_shared<uint8_t>(
              arm_cell_interfaces::msg::DetectTargetResultCode::
                  DETECT_RESULT_INVALID_RESULT);
          auto coordinator = std::make_shared<CycleCoordinator>(
              make_callbacks(handle, vision_result_code));
          cancellation_registry_.register_execution(key, [this, coordinator]() {
            coordinator->set_cancel_requested(true);
            recovery_cv_.notify_all();
          });
          std::thread([this, handle, coordinator, key, vision_result_code]() {
            const auto & goal = *handle->get_goal();
            RCLCPP_INFO(get_logger(), "mission started quantity=%u",
                        goal.quantity);
            execute_goal(handle, coordinator, key, vision_result_code);
          }).detach();
        });
  }

private:
  void record_action_lifecycle(
    const std::string & key, const std::string & submission_id, const char * kind,
    const char * event, const std::string & goal_uuid = {}, const char * terminal = "",
    int outcome = -1, int material_placed = -1) noexcept
  {
    try {
      if (!diagnostic_capture_enabled_) {return;}
      QueuedOrchestrationRecord queued;
      queued.decision.event = CycleCoordinator::DiagnosticEvent::ACTION_LIFECYCLE;
      if (key.size() == queued.execute_cycle_id.size()) {
        std::copy(key.begin(), key.end(), queued.execute_cycle_id.begin());
      }
      const auto copy = [](const std::string & value, auto & destination) {
          std::copy_n(value.begin(), std::min(value.size(), destination.size() - 1), destination.begin());
        };
      copy(submission_id, queued.submission_id);
      copy(goal_uuid, queued.action_goal_uuid);
      copy(kind, queued.action_kind);
      copy(event, queued.action_event);
      copy(terminal, queued.action_terminal);
      queued.action_outcome = outcome;
      queued.material_placed = material_placed;
      const bool enqueued = diagnostic_capture_->enqueue(queued);
#ifdef ARM_CELL_ORCHESTRATION_TEST
      if (enqueued && test_action_record_) {test_action_record_(queued);}
#endif
      if (!enqueued && !diagnostic_drop_reported_.exchange(true)) {
        RCLCPP_WARN(get_logger(), "Orchestration diagnostic record was dropped");
      }
    } catch (...) {
      if (!diagnostic_drop_reported_.exchange(true)) {
        RCLCPP_WARN(get_logger(), "Orchestration diagnostic record failed");
      }
    }
  }

  enum class TaskExecutionLifecycleEvent : std::uint8_t {GOAL_REJECTED, TERMINAL, SAFETY_HANDOFF};

  template<typename ActionT>
  struct TaskExecutionLifecycle
  {
    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
    std::mutex feedback_handoff_mutex;
    bool goal_response_received{false};
    typename ClientGoalHandle::SharedPtr goal_handle;
    std::optional<typename ClientGoalHandle::WrappedResult> terminal_result;
    bool safety_preempted{false};
    bool safety_cancel_requested{false};
    bool handed_off{false};
  };

  template<typename ActionT>
  TaskExecutionLifecycleEvent await_task_execution_lifecycle(
    const std::shared_ptr<CancellationRegistry::ActionSubmission> & submission,
    const std::shared_ptr<TaskExecutionLifecycle<ActionT>> & lifecycle)
  {
    std::unique_lock<std::mutex> lock(recovery_mutex_);
    while (true) {
      const auto now = MaterialAdmissionGate::Clock::now();
      const auto safety = safety_observation_locked(now);
      if (!safety.normal_motion_allowed()) {
        lifecycle->safety_preempted = true;
        if (!lifecycle->safety_cancel_requested) {
          lifecycle->safety_cancel_requested = true;
          lock.unlock();
          submission->request_cancel();
          lock.lock();
          continue;
        }
      }
      const bool motion_status_fresh = latest_motion_receipt_ !=
        std::chrono::steady_clock::time_point{} && now >= latest_motion_receipt_ &&
      now - latest_motion_receipt_ <= kMotionStatusFreshnessTimeout;
      const bool motion_inactive = motion_status_fresh &&
        latest_motion_.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
        !latest_motion_.execution_active && !latest_motion_.has_active_execution &&
        latest_motion_.backend_inactivity_confirmed;
      const bool safe_recovery_handoff = lifecycle->safety_preempted && motion_inactive && (
        recovery_ready_for_immediate_return(safety, latest_motion_) ||
        recovery_ready_for_batch_continuation(safety, latest_motion_, motion_status_fresh) ||
        recovery_severity_escalation_ready_for_immediate_return(safety));
      if (lifecycle->terminal_result) {
        return TaskExecutionLifecycleEvent::TERMINAL;
      }
      if (lifecycle->goal_response_received && !lifecycle->goal_handle) {
        return TaskExecutionLifecycleEvent::GOAL_REJECTED;
      }
      if (safe_recovery_handoff) {
        lock.unlock();
        std::unique_lock<std::mutex> feedback_handoff_lock(
          lifecycle->feedback_handoff_mutex);
        lock.lock();
        if (lifecycle->terminal_result ||
          (lifecycle->goal_response_received && !lifecycle->goal_handle))
        {
          feedback_handoff_lock.unlock();
          continue;
        }
        const auto handoff_now = MaterialAdmissionGate::Clock::now();
        const auto handoff_safety = safety_observation_locked(handoff_now);
        const bool handoff_motion_status_fresh = latest_motion_receipt_ !=
          std::chrono::steady_clock::time_point{} && handoff_now >= latest_motion_receipt_ &&
          handoff_now - latest_motion_receipt_ <= kMotionStatusFreshnessTimeout;
        const bool handoff_motion_inactive = handoff_motion_status_fresh &&
          latest_motion_.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
          !latest_motion_.execution_active && !latest_motion_.has_active_execution &&
          latest_motion_.backend_inactivity_confirmed;
        const bool handoff_still_ready = !handoff_safety.normal_motion_allowed() &&
          handoff_motion_inactive && (
          recovery_ready_for_immediate_return(handoff_safety, latest_motion_) ||
          recovery_ready_for_batch_continuation(
            handoff_safety, latest_motion_, handoff_motion_status_fresh) ||
          recovery_severity_escalation_ready_for_immediate_return(handoff_safety));
        if (!handoff_still_ready) {
          feedback_handoff_lock.unlock();
          continue;
        }
        lifecycle->handed_off = true;
        lifecycle->goal_handle.reset();
        lock.unlock();
        feedback_handoff_lock.unlock();
        submission->request_cancel();
        return TaskExecutionLifecycleEvent::SAFETY_HANDOFF;
      }
      recovery_cv_.wait(lock);
    }
  }

  static std::string uuid_key(const rclcpp_action::GoalUUID & uuid)
  {
    return std::string(uuid.begin(), uuid.end());
  }

  template<typename UuidT>
  static std::string uuid_string(const UuidT & uuid)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : uuid) {
      stream << std::setw(2) << static_cast<unsigned int>(static_cast<unsigned char>(byte));
    }
    return stream.str();
  }

  static std::string delivery_identity_key(
    const unique_identifier_msgs::msg::UUID & uuid)
  {
    bool all_zero = true;
    std::string key;
    key.reserve(uuid.uuid.size());
    for (const auto byte : uuid.uuid) {
      all_zero = all_zero && byte == 0;
      key.push_back(static_cast<char>(byte));
    }
    return all_zero ? std::string{} : key;
  }

  void clear_material_admission_wait_locked()
  {
    last_material_admission_wait_delivery_key_.clear();
    last_material_admission_wait_reason_.clear();
  }

  std::string vision_calibration_block_reason_locked(
    MaterialAdmissionGate::Clock::time_point now) const
  {
    if (calibration_in_progress_) {
      return "vision_calibration_in_progress";
    }
    if (!vision_calibration_received_ || now < latest_vision_calibration_receipt_ ||
      now - latest_vision_calibration_receipt_ > kVisionCalibrationStatusFreshnessTimeout)
    {
      return "vision_calibration_status_unavailable";
    }
    if (latest_vision_calibration_.state == VisionCalibration::STATE_READY) {
      return {};
    }
    if (latest_vision_calibration_.state == VisionCalibration::STATE_NEEDS_CALIBRATION) {
      return "vision_calibration_needs_calibration";
    }
    if (latest_vision_calibration_.state == VisionCalibration::STATE_CALIBRATING) {
      return "vision_calibration_in_progress";
    }
    return "vision_calibration_status_unavailable";
  }

  void handle_vision_calibration_request(std_srvs::srv::Trigger::Response & response)
  {
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      if (cycle_in_progress_) {
        response.success = false;
        response.message = "active Mission; Calibration request rejected";
        return;
      }
      if (calibration_in_progress_) {
        response.success = false;
        response.message = "Calibration request already in progress";
        return;
      }
      if (!vision_calibration_client_->service_is_ready()) {
        response.success = false;
        response.message = "Vision Calibration service is unavailable";
        return;
      }
      // Serialize the Mission admission decision with this request claim.
      calibration_in_progress_ = true;
    }
    try {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = vision_calibration_client_->async_send_request(request);
      const auto vision_response = future.get();
      response.success = vision_response->success;
      response.message = vision_response->message;
    } catch (const std::exception & error) {
      response.success = false;
      response.message = std::string("Vision Calibration request failed: ") + error.what();
    } catch (...) {
      response.success = false;
      response.message = "Vision Calibration request failed: unknown error";
    }
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      calibration_in_progress_ = false;
    }
    attempt_material_admission();
  }

  void attempt_material_admission()
  {
    std::optional<MaterialAdmissionGate::Admission> admission;
    std::string block_reason;
    bool admission_wait_changed = false;
    bool admission_wait_started = false;
    bool admission_wait_ended = false;
    std::string admission_wait_end_reason = "admitted";
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      if (cycle_in_progress_) {
        return;
      }
      unique_identifier_msgs::msg::UUID current_delivery_id;
      const auto now = MaterialAdmissionGate::Clock::now();
      admission = material_admission_gate_.try_admit(
        safety_observation_locked(now), mission_target_id_, &block_reason,
        &current_delivery_id, vision_calibration_block_reason_locked(now));
      const auto current_key = delivery_identity_key(current_delivery_id);
      if (!admission) {
        if (block_reason == "readiness_already_admitted") {
          admission_wait_ended = !last_material_admission_wait_reason_.empty();
          admission_wait_end_reason = "delivery_identity_already_consumed";
          clear_material_admission_wait_locked();
        } else {
          if (current_key != last_material_admission_wait_delivery_key_ ||
            block_reason != last_material_admission_wait_reason_)
          {
            const bool delivery_changed =
              current_key != last_material_admission_wait_delivery_key_;
            admission_wait_started = last_material_admission_wait_reason_.empty() ||
              delivery_changed;
            if (delivery_changed && !last_material_admission_wait_reason_.empty()) {
              admission_wait_ended = true;
              admission_wait_end_reason = "readiness_identity_replaced";
            }
            admission_wait_changed = true;
            last_material_admission_wait_delivery_key_ = current_key;
            last_material_admission_wait_reason_ = block_reason;
          }
        }
      } else {
        cycle_in_progress_ = true;
        authorized_material_deliveries_.emplace(
          delivery_identity_key(admission->delivery_id), *admission);
        admission_wait_ended = !last_material_admission_wait_reason_.empty();
        clear_material_admission_wait_locked();
      }
    }

    if (!admission) {
      if (admission_wait_ended) {
        RCLCPP_INFO(get_logger(), "material admission wait ended reason=%s",
                    admission_wait_end_reason.c_str());
      }
      if (admission_wait_changed) {
        RCLCPP_INFO(get_logger(),
                    admission_wait_started
                        ? "material admission wait entered reason=%s"
                        : "material admission wait reason changed reason=%s",
                    block_reason.c_str());
      }
      return;
    }
    if (admission_wait_ended) {
      RCLCPP_INFO(get_logger(),
                  "material admission wait ended reason=admitted");
    }

    ExecuteCycle::Goal goal;
    goal.target_id = admission->material_type;
    goal.delivery_id = admission->delivery_id;
    goal.quantity = admission->quantity;
    const auto delivery_key = delivery_identity_key(admission->delivery_id);
    rclcpp_action::Client<ExecuteCycle>::SendGoalOptions options;
    options.goal_response_callback =
        [this, delivery_key](
            const rclcpp_action::ClientGoalHandle<ExecuteCycle>::SharedPtr
                &goal_handle) {
          if (!goal_handle) {
            {
              std::lock_guard<std::mutex> lock(recovery_mutex_);
              authorized_material_deliveries_.erase(delivery_key);
              cycle_in_progress_ = false;
            }
            RCLCPP_ERROR(get_logger(), "material admission action rejected "
                                       "reason=execute_cycle_goal_rejected");
            attempt_material_admission();
            return;
          }
          RCLCPP_DEBUG(get_logger(), "material admission action accepted");
        };
    try {
      material_cycle_client_->async_send_goal(goal, options);
      RCLCPP_DEBUG(get_logger(), "material admission action dispatched");
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        authorized_material_deliveries_.erase(delivery_key);
        cycle_in_progress_ = false;
      }
      RCLCPP_ERROR(get_logger(),
                   "material admission action submission failed "
                   "reason=execute_cycle_goal_submission_failed diagnostic=%s",
                   error.what());
      attempt_material_admission();
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        authorized_material_deliveries_.erase(delivery_key);
        cycle_in_progress_ = false;
      }
      RCLCPP_ERROR(get_logger(), "material admission action submission failed "
                                 "reason=execute_cycle_goal_submission_failed "
                                 "diagnostic=unknown_exception");
      attempt_material_admission();
    }
  }

  void execute_goal(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<CycleCoordinator> & coordinator,
    const std::string & key,
    const std::shared_ptr<uint8_t> & vision_result_code)
  {
    const auto & goal = *handle->get_goal();
    QueuedOrchestrationRecord started;
    started.decision.event = CycleCoordinator::DiagnosticEvent::MISSION_STARTED;
    started.decision.requested_quantity = goal.quantity;
    started.decision.result_available = false;
    std::copy_n(handle->get_goal_id().begin(), started.execute_cycle_id.size(),
      started.execute_cycle_id.begin());
    std::copy_n(goal.delivery_id.uuid.begin(), started.delivery_id.size(),
      started.delivery_id.begin());
    if (goal.target_id.size() <= started.target_id.size()) {
      started.target_id_size = static_cast<std::uint8_t>(goal.target_id.size());
      std::copy_n(goal.target_id.begin(), started.target_id_size, started.target_id.begin());
    }
    if (!diagnostic_capture_->enqueue(started)) {
      RCLCPP_WARN(get_logger(), "Orchestration mission start record was dropped");
    }
    const auto result = coordinator->execute(
      handle->get_goal()->target_id, handle->get_goal()->quantity);
    {
      CycleCoordinator::DiagnosticRecord outcome;
      outcome.event = CycleCoordinator::DiagnosticEvent::MISSION_OUTCOME;
      outcome.final_result = result.exit_reason.value;
      outcome.mission_exit_reason = result.exit_reason.value;
      outcome.requested_quantity = handle->get_goal()->quantity;
      outcome.processed_count = result.processed_count;
      outcome.result_available = false;
      if (result.motion_failure) {
        const auto & failure = *result.motion_failure;
        outcome.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
        outcome.failure_code = failure.code.value;
        outcome.result_available = failure.result_code_available;
        outcome.motion_task_type = failure.task_type;
        outcome.reason = failure.code.value ==
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY ?
          CycleCoordinator::DiagnosticReason::UNEXPECTED_BUSY :
          CycleCoordinator::DiagnosticReason::NONE;
        outcome.decision = CycleCoordinator::DiagnosticDecision::STOP;
        outcome.result_detail_size = static_cast<std::uint8_t>(std::min(
          failure.diagnostic_detail.size(), outcome.result_detail.size()));
        std::copy_n(failure.diagnostic_detail.begin(), outcome.result_detail_size,
          outcome.result_detail.begin());
        outcome.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
          failure.motion_execution_id.size(), outcome.motion_execution_id.size()));
        std::copy_n(failure.motion_execution_id.begin(), outcome.motion_execution_id_size,
          outcome.motion_execution_id.begin());
        outcome.motion_action_terminal_size = static_cast<std::uint8_t>(std::min(
          failure.action_terminal.size(), outcome.motion_action_terminal.size()));
        std::copy_n(failure.action_terminal.begin(), outcome.motion_action_terminal_size,
          outcome.motion_action_terminal.begin());
      }
      QueuedOrchestrationRecord queued;
      queued.decision = outcome;
      std::copy_n(handle->get_goal_id().begin(), queued.execute_cycle_id.size(),
        queued.execute_cycle_id.begin());
      std::copy_n(goal.delivery_id.uuid.begin(), queued.delivery_id.size(), queued.delivery_id.begin());
      if (goal.target_id.size() <= queued.target_id.size()) {
        queued.target_id_size = static_cast<std::uint8_t>(goal.target_id.size());
        std::copy_n(goal.target_id.begin(), queued.target_id_size, queued.target_id.begin());
      }
      if (!diagnostic_capture_->enqueue(queued) && !diagnostic_drop_reported_.exchange(true))
      {
        RCLCPP_WARN(
          get_logger(),
          "Orchestration diagnostic record was dropped (queue, size, storage, or byte limit)");
      }
    }
    if (result.exit_reason.value ==
        arm_cell_interfaces::msg::MissionExitReason::
            MISSION_EXIT_CONFIGURATION_ERROR) {
      RCLCPP_ERROR(
          get_logger(),
          "mission terminated "
          "exit_reason=%s reason=recipe_configuration_terminal diagnostic=%s",
          mission_exit_reason_name(result.exit_reason.value),
          result.diagnostic_detail.c_str());
    }
    if (result.motion_failure) {
      const auto & failure = *result.motion_failure;
      const auto & goal = *handle->get_goal();
      RCLCPP_ERROR(get_logger(),
                   "Mission failed during Motion execution task=%s reason=%s "
                   "exit_reason=%s cause=%s processed=%llu/%u",
                   motion_task_type_name(failure.task_type),
                   motion_result_code_field(failure.code.value,
                                            failure.result_code_available)
                       .c_str(),
                   mission_exit_reason_name(result.exit_reason.value),
                   failure.diagnostic_detail.c_str(),
                   static_cast<unsigned long long>(result.processed_count),
                   goal.quantity);
    } else if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_DEPLETED ||
      result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_QUANTITY_REACHED)
    {
      const auto & goal = *handle->get_goal();
      RCLCPP_INFO(get_logger(),
                  "mission completed exit_reason=%s processed=%llu/%u",
                  mission_exit_reason_name(result.exit_reason.value),
                  static_cast<unsigned long long>(result.processed_count),
                  goal.quantity);
    } else if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CANCELED)
    {
      const auto & goal = *handle->get_goal();
      RCLCPP_INFO(get_logger(),
                  "mission canceled exit_reason=%s processed=%llu/%u",
                  mission_exit_reason_name(result.exit_reason.value),
                  static_cast<unsigned long long>(result.processed_count),
                  goal.quantity);
    } else if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR)
    {
      // The configuration terminal above owns this mission failure summary.
    } else {
      const bool vision_failure = result.exit_reason.value ==
        arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_VISION_ERROR &&
        *vision_result_code !=
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
      const bool safety_failure = result.exit_reason.value ==
        arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED;
      const char * causal_owner = vision_failure ? "Vision" :
        safety_failure ? "Safety" : "Orchestration";
      if (vision_failure) {
        RCLCPP_ERROR(get_logger(),
                     "mission operation failed exit_reason=%s causal_owner=%s "
                     "causal_result_code=%s diagnostic=%s",
                     mission_exit_reason_name(result.exit_reason.value),
                     causal_owner,
                     detect_target_result_code_name(*vision_result_code),
                     result.diagnostic_detail.c_str());
      } else {
        RCLCPP_ERROR(get_logger(),
                     "mission operation failed exit_reason=%s causal_owner=%s "
                     "causal_result=%s diagnostic=%s",
                     mission_exit_reason_name(result.exit_reason.value),
                     causal_owner,
                     mission_exit_reason_name(result.exit_reason.value),
                     result.diagnostic_detail.c_str());
      }
    }
    auto action_result = std::make_shared<ExecuteCycle::Result>();
    action_result->exit_reason = result.exit_reason;
    action_result->diagnostic_detail = result.diagnostic_detail;
    complete_execute_cycle_goal(handle, action_result);

    cancellation_registry_.erase_execution(key);
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      erase_recovery_entry(key, recovery_entries_);
      cancellation_started_.erase(key);
      cycle_in_progress_ = false;
    }
    attempt_material_admission();
  }

  CycleCoordinator::Callbacks make_callbacks(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<uint8_t> & vision_result_code)
  {
    const auto key = uuid_key(handle->get_goal_id());
    auto previous_phase = std::make_shared<uint8_t>(
      arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_UNKNOWN);
    auto processed_count = std::make_shared<uint64_t>(0U);
    auto retry_count = std::make_shared<uint8_t>(0U);
    auto interruption_count = std::make_shared<uint32_t>(0U);
    auto first_interruption_reason =
      std::make_shared<arm_cell_interfaces::msg::MissionExitReason>();
    auto first_interruption_causes = std::make_shared<arm_cell_interfaces::msg::SafetyCauseSet>();
    auto first_interruption_stop_mode = std::make_shared<arm_cell_interfaces::msg::StopMode>();
    const auto decorate_feedback = [handle, processed_count, interruption_count,
      first_interruption_reason, first_interruption_causes, first_interruption_stop_mode](
      ExecuteCycle::Feedback & feedback) {
        feedback.delivery_id = handle->get_goal()->delivery_id;
        feedback.quantity = handle->get_goal()->quantity;
        feedback.batch_processed_count = *processed_count;
        feedback.interruption_count = *interruption_count;
        feedback.first_interruption_reason = *first_interruption_reason;
        feedback.first_interruption_causes = *first_interruption_causes;
        feedback.first_interruption_stop_mode = *first_interruption_stop_mode;
      };
    CycleCoordinator::Callbacks callbacks;
    auto selected_recipe_id = std::make_shared<std::string>();
    auto previous_recovery_diagnostic_signature =
      std::make_shared<std::optional<RecoveryObservationDiagnosticSignature>>();
    callbacks.detect_target = [this, handle](const DetectTarget::Request & request) {
        return detect_target(handle, request);
      };
    callbacks.detect_target_cancellable = [this, handle](
      const DetectTarget::Request & request,
      const CycleCoordinator::CancelPredicateFn & cancel) {
        return detect_target(handle, request, cancel);
      };
    callbacks.read_vision_diagnostic_generation = [this]() {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        return vision_diagnostic_generation_;
      };
    callbacks.wait_for_vision_idle = [this, handle](uint64_t request_generation,
        const CycleCoordinator::CancelPredicateFn & cancel) {
        return wait_for_vision_idle(handle, request_generation, cancel);
      };
    callbacks.record_vision_result = [this, handle, vision_result_code](uint8_t result_code) {
        *vision_result_code = result_code;
        if (result_code ==
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS)
        {
          RCLCPP_INFO(get_logger(), "Vision target acquired");
        }
      };
    const auto logger_level = rcutils_logging_get_logger_level(get_logger().get_name());
    const auto effective_logger_level = logger_level == RCUTILS_LOG_SEVERITY_UNSET ?
      rcutils_logging_get_default_logger_level() : logger_level;
    const bool debug_logging_enabled = effective_logger_level <= RCUTILS_LOG_SEVERITY_DEBUG;
    if (debug_logging_enabled) {
      callbacks.record_diagnostic = [this, handle](const std::string &record) {
        RCLCPP_DEBUG(get_logger(), "mission diagnostic %s", record.c_str());
      };
    }
    if (diagnostic_capture_enabled_) {
      callbacks.record_structured_diagnostic = [this, handle](
        const CycleCoordinator::DiagnosticRecord & record) {
          const auto & goal = *handle->get_goal();
          QueuedOrchestrationRecord queued;
          queued.decision = record;
          std::copy_n(
            handle->get_goal_id().begin(), queued.execute_cycle_id.size(),
            queued.execute_cycle_id.begin());
          std::copy_n(
            goal.delivery_id.uuid.begin(), queued.delivery_id.size(),
            queued.delivery_id.begin());
          const bool target_id_bounded = goal.target_id.size() <= queued.target_id.size();
          queued.target_id_size = static_cast<std::uint8_t>(goal.target_id.size());
          if (target_id_bounded) {
            std::copy_n(goal.target_id.begin(), queued.target_id_size, queued.target_id.begin());
          }
          if ((!target_id_bounded || !diagnostic_capture_->enqueue(queued)) &&
            !diagnostic_drop_reported_.exchange(true))
          {
            RCLCPP_WARN(
              get_logger(),
              "Orchestration diagnostic record was dropped (queue, size, storage, or byte limit)");
          }
        };
    }
    callbacks.execute_task = [this, handle, key, selected_recipe_id](
      const MotionAction::Goal & goal) {
        return execute_task(handle, key, goal, *selected_recipe_id);
      };
    callbacks.execute_task_with_feedback = [this, handle, key, selected_recipe_id](
      const MotionAction::Goal & goal, const CycleCoordinator::MotionFeedbackFn & feedback) {
        return execute_task(handle, key, goal, *selected_recipe_id, feedback);
      };
    callbacks.execute_pnp = [this, key, selected_recipe_id](
      const PnpAction::Goal & goal) {
        return execute_pnp(key, goal, *selected_recipe_id, {});
      };
    callbacks.execute_pnp_with_feedback = [this, key, selected_recipe_id](
      const PnpAction::Goal & goal, const CycleCoordinator::PnpFeedbackFn & feedback) {
        return execute_pnp(key, goal, *selected_recipe_id, feedback);
      };
    callbacks.execute_disturb = [this, key](const DisturbAction::Goal & goal) {
        return execute_disturb(key, goal);
      };
    callbacks.execute_task_scoped = [this, handle, key, selected_recipe_id](
      const MotionAction::Goal & goal, const CycleCoordinator::MotionFeedbackFn & feedback,
      const CycleCoordinator::Submission & submission) {
        return execute_task(handle, key, goal, *selected_recipe_id, feedback, submission);
      };
    callbacks.execute_pnp_scoped = [this, key, selected_recipe_id](
      const PnpAction::Goal & goal, const CycleCoordinator::PnpFeedbackFn & feedback,
      const CycleCoordinator::Submission & submission) {
        return execute_pnp(key, goal, *selected_recipe_id, feedback, submission);
      };
    callbacks.execute_disturb_scoped = [this, key](const DisturbAction::Goal & goal,
      const CycleCoordinator::Submission & submission) {
        return execute_disturb(key, goal, submission);
      };
    callbacks.read_safety = [this, key]() {return read_safety(key);};
    callbacks.read_motion_status = [this, handle](bool wait_for_confirmed_stop)
      -> std::optional<MotionStatus> {
        std::unique_lock<std::mutex> lock(recovery_mutex_);
        if (wait_for_confirmed_stop) {
          recovery_cv_.wait_for(lock, kMotionStatusFreshnessTimeout, [this, handle]() {
            const auto now = std::chrono::steady_clock::now();
            const bool fresh = latest_motion_receipt_ != std::chrono::steady_clock::time_point{} &&
              now >= latest_motion_receipt_ &&
              now - latest_motion_receipt_ <= kMotionStatusFreshnessTimeout;
            const bool stopped = fresh &&
              latest_motion_.execution_state == MotionStatus::MOTION_STATE_STOPPED &&
              !latest_motion_.execution_active && !latest_motion_.has_active_execution &&
              latest_motion_.backend_inactivity_confirmed;
            return stopped || handle->is_canceling();
          });
        }
        const auto decision = std::chrono::steady_clock::now();
        std::optional<MotionStatus> status;
        const auto observed_at = latest_motion_receipt_;
        if (observed_at != std::chrono::steady_clock::time_point{} &&
          decision >= observed_at && decision - observed_at <= kMotionStatusFreshnessTimeout)
        {
          status = latest_motion_;
        }
        lock.unlock();
        const auto observed = observed_at != std::chrono::steady_clock::time_point{};
        if (!status) {
          RCLCPP_WARN(
              get_logger(),
              "Motion holding observation unavailable for retry admission reason=%s",
              !observed ? "motion_status_unavailable" : "motion_status_stale");
        } else {
          RCLCPP_DEBUG(
              get_logger(),
              "Motion holding observation for retry admission usable=true reason=fresh_motion_status");
        }
        return status;
      };
    callbacks.wait_for_recovery_cancellable = [this, handle,
        previous_recovery_diagnostic_signature](
      const CycleCoordinator::CancelPredicateFn & canceled,
      CycleCoordinator::RecoveryObservation & observation) {
        return wait_for_recovery(
          handle, previous_recovery_diagnostic_signature, canceled, observation);
      };
    callbacks.wait_for_batch_continuation = [this, handle](
      const CycleCoordinator::CancelPredicateFn & canceled) {
        std::unique_lock<std::mutex> lock(recovery_mutex_);
        const auto & mission_goal = *handle->get_goal();
        const auto ready = [this, &mission_goal]() {
            const auto now = std::chrono::steady_clock::now();
            const auto safety = safety_observation_locked(MaterialAdmissionGate::Clock::now());
            const bool calibration_fresh = vision_calibration_received_ &&
              now >= latest_vision_calibration_receipt_ &&
              now - latest_vision_calibration_receipt_ <= kVisionCalibrationStatusFreshnessTimeout;
            const bool delivery_admitted = material_admission_gate_.has_admitted_delivery(
              mission_goal.delivery_id, mission_goal.target_id);
            const bool motion_fresh = latest_motion_receipt_ !=
              std::chrono::steady_clock::time_point{} && now >= latest_motion_receipt_ &&
              now - latest_motion_receipt_ <= kMotionStatusFreshnessTimeout;
            return batch_continuation_ready(
              safety, delivery_admitted, calibration_fresh,
              latest_vision_calibration_.state, latest_motion_, motion_fresh);
          };
        while (!(canceled && canceled())) {
          if (ready()) {
            return true;
          }
          recovery_cv_.wait_for(lock, std::chrono::milliseconds(100));
        }
        return false;
      };
    callbacks.read_recovery_entry = [this, key]() {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        return read_recovery_entry(key, recovery_entries_);
      };
    callbacks.record_recovery_result = [this, handle](
                                           const CycleCoordinator::MotionResult
                                               &result) {
      if (result.code.value !=
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        RCLCPP_WARN(get_logger(),
                    "controlled RETRACT failed "
                    " result_code=%s diagnostic=%s",
                    motion_result_code_field(result.code.value,
                                             result.result_code_available)
                        .c_str(),
                    result.diagnostic_detail.c_str());
      }
      RCLCPP_DEBUG(get_logger(),
                   "controlled recovery result "
                   "original_mission_result=MISSION_EXIT_SAFETY_PREEMPTED "
                   "result_code=%s diagnostic=%s",
                   motion_result_code_field(result.code.value,
                                            result.result_code_available)
                       .c_str(),
                   result.diagnostic_detail.c_str());
    };
    callbacks.load_recipe = [this, selected_recipe_id](const std::string & target_id) {
        const auto loaded = load_recipe(target_id);
        if (!loaded.recipe) {
          RCLCPP_DEBUG(get_logger(), "recipe rejected source=%s reason=%s",
                       loaded.source.c_str(),
                       recipe_reason_token(loaded.reason));
          return std::optional<CycleCoordinator::MissionRecipe>{};
        }
        const auto & recipe = *loaded.recipe;
        *selected_recipe_id = recipe.recipe_id;
        RCLCPP_DEBUG(
          get_logger(), "recipe loaded recipe_id=%s",
          recipe.recipe_id.c_str());
        RCLCPP_DEBUG(get_logger(), "mission recipe selected recipe_id=%s",
                     recipe.recipe_id.c_str());
        return loaded.recipe;
      };

    callbacks.publish_phase = [this, handle, previous_phase, processed_count, retry_count,
      interruption_count, first_interruption_reason, first_interruption_causes,
      first_interruption_stop_mode, decorate_feedback](
      uint8_t phase) {
        if (phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_WAITING_RECOVERY) {
          ++(*interruption_count);
          const auto safety = read_safety(uuid_key(handle->get_goal_id()));
          if (*interruption_count == 1U) {
            first_interruption_reason->value =
              arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED;
            if (safety.delivery_valid) {
              first_interruption_causes->value = safety.state.active_causes.value |
                safety.state.latched_causes.value;
              *first_interruption_stop_mode = safety.state.selected_stop_mode;
            }
          }
        }
        auto feedback = std::make_shared<ExecuteCycle::Feedback>();
        feedback->phase.value = phase;
        decorate_feedback(*feedback);
        feedback->retry_count = *retry_count;
        handle->publish_feedback(feedback);
        if (*previous_phase != phase) {
          const auto & goal = *handle->get_goal();
          const char * name = phase ==
            arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_ENSURING_HOME ? "GO_HOME" :
            phase ==
            arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_DETECTING_TARGET ? "DETECT_TARGET"
            :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_PICKING ? "PICK" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_PLACING ? "PLACE" :
            phase ==
            arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_RETURNING_HOME ? "GO_HOME" :
            phase ==
            arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_WAITING_RECOVERY ? "WAIT_RECOVERY"
            :
            phase ==
            arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_RETRACTING ? "RETRACT" :
            "FINISHED";
          if (phase != arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_FINISHED) {
            RCLCPP_INFO(get_logger(),
                        "mission phase started phase=%s processed=%llu/%u",
                        name, static_cast<unsigned long long>(*processed_count),
                        goal.quantity);
          }
          *previous_phase = phase;
        }
      };
    callbacks.publish_processed_count = [this, handle, processed_count, retry_count,
      decorate_feedback](
      uint64_t count) {
        *processed_count = count;
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          const auto accounting_result = material_admission_gate_.record_successful_place(
            uuid_key(handle->get_goal_id()), count);
          if (accounting_result == MaterialAdmissionGate::PlaceCommitResult::DUPLICATE) {
            RCLCPP_DEBUG(get_logger(),
                         "duplicate PLACE accounting callback ignored "
                         "processed_count=%llu",
                         static_cast<unsigned long long>(count));
          } else if (
            accounting_result == MaterialAdmissionGate::PlaceCommitResult::ACCOUNTING_INCONSISTENT)
          {
            RCLCPP_ERROR(
                get_logger(),
                "PLACE succeeded but material accounting is inconsistent; "
                "later ExecuteCycle admission blocked processed_count=%llu",
                static_cast<unsigned long long>(count));
          }
        }
        auto feedback = std::make_shared<ExecuteCycle::Feedback>();
        feedback->phase.value = arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_PLACING;
        decorate_feedback(*feedback);
        feedback->batch_processed_count = count;
        feedback->retry_count = *retry_count;
        handle->publish_feedback(feedback);
      };
    callbacks.publish_retry_count = [this, handle, previous_phase, processed_count, retry_count,
      decorate_feedback](
      uint8_t count) {
        *retry_count = count;
        auto feedback = std::make_shared<ExecuteCycle::Feedback>();
        feedback->phase.value = *previous_phase;
        decorate_feedback(*feedback);
        feedback->retry_count = count;
        handle->publish_feedback(feedback);
      };
    return callbacks;
  }

  bool wait_for_recovery(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<std::optional<RecoveryObservationDiagnosticSignature>> &
    previous_diagnostic_signature,
    const CycleCoordinator::CancelPredicateFn & canceled,
    CycleCoordinator::RecoveryObservation & observation)
  {
    const auto execute_cycle_id = uuid_string(handle->get_goal_id());
    const auto & mission_goal = *handle->get_goal();
    const auto delivery_id = uuid_string(mission_goal.delivery_id.uuid);

    std::unique_lock<std::mutex> lock(recovery_mutex_);
    if (canceled && canceled()) {
      return false;
    }
    const auto current_motion = [this]() {return latest_motion_;};
    const auto current_safety = [this]() {
        return safety_observation_locked(MaterialAdmissionGate::Clock::now());
      };
    const auto motion_status_is_fresh = [this]() {
        const auto now = std::chrono::steady_clock::now();
        return latest_motion_receipt_ != std::chrono::steady_clock::time_point{} &&
               now >= latest_motion_receipt_ &&
               now - latest_motion_receipt_ <= kMotionStatusFreshnessTimeout;
      };
    const auto immediate_return_ready = [this, &current_motion]() {
        const auto safety = safety_observation_locked(MaterialAdmissionGate::Clock::now());
        const auto motion = current_motion();
        const auto now = std::chrono::steady_clock::now();
        const bool fresh = latest_motion_receipt_ != std::chrono::steady_clock::time_point{} &&
          now >= latest_motion_receipt_ &&
          now - latest_motion_receipt_ <= kMotionStatusFreshnessTimeout;
        return (recovery_ready_for_immediate_return(safety, motion) && fresh) ||
               recovery_ready_for_batch_continuation(safety, motion, fresh) ||
               recovery_severity_escalation_ready_for_immediate_return(safety);
      };
    if (recovery_generation_ != last_recovery_returned_generation_ && immediate_return_ready()) {
      last_recovery_returned_generation_ = recovery_generation_;
      auto result = CycleCoordinator::RecoveryObservation{current_safety(), current_motion()};
      result.motion_status_fresh = motion_status_is_fresh();
      const auto decision = std::chrono::steady_clock::now();
      const auto motion_receipt = latest_motion_receipt_;
      const auto motion_age = motion_receipt == std::chrono::steady_clock::time_point{} ? -1.0 :
      std::chrono::duration<double, std::milli>(decision - motion_receipt).count();
      const bool motion_fresh = motion_age >= 0.0 &&
        motion_age <= kMotionStatusFreshnessTimeout.count();
      const auto signature = recovery_observation_diagnostic_signature(
        result.safety, result.motion,
        motion_receipt != std::chrono::steady_clock::time_point{}, motion_fresh);
      const bool should_log = !previous_diagnostic_signature->has_value() ||
        previous_diagnostic_signature->value() != signature;
      previous_diagnostic_signature->emplace(signature);
      const char * wake_reason =
        recovery_ready_for_immediate_return(result.safety, result.motion) &&
        result.motion_status_fresh ? "retract_ready" :
        recovery_ready_for_batch_continuation(
          result.safety, result.motion, result.motion_status_fresh) ? "batch_continuation_ready" :
        "severity_escalation";
      lock.unlock();
      if (should_log) {
        RCLCPP_DEBUG(get_logger(),
                     "controlled recovery wait condition changed reason=%s",
                     wake_reason);
      }
      observation = result;
      return true;
    }

    const auto generation = recovery_generation_;
    recovery_cv_.wait(
      lock, [this, generation, &canceled]() {
        return recovery_generation_ != generation || (canceled && canceled());
      });
    if (canceled && canceled()) {
      lock.unlock();
      return false;
    }
    last_recovery_returned_generation_ = recovery_generation_;
    auto result = CycleCoordinator::RecoveryObservation{current_safety(), current_motion()};
    const auto decision = std::chrono::steady_clock::now();
    const auto motion_receipt = latest_motion_receipt_;
    const auto motion_age = motion_receipt == std::chrono::steady_clock::time_point{} ? -1.0 :
    std::chrono::duration<double, std::milli>(decision - motion_receipt).count();
    const bool motion_fresh = motion_age >= 0.0 &&
      motion_age <= kMotionStatusFreshnessTimeout.count();
    result.motion_status_fresh = motion_fresh;
    const auto signature = recovery_observation_diagnostic_signature(
      result.safety, result.motion,
      motion_receipt != std::chrono::steady_clock::time_point{}, motion_fresh);
    const bool should_log = !previous_diagnostic_signature->has_value() ||
      previous_diagnostic_signature->value() != signature;
    previous_diagnostic_signature->emplace(signature);
    const char * wake_reason =
      recovery_ready_for_immediate_return(result.safety, result.motion) && motion_fresh ?
      "retract_ready" : recovery_ready_for_batch_continuation(
        result.safety, result.motion, motion_fresh) ? "batch_continuation_ready" :
      recovery_severity_escalation_ready_for_immediate_return(result.safety) ?
      "severity_escalation" : !result.safety.delivery_valid ? "awaiting_current_Safety_permission" :
      !result.safety.recovery_motion_allowed() ? "awaiting_Safety_recovery_authorization" :
      result.motion.execution_active || result.motion.has_active_execution ||
      !result.motion.backend_inactivity_confirmed ? "awaiting_confirmed_Motion_inactivity" :
      "evaluating_stopped_Motion_and_holding";
    lock.unlock();
    if (should_log) {
      RCLCPP_DEBUG(get_logger(),
                   "controlled recovery wait condition changed reason=%s",
                   wake_reason);
    }
    observation = result;
    return true;
  }

  SafetyObservation safety_observation_locked(MaterialAdmissionGate::Clock::time_point now) const
  {
    return evaluate_safety_observation(
      latest_safety_, safety_state_received_, latest_safety_receipt_, now,
      std::chrono::milliseconds(safety_state_freshness_timeout_ms_));
  }

  SafetyObservation read_safety(const std::string & key)
  {
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    const auto observation = safety_observation_locked(MaterialAdmissionGate::Clock::now());
    observe_recovery_entry(key, observation, recovery_entries_);
    return observation;
  }

  DetectTarget::Response detect_target(
    const std::shared_ptr<GoalHandle> & handle, const DetectTarget::Request & request,
    const CycleCoordinator::CancelPredicateFn & cancel = {})
  {
    DetectTarget::Response response;
    const auto & goal = *handle->get_goal();
    const auto execute_cycle_id = uuid_string(handle->get_goal_id());
    const auto delivery_id = uuid_string(goal.delivery_id.uuid);
    const auto service_wait_budget = std::chrono::seconds(1);
    const auto service_deadline = std::chrono::steady_clock::now() + service_wait_budget;
    while (!detect_client_->service_is_ready() &&
      std::chrono::steady_clock::now() < service_deadline)
    {
      if (cancel && cancel()) {
        response.result_code.value =
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT;
        response.diagnostic_detail = "Vision acquisition canceled before request submission";
        return response;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!detect_client_->service_is_ready()) {
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SENSOR_ERROR;
      response.diagnostic_detail = "Vision service unavailable";
      RCLCPP_DEBUG(get_logger(),
                   "Vision service wait ended reason=service_unavailable");
      return response;
    }
    auto request_ptr = std::make_shared<DetectTarget::Request>(request);
    if (request_ptr->timeout.sec == 0 && request_ptr->timeout.nanosec == 0) {
      request_ptr->timeout.sec = static_cast<int32_t>(vision_request_timeout_ms_ / 1000);
      request_ptr->timeout.nanosec = static_cast<uint32_t>(
        (vision_request_timeout_ms_ % 1000) * 1'000'000);
    }
    const auto request_timeout = arm_cell_orchestration::vision_request_timeout(
      request_ptr->timeout);
    if (!request_timeout) {
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT;
      response.diagnostic_detail = "Vision timeout must be a non-negative duration";
      RCLCPP_DEBUG(get_logger(),
                   "vision request rejected "
                   "result_code=%s reason=invalid_timeout",
                   detect_target_result_code_name(response.result_code.value));
      return response;
    }
    RCLCPP_DEBUG(get_logger(), "vision request dispatched");
    auto future = detect_client_->async_send_request(request_ptr);
    const auto orchestration_wait_start = std::chrono::steady_clock::now();
    const auto response_wait_budget =
      arm_cell_orchestration::vision_response_wait_timeout(*request_timeout);
    const auto response_deadline = orchestration_wait_start + response_wait_budget;
    while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready &&
      std::chrono::steady_clock::now() < response_deadline)
    {
      if (cancel && cancel()) {
        detect_client_->remove_pending_request(future);
        response.result_code.value =
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT;
        response.diagnostic_detail = "Vision acquisition canceled";
        RCLCPP_DEBUG(get_logger(), "Vision response discarded reason=canceled");
        return response;
      }
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      detect_client_->remove_pending_request(future);
      const auto decision_time = std::chrono::steady_clock::now();
      const auto response_wait_ms = std::chrono::duration<double, std::milli>(
        decision_time - orchestration_wait_start).count();
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT;
      response.diagnostic_detail = "Vision request timed out";
      RCLCPP_DEBUG(get_logger(),
                   "Vision response wait ended reason=vision_response_timeout "
                   "elapsed_ms=%.0f",
                   response_wait_ms);
      return response;
    }
    auto vision_response = *future.get();
    const auto decision_time = std::chrono::steady_clock::now();
    const auto response_wait_ms = std::chrono::duration<double, std::milli>(
      decision_time - orchestration_wait_start).count();
    RCLCPP_DEBUG(
        get_logger(), "Vision response received result=%s elapsed_ms=%.0f",
        detect_target_result_code_name(vision_response.result_code.value),
        response_wait_ms);
    return vision_response;
  }

  bool wait_for_vision_idle(
    const std::shared_ptr<GoalHandle> & handle, uint64_t request_generation,
    const CycleCoordinator::CancelPredicateFn & cancel)
  {
    const auto wait_budget = std::chrono::seconds(30);
    const auto wait_start = std::chrono::steady_clock::now();
    const auto deadline = wait_start + wait_budget;
    const auto & mission_goal = *handle->get_goal();
    const auto execute_cycle_id = uuid_string(handle->get_goal_id());
    const auto delivery_id = uuid_string(mission_goal.delivery_id.uuid);
    RCLCPP_DEBUG(
        get_logger(),
        "Vision busy wait started reason=awaiting_current_idle_observation");
    std::unique_lock<std::mutex> lock(recovery_mutex_);
    const auto finish = [this, &lock, &handle, request_generation, wait_start, deadline,
        &execute_cycle_id, &delivery_id, &mission_goal](bool observed, const char * reason) {
        const auto has_observation = vision_diagnostic_received_;
        const auto state = latest_vision_diagnostic_.state;
        const auto decision_time = std::chrono::steady_clock::now();
        const auto observation_age_ns = has_observation &&
          decision_time >= latest_vision_diagnostic_receipt_ ?
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          decision_time - latest_vision_diagnostic_receipt_).count() : -1;
        const auto state_name = state == VisionDiagnostic::IDLE ? "IDLE" :
          state == VisionDiagnostic::DETECTING ? "DETECTING" : "UNKNOWN";
        if (diagnostic_capture_enabled_) {
          CycleCoordinator::DiagnosticRecord decision;
          decision.event = CycleCoordinator::DiagnosticEvent::VISION_IDLE_WAIT;
          decision.wait_started = true;
          decision.wait_reason_size = static_cast<std::uint8_t>(std::min(
              std::strlen(reason), decision.wait_reason.size()));
          std::copy_n(reason, decision.wait_reason_size, decision.wait_reason.begin());
          const auto to_ns = [](std::chrono::steady_clock::time_point value) {
              return std::chrono::duration_cast<std::chrono::nanoseconds>(
                value.time_since_epoch()).count();
            };
          decision.wait_start_steady_ns = to_ns(wait_start);
          decision.wait_deadline_steady_ns = to_ns(deadline);
          decision.wait_decision_steady_ns = to_ns(decision_time);
          decision.wait_elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            decision_time - wait_start).count();
          decision.vision_observation_age_ns = observation_age_ns;
          decision.requested_vision_generation = request_generation;
          decision.observed_vision_generation = vision_diagnostic_generation_;
          decision.vision_state = state;
          decision.vision_observation_received = has_observation;
          QueuedOrchestrationRecord queued;
          queued.decision = decision;
          std::copy_n(
            handle->get_goal_id().begin(), queued.execute_cycle_id.size(),
            queued.execute_cycle_id.begin());
          std::copy_n(
            mission_goal.delivery_id.uuid.begin(), queued.delivery_id.size(),
            queued.delivery_id.begin());
          const auto & target = mission_goal.target_id;
          if (target.size() <= queued.target_id.size()) {
            queued.target_id_size = static_cast<std::uint8_t>(target.size());
            std::copy_n(target.begin(), queued.target_id_size, queued.target_id.begin());
          }
          if ((target.size() > queued.target_id.size() || !diagnostic_capture_->enqueue(queued)) &&
            !diagnostic_drop_reported_.exchange(true))
          {
            RCLCPP_WARN(
              get_logger(),
              "Orchestration diagnostic record was dropped (queue, size, storage, or byte limit)");
          }
        }
        lock.unlock();
        RCLCPP_DEBUG(get_logger(),
                     "Vision busy wait ended reason=%s last_state=%s", reason,
                     has_observation ? state_name : "unavailable");
        return observed;
      };
    if (!vision_diagnostic_received_) {
      return finish(false, "vision_diagnostic_unavailable");
    }
    if (!safety_observation_locked(std::chrono::steady_clock::now()).normal_motion_allowed()) {
      return finish(false, "safety_permission_unavailable");
    }
    const auto initial_safety_generation = safety_preempt_generation_;
    while (true) {
      if (cancel && cancel()) {
        return finish(false, "mission_cancelled");
      }
      const auto now = std::chrono::steady_clock::now();
      const auto age = now - latest_vision_diagnostic_receipt_;
      if (age < std::chrono::steady_clock::duration::zero() || age > wait_budget ||
        now >= deadline)
      {
        return finish(false, now >= deadline ? "busy_idle_timeout" : "vision_diagnostic_stale");
      }
      if (vision_diagnostic_generation_ > request_generation &&
        latest_vision_diagnostic_.state == VisionDiagnostic::IDLE)
      {
        const auto generation = vision_diagnostic_generation_;
        recovery_cv_.wait_for(
          lock, std::chrono::milliseconds(50), [this, generation,
          initial_safety_generation, &cancel]() {
            return vision_diagnostic_generation_ != generation ||
            safety_preempt_generation_ != initial_safety_generation ||
            (cancel && cancel());
          });
        if (cancel && cancel()) {
          return finish(false, "mission_cancelled_during_idle_confirmation");
        }
        if (safety_preempt_generation_ != initial_safety_generation) {
          return finish(false, "safety_preempted_during_idle_confirmation");
        }
        if (vision_diagnostic_generation_ != generation) {
          continue;
        }
        const auto final_now = std::chrono::steady_clock::now();
        if (final_now - latest_vision_diagnostic_receipt_ <= wait_budget &&
          latest_vision_diagnostic_.state == VisionDiagnostic::IDLE)
        {
          return finish(true, "new_generation_idle_confirmed");
        }
        return finish(false, "idle_observation_stale_or_changed");
      }
      if (vision_diagnostic_generation_ > request_generation &&
        latest_vision_diagnostic_.state != VisionDiagnostic::DETECTING)
      {
        return finish(false, "diagnostic_transition_not_detecting_or_idle");
      }
      const auto generation = vision_diagnostic_generation_;
      recovery_cv_.wait_until(
        lock, deadline, [this, generation, initial_safety_generation,
        &cancel]() {
          return vision_diagnostic_generation_ != generation ||
          safety_preempt_generation_ != initial_safety_generation ||
          (cancel && cancel());
        });
      if (safety_preempt_generation_ != initial_safety_generation) {
        return finish(false, "safety_preempted_during_wait");
      }
    }
  }

  CycleCoordinator::MotionResult execute_task(
    const std::shared_ptr<GoalHandle> & mission_handle,
    const std::string & key, const MotionAction::Goal & goal,
    const std::string & recipe_id,
    const CycleCoordinator::MotionFeedbackFn & feedback = {},
    const CycleCoordinator::Submission & supplied_submission = {})
  {
    CycleCoordinator::MotionResult result;
    const auto & mission_goal = *mission_handle->get_goal();
    const auto record_motion_diagnostic = [this, mission_handle, &mission_goal](
      CycleCoordinator::DiagnosticRecord record) {
        if (!diagnostic_capture_enabled_) {
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          record.has_motion_status =
            latest_motion_receipt_ != std::chrono::steady_clock::time_point{};
          record.motion_status_generation = motion_status_generation_;
          if (record.has_motion_status) {
            record.motion_status_stamp_ns =
              static_cast<std::int64_t>(latest_motion_.header.stamp.sec) * 1000000000LL +
              latest_motion_.header.stamp.nanosec;
            const auto status_execution_id = uuid_string(latest_motion_.active_execution_id.uuid);
            record.motion_status_execution_id_size = static_cast<std::uint8_t>(std::min(
              status_execution_id.size(), record.motion_status_execution_id.size()));
            std::copy_n(status_execution_id.begin(), record.motion_status_execution_id_size,
              record.motion_status_execution_id.begin());
          }
          record.motion_observation_age_ns = record.has_motion_status && now >= latest_motion_receipt_ ?
            std::chrono::duration_cast<std::chrono::nanoseconds>(
            now - latest_motion_receipt_).count() : -1;
          if (record.has_motion_status) {
            record.motion_state = latest_motion_.execution_state;
            record.motion_active = latest_motion_.execution_active;
            record.has_active_execution = latest_motion_.has_active_execution;
            record.backend_inactivity_confirmed = latest_motion_.backend_inactivity_confirmed;
          }
        }
        QueuedOrchestrationRecord queued;
        queued.decision = record;
        std::copy_n(
          mission_handle->get_goal_id().begin(), queued.execute_cycle_id.size(),
          queued.execute_cycle_id.begin());
        std::copy_n(
          mission_goal.delivery_id.uuid.begin(), queued.delivery_id.size(), queued.delivery_id.begin());
        if (mission_goal.target_id.size() <= queued.target_id.size()) {
          queued.target_id_size = static_cast<std::uint8_t>(mission_goal.target_id.size());
          std::copy_n(mission_goal.target_id.begin(), queued.target_id_size, queued.target_id.begin());
        }
        if ((mission_goal.target_id.size() > queued.target_id.size() ||
          !diagnostic_capture_->enqueue(queued)) && !diagnostic_drop_reported_.exchange(true))
        {
          RCLCPP_WARN(
            get_logger(),
            "Orchestration diagnostic record was dropped (queue, size, storage, or byte limit)");
        }
      };
    const auto record_action_communication = [
      &record_motion_diagnostic, &goal](const char * stage, const std::string & detail) {
        CycleCoordinator::DiagnosticRecord record;
        record.event = CycleCoordinator::DiagnosticEvent::MOTION_ACTION_COMMUNICATION;
        record.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
        record.reason = CycleCoordinator::DiagnosticReason::ACTION_COMMUNICATION_UNKNOWN;
        record.motion_task_type = goal.task_type.value;
        record.result_available = false;
        record.result_detail_size = static_cast<std::uint8_t>(std::min(
          detail.size(), record.result_detail.size()));
        std::copy_n(detail.begin(), record.result_detail_size, record.result_detail.begin());
        record.wait_reason_size = static_cast<std::uint8_t>(std::min(
          std::strlen(stage), record.wait_reason.size()));
        std::copy_n(stage, record.wait_reason_size, record.wait_reason.begin());
        record_motion_diagnostic(record);
      };
    if (goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK) {
      RCLCPP_DEBUG(get_logger(), "recipe applied to PICK recipe_id=%s",
                   recipe_id.c_str());
    } else if (goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) {
      RCLCPP_DEBUG(get_logger(), "recipe applied to PLACE recipe_id=%s",
                   recipe_id.c_str());
    }
    RCLCPP_DEBUG(
        get_logger(), "mission motion dispatch recipe_id=%s task_type=%s",
        recipe_id.c_str(), motion_task_type_name(goal.task_type.value));
    if (!motion_client_->wait_for_action_server(std::chrono::seconds(1))) {
      result.diagnostic_detail = "Motion action unavailable";
      result.result_code_available = false;
      record_action_communication("action_server_wait", result.diagnostic_detail);
      return result;
    }
    using MotionGoalHandle = rclcpp_action::ClientGoalHandle<MotionAction>;
    auto feedback_supervision = std::make_shared<MotionFeedbackSupervision>();
    rclcpp_action::Client<MotionAction>::SendGoalOptions options;
    auto motion_submission = supplied_submission ? supplied_submission :
      std::make_shared<CancellationRegistry::ActionSubmission>();
    if (!cancellation_registry_.register_action_submission(key, motion_submission)) {
      result.diagnostic_detail = "Motion Goal blocked by an unresolved Action Goal";
      result.result_code_available = false;
      record_action_communication("active_goal_unresolved", result.diagnostic_detail);
      return result;
    }
    record_action_lifecycle(key, motion_submission->submission_id(), "motion", "submitted");
    motion_submission->set_cancel_observer([this, key, action_id = motion_submission->submission_id()]() {
        record_action_lifecycle(key, action_id, "motion", "cancel_requested");
      });
    options.goal_response_callback = [this, key, motion_submission, feedback_supervision](
      const MotionGoalHandle::SharedPtr & accepted_handle) {
        if (!accepted_handle) {
          record_action_lifecycle(key, motion_submission->submission_id(), "motion", "rejected");
          return;
        }
        motion_submission->set_goal_uuid(uuid_string(accepted_handle->get_goal_id()));
        record_action_lifecycle(key, motion_submission->submission_id(), "motion", "accepted",
          motion_submission->goal_uuid());
        feedback_supervision->accept(
          accepted_handle->get_goal_id(), std::chrono::steady_clock::now());
        motion_submission->set_cancel_fn([this, accepted_handle, key, action_id = motion_submission->submission_id()]() {
          record_action_lifecycle(key, action_id, "motion", "cancel_sent",
            uuid_string(accepted_handle->get_goal_id()));
          try {
                motion_client_->async_cancel_goal(accepted_handle);
              } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
                RCLCPP_DEBUG(get_logger(), "Action already terminal before cancel delivery");
              }
        });
      };
    const std::weak_ptr<CancellationRegistry::ActionSubmission> weak_motion = motion_submission;
    const auto motion_action_id = motion_submission->submission_id();
    options.result_callback = [this, key, motion_action_id, feedback_supervision, weak_motion](
      const MotionGoalHandle::WrappedResult & wrapped_result) {
        if (const auto submission = weak_motion.lock()) {submission->set_cancel_fn({});}
        feedback_supervision->terminal(wrapped_result.goal_id);
        record_action_lifecycle(key, motion_action_id, "motion", "terminal",
          uuid_string(wrapped_result.goal_id),
          wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
          wrapped_result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" :
          wrapped_result.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" : "UNKNOWN",
          wrapped_result.result ? wrapped_result.result->result_code.value : -1);
      };
    options.feedback_callback = [this, feedback, feedback_supervision, record_motion_diagnostic,
        mission_handle, &mission_goal, goal](
      const MotionGoalHandle::SharedPtr & motion_goal_handle,
      const std::shared_ptr<const MotionAction::Feedback> motion_feedback) {
        if (!motion_goal_handle) {
          return;
        }
        feedback_supervision->observe_and_forward(
          motion_goal_handle->get_goal_id(), motion_feedback->phase.value,
          std::chrono::steady_clock::now(),
          [this, feedback, feedback_supervision, record_motion_diagnostic, mission_handle,
            &mission_goal, goal, motion_goal_handle, motion_feedback](
            MotionFeedbackSupervision::Observation observation, std::uint8_t) {
            const auto motion_id = uuid_string(motion_goal_handle->get_goal_id());
            auto release_record = release_attempt_diagnostic(
              goal.task_type.value, *motion_feedback, motion_id);
            if (release_record) {
              record_motion_diagnostic(*release_record);
              const auto failure_name = motion_feedback->release_last_failure ==
                arm_cell_interfaces::action::ExecuteTask::Feedback::
                RELEASE_FAILURE_CONFIRMATION_TIMEOUT ? "release_confirmation_timeout" : "none";
              RCLCPP_INFO(
                  get_logger(),
                  "PLACE release OPEN progress "
                  " attempt=%u/%u last_failure=%s",
                  static_cast<unsigned>(motion_feedback->release_attempt),
                  static_cast<unsigned>(motion_feedback->release_retry_budget +
                                        1),
                  failure_name);
            }
            if (observation == MotionFeedbackSupervision::Observation::RECOVERED) {
              CycleCoordinator::DiagnosticRecord record;
              record.event = CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_RECOVERED;
              record.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
              record.reason = CycleCoordinator::DiagnosticReason::FEEDBACK_RECOVERED;
              record.observed_motion_phase = motion_feedback->phase.value;
              const auto motion_id = uuid_string(motion_goal_handle->get_goal_id());
              record.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
                motion_id.size(), record.motion_execution_id.size()));
              std::copy_n(motion_id.begin(), record.motion_execution_id_size,
                record.motion_execution_id.begin());
              record.wait_elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                feedback_supervision->last_gap()).count();
              record_motion_diagnostic(record);
              RCLCPP_WARN(get_logger(),
                          "Motion feedback recovered "
                          " phase=%u after %.3f seconds",
                          static_cast<unsigned>(motion_feedback->phase.value),
                          std::chrono::duration<double>(
                              feedback_supervision->last_gap())
                              .count());
            }
            if (feedback) {
              feedback(motion_feedback->phase.value);
            }
          });
      };
    using MotionGoalFuture = std::shared_future<MotionGoalHandle::SharedPtr>;
    MotionGoalFuture goal_future;
    result.code.value = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    result.result_code_available = false;
    const auto motion_action_started = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      goal_future = motion_client_->async_send_goal(goal, options);
    }
    if (goal_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      if (!result.result_code_available) {
        result.diagnostic_detail = "Motion goal acceptance timed out; submission outcome unknown";
        record_action_communication("goal_acceptance_wait", result.diagnostic_detail);
      }
      RCLCPP_DEBUG(get_logger(),
                   "Motion wait ended reason=goal_acceptance_timeout");
      return result;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      cancellation_registry_.clear_active_action(key, motion_submission);
      if (!result.result_code_available) {
        result.diagnostic_detail =
          "Motion Action Goal rejected before an ExecuteTask Result was available";
      }
      RCLCPP_DEBUG(get_logger(),
                   "Motion Goal rejected reason=protocol_goal_rejected");
      return result;
    }
    result.motion_execution_id = uuid_string(goal_handle->get_goal_id());
    CycleCoordinator::DiagnosticRecord accepted_record;
    accepted_record.event = CycleCoordinator::DiagnosticEvent::MOTION_GOAL_ACCEPTED;
    accepted_record.motion_task_type = goal.task_type.value;
    accepted_record.decision = CycleCoordinator::DiagnosticDecision::ALLOWED;
    accepted_record.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
      result.motion_execution_id.size(), accepted_record.motion_execution_id.size()));
    std::copy_n(result.motion_execution_id.begin(), accepted_record.motion_execution_id_size,
      accepted_record.motion_execution_id.begin());
    record_motion_diagnostic(accepted_record);
    RCLCPP_DEBUG(get_logger(), "mission motion accepted task_type=%s",
                 motion_task_type_name(goal.task_type.value));
    auto result_future = motion_client_->async_get_result(goal_handle);
    RCLCPP_DEBUG(get_logger(),
                 "Motion result wait started reason=awaiting_terminal_result");
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      const auto now = std::chrono::steady_clock::now();
      if (feedback_supervision->stale(now)) {
        CycleCoordinator::DiagnosticRecord record;
        record.event = CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_GAP;
        record.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
        record.reason = CycleCoordinator::DiagnosticReason::FEEDBACK_MISSING;
        record.observed_motion_phase = feedback_supervision->phase();
        record.wait_elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          now - feedback_supervision->last_feedback()).count();
        record.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
          result.motion_execution_id.size(), record.motion_execution_id.size()));
        std::copy_n(result.motion_execution_id.begin(), record.motion_execution_id_size,
          record.motion_execution_id.begin());
        record_motion_diagnostic(record);
        RCLCPP_WARN(get_logger(),
                    "Motion feedback missing for at least 1.5 seconds "
                    " phase=%u; Action remains supervised",
                    static_cast<unsigned>(record.observed_motion_phase));
      }
      if (!motion_client_->action_server_is_ready() &&
        !motion_client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        cancellation_registry_.clear_active_action(key, motion_submission);
        if (!result.result_code_available) {
          result.diagnostic_detail = "Motion action server lost before terminal result";
          record_action_communication("terminal_result_wait", result.diagnostic_detail);
        }
        RCLCPP_DEBUG(get_logger(),
                     "Motion wait ended "
                     "reason=action_server_lost_before_terminal_result");
        return result;
      }
    }
    const auto motion_result = result_future.get();
    feedback_supervision->terminal(goal_handle->get_goal_id());
    if (!motion_result.result) {
      cancellation_registry_.clear_active_action(key, motion_submission);
      if (!result.result_code_available) {
        result.diagnostic_detail = "Motion Action ended without an ExecuteTask Result";
        record_action_communication("terminal_result_payload", result.diagnostic_detail);
      }
      return result;
    }
    const char * action_terminal =
      motion_result.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
      motion_result.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" :
      motion_result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "UNKNOWN";
    result.code = motion_result.result->result_code;
    result.result_code_available = true;
    result.object_disposition = motion_result.result->object_disposition;
    result.diagnostic_detail = motion_result.result->diagnostic_detail;
    result.action_terminal = action_terminal;
    result.pick_grasp_insertion_started =
      motion_result.result->pick_grasp_insertion_started;
    result.pick_grasp_insertion_started_available = true;
    CycleCoordinator::DiagnosticRecord action_result_record;
    action_result_record.event = CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT;
    action_result_record.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
    action_result_record.failure_code = result.code.value;
    action_result_record.result_available = true;
    action_result_record.reason = result.code.value ==
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY ?
      CycleCoordinator::DiagnosticReason::UNEXPECTED_BUSY : CycleCoordinator::DiagnosticReason::NONE;
    action_result_record.decision = result.code.value ==
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY ?
      CycleCoordinator::DiagnosticDecision::STOP : CycleCoordinator::DiagnosticDecision::NONE;
    action_result_record.observed_motion_phase = feedback_supervision->phase();
    action_result_record.motion_execution_id_size = static_cast<std::uint8_t>(std::min(
      result.motion_execution_id.size(), action_result_record.motion_execution_id.size()));
    std::copy_n(result.motion_execution_id.begin(), action_result_record.motion_execution_id_size,
      action_result_record.motion_execution_id.begin());
    action_result_record.result_detail_size = static_cast<std::uint8_t>(std::min(
      result.diagnostic_detail.size(), action_result_record.result_detail.size()));
    std::copy_n(result.diagnostic_detail.begin(), action_result_record.result_detail_size,
      action_result_record.result_detail.begin());
    action_result_record.motion_action_terminal_size = static_cast<std::uint8_t>(std::min(
      result.action_terminal.size(), action_result_record.motion_action_terminal.size()));
    std::copy_n(result.action_terminal.begin(), action_result_record.motion_action_terminal_size,
      action_result_record.motion_action_terminal.begin());
    action_result_record.motion_action_elapsed_ns = std::chrono::duration_cast<
      std::chrono::nanoseconds>(std::chrono::steady_clock::now() - motion_action_started).count();
    record_motion_diagnostic(action_result_record);
    if (motion_result.code != rclcpp_action::ResultCode::SUCCEEDED &&
      result.code.value == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),
                   "Motion Action terminal state conflicts with its task "
                   "Result action_terminal=%s result=%s",
                   action_terminal, motion_result_code_name(result.code.value));
    }
    RCLCPP_DEBUG(get_logger(),
                 "Motion result wait ended action_terminal=%s result=%s",
                 action_terminal,
                 result.result_code_available
                     ? motion_result_code_name(result.code.value)
                     : "UNAVAILABLE");
    if (result.code.value == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      const char * operation =
        goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ? "PICK" :
        goal.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE ? "PLACE" :
        goal.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME ? "GO_HOME" :
        goal.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_RETRACT ? "RETRACT" :
        "UNKNOWN";
      RCLCPP_INFO(get_logger(),
                  "mission operation completed operation=%s result=%s",
                  operation, motion_result_code_name(result.code.value));
    }
    cancellation_registry_.clear_active_action(key, motion_submission);
    if (result.result_code_available &&
      result.code.value == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY)
    {
      RCLCPP_ERROR(get_logger(),
                   "Motion returned unexpected BUSY; mission operation will "
                   "fail without resubmission "
                   " result=%s reason=%s",
                   motion_result_code_name(result.code.value),
                   result.diagnostic_detail.c_str());
    }
    return result;
  }

  CycleCoordinator::PnpResult execute_pnp(
    const std::string & key, const PnpAction::Goal & goal, const std::string & recipe_id,
    const CycleCoordinator::PnpFeedbackFn & feedback,
    const CycleCoordinator::Submission & supplied_submission = {})
  {
    CycleCoordinator::PnpResult result;
    result.detail_code.value =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    if (!pnp_client_->wait_for_action_server(std::chrono::seconds(1))) {
      result.diagnostic_detail = "ExecutePnp action unavailable";
      return result;
    }
    using PnpGoalHandle = rclcpp_action::ClientGoalHandle<PnpAction>;
    rclcpp_action::Client<PnpAction>::SendGoalOptions options;
    auto submission = supplied_submission ? supplied_submission :
      std::make_shared<CancellationRegistry::ActionSubmission>();
    if (!cancellation_registry_.register_action_submission(key, submission)) {
      result.diagnostic_detail = "ExecutePnp blocked by an unresolved Action Goal";
      return result;
    }
    record_action_lifecycle(key, submission->submission_id(), "pnp", "submitted");
    submission->set_cancel_observer([this, key, action_id = submission->submission_id()]() {
        record_action_lifecycle(key, action_id, "pnp", "cancel_requested");
      });
    auto lifecycle = std::make_shared<TaskExecutionLifecycle<PnpAction>>();
    options.goal_response_callback = [this, lifecycle, submission, key](
      const PnpGoalHandle::SharedPtr & accepted) {
        if (accepted) {
          submission->set_goal_uuid(uuid_string(accepted->get_goal_id()));
          submission->set_cancel_fn(
            [this, accepted, key, action_id = submission->submission_id()]() {
              record_action_lifecycle(key, action_id, "pnp", "cancel_sent",
                uuid_string(accepted->get_goal_id()));
              try {
                pnp_client_->async_cancel_goal(accepted);
              } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
                RCLCPP_DEBUG(get_logger(), "Action already terminal before cancel delivery");
              }
            });
        }
        bool handed_off;
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          lifecycle->goal_response_received = true;
          handed_off = lifecycle->handed_off;
          if (!handed_off) {lifecycle->goal_handle = accepted;}
        }
        record_action_lifecycle(key, submission->submission_id(), "pnp",
          accepted ? "accepted" : "rejected", submission->goal_uuid());
        if (handed_off) {
          RCLCPP_DEBUG(get_logger(), "Interrupted Action received a late Goal response: %s",
            accepted ? "accepted" : "rejected");
        }
        recovery_cv_.notify_all();
      };
    options.feedback_callback = [this, feedback, lifecycle, key](
      const PnpGoalHandle::SharedPtr & handle,
      const std::shared_ptr<const PnpAction::Feedback> message) {
        std::lock_guard<std::mutex> feedback_handoff_lock(lifecycle->feedback_handoff_mutex);
        bool handed_off;
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          handed_off = lifecycle->handed_off;
        }
        if (handed_off) {
          (void)handle;
#ifdef ARM_CELL_ORCHESTRATION_TEST
          if (test_late_feedback_) {test_late_feedback_();}
#endif
        } else if (feedback && message) {feedback(message->next_acquisition_hint);}
      };
    const std::weak_ptr<CancellationRegistry::ActionSubmission> weak_submission = submission;
    const auto action_id = submission->submission_id();
    options.result_callback = [this, lifecycle, weak_submission, key, action_id](
      const PnpGoalHandle::WrappedResult & wrapped_result) {
        if (const auto action = weak_submission.lock()) {action->set_cancel_fn({});}
        bool handed_off;
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          handed_off = lifecycle->handed_off;
          if (!handed_off && !lifecycle->terminal_result) {
            lifecycle->terminal_result = wrapped_result;
          }
          lifecycle->goal_handle.reset();
        }
        record_action_lifecycle(key, action_id, "pnp",
            handed_off ? "late_terminal" : "terminal",
            uuid_string(wrapped_result.goal_id),
            wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
            wrapped_result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" :
            wrapped_result.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" : "UNKNOWN",
            wrapped_result.result ? wrapped_result.result->pnp_outcome : -1,
            wrapped_result.result ? static_cast<int>(wrapped_result.result->material_placed) : -1);
        if (handed_off) {
          RCLCPP_DEBUG(get_logger(), "Interrupted PnP Action received a late terminal result");
        }
        recovery_cv_.notify_all();
      };
    (void)pnp_client_->async_send_goal(goal, options);
    const auto lifecycle_event = await_task_execution_lifecycle<PnpAction>(submission, lifecycle);
    result.safety_preempted = lifecycle->safety_preempted;
    if (lifecycle_event == TaskExecutionLifecycleEvent::SAFETY_HANDOFF) {
      result.safety_preempted = true;
      result.safety_handoff = true;
      record_action_lifecycle(key, submission->submission_id(), "pnp", "handoff",
        submission->goal_uuid());
      RCLCPP_INFO(get_logger(), "Interrupted Action handed to Safety recovery after Motion inactivity");
      result.diagnostic_detail =
        "ExecutePnp handed to existing Safety recovery after confirmed Motion inactivity";
      cancellation_registry_.clear_active_action(key, submission);
      return result;
    }
    if (lifecycle_event == TaskExecutionLifecycleEvent::GOAL_REJECTED) {
      result.goal_rejected = true;
      result.diagnostic_detail = "ExecutePnp Goal rejected";
      cancellation_registry_.clear_active_action(key, submission);
      return result;
    }
    const auto & wrapped = *lifecycle->terminal_result;
    result.goal_uuid = submission->goal_uuid();
    result.action_terminal = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
      wrapped.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" :
      wrapped.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "UNKNOWN";
    if (!wrapped.result) {
      result.diagnostic_detail = "ExecutePnp ended without a result payload";
      cancellation_registry_.clear_active_action(key, submission);
      return result;
    }
    result.outcome = wrapped.result->pnp_outcome;
    result.material_placed = wrapped.result->material_placed;
    result.detail_code = wrapped.result->result_code;
    result.diagnostic_detail = wrapped.result->diagnostic_detail;
    result.result_code_available = true;
    if (result.outcome == PnpAction::Result::PNP_OUTCOME_COMPLETED && !result.material_placed) {
      result.outcome = PnpAction::Result::PNP_OUTCOME_FAILED;
      result.diagnostic_detail = "ExecutePnp COMPLETED result omitted material_placed";
    }
    RCLCPP_DEBUG(get_logger(),
      "ExecutePnp terminal recipe_id=%s goal_uuid=%s outcome=%u material_placed=%s action=%s",
      recipe_id.c_str(), result.goal_uuid.c_str(), static_cast<unsigned>(result.outcome),
      result.material_placed ? "true" : "false", result.action_terminal.c_str());
    cancellation_registry_.clear_active_action(key, submission);
    return result;
  }

  CycleCoordinator::DisturbResult execute_disturb(
    const std::string & key, const DisturbAction::Goal & goal,
    const CycleCoordinator::Submission & supplied_submission = {})
  {
    CycleCoordinator::DisturbResult result;
    if (!disturb_client_->wait_for_action_server(std::chrono::seconds(1))) {
      result.diagnostic_detail = "ExecuteDisturb action unavailable";
      return result;
    }
    using DisturbGoalHandle = rclcpp_action::ClientGoalHandle<DisturbAction>;
    rclcpp_action::Client<DisturbAction>::SendGoalOptions options;
    auto submission = supplied_submission ? supplied_submission :
      std::make_shared<CancellationRegistry::ActionSubmission>();
    if (!cancellation_registry_.register_action_submission(key, submission)) {
      result.diagnostic_detail = "ExecuteDisturb blocked by an unresolved Action Goal";
      return result;
    }
    record_action_lifecycle(key, submission->submission_id(), "disturb", "submitted");
    submission->set_cancel_observer([this, key, action_id = submission->submission_id()]() {
        record_action_lifecycle(key, action_id, "disturb", "cancel_requested");
      });
    auto lifecycle = std::make_shared<TaskExecutionLifecycle<DisturbAction>>();
    options.goal_response_callback = [this, lifecycle, submission, key](
      const DisturbGoalHandle::SharedPtr & accepted) {
        if (accepted) {
          submission->set_goal_uuid(uuid_string(accepted->get_goal_id()));
          submission->set_cancel_fn(
            [this, accepted, key, action_id = submission->submission_id()]() {
              record_action_lifecycle(key, action_id, "disturb", "cancel_sent",
                uuid_string(accepted->get_goal_id()));
              try {
                disturb_client_->async_cancel_goal(accepted);
              } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
                RCLCPP_DEBUG(get_logger(), "Action already terminal before cancel delivery");
              }
            });
        }
        bool handed_off;
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          lifecycle->goal_response_received = true;
          handed_off = lifecycle->handed_off;
          if (!handed_off) {lifecycle->goal_handle = accepted;}
        }
        record_action_lifecycle(key, submission->submission_id(), "disturb",
          accepted ? "accepted" : "rejected", submission->goal_uuid());
        if (handed_off) {
          RCLCPP_DEBUG(get_logger(), "Interrupted Action received a late Goal response: %s",
            accepted ? "accepted" : "rejected");
        }
        recovery_cv_.notify_all();
      };
    const std::weak_ptr<CancellationRegistry::ActionSubmission> weak_submission = submission;
    const auto action_id = submission->submission_id();
    options.result_callback = [this, lifecycle, weak_submission, key, action_id](
      const DisturbGoalHandle::WrappedResult & wrapped_result) {
        if (const auto action = weak_submission.lock()) {action->set_cancel_fn({});}
        bool handed_off;
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          handed_off = lifecycle->handed_off;
          if (!handed_off && !lifecycle->terminal_result) {
            lifecycle->terminal_result = wrapped_result;
          }
          lifecycle->goal_handle.reset();
        }
        record_action_lifecycle(key, action_id, "disturb",
            handed_off ? "late_terminal" : "terminal",
            uuid_string(wrapped_result.goal_id),
            wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
            wrapped_result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" :
            wrapped_result.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" : "UNKNOWN",
            wrapped_result.result ? wrapped_result.result->outcome.value : -1);
        if (handed_off) {
          RCLCPP_DEBUG(get_logger(), "Interrupted Disturb Action received a late terminal result");
        }
        recovery_cv_.notify_all();
      };
    (void)disturb_client_->async_send_goal(goal, options);
    const auto lifecycle_event =
      await_task_execution_lifecycle<DisturbAction>(submission, lifecycle);
    result.safety_preempted = lifecycle->safety_preempted;
    if (lifecycle_event == TaskExecutionLifecycleEvent::SAFETY_HANDOFF) {
      result.safety_preempted = true;
      result.safety_handoff = true;
      record_action_lifecycle(key, submission->submission_id(), "disturb", "handoff",
        submission->goal_uuid());
      RCLCPP_INFO(get_logger(), "Interrupted Action handed to Safety recovery after Motion inactivity");
      result.diagnostic_detail =
        "ExecuteDisturb handed to existing Safety recovery after confirmed Motion inactivity";
      cancellation_registry_.clear_active_action(key, submission);
      return result;
    }
    if (lifecycle_event == TaskExecutionLifecycleEvent::GOAL_REJECTED) {
      result.goal_rejected = true;
      result.diagnostic_detail = "ExecuteDisturb Goal rejected";
      cancellation_registry_.clear_active_action(key, submission);
      return result;
    }
    const auto & wrapped = *lifecycle->terminal_result;
    result.action_terminal = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
      wrapped.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" :
      wrapped.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "UNKNOWN";
    if (!wrapped.result) {
      if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
        result.outcome = arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_CANCELED;
      }
      result.diagnostic_detail = "ExecuteDisturb ended without a result payload";
      cancellation_registry_.clear_active_action(key, submission);
      return result;
    }
    result.outcome = wrapped.result->outcome.value;
    result.diagnostic_detail = wrapped.result->diagnostic_detail;
    RCLCPP_DEBUG(get_logger(),
      "ExecuteDisturb terminal acquisition_id=%s outcome=%u action=%s",
      goal.acquisition_id.c_str(), static_cast<unsigned>(result.outcome),
      result.action_terminal.c_str());
    cancellation_registry_.clear_active_action(key, submission);
    return result;
  }

  RecipeLoadResult load_recipe(
    const std::string & target_id) const
  {
    if (target_id != mission_target_id_) {
      return RecipeLoadResult{
        std::nullopt, RecipeDiagnosticReason::TARGET_MISMATCH, std::filesystem::path{}};
    }
    return recipe_repository_->load(target_id);
  }

  rclcpp::Client<DetectTarget>::SharedPtr detect_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr vision_calibration_client_;
  rclcpp::CallbackGroup::SharedPtr vision_calibration_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr vision_calibration_service_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr vision_calibration_service_;
  rclcpp_action::Client<MotionAction>::SharedPtr motion_client_;
  rclcpp_action::Client<PnpAction>::SharedPtr pnp_client_;
  rclcpp_action::Client<DisturbAction>::SharedPtr disturb_client_;
  rclcpp_action::Client<ExecuteCycle>::SharedPtr material_cycle_client_;
  rclcpp_action::Server<ExecuteCycle>::SharedPtr action_server_;
  rclcpp::Subscription<SafetyState>::SharedPtr safety_subscription_;
  rclcpp::Subscription<MaterialReadiness>::SharedPtr material_readiness_subscription_;
  rclcpp::Subscription<MotionStatus>::SharedPtr motion_subscription_;
  rclcpp::Subscription<VisionDiagnostic>::SharedPtr vision_diagnostic_subscription_;
  rclcpp::Subscription<VisionCalibration>::SharedPtr vision_calibration_subscription_;
  SafetyState latest_safety_;
  MaterialAdmissionGate::Clock::time_point latest_safety_receipt_{};
  bool safety_state_received_{false};
  bool cycle_in_progress_{false};
  bool calibration_in_progress_{false};
  MotionStatus latest_motion_;
  std::chrono::steady_clock::time_point latest_motion_receipt_{};
  std::uint64_t motion_status_generation_{0};
  VisionDiagnostic latest_vision_diagnostic_;
  std::chrono::steady_clock::time_point latest_vision_diagnostic_receipt_{};
  bool vision_diagnostic_received_{false};
  uint64_t vision_diagnostic_generation_{0};
  VisionCalibration latest_vision_calibration_;
  std::chrono::steady_clock::time_point latest_vision_calibration_receipt_{};
  bool vision_calibration_received_{false};
  std::mutex recovery_mutex_;
  std::condition_variable recovery_cv_;
  uint64_t recovery_generation_{0};
  uint64_t safety_preempt_generation_{0};
  uint64_t last_recovery_returned_generation_{0};
  RecoveryEntryMap recovery_entries_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> cancellation_started_;
  MaterialAdmissionGate material_admission_gate_;
  std::string last_material_admission_wait_delivery_key_;
  std::string last_material_admission_wait_reason_;
  std::unordered_map<std::string, MaterialAdmissionGate::Admission>
  authorized_material_deliveries_;
  CancellationRegistry cancellation_registry_;
  std::string mission_target_id_;
  std::unique_ptr<RecipeRepository> recipe_repository_;
  bool diagnostic_capture_enabled_{false};
  int64_t vision_request_timeout_ms_{30000};
  int safety_state_freshness_timeout_ms_{500};
  std::unique_ptr<OrchestrationDiagnosticCapture> diagnostic_capture_;
  std::atomic_bool diagnostic_drop_reported_{false};
};

}  // namespace arm_cell_orchestration

#ifndef ARM_CELL_ORCHESTRATION_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<arm_cell_orchestration::OrchestrationNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#endif  // ARM_CELL_ORCHESTRATION_TEST
