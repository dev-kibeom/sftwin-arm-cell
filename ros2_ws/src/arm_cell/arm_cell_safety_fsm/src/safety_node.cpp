#include "arm_cell_safety_fsm/safety_node.hpp"
#include "arm_cell_safety_fsm/source_timestamp.hpp"

#include <chrono>
#include <algorithm>
#include <array>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace arm_cell_safety_fsm
{

namespace
{
constexpr std::size_t kDiagnosticQueueCapacity = 64;
constexpr std::size_t kDiagnosticRecordLimit = 4096;
constexpr std::uint64_t kDiagnosticByteLimit = 1024 * 1024;

struct SafetyDiagnosticRecord
{
  enum class Kind : std::uint8_t {RESET, DECISION};
  Kind kind{Kind::DECISION};
  std::array<std::uint8_t, 16> request_id{};
  std::array<char, 64> motion_execution_id{};
  std::array<char, 256> reason{};
  std::array<char, 768> detail{};
  std::uint16_t reason_size{0};
  std::uint16_t detail_size{0};
  std::int64_t decision_steady_ns{0};
  std::int8_t previous_state{-1};
  std::uint8_t state{0};
  std::uint8_t capability{0};
  std::uint8_t stop_mode{0};
  std::uint32_t active_causes{0};
  std::uint32_t latched_causes{0};
  float max_velocity_scale{0.0F};
  float max_acceleration_scale{0.0F};
  bool operator_acknowledged{false};
  bool applied{false};
  bool e_stop_usable{false};
  bool e_stop_active{false};
  bool sto_active{false};
  bool inputs_fresh{false};
  bool envelope_valid{false};
  bool text_truncated{false};
  std::uint64_t local_sequence{0};
  std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)> inputs{};
};

template<std::size_t N>
std::uint16_t copy_bounded(std::array<char, N> & output, const std::string & input)
{
  const auto size = std::min(input.size(), N);
  std::copy_n(input.begin(), size, output.begin());
  return static_cast<std::uint16_t>(size);
}

const char * safety_state_name(uint8_t state);
const char * motion_capability_name(uint8_t capability);
const char * stop_mode_name(uint8_t mode);

std::string json_escape(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned int>(character) << std::dec;
        } else {
          escaped << character;
        }
    }
  }
  return escaped.str();
}

std::string json_escape(const char * value, std::size_t size)
{
  return json_escape(std::string(value, size));
}

std::string serialize_safety_record(const SafetyDiagnosticRecord & record, std::uint64_t sequence)
{
  std::ostringstream out;
  out << "{\"schema_version\":1,\"local_sequence\":" << sequence
      << ",\"time_basis\":\"local_steady_enqueue\",\"component\":\"safety\",\"event\":\""
      << (record.kind ==
  SafetyDiagnosticRecord::Kind::RESET ? "reset" : "decision_transition") << '\"';
  if (record.kind == SafetyDiagnosticRecord::Kind::RESET) {
    out << ",\"request_id\":\"" << std::hex << std::setfill('0');
    for (const auto byte : record.request_id) {
      out << std::setw(2) << static_cast<unsigned>(byte);
    }
    out << std::dec << "\",\"decision_steady_ns\":" << record.decision_steady_ns
        << ",\"operator_acknowledged\":" << (record.operator_acknowledged ? "true" : "false")
        << ",\"applied\":" << (record.applied ? "true" : "false")
        << ",\"reason\":\"" << json_escape(record.reason.data(), record.reason_size) << '\"'
        << ",\"reason_truncated\":" << (record.text_truncated ? "true" : "false");
  } else {
    out << ",\"motion_execution_id\":\""
        << json_escape(
      record.motion_execution_id.data(), static_cast<std::size_t>(
        std::find(record.motion_execution_id.begin(), record.motion_execution_id.end(), '\0') -
        record.motion_execution_id.begin()))
        << "\",\"decision_steady_ns\":" << record.decision_steady_ns
        << ",\"previous_state\":\"" << (record.previous_state < 0 ? "unavailable" :
    safety_state_name(static_cast<std::uint8_t>(record.previous_state)))
        << "\",\"state\":\"" << safety_state_name(record.state)
        << "\",\"capability\":\"" << motion_capability_name(record.capability)
        << "\",\"stop_mode\":\"" << stop_mode_name(record.stop_mode)
        << "\",\"active_causes\":" << record.active_causes
        << ",\"latched_causes\":" << record.latched_causes
        << ",\"e_stop\":\"" << (!record.e_stop_usable ? "unknown" :
    record.e_stop_active ? "active" : "inactive")
        << "\",\"sto\":\"" << (!record.e_stop_usable ? "unknown" :
    record.sto_active ? "active" : "inactive")
        << "\",\"inputs_fresh\":" << (record.inputs_fresh ? "true" : "false")
        << ",\"envelope_valid\":" << (record.envelope_valid ? "true" : "false")
        << ",\"max_velocity_scale\":" << record.max_velocity_scale
        << ",\"max_acceleration_scale\":" << record.max_acceleration_scale
        << ",\"required_inputs\":[";
    for (std::size_t i = 0; i < record.inputs.size(); ++i) {
      const auto & input = record.inputs[i];
      if (i != 0) {out << ',';}
      out << "{\"name\":\"" << safety_input_name(static_cast<SafetyInput>(i))
          << "\",\"received\":" << (input.received ? "true" : "false")
          << ",\"message_valid\":" << (input.message_valid ? "true" : "false")
          << ",\"source_stamp_plausible\":" << (input.source_timestamp_plausible ? "true" : "false")
          << ",\"source_stamp_ns\":" << (input.source_time_available ? input.source_stamp_ns : -1)
          << ",\"source_evaluation_ros_time_ns\":" <<
        (input.source_time_available ? input.source_evaluation_ros_time_ns : -1)
          << ",\"source_age_ns\":" << (input.source_time_available ? input.source_age_ns : -1)
          << ",\"source_maximum_age_ns\":" <<
        (input.source_time_available ? input.source_maximum_age_ns : -1)
          << ",\"source_time_basis\":\"ros_time_at_receipt\""
          << ",\"age_available\":" << (input.age_available ? "true" : "false")
          << ",\"age_ms\":" << (input.age_available ? input.receipt_age.count() : -1)
          << ",\"receipt_freshness_timeout_ns\":" << input.receipt_timeout_ns
          << ",\"reason\":\"" << safety_input_reason_name(input.reason) << "\"}";
    }
    out << "],\"detail\":\"" << json_escape(record.detail.data(), record.detail_size)
        << "\",\"detail_truncated\":" << (record.text_truncated ? "true" : "false");
  }
  out << '}';
  return out.str();
}

bool source_timestamp_plausible(
  const std_msgs::msg::Header & header, const rclcpp::Time & now,
  std::chrono::milliseconds maximum_age)
{
  return source_timestamp_is_plausible(
    rclcpp::Time(header.stamp).nanoseconds(), now.nanoseconds(), maximum_age);
}

std::string uuid_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid.uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

const char * safety_state_name(uint8_t state)
{
  switch (state) {
    case SafetyState::SAFETY_STATE_SAFE: return "SAFE";
    case SafetyState::SAFETY_STATE_INTERLOCKED: return "INTERLOCKED";
    case SafetyState::SAFETY_STATE_STOPPING: return "STOPPING";
    case SafetyState::SAFETY_STATE_STOPPED: return "STOPPED";
    case SafetyState::SAFETY_STATE_RECOVERY_REQUIRED: return "RECOVERY_REQUIRED";
    case SafetyState::SAFETY_STATE_EMERGENCY_STOPPED: return "EMERGENCY_STOPPED";
    default: return "UNKNOWN";
  }
}

const char * motion_capability_name(uint8_t capability)
{
  switch (capability) {
    case MotionCapability::MOTION_NONE: return "MOTION_NONE";
    case MotionCapability::MOTION_RECOVERY_ONLY: return "MOTION_RECOVERY_ONLY";
    case MotionCapability::MOTION_NORMAL: return "MOTION_NORMAL";
    default: return "UNKNOWN";
  }
}

const char * stop_mode_name(uint8_t mode)
{
  switch (mode) {
    case StopMode::STOP_MODE_CONTROLLED: return "STOP_MODE_CONTROLLED";
    case StopMode::STOP_MODE_IMMEDIATE: return "STOP_MODE_IMMEDIATE";
    case StopMode::STOP_MODE_EMERGENCY: return "STOP_MODE_EMERGENCY";
    default: return "UNKNOWN";
  }
}

std::string safety_causes_text(uint32_t causes)
{
  std::string names;
  const auto append = [&names, causes](uint32_t flag, const char * name) {
      if ((causes & flag) != 0U) {
        if (!names.empty()) {names += '|';}
        names += name;
      }
    };
  append(SafetyCauseSet::E_STOP, "E_STOP");
  append(SafetyCauseSet::STO, "STO");
  append(SafetyCauseSet::COMMUNICATION_LOSS, "COMMUNICATION_LOSS");
  append(SafetyCauseSet::COLLISION, "COLLISION");
  append(SafetyCauseSet::PREMATURE_UNDOCK, "PREMATURE_UNDOCK");
  append(SafetyCauseSet::PACKML_ABORT, "PACKML_ABORT");
  append(SafetyCauseSet::GRIPPER_FAILURE, "GRIPPER_FAILURE");
  append(SafetyCauseSet::OTHER, "OTHER");
  append(SafetyCauseSet::REQUIRED_INPUT_INVALID, "REQUIRED_INPUT_INVALID");
  constexpr uint32_t known_causes = SafetyCauseSet::E_STOP | SafetyCauseSet::STO |
    SafetyCauseSet::COMMUNICATION_LOSS | SafetyCauseSet::COLLISION |
    SafetyCauseSet::PREMATURE_UNDOCK | SafetyCauseSet::PACKML_ABORT |
    SafetyCauseSet::GRIPPER_FAILURE | SafetyCauseSet::OTHER |
    SafetyCauseSet::REQUIRED_INPUT_INVALID;
  if ((causes & ~known_causes) != 0U) {
    if (!names.empty()) {names += '|';}
    names += "UNKNOWN_CAUSE";
  }
  return names.empty() ? "NONE" : names;
}

std::string safety_decision_reason(
  const SafetyState & state,
  const std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)> & inputs)
{
  std::string reason;
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    if (inputs[i].reason != SafetyInputReason::OK) {
      if (!reason.empty()) {reason += "; ";}
      reason += safety_input_name(static_cast<SafetyInput>(i));
      reason += ':';
      reason += safety_input_reason_name(inputs[i].reason);
    }
  }
  if (state.active_causes.value != SafetyCauseSet::NONE) {
    if (!reason.empty()) {reason += "; ";}
    reason += "active cause=" + safety_causes_text(state.active_causes.value);
  }
  if (state.latched_causes.value != SafetyCauseSet::NONE) {
    if (!reason.empty()) {reason += "; ";}
    reason += "latched cause=" + safety_causes_text(state.latched_causes.value);
  }
  if (state.safety_state != SafetyState::SAFETY_STATE_SAFE && reason.empty()) {
    reason = "state=" + std::string(safety_state_name(state.safety_state));
  }
  return reason.empty() ? "no active Safety block" : reason;
}

}  // namespace

class SafetyDiagnosticCapture final
{
public:
  SafetyDiagnosticCapture(std::string directory, rclcpp::Logger logger)
  : directory_(std::move(directory)), logger_(std::move(logger)), writer_([this]() {write_loop();})
  {}

  ~SafetyDiagnosticCapture()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (writer_.joinable()) {writer_.join();}
  }

  bool enqueue(const SafetyDiagnosticRecord & record)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || failed_ || truncated_ || queue_.size() >= kDiagnosticQueueCapacity) {
        ++dropped_count_;
        return false;
      }
      auto bounded_record = record;
      bounded_record.local_sequence = ++sequence_;
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
        RCLCPP_WARN(logger_, "Safety diagnostic capture total storage limit is already full");
        return;
      }
      const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      std::ofstream output(
        std::filesystem::path(directory_) / ("safety-" + std::to_string(stamp) + ".jsonl"),
        std::ios::out | std::ios::binary);
      if (!output) {throw std::runtime_error("open failed");}
      std::uint64_t bytes = 0;
      for (;; ) {
        SafetyDiagnosticRecord record;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
          if (queue_.empty() && stopping_) {break;}
          record = std::move(queue_.front());
          queue_.pop_front();
          record_in_flight = true;
        }
        auto line = serialize_safety_record(record, record.local_sequence) + '\n';
        if (line.size() > kDiagnosticRecordLimit) {
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
            "Safety diagnostic capture reached its byte limit; further records are dropped");
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
        logger_, "Safety diagnostic capture failed; Safety behavior is unchanged: %s",
        error.what());
    }
  }

  std::string directory_;
  rclcpp::Logger logger_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<SafetyDiagnosticRecord> queue_;
  bool stopping_{false};
  bool failed_{false};
  bool truncated_{false};
  std::uint64_t sequence_{0};
  std::uint64_t dropped_count_{0};
  std::thread writer_;
};

SafetyNode::SafetyNode(const rclcpp::NodeOptions & options)
: Node("safety_node", options)
{
  const bool diagnostic_capture_enabled = declare_parameter<bool>(
    "diagnostics.capture_enabled", false);
  const auto diagnostics_directory = declare_parameter<std::string>(
    "diagnostics.output_directory", ".local_artifacts/observability/safety");
  if (diagnostic_capture_enabled) {
    diagnostic_capture_ = std::make_unique<SafetyDiagnosticCapture>(
      diagnostics_directory, get_logger());
  }
  const auto freshness_timeout_ms = declare_parameter<int64_t>(
    "freshness_timeout_ms", 500);
  if (freshness_timeout_ms < 0) {
    throw std::invalid_argument("freshness_timeout_ms must be non-negative");
  }
  const auto source_timestamp_max_age_ms = declare_parameter<int64_t>(
    "source_timestamp_max_age_ms", 2000);
  if (source_timestamp_max_age_ms < 0) {
    throw std::invalid_argument("source_timestamp_max_age_ms must be non-negative");
  }
  const auto source_timestamp_max_age = std::chrono::milliseconds(source_timestamp_max_age_ms);
  core_ = std::make_shared<SafetyCore>(std::chrono::milliseconds(freshness_timeout_ms));
  state_publisher_ = create_publisher<SafetyState>("/safety/state", rclcpp::QoS(10));
  hardware_subscription_ = create_subscription<SafetyHardwareState>(
    "/safety/hardware_state", rclcpp::QoS(10),
    [this, source_timestamp_max_age](const SafetyHardwareState::SharedPtr message) {
      const auto current_ros_time = now();
      const bool timestamp_plausible = source_timestamp_plausible(
        message->header, current_ros_time, source_timestamp_max_age);
      log_source_timestamp_quality_event(
        get_logger(), hardware_source_timestamp_tracker_.observe(timestamp_plausible),
        "Safety-HW");
      const auto source_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
      const auto evaluation_ros_time_ns = current_ros_time.nanoseconds();
      core_->update_hardware(
        *message, SafetyCore::Clock::now(), timestamp_plausible,
        source_stamp_ns, evaluation_ros_time_ns,
        std::chrono::duration_cast<std::chrono::nanoseconds>(source_timestamp_max_age).count());
    });
  motion_subscription_ = create_subscription<MotionStatus>(
    "/motion/state", rclcpp::QoS(10),
    [this, source_timestamp_max_age](const MotionStatus::SharedPtr message) {
      const auto current_ros_time = now();
      const bool timestamp_plausible = source_timestamp_plausible(
        message->header, current_ros_time, source_timestamp_max_age);
      const auto source_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
      const auto evaluation_ros_time_ns = current_ros_time.nanoseconds();
      log_source_timestamp_quality_event(
        get_logger(), motion_source_timestamp_tracker_.observe(timestamp_plausible),
        "MotionStatus");
      core_->update_motion(
        *message, SafetyCore::Clock::now(), timestamp_plausible, source_stamp_ns,
        evaluation_ros_time_ns,
        std::chrono::duration_cast<std::chrono::nanoseconds>(source_timestamp_max_age).count());
      const auto active_execution_id = uuid_string(message->active_execution_id);
      if (active_execution_id.find_first_not_of('0') != std::string::npos) {
        last_motion_execution_id_ = active_execution_id;
      }
      const bool stopped = message->execution_state == MotionStatus::MOTION_STATE_STOPPED &&
      !message->execution_active && !message->has_active_execution &&
      message->backend_inactivity_confirmed;
      if (!stopped) {
        actual_stop_confirmed_logged_ = false;
      } else if (awaiting_stop_confirmation_ &&
                 !actual_stop_confirmed_logged_) {
        RCLCPP_INFO(
            get_logger(),
            "MotionStatus confirmed actual STOPPED and backend inactivity");
        actual_stop_confirmed_logged_ = true;
        awaiting_stop_confirmation_ = false;
      }
    });
  stop_client_ = create_client<arm_cell_interfaces::srv::StopMotion>("/safety/stop_motion");
  reset_service_ = create_service<arm_cell_interfaces::srv::ResetSafety>(
    "/safety/reset",
    [this](
      const std::shared_ptr<arm_cell_interfaces::srv::ResetSafety::Request> request,
      std::shared_ptr<arm_cell_interfaces::srv::ResetSafety::Response> response) {
      std::string reset_detail;
      const auto reset_decision_time = SafetyCore::Clock::now();
      response->applied = core_->reset(
        request->operator_acknowledged, reset_decision_time, &reset_detail);
      response->diagnostic_detail = response->applied ?
      "Safety reset policy applied; observe /safety/state for canonical outcome" :
      "Safety reset rejected: " + reset_detail;
      if (diagnostic_capture_) {
        SafetyDiagnosticRecord record;
        record.kind = SafetyDiagnosticRecord::Kind::RESET;
        std::copy(
          request->request_id.uuid.begin(), request->request_id.uuid.end(),
          record.request_id.begin());
        record.decision_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          reset_decision_time.time_since_epoch()).count();
        record.operator_acknowledged = request->operator_acknowledged;
        record.applied = response->applied;
        record.reason_size = copy_bounded(
          record.reason,
          response->applied ? "reset_policy_applied" : reset_detail);
        record.text_truncated = record.reason_size == record.reason.size() &&
        (!response->applied && reset_detail.size() > record.reason.size());
        if (!diagnostic_capture_->enqueue(record) && !diagnostic_drop_reported_) {
          RCLCPP_WARN(
            get_logger(),
            "Safety diagnostic record was dropped (queue, size, storage, or byte limit)");
          diagnostic_drop_reported_ = true;
        }
      }
      RCLCPP_DEBUG(get_logger(),
                   "Safety diagnostic reset operator_acknowledged=%s "
                   "applied=%s reason=%s",
                   request->operator_acknowledged ? "true" : "false",
                   response->applied ? "true" : "false",
                   response->applied ? "reset_policy_applied"
                                     : reset_detail.c_str());
      if (!response->applied) {
        RCLCPP_DEBUG(get_logger(), "Safety reset rejected reason=%s",
                     reset_detail.c_str());
      }
      publish_state();
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {publish_state();});
}

void SafetyNode::dispatch_stop(const SafetyState & state)
{
  if (core_->motion_inactive()) {
    has_dispatched_stop_ = false;
    stop_request_pending_ = false;
    awaiting_stop_confirmation_ = false;
    actual_stop_confirmed_logged_ = false;
    stop_required_reported_ = false;
    stop_rejection_reported_ = false;
    stop_service_unavailable_reported_ = false;
    return;
  }
  if (!core_->should_dispatch_stop()) {
    stop_required_reported_ = false;
    stop_rejection_reported_ = false;
    stop_service_unavailable_reported_ = false;
    return;
  }

  const auto stop_mode = state.selected_stop_mode.value;
  const auto causes = state.active_causes.value;
  if (!stop_required_reported_ || stop_mode != last_reported_stop_mode_ ||
    causes != last_reported_stop_causes_)
  {
    RCLCPP_WARN(get_logger(),
                "Motion stop required selected_stop_mode=%s active_causes=%s",
                stop_mode_name(stop_mode), safety_causes_text(causes).c_str());
    stop_required_reported_ = true;
    last_reported_stop_mode_ = stop_mode;
    last_reported_stop_causes_ = causes;
  }
  if (stop_request_pending_) {
    return;
  }
  if (!stop_client_->service_is_ready()) {
    if (!stop_service_unavailable_reported_) {
      RCLCPP_WARN(
          get_logger(),
          "StopMotion service unavailable while Motion stop is required");
      stop_service_unavailable_reported_ = true;
    }
    return;
  }
  if (stop_service_unavailable_reported_) {
    RCLCPP_DEBUG(get_logger(), "StopMotion service recovered; direct stop can be requested");
  }
  stop_service_unavailable_reported_ = false;
  const bool new_cause = (causes & ~last_stop_causes_) != 0;
  const bool escalation = !has_dispatched_stop_ || stop_mode > last_stop_mode_;
  const bool mode_is_not_a_downgrade = !has_dispatched_stop_ || stop_mode >= last_stop_mode_;
  if (!mode_is_not_a_downgrade || (!escalation && !new_cause)) {
    return;
  }

  auto request = std::make_shared<arm_cell_interfaces::srv::StopMotion::Request>();
  request->request_id.uuid.fill(stop_request_sequence_++);
  request->stop_mode.value = stop_mode;
  request->causes.value = causes;
  const auto request_id = uuid_string(request->request_id);
  last_stop_request_id_ = request_id;
  stop_request_pending_ = true;
  has_dispatched_stop_ = true;
  awaiting_stop_confirmation_ = true;
  actual_stop_confirmed_logged_ = false;
  last_stop_mode_ = stop_mode;
  last_stop_causes_ = causes;
  const auto motion_execution_id = last_motion_execution_id_;
  if (!stop_rejection_reported_ ||
    stop_mode != last_rejected_stop_mode_ || causes != last_rejected_stop_causes_ ||
    motion_execution_id != last_rejected_motion_execution_id_)
  {
    RCLCPP_INFO(get_logger(),
                "StopMotion dispatched selected_stop_mode=%s active_causes=%s",
                stop_mode_name(stop_mode), safety_causes_text(causes).c_str());
  }
  stop_client_->async_send_request(
    request,
    [this, request_id, motion_execution_id, stop_mode, causes](
      rclcpp::Client<arm_cell_interfaces::srv::StopMotion>::SharedFuture future) {
      stop_request_pending_ = false;
      if (future.get()->accepted) {
        if (stop_rejection_reported_) {
          RCLCPP_DEBUG(get_logger(), "StopMotion request rejection recovered");
          stop_rejection_reported_ = false;
        }
        RCLCPP_DEBUG(get_logger(),
                     "StopMotion accepted for processing selected_stop_mode=%s "
                     "active_causes=%s; awaiting MotionStatus",
                     stop_mode_name(stop_mode),
                     safety_causes_text(causes).c_str());
      } else {
        if (!stop_rejection_reported_ ||
        stop_mode != last_rejected_stop_mode_ || causes != last_rejected_stop_causes_ ||
        motion_execution_id != last_rejected_motion_execution_id_)
        {
          RCLCPP_WARN(get_logger(),
                      "StopMotion rejected "
                      "selected_stop_mode=%s active_causes=%s",
                      stop_mode_name(stop_mode),
                      safety_causes_text(causes).c_str());
          stop_rejection_reported_ = true;
          last_rejected_stop_mode_ = stop_mode;
          last_rejected_stop_causes_ = causes;
          last_rejected_motion_execution_id_ = motion_execution_id;
        }
        has_dispatched_stop_ = false;
      }
    });
}

void SafetyNode::publish_state()
{
  const auto decision_time = SafetyCore::Clock::now();
  auto state = core_->evaluate(decision_time);
  const auto input_diagnostics = core_->input_diagnostics(decision_time);
  state.header.stamp = now();
  const auto decision_reason = safety_decision_reason(state, input_diagnostics);
  const bool state_changed = !has_logged_state_ ||
    state.safety_state != last_logged_state_.safety_state ||
    state.motion_capability.value != last_logged_state_.motion_capability.value ||
    state.selected_stop_mode.value != last_logged_state_.selected_stop_mode.value ||
    state.active_causes.value != last_logged_state_.active_causes.value ||
    state.latched_causes.value != last_logged_state_.latched_causes.value ||
    state.valid != last_logged_state_.valid ||
    state.required_inputs_fresh != last_logged_state_.required_inputs_fresh ||
    state.motion_envelope_valid != last_logged_state_.motion_envelope_valid ||
    state.max_velocity_scale != last_logged_state_.max_velocity_scale ||
    state.max_acceleration_scale != last_logged_state_.max_acceleration_scale ||
    state.diagnostic_detail != last_logged_state_.diagnostic_detail;
  const bool input_decision_changed = diagnostic_capture_ &&
    (!has_logged_input_diagnostics_ || safety_input_decision_changed(
      input_diagnostics, last_logged_input_diagnostics_));
  if (safety_diagnostic_record_required(
      static_cast<bool>(diagnostic_capture_), state_changed, input_decision_changed))
  {
    const auto & hardware_diagnostic =
      input_diagnostics[static_cast<std::size_t>(SafetyInput::HARDWARE)];
    const bool hardware_observation_usable =
      hardware_diagnostic.reason == SafetyInputReason::OK;
    if (diagnostic_capture_) {
      SafetyDiagnosticRecord record;
      record.kind = SafetyDiagnosticRecord::Kind::DECISION;
      copy_bounded(record.motion_execution_id, last_motion_execution_id_);
      record.decision_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        decision_time.time_since_epoch()).count();
      record.previous_state =
        has_logged_state_ ? static_cast<std::int8_t>(last_logged_state_.safety_state) : -1;
      record.state = state.safety_state;
      record.capability = state.motion_capability.value;
      record.stop_mode = state.selected_stop_mode.value;
      record.active_causes = state.active_causes.value;
      record.latched_causes = state.latched_causes.value;
      record.max_velocity_scale = state.max_velocity_scale;
      record.max_acceleration_scale = state.max_acceleration_scale;
      record.e_stop_usable = hardware_observation_usable;
      record.e_stop_active = (state.active_causes.value & SafetyCauseSet::E_STOP) != 0;
      record.sto_active = (state.active_causes.value & SafetyCauseSet::STO) != 0;
      record.inputs_fresh = state.required_inputs_fresh;
      record.envelope_valid = state.motion_envelope_valid;
      record.inputs = input_diagnostics;
      record.detail_size = copy_bounded(record.detail, state.diagnostic_detail);
      record.text_truncated = record.detail_size == record.detail.size() &&
        state.diagnostic_detail.size() > record.detail.size();
      if (!diagnostic_capture_->enqueue(record) && !diagnostic_drop_reported_) {
        RCLCPP_WARN(
          get_logger(),
          "Safety diagnostic record was dropped (queue, size, storage, or byte limit)");
        diagnostic_drop_reported_ = true;
      }
    }
  }
  if (state_changed || input_decision_changed) {
    last_logged_state_ = state;
    has_logged_state_ = true;
    if (diagnostic_capture_) {
      last_logged_input_diagnostics_ = input_diagnostics;
      has_logged_input_diagnostics_ = true;
    }
  }
  const bool state_output_changed = !has_operator_logged_state_ ||
    state.safety_state != last_operator_logged_state_.safety_state ||
    state.motion_capability.value != last_operator_logged_state_.motion_capability.value ||
    state.selected_stop_mode.value != last_operator_logged_state_.selected_stop_mode.value ||
    state.active_causes.value != last_operator_logged_state_.active_causes.value ||
    state.latched_causes.value != last_operator_logged_state_.latched_causes.value ||
    state.valid != last_operator_logged_state_.valid ||
    state.required_inputs_fresh != last_operator_logged_state_.required_inputs_fresh ||
    state.motion_envelope_valid != last_operator_logged_state_.motion_envelope_valid;
  const bool decision_reason_changed = decision_reason != last_logged_decision_reason_;
  if (state_output_changed || decision_reason_changed) {
    const bool degraded = state.safety_state != SafetyState::SAFETY_STATE_SAFE ||
      state.motion_capability.value == MotionCapability::MOTION_NONE ||
      state.active_causes.value != SafetyCauseSet::NONE ||
      state.latched_causes.value != SafetyCauseSet::NONE || !state.valid ||
      !state.required_inputs_fresh || !state.motion_envelope_valid;
    if (!state_output_changed) {
      RCLCPP_DEBUG(
        get_logger(),
        "Safety decision explanation changed while output remained state=%s capability=%s reason=%s",
        safety_state_name(state.safety_state),
        motion_capability_name(state.motion_capability.value), decision_reason.c_str());
    } else if (degraded) {
      RCLCPP_WARN(
        get_logger(), "Safety output changed state=%s capability=%s stop_mode=%s active_causes=%s latched_causes=%s reason=%s", safety_state_name(
          state.safety_state), motion_capability_name(state.motion_capability.value), stop_mode_name(
          state.selected_stop_mode.value), safety_causes_text(
          state.active_causes.value).c_str(), safety_causes_text(
          state.latched_causes.value).c_str(), decision_reason.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Safety output changed state=%s capability=%s stop_mode=%s reason=%s", safety_state_name(
          state.safety_state), motion_capability_name(state.motion_capability.value), stop_mode_name(
          state.selected_stop_mode.value), decision_reason.c_str());
    }
    last_operator_logged_state_ = state;
    has_operator_logged_state_ = true;
    last_logged_decision_reason_ = decision_reason;
  }
  state_publisher_->publish(state);
  dispatch_stop(state);
}

SafetyNode::~SafetyNode() = default;

}  // namespace arm_cell_safety_fsm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_cell_safety_fsm::SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
