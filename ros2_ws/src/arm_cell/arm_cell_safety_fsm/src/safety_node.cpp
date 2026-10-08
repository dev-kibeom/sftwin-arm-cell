#include "arm_cell_safety_fsm/safety_node.hpp"
#include "arm_cell_safety_fsm/source_timestamp.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace arm_cell_safety_fsm
{

namespace
{
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
}  // namespace

SafetyNode::SafetyNode(const rclcpp::NodeOptions & options)
: Node("safety_node", options)
{
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
  const auto degraded_freshness_threshold_ms = declare_parameter<int64_t>(
    "degraded_freshness_threshold_ms", 0);
  const auto degraded_freshness_input_names = declare_parameter<std::vector<std::string>>(
    "degraded_freshness_inputs", std::vector<std::string>{"AMR", "PACKML"});
  const auto degraded_velocity_scale = declare_parameter<double>(
    "degraded_velocity_scale", 0.5);
  const auto degraded_acceleration_scale = declare_parameter<double>(
    "degraded_acceleration_scale", 0.5);
  if (
    degraded_freshness_threshold_ms < 0 ||
    (degraded_freshness_threshold_ms > 0 &&
    degraded_freshness_threshold_ms >= freshness_timeout_ms))
  {
    throw std::invalid_argument(
            "degraded_freshness_threshold_ms must be zero or less than freshness_timeout_ms");
  }
  std::vector<SafetyInput> degraded_freshness_inputs;
  for (const auto & input_name : degraded_freshness_input_names) {
    if (input_name == "AMR") {
      degraded_freshness_inputs.push_back(SafetyInput::AMR);
    } else if (input_name == "PACKML") {
      degraded_freshness_inputs.push_back(SafetyInput::PACKML);
    } else {
      throw std::invalid_argument(
              "degraded_freshness_inputs supports external communication inputs AMR and PACKML");
    }
  }
  if (
    !std::isfinite(degraded_velocity_scale) || degraded_velocity_scale <= 0.0 ||
    degraded_velocity_scale > 1.0 || !std::isfinite(degraded_acceleration_scale) ||
    degraded_acceleration_scale <= 0.0 || degraded_acceleration_scale > 1.0)
  {
    throw std::invalid_argument("degraded motion scales must be finite and in (0, 1]");
  }
  core_ = std::make_shared<SafetyCore>(
    std::chrono::milliseconds(freshness_timeout_ms),
    std::chrono::milliseconds(degraded_freshness_threshold_ms),
    static_cast<float>(degraded_velocity_scale),
    static_cast<float>(degraded_acceleration_scale), degraded_freshness_inputs);
  state_publisher_ = create_publisher<SafetyState>("/safety/state", rclcpp::QoS(10));
  amr_subscription_ = create_subscription<AMRDockingState>(
    "/amr/docking_report", rclcpp::QoS(10),
    [this, source_timestamp_max_age](const AMRDockingState::SharedPtr message) {
      core_->update_amr(
        *message, SafetyCore::Clock::now(),
        source_timestamp_plausible(message->header, now(), source_timestamp_max_age));
    });
  packml_subscription_ = create_subscription<PackMLState>(
    "/packml/state", rclcpp::QoS(10),
    [this, source_timestamp_max_age](const PackMLState::SharedPtr message) {
      core_->update_packml(
        *message, SafetyCore::Clock::now(),
        source_timestamp_plausible(message->header, now(), source_timestamp_max_age));
    });
  hardware_subscription_ = create_subscription<SafetyHardwareState>(
    "/safety/hardware_state", rclcpp::QoS(10),
    [this, source_timestamp_max_age](const SafetyHardwareState::SharedPtr message) {
      core_->update_hardware(
        *message, SafetyCore::Clock::now(),
        source_timestamp_plausible(message->header, now(), source_timestamp_max_age));
    });
  motion_subscription_ = create_subscription<MotionStatus>(
    "/motion/state", rclcpp::QoS(10),
    [this, source_timestamp_max_age](const MotionStatus::SharedPtr message) {
      core_->update_motion(
        *message, SafetyCore::Clock::now(),
        source_timestamp_plausible(message->header, now(), source_timestamp_max_age));
      const auto active_execution_id = uuid_string(message->active_execution_id);
      if (active_execution_id.find_first_not_of('0') != std::string::npos) {
        last_motion_execution_id_ = active_execution_id;
      }
      const bool stopped = message->execution_state == MotionStatus::MOTION_STATE_STOPPED &&
      !message->execution_active && !message->has_active_execution &&
      message->backend_inactivity_confirmed;
      if (!stopped) {
        actual_stop_confirmed_logged_ = false;
      } else if (awaiting_stop_confirmation_ && !actual_stop_confirmed_logged_) {
        const auto reported_request_id = message->has_last_stop_request ?
        uuid_string(message->last_stop_request_id) : last_stop_request_id_;
        RCLCPP_INFO(
          get_logger(),
          "MotionStatus confirmed actual STOPPED and backend inactivity request_id=%s dispatched_request_id=%s motion_execution_id=%s",
          reported_request_id.c_str(), last_stop_request_id_.c_str(),
          last_motion_execution_id_.c_str());
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
      response->applied = core_->reset(
        request->operator_acknowledged, SafetyCore::Clock::now());
      response->diagnostic_detail = response->applied ?
      "Safety reset policy applied; observe /safety/state for canonical outcome" :
      "Safety reset policy rejected the request; inspect /safety/state and current inputs";
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
    stop_service_unavailable_reported_ = false;
    return;
  }
  if (!core_->should_dispatch_stop()) {
    stop_required_reported_ = false;
    stop_service_unavailable_reported_ = false;
    return;
  }

  const auto stop_mode = state.selected_stop_mode.value;
  const auto causes = state.active_causes.value;
  if (!stop_required_reported_ || stop_mode != last_stop_mode_ || causes != last_stop_causes_) {
    RCLCPP_WARN(
      get_logger(),
      "Motion stop required motion_execution_id=%s selected_stop_mode=%u active_causes=%u",
      last_motion_execution_id_.c_str(), stop_mode, causes);
    stop_required_reported_ = true;
  }
  if (stop_request_pending_) {
    return;
  }
  if (!stop_client_->service_is_ready()) {
    if (!stop_service_unavailable_reported_) {
      RCLCPP_WARN(
        get_logger(),
        "StopMotion service unavailable while Motion stop is required motion_execution_id=%s",
        last_motion_execution_id_.c_str());
      stop_service_unavailable_reported_ = true;
    }
    return;
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
  RCLCPP_INFO(
    get_logger(),
    "StopMotion dispatched request_id=%s motion_execution_id=%s selected_stop_mode=%u active_causes=%u",
    request_id.c_str(), last_motion_execution_id_.c_str(), stop_mode, causes);
  stop_client_->async_send_request(
    request,
    [this, request_id, motion_execution_id, stop_mode, causes](
      rclcpp::Client<arm_cell_interfaces::srv::StopMotion>::SharedFuture future) {
      stop_request_pending_ = false;
      if (future.get()->accepted) {
        RCLCPP_INFO(
          get_logger(),
          "StopMotion accepted for processing request_id=%s motion_execution_id=%s selected_stop_mode=%u active_causes=%u; awaiting MotionStatus",
          request_id.c_str(), motion_execution_id.c_str(), stop_mode, causes);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "StopMotion rejected request_id=%s motion_execution_id=%s selected_stop_mode=%u active_causes=%u",
          request_id.c_str(), motion_execution_id.c_str(), stop_mode, causes);
        has_dispatched_stop_ = false;
      }
    });
}

void SafetyNode::publish_state()
{
  auto state = core_->evaluate(SafetyCore::Clock::now());
  state.header.stamp = now();
  if (!has_logged_state_ || state.safety_state != last_logged_state_.safety_state ||
    state.motion_capability.value != last_logged_state_.motion_capability.value ||
    state.selected_stop_mode.value != last_logged_state_.selected_stop_mode.value ||
    state.active_causes.value != last_logged_state_.active_causes.value ||
    state.latched_causes.value != last_logged_state_.latched_causes.value ||
    state.valid != last_logged_state_.valid ||
    state.required_inputs_fresh != last_logged_state_.required_inputs_fresh ||
    state.motion_envelope_valid != last_logged_state_.motion_envelope_valid ||
    state.max_velocity_scale != last_logged_state_.max_velocity_scale ||
    state.max_acceleration_scale != last_logged_state_.max_acceleration_scale ||
    state.diagnostic_detail != last_logged_state_.diagnostic_detail)
  {
    const bool degraded = state.safety_state != SafetyState::SAFETY_STATE_SAFE ||
      state.motion_capability.value == MotionCapability::MOTION_NONE ||
      (state.motion_envelope_valid &&
      (state.max_velocity_scale < 1.0F || state.max_acceleration_scale < 1.0F));
    if (degraded) {
      RCLCPP_WARN(
        get_logger(),
        "Safety transition state=%s capability=%u selected_stop_mode=%u active_causes=%u latched_causes=%u valid=%s inputs_fresh=%s envelope_valid=%s max_velocity_scale=%.3f max_acceleration_scale=%.3f diagnostic=%s",
        safety_state_name(state.safety_state), state.motion_capability.value,
        state.selected_stop_mode.value, state.active_causes.value, state.latched_causes.value,
        state.valid ? "true" : "false", state.required_inputs_fresh ? "true" : "false",
        state.motion_envelope_valid ? "true" : "false", state.max_velocity_scale,
        state.max_acceleration_scale,
        state.diagnostic_detail.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Safety transition state=%s capability=%u selected_stop_mode=%u active_causes=%u latched_causes=%u valid=%s inputs_fresh=%s envelope_valid=%s max_velocity_scale=%.3f max_acceleration_scale=%.3f",
        safety_state_name(state.safety_state), state.motion_capability.value,
        state.selected_stop_mode.value, state.active_causes.value, state.latched_causes.value,
        state.valid ? "true" : "false", state.required_inputs_fresh ? "true" : "false",
        state.motion_envelope_valid ? "true" : "false", state.max_velocity_scale,
        state.max_acceleration_scale);
    }
    last_logged_state_ = state;
    has_logged_state_ = true;
  }
  state_publisher_->publish(state);
  dispatch_stop(state);
}

}  // namespace arm_cell_safety_fsm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_cell_safety_fsm::SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
