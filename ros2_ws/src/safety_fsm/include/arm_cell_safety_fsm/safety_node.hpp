#pragma once

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/srv/stop_motion.hpp>
#include <arm_cell_interfaces/srv/reset_safety.hpp>

#include "arm_cell_safety_fsm/safety_core.hpp"
#include "arm_cell_safety_fsm/source_timestamp.hpp"

namespace arm_cell_safety_fsm
{

class SafetyDiagnosticCapture;

class SafetyNode final : public rclcpp::Node
{
public:
  explicit SafetyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SafetyNode() override;

private:
  void publish_state();

  std::shared_ptr<SafetyCore> core_;
  std::unique_ptr<SafetyDiagnosticCapture> diagnostic_capture_;
  bool diagnostic_drop_reported_{false};
  void dispatch_stop(const SafetyState & state);
  rclcpp::Publisher<SafetyState>::SharedPtr state_publisher_;
  rclcpp::Subscription<SafetyHardwareState>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<MotionStatus>::SharedPtr motion_subscription_;
  rclcpp::Client<arm_cell_interfaces::srv::StopMotion>::SharedPtr stop_client_;
  rclcpp::Service<arm_cell_interfaces::srv::ResetSafety>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool stop_request_pending_{false};
  bool stop_service_unavailable_reported_{false};
  bool has_dispatched_stop_{false};
  bool stop_required_reported_{false};
  bool stop_rejection_reported_{false};
  uint8_t last_rejected_stop_mode_{0};
  uint32_t last_rejected_stop_causes_{0};
  std::string last_rejected_motion_execution_id_;
  uint8_t last_reported_stop_mode_{0};
  uint32_t last_reported_stop_causes_{0};
  bool awaiting_stop_confirmation_{false};
  bool actual_stop_confirmed_logged_{false};
  bool has_logged_state_{false};
  bool has_logged_input_diagnostics_{false};
  SourceTimestampAnomalyTracker hardware_source_timestamp_tracker_;
  SourceTimestampAnomalyTracker motion_source_timestamp_tracker_;
  SafetyState last_logged_state_;
  std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)>
  last_logged_input_diagnostics_{};
  SafetyState last_operator_logged_state_;
  bool has_operator_logged_state_{false};
  std::string last_logged_decision_reason_;
  std::string last_motion_execution_id_;
  std::string last_stop_request_id_;
  uint8_t last_stop_mode_{0};
  uint32_t last_stop_causes_{0};
  uint8_t stop_request_sequence_{0};
};

}  // namespace arm_cell_safety_fsm
