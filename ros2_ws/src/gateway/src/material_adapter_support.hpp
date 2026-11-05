#pragma once

#include <algorithm>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>

namespace arm_cell_gateway::material_adapter_support
{

inline bool usable_id(const unique_identifier_msgs::msg::UUID & id)
{
  return std::any_of(id.uuid.begin(), id.uuid.end(), [](uint8_t byte) {return byte != 0;});
}

inline std::string format_id(const unique_identifier_msgs::msg::UUID & id)
{
  static constexpr char digits[] = "0123456789abcdef";
  std::string text;
  text.reserve(32);
  for (const auto byte : id.uuid) {
    text.push_back(digits[(byte >> 4U) & 0x0fU]);
    text.push_back(digits[byte & 0x0fU]);
  }
  return text;
}

inline bool source_is_fresh(
  const rclcpp::Node & node, const std_msgs::msg::Header & header,
  const rclcpp::Time & current, std::chrono::milliseconds timeout)
{
  const rclcpp::Time source(header.stamp, node.get_clock()->get_clock_type());
  const auto age = current - source;
  return age.nanoseconds() >= 0 && age <= rclcpp::Duration(timeout);
}

inline void publish_readiness(
  rclcpp::Node & node,
  const rclcpp::Publisher<arm_cell_interfaces::msg::MaterialReadiness>::SharedPtr & publisher,
  const arm_cell_interfaces::msg::MaterialReadiness & readiness,
  const char * blocking_reason = nullptr)
{
  if (readiness.valid && readiness.material_ready) {
    RCLCPP_INFO(node.get_logger(), "material readiness changed state=READY");
  } else if (blocking_reason == nullptr) {
    RCLCPP_INFO(node.get_logger(), "material readiness wait started reason=awaiting_transfer_completion");
  } else {
    RCLCPP_WARN(node.get_logger(), "material readiness cannot be confirmed reason=%s", blocking_reason);
  }
  publisher->publish(readiness);
}

}  // namespace arm_cell_gateway::material_adapter_support
