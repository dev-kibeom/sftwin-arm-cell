#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/srv/request_material.hpp>
#include <arm_cell_interfaces/srv/request_material_supply.hpp>

#include "arm_cell_integration/material_handoff_coordinator.hpp"

namespace arm_cell_integration
{

class MaterialHandoffNode final : public rclcpp::Node
{
public:
  using Clock = MaterialHandoffCoordinator::Clock;
  using RequestMaterialSupply = arm_cell_interfaces::srv::RequestMaterialSupply;
  using RequestMaterial = arm_cell_interfaces::srv::RequestMaterial;
  using HandoffState = arm_cell_interfaces::msg::MaterialHandoffState;
  using Readiness = arm_cell_interfaces::msg::MaterialReadiness;

  MaterialHandoffNode()
  : Node("material_handoff_node"), coordinator_(
      std::chrono::milliseconds(declare_parameter<int64_t>("transfer_duration_ms", 100)),
      std::chrono::milliseconds(
        declare_parameter<int64_t>("source_freshness_timeout_ms", 500)))
  {
    readiness_publisher_ = create_publisher<Readiness>(
      "/integration/material_readiness",
      rclcpp::QoS(1).reliable().transient_local());
    vda_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    handoff_subscription_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    vda_client_ = create_client<RequestMaterial>(
      "/vda/request_material", rmw_qos_profile_services_default,
      vda_client_callback_group_);
    rclcpp::SubscriptionOptions handoff_subscription_options;
    handoff_subscription_options.callback_group = handoff_subscription_callback_group_;
      handoff_subscription_ = create_subscription<HandoffState>(
      "/vda/material_handoff_state", rclcpp::QoS(10),
      [this](const HandoffState::SharedPtr state) {
        std::optional<Readiness> output;
        {
          std::lock_guard<std::mutex> lock(coordinator_mutex_);
          output = coordinator_.observe(
            *state, Clock::now(), static_cast<builtin_interfaces::msg::Time>(now()));
        }
        if (output) {
          publish_readiness(*output);
        }
      }, handoff_subscription_options);
    request_service_ = create_service<RequestMaterialSupply>(
      "/integration/request_material",
      [this](
        const std::shared_ptr<RequestMaterialSupply::Request> hub_request,
        std::shared_ptr<RequestMaterialSupply::Response> response) {
        std::lock_guard<std::mutex> lock(request_mutex_);
        const auto request_id = format_id(hub_request->request_id);
        RCLCPP_INFO(
          get_logger(), "material request received request_id=%s", request_id.c_str());
        const auto delivery_id = new_delivery_id();
        const auto delivery_key = format_id(delivery_id);
        response->accepted = false;
        if (!vda_client_->wait_for_service(std::chrono::seconds(1))) {
          response->diagnostic_detail = "VDA material request endpoint unavailable";
          RCLCPP_ERROR(
            get_logger(),
            "material request rejected request_id=%s delivery_id=%s reason=%s",
            request_id.c_str(), delivery_key.c_str(), response->diagnostic_detail.c_str());
          return;
        }
        {
          std::lock_guard<std::mutex> coordinator_lock(coordinator_mutex_);
          if (!coordinator_.can_begin(delivery_id)) {
            response->diagnostic_detail = "a material delivery is already active";
            RCLCPP_WARN(
              get_logger(), "material request rejected request_id=%s reason=%s",
              request_id.c_str(), response->diagnostic_detail.c_str());
            return;
          }
          if (!coordinator_.begin(delivery_id, Clock::now())) {
            response->diagnostic_detail = "unable to begin material delivery";
            RCLCPP_ERROR(
              get_logger(),
              "material request rejected request_id=%s delivery_id=%s reason=%s",
              request_id.c_str(), delivery_key.c_str(), response->diagnostic_detail.c_str());
            return;
          }
        }
        auto request = std::make_shared<RequestMaterial::Request>();
        request->delivery_id = delivery_id;
        auto future = vda_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
          {
            std::lock_guard<std::mutex> coordinator_lock(coordinator_mutex_);
            coordinator_.reject_unaccepted(delivery_id);
          }
          response->diagnostic_detail = "VDA material request timed out";
          RCLCPP_ERROR(
            get_logger(),
            "material request rejected request_id=%s delivery_id=%s reason=%s",
            request_id.c_str(), delivery_key.c_str(), response->diagnostic_detail.c_str());
          return;
        }
        const auto result = future.get();
        if (!result->accepted) {
          {
            std::lock_guard<std::mutex> coordinator_lock(coordinator_mutex_);
            coordinator_.reject_unaccepted(delivery_id);
          }
          response->diagnostic_detail = result->diagnostic_detail;
          RCLCPP_ERROR(
            get_logger(),
            "material request rejected request_id=%s delivery_id=%s reason=%s",
            request_id.c_str(), delivery_key.c_str(), response->diagnostic_detail.c_str());
          return;
        }
        response->accepted = true;
        response->delivery_id = delivery_id;
        response->diagnostic_detail = "delivery accepted: " + format_id(delivery_id);
        Readiness current_delivery;
        current_delivery.header.stamp = static_cast<builtin_interfaces::msg::Time>(
          now());
        current_delivery.delivery_id = delivery_id;
        current_delivery.material_ready = false;
        current_delivery.valid = true;
        publish_readiness(current_delivery);
        RCLCPP_INFO(
          get_logger(), "material request accepted request_id=%s delivery_id=%s",
          request_id.c_str(), delivery_key.c_str());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(25), [this]() {
      std::optional<Readiness> output;
      {
        std::lock_guard<std::mutex> lock(coordinator_mutex_);
        output = coordinator_.tick(
          Clock::now(), static_cast<builtin_interfaces::msg::Time>(now()));
      }
      if (output) {
        publish_readiness(*output);
      }
    });
  }

private:
  void publish_readiness(const Readiness & readiness)
  {
    const auto delivery_id = format_id(readiness.delivery_id);
    if (readiness.valid && readiness.material_ready) {
      RCLCPP_INFO(
        get_logger(), "material readiness changed delivery_id=%s state=READY",
        delivery_id.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "material readiness changed delivery_id=%s valid=%s state=NOT_READY",
        delivery_id.c_str(), readiness.valid ? "true" : "false");
    }
    readiness_publisher_->publish(readiness);
  }

  static arm_cell_interfaces::msg::MaterialHandoffState::SharedPtr new_state()
  {
    return std::make_shared<arm_cell_interfaces::msg::MaterialHandoffState>();
  }

  static MaterialHandoffCoordinator::DeliveryId new_delivery_id()
  {
    MaterialHandoffCoordinator::DeliveryId id;
    std::random_device random;
    for (auto & byte : id.uuid) {
      byte = static_cast<uint8_t>(random());
    }
    id.uuid[6] = static_cast<uint8_t>((id.uuid[6] & 0x0fU) | 0x40U);
    id.uuid[8] = static_cast<uint8_t>((id.uuid[8] & 0x3fU) | 0x80U);
    return id;
  }

  static std::string format_id(const MaterialHandoffCoordinator::DeliveryId & id)
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

  MaterialHandoffCoordinator coordinator_;
  std::mutex request_mutex_;
  std::mutex coordinator_mutex_;
  rclcpp::CallbackGroup::SharedPtr vda_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr handoff_subscription_callback_group_;
  rclcpp::Publisher<Readiness>::SharedPtr readiness_publisher_;
  rclcpp::Client<RequestMaterial>::SharedPtr vda_client_;
  rclcpp::Subscription<HandoffState>::SharedPtr handoff_subscription_;
  rclcpp::Service<RequestMaterialSupply>::SharedPtr request_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arm_cell_integration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<arm_cell_integration::MaterialHandoffNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
