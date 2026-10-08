#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <arm_cell_interfaces/srv/request_material.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>

#include "arm_cell_vda_adapter/external_state_mock.hpp"
#include "arm_cell_vda_adapter/material_delivery_mock.hpp"

namespace arm_cell_vda_adapter
{

class ExternalStateMockNode final : public rclcpp::Node
{
public:
  ExternalStateMockNode()
  : Node("external_state_mock"), material_delivery_(std::chrono::milliseconds(100))
  {
    const auto period_ms = declare_parameter<int64_t>("publish_period_ms", 100);
    if (period_ms <= 0) {
      throw std::invalid_argument("publish_period_ms must be greater than zero");
    }
    const auto degraded_period_ms = declare_parameter<int64_t>(
      "degraded_publish_period_ms", 0);
    if (degraded_period_ms != 0 && degraded_period_ms < period_ms) {
      throw std::invalid_argument(
              "degraded_publish_period_ms must be zero or at least publish_period_ms");
    }
    degraded_publish_period_ = std::chrono::milliseconds(degraded_period_ms);
    const auto material_phase_period_ms =
      declare_parameter<int64_t>("material_phase_period_ms", 100);
    if (material_phase_period_ms <= 0) {
      throw std::invalid_argument("material_phase_period_ms must be greater than zero");
    }
    material_delivery_ = MaterialDeliveryMock(std::chrono::milliseconds(material_phase_period_ms));

    amr_publisher_ = create_publisher<AMRDockingState>("/amr/docking_report", 10);
    packml_publisher_ = create_publisher<PackMLState>("/packml/state", 10);
    hardware_publisher_ = create_publisher<SafetyHardwareState>("/safety/hardware_state", 10);
    material_handoff_publisher_ = create_publisher<arm_cell_interfaces::msg::MaterialHandoffState>(
      "/vda/material_handoff_state", rclcpp::QoS(10));
    material_request_service_ = create_service<arm_cell_interfaces::srv::RequestMaterial>(
      "/vda/request_material",
      [this](
        const std::shared_ptr<arm_cell_interfaces::srv::RequestMaterial::Request> request,
        std::shared_ptr<arm_cell_interfaces::srv::RequestMaterial::Response> response) {
        response->accepted = material_delivery_.request(
          request->delivery_id, MaterialDeliveryMock::Clock::now());
        response->diagnostic_detail = response->accepted ?
        "delivery episode accepted" :
        "delivery identity is invalid, duplicated, or active";
        if (response->accepted) {
          publish_material_handoff();
        }
      });
    create_fault_service("material_handoff_failure", Fault::kMaterialHandoffFailure);

    create_fault_service("e_stop", Fault::kEStop);
    create_fault_service("premature_undock", Fault::kPrematureUndock);
    create_fault_service("packml_abort", Fault::kPackmlAbort);
    create_fault_service("invalid_state", Fault::kInvalidState);
    create_fault_service("communication_loss", Fault::kCommunicationLoss);
    create_fault_service("communication_degradation", Fault::kCommunicationDegradation);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms), [this]() {
        publish();
        publish_material_handoff();
      });
  }

private:
  void create_fault_service(const std::string & name, Fault fault)
  {
    fault_services_.push_back(
      create_service<std_srvs::srv::SetBool>(
        "~/fault/" + name,
        [this, fault, name](
          const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          mock_.set_fault(fault, request->data);
          if (name == "material_handoff_failure" && request->data) {
            material_delivery_.fail_active();
            publish_material_handoff();
          }
          response->success = true;
          response->message = request->data ? "fault activated" : "fault cleared";
          publish();
        }));
  }

  void publish()
  {
    const auto states = mock_.snapshot(now());
    if (!states.publish) {
      return;
    }
    const auto steady_now = std::chrono::steady_clock::now();
    if (
      states.communication_degraded && degraded_publish_period_.count() > 0 &&
      last_external_publish_.time_since_epoch().count() > 0 &&
      steady_now - last_external_publish_ < degraded_publish_period_)
    {
      return;
    }
    amr_publisher_->publish(states.amr);
    packml_publisher_->publish(states.packml);
    hardware_publisher_->publish(states.hardware);
    last_external_publish_ = steady_now;
  }

  void publish_material_handoff()
  {
    const auto state = material_delivery_.snapshot(
      MaterialDeliveryMock::Clock::now(),
      static_cast<builtin_interfaces::msg::Time>(now()));
    if (state) {
      material_handoff_publisher_->publish(*state);
    }
  }

  ExternalStateMock mock_;
  MaterialDeliveryMock material_delivery_;
  rclcpp::Publisher<AMRDockingState>::SharedPtr amr_publisher_;
  rclcpp::Publisher<PackMLState>::SharedPtr packml_publisher_;
  rclcpp::Publisher<SafetyHardwareState>::SharedPtr hardware_publisher_;
  rclcpp::Publisher<arm_cell_interfaces::msg::MaterialHandoffState>::SharedPtr
    material_handoff_publisher_;
  rclcpp::Service<arm_cell_interfaces::srv::RequestMaterial>::SharedPtr
    material_request_service_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> fault_services_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds degraded_publish_period_{0};
  std::chrono::steady_clock::time_point last_external_publish_{};
};

}  // namespace arm_cell_vda_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_cell_vda_adapter::ExternalStateMockNode>());
  rclcpp::shutdown();
  return 0;
}
