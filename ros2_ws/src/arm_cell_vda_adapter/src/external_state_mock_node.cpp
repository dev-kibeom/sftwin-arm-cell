#include <chrono>
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <arm_cell_interfaces/msg/handoff_trigger.hpp>
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
    accepted_handoff_subscription_ =
      create_subscription<arm_cell_interfaces::msg::HandoffTrigger>(
      "/integration/accepted_handoff", rclcpp::QoS(10),
      [this](const arm_cell_interfaces::msg::HandoffTrigger::SharedPtr trigger) {
        if (material_delivery_.consume_accepted_trigger(
          *trigger, MaterialDeliveryMock::Clock::now()))
        {
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
    create_safety_signal_loss_service();

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
          const bool accepted = mock_.set_fault(fault, request->data);
          if (name == "material_handoff_failure" && request->data) {
            material_delivery_.fail_active();
            publish_material_handoff();
          }
          response->success = accepted;
          response->message = accepted ?
            (request->data ? "fault activated" : "fault cleared") :
            "communication_loss and safety_signal_loss cannot be active together";
          publish();
        }));
  }

  void create_safety_signal_loss_service()
  {
    safety_signal_loss_service_ = create_service<std_srvs::srv::SetBool>(
      "~/fault/safety_signal_loss",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (request->data) {
          if (safety_signal_loss_timer_) {
            response->success = false;
            response->message = "safety signal loss is already active";
            return;
          }
          if (!mock_.set_fault(Fault::kSafetySignalLoss, true)) {
            response->success = false;
            response->message = "communication_loss is already active";
            return;
          }
          safety_signal_loss_timer_ = create_wall_timer(
            std::chrono::seconds(5), [this]() {
              safety_signal_loss_timer_->cancel();
              safety_signal_loss_timer_.reset();
              mock_.set_fault(Fault::kSafetySignalLoss, false);
              publish_safety_hardware();
            });
          response->success = true;
          response->message = "Safety-HW publication suppressed for five seconds";
          return;
        }
        const bool was_active = static_cast<bool>(safety_signal_loss_timer_);
        if (safety_signal_loss_timer_) {
          safety_signal_loss_timer_->cancel();
          safety_signal_loss_timer_.reset();
        }
        mock_.set_fault(Fault::kSafetySignalLoss, false);
        response->success = true;
        response->message = was_active ?
          "Safety-HW suppression canceled" : "no Safety-HW suppression was active";
        if (was_active) {
          publish_safety_hardware();
        }
      });
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
    if (states.publish_safety_hardware) {
      hardware_publisher_->publish(states.hardware);
    }
    last_external_publish_ = steady_now;
  }

  void publish_safety_hardware()
  {
    const auto states = mock_.snapshot(now());
    if (states.publish && states.publish_safety_hardware) {
      hardware_publisher_->publish(states.hardware);
    }
  }

  void publish_material_handoff()
  {
    const auto states = material_delivery_.snapshot_all(
      MaterialDeliveryMock::Clock::now(),
      static_cast<builtin_interfaces::msg::Time>(now()));
    for (const auto & state : states) {
      const auto phase = last_material_phases_.find(state.delivery_id.uuid);
      if (phase == last_material_phases_.end() || phase->second != state.phase) {
        const auto delivery_id = format_delivery_id(state.delivery_id);
        if (state.phase == arm_cell_interfaces::msg::MaterialHandoffState::HANDOFF_FAILED) {
          RCLCPP_ERROR(get_logger(),
                       "VDA mock handoff phase failed phase=FAILED");
        } else if (!state.valid) {
          RCLCPP_WARN(get_logger(), "VDA mock handoff phase invalid phase=%s",
                      phase_name(state.phase));
        } else {
          RCLCPP_INFO(get_logger(), "VDA mock handoff phase changed phase=%s",
                      phase_name(state.phase));
        }
        last_material_phases_[state.delivery_id.uuid] = state.phase;
      }
      material_handoff_publisher_->publish(state);
    }
  }

  static std::string format_delivery_id(
    const unique_identifier_msgs::msg::UUID & delivery_id)
  {
    static constexpr char digits[] = "0123456789abcdef";
    std::string text;
    text.reserve(32);
    for (const auto byte : delivery_id.uuid) {
      text.push_back(digits[(byte >> 4U) & 0x0fU]);
      text.push_back(digits[byte & 0x0fU]);
    }
    return text;
  }

  static const char * phase_name(uint8_t phase)
  {
    using Handoff = arm_cell_interfaces::msg::MaterialHandoffState;
    switch (phase) {
      case Handoff::HANDOFF_UNKNOWN:
        return "HANDOFF_UNKNOWN";
      case Handoff::HANDOFF_ARRIVED:
        return "ARRIVED";
      case Handoff::HANDOFF_DOCKED:
        return "DOCKED";
      case Handoff::HANDOFF_UNLOADED:
        return "UNLOADED";
      case Handoff::HANDOFF_DEPARTED:
        return "DEPARTED";
      case Handoff::HANDOFF_FAILED:
        return "HANDOFF_FAILED";
      default:
        return "UNKNOWN";
    }
  }

  ExternalStateMock mock_;
  MaterialDeliveryMock material_delivery_;
  rclcpp::Publisher<AMRDockingState>::SharedPtr amr_publisher_;
  rclcpp::Publisher<PackMLState>::SharedPtr packml_publisher_;
  rclcpp::Publisher<SafetyHardwareState>::SharedPtr hardware_publisher_;
  rclcpp::Publisher<arm_cell_interfaces::msg::MaterialHandoffState>::SharedPtr
    material_handoff_publisher_;
  rclcpp::Subscription<arm_cell_interfaces::msg::HandoffTrigger>::SharedPtr
    accepted_handoff_subscription_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> fault_services_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr safety_signal_loss_service_;
  rclcpp::TimerBase::SharedPtr safety_signal_loss_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds degraded_publish_period_{0};
  std::chrono::steady_clock::time_point last_external_publish_{};
  std::map<std::array<uint8_t, 16>, uint8_t> last_material_phases_;
};

}  // namespace arm_cell_vda_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_cell_vda_adapter::ExternalStateMockNode>());
  rclcpp::shutdown();
  return 0;
}
