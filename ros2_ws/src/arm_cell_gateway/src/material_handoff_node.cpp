#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/handoff_trigger.hpp>
#include <arm_cell_interfaces/msg/handoff_trigger_status.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/transfer_completion.hpp>

#include "material_handoff_coordinator.hpp"
#include "material_adapter_support.hpp"

namespace arm_cell_gateway
{

class MaterialHandoffNode final : public rclcpp::Node
{
public:
  using Clock = MaterialHandoffCoordinator::Clock;
  using HandoffState = arm_cell_interfaces::msg::MaterialHandoffState;
  using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
  using Trigger = arm_cell_interfaces::msg::HandoffTrigger;
  using TriggerStatus = arm_cell_interfaces::msg::HandoffTriggerStatus;
  using Completion = arm_cell_interfaces::msg::TransferCompletion;
  using Coordinator = MaterialHandoffCoordinator;

  MaterialHandoffNode()
  : Node("material_handoff_node"),
    source_freshness_timeout_(std::chrono::milliseconds(
        declare_parameter<int64_t>("source_freshness_timeout_ms", 500))),
    delivery_deadline_(std::chrono::milliseconds(
        declare_parameter<int64_t>("delivery_deadline_ms", 120000))),
    coordinator_(delivery_deadline_)
  {
    if (source_freshness_timeout_.count() <= 0) {
      throw std::invalid_argument("material source freshness bound must be positive");
    }
    readiness_publisher_ = create_publisher<Readiness>(
      "/integration/material_readiness",
      rclcpp::QoS(64).reliable().transient_local());
    accepted_publisher_ = create_publisher<Trigger>(
      "/integration/accepted_handoff", rclcpp::QoS(10).reliable());
    trigger_status_publisher_ = create_publisher<TriggerStatus>(
      "/integration/handoff_trigger_status", rclcpp::QoS(10).reliable());
    handoff_subscription_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    trigger_subscription_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions handoff_options;
    handoff_options.callback_group = handoff_subscription_callback_group_;
    handoff_subscription_ = create_subscription<HandoffState>(
      "/vda/material_handoff_state", rclcpp::QoS(10),
      [this](const HandoffState::SharedPtr state) {
        auto observed = *state;
        if (!material_adapter_support::source_is_fresh(
          *this, state->header, now(), source_freshness_timeout_))
        {
          observed.valid = false;
        }
        std::optional<Readiness> output;
        {
          std::lock_guard<std::mutex> lock(coordinator_mutex_);
          output = coordinator_.observe_vda(
            observed, Clock::now(), static_cast<builtin_interfaces::msg::Time>(now()));
        }
        if (output) {
          material_adapter_support::publish_readiness(
            *this, readiness_publisher_, *output,
            "VDA handoff evidence could not confirm delivery");
        }
      }, handoff_options);

    rclcpp::SubscriptionOptions trigger_options;
    trigger_options.callback_group = trigger_subscription_callback_group_;
    trigger_subscription_ = create_subscription<Trigger>(
      "/external/handoff_trigger", rclcpp::QoS(10).reliable(),
      [this](const Trigger::SharedPtr trigger) {accept_trigger(*trigger);}, trigger_options);
    completion_subscription_ = create_subscription<Completion>(
      "/material_supply/transfer_completion", rclcpp::QoS(10).reliable(),
      [this](const Completion::SharedPtr completion) {
        auto observed = *completion;
        const bool fresh = material_adapter_support::source_is_fresh(
          *this, completion->header, now(), source_freshness_timeout_);
        if (!fresh) {
          observed.valid = false;
        }
        Coordinator::CompletionObservation observation;
        {
          std::lock_guard<std::mutex> lock(coordinator_mutex_);
          observation = coordinator_.observe_completion(
            observed, Clock::now(), static_cast<builtin_interfaces::msg::Time>(now()));
        }
        log_completion_observation(*completion, observation, fresh);
        if (observation.readiness) {
          material_adapter_support::publish_readiness(
            *this, readiness_publisher_, *observation.readiness,
            "transfer completion validation failed; see correlated completion result");
        }
      }, trigger_options);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        std::optional<Readiness> output;
        {
          std::lock_guard<std::mutex> lock(coordinator_mutex_);
          output = coordinator_.tick(
            Clock::now(), static_cast<builtin_interfaces::msg::Time>(now()));
        }
        if (output) {
          material_adapter_support::publish_readiness(
            *this, readiness_publisher_, *output,
            "delivery deadline expired before complete handoff evidence");
          RCLCPP_ERROR(
            get_logger(),
            "material handoff delivery failed reason=delivery_deadline_expired");
        }
      });
    RCLCPP_INFO(get_logger(), "material handoff gateway ready");
  }

private:
  void accept_trigger(const Trigger & trigger)
  {
    const auto ros_now = now();
    if (!material_adapter_support::usable_id(trigger.delivery_id)) {
      RCLCPP_DEBUG(get_logger(), "handoff trigger rejected reason=invalid_delivery_id");
      return;
    }
    TriggerStatus status;
    status.header.stamp = static_cast<builtin_interfaces::msg::Time>(ros_now);
    status.delivery_id = trigger.delivery_id;
    status.state = TriggerStatus::TRIGGER_REJECTED;
    std::string rejection_reason;
    if (!material_adapter_support::source_is_fresh(
        *this, trigger.header, ros_now, source_freshness_timeout_))
    {
      rejection_reason = "stale_source_timestamp";
      status.rejection_reason = TriggerStatus::REJECTION_STALE_SOURCE_TIMESTAMP;
    } else {
      std::lock_guard<std::mutex> lock(coordinator_mutex_);
      rejection_reason = coordinator_.rejection_reason(trigger);
      if (rejection_reason.empty() && !coordinator_.begin(trigger, Clock::now())) {
        rejection_reason = "delivery_not_accepted";
      }
      if (!rejection_reason.empty()) {
        status.rejection_reason = rejection_code(rejection_reason);
      }
    }
    if (!rejection_reason.empty()) {
      status.diagnostic_detail = rejection_reason;
      trigger_status_publisher_->publish(status);
      RCLCPP_DEBUG(get_logger(), "handoff trigger rejected reason=%s", rejection_reason.c_str());
      return;
    }

    status.state = TriggerStatus::TRIGGER_ACCEPTED;
    status.rejection_reason = TriggerStatus::REJECTION_NONE;
    trigger_status_publisher_->publish(status);
    Readiness pending;
    pending.header.stamp = static_cast<builtin_interfaces::msg::Time>(ros_now);
    pending.delivery_id = trigger.delivery_id;
    pending.material_type = trigger.material_type;
    pending.quantity = trigger.quantity;
    pending.material_ready = false;
    pending.valid = true;
    material_adapter_support::publish_readiness(*this, readiness_publisher_, pending);
    accepted_publisher_->publish(trigger);
    RCLCPP_INFO(
      get_logger(), "material handoff accepted type=%s quantity=%u",
      trigger.material_type.c_str(), trigger.quantity);
  }

  void log_completion_observation(
    const Completion & completion, const Coordinator::CompletionObservation & observation,
    bool fresh_source)
  {
    using Disposition = Coordinator::CompletionDisposition;
    if (observation.disposition == Disposition::kNoActiveDelivery) {
      return;
    }
    const auto key =
      std::make_pair(completion.delivery_id.uuid, static_cast<int>(observation.disposition));
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      if (!logged_observations_.insert(key).second) {
        return;
      }
      logged_order_.push_back(key);
      if (logged_order_.size() > 128) {
        logged_observations_.erase(logged_order_.front());
        logged_order_.pop_front();
      }
    }
    switch (observation.disposition) {
      case Disposition::kCompleted:
        RCLCPP_INFO(
          get_logger(), "material handoff supply completed requested=%u supplied=%u",
          completion.requested_quantity, completion.supplied_quantity);
        break;
      case Disposition::kDeliveryMismatch:
        RCLCPP_DEBUG(get_logger(), "transfer completion ignored reason=delivery_mismatch");
        break;
      case Disposition::kPreviousTerminalFailure:
        RCLCPP_DEBUG(get_logger(), "transfer completion ignored reason=delivery_already_finalized");
        break;
      case Disposition::kDeadlineExpired:
        RCLCPP_ERROR(
          get_logger(), "material handoff delivery failed reason=delivery_deadline_expired");
        break;
      case Disposition::kMetadataMismatch:
        RCLCPP_ERROR(
          get_logger(),
          "material handoff delivery failed reason=material_or_requested_quantity_mismatch");
        break;
      case Disposition::kInvalidObservation:
        RCLCPP_ERROR(
          get_logger(), "material handoff delivery failed reason=%s",
          fresh_source ? "completion_invalid" : "stale_source_timestamp");
        break;
      case Disposition::kTransferFailure:
        RCLCPP_ERROR(
          get_logger(), "material handoff delivery failed reason=supply_failure requested=%u supplied=%u",
          completion.requested_quantity, completion.supplied_quantity);
        break;
      case Disposition::kQuantityMismatch:
        RCLCPP_ERROR(
          get_logger(), "material handoff delivery failed reason=quantity_mismatch requested=%u supplied=%u",
          completion.requested_quantity, completion.supplied_quantity);
        break;
      case Disposition::kNoActiveDelivery:
        break;
    }
  }

  static uint8_t rejection_code(const std::string & reason)
  {
    if (reason == "invalid_trigger") {return TriggerStatus::REJECTION_INVALID_TRIGGER;}
    if (reason == "invalid_quantity") {return TriggerStatus::REJECTION_INVALID_QUANTITY;}
    if (reason == "unsupported_material_type") {
      return TriggerStatus::REJECTION_UNSUPPORTED_MATERIAL_TYPE;
    }
    if (reason == "duplicate_delivery_id") {return TriggerStatus::REJECTION_DUPLICATE_DELIVERY_ID;}
    if (reason == "invalid_delivery_id") {return TriggerStatus::REJECTION_INVALID_DELIVERY_ID;}
    return TriggerStatus::REJECTION_INVALID_TRIGGER;
  }

  std::chrono::milliseconds source_freshness_timeout_;
  std::chrono::milliseconds delivery_deadline_;
  Coordinator coordinator_;
  using CompletionLogKey = std::pair<std::array<uint8_t, 16>, int>;
  std::set<CompletionLogKey> logged_observations_;
  std::deque<CompletionLogKey> logged_order_;
  std::mutex coordinator_mutex_;
  std::mutex log_mutex_;
  rclcpp::CallbackGroup::SharedPtr handoff_subscription_callback_group_;
  rclcpp::CallbackGroup::SharedPtr trigger_subscription_callback_group_;
  rclcpp::Publisher<Readiness>::SharedPtr readiness_publisher_;
  rclcpp::Publisher<Trigger>::SharedPtr accepted_publisher_;
  rclcpp::Publisher<TriggerStatus>::SharedPtr trigger_status_publisher_;
  rclcpp::Subscription<HandoffState>::SharedPtr handoff_subscription_;
  rclcpp::Subscription<Trigger>::SharedPtr trigger_subscription_;
  rclcpp::Subscription<Completion>::SharedPtr completion_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arm_cell_gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<arm_cell_gateway::MaterialHandoffNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
