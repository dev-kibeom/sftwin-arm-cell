#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <arm_cell_interfaces/msg/detect_target_result_code.hpp>
#include <arm_cell_interfaces/srv/detect_target.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arm_cell_vision/fixed_target_registry.hpp"
#include "arm_cell_vision/srv/register_fixed_target.hpp"

namespace arm_cell_vision
{
namespace
{
int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class FixedDetectTargetNode : public rclcpp::Node
{
public:
  FixedDetectTargetNode()
  : Node("fixed_detect_target_node"),
    configured_target_id_(declare_parameter<std::string>("target_id", "")),
    registry_(
      configured_target_id_,
      [this]() {
        return steady_now_ns();
      })
  {
    detect_service_ = create_service<arm_cell_interfaces::srv::DetectTarget>(
      "/vision/detect_target",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<arm_cell_interfaces::srv::DetectTarget::Request> request,
        std::shared_ptr<arm_cell_interfaces::srv::DetectTarget::Response> response) {
        handle_detect(*request, *response);
      });
    registration_service_ = create_service<arm_cell_vision::srv::RegisterFixedTarget>(
      "~/register_target",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<arm_cell_vision::srv::RegisterFixedTarget::Request> request,
        std::shared_ptr<arm_cell_vision::srv::RegisterFixedTarget::Response> response) {
        handle_registration(*request, *response);
      });
  }

private:
  static int64_t duration_ns(const builtin_interfaces::msg::Duration & duration)
  {
    if (duration.sec < 0 || duration.nanosec >= 1'000'000'000U) {
      return -1;
    }
    const auto seconds = static_cast<int64_t>(duration.sec);
    if (seconds > std::numeric_limits<int64_t>::max() / 1'000'000'000LL) {
      return -1;
    }
    return seconds * 1'000'000'000LL + duration.nanosec;
  }

  void handle_registration(
    const srv::RegisterFixedTarget::Request & request,
    srv::RegisterFixedTarget::Response & response)
  {
    FixedTargetRegistration registration;
    registration.target_id = request.target_id;
    registration.run_id = request.run_id;
    registration.pose = request.pose;
    registration.has_target_yaw = request.has_target_yaw;
    registration.ttl_ns = duration_ns(request.ttl);
    if (registration.ttl_ns == 0) {
      registration.ttl_ns = 30'000'000'000LL;
    }

    const auto result = registry_.register_target(registration);
    response.accepted = result.accepted;
    response.receipt_sequence = result.receipt_sequence;
    response.receipt_time_ns = result.receipt_time_ns;
    response.diagnostic = result.diagnostic;
  }

  void handle_detect(
    const arm_cell_interfaces::srv::DetectTarget::Request & request,
    arm_cell_interfaces::srv::DetectTarget::Response & response)
  {
    const auto result = registry_.consume(request.target_id, steady_now_ns());
    if (!result) {
      response.result_code.value =
        request.target_id == registry_target_id() ?
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND :
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT;
      response.diagnostic_detail = "no valid fixed target registration";
      return;
    }

    response.result_code.value =
      arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    response.target_pose = result->pose;
    response.has_target_pose = true;
    response.has_target_yaw = result->has_target_yaw;
    response.diagnostic_detail = "fixed validation registration";
  }

  const std::string & registry_target_id() const
  {
    return configured_target_id_;
  }

  std::string configured_target_id_;
  FixedTargetRegistry registry_;
  rclcpp::Service<arm_cell_interfaces::srv::DetectTarget>::SharedPtr detect_service_;
  rclcpp::Service<srv::RegisterFixedTarget>::SharedPtr registration_service_;
};

}  // namespace arm_cell_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_cell_vision::FixedDetectTargetNode>());
  rclcpp::shutdown();
  return 0;
}
