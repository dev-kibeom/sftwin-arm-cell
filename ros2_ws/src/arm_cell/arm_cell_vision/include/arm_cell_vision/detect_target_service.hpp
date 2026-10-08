#ifndef ARM_CELL_VISION__DETECT_TARGET_SERVICE_HPP_
#define ARM_CELL_VISION__DETECT_TARGET_SERVICE_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <arm_cell_interfaces/srv/detect_target.hpp>
#include <arm_cell_interfaces/msg/vision_diagnostic.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "arm_cell_vision/detect_target_processor.hpp"
#include "arm_cell_vision/vision_ros_ingress.hpp"

namespace arm_cell_vision
{

class DetectTargetService
{
public:
  using Request = arm_cell_interfaces::srv::DetectTarget::Request;
  using Response = arm_cell_interfaces::srv::DetectTarget::Response;
  using SteadyNow = std::function<int64_t()>;
  using TransformLookup = std::function<std::optional<geometry_msgs::msg::TransformStamped>(
        const std::string &, int64_t)>;

  DetectTargetService(
    VisionRosIngress & ingress,
    DetectTargetProcessor processor,
    SteadyNow steady_now,
    TransformLookup transform_lookup,
    std::vector<DetectorProfile> detector_profiles = {});

  rclcpp::Service<arm_cell_interfaces::srv::DetectTarget>::SharedPtr advertise(
    rclcpp::Node & node);

  void handle(const Request & request, Response & response);

private:
  VisionRosIngress & ingress_;
  DetectTargetProcessor processor_;
  SteadyNow steady_now_;
  TransformLookup transform_lookup_;
  std::vector<DetectorProfile> detector_profiles_;
  rclcpp::Logger logger_{rclcpp::get_logger("detect_target_service")};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<arm_cell_interfaces::srv::DetectTarget>::SharedPtr service_server_;
  rclcpp::Publisher<arm_cell_interfaces::msg::VisionDiagnostic>::SharedPtr diagnostic_publisher_;
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DETECT_TARGET_SERVICE_HPP_
