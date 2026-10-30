#ifndef ARM_CELL_VISION__DETECT_TARGET_SERVICE_HPP_
#define ARM_CELL_VISION__DETECT_TARGET_SERVICE_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <mutex>
#include <string>
#include <vector>

#include <arm_cell_interfaces/srv/detect_target.hpp>
#include <arm_cell_interfaces/msg/vision_diagnostic.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "arm_cell_vision/detect_target_processor.hpp"
#include "arm_cell_vision/detect_target_acquisition.hpp"
#include "arm_cell_vision/vision_ros_ingress.hpp"

namespace arm_cell_vision
{

std::string make_acquisition_id(const std::string & vision_run_id, uint64_t generation);

struct AcquisitionDiagnosticSummary
{
  std::string detail;
  std::string failure_cause;
};

class DetectTargetService
{
public:
  using Request = arm_cell_interfaces::srv::DetectTarget::Request;
  using Response = arm_cell_interfaces::srv::DetectTarget::Response;
  using SteadyNow = std::function<int64_t()>;
  using AcquisitionBegin = std::function<uint8_t(
        const std::string &, uint64_t, int64_t, int64_t, const std::string &)>;
  using AcquisitionEnd = std::function<void()>;
  using AcquisitionResultLookup = std::function<SharedAcquisitionResult(
        const std::string &, uint64_t)>;
  using AcquisitionDiagnosticLookup = std::function<AcquisitionDiagnosticSummary()>;
  using AcquisitionTerminal = std::function<void(
      const std::string &, uint64_t, const Response &, const SharedAcquisitionResult &,
      const VisionRequestDiagnosticRecord &)>;
  using SupportNormalLookup = std::function<std::optional<geometry_msgs::msg::Vector3>(
        const std::string &)>;

  DetectTargetService(
    VisionRosIngress & ingress,
    SteadyNow steady_now,
    AcquisitionBegin acquisition_begin = {},
    AcquisitionEnd acquisition_end = {},
    AcquisitionResultLookup acquisition_result_lookup = {},
    AcquisitionDiagnosticLookup acquisition_diagnostic_lookup = {},
    AcquisitionTerminal acquisition_terminal = {},
    std::string vision_run_id = {},
    SupportNormalLookup support_normal_lookup = {});

  rclcpp::Service<arm_cell_interfaces::srv::DetectTarget>::SharedPtr advertise(
    rclcpp::Node & node);

  void handle(const Request & request, Response & response);
  void set_mislocalize_next(bool active);
  [[nodiscard]] bool mislocalize_pending() const;

private:
  VisionRosIngress & ingress_;
  SteadyNow steady_now_;
  AcquisitionBegin acquisition_begin_;
  AcquisitionEnd acquisition_end_;
  AcquisitionResultLookup acquisition_result_lookup_;
  AcquisitionDiagnosticLookup acquisition_diagnostic_lookup_;
  AcquisitionTerminal acquisition_terminal_;
  SupportNormalLookup support_normal_lookup_;
  std::string vision_run_id_;
  std::mutex acquisition_mutex_;
  mutable std::mutex scenario_mutex_;
  bool mislocalize_armed_{false};
  uint64_t mislocalize_after_generation_{0};
  rclcpp::Logger logger_{rclcpp::get_logger("detect_target_service")};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<arm_cell_interfaces::srv::DetectTarget>::SharedPtr service_server_;
  rclcpp::Publisher<arm_cell_interfaces::msg::VisionDiagnostic>::SharedPtr diagnostic_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mislocalize_service_;
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DETECT_TARGET_SERVICE_HPP_
