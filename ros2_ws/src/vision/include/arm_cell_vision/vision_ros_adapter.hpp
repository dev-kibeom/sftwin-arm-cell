#ifndef ARM_CELL_VISION__VISION_ROS_ADAPTER_HPP_
#define ARM_CELL_VISION__VISION_ROS_ADAPTER_HPP_

#include <cstdint>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "arm_cell_vision/vision_sensor_ingress.hpp"

namespace arm_cell_vision
{

struct ReceiptTimes
{
  int64_t rgb_monotonic_ns;
  int64_t depth_monotonic_ns;
  int64_t camera_info_monotonic_ns;
};

class VisionRosAdapter
{
public:
  static SynchronizedObservationCandidate to_candidate(
    const sensor_msgs::msg::Image & rgb,
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & camera_info,
    ReceiptTimes receipts);
  static bool has_valid_calibration(const sensor_msgs::msg::CameraInfo & camera_info);
  static bool within_sync_slop(
    const SynchronizedObservationCandidate & candidate,
    int64_t sync_slop_ns);
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__VISION_ROS_ADAPTER_HPP_
