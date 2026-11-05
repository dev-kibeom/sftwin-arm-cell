#include "arm_cell_vision/vision_ros_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace arm_cell_vision
{
namespace
{

int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

SensorSample image_sample(const sensor_msgs::msg::Image & image, int64_t receipt)
{
  return {
    stamp_nanoseconds(image.header.stamp), receipt, static_cast<int32_t>(image.width),
    static_cast<int32_t>(image.height), image.header.frame_id};
}

SensorSample info_sample(const sensor_msgs::msg::CameraInfo & info, int64_t receipt)
{
  return {
    stamp_nanoseconds(info.header.stamp), receipt, static_cast<int32_t>(info.width),
    static_cast<int32_t>(info.height), info.header.frame_id};
}

template<size_t Size>
bool finite_values(const std::array<double, Size> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

}  // namespace

SynchronizedObservationCandidate VisionRosAdapter::to_candidate(
  const sensor_msgs::msg::Image & rgb,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  ReceiptTimes receipts)
{
  return {
    image_sample(rgb, receipts.rgb_monotonic_ns),
    image_sample(depth, receipts.depth_monotonic_ns),
    info_sample(camera_info, receipts.camera_info_monotonic_ns),
    stamp_nanoseconds(rgb.header.stamp)};
}

bool VisionRosAdapter::has_valid_calibration(const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (camera_info.header.frame_id.empty() || camera_info.width == 0 || camera_info.height == 0 ||
    camera_info.distortion_model.empty() || !finite_values(camera_info.k) ||
    !finite_values(camera_info.p) || camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 ||
    camera_info.p[0] <= 0.0 || camera_info.p[5] <= 0.0)
  {
    return false;
  }
  return std::all_of(
    camera_info.d.begin(), camera_info.d.end(),
    [](double value) {return std::isfinite(value);});
}

bool VisionRosAdapter::within_sync_slop(
  const SynchronizedObservationCandidate & candidate,
  int64_t sync_slop_ns)
{
  if (sync_slop_ns < 0 || !candidate.rgb.stamp_ns || !candidate.depth.stamp_ns) {
    return false;
  }
  const auto minimum = std::min(*candidate.rgb.stamp_ns, *candidate.depth.stamp_ns);
  const auto maximum = std::max(*candidate.rgb.stamp_ns, *candidate.depth.stamp_ns);
  return maximum - minimum <= sync_slop_ns;
}

}  // namespace arm_cell_vision
