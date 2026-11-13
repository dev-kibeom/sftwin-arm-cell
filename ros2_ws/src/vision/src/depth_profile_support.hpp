// Internal helpers for the depth profile pipeline; not part of the ROS interface.
#ifndef ARM_CELL_VISION__DEPTH_PROFILE_SUPPORT_HPP_
#define ARM_CELL_VISION__DEPTH_PROFILE_SUPPORT_HPP_

#include "arm_cell_vision/detect_target_processor.hpp"

#include <cmath>
#include <cstddef>
#include <opencv2/aruco.hpp>

namespace arm_cell_vision
{
namespace depth_profile
{
inline bool finite(double value) {return std::isfinite(value);}
DetectionResult failure(uint8_t code, const char * detail, const char * processing_stage);
bool valid_profile(const DetectorProfile & profile, bool calibration_requested);
bool valid_camera(const sensor_msgs::msg::CameraInfo & info);
bool valid_image_storage(const sensor_msgs::msg::Image & image, size_t bytes_per_pixel);
bool finite_transform(const geometry_msgs::msg::TransformStamped & transform);
bool read_depth(const sensor_msgs::msg::Image & image, size_t row, size_t column, float & depth);
tf2::Transform transform_from_ros(const geometry_msgs::msg::Transform & transform);
cv::Ptr<cv::aruco::Dictionary> resolve_fiducial_dictionary(const std::string & name);
}  // namespace depth_profile
}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DEPTH_PROFILE_SUPPORT_HPP_
