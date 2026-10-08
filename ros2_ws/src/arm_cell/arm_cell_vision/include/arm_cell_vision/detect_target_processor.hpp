#ifndef ARM_CELL_VISION__DETECT_TARGET_PROCESSOR_HPP_
#define ARM_CELL_VISION__DETECT_TARGET_PROCESSOR_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <arm_cell_interfaces/msg/detect_target_result_code.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace arm_cell_vision
{

struct DetectionInput
{
  sensor_msgs::msg::Image rgb;
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera_info;
  geometry_msgs::msg::TransformStamped optical_to_base;
};

struct DetectionResult
{
  uint8_t result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT;
  geometry_msgs::msg::PoseStamped target_pose;
  bool has_target_pose = false;
  bool has_target_yaw = false;
  float estimated_height_m = 0.0F;
  bool has_estimated_height = false;
  std::string diagnostic_detail;
  std_msgs::msg::Header source_rgb_header;
  bool centroid_pixel_valid = false;
  float centroid_pixel_x = 0.0F;
  float centroid_pixel_y = 0.0F;
  bool object_region_valid = false;
  uint32_t object_region_x = 0;
  uint32_t object_region_y = 0;
  uint32_t object_region_width = 0;
  uint32_t object_region_height = 0;
  bool support_region_valid = false;
  std::vector<geometry_msgs::msg::Point32> support_region;
};

struct DetectorProfile
{
  std::string target_id;
  std::string shape;
  std::array<double, 3> dimensions_m{};
  std::array<double, 3> dimension_tolerance_m{};
  std::array<double, 3> workspace_roi_min_m{};
  std::array<double, 3> workspace_roi_max_m{};
  double support_contact_tolerance_m = 0.0;
  double top_surface_geometry_tolerance_m = 0.0;
  std::string fiducial_dictionary;
  int fiducial_marker_id = -1;
  double fiducial_size_m = 0.0;
  double fiducial_pose_translation_scale = 1.0;
  std::array<double, 3> marker_in_support_m{};
  std::array<double, 3> marker_rpy_in_support_rad{};
};

class DetectTargetProcessor
{
public:
  DetectionResult process(const DetectionInput & input) const;
  DetectionResult process(
    const DetectionInput & input, const std::string & target_id,
    const std::vector<DetectorProfile> & profiles) const;
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DETECT_TARGET_PROCESSOR_HPP_
