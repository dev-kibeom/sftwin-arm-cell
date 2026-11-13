#include "depth_profile_support.hpp"

#include <algorithm>
#include <array>

namespace arm_cell_vision
{
namespace depth_profile
{

cv::Ptr<cv::aruco::Dictionary> resolve_fiducial_dictionary(const std::string & name)
{
  using DictionaryName = cv::aruco::PREDEFINED_DICTIONARY_NAME;
  static const std::array<std::pair<const char *, DictionaryName>, 21> dictionaries = {{
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
    {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
    {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
    {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
  }};
  const auto found = std::find_if(
    dictionaries.begin(), dictionaries.end(), [&name](const auto & entry) {
      return name == entry.first;
    });
  if (found == dictionaries.end()) {
    return {};
  }
  return cv::aruco::getPredefinedDictionary(found->second);
}

template<size_t Size>
bool finite_values(const std::array<double, Size> & values)
{
  return std::all_of(values.begin(), values.end(), finite);
}

bool finite_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  return finite(translation.x) && finite(translation.y) && finite(translation.z) &&
         finite(rotation.x) && finite(rotation.y) && finite(rotation.z) &&
         finite(rotation.w) &&
         (rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
         rotation.w * rotation.w > 1e-12);
}

bool valid_profile(const DetectorProfile & profile, bool calibration_requested)
{
  const auto dictionary = resolve_fiducial_dictionary(profile.fiducial_dictionary);
  if (profile.target_id.empty() || profile.material_id != profile.target_id ||
    profile.shape != "box" ||
    (calibration_requested && (dictionary.empty() || profile.fiducial_marker_id < 0 ||
    profile.fiducial_marker_id >= dictionary->bytesList.rows ||
    !finite(profile.fiducial_size_m) ||
    profile.fiducial_size_m <= 0.0 ||
    !finite(profile.fiducial_pose_translation_scale) ||
    profile.fiducial_pose_translation_scale < 0.8 ||
    profile.fiducial_pose_translation_scale > 1.2)) ||
    !finite(profile.fiducial_size_m) || profile.fiducial_size_m <= 0.0 ||
    !finite(profile.support_contact_tolerance_m) ||
    profile.support_contact_tolerance_m <= 0.0 ||
    !finite(profile.top_surface_geometry_tolerance_m) ||
    profile.top_surface_geometry_tolerance_m <= 0.0 ||
    !finite(profile.high_confidence_observed_area_ratio) ||
    !finite(profile.medium_confidence_observed_area_ratio) ||
    profile.medium_confidence_observed_area_ratio <= 0.0 ||
    profile.high_confidence_observed_area_ratio > 1.0 ||
    profile.medium_confidence_observed_area_ratio > profile.high_confidence_observed_area_ratio ||
    !finite(profile.minimum_observation_coverage) ||
    profile.minimum_observation_coverage <= 0.0 ||
    profile.minimum_observation_coverage > 1.0 ||
    !finite_values(profile.dimensions_m) ||
    !finite_values(profile.dimension_tolerance_m) ||
    !finite_values(profile.workspace_roi_min_m) ||
    !finite_values(profile.workspace_roi_max_m) ||
    !finite_values(profile.tray_roi_min_m) ||
    !finite_values(profile.tray_roi_max_m) ||
    !finite_values(profile.marker_in_support_m) ||
    !finite_values(profile.marker_rpy_in_support_rad))
  {
    return false;
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    if (profile.dimensions_m[axis] <= 0.0 || profile.dimension_tolerance_m[axis] < 0.0 ||
      profile.workspace_roi_min_m[axis] >= profile.workspace_roi_max_m[axis])
    {
      return false;
    }
  }
  const bool tray_roi_configured = profile.tray_roi_min_m[0] != 0.0 ||
    profile.tray_roi_min_m[1] != 0.0 || profile.tray_roi_max_m[0] != 0.0 ||
    profile.tray_roi_max_m[1] != 0.0;
  if (tray_roi_configured &&
    (profile.tray_roi_min_m[0] >= profile.tray_roi_max_m[0] ||
    profile.tray_roi_min_m[1] >= profile.tray_roi_max_m[1]))
  {
    return false;
  }
  return true;
}

bool valid_camera(const sensor_msgs::msg::CameraInfo & info)
{
  return info.width > 0 && info.height > 0 && !info.header.frame_id.empty() &&
         finite(info.k[0]) && finite(info.k[2]) && finite(info.k[4]) &&
         finite(info.k[5]) && info.k[0] > 0.0 && info.k[4] > 0.0 &&
         std::all_of(info.d.begin(), info.d.end(), finite);
}

bool valid_image_storage(const sensor_msgs::msg::Image & image, size_t bytes_per_pixel)
{
  return image.width > 0 && image.height > 0 &&
         image.step >= image.width * bytes_per_pixel &&
         image.data.size() >= static_cast<size_t>(image.step) * image.height;
}


}  // namespace depth_profile

}  // namespace arm_cell_vision
