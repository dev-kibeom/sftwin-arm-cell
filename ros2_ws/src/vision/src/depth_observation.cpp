#include "depth_profile_support.hpp"

#include <array>
#include <cstring>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_vision
{
namespace depth_profile
{

DetectionResult failure(uint8_t code, const char * detail, const char * processing_stage)
{
  DetectionResult output;
  output.result_code = code;
  output.failure_cause = detail;
  output.diagnostic_detail = detail;
  output.processing_stage = processing_stage;
  return output;
}

bool read_depth(const sensor_msgs::msg::Image & image, size_t row, size_t column, float & depth)
{
  const auto bytes_per_pixel = image.encoding == "32FC1" ? sizeof(float) : sizeof(uint16_t);
  const auto offset = row * image.step + column * bytes_per_pixel;
  if (image.encoding == "32FC1") {
    std::memcpy(&depth, image.data.data() + offset, sizeof(float));
    return true;
  }
  if (image.encoding == "16UC1") {
    uint16_t millimeters = 0;
    std::memcpy(&millimeters, image.data.data() + offset, sizeof(uint16_t));
    depth = static_cast<float>(millimeters) / 1000.0F;
    return true;
  }
  return false;
}

tf2::Transform transform_from_ros(const geometry_msgs::msg::Transform & transform)
{
  tf2::Quaternion rotation(
    transform.rotation.x, transform.rotation.y, transform.rotation.z,
    transform.rotation.w);
  rotation.normalize();
  return tf2::Transform(
    rotation,
    tf2::Vector3(
      transform.translation.x, transform.translation.y, transform.translation.z));
}


}  // namespace depth_profile

using depth_profile::finite;
using depth_profile::read_depth;

std::optional<TrayDepthCoverage> tray_depth_coverage(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera_info,
  const DetectorProfile & profile, const tf2::Transform & support_T_camera)
{
  const auto camera_T_support = support_T_camera.inverse();
  std::vector<cv::Point3f> workspace_corners;
  const bool tray_roi_configured = profile.tray_roi_min_m[0] < profile.tray_roi_max_m[0] &&
    profile.tray_roi_min_m[1] < profile.tray_roi_max_m[1];
  const auto min_x = tray_roi_configured ? profile.tray_roi_min_m[0] :
    profile.workspace_roi_min_m[0];
  const auto min_y = tray_roi_configured ? profile.tray_roi_min_m[1] :
    profile.workspace_roi_min_m[1];
  const auto max_x = tray_roi_configured ? profile.tray_roi_max_m[0] :
    profile.workspace_roi_max_m[0];
  const auto max_y = tray_roi_configured ? profile.tray_roi_max_m[1] :
    profile.workspace_roi_max_m[1];
  for (const auto & xy : std::array<std::array<double, 2>, 4>{{
      {{min_x, min_y}}, {{max_x, min_y}}, {{max_x, max_y}}, {{min_x, max_y}}}})
  {
    const auto camera = camera_T_support * tf2::Vector3(xy[0], xy[1], 0.0);
    if (!finite(camera.x()) || !finite(camera.y()) || !finite(camera.z()) || camera.z() <= 0.0) {
      return std::nullopt;
    }
    workspace_corners.emplace_back(camera.x(), camera.y(), camera.z());
  }
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], 0.0, camera_info.k[2],
    0.0, camera_info.k[4], camera_info.k[5], 0.0, 0.0, 1.0);
  const cv::Mat distortion = camera_info.d.empty() ?
    cv::Mat::zeros(1, 5, CV_64F) : cv::Mat(camera_info.d, true);
  std::vector<cv::Point2f> projected;
  try {
    cv::projectPoints(
      workspace_corners, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
      camera_matrix, distortion, projected);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
  if (projected.size() != 4) {
    return std::nullopt;
  }
  std::vector<cv::Point> polygon;
  polygon.reserve(projected.size());
  bool fully_visible = true;
  for (const auto & point : projected) {
    fully_visible = fully_visible && point.x >= 0.0F && point.y >= 0.0F &&
      point.x < static_cast<float>(depth.width) && point.y < static_cast<float>(depth.height);
    polygon.emplace_back(cvRound(point.x), cvRound(point.y));
  }
  cv::Mat workspace_mask(
    static_cast<int>(depth.height), static_cast<int>(depth.width), CV_8UC1, cv::Scalar(0));
  cv::fillConvexPoly(workspace_mask, polygon, cv::Scalar(255));
  const auto expected = static_cast<uint64_t>(cv::countNonZero(workspace_mask));
  if (expected == 0) {
    return std::nullopt;
  }
  uint64_t valid = 0;
  for (size_t row = 0; row < depth.height; ++row) {
    const auto * mask_row = workspace_mask.ptr<uint8_t>(static_cast<int>(row));
    for (size_t column = 0; column < depth.width; ++column) {
      if (mask_row[column] == 0) {
        continue;
      }
      float depth_m = 0.0F;
      if (read_depth(depth, row, column, depth_m) && finite(depth_m) && depth_m > 0.0F) {
        ++valid;
      }
    }
  }
  return TrayDepthCoverage{
    static_cast<double>(valid) / static_cast<double>(expected), fully_visible};
}


}  // namespace arm_cell_vision
