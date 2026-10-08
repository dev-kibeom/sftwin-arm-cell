#include "arm_cell_vision/detect_target_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_vision
{
namespace
{

using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

struct Point
{
  double x;
  double y;
  double z;
  size_t index;
};

DetectionResult result(uint8_t code, const char * detail)
{
  DetectionResult output;
  output.result_code = code;
  output.diagnostic_detail = detail;
  return output;
}

bool finite_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  return std::isfinite(translation.x) && std::isfinite(translation.y) &&
         std::isfinite(translation.z) && std::isfinite(rotation.x) &&
         std::isfinite(rotation.y) && std::isfinite(rotation.z) &&
         std::isfinite(rotation.w) &&
         (rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z +
         rotation.w * rotation.w > 1e-12);
}

bool valid_camera(const sensor_msgs::msg::CameraInfo & info)
{
  return info.width > 0 && info.height > 0 && !info.header.frame_id.empty() &&
         std::isfinite(info.k[0]) && std::isfinite(info.k[2]) &&
         std::isfinite(info.k[4]) && std::isfinite(info.k[5]) && info.k[0] > 0.0 &&
         info.k[4] > 0.0;
}

bool valid_image_storage(const sensor_msgs::msg::Image & image, size_t bytes_per_pixel)
{
  return image.width > 0 && image.height > 0 && image.step >= image.width * bytes_per_pixel &&
         image.data.size() >= image.step * image.height;
}

bool read_depth(const sensor_msgs::msg::Image & image, size_t index, float & depth)
{
  if (image.encoding == "32FC1") {
    std::memcpy(&depth, image.data.data() + index * sizeof(float), sizeof(float));
    return true;
  }
  if (image.encoding == "16UC1") {
    uint16_t millimeters = 0;
    std::memcpy(&millimeters, image.data.data() + index * sizeof(uint16_t), sizeof(uint16_t));
    depth = static_cast<float>(millimeters) / 1000.0F;
    return true;
  }
  return false;
}

double median(std::vector<double> values)
{
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 == 1) {
    return *middle;
  }
  const auto lower = std::max_element(values.begin(), middle);
  return (*lower + *middle) / 2.0;
}

}  // namespace

DetectionResult DetectTargetProcessor::process(const DetectionInput & input) const
{
  if (!valid_camera(input.camera_info) || input.camera_info.header.frame_id !=
    input.depth.header.frame_id)
  {
    return result(ResultCode::DETECT_RESULT_SENSOR_ERROR, "invalid camera calibration");
  }
  if (input.rgb.width != input.depth.width || input.rgb.height != input.depth.height ||
    input.rgb.width != input.camera_info.width || input.rgb.height != input.camera_info.height ||
    input.rgb.header.frame_id != input.depth.header.frame_id || input.rgb.encoding != "rgb8" ||
    !valid_image_storage(input.rgb, 3) ||
    !(input.depth.encoding == "32FC1" || input.depth.encoding == "16UC1") ||
    !valid_image_storage(
      input.depth,
      input.depth.encoding == "32FC1" ? sizeof(float) : sizeof(uint16_t)))
  {
    return result(ResultCode::DETECT_RESULT_SENSOR_ERROR, "incompatible RGB-D image data");
  }
  if (input.optical_to_base.header.frame_id != "base_link" ||
    input.optical_to_base.child_frame_id != input.depth.header.frame_id ||
    !finite_transform(input.optical_to_base))
  {
    return result(ResultCode::DETECT_RESULT_TF_ERROR, "invalid optical to base transform");
  }

  std::vector<double> valid_depths;
  valid_depths.reserve(input.depth.width * input.depth.height);
  for (size_t row = 0; row < input.depth.height; ++row) {
    for (size_t column = 0; column < input.depth.width; ++column) {
      float depth = 0.0F;
      const auto index = row * input.depth.step /
        (input.depth.encoding == "32FC1" ? sizeof(float) : sizeof(uint16_t)) + column;
      if (!read_depth(input.depth, index, depth) || !std::isfinite(depth) || depth <= 0.0F) {
        continue;
      }
      valid_depths.push_back(depth);
    }
  }
  if (valid_depths.empty()) {
    return result(ResultCode::DETECT_RESULT_SENSOR_ERROR, "no valid depth samples");
  }

  const auto support_depth = median(valid_depths);
  std::vector<Point> object_points;
  std::vector<double> object_depths;
  for (size_t row = 0; row < input.depth.height; ++row) {
    for (size_t column = 0; column < input.depth.width; ++column) {
      float depth = 0.0F;
      const auto index = row * input.depth.step /
        (input.depth.encoding == "32FC1" ? sizeof(float) : sizeof(uint16_t)) + column;
      if (!read_depth(input.depth, index, depth) || !std::isfinite(depth) ||
        depth <= 0.0F || support_depth - depth < 0.01)
      {
        continue;
      }
      object_points.push_back(
        {
          (static_cast<double>(column) - input.camera_info.k[2]) * depth / input.camera_info.k[0],
          (static_cast<double>(row) - input.camera_info.k[5]) * depth / input.camera_info.k[4],
          depth, index});
      object_depths.push_back(depth);
    }
  }
  if (object_points.empty()) {
    return result(ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND, "no segmented target");
  }
  if (object_points.size() < 2) {
    return result(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "insufficient target geometry");
  }

  const auto minmax_x = std::minmax_element(
    object_points.begin(), object_points.end(), [](const Point & left, const Point & right) {
      return left.x < right.x;
    });
  const auto minmax_y = std::minmax_element(
    object_points.begin(), object_points.end(), [](const Point & left, const Point & right) {
      return left.y < right.y;
    });
  if (minmax_x.second->x - minmax_x.first->x < 1e-4 &&
    minmax_y.second->y - minmax_y.first->y < 1e-4)
  {
    return result(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "target has no spatial extent");
  }

  const auto mean_x = std::accumulate(
    object_points.begin(), object_points.end(), 0.0,
    [](double sum, const Point & point) {return sum + point.x;}) / object_points.size();
  const auto mean_y = std::accumulate(
    object_points.begin(), object_points.end(), 0.0,
    [](double sum, const Point & point) {return sum + point.y;}) / object_points.size();
  const auto mean_z = std::accumulate(
    object_points.begin(), object_points.end(), 0.0,
    [](double sum, const Point & point) {return sum + point.z;}) / object_points.size();
  double covariance_xx = 0.0;
  double covariance_xy = 0.0;
  double covariance_yy = 0.0;
  for (const auto & point : object_points) {
    covariance_xx += (point.x - mean_x) * (point.x - mean_x);
    covariance_xy += (point.x - mean_x) * (point.y - mean_y);
    covariance_yy += (point.y - mean_y) * (point.y - mean_y);
  }
  const auto trace = covariance_xx + covariance_yy;
  const auto discriminant = std::sqrt(
    std::max(
      0.0, (covariance_xx - covariance_yy) * (covariance_xx - covariance_yy) +
      4.0 * covariance_xy * covariance_xy));
  const auto largest_eigenvalue = (trace + discriminant) / 2.0;
  const auto smallest_eigenvalue = (trace - discriminant) / 2.0;
  const auto & transform = input.optical_to_base.transform;
  tf2::Quaternion transform_rotation(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  transform_rotation.normalize();
  const tf2::Vector3 base_point =
    tf2::quatRotate(transform_rotation, tf2::Vector3(mean_x, mean_y, mean_z)) +
    tf2::Vector3(transform.translation.x, transform.translation.y, transform.translation.z);

  DetectionResult output;
  output.result_code = ResultCode::DETECT_RESULT_SUCCESS;
  output.target_pose.header = input.depth.header;
  output.target_pose.header.frame_id = "base_link";
  output.target_pose.pose.position.x = base_point.x();
  output.target_pose.pose.position.y = base_point.y();
  output.target_pose.pose.position.z = base_point.z();
  output.has_target_pose = true;
  output.estimated_height_m = static_cast<float>(support_depth - median(object_depths));
  output.has_estimated_height = std::isfinite(output.estimated_height_m) &&
    output.estimated_height_m > 0.0F;

  if (smallest_eigenvalue <= 1e-12 || largest_eigenvalue / smallest_eigenvalue < 1.5) {
    output.diagnostic_detail = "target detected; yaw unavailable";
    return output;
  }

  const auto yaw_optical =
    0.5 * std::atan2(2.0 * covariance_xy, covariance_xx - covariance_yy);
  const auto base_axis = tf2::quatRotate(
    transform_rotation, tf2::Vector3(std::cos(yaw_optical), std::sin(yaw_optical), 0.0));
  const auto yaw_base = std::atan2(base_axis.y(), base_axis.x());
  tf2::Quaternion object_rotation;
  object_rotation.setRPY(0.0, 0.0, yaw_base);
  object_rotation.normalize();
  output.target_pose.pose.orientation.x = object_rotation.x();
  output.target_pose.pose.orientation.y = object_rotation.y();
  output.target_pose.pose.orientation.z = object_rotation.z();
  output.target_pose.pose.orientation.w = object_rotation.w();
  output.has_target_yaw = true;
  output.diagnostic_detail = "target detected";
  return output;
}

}  // namespace arm_cell_vision
