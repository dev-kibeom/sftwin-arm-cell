#include "arm_cell_vision/detect_target_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_vision
{
namespace
{

using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

struct SamplePoint
{
  tf2::Vector3 support;
  tf2::Vector3 camera;
  uint32_t pixel_x;
  uint32_t pixel_y;
};

struct Candidate
{
  std::vector<SamplePoint> points;
};

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

DetectionResult failure(uint8_t code, const char * detail)
{
  DetectionResult output;
  output.result_code = code;
  output.diagnostic_detail = detail;
  return output;
}

bool finite(double value)
{
  return std::isfinite(value);
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

bool valid_profile(const DetectorProfile & profile)
{
  const auto dictionary = resolve_fiducial_dictionary(profile.fiducial_dictionary);
  if (profile.target_id != "RawPart" || profile.shape != "box" ||
    dictionary.empty() || profile.fiducial_marker_id < 0 ||
    profile.fiducial_marker_id >= dictionary->bytesList.rows ||
    !finite(profile.fiducial_size_m) ||
    profile.fiducial_size_m <= 0.0 ||
    !finite(profile.fiducial_pose_translation_scale) ||
    profile.fiducial_pose_translation_scale < 0.8 ||
    profile.fiducial_pose_translation_scale > 1.2 ||
    !finite(profile.support_contact_tolerance_m) ||
    profile.support_contact_tolerance_m <= 0.0 ||
    !finite(profile.top_surface_geometry_tolerance_m) ||
    profile.top_surface_geometry_tolerance_m <= 0.0 ||
    !finite_values(profile.dimensions_m) ||
    !finite_values(profile.dimension_tolerance_m) ||
    !finite_values(profile.workspace_roi_min_m) ||
    !finite_values(profile.workspace_roi_max_m) ||
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

tf2::Transform transform_from_profile(const DetectorProfile & profile)
{
  tf2::Quaternion rotation;
  rotation.setRPY(
    profile.marker_rpy_in_support_rad[0], profile.marker_rpy_in_support_rad[1],
    profile.marker_rpy_in_support_rad[2]);
  rotation.normalize();
  return tf2::Transform(
    rotation,
    tf2::Vector3(
      profile.marker_in_support_m[0], profile.marker_in_support_m[1],
      profile.marker_in_support_m[2]));
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

std::vector<Candidate> depth_candidates(
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const DetectorProfile & profile,
  const tf2::Transform & support_T_camera)
{
  const auto count = static_cast<size_t>(depth.width) * depth.height;
  std::vector<SamplePoint> organized(count);
  std::vector<uint8_t> foreground(count, 0);
  const auto fx = camera_info.k[0];
  const auto fy = camera_info.k[4];
  const auto cx = camera_info.k[2];
  const auto cy = camera_info.k[5];

  std::vector<cv::Point2f> pixels;
  std::vector<cv::Point2f> normalized;
  if (!camera_info.d.empty()) {
    pixels.reserve(count);
    for (size_t row = 0; row < depth.height; ++row) {
      for (size_t column = 0; column < depth.width; ++column) {
        pixels.emplace_back(static_cast<float>(column), static_cast<float>(row));
      }
    }
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    const cv::Mat distortion(camera_info.d, true);
    cv::undistortPoints(pixels, normalized, camera_matrix, distortion);
  }

  for (size_t row = 0; row < depth.height; ++row) {
    for (size_t column = 0; column < depth.width; ++column) {
      float depth_m = 0.0F;
      if (!read_depth(depth, row, column, depth_m) || !finite(depth_m) || depth_m <= 0.0F) {
        continue;
      }
      double normalized_x = (static_cast<double>(column) - cx) / fx;
      double normalized_y = (static_cast<double>(row) - cy) / fy;
      if (!normalized.empty()) {
        const auto index = row * depth.width + column;
        normalized_x = normalized[index].x;
        normalized_y = normalized[index].y;
      }
      const tf2::Vector3 camera_point(
        normalized_x * depth_m, normalized_y * depth_m, depth_m);
      const auto support_point = support_T_camera * camera_point;
      const auto index = row * depth.width + column;
      organized[index] = {
        support_point, camera_point, static_cast<uint32_t>(column),
        static_cast<uint32_t>(row)};
      foreground[index] =
        support_point.x() >= profile.workspace_roi_min_m[0] &&
        support_point.x() <= profile.workspace_roi_max_m[0] &&
        support_point.y() >= profile.workspace_roi_min_m[1] &&
        support_point.y() <= profile.workspace_roi_max_m[1] &&
        support_point.z() >= profile.workspace_roi_min_m[2] &&
        support_point.z() <= profile.workspace_roi_max_m[2];
    }
  }

  std::vector<Candidate> candidates;
  std::vector<uint8_t> visited(count, 0);
  const auto continuity_sq = profile.top_surface_geometry_tolerance_m *
    profile.top_surface_geometry_tolerance_m;
  for (size_t seed = 0; seed < count; ++seed) {
    if (!foreground[seed] || visited[seed]) {
      continue;
    }
    Candidate candidate;
    std::queue<size_t> pending;
    pending.push(seed);
    visited[seed] = 1;
    while (!pending.empty()) {
      const auto index = pending.front();
      pending.pop();
      candidate.points.push_back(organized[index]);
      const auto row = index / depth.width;
      const auto column = index % depth.width;
      for (int row_offset = -1; row_offset <= 1; ++row_offset) {
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
          const auto next_row = static_cast<int64_t>(row) + row_offset;
          const auto next_column = static_cast<int64_t>(column) + column_offset;
          if ((row_offset == 0 && column_offset == 0) || next_row < 0 || next_column < 0 ||
            next_row >= depth.height || next_column >= depth.width)
          {
            continue;
          }
          const auto next = static_cast<size_t>(next_row) * depth.width +
            static_cast<size_t>(next_column);
          if (!foreground[next] || visited[next]) {
            continue;
          }
          if ((organized[next].support - organized[index].support).length2() <= continuity_sq) {
            visited[next] = 1;
            pending.push(next);
          }
        }
      }
    }
    candidates.push_back(std::move(candidate));
  }
  return candidates;
}

bool matches_rawpart_profile(
  const Candidate & candidate, const DetectorProfile & profile,
  std::vector<SamplePoint> & top_surface, tf2::Vector3 & support_centroid,
  double & support_height)
{
  if (candidate.points.size() < 4) {
    return false;
  }
  std::vector<double> heights;
  heights.reserve(candidate.points.size());
  for (const auto & point : candidate.points) {
    heights.push_back(point.support.z());
  }
  std::sort(heights.begin(), heights.end());
  const auto top_quantile_index = static_cast<size_t>(0.9 * (heights.size() - 1));
  const auto top_quantile = heights[top_quantile_index];
  for (const auto & point : candidate.points) {
    if (point.support.z() >= top_quantile - profile.top_surface_geometry_tolerance_m) {
      top_surface.push_back(point);
    }
  }
  if (top_surface.size() < 4) {
    return false;
  }

  auto min_x = std::numeric_limits<double>::infinity();
  auto max_x = -std::numeric_limits<double>::infinity();
  auto min_y = std::numeric_limits<double>::infinity();
  auto max_y = -std::numeric_limits<double>::infinity();
  auto sum_x = 0.0;
  auto sum_y = 0.0;
  auto sum_z = 0.0;
  for (const auto & point : top_surface) {
    min_x = std::min(min_x, point.support.x());
    max_x = std::max(max_x, point.support.x());
    min_y = std::min(min_y, point.support.y());
    max_y = std::max(max_y, point.support.y());
    sum_x += point.support.x();
    sum_y += point.support.y();
    sum_z += point.support.z();
  }
  support_centroid = tf2::Vector3(
    sum_x / top_surface.size(), sum_y / top_surface.size(), sum_z / top_surface.size());
  support_height = support_centroid.z();
  const auto extent_x = max_x - min_x;
  const auto extent_y = max_y - min_y;
  if (std::abs(extent_x - profile.dimensions_m[0]) > profile.dimension_tolerance_m[0] ||
    std::abs(extent_y - profile.dimensions_m[1]) > profile.dimension_tolerance_m[1])
  {
    return false;
  }
  if (std::abs(support_height - profile.dimensions_m[2]) >
    profile.dimension_tolerance_m[2] + profile.support_contact_tolerance_m)
  {
    return false;
  }
  const auto mean_height = support_height;
  const auto max_residual = std::max_element(
    top_surface.begin(), top_surface.end(), [mean_height](const auto & left, const auto & right) {
      return std::abs(left.support.z() - mean_height) <
      std::abs(right.support.z() - mean_height);
    });
  return std::abs(max_residual->support.z() - mean_height) <=
         profile.top_surface_geometry_tolerance_m;
}

}  // namespace

DetectionResult DetectTargetProcessor::process(
  const DetectionInput & input, const std::string & target_id,
  const std::vector<DetectorProfile> & profiles) const
{
  const auto found_profile = std::find_if(
    profiles.begin(), profiles.end(), [&target_id](const DetectorProfile & profile) {
      return profile.target_id == target_id;
    });
  if (target_id.empty() || found_profile == profiles.end()) {
    return failure(ResultCode::DETECT_RESULT_INVALID_RESULT, "unknown target profile");
  }
  const auto & profile = *found_profile;
  if (!valid_profile(profile)) {
    return failure(ResultCode::DETECT_RESULT_INVALID_RESULT, "invalid RawPart profile config");
  }
  if (!valid_camera(input.camera_info) || input.camera_info.header.frame_id !=
    input.depth.header.frame_id)
  {
    return failure(ResultCode::DETECT_RESULT_SENSOR_ERROR, "invalid camera calibration");
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
    return failure(ResultCode::DETECT_RESULT_SENSOR_ERROR, "incompatible RGB-D image data");
  }
  if (input.optical_to_base.header.frame_id != "base_link" ||
    input.optical_to_base.child_frame_id != input.depth.header.frame_id ||
    !finite_transform(input.optical_to_base))
  {
    return failure(ResultCode::DETECT_RESULT_TF_ERROR, "invalid optical to base transform");
  }

  cv::Mat rgb(
    static_cast<int>(input.rgb.height), static_cast<int>(input.rgb.width), CV_8UC3,
    const_cast<uint8_t *>(input.rgb.data.data()), input.rgb.step);
  cv::Mat gray;
  cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
  const auto dictionary = resolve_fiducial_dictionary(profile.fiducial_dictionary);
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  const auto detector_parameters = cv::aruco::DetectorParameters::create();
  try {
    cv::aruco::detectMarkers(
      gray, dictionary, marker_corners, marker_ids, detector_parameters);
  } catch (const cv::Exception &) {
    return failure(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "fiducial detection failed");
  }
  std::vector<size_t> matching_markers;
  for (size_t index = 0; index < marker_ids.size(); ++index) {
    if (marker_ids[index] == profile.fiducial_marker_id) {
      matching_markers.push_back(index);
    }
  }
  if (matching_markers.size() != 1) {
    return failure(
      ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
      matching_markers.empty() ? "configured support fiducial is missing" :
      "configured support fiducial is ambiguous");
  }

  const auto & camera_info = input.camera_info;
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], 0.0, camera_info.k[2],
    0.0, camera_info.k[4], camera_info.k[5], 0.0, 0.0, 1.0);
  const cv::Mat distortion = camera_info.d.empty() ?
    cv::Mat::zeros(1, 5, CV_64F) : cv::Mat(camera_info.d, true);
  std::vector<cv::Vec3d> rotation_vectors;
  std::vector<cv::Vec3d> translation_vectors;
  try {
    cv::aruco::estimatePoseSingleMarkers(
      marker_corners, static_cast<float>(profile.fiducial_size_m), camera_matrix,
      distortion, rotation_vectors, translation_vectors);
  } catch (const cv::Exception &) {
    return failure(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "support pose estimation failed");
  }
  const auto marker_index = matching_markers.front();
  if (marker_index >= rotation_vectors.size() || marker_index >= translation_vectors.size()) {
    return failure(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "support pose is unavailable");
  }
  cv::Mat camera_R_marker;
  cv::Rodrigues(rotation_vectors[marker_index], camera_R_marker);
  camera_R_marker.convertTo(camera_R_marker, CV_64F);
  tf2::Quaternion camera_q_marker;
  tf2::Matrix3x3(
    camera_R_marker.at<double>(0, 0), camera_R_marker.at<double>(0, 1),
    camera_R_marker.at<double>(0, 2), camera_R_marker.at<double>(1, 0),
    camera_R_marker.at<double>(1, 1), camera_R_marker.at<double>(1, 2),
    camera_R_marker.at<double>(2, 0), camera_R_marker.at<double>(2, 1),
    camera_R_marker.at<double>(2, 2)).getRotation(camera_q_marker);
  // Sample synchronized depth at the detected marker corners. The configured
  // translation scale calibrates the RGB pose against this camera's rendered
  // projection while retaining the independent support-plane consistency
  // check below.
  std::vector<tf2::Vector3> support_depth_points;
  for (const auto & corner : marker_corners[marker_index]) {
    const auto column = static_cast<size_t>(std::clamp(
        std::lround(corner.x), 0L, static_cast<long>(input.depth.width - 1)));
    const auto row = static_cast<size_t>(std::clamp(
        std::lround(corner.y), 0L, static_cast<long>(input.depth.height - 1)));
    float depth_m = 0.0F;
    if (!read_depth(input.depth, row, column, depth_m) || !finite(depth_m) || depth_m <= 0.0F) {
      continue;
    }
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(
      std::vector<cv::Point2f>{corner}, undistorted, camera_matrix, distortion);
    const auto camera_point = tf2::Vector3(
      undistorted.front().x * depth_m, undistorted.front().y * depth_m, depth_m);
    support_depth_points.push_back(camera_point);
  }
  if (support_depth_points.size() < 3) {
    return failure(
      ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
      "support depth is unavailable for geometric validation");
  }
  const auto & marker_translation = translation_vectors[marker_index];
  const tf2::Transform camera_T_marker(
    camera_q_marker,
    tf2::Vector3(
      marker_translation[0] * profile.fiducial_pose_translation_scale,
      marker_translation[1] * profile.fiducial_pose_translation_scale,
      marker_translation[2] * profile.fiducial_pose_translation_scale));
  const auto support_T_camera = transform_from_profile(profile) * camera_T_marker.inverse();

  for (const auto & camera_point : support_depth_points) {
    const auto support_point = support_T_camera * camera_point;
    if (std::abs(support_point.z() - profile.marker_in_support_m[2]) >
      profile.support_contact_tolerance_m)
    {
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
        "support depth conflicts with configured support plane");
    }
  }

  std::vector<Candidate> candidates;
  try {
    candidates = depth_candidates(input.depth, camera_info, profile, support_T_camera);
  } catch (const cv::Exception &) {
    return failure(
      ResultCode::DETECT_RESULT_SENSOR_ERROR,
      "depth geometry could not be deprojected");
  }
  std::vector<std::vector<SamplePoint>> matches;
  std::vector<size_t> matched_candidate_indices;
  std::vector<tf2::Vector3> matched_centroids;
  std::vector<double> matched_heights;
  for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
    const auto & candidate = candidates[candidate_index];
    std::vector<SamplePoint> top_surface;
    tf2::Vector3 centroid;
    double height = 0.0;
    if (matches_rawpart_profile(candidate, profile, top_surface, centroid, height)) {
      matches.push_back(std::move(top_surface));
      matched_candidate_indices.push_back(candidate_index);
      matched_centroids.push_back(centroid);
      matched_heights.push_back(height);
    }
  }
  if (matches.empty()) {
    return failure(ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND, "no matching RawPart candidate");
  }
  if (matches.size() != 1) {
    return failure(
      ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
      "multiple RawPart candidates satisfy the configured profile");
  }

  const auto camera_centroid = support_T_camera.inverse() * matched_centroids.front();
  const auto base_T_camera = transform_from_ros(input.optical_to_base.transform);
  const auto base_centroid = base_T_camera * camera_centroid;
  DetectionResult output;
  output.source_rgb_header = input.rgb.header;
  output.result_code = ResultCode::DETECT_RESULT_SUCCESS;
  output.target_pose.header = input.depth.header;
  output.target_pose.header.frame_id = "base_link";
  output.target_pose.pose.position.x = base_centroid.x();
  output.target_pose.pose.position.y = base_centroid.y();
  output.target_pose.pose.position.z = base_centroid.z();
  output.has_target_pose = true;
  output.has_target_yaw = false;
  output.estimated_height_m = static_cast<float>(matched_heights.front());
  output.has_estimated_height = finite(output.estimated_height_m) &&
    output.estimated_height_m > 0.0F;
  const auto & selected = candidates[matched_candidate_indices.front()];
  auto min_x = input.depth.width;
  auto min_y = input.depth.height;
  uint32_t max_x = 0;
  uint32_t max_y = 0;
  for (const auto & point : selected.points) {
    min_x = std::min(min_x, point.pixel_x);
    min_y = std::min(min_y, point.pixel_y);
    max_x = std::max(max_x, point.pixel_x);
    max_y = std::max(max_y, point.pixel_y);
  }
  output.object_region_valid = true;
  output.object_region_x = min_x;
  output.object_region_y = min_y;
  output.object_region_width = max_x - min_x + 1;
  output.object_region_height = max_y - min_y + 1;
  float centroid_x = 0.0F;
  float centroid_y = 0.0F;
  for (const auto & point : matches.front()) {
    centroid_x += point.pixel_x;
    centroid_y += point.pixel_y;
  }
  if (!matches.front().empty()) {
    output.centroid_pixel_valid = true;
    output.centroid_pixel_x = centroid_x / static_cast<float>(matches.front().size());
    output.centroid_pixel_y = centroid_y / static_cast<float>(matches.front().size());
  }
  output.support_region_valid = true;
  for (const auto & corner : marker_corners[marker_index]) {
    geometry_msgs::msg::Point32 point;
    point.x = corner.x;
    point.y = corner.y;
    output.support_region.push_back(point);
  }
  output.diagnostic_detail = "RawPart detected from configured support-relative depth geometry";
  return output;
}

}  // namespace arm_cell_vision
