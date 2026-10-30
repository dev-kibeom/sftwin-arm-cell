#include "arm_cell_vision/detect_target_processor.hpp"
#include "arm_cell_vision/cuboid_top_face.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <sstream>
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
  bool touches_image_boundary = false;
};

double wrap_square_yaw(double yaw)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  yaw = std::fmod(yaw, kHalfPi);
  return yaw < 0.0 ? yaw + kHalfPi : yaw;
}

double wrap_rectangle_yaw(double yaw)
{
  constexpr double kPi = 3.14159265358979323846;
  yaw = std::fmod(yaw, kPi);
  return yaw < 0.0 ? yaw + kPi : yaw;
}

bool rectangle_observation_qualified(
  double width, double height, const DetectorProfile & profile,
  const RawPartCandidateDiagnostic & diagnostic, double top_surface_flatness)
{
  return finite(width) && finite(height) && width > 0.0 && height > 0.0 &&
         diagnostic.inlier_fraction >= 0.95 && diagnostic.coverage >= 0.80 &&
         diagnostic.edge_support >= 0.50 &&
         top_surface_flatness <= profile.top_surface_geometry_tolerance_m;
}

double observed_planar_pixel_spacing(const std::vector<SamplePoint> & top_surface)
{
  std::map<uint64_t, const SamplePoint *> pixels;
  for (const auto & point : top_surface) {
    const auto key = (static_cast<uint64_t>(point.pixel_y) << 32) | point.pixel_x;
    pixels.emplace(key, &point);
  }
  std::array<std::vector<double>, 2> spacings;
  spacings[0].reserve(top_surface.size());
  spacings[1].reserve(top_surface.size());
  for (const auto & point : top_surface) {
    const std::array<std::pair<int, int>, 2> offsets = {{{1, 0}, {0, 1}}};
    for (size_t axis = 0; axis < offsets.size(); ++axis) {
      const auto & offset = offsets[axis];
      const auto next_x = static_cast<int64_t>(point.pixel_x) + offset.first;
      const auto next_y = static_cast<int64_t>(point.pixel_y) + offset.second;
      if (next_x < 0 || next_y < 0) {continue;}
      const auto key = (static_cast<uint64_t>(next_y) << 32) |
        static_cast<uint32_t>(next_x);
      const auto next = pixels.find(key);
      if (next != pixels.end()) {
        spacings[axis].push_back(std::hypot(
            point.support.x() - next->second->support.x(),
            point.support.y() - next->second->support.y()));
      }
    }
  }
  if (spacings[0].empty() || spacings[1].empty()) {return 0.0;}
  std::array<double, 2> median_spacing{};
  for (size_t axis = 0; axis < spacings.size(); ++axis) {
    const auto middle = spacings[axis].begin() + spacings[axis].size() / 2;
    std::nth_element(spacings[axis].begin(), middle, spacings[axis].end());
    median_spacing[axis] = *middle;
  }
  return std::hypot(median_spacing[0], median_spacing[1]);
}

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

DetectionResult failure(uint8_t code, const char * detail, const char * processing_stage)
{
  DetectionResult output;
  output.result_code = code;
  output.failure_cause = detail;
  output.diagnostic_detail = detail;
  output.processing_stage = processing_stage;
  return output;
}

bool finite(double value)
{
  return std::isfinite(value);
}

double normalized_residual(double residual, double tolerance)
{
  if (tolerance > 0.0) {
    return residual / tolerance;
  }
  return residual == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
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

bool inside_fiducial_footprint(const DetectorProfile & profile, double x, double y)
{
  const auto dx = x - profile.marker_in_support_m[0];
  const auto dy = y - profile.marker_in_support_m[1];
  const auto yaw = profile.marker_rpy_in_support_rad[2];
  const auto local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
  const auto local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);
  return std::abs(local_x) <= profile.fiducial_size_m / 2.0 &&
         std::abs(local_y) <= profile.fiducial_size_m / 2.0;
}

bool valid_profile(const DetectorProfile & profile, bool calibration_requested)
{
  const auto dictionary = resolve_fiducial_dictionary(profile.fiducial_dictionary);
  if (profile.target_id != "RawPart" || profile.material_id != profile.target_id ||
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
  const tf2::Transform & support_T_camera,
  size_t & workspace_xyz_points,
  std::string & candidate_generation_reason)
{
  workspace_xyz_points = 0;
  candidate_generation_reason = "workspace_projection_unavailable";
  const auto fx = camera_info.k[0];
  const auto fy = camera_info.k[4];
  const auto cx = camera_info.k[2];
  const auto cy = camera_info.k[5];
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  const cv::Mat distortion = camera_info.d.empty() ?
    cv::Mat::zeros(1, 5, CV_64F) : cv::Mat(camera_info.d, true);
  const bool has_distortion = std::any_of(
    camera_info.d.begin(), camera_info.d.end(), [](double coefficient) {
      return coefficient != 0.0;
    });

  const auto camera_T_support = support_T_camera.inverse();
  std::vector<cv::Point3f> workspace_corners;
  for (const auto x : {profile.workspace_roi_min_m[0], profile.workspace_roi_max_m[0]}) {
    for (const auto y : {profile.workspace_roi_min_m[1], profile.workspace_roi_max_m[1]}) {
      for (const auto z : {profile.workspace_roi_min_m[2], profile.workspace_roi_max_m[2]}) {
        const auto point = camera_T_support * tf2::Vector3(x, y, z);
        if (!finite(point.x()) || !finite(point.y()) || !finite(point.z()) || point.z() <= 0.0) {
          candidate_generation_reason = "workspace_corner_not_projectable";
          return {};
        }
        workspace_corners.emplace_back(point.x(), point.y(), point.z());
      }
    }
  }
  std::vector<cv::Point2f> projected_corners;
  try {
    cv::projectPoints(
      workspace_corners, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
      camera_matrix, distortion, projected_corners);
  } catch (const cv::Exception &) {
    candidate_generation_reason = "workspace_projection_failed";
    return {};
  }
  if (projected_corners.empty()) {
    candidate_generation_reason = "workspace_projection_empty";
    return {};
  }
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : projected_corners) {
    min_x = std::min(min_x, static_cast<double>(point.x));
    max_x = std::max(max_x, static_cast<double>(point.x));
    min_y = std::min(min_y, static_cast<double>(point.y));
    max_y = std::max(max_y, static_cast<double>(point.y));
  }
  if (max_x < 0.0 || max_y < 0.0 || min_x >= depth.width || min_y >= depth.height) {
    candidate_generation_reason = "workspace_roi_outside_image";
    return {};
  }
  const auto first_column = static_cast<size_t>(std::max(0.0, std::floor(min_x)));
  const auto last_column = static_cast<size_t>(std::min(
      static_cast<double>(depth.width - 1), std::ceil(max_x)));
  const auto first_row = static_cast<size_t>(std::max(0.0, std::floor(min_y)));
  const auto last_row = static_cast<size_t>(std::min(
      static_cast<double>(depth.height - 1), std::ceil(max_y)));
  const auto region_width = last_column - first_column + 1;
  const auto region_height = last_row - first_row + 1;
  const auto count = region_width * region_height;
  std::vector<SamplePoint> organized(count);
  std::vector<uint8_t> foreground(count, 0);

  std::vector<cv::Point2f> pixels;
  std::vector<cv::Point2f> normalized;
  if (has_distortion) {
    pixels.reserve(count);
    for (size_t row = first_row; row <= last_row; ++row) {
      for (size_t column = first_column; column <= last_column; ++column) {
        pixels.emplace_back(static_cast<float>(column), static_cast<float>(row));
      }
    }
    cv::undistortPoints(pixels, normalized, camera_matrix, distortion);
  }

  for (size_t row = first_row; row <= last_row; ++row) {
    for (size_t column = first_column; column <= last_column; ++column) {
      const auto index = (row - first_row) * region_width + column - first_column;
      float depth_m = 0.0F;
      if (!read_depth(depth, row, column, depth_m) || !finite(depth_m) || depth_m <= 0.0F) {
        continue;
      }
      double normalized_x = (static_cast<double>(column) - cx) / fx;
      double normalized_y = (static_cast<double>(row) - cy) / fy;
      if (!normalized.empty()) {
        normalized_x = normalized[index].x;
        normalized_y = normalized[index].y;
      }
      const tf2::Vector3 camera_point(
        normalized_x * depth_m, normalized_y * depth_m, depth_m);
      const auto support_point = support_T_camera * camera_point;
      if (inside_fiducial_footprint(profile, support_point.x(), support_point.y())) {
        continue;
      }
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
      workspace_xyz_points += foreground[index] != 0;
    }
  }

  std::vector<Candidate> candidates;
  candidate_generation_reason = "workspace_xyz_filter_empty";
  if (workspace_xyz_points != 0) {
    candidate_generation_reason = "connected_components_formed";
  }
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
      const auto row = index / region_width;
      const auto column = index % region_width;
      if (organized[index].pixel_y == 0 || organized[index].pixel_x == 0 ||
        organized[index].pixel_y + 1 >= depth.height ||
        organized[index].pixel_x + 1 >= depth.width)
      {
        candidate.touches_image_boundary = true;
      }
      for (int row_offset = -1; row_offset <= 1; ++row_offset) {
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
          const auto next_row = static_cast<int64_t>(row) + row_offset;
          const auto next_column = static_cast<int64_t>(column) + column_offset;
          if ((row_offset == 0 && column_offset == 0) || next_row < 0 || next_column < 0 ||
            next_row >= static_cast<int64_t>(region_height) ||
            next_column >= static_cast<int64_t>(region_width))
          {
            continue;
          }
          const auto next = static_cast<size_t>(next_row) * region_width +
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
  if (candidates.empty() && workspace_xyz_points != 0) {
    candidate_generation_reason = "no_connected_components";
  }
  return candidates;
}

std::optional<TrayDepthCoverage> rawpart_tray_depth_coverage_impl(
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

bool matches_rawpart_profile(
  const Candidate & candidate, const DetectorProfile & profile,
  std::vector<SamplePoint> & top_surface, tf2::Vector3 & support_centroid,
  double & support_height, std::array<double, 6> & stability_features,
  double & observation_quality,
  RawPartCandidateDiagnostic & diagnostic)
{
  diagnostic.rejection_reason = "insufficient_evidence";
  if (candidate.points.size() < 4) {
    diagnostic.rejection_reason = "candidate_too_few_points";
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
    diagnostic.rejection_reason = "top_surface_too_few_points";
    return false;
  }

  auto sum_x = 0.0;
  auto sum_y = 0.0;
  auto sum_z = 0.0;
  for (const auto & point : top_surface) {
    sum_x += point.support.x();
    sum_y += point.support.y();
    sum_z += point.support.z();
  }
  support_centroid = tf2::Vector3(
    sum_x / top_surface.size(), sum_y / top_surface.size(), sum_z / top_surface.size());
  support_height = support_centroid.z();
  diagnostic.measured_vertical_height_m = support_height;
  diagnostic.expected_vertical_height_m = profile.dimensions_m[2];
  diagnostic.vertical_height_tolerance_m =
    profile.dimension_tolerance_m[2] + profile.support_contact_tolerance_m;
  std::vector<cv::Point2f> projected;
  projected.reserve(top_surface.size());
  for (const auto & point : top_surface) {
    projected.emplace_back(point.support.x(), point.support.y());
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(projected, hull);
  if (hull.size() < 4) {
    diagnostic.rejection_reason = "insufficient_evidence";
    return false;
  }
  const auto rectangle = cv::minAreaRect(hull);
  auto width = static_cast<double>(rectangle.size.width);
  auto height = static_cast<double>(rectangle.size.height);
  auto yaw = rectangle.angle * CV_PI / 180.0;
  if (width < height) {
    std::swap(width, height);
    yaw += CV_PI / 2.0;
  }
  yaw = wrap_rectangle_yaw(yaw);
  diagnostic.measured_width_m = width;
  diagnostic.measured_height_m = height;
  diagnostic.expected_width_m = profile.dimensions_m[0];
  diagnostic.expected_height_m = profile.dimensions_m[1];
  diagnostic.width_tolerance_m = profile.dimension_tolerance_m[0];
  diagnostic.height_tolerance_m = profile.dimension_tolerance_m[1];
  diagnostic.position_x_m = support_centroid.x();
  diagnostic.position_y_m = support_centroid.y();
  diagnostic.position_z_m = support_centroid.z();
  diagnostic.observed_yaw_rad = yaw;
  diagnostic.orientation_symmetry_order = 0;
  const auto hull_area = std::abs(cv::contourArea(hull));
  const auto hull_coverage = std::clamp(hull_area / std::max(1e-9, width * height), 0.0, 1.0);
  constexpr int kCoverageBins = 12;
  std::array<bool, kCoverageBins * kCoverageBins> occupied_bins{};
  for (const auto & point : projected) {
    const auto dx = point.x - rectangle.center.x;
    const auto dy = point.y - rectangle.center.y;
    const auto local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
    const auto local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);
    const auto bin_x = std::clamp(static_cast<int>((local_x / width + 0.5) * kCoverageBins),
      0, kCoverageBins - 1);
    const auto bin_y = std::clamp(static_cast<int>((local_y / height + 0.5) * kCoverageBins),
      0, kCoverageBins - 1);
    occupied_bins[bin_y * kCoverageBins + bin_x] = true;
  }
  const auto occupied_count = static_cast<double>(std::count(
      occupied_bins.begin(), occupied_bins.end(), true));
  diagnostic.coverage = std::min(hull_coverage,
    occupied_count / static_cast<double>(occupied_bins.size()));
  cv::Point2f corners[4];
  rectangle.points(corners);
  std::array<size_t, 4> edge_counts{};
  constexpr size_t kEdgeCoverageBins = 8;
  std::array<std::array<bool, kEdgeCoverageBins>, 4> edge_coverage_bins{};
  size_t inliers = 0;
  for (size_t point_index = 0; point_index < projected.size(); ++point_index) {
    const auto & point = projected[point_index];
    if (std::abs(top_surface[point_index].support.z() - support_height) <=
      profile.top_surface_geometry_tolerance_m) {++inliers;}
    constexpr double kEdgeBand = 0.0025;
    for (size_t edge = 0; edge < 4; ++edge) {
      const auto & a = corners[edge];
      const auto & b = corners[(edge + 1) % 4];
      const auto ex = static_cast<double>(b.x - a.x);
      const auto ey = static_cast<double>(b.y - a.y);
      const auto ax = static_cast<double>(a.x);
      const auto ay = static_cast<double>(a.y);
      const auto length_sq = ex * ex + ey * ey;
      const auto t = std::clamp(((point.x - ax) * ex + (point.y - ay) * ey) /
        std::max(1e-12, length_sq), 0.0, 1.0);
      const auto distance = std::hypot(point.x - (ax + t * ex), point.y - (ay + t * ey));
      if (distance <= kEdgeBand) {
        ++edge_counts[edge];
        const auto bin = std::min(
          kEdgeCoverageBins - 1, static_cast<size_t>(t * kEdgeCoverageBins));
        edge_coverage_bins[edge][bin] = true;
      }
    }
  }
  diagnostic.inlier_fraction = static_cast<double>(inliers) / projected.size();
  diagnostic.edge_support = 1.0;
  for (size_t edge = 0; edge < edge_counts.size(); ++edge) {
    const auto point_support = static_cast<double>(edge_counts[edge]) /
      std::max(1.0, std::sqrt(static_cast<double>(projected.size())) * 0.5);
    const auto occupied_edge_bins = static_cast<double>(std::count(
        edge_coverage_bins[edge].begin(), edge_coverage_bins[edge].end(), true));
    const auto edge_extent_support = occupied_edge_bins / kEdgeCoverageBins;
    diagnostic.edge_support = std::min(diagnostic.edge_support,
      std::clamp(std::min(point_support, edge_extent_support), 0.0, 1.0));
  }
  for (const auto & corner : corners) {
    if (corner.x <= profile.workspace_roi_min_m[0] ||
      corner.x >= profile.workspace_roi_max_m[0] ||
      corner.y <= profile.workspace_roi_min_m[1] ||
      corner.y >= profile.workspace_roi_max_m[1])
    {
      diagnostic.rejection_reason = "workspace_roi_exceeded";
      return false;
    }
  }
  diagnostic.max_normalized_residual = std::max(
    normalized_residual(std::abs(width - profile.dimensions_m[0]), profile.dimension_tolerance_m[0]),
    normalized_residual(std::abs(height - profile.dimensions_m[1]), profile.dimension_tolerance_m[1]));
  for (const auto & corner : corners) {
    geometry_msgs::msg::Point32 point;
    point.x = corner.x;
    point.y = corner.y;
    diagnostic.fitted_outline_support.push_back(point);
    const auto nearest = std::min_element(
      top_surface.begin(), top_surface.end(), [&](const SamplePoint & left, const SamplePoint & right) {
        return std::hypot(left.support.x() - corner.x, left.support.y() - corner.y) <
               std::hypot(right.support.x() - corner.x, right.support.y() - corner.y);
      });
    geometry_msgs::msg::Point32 pixel;
    pixel.x = nearest->pixel_x;
    pixel.y = nearest->pixel_y;
    diagnostic.fitted_outline_pixels.push_back(pixel);
  }
  for (const auto side : {-0.4, 0.4}) {
    const auto x = rectangle.center.x + side * width * std::cos(yaw);
    const auto y = rectangle.center.y + side * width * std::sin(yaw);
    const auto nearest = std::min_element(
      top_surface.begin(), top_surface.end(), [&](const SamplePoint & left, const SamplePoint & right) {
        return std::hypot(left.support.x() - x, left.support.y() - y) <
               std::hypot(right.support.x() - x, right.support.y() - y);
      });
    geometry_msgs::msg::Point32 pixel;
    pixel.x = nearest->pixel_x;
    pixel.y = nearest->pixel_y;
    diagnostic.orientation_axis_pixels.push_back(pixel);
  }
  auto min_z = std::numeric_limits<double>::infinity();
  auto max_z = -std::numeric_limits<double>::infinity();
  for (const auto & point : candidate.points) {
    min_z = std::min(min_z, point.support.z());
    max_z = std::max(max_z, point.support.z());
  }
  auto maximum_flatness_residual = 0.0;
  for (const auto & point : top_surface) {
    maximum_flatness_residual = std::max(
      maximum_flatness_residual, std::abs(point.support.z() - support_height));
  }
  if (support_height <= profile.support_contact_tolerance_m) {
    diagnostic.rejection_reason = "support_height_invalid";
    return false;
  }
  if (candidate.touches_image_boundary) {
    diagnostic.rejection_reason = "image_boundary_clipped";
    return false;
  }
  if (!rectangle_observation_qualified(
      width, height, profile, diagnostic, maximum_flatness_residual)) {
    diagnostic.orientation_symmetry_order = 0;
    diagnostic.rejection_reason = "insufficient_evidence";
    return false;
  }
  // Each fitted side is quantized by observed image samples. Use the observed
  // horizontal and vertical support-plane pixel spacing for the side-fit delta.
  const auto side_measurement_resolution = observed_planar_pixel_spacing(top_surface);
  const bool square_within_observation_tolerance = std::abs(width - height) <=
    side_measurement_resolution;
  diagnostic.orientation_symmetry_order = square_within_observation_tolerance ? 4 : 2;
  diagnostic.observed_yaw_valid = true;
  if (square_within_observation_tolerance) {yaw = wrap_square_yaw(yaw);}
  diagnostic.observed_yaw_rad = yaw;
  // Nominal dimensions remain diagnostic-only and never affect approval or rank.
  diagnostic.selection_residual = 0.0;
  observation_quality = static_cast<double>(top_surface.size()) * diagnostic.coverage *
    diagnostic.edge_support;
  stability_features = {support_centroid.x(), support_centroid.y(), support_height,
    width, height, max_z - min_z};
  const auto valid = std::all_of(stability_features.begin(), stability_features.end(), finite);
  if (valid) {
    diagnostic.profile_match = true;
    diagnostic.rejection_reason = "none";
  }
  return valid;
}

}  // namespace

std::optional<TrayDepthCoverage> rawpart_tray_depth_coverage(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera_info,
  const DetectorProfile & profile, const tf2::Transform & support_T_camera)
{
  return rawpart_tray_depth_coverage_impl(depth, camera_info, profile, support_T_camera);
}

void DetectTargetProcessor::set_loaded_support_calibration(
  const SupportCalibrationSnapshot & calibration)
{
  if (!calibration.ready) {
    throw std::invalid_argument("loaded support calibration must be READY");
  }
  std::lock_guard<std::mutex> lock(selection_state_->mutex);
  selection_state_->explicit_calibration_observations = 0;
  selection_state_->recent_depth_validations.clear();
  selection_state_->cached_tray_from_camera = calibration.tray_from_camera;
  selection_state_->cached_support_region = calibration.support_region;
}

void DetectTargetProcessor::clear_loaded_support_calibration()
{
  std::lock_guard<std::mutex> lock(selection_state_->mutex);
  selection_state_->cached_tray_from_camera.reset();
  selection_state_->cached_support_region.clear();
  selection_state_->explicit_calibration_observations = 0;
  selection_state_->recent_depth_validations.clear();
}

SupportCalibrationSnapshot DetectTargetProcessor::support_calibration_snapshot() const
{
  std::lock_guard<std::mutex> lock(selection_state_->mutex);
  SupportCalibrationSnapshot snapshot;
  snapshot.accepted_frames = selection_state_->explicit_calibration_observations;
  snapshot.required_frames = snapshot.accepted_frames > 0 ? 1U : 0U;
  if (!selection_state_->recent_depth_validations.empty()) {
    const auto & validation = selection_state_->recent_depth_validations.back();
    if (validation.residual_available) {
      snapshot.median_depth_residual_m = validation.median_signed_residual_m;
    }
  }
  snapshot.ready = selection_state_->cached_tray_from_camera.has_value();
  snapshot.recent_depth_validations = selection_state_->recent_depth_validations;
  snapshot.support_region = selection_state_->cached_support_region;
  if (snapshot.ready) {
    snapshot.tray_from_camera = *selection_state_->cached_tray_from_camera;
  }
  return snapshot;
}

DetectionResult DetectTargetProcessor::process(
  const DetectionInput & input, const std::string & target_id,
  const std::vector<DetectorProfile> & profiles, bool calibration_only) const
{
  auto result = process_profile(input, target_id, profiles, calibration_only);
  const auto profile = std::find_if(
    profiles.begin(), profiles.end(), [&](const auto & p) {
      return p.target_id == target_id;
    });
  if (!calibration_only && profile != profiles.end()) {
    result.algorithm = profile->algorithm;
    result.applied_profile = std::make_shared<const DetectorProfile>(*profile);
  }
  return result;
}

DetectionResult DetectTargetProcessor::process_profile(
  const DetectionInput & input, const std::string & target_id,
  const std::vector<DetectorProfile> & profiles, bool calibration_only) const
{
  const auto found_profile = std::find_if(
    profiles.begin(), profiles.end(), [&target_id](const DetectorProfile & profile) {
      return profile.target_id == target_id;
    });
  if (target_id.empty() || found_profile == profiles.end()) {
    return failure(ResultCode::DETECT_RESULT_INVALID_RESULT, "unknown target profile", "profile_resolution");
  }
  const auto & profile = *found_profile;
  if (profile.algorithm != "cuboid_top_face" ||
    !valid_profile(profile, calibration_only))
  {
    return failure(
      ResultCode::DETECT_RESULT_INVALID_RESULT, "invalid RawPart profile config",
      "profile_resolution");
  }
  if (!calibration_only && !support_calibration_snapshot().ready) {
    return failure(
      ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
      "selected Calibration file is not READY", "calibration_readiness");
  }
  if (!valid_camera(input.camera_info) || input.camera_info.header.frame_id !=
    input.depth.header.frame_id)
  {
    return failure(ResultCode::DETECT_RESULT_SENSOR_ERROR, "invalid camera calibration", "observation_validation");
  }
  const bool rgb_available = input.rgb.width > 0 && input.rgb.height > 0;
  const bool rgb_compatible = rgb_available &&
    input.rgb.width == input.depth.width && input.rgb.height == input.depth.height &&
    input.rgb.width == input.camera_info.width && input.rgb.height == input.camera_info.height &&
    input.rgb.header.frame_id == input.depth.header.frame_id && input.rgb.encoding == "rgb8" &&
    valid_image_storage(input.rgb, 3);
  if ((calibration_only || !support_calibration_snapshot().ready) && !rgb_compatible) {
    return failure(
      ResultCode::DETECT_RESULT_SENSOR_ERROR,
      "compatible RGB image is required for support fiducial calibration",
      "observation_validation");
  }
  if (input.depth.width != input.camera_info.width ||
    input.depth.height != input.camera_info.height ||
    !(input.depth.encoding == "32FC1" || input.depth.encoding == "16UC1") ||
    !valid_image_storage(
      input.depth,
      input.depth.encoding == "32FC1" ? sizeof(float) : sizeof(uint16_t)))
  {
    return failure(ResultCode::DETECT_RESULT_SENSOR_ERROR, "incompatible depth image data", "observation_validation");
  }
  if (!calibration_only && (input.optical_to_base.header.frame_id != "base_link" ||
    input.optical_to_base.child_frame_id != input.depth.header.frame_id ||
    !finite_transform(input.optical_to_base)))
  {
    return failure(ResultCode::DETECT_RESULT_TF_ERROR, "invalid optical to base transform", "optical_to_base_transform");
  }

  tf2::Transform support_T_camera;
  bool have_cached_calibration = false;
  {
    std::lock_guard<std::mutex> lock(selection_state_->mutex);
    if (selection_state_->cached_tray_from_camera) {
      support_T_camera = transform_from_ros(
        selection_state_->cached_tray_from_camera->transform);
      have_cached_calibration = true;
    }
  }
  const auto & camera_info = input.camera_info;
  if (!have_cached_calibration) {
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
      return failure(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "fiducial detection failed", "support_reference_estimation");
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
        "configured support fiducial is ambiguous", "support_reference_estimation");
    }

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
      return failure(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "support pose estimation failed", "support_reference_estimation");
    }
    const auto marker_index = matching_markers.front();
    if (marker_index >= rotation_vectors.size() || marker_index >= translation_vectors.size()) {
      return failure(ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "support pose is unavailable", "support_reference_estimation");
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
    constexpr std::array<double, 3> kMarkerSampleFractions{0.25, 0.5, 0.75};
    std::vector<cv::Point2f> marker_sample_pixels;
    marker_sample_pixels.reserve(kMarkerSampleFractions.size() * kMarkerSampleFractions.size());
    const auto & corners = marker_corners[marker_index];
    for (const auto v : kMarkerSampleFractions) {
      for (const auto u : kMarkerSampleFractions) {
        marker_sample_pixels.push_back(
          (1.0 - u) * (1.0 - v) * corners[0] +
          u * (1.0 - v) * corners[1] + u * v * corners[2] +
          (1.0 - u) * v * corners[3]);
      }
    }
    std::vector<cv::Point2f> undistorted_marker_samples;
    cv::undistortPoints(
      marker_sample_pixels, undistorted_marker_samples, camera_matrix, distortion);
    std::vector<tf2::Vector3> support_depth_points;
    for (size_t index = 0; index < marker_sample_pixels.size(); ++index) {
      const auto & pixel = marker_sample_pixels[index];
      const auto column = static_cast<size_t>(std::clamp(
          std::lround(pixel.x), 0L, static_cast<long>(input.depth.width - 1)));
      const auto row = static_cast<size_t>(std::clamp(
          std::lround(pixel.y), 0L, static_cast<long>(input.depth.height - 1)));
      float depth_m = 0.0F;
      if (!read_depth(input.depth, row, column, depth_m) || !finite(depth_m) || depth_m <= 0.0F) {
        continue;
      }
      support_depth_points.emplace_back(
        undistorted_marker_samples[index].x * depth_m,
        undistorted_marker_samples[index].y * depth_m, depth_m);
    }
    auto retain_depth_validation = [this](
      SupportCalibrationSnapshot::DepthValidation evidence) {
        std::lock_guard<std::mutex> lock(selection_state_->mutex);
        auto & validations = selection_state_->recent_depth_validations;
        validations.clear();
        validations.push_back(std::move(evidence));
      };
    SupportCalibrationSnapshot::DepthValidation depth_validation;
    depth_validation.valid_sample_count = static_cast<uint32_t>(support_depth_points.size());
    depth_validation.support_contact_tolerance_m = profile.support_contact_tolerance_m;
    depth_validation.configured_marker_height_m = profile.marker_in_support_m[2];
    if (support_depth_points.size() < 5) {
      depth_validation.accepted = false;
      depth_validation.reason = "insufficient_valid_support_depth_samples";
      retain_depth_validation(std::move(depth_validation));
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
        "support depth is unavailable for geometric validation", "support_reference_estimation");
    }
    const auto & marker_translation = translation_vectors[marker_index];
    const tf2::Transform camera_T_marker(
      camera_q_marker,
      tf2::Vector3(
        marker_translation[0] * profile.fiducial_pose_translation_scale,
        marker_translation[1] * profile.fiducial_pose_translation_scale,
        marker_translation[2] * profile.fiducial_pose_translation_scale));
    support_T_camera = transform_from_profile(profile) * camera_T_marker.inverse();

    std::vector<double> support_plane_residuals;
    support_plane_residuals.reserve(support_depth_points.size());
    for (const auto & camera_point : support_depth_points) {
      const auto support_point = support_T_camera * camera_point;
      support_plane_residuals.push_back(
        support_point.z() - profile.marker_in_support_m[2]);
    }
    std::sort(support_plane_residuals.begin(), support_plane_residuals.end());
    const auto middle = support_plane_residuals.size() / 2;
    const double median_support_plane_residual = support_plane_residuals.size() % 2 == 0 ?
      (support_plane_residuals[middle - 1] + support_plane_residuals[middle]) / 2.0 :
      support_plane_residuals[middle];
    depth_validation.residual_available = true;
    depth_validation.median_signed_residual_m = median_support_plane_residual;
    depth_validation.minimum_signed_residual_m = support_plane_residuals.front();
    depth_validation.maximum_signed_residual_m = support_plane_residuals.back();
    depth_validation.accepted = std::abs(median_support_plane_residual) <=
      profile.support_contact_tolerance_m;
    depth_validation.reason = depth_validation.accepted ? "accepted" :
      "support_depth_conflicts_with_configured_support_plane";
    retain_depth_validation(depth_validation);
    if (std::abs(median_support_plane_residual) > profile.support_contact_tolerance_m) {
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
        "support depth conflicts with configured support plane", "support_reference_estimation");
    }

    geometry_msgs::msg::Transform candidate;
    candidate.translation.x = support_T_camera.getOrigin().x();
    candidate.translation.y = support_T_camera.getOrigin().y();
    candidate.translation.z = support_T_camera.getOrigin().z();
    candidate.rotation.x = support_T_camera.getRotation().x();
    candidate.rotation.y = support_T_camera.getRotation().y();
    candidate.rotation.z = support_T_camera.getRotation().z();
    candidate.rotation.w = support_T_camera.getRotation().w();
    std::lock_guard<std::mutex> lock(selection_state_->mutex);
    geometry_msgs::msg::TransformStamped cached;
    cached.header = input.depth.header;
    cached.header.frame_id = "Tray";
    cached.child_frame_id = input.depth.header.frame_id;
    cached.transform.translation.x = candidate.translation.x;
    cached.transform.translation.y = candidate.translation.y;
    cached.transform.translation.z = candidate.translation.z;
    cached.transform.rotation.x = candidate.rotation.x;
    cached.transform.rotation.y = candidate.rotation.y;
    cached.transform.rotation.z = candidate.rotation.z;
    cached.transform.rotation.w = candidate.rotation.w;
    selection_state_->cached_tray_from_camera = cached;
    selection_state_->explicit_calibration_observations = 1;
    selection_state_->cached_support_region.clear();
    for (const auto & corner : marker_corners[marker_index]) {
      geometry_msgs::msg::Point32 point;
      point.x = corner.x;
      point.y = corner.y;
      selection_state_->cached_support_region.push_back(point);
    }
  }

  if (calibration_only) {
    return failure(
      ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
      "support calibration ready; RawPart detection requires active acquisition",
      "calibration_only");
  }

  if (profile.algorithm == "cuboid_top_face") {
    DetectionResult result;
    try {
      result = process_cuboid_top_face(input, profile, support_T_camera);
    } catch (const std::exception & error) {
      result.algorithm = profile.algorithm;
      result.result_code = ResultCode::DETECT_RESULT_GEOMETRY_ERROR;
      result.processing_stage = "cuboid_top_face_geometry";
      result.failure_cause = "Cuboid required geometry/occupancy failed";
      result.diagnostic_detail = std::string("Cuboid required geometry/occupancy failed: ") +
        error.what();
      return result;
    }
    if (result.result_code == ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET &&
      result.processing_stage == "cuboid_top_face_geometry" &&
      result.rawpart_workspace_xyz_points == 0 &&
      result.diagnostic_detail.find("cuboid top geometry unresolved;") == 0)
    {
      const auto coverage = rawpart_tray_depth_coverage(
        input.depth, camera_info, profile,
        support_T_camera);
      if (coverage && coverage->fully_visible &&
        coverage->valid_depth_fraction >= profile.minimum_observation_coverage)
      {
        result.result_code = ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND;
        result.rawpart_absence_confirmation_available = true;
        result.rawpart_absence_confirmed = true;
        result.diagnostic_detail = "valid workspace coverage confirms absence";
      }
    }
    return result;
  }

  std::vector<Candidate> candidates;
  size_t workspace_xyz_points = 0;
  std::string candidate_generation_reason;
  try {
    candidates = depth_candidates(
      input.depth, camera_info, profile, support_T_camera, workspace_xyz_points,
      candidate_generation_reason);
  } catch (const cv::Exception &) {
    return failure(
      ResultCode::DETECT_RESULT_SENSOR_ERROR,
      "depth geometry could not be deprojected", "depth_candidate_generation");
  }
  std::vector<std::vector<SamplePoint>> matches;
  std::vector<size_t> matched_candidate_indices;
  std::vector<tf2::Vector3> matched_centroids;
  std::vector<double> matched_heights;
  std::vector<std::array<double, 6>> matched_features;
  std::vector<double> matched_observation_qualities;
  std::vector<RawPartCandidateDiagnostic> candidate_diagnostics;
  std::map<std::string, size_t> rejected_candidates;
  for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
    const auto & candidate = candidates[candidate_index];
    std::vector<SamplePoint> top_surface;
    tf2::Vector3 centroid;
    double height = 0.0;
    std::array<double, 6> features{};
    double observation_quality = 0.0;
    RawPartCandidateDiagnostic diagnostic;
    diagnostic.candidate_id = static_cast<uint32_t>(candidate_index);
    if (matches_rawpart_profile(
        candidate, profile, top_surface, centroid, height, features,
        observation_quality, diagnostic))
    {
      matches.push_back(std::move(top_surface));
      matched_candidate_indices.push_back(candidate_index);
      matched_centroids.push_back(centroid);
      matched_heights.push_back(height);
      matched_features.push_back(features);
      matched_observation_qualities.push_back(observation_quality);
    } else {
      ++rejected_candidates[diagnostic.rejection_reason];
    }
    candidate_diagnostics.push_back(std::move(diagnostic));
  }
  if (matches.empty()) {
    const auto observation_coverage = rawpart_tray_depth_coverage(
      input.depth, camera_info, profile, support_T_camera);
    std::ostringstream trace;
    trace << "rawpart_detection{candidate_generation=" << candidate_generation_reason
          << ",workspace_xyz_points=" << workspace_xyz_points
          << ",candidates=" << candidates.size() << ",profile_matches=0,rejected={";
    bool first = true;
    for (const auto & [reason, count] : rejected_candidates) {
      if (!first) {trace << ',';}
      trace << reason << ':' << count;
      first = false;
    }
    if (first) {trace << "none";}
    trace << "}";
    const auto with_trace = [&trace, &candidates, &candidate_diagnostics, workspace_xyz_points,
        &candidate_generation_reason, &rejected_candidates](uint8_t code, const char * reason) {
        auto result = failure(code, reason, "absence_confirmation");
        result.has_rawpart_diagnostics = true;
        result.processing_stage = "absence_confirmation";
        result.rawpart_workspace_xyz_points = workspace_xyz_points;
        result.rawpart_candidate_count = candidates.size();
        result.rawpart_profile_match_count = 0;
        result.rawpart_candidate_generation = candidate_generation_reason;
        result.rawpart_candidate_diagnostics = candidate_diagnostics;
        for (const auto & rejection : rejected_candidates) {
          result.rawpart_rejected_by_reason.emplace(rejection.first, rejection.second);
        }
        result.rawpart_absence_confirmation_available = true;
        result.rawpart_absence_confirmed = false;
        result.diagnostic_detail = trace.str() +
          "; candidate_result=no_matching_profile_candidate; absence_confirmation=rejected; reason="
          + reason;
        return result;
      };
    if (!observation_coverage) {
      return with_trace(
        ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
        "Tray ROI has no visible depth pixels");
    }
    if (!observation_coverage->fully_visible) {
      return with_trace(
        ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
        "workspace is not fully visible in the current observation");
    }
    if (observation_coverage->valid_depth_fraction < profile.minimum_observation_coverage) {
      return with_trace(
        ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
        "insufficient valid depth coverage in RawPart workspace");
    }
    if (rejected_candidates.count("insufficient_evidence") != 0) {
      auto result = with_trace(
        ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
        "RawPart candidate geometry has insufficient evidence");
      result.processing_stage = "candidate_profile_validation";
      result.rawpart_candidate_diagnostics = std::move(candidate_diagnostics);
      return result;
    }
    if (rejected_candidates.count("workspace_roi_exceeded") != 0) {
      return with_trace(
        ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
        "RawPart Rectangle exceeds the configured Workspace ROI");
    }
    if (rejected_candidates.count("image_boundary_clipped") != 0) {
      return with_trace(
        ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
        "RawPart candidate is clipped by the camera image boundary");
    }
    auto result = failure(
      ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND, "no matching RawPart candidate",
      "absence_confirmation");
    result.has_rawpart_diagnostics = true;
    result.processing_stage = "absence_confirmation";
    result.rawpart_workspace_xyz_points = workspace_xyz_points;
    result.rawpart_candidate_count = candidates.size();
    result.rawpart_candidate_generation = candidate_generation_reason;
    result.rawpart_absence_confirmation_available = true;
    result.rawpart_absence_confirmed = true;
    for (const auto & rejection : rejected_candidates) {
      result.rawpart_rejected_by_reason.emplace(rejection.first, rejection.second);
    }
    result.diagnostic_detail = trace.str() +
      "; candidate_result=no_matching_profile_candidate; "
      "absence_confirmation=confirmed_valid_workspace_coverage; "
      "reason=no matching RawPart candidate";
    return result;
  }

  size_t selected_match = static_cast<size_t>(std::distance(
      matched_observation_qualities.begin(), std::max_element(
        matched_observation_qualities.begin(), matched_observation_qualities.end())));

  std::vector<size_t> greedy_order(matches.size());
  for (size_t index = 0; index < greedy_order.size(); ++index) {greedy_order[index] = index;}
  std::stable_sort(greedy_order.begin(), greedy_order.end(), [&](size_t left, size_t right) {
    if (matched_observation_qualities[left] != matched_observation_qualities[right]) {
      return matched_observation_qualities[left] > matched_observation_qualities[right];
    }
    return false;
  });
  for (size_t rank = 0; rank < greedy_order.size(); ++rank) {
    candidate_diagnostics[matched_candidate_indices[greedy_order[rank]]].greedy_rank =
      static_cast<uint32_t>(rank + 1);
  }

  const auto camera_centroid = support_T_camera.inverse() * matched_centroids[selected_match];
  const auto base_T_camera = transform_from_ros(input.optical_to_base.transform);
  const auto base_centroid = base_T_camera * camera_centroid;
  const auto & selected_diagnostic =
    candidate_diagnostics[matched_candidate_indices[selected_match]];
  if (!selected_diagnostic.observed_yaw_valid ||
    !finite(selected_diagnostic.observed_yaw_rad) ||
    (selected_diagnostic.orientation_symmetry_order != 2 &&
    selected_diagnostic.orientation_symmetry_order != 4))
  {
    return failure(
      ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
      "qualified RawPart rectangle has no valid orientation contract", "target_pose_estimation");
  }
  DetectionResult output;
  if (rgb_compatible) {output.source_rgb_header = input.rgb.header;}
  output.result_code = ResultCode::DETECT_RESULT_SUCCESS;
  output.processing_stage = "target_pose_estimation";
  output.target_pose.header = input.depth.header;
  output.target_pose.header.frame_id = "base_link";
  output.target_pose.pose.position.x = base_centroid.x();
  output.target_pose.pose.position.y = base_centroid.y();
  output.target_pose.pose.position.z = base_centroid.z();
  {
    const auto base_T_support = base_T_camera * support_T_camera.inverse();
    tf2::Quaternion support_orientation(0.0, 0.0,
      std::sin(selected_diagnostic.observed_yaw_rad / 2.0),
      std::cos(selected_diagnostic.observed_yaw_rad / 2.0));
    auto measured_orientation = base_T_support.getRotation() * support_orientation;
    measured_orientation.normalize();
    const auto orientation_norm = measured_orientation.length();
    if (!finite(orientation_norm) || std::abs(orientation_norm - 1.0) > 1e-6) {
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR,
        "qualified RawPart rectangle produced an invalid orientation", "target_pose_estimation");
    }
    output.target_pose.pose.orientation.x = measured_orientation.x();
    output.target_pose.pose.orientation.y = measured_orientation.y();
    output.target_pose.pose.orientation.z = measured_orientation.z();
    output.target_pose.pose.orientation.w = measured_orientation.w();
  }
  output.has_target_pose = true;
  output.profile_candidate_valid = true;
  output.profile_candidate_features = matched_features[selected_match];
  output.orientation_symmetry_order = selected_diagnostic.orientation_symmetry_order;
  output.observed_yaw_rad = selected_diagnostic.observed_yaw_rad;
  candidate_diagnostics[matched_candidate_indices[selected_match]].selected_for_target = true;
  output.has_observed_rectangle = true;
  output.rectangle_long_side_m = static_cast<float>(selected_diagnostic.measured_width_m);
  output.rectangle_short_side_m = static_cast<float>(selected_diagnostic.measured_height_m);
  output.rawpart_candidate_diagnostics = std::move(candidate_diagnostics);
  output.has_rawpart_diagnostics = true;
  output.rawpart_workspace_xyz_points = workspace_xyz_points;
  output.rawpart_candidate_count = candidates.size();
  output.rawpart_profile_match_count = matches.size();
  output.rawpart_candidate_generation = candidate_generation_reason;
  for (const auto & rejection : rejected_candidates) {
    output.rawpart_rejected_by_reason.emplace(rejection.first, rejection.second);
  }
  output.rawpart_selected_candidate = "features:";
  for (size_t index = 0; index < matched_features[selected_match].size(); ++index) {
    if (index != 0) {output.rawpart_selected_candidate += ",";}
    output.rawpart_selected_candidate += std::to_string(matched_features[selected_match][index]);
  }
  output.estimated_height_m = static_cast<float>(matched_heights[selected_match]);
  output.has_estimated_height = finite(output.estimated_height_m) &&
    output.estimated_height_m > 0.0F;
  const auto & selected = candidates[matched_candidate_indices[selected_match]];
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
  for (const auto & point : matches[selected_match]) {
    centroid_x += point.pixel_x;
    centroid_y += point.pixel_y;
  }
  if (!matches[selected_match].empty()) {
    output.centroid_pixel_valid = true;
    output.centroid_pixel_x = centroid_x / static_cast<float>(matches[selected_match].size());
    output.centroid_pixel_y = centroid_y / static_cast<float>(matches[selected_match].size());
  }
  {
    std::lock_guard<std::mutex> lock(selection_state_->mutex);
    output.support_region = selection_state_->cached_support_region;
  }
  output.support_region_valid = !output.support_region.empty();
  if (!output.support_region_valid) {
    // A selected candidate remains usable if marker pixels were not cached.
    output.support_region.clear();
  }
  output.diagnostic_detail = "RawPart detected from configured support-relative depth geometry";
  return output;
}

}  // namespace arm_cell_vision
