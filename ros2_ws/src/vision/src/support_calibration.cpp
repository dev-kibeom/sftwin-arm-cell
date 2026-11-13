#include "depth_profile_support.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_vision
{
using depth_profile::failure;
using depth_profile::finite;
using depth_profile::read_depth;
using depth_profile::resolve_fiducial_dictionary;
using depth_profile::transform_from_ros;
using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

namespace
{
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

}  // namespace

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

std::optional<DetectionResult> DetectTargetProcessor::resolve_support_calibration(
  const DetectionInput & input, const DetectorProfile & profile,
  tf2::Transform & support_T_camera) const
{
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
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "fiducial detection failed",
        "support_reference_estimation");
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
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "support pose estimation failed",
        "support_reference_estimation");
    }
    const auto marker_index = matching_markers.front();
    if (marker_index >= rotation_vectors.size() || marker_index >= translation_vectors.size()) {
      return failure(
        ResultCode::DETECT_RESULT_GEOMETRY_ERROR, "support pose is unavailable",
        "support_reference_estimation");
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

  return std::nullopt;
}

}  // namespace arm_cell_vision
