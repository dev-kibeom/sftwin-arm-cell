#include "depth_profile_support.hpp"
#include "arm_cell_vision/cuboid_top_face.hpp"

#include <algorithm>
#include <exception>
#include <string>

namespace arm_cell_vision
{
using depth_profile::failure;
using depth_profile::finite_transform;
using depth_profile::valid_camera;
using depth_profile::valid_image_storage;
using depth_profile::valid_profile;
using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

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
    return failure(
      ResultCode::DETECT_RESULT_INVALID_RESULT, "unknown target profile",
      "profile_resolution");
  }
  const auto & profile = *found_profile;
  if (profile.algorithm != "cuboid_top_face" ||
    !valid_profile(profile, calibration_only))
  {
    return failure(
      ResultCode::DETECT_RESULT_INVALID_RESULT, "invalid cuboid profile config",
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
    return failure(
      ResultCode::DETECT_RESULT_SENSOR_ERROR, "invalid camera calibration",
      "observation_validation");
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
    return failure(
      ResultCode::DETECT_RESULT_SENSOR_ERROR, "incompatible depth image data",
      "observation_validation");
  }
  if (!calibration_only && (input.optical_to_base.header.frame_id != "base_link" ||
    input.optical_to_base.child_frame_id != input.depth.header.frame_id ||
    !finite_transform(input.optical_to_base)))
  {
    return failure(
      ResultCode::DETECT_RESULT_TF_ERROR, "invalid optical to base transform",
      "optical_to_base_transform");
  }

  tf2::Transform support_T_camera;
  if (const auto error = resolve_support_calibration(input, profile, support_T_camera)) {
    return *error;
  }
  if (calibration_only) {
    return failure(
      ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET,
      "support calibration ready; cuboid detection requires active acquisition",
      "calibration_only");
  }

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
    const auto coverage = tray_depth_coverage(
      input.depth, input.camera_info, profile,
      support_T_camera);
    if (coverage && coverage->fully_visible &&
      coverage->valid_depth_fraction >= profile.minimum_observation_coverage)
    {
      result.result_code = ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND;
      result.rawpart_absence_confirmation_available = true;
      result.rawpart_absence_confirmed = true;
      result.diagnostic_detail = "valid workspace coverage confirms absence";
    } else if (!coverage) {
      result.diagnostic_detail += "; tray_depth_coverage_unavailable";
    } else if (!coverage->fully_visible) {
      result.diagnostic_detail += "; tray_roi_outside_depth_frame";
    } else {
      result.diagnostic_detail += "; insufficient_tray_depth_coverage";
    }
  }
  return result;
}

}  // namespace arm_cell_vision
