#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "arm_cell_vision/detect_target_processor.hpp"

namespace
{

using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

constexpr int kWidth = 320;
constexpr int kHeight = 300;
constexpr double kFocalLength = 1000.0;
constexpr double kMarkerX = -0.085;
constexpr double kMarkerY = -0.105;

arm_cell_vision::DetectorProfile rawpart_profile()
{
  arm_cell_vision::DetectorProfile profile;
  profile.target_id = "RawPart";
  profile.material_id = "RawPart";
  profile.shape = "box";
  profile.dimensions_m = {0.07, 0.07, 0.08};
  profile.dimension_tolerance_m = {0.01, 0.01, 0.01};
  profile.workspace_roi_min_m = {-0.12, -0.14, 0.01};
  profile.workspace_roi_max_m = {0.12, 0.14, 0.12};
  profile.support_contact_tolerance_m = 0.01;
  profile.top_surface_geometry_tolerance_m = 0.01;
  profile.fiducial_dictionary = "DICT_4X4_50";
  profile.fiducial_marker_id = 0;
  profile.fiducial_size_m = 0.04;
  profile.fiducial_pose_translation_scale = 1.0011;
  profile.marker_in_support_m = {kMarkerX, kMarkerY, 0.0002};
  profile.marker_rpy_in_support_rad = {0.0, 0.0, 0.0};
  return profile;
}

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_id(const std::string & name)
{
  if (name == "DICT_5X5_50") {
    return cv::aruco::DICT_5X5_50;
  }
  return cv::aruco::DICT_4X4_50;
}

arm_cell_vision::DetectionInput rgbd_input(
  bool with_marker, const std::string & dictionary_name = "DICT_4X4_50",
  int rendered_marker_pixels = 41)
{
  arm_cell_vision::DetectionInput input;
  input.rgb.header.frame_id = "camera_color_optical_frame";
  input.rgb.header.stamp.sec = 42;
  input.rgb.header.stamp.nanosec = 1234;
  input.rgb.width = kWidth;
  input.rgb.height = kHeight;
  input.rgb.encoding = "rgb8";
  input.rgb.step = kWidth * 3;
  input.rgb.data.assign(input.rgb.step * kHeight, 255);

  if (with_marker) {
    cv::Mat marker;
    cv::aruco::drawMarker(
      cv::aruco::getPredefinedDictionary(dictionary_id(dictionary_name)), 0,
      rendered_marker_pixels, marker, 1);
    constexpr int left = 55;
    constexpr int top = 235;
    for (int row = 0; row < marker.rows; ++row) {
      for (int column = 0; column < marker.cols; ++column) {
        const auto value = marker.at<uint8_t>(row, column);
        const auto pixel = (static_cast<size_t>(top + row) * kWidth + left + column) * 3;
        std::fill_n(input.rgb.data.begin() + pixel, 3, value);
      }
    }
  }

  input.depth.header.frame_id = input.rgb.header.frame_id;
  input.depth.width = kWidth;
  input.depth.height = kHeight;
  input.depth.encoding = "32FC1";
  input.depth.step = kWidth * sizeof(float);
  std::vector<float> depth(kWidth * kHeight, 1.0F);
  for (int row = 112; row < 188; ++row) {
    for (int column = 122; column < 198; ++column) {
      depth[static_cast<size_t>(row) * kWidth + column] = 0.92F;
    }
  }
  for (int row = 168; row < 184; ++row) {
    for (int column = 72; column < 88; ++column) {
      depth[static_cast<size_t>(row) * kWidth + column] = 0.90F;
    }
  }
  input.depth.data.resize(input.depth.step * kHeight);
  std::memcpy(input.depth.data.data(), depth.data(), input.depth.data.size());

  input.camera_info.header.frame_id = input.rgb.header.frame_id;
  input.camera_info.width = kWidth;
  input.camera_info.height = kHeight;
  input.camera_info.k[0] = kFocalLength;
  input.camera_info.k[2] = kWidth / 2.0;
  input.camera_info.k[4] = kFocalLength;
  input.camera_info.k[5] = kHeight / 2.0;
  input.camera_info.d.assign(5, 0.0);

  input.optical_to_base.header.frame_id = "base_link";
  input.optical_to_base.child_frame_id = input.rgb.header.frame_id;
  input.optical_to_base.transform.rotation.w = 1.0;
  return input;
}

arm_cell_vision::SupportCalibrationSnapshot fixture_calibration()
{
  static const auto calibration = []() {
      auto input = rgbd_input(true);
      arm_cell_vision::DetectTargetProcessor calibrator;
      (void)calibrator.process(input, "RawPart", {rawpart_profile()}, true);
      return calibrator.support_calibration_snapshot();
    }();
  return calibration;
}

void install_fixture_calibration(arm_cell_vision::DetectTargetProcessor & processor)
{
  processor.set_loaded_support_calibration(fixture_calibration());
}

arm_cell_vision::DetectionResult process_with_fixture_calibration(
  const arm_cell_vision::DetectionInput & input, const std::string & target_id,
  const std::vector<arm_cell_vision::DetectorProfile> & profiles)
{
  arm_cell_vision::DetectTargetProcessor processor;
  install_fixture_calibration(processor);
  return processor.process(input, target_id, profiles);
}

void set_candidate(
  arm_cell_vision::DetectionInput & input, int left, int top, int width, int height,
  float depth_m)
{
  for (int row = top; row < top + height; ++row) {
    for (int column = left; column < left + width; ++column) {
      const auto offset = (static_cast<size_t>(row) * kWidth + column) * sizeof(float);
      std::memcpy(input.depth.data.data() + offset, &depth_m, sizeof(depth_m));
    }
  }
}

void clear_candidate(
  arm_cell_vision::DetectionInput & input, int left, int top, int width,
  int height)
{
  set_candidate(input, left, top, width, height, 1.0F);
}

void set_depth_pixel(arm_cell_vision::DetectionInput & input, int x, int y, float depth_m)
{
  const auto offset = (static_cast<size_t>(y) * kWidth + x) * sizeof(float);
  std::memcpy(input.depth.data.data() + offset, &depth_m, sizeof(depth_m));
}

void set_rotated_rectangle(
  arm_cell_vision::DetectionInput & input, double angle_rad,
  int long_side_pixels, int short_side_pixels, float top_depth_m = 0.92F);

void set_rotated_square(
  arm_cell_vision::DetectionInput & input, double angle_rad, int side_pixels = 76)
{
  set_rotated_rectangle(input, angle_rad, side_pixels, side_pixels);
}

void set_rotated_rectangle(
  arm_cell_vision::DetectionInput & input, double angle_rad,
  int long_side_pixels, int short_side_pixels, float top_depth_m)
{
  clear_candidate(input, 122, 112, 76, 76);
  cv::Mat mask(kHeight, kWidth, CV_8UC1, cv::Scalar(0));
  const auto long_half = long_side_pixels / 2.0;
  const auto short_half = short_side_pixels / 2.0;
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  std::vector<cv::Point> corners;
  for (const auto & p : std::array<cv::Point2d, 4>{
      cv::Point2d(-long_half, -short_half), cv::Point2d(long_half, -short_half),
      cv::Point2d(long_half, short_half), cv::Point2d(-long_half, short_half)})
  {
    corners.emplace_back(
      static_cast<int>(std::lround(160.0 + p.x * c - p.y * s)),
      static_cast<int>(std::lround(150.0 + p.x * s + p.y * c)));
  }
  cv::fillConvexPoly(mask, corners, cv::Scalar(1));
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      if (mask.at<uint8_t>(y, x) != 0) {
        set_depth_pixel(input, x, y, top_depth_m);
      }
    }
  }
}

TEST(RawPartDetectTarget, CuboidAlgorithmRemainsSelectableAcrossAcquisitions)
{
  auto input = rgbd_input(true);
  clear_candidate(input, 72, 168, 16, 16);
  arm_cell_vision::DetectTargetProcessor processor;
  processor.set_loaded_support_calibration(fixture_calibration());
  auto profile = rawpart_profile();
  for (const auto & algorithm : {"cuboid_top_face", "cuboid_top_face"}) {
    profile.algorithm = algorithm;
    const auto result = processor.process(input, "RawPart", {profile});
    ASSERT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
    EXPECT_EQ(result.algorithm, algorithm);
    EXPECT_EQ(result.ordered_candidates.at(0).target_pose.header.frame_id, "base_link");
    EXPECT_EQ(result.ordered_candidates.at(0).target_pose.header.stamp, input.depth.header.stamp);
  }
}

TEST(RawPartDetectTarget, RejectsLegacyRectangleProfile)
{
  auto input = rgbd_input(true);
  arm_cell_vision::DetectTargetProcessor processor;
  processor.set_loaded_support_calibration(fixture_calibration());
  auto profile = rawpart_profile();
  profile.algorithm = "legacy_rectangle";

  const auto result = processor.process(input, "RawPart", {profile});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_EQ(result.processing_stage, "profile_resolution");
}

TEST(RawPartDetectTarget, LoadedCalibrationAllowsDepthOnlyCuboidDetection)
{
  arm_cell_vision::DetectTargetProcessor processor;
  processor.set_loaded_support_calibration(fixture_calibration());
  auto profile = rawpart_profile();
  for (const auto * algorithm : {"cuboid_top_face"}) {
    profile.algorithm = algorithm;
    auto rgbd = rgbd_input(true);
    clear_candidate(rgbd, 72, 168, 16, 16);
    const auto with_rgb = processor.process(rgbd, "RawPart", {profile});
    auto invalid_encoding_input = rgbd;
    invalid_encoding_input.rgb.encoding = "bgr8";
    auto short_buffer_input = rgbd;
    short_buffer_input.rgb.data.resize(short_buffer_input.rgb.data.size() - 1);
    rgbd.rgb = sensor_msgs::msg::Image{};
    const auto depth_only = processor.process(rgbd, "RawPart", {profile});

    ASSERT_EQ(with_rgb.result_code, ResultCode::DETECT_RESULT_SUCCESS)
      << with_rgb.diagnostic_detail;
    ASSERT_EQ(depth_only.result_code, with_rgb.result_code) << depth_only.diagnostic_detail;
    ASSERT_EQ(depth_only.has_target_pose, with_rgb.has_target_pose);
    ASSERT_EQ(depth_only.ordered_candidates.size(), with_rgb.ordered_candidates.size());
    EXPECT_NEAR(depth_only.target_pose.pose.position.x, with_rgb.target_pose.pose.position.x, 1e-9);
    EXPECT_NEAR(depth_only.target_pose.pose.position.y, with_rgb.target_pose.pose.position.y, 1e-9);
    EXPECT_NEAR(depth_only.target_pose.pose.position.z, with_rgb.target_pose.pose.position.z, 1e-9);
    EXPECT_TRUE(depth_only.source_rgb_header.frame_id.empty());

    const auto invalid_encoding = processor.process(
      invalid_encoding_input, "RawPart", {profile});
    EXPECT_EQ(invalid_encoding.result_code, with_rgb.result_code);
    EXPECT_EQ(invalid_encoding.ordered_candidates.size(), with_rgb.ordered_candidates.size());
    EXPECT_NEAR(invalid_encoding.target_pose.pose.position.x,
      with_rgb.target_pose.pose.position.x, 1e-9);
    EXPECT_NEAR(invalid_encoding.target_pose.pose.position.y,
      with_rgb.target_pose.pose.position.y, 1e-9);
    EXPECT_TRUE(invalid_encoding.source_rgb_header.frame_id.empty());

    const auto short_buffer = processor.process(short_buffer_input, "RawPart", {profile});
    EXPECT_EQ(short_buffer.result_code, with_rgb.result_code);
    EXPECT_EQ(short_buffer.ordered_candidates.size(), with_rgb.ordered_candidates.size());
    EXPECT_NEAR(short_buffer.target_pose.pose.position.x,
      with_rgb.target_pose.pose.position.x, 1e-9);
    EXPECT_NEAR(short_buffer.target_pose.pose.position.y,
      with_rgb.target_pose.pose.position.y, 1e-9);
    EXPECT_TRUE(short_buffer.source_rgb_header.frame_id.empty());
  }
}


TEST(RawPartDetectTarget, RecordsProfileResolvedByTheAcquisitionTargetId)
{
  auto input = rgbd_input(true);
  auto distractor = rawpart_profile();
  distractor.target_id = "OtherTarget";
  distractor.cuboid.grid_resolution_m = 0.009;
  auto selected = rawpart_profile();
  selected.target_id = "RawPart";
  selected.cuboid.grid_resolution_m = 0.0031;
  arm_cell_vision::DetectTargetProcessor processor;
  install_fixture_calibration(processor);
  const auto result = processor.process(input, "RawPart", {distractor, selected});
  ASSERT_TRUE(result.applied_profile);
  EXPECT_EQ(result.applied_profile->target_id, "RawPart");
  EXPECT_DOUBLE_EQ(result.applied_profile->cuboid.grid_resolution_m, 0.0031);
}


TEST(RawPartDetectTarget, ObservedSupportSelectsCompetingRecipeFacesWithoutConfidenceDowngrade)
{
  for (const float depth : {0.92F, 0.93F}) {
    auto input = rgbd_input(true);
    set_rotated_rectangle(input, 0.0, 86, 76, depth);
    const auto result = process_with_fixture_calibration(input, "RawPart", {rawpart_profile()});
    ASSERT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS);
    EXPECT_FALSE(result.ordered_candidates.empty());
    const auto selected = std::find_if(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
      [](const auto & h) {return h.selection_state == "SELECTED" && h.disposition != "removed_by_deduplication";});
    const auto alternative = std::find_if(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
      [](const auto & h) {return h.selection_state == "ALTERNATIVE" && h.disposition != "removed_by_deduplication";});
    ASSERT_NE(selected, result.cuboid_hypotheses.end());
    ASSERT_NE(alternative, result.cuboid_hypotheses.end());
    EXPECT_GT(selected->observed_support_cells, alternative->observed_support_cells);
    EXPECT_NE(alternative->confidence_grade, "LOW");
    for (const auto & candidate : result.ordered_candidates) {
      EXPECT_NE(candidate.rawpart_selected_candidate, std::to_string(alternative->candidate_id));
    }
  }
}

TEST(RawPartDetectTarget, ReportsMeasuredRectangleSidesAndConsistentTwofoldPose)
{
  tf2::Quaternion initial_orientation;
  double initial_observed_yaw = 0.0;
  bool initial_set = false;
  for (const double degrees : {0.0, 15.0, 30.0, 45.0}) {
    auto input = rgbd_input(true);
    set_rotated_rectangle(input, degrees * CV_PI / 180.0, 86, 76);
    auto profile = rawpart_profile();
    // Isolate Pose transport from overlapping Recipe-face tolerances.
    profile.dimension_tolerance_m = {0.0035, 0.0035, 0.0035};
    const auto result = process_with_fixture_calibration(input, "RawPart", {profile});
    ASSERT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
      << degrees << " deg: " << result.diagnostic_detail;
    ASSERT_TRUE(result.ordered_candidates.at(0).has_observed_rectangle);
    EXPECT_EQ(result.ordered_candidates.at(0).orientation_symmetry_order, 2U);
    EXPECT_GT(result.ordered_candidates.at(0).rectangle_long_side_m, result.ordered_candidates.at(0).rectangle_short_side_m);
    EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_long_side_m, 0.08, 0.012);
    EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_short_side_m, 0.07, 0.012);
    tf2::Quaternion orientation(
      result.ordered_candidates.at(0).target_pose.pose.orientation.x, result.ordered_candidates.at(0).target_pose.pose.orientation.y,
      result.ordered_candidates.at(0).target_pose.pose.orientation.z, result.ordered_candidates.at(0).target_pose.pose.orientation.w);
    orientation.normalize();
    if (!initial_set) {
      initial_orientation = orientation;
      initial_observed_yaw = result.ordered_candidates.at(0).observed_yaw_rad;
      initial_set = true;
    }
    auto relative = initial_orientation.inverse() * orientation;
    relative.normalize();
    const auto relative_yaw = 2.0 * std::atan2(relative.z(), relative.w());
    EXPECT_LT(std::abs(std::remainder(
        relative_yaw - (result.ordered_candidates.at(0).observed_yaw_rad - initial_observed_yaw), CV_PI)), 0.02);
    EXPECT_TRUE(result.ordered_candidates.at(0).has_estimated_height);
    EXPECT_NEAR(result.ordered_candidates.at(0).estimated_height_m, 0.07, 0.0035);
  }
}

TEST(RawPartDetectTarget, SideLyingRawPartReportsMeasuredTopAndHeightAsRectangle)
{
  for (const double degrees : {0.0, 15.0, 30.0, 45.0}) {
    auto input = rgbd_input(true);
    set_rotated_rectangle(input, degrees * CV_PI / 180.0, 86, 76, 0.93F);
    auto profile = rawpart_profile();
    // Isolate Pose transport from overlapping Recipe-face tolerances.
    profile.dimension_tolerance_m = {0.0035, 0.0035, 0.0035};
    const auto result = process_with_fixture_calibration(input, "RawPart", {profile});
    ASSERT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
      << degrees << " deg: " << result.diagnostic_detail;
    ASSERT_TRUE(result.ordered_candidates.at(0).has_observed_rectangle);
    EXPECT_EQ(result.ordered_candidates.at(0).orientation_symmetry_order, 2U);
    EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_long_side_m, 0.08, 0.012);
    EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_short_side_m, 0.07, 0.012);
    ASSERT_TRUE(result.ordered_candidates.at(0).has_estimated_height);
    EXPECT_NEAR(result.ordered_candidates.at(0).estimated_height_m, 0.07, 0.01);
    tf2::Quaternion orientation(
      result.ordered_candidates.at(0).target_pose.pose.orientation.x, result.ordered_candidates.at(0).target_pose.pose.orientation.y,
      result.ordered_candidates.at(0).target_pose.pose.orientation.z, result.ordered_candidates.at(0).target_pose.pose.orientation.w);
    EXPECT_NEAR(orientation.length(), 1.0, 1e-6);
  }
}

TEST(RawPartDetectTarget, FitsWithOrdinaryDepthNoiseAndSparseBoundaryOutlier)
{
  for (const double degrees : {0.0, 15.0, 30.0, 45.0}) {
    auto input = rgbd_input(true);
    set_rotated_square(input, degrees * CV_PI / 180.0);
    for (int y = 112; y < 188; ++y) {
      for (int x = 122; x < 198; ++x) {
        const auto offset = (static_cast<size_t>(y) * kWidth + x) * sizeof(float);
        float depth = 0.0F;
        std::memcpy(&depth, input.depth.data.data() + offset, sizeof(depth));
        if (depth < 0.95F) {
          const auto jitter_mm = ((x * 17 + y * 31) % 5) - 2;
          set_depth_pixel(input, x, y, depth + static_cast<float>(jitter_mm) * 0.001F);
        }
      }
    }
    set_depth_pixel(input, 158, 112, 0.921F);
    const auto result = process_with_fixture_calibration(
      input, "RawPart", {rawpart_profile()});
    ASSERT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
      << degrees << " deg: " << result.diagnostic_detail;
    const auto match = std::find_if(
      result.rawpart_candidate_diagnostics.begin(), result.rawpart_candidate_diagnostics.end(),
      [](const auto & candidate) {return candidate.profile_match;});
    ASSERT_NE(match, result.rawpart_candidate_diagnostics.end());
    EXPECT_EQ(match->orientation_symmetry_order, 4U) << degrees << " deg";
  }
}




TEST(RawPartCalibration, RuntimeDoesNotCreateCalibrationFromObservedFrames)
{
  auto input = rgbd_input(true);
  arm_cell_vision::DetectTargetProcessor processor;
  const auto result = processor.process(input, "RawPart", {rawpart_profile()});
  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_EQ(result.processing_stage, "calibration_readiness");
  EXPECT_FALSE(processor.support_calibration_snapshot().ready);
  EXPECT_EQ(processor.support_calibration_snapshot().accepted_frames, 0U);
}

TEST(RawPartCalibration, ExplicitRequestUsesOneObservationAndDoesNotRunDetection)
{
  auto input = rgbd_input(true);
  arm_cell_vision::DetectTargetProcessor calibrator;
  const auto result = calibrator.process(input, "RawPart", {rawpart_profile()}, true);
  const auto calibration = calibrator.support_calibration_snapshot();
  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_FALSE(result.has_target_pose);
  EXPECT_TRUE(calibration.ready);
  EXPECT_EQ(calibration.accepted_frames, 1U);
  EXPECT_EQ(calibration.required_frames, 1U);
  ASSERT_EQ(calibration.recent_depth_validations.size(), 1U);
  EXPECT_TRUE(calibration.recent_depth_validations.back().accepted);
}

TEST(RawPartCalibration, ExplicitFiducialCalibrationStillRequiresRgb)
{
  auto input = rgbd_input(true);
  input.rgb = sensor_msgs::msg::Image{};
  arm_cell_vision::DetectTargetProcessor calibrator;

  const auto result = calibrator.process(input, "RawPart", {rawpart_profile()}, true);

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SENSOR_ERROR);
  EXPECT_EQ(result.processing_stage, "observation_validation");
  EXPECT_FALSE(calibrator.support_calibration_snapshot().ready);
}

TEST(RawPartCalibration, ExplicitCalibrationDoesNotRequireAnOpticalToBaseTransform)
{
  auto input = rgbd_input(true);
  input.optical_to_base = geometry_msgs::msg::TransformStamped{};
  arm_cell_vision::DetectTargetProcessor calibrator;

  const auto result = calibrator.process(
    input, "RawPart", {rawpart_profile()}, true);

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_TRUE(calibrator.support_calibration_snapshot().ready);
}

TEST(RawPartCalibration, LoadedTransformSupportsRuntimeWithoutMarkerObservation)
{
  arm_cell_vision::DetectTargetProcessor processor;
  install_fixture_calibration(processor);
  auto input = rgbd_input(false);
  const auto result = processor.process(input, "RawPart", {rawpart_profile()});
  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  EXPECT_FALSE(result.ordered_candidates.empty());
  EXPECT_TRUE(processor.support_calibration_snapshot().ready);
}

TEST(RawPartCalibration, FailedExplicitRequestLeavesLoadedRuntimeTransformUsable)
{
  arm_cell_vision::DetectTargetProcessor runtime;
  install_fixture_calibration(runtime);
  auto missing_marker = rgbd_input(false);
  arm_cell_vision::DetectTargetProcessor calibrator;
  const auto failed = calibrator.process(missing_marker, "RawPart", {rawpart_profile()}, true);
  EXPECT_EQ(failed.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_FALSE(calibrator.support_calibration_snapshot().ready);
  EXPECT_TRUE(runtime.support_calibration_snapshot().ready);
  auto runtime_input = rgbd_input(false);
  EXPECT_EQ(runtime.process(runtime_input, "RawPart", {rawpart_profile()}).result_code,
    ResultCode::DETECT_RESULT_SUCCESS);
}



TEST(RawPartCalibration, RuntimeDoesNotDependOnArUcoDictionaryConfiguration)
{
  auto profile = rawpart_profile();
  profile.fiducial_dictionary = "DICT_NOT_SUPPORTED";
  const auto runtime = process_with_fixture_calibration(
    rgbd_input(false), "RawPart", {profile});
  EXPECT_EQ(runtime.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << runtime.diagnostic_detail;

  arm_cell_vision::DetectTargetProcessor calibrator;
  const auto request = calibrator.process(
    rgbd_input(true), "RawPart", {profile}, true);
  EXPECT_EQ(request.result_code, ResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_FALSE(calibrator.support_calibration_snapshot().ready);
}

TEST(RawPartCalibration, ExplicitRequestUsesConfiguredFiducialDictionary)
{
  auto profile = rawpart_profile();
  profile.fiducial_dictionary = "DICT_5X5_50";
  arm_cell_vision::DetectTargetProcessor calibrator;
  (void)calibrator.process(rgbd_input(true, profile.fiducial_dictionary),
    "RawPart", {profile}, true);
  EXPECT_TRUE(calibrator.support_calibration_snapshot().ready);
}

TEST(RawPartDetectTarget, ProcessingStageIsIndependentOfDiagnosticText)
{
  auto input = rgbd_input(true);
  input.camera_info.k[0] = 0.0;
  auto result = process_with_fixture_calibration(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SENSOR_ERROR);
  EXPECT_EQ(result.processing_stage, "observation_validation");
  EXPECT_TRUE(result.disturbance_candidates.empty());
  result.diagnostic_detail = "camera model input rejected by a renamed validation message";
  EXPECT_EQ(result.processing_stage, "observation_validation");
}

TEST(RawPartDetectTarget, UnresolvedOutlineMeanUsesObservationTimeBaseLinkFrame)
{
  auto input = rgbd_input(false);
  clear_candidate(input, 122, 112, 76, 76);
  clear_candidate(input, 72, 168, 16, 16);
  set_candidate(input, 151, 141, 19, 19, 0.97F);
  input.detailed_diagnostics_enabled = true;
  input.optical_to_base.transform.translation.x = 0.2;
  input.optical_to_base.transform.translation.y = -0.1;
  input.optical_to_base.transform.translation.z = 0.3;
  tf2::Quaternion rotation; rotation.setRPY(0.1, -0.2, 0.3);
  input.optical_to_base.transform.rotation.x = rotation.x();
  input.optical_to_base.transform.rotation.y = rotation.y();
  input.optical_to_base.transform.rotation.z = rotation.z();
  input.optical_to_base.transform.rotation.w = rotation.w();

  const auto result = process_with_fixture_calibration(input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  const auto & disturbance = result.disturbance_candidates.front();
  EXPECT_EQ(disturbance.center.header.frame_id, "base_link");
  EXPECT_EQ(disturbance.center.header.stamp, input.depth.header.stamp);
  EXPECT_TRUE(std::isfinite(disturbance.center.pose.position.x));
  EXPECT_TRUE(std::isfinite(disturbance.center.pose.position.y));
  EXPECT_TRUE(std::isfinite(disturbance.center.pose.position.z));
  EXPECT_TRUE(std::isfinite(disturbance.observed_height_m));
  EXPECT_EQ(
    disturbance.category,
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED);
  auto camera_input = input;
  camera_input.optical_to_base.transform.translation.x = 0;
  camera_input.optical_to_base.transform.translation.y = 0;
  camera_input.optical_to_base.transform.translation.z = 0;
  camera_input.optical_to_base.transform.rotation.x = 0;
  camera_input.optical_to_base.transform.rotation.y = 0;
  camera_input.optical_to_base.transform.rotation.z = 0;
  camera_input.optical_to_base.transform.rotation.w = 1;
  const auto reference = process_with_fixture_calibration(camera_input, "RawPart", {rawpart_profile()});
  ASSERT_EQ(reference.disturbance_candidates.size(), 1U);
  const auto & p = reference.disturbance_candidates[0].center.pose.position;
  const tf2::Transform base_T_camera(rotation, {0.2, -0.1, 0.3});
  const auto expected = base_T_camera * tf2::Vector3(p.x, p.y, p.z);
  EXPECT_NEAR(disturbance.center.pose.position.x, expected.x(), 1e-12);
  EXPECT_NEAR(disturbance.center.pose.position.y, expected.y(), 1e-12);
  EXPECT_NEAR(disturbance.center.pose.position.z, expected.z(), 1e-12);
  EXPECT_DOUBLE_EQ(disturbance.observed_height_m, reference.disturbance_candidates[0].observed_height_m);

}







TEST(RawPartDetectTarget, ValidCandidateCanBeSelectedWithLowWholeWorkspaceCoverage)
{
  auto input = rgbd_input(true);
  std::fill(input.depth.data.begin(), input.depth.data.end(), 0U);
  set_candidate(input, 55, 235, 41, 41, 1.0F);
  set_candidate(input, 122, 112, 76, 76, 0.92F);

  auto profile = rawpart_profile();
  // This foreground-only observation tests loaded calibration/coverage, not
  // ambiguous 70/80 mm face selection. A cube has one Recipe face class.
  profile.dimensions_m = {0.07, 0.07, 0.07};
  const auto result = process_with_fixture_calibration(input, "RawPart", {profile});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << result.diagnostic_detail;
  EXPECT_TRUE(result.ordered_candidates.at(0).profile_candidate_valid);
}

TEST(RawPartDetectTarget, UnobservableWorkspaceAndInsufficientDepthAreNotDepletion)
{
  auto profile = rawpart_profile();
  profile.algorithm = "cuboid_top_face";

  auto clipped_workspace = rgbd_input(true);
  std::fill(clipped_workspace.depth.data.begin(), clipped_workspace.depth.data.end(), 0U);
  for (int y = 230; y < 282; ++y) {
    for (int x = 50; x < 102; ++x) {
      set_depth_pixel(clipped_workspace, x, y, 1.0F);
    }
  }
  set_candidate(clipped_workspace, 0, 105, 14, 76, 0.92F);
  profile.workspace_roi_min_m = {-0.5, -0.5, 0.01};
  profile.workspace_roi_max_m = {0.5, 0.5, 0.12};
  const auto clipped_result = process_with_fixture_calibration(
    clipped_workspace, "RawPart", {profile});
  EXPECT_NE(clipped_result.result_code, ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
  if (clipped_result.has_target_pose) {
    EXPECT_FALSE(clipped_result.ordered_candidates.at(0).has_observed_rectangle);
  }

  auto insufficient_depth = rgbd_input(true);
  std::fill(insufficient_depth.depth.data.begin(), insufficient_depth.depth.data.end(), 0U);
  for (int y = 235; y < 276; ++y) {
    for (int x = 55; x < 96; ++x) {
      set_depth_pixel(insufficient_depth, x, y, 1.0F);
    }
  }
  profile = rawpart_profile();
  profile.algorithm = "cuboid_top_face";
  const auto insufficient_result = process_with_fixture_calibration(
    insufficient_depth, "RawPart", {profile});
  EXPECT_EQ(insufficient_result.result_code, ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_FALSE(insufficient_result.has_target_pose);
}

TEST(RawPartDetectTarget, UniqueRoiAndCameraClippedGeometryIsMedium)
{
  auto outside_workspace = rgbd_input(true);
  clear_candidate(outside_workspace, 122, 112, 76, 76);
  clear_candidate(outside_workspace, 72, 168, 16, 16);
  set_candidate(outside_workspace, 122, 112, 76, 76, 0.92F);
  auto profile = rawpart_profile();
  profile.algorithm = "cuboid_top_face";
  profile.dimensions_m = {0.07, 0.07, 0.07};
  profile.workspace_roi_max_m[0] = 0.025;
  const auto outside_result = process_with_fixture_calibration(
    outside_workspace, "RawPart", {profile});
  ASSERT_EQ(outside_result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << outside_result.diagnostic_detail;
  EXPECT_LE(
    outside_result.ordered_candidates.at(0).target_pose.pose.position.x, profile.workspace_roi_max_m[0]);
  EXPECT_FALSE(outside_result.ordered_candidates.at(0).has_observed_rectangle);

  auto clipped_at_image_edge = rgbd_input(true);
  clear_candidate(clipped_at_image_edge, 122, 112, 76, 76);
  clear_candidate(clipped_at_image_edge, 72, 168, 16, 16);
  set_candidate(clipped_at_image_edge, 0, 112, 40, 76, 0.92F);
  profile = rawpart_profile();
  profile.algorithm = "cuboid_top_face";
  profile.dimensions_m = {0.07, 0.07, 0.07};
  profile.workspace_roi_min_m[0] = -0.20;
  profile.tray_roi_min_m = {-0.08, -0.08};
  profile.tray_roi_max_m = {0.08, 0.08};
  const auto clipped_result = process_with_fixture_calibration(
    clipped_at_image_edge, "RawPart", {profile});
  ASSERT_EQ(clipped_result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << clipped_result.diagnostic_detail;
  EXPECT_FALSE(clipped_result.ordered_candidates.empty());
  EXPECT_EQ(clipped_result.ordered_candidates.at(0).confidence_grade, "MEDIUM");
  EXPECT_EQ(outside_result.ordered_candidates.at(0).confidence_grade, "MEDIUM");
  EXPECT_FALSE(clipped_result.ordered_candidates.at(0).has_observed_rectangle);
}

TEST(RawPartDetectTarget, CuboidGeometryFailureIsNotRewrittenAsAbsence)
{
  auto input = rgbd_input(true);
  const auto profile = [] {
      auto value = rawpart_profile();
      value.algorithm = "cuboid_top_face";
      value.cuboid.grid_resolution_m = 0.0;
      return value;
    }();
  const auto result = process_with_fixture_calibration(input, "RawPart", {profile});
  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_FALSE(result.rawpart_absence_confirmed);
  EXPECT_EQ(result.failure_cause, "Cuboid required geometry/occupancy failed");
  EXPECT_NE(result.diagnostic_detail.find("Cuboid required geometry/occupancy failed"),
    std::string::npos);
}

TEST(RawPartDetectTarget, RuntimeDoesNotRepeatSupportDepthCalibrationCheck)
{
  auto input = rgbd_input(true);
  std::vector<float> depth(kWidth * kHeight, 0.0F);
  for (int row = 112; row < 188; ++row) {
    for (int column = 122; column < 198; ++column) {
      depth[static_cast<size_t>(row) * kWidth + column] = 0.92F;
    }
  }
  for (int row = 168; row < 184; ++row) {
    for (int column = 72; column < 88; ++column) {
      depth[static_cast<size_t>(row) * kWidth + column] = 0.90F;
    }
  }
  std::memcpy(input.depth.data.data(), depth.data(), input.depth.data.size());
  auto profile = rawpart_profile();
  // This foreground-only observation tests loaded calibration/coverage, not
  // ambiguous 70/80 mm face selection. A cube has one Recipe face class.
  profile.dimensions_m = {0.07, 0.07, 0.07};
  const auto result = process_with_fixture_calibration(input, "RawPart", {profile});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  EXPECT_FALSE(result.ordered_candidates.empty());
}

TEST(RawPartDetectTarget, RuntimeUsesSelectedTransformDespiteSupportDepthChange)
{
  auto input = rgbd_input(true);
  std::vector<float> depth(kWidth * kHeight, 0.96F);
  for (int row = 112; row < 188; ++row) {
    for (int column = 122; column < 198; ++column) {
      depth[static_cast<size_t>(row) * kWidth + column] = 0.92F;
    }
  }
  for (int row = 168; row < 184; ++row) {
    for (int column = 72; column < 88; ++column) {
      depth[static_cast<size_t>(row) * kWidth + column] = 0.90F;
    }
  }
  std::memcpy(input.depth.data.data(), depth.data(), input.depth.data.size());
  const auto result = process_with_fixture_calibration(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  EXPECT_FALSE(result.ordered_candidates.empty());
}

TEST(RawPartDetectTarget, UnknownTargetIdDoesNotUseAnotherConfiguredProfile)
{
  auto input = rgbd_input(true);
  const auto result = process_with_fixture_calibration(
    input, "rawpart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_FALSE(result.has_target_pose);
}

}  // namespace
