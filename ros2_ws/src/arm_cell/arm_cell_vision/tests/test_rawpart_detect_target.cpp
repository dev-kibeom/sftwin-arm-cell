#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

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

TEST(RawPartDetectTarget, ResolvesConfiguredTargetFromSupportRelativeDepthGeometry)
{
  auto input = rgbd_input(true);
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << result.diagnostic_detail;
  ASSERT_TRUE(result.has_target_pose);
  EXPECT_FALSE(result.has_target_yaw);
  EXPECT_NEAR(result.target_pose.pose.position.x, 0.0, 0.003);
  EXPECT_NEAR(result.target_pose.pose.position.y, 0.0, 0.003);
  EXPECT_NEAR(result.target_pose.pose.position.z, 0.92, 0.003);
  ASSERT_TRUE(result.has_estimated_height);
  EXPECT_NEAR(result.estimated_height_m, 0.08, 0.003);
  EXPECT_EQ(result.source_rgb_header.stamp.sec, 42);
  EXPECT_EQ(result.source_rgb_header.stamp.nanosec, 1234U);
  EXPECT_EQ(result.source_rgb_header.frame_id, "camera_color_optical_frame");
  ASSERT_TRUE(result.object_region_valid);
  EXPECT_EQ(result.object_region_x, 122U);
  EXPECT_EQ(result.object_region_y, 112U);
  EXPECT_EQ(result.object_region_width, 76U);
  EXPECT_EQ(result.object_region_height, 76U);
  ASSERT_TRUE(result.support_region_valid);
  EXPECT_EQ(result.support_region.size(), 4U);
  EXPECT_TRUE(result.centroid_pixel_valid);
  EXPECT_FALSE(result.has_target_yaw);
}

TEST(RawPartDetectTarget, AnchorsMarkerTranslationToDepthWhenImageScaleDiffers)
{
  auto input = rgbd_input(true, "DICT_4X4_50", 38);
  auto profile = rawpart_profile();
  profile.fiducial_pose_translation_scale = 0.925;
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {profile});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << result.diagnostic_detail;
  EXPECT_TRUE(result.has_target_pose);
  EXPECT_NEAR(result.target_pose.pose.position.z, 0.92, 0.005);
}

TEST(RawPartDetectTarget, UsesConfiguredDictionaryForSupportFiducialDetection)
{
  auto profile = rawpart_profile();
  profile.fiducial_dictionary = "DICT_5X5_50";
  auto input = rgbd_input(true, profile.fiducial_dictionary);
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {profile});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_SUCCESS)
    << result.diagnostic_detail;
  EXPECT_TRUE(result.has_target_pose);
}

TEST(RawPartDetectTarget, UnsupportedConfiguredDictionaryIsInvalidResult)
{
  auto profile = rawpart_profile();
  profile.fiducial_dictionary = "DICT_NOT_SUPPORTED";
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    rgbd_input(true), "RawPart", {profile});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_FALSE(result.has_target_pose);
}

TEST(RawPartDetectTarget, MissingSupportFiducialIsFailureRatherThanDepletion)
{
  auto input = rgbd_input(false);
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_NE(
    result.result_code, ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
  EXPECT_FALSE(result.has_target_pose);
}

TEST(RawPartDetectTarget, ValidSupportWithNoMatchingDepthObjectReportsDepletion)
{
  auto input = rgbd_input(true);
  for (int row = 112; row < 188; ++row) {
    for (int column = 122; column < 198; ++column) {
      const auto offset =
        (static_cast<size_t>(row) * kWidth + column) * sizeof(float);
      const float support_depth = 1.0F;
      std::memcpy(input.depth.data.data() + offset, &support_depth, sizeof(support_depth));
    }
  }
  for (int row = 168; row < 184; ++row) {
    for (int column = 72; column < 88; ++column) {
      const auto offset =
        (static_cast<size_t>(row) * kWidth + column) * sizeof(float);
      const float support_depth = 1.0F;
      std::memcpy(input.depth.data.data() + offset, &support_depth, sizeof(support_depth));
    }
  }
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
  EXPECT_FALSE(result.has_target_pose);
}

TEST(RawPartDetectTarget, MissingSupportDepthIsGeometryFailureEvenWithTargetDepth)
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
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_NE(result.result_code, ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
  EXPECT_FALSE(result.has_target_pose);
}

TEST(RawPartDetectTarget, InconsistentSupportPlaneDepthIsGeometryFailure)
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
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "RawPart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_FALSE(result.has_target_pose);
}

TEST(RawPartDetectTarget, UnknownTargetIdDoesNotUseAnotherConfiguredProfile)
{
  auto input = rgbd_input(true);
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input, "rawpart", {rawpart_profile()});

  EXPECT_EQ(result.result_code, ResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_FALSE(result.has_target_pose);
}

}  // namespace
