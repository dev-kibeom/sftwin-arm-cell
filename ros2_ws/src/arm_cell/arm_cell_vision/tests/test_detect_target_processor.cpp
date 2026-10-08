#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "arm_cell_vision/detect_target_processor.hpp"

namespace
{

sensor_msgs::msg::CameraInfo camera_info()
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera_color_optical_frame";
  info.width = 8;
  info.height = 6;
  info.distortion_model = "plumb_bob";
  info.k[0] = 100.0;
  info.k[4] = 100.0;
  info.k[2] = 3.5;
  info.k[5] = 2.5;
  info.p[0] = 100.0;
  info.p[5] = 100.0;
  info.p[2] = 3.5;
  info.p[6] = 2.5;
  return info;
}

sensor_msgs::msg::Image depth_image(const std::vector<float> & values)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera_color_optical_frame";
  image.width = 8;
  image.height = 6;
  image.encoding = "32FC1";
  image.is_bigendian = false;
  image.step = image.width * sizeof(float);
  image.data.resize(values.size() * sizeof(float));
  std::memcpy(image.data.data(), values.data(), image.data.size());
  return image;
}

sensor_msgs::msg::Image rgb_image()
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera_color_optical_frame";
  image.width = 8;
  image.height = 6;
  image.encoding = "rgb8";
  image.step = image.width * 3;
  image.data.resize(image.step * image.height, 100);
  return image;
}

geometry_msgs::msg::TransformStamped identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "camera_color_optical_frame";
  transform.transform.rotation.w = 1.0;
  return transform;
}

arm_cell_vision::DetectionInput input_with_depth(const std::vector<float> & values)
{
  return {rgb_image(), depth_image(values), camera_info(), identity_transform()};
}

std::vector<float> background_depth()
{
  return std::vector<float>(8 * 6, 1.0F);
}

TEST(DetectTargetProcessor, FindsObjectAndReturnsObjectCentricBasePose)
{
  auto values = background_depth();
  for (size_t row = 2; row <= 3; ++row) {
    for (size_t column = 2; column <= 5; ++column) {
      values[row * 8 + column] = 0.9F;
    }
  }

  const auto result = arm_cell_vision::DetectTargetProcessor().process(input_with_depth(values));

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  ASSERT_TRUE(result.has_target_pose);
  EXPECT_TRUE(result.has_target_yaw);
  EXPECT_EQ(result.target_pose.header.frame_id, "base_link");
  EXPECT_NEAR(result.target_pose.pose.position.z, 0.9, 1e-5);
  ASSERT_TRUE(result.has_estimated_height);
  EXPECT_NEAR(result.estimated_height_m, 0.1, 1e-5);
}

TEST(DetectTargetProcessor, DistinguishesNormalDepletionFromSensorFailure)
{
  const auto result = arm_cell_vision::DetectTargetProcessor().process(
    input_with_depth(background_depth()));

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
  EXPECT_FALSE(result.has_target_pose);
}

TEST(DetectTargetProcessor, RejectsMalformedDepthAsSensorError)
{
  auto input = input_with_depth(background_depth());
  input.depth.data.resize(3);

  const auto result = arm_cell_vision::DetectTargetProcessor().process(input);

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SENSOR_ERROR);
}

TEST(DetectTargetProcessor, RejectsMissingOpticalToBaseTransform)
{
  auto input = input_with_depth(background_depth());
  input.optical_to_base.header.frame_id.clear();

  const auto result = arm_cell_vision::DetectTargetProcessor().process(input);

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TF_ERROR);
}

TEST(DetectTargetProcessor, RejectsInsufficientObjectGeometry)
{
  auto values = background_depth();
  values[3 * 8 + 4] = 0.9F;

  const auto result = arm_cell_vision::DetectTargetProcessor().process(input_with_depth(values));

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_GEOMETRY_ERROR);
}

TEST(DetectTargetProcessor, PreservesPositionWhenYawIsUnobservable)
{
  auto values = background_depth();
  for (size_t row = 2; row <= 3; ++row) {
    for (size_t column = 3; column <= 4; ++column) {
      values[row * 8 + column] = 0.9F;
    }
  }

  const auto result = arm_cell_vision::DetectTargetProcessor().process(input_with_depth(values));

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  EXPECT_TRUE(result.has_target_pose);
  EXPECT_FALSE(result.has_target_yaw);
  EXPECT_NEAR(result.target_pose.pose.position.z, 0.9, 1e-5);
}

}  // namespace
