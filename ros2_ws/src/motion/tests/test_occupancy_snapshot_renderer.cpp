#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "arm_cell_motion/occupancy_snapshot_renderer.hpp"
#include "arm_cell_motion/planning_scene_builder.hpp"

namespace arm_cell_motion
{
namespace
{
using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;

Primitive occupancy_box(double x, double y, double z)
{
  Primitive box;
  box.occupancy_class = Primitive::OBSERVED_OCCUPANCY;
  box.shape_type = Primitive::SOLID_PRIMITIVE;
  box.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.shape.dimensions = {0.10, 0.20, 0.30};
  box.pose.position.x = x;
  box.pose.position.y = y;
  box.pose.position.z = z;
  box.pose.orientation.z = std::sin(0.3);
  box.pose.orientation.w = std::cos(0.3);
  return box;
}

cv::Mat decoded_png(const OccupancySnapshot & snapshot)
{
  return cv::imdecode(snapshot.png, cv::IMREAD_COLOR);
}

TEST(OccupancySnapshotRenderer, RendersPlanningOccupancyObjectsInXYAndXZ)
{
  const std::vector<Primitive> planned_occupancy{
    occupancy_box(0.42, 0.18, 0.31), occupancy_box(0.51, -0.12, 0.27)};
  const auto image = render_occupancy_snapshot(planned_occupancy, "acq-17", "candidate-2");

  ASSERT_EQ(image.status, "rendered");
  ASSERT_FALSE(image.png.empty());
  const auto decoded = decoded_png(image);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, 1470);
  EXPECT_EQ(decoded.rows, 470);

  // The scene builder and renderer consume the same Motion primitive list.
  moveit_msgs::msg::PlanningScene fixed_scene;
  PlanningSceneBuilder builder(fixed_scene, planned_occupancy, std_msgs::msg::Header{});
  PlanningScenePolicy include_occupancy;
  include_occupancy.vision_occupancy = VisionOccupancyPolicy::INCLUDE;
  const auto planning_map = builder.build(include_occupancy);
  ASSERT_TRUE(planning_map.valid);
  ASSERT_EQ(planning_map.candidate_ids.size(), planned_occupancy.size());
  ASSERT_EQ(planning_map.diff.world.collision_objects.size(), planned_occupancy.size());
  for (std::size_t index = 0; index < planned_occupancy.size(); ++index) {
    const auto & object = planning_map.diff.world.collision_objects[index];
    ASSERT_EQ(object.id, planning_map.candidate_ids[index]);
    ASSERT_EQ(object.primitives.size(), 1U);
    EXPECT_EQ(object.primitives.front().dimensions, planned_occupancy[index].shape.dimensions);
    EXPECT_DOUBLE_EQ(object.primitive_poses.front().position.x,
      planned_occupancy[index].pose.position.x);
    EXPECT_DOUBLE_EQ(object.primitive_poses.front().position.y,
      planned_occupancy[index].pose.position.y);
    EXPECT_DOUBLE_EQ(object.primitive_poses.front().position.z,
      planned_occupancy[index].pose.position.z);
    EXPECT_DOUBLE_EQ(object.primitive_poses.front().orientation.z,
      planned_occupancy[index].pose.orientation.z);
    EXPECT_DOUBLE_EQ(object.primitive_poses.front().orientation.w,
      planned_occupancy[index].pose.orientation.w);
  }

  // Orange occupancy pixels in both panels prove the PNG contains rendered geometry.
  std::size_t orange_pixels = 0U;
  for (int row = 0; row < decoded.rows; ++row) {
    for (int column = 0; column < decoded.cols; ++column) {
      const auto pixel = decoded.at<cv::Vec3b>(row, column);
      if (pixel[2] > 180 && pixel[1] > 70 && pixel[1] < 190 && pixel[0] < 80) {
        ++orange_pixels;
      }
    }
  }
  EXPECT_GT(orange_pixels, 100U);

  const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto saved_path = std::filesystem::temp_directory_path() /
    ("arm_cell_occupancy_snapshot_" + std::to_string(unique)) / "attempt.png";
  ASSERT_NO_THROW(save_occupancy_snapshot_png(saved_path, image));
  EXPECT_TRUE(std::filesystem::is_regular_file(saved_path));
  const auto saved = cv::imread(saved_path.string(), cv::IMREAD_COLOR);
  EXPECT_FALSE(saved.empty());
  std::filesystem::remove_all(saved_path.parent_path());
}

TEST(OccupancySnapshotRenderer, ProducesAnExplicitEmptyMapPng)
{
  const auto image = render_occupancy_snapshot({}, "acq-empty", "candidate-empty");
  EXPECT_EQ(image.status, "empty_map_rendered");
  ASSERT_FALSE(image.png.empty());
  const auto decoded = decoded_png(image);
  ASSERT_FALSE(decoded.empty());
  EXPECT_EQ(decoded.cols, 1470);
  EXPECT_EQ(decoded.rows, 470);

  const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto saved_path = std::filesystem::temp_directory_path() /
    ("arm_cell_occupancy_snapshot_empty_" + std::to_string(unique) + ".png");
  ASSERT_NO_THROW(save_occupancy_snapshot_png(saved_path, image));
  EXPECT_FALSE(cv::imread(saved_path.string(), cv::IMREAD_COLOR).empty());
  std::filesystem::remove(saved_path);
}
}  // namespace
}  // namespace arm_cell_motion
