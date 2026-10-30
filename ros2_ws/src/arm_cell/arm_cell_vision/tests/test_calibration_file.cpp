#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "arm_cell_vision/calibration_file.hpp"

namespace
{

class CalibrationFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() / "arm_cell_calibration_file_test";
    std::filesystem::remove_all(root_);
    std::filesystem::create_directories(root_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  void write_valid_file(const std::filesystem::path & path)
  {
    std::ofstream file(path);
    file << "%YAML:1.0\n"
         << "tray_T_camera: { translation: [1.0, 2.0, 3.0], quaternion_xyzw: [0.0, 0.0, 0.0, 1.0] }\n"
         << "reference_frame: Tray\n"
         << "camera_optical_frame: camera_color_optical_frame\n"
         << "camera_info_topic: /camera/color/camera_info\n"
         << "camera_info: { width: 640, height: 480, distortion_model: plumb_bob, "
         << "k: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0], "
         << "d: [0.0, 0.0, 0.0, 0.0, 0.0], "
         << "r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], "
         << "p: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0] }\n";
  }

  std::filesystem::path root_;
};

TEST_F(CalibrationFileTest, MissingSelectedFileFailsWithCause)
{
  arm_cell_vision::CalibrationFileData output;
  std::string diagnostic;
  EXPECT_FALSE(arm_cell_vision::CalibrationFile::load(
      (root_ / "missing.yaml").string(), "/camera/color/camera_info", output, diagnostic));
  EXPECT_FALSE(diagnostic.empty());
}

TEST_F(CalibrationFileTest, MalformedOrNonRigidTransformFileFails)
{
  const auto path = root_ / "invalid.yaml";
  std::ofstream(path) << "%YAML:1.0\ntray_T_camera: { translation: [0.0, 0.0], quaternion_xyzw: [0.0, 0.0, 0.0, 0.0] }\n";
  arm_cell_vision::CalibrationFileData output;
  std::string diagnostic;
  EXPECT_FALSE(arm_cell_vision::CalibrationFile::load(
      path.string(), "/camera/color/camera_info", output, diagnostic));
  EXPECT_FALSE(diagnostic.empty());
}

TEST_F(CalibrationFileTest, LoadedFileMustMatchCameraInfoAndFrame)
{
  const auto path = root_ / "selected.yaml";
  write_valid_file(path);
  arm_cell_vision::CalibrationFileData calibration;
  std::string diagnostic;
  ASSERT_TRUE(arm_cell_vision::CalibrationFile::load(
      path.string(), "/camera/color/camera_info", calibration, diagnostic)) << diagnostic;

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.header.frame_id = "camera_color_optical_frame";
  camera_info.distortion_model = "plumb_bob";
  camera_info.k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info.p = {500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  EXPECT_TRUE(arm_cell_vision::CalibrationFile::compatible(
      calibration, camera_info, "/camera/color/camera_info", camera_info.header.frame_id, diagnostic)) << diagnostic;
  camera_info.k[0] += 1.0;
  EXPECT_FALSE(arm_cell_vision::CalibrationFile::compatible(
      calibration, camera_info, "/camera/color/camera_info", camera_info.header.frame_id, diagnostic));
  EXPECT_FALSE(arm_cell_vision::CalibrationFile::compatible(
      calibration, camera_info, "/camera/other/camera_info", camera_info.header.frame_id, diagnostic));
}

TEST_F(CalibrationFileTest, ExplicitCalibrationWritesTimestampedSiblingWithoutOverwrite)
{
  const auto configured = root_ / "selected.yaml";
  write_valid_file(configured);
  arm_cell_vision::CalibrationFileData calibration;
  std::string diagnostic;
  ASSERT_TRUE(arm_cell_vision::CalibrationFile::load(
      configured.string(), "/camera/color/camera_info", calibration, diagnostic)) << diagnostic;

  std::string created_path;
  ASSERT_TRUE(arm_cell_vision::CalibrationFile::save_new_sibling(
      configured.string(), calibration, created_path, diagnostic)) << diagnostic;
  EXPECT_NE(created_path, configured.string());
  EXPECT_TRUE(std::filesystem::exists(configured));
  EXPECT_TRUE(std::filesystem::exists(created_path));
  EXPECT_TRUE(std::filesystem::path(created_path).parent_path() == configured.parent_path());
  std::string second_path;
  ASSERT_TRUE(arm_cell_vision::CalibrationFile::save_new_sibling(
      configured.string(), calibration, second_path, diagnostic)) << diagnostic;
  EXPECT_NE(created_path, second_path);
  EXPECT_TRUE(std::filesystem::exists(created_path));
}

}  // namespace
