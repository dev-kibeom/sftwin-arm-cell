#ifndef ARM_CELL_VISION__CALIBRATION_FILE_HPP_
#define ARM_CELL_VISION__CALIBRATION_FILE_HPP_

#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace arm_cell_vision
{

struct CalibrationFileData
{
  geometry_msgs::msg::TransformStamped tray_from_camera;
  sensor_msgs::msg::CameraInfo camera_info;
  std::string camera_info_topic;
};

class CalibrationFile
{
public:
  static bool load(
    const std::string & path, const std::string & configured_camera_info_topic,
    CalibrationFileData & output, std::string & diagnostic);

  static bool compatible(
    const CalibrationFileData & calibration, const sensor_msgs::msg::CameraInfo & camera_info,
    const std::string & configured_camera_info_topic, const std::string & camera_frame,
    std::string & diagnostic);

  static bool save_new_sibling(
    const std::string & configured_path, const CalibrationFileData & calibration,
    std::string & created_path, std::string & diagnostic);
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__CALIBRATION_FILE_HPP_
