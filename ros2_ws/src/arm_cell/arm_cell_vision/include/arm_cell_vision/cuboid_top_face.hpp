#ifndef ARM_CELL_VISION__CUBOID_TOP_FACE_HPP_
#define ARM_CELL_VISION__CUBOID_TOP_FACE_HPP_

#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <tf2/LinearMath/Transform.h>
#include "arm_cell_vision/detect_target_processor.hpp"

namespace arm_cell_vision
{
std::shared_ptr<const std::vector<cv::Point2f>> normalized_camera_ray_lookup(
  const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height);
// Orthographic support-plane observation. NaN cells have no measured depth.
// origin is the XY coordinate of cell (0,0), not a simulator/world reference.
struct CuboidHeightObservation
{
  cv::Mat heights;  // CV_64F, support-relative metres
  cv::Point2d origin;
  double spacing_m;
  cv::Mat image_boundary;  // CV_8U when known; projected source-image edge samples.
};

void validate_detector_algorithm(const std::string & algorithm);
void validate_cuboid_config(const CuboidTopFaceConfig & config);
DetectionResult detect_cuboid_top_face(
  const CuboidHeightObservation & observation, const DetectorProfile & profile,
  bool retain_replay_evidence = false);
DetectionResult process_cuboid_top_face(
  const DetectionInput & input, const DetectorProfile & profile,
  const tf2::Transform & support_T_camera);
}  // namespace arm_cell_vision
#endif
