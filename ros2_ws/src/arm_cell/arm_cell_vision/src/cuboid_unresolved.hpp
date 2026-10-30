#pragma once
#include "cuboid_top_face_geometry.hpp"
#include <string>

namespace arm_cell_vision::cuboid_unresolved
{
using cuboid_geometry::Fit;
Fit handoff_model(const Fit & fit);
cv::Mat unresolved_observation_cells(
  const std::vector<Fit> &, const CuboidHeightObservation &,
  const DetectorProfile &, const cv::Mat &);
}  // namespace arm_cell_vision::cuboid_unresolved
