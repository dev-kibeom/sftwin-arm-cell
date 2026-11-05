#include "cuboid_unresolved.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <utility>

namespace arm_cell_vision::cuboid_unresolved
{
using namespace cuboid_geometry;
constexpr double pi = 3.14159265358979323846;
Fit handoff_model(const Fit & fit)
{
  Fit model = fit;
  model.center = fit.observed_center;
  model.top = fit.observed_top;
  model.face.length = static_cast<float>(fit.measured_length);
  model.face.width = static_cast<float>(fit.measured_width);
  model.face.thickness = static_cast<float>(fit.face.thickness);
  if (model.face.length < model.face.width) {
    std::swap(model.face.length, model.face.width);
    model.yaw += pi / 2;
  }
  return model;
}

double median(std::vector<double> values)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 != 0) {return *middle;}
  const double upper = *middle;
  const double lower = *std::max_element(values.begin(), middle);
  return (lower + upper) / 2;
}

cv::Mat model_association(
  const Fit & fit, const CuboidHeightObservation & obs, const DetectorProfile & profile,
  const cv::Mat & missing)
{
  const auto model = handoff_model(fit);
  cv::Mat association = cv::Mat::zeros(obs.heights.size(), CV_8U);
  const double bottom = std::max(0.0, model.top - model.face.thickness);
  // A covered model must not absorb actual upper evidence into its own top.
  const double association_top = model.top +
    (fit.covered_by_upper_surface ? 0.0 : profile.cuboid.height_tolerance_m);
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      const auto relative = obs.origin + cv::Point2d(col, row) * obs.spacing_m - model.center;
      if (std::abs(relative.dot(axis(model.yaw))) > model.face.length / 2 + 1e-9 ||
        std::abs(relative.dot(normal(model.yaw))) > model.face.width / 2 + 1e-9) {continue;}
      const double height = obs.heights.at<double>(row, col);
      if ((!missing.empty() && missing.at<uint8_t>(row, col)) ||
        (height > bottom + profile.cuboid.height_tolerance_m &&
        height <= association_top))
      {
        association.at<uint8_t>(row, col) = 255;
      }
    }
  }
  return association;
}

cv::Mat unresolved_observation_cells(
  const std::vector<Fit> & models, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, const cv::Mat & missing)
{
  cv::Mat unresolved = obs.heights > profile.support_contact_tolerance_m;
  for (const auto & fit : models) {
    unresolved.setTo(0, model_association(fit, obs, profile, missing));
  }
  return unresolved;
}

}  // namespace arm_cell_vision::cuboid_unresolved
