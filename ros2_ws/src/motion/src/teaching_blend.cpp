#include "arm_cell_motion/trajectory_generation.hpp"

#include <algorithm>

namespace arm_cell_motion
{
namespace
{
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

struct JointPath
{
  std::vector<Eigen::VectorXd> positions;
  std::vector<double> distances;

  Eigen::VectorXd at(double distance) const
  {
    if (distance <= 0.) {return positions.front();}
    if (distance >= distances.back()) {return positions.back();}
    const auto next = std::upper_bound(distances.begin(), distances.end(), distance);
    const auto index = static_cast<std::size_t>(next - distances.begin());
    const double progress = (distance - distances[index - 1]) /
      (distances[index] - distances[index - 1]);
    return positions[index - 1] + progress * (positions[index] - positions[index - 1]);
  }
};
}

bool blend_taught_joint_paths(
  const std::vector<JointTrajectory> & segments, moveit_msgs::msg::RobotTrajectory & result)
{
  result = {};
  if (segments.size() < 2 || segments.front().joint_names.empty()) {return false;}
  const auto & names = segments.front().joint_names;

  std::vector<JointPath> paths;
  paths.reserve(segments.size());
  for (const auto & segment : segments) {
    if (segment.joint_names != names || segment.points.size() < 2) {return false;}
    JointPath path;
    for (const auto & point : segment.points) {
      if (point.positions.size() != names.size()) {return false;}
      Eigen::VectorXd position = Eigen::Map<const Eigen::VectorXd>(
        point.positions.data(), point.positions.size());
      if (!position.allFinite()) {return false;}
      const double length = path.positions.empty() ? 0. :
        (position - path.positions.back()).norm();
      if (!path.positions.empty() && length < 1e-9) {continue;}
      path.distances.push_back(path.distances.empty() ? 0. : path.distances.back() + length);
      path.positions.push_back(std::move(position));
    }
    if (path.positions.size() < 2 || !std::isfinite(path.distances.back()) ||
      (!paths.empty() && !path.positions.front().isApprox(paths.back().positions.back(), 1e-9)))
    {
      return false;
    }
    paths.push_back(std::move(path));
  }

  // Trim at most 0.04 rad of joint-space path on either side. Reserve the
  // middle third of every segment so adjacent blends cannot overlap.
  constexpr double max_trim_rad = .04;
  constexpr int blend_samples = 12;
  std::vector<double> trims{0.};
  for (std::size_t i = 1; i < paths.size(); ++i) {
    trims.push_back(
      std::min(
        {max_trim_rad, paths[i - 1].distances.back() / 3., paths[i].distances.back() / 3.}));
  }
  trims.push_back(0.);

  JointTrajectory trajectory;
  trajectory.joint_names = names;
  const auto append = [&](const Eigen::VectorXd & position) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(position.data(), position.data() + position.size());
      trajectory.points.push_back(std::move(point));
    };
  append(paths.front().positions.front());
  for (std::size_t i = 0; i < paths.size(); ++i) {
    const auto & path = paths[i];
    const auto end = path.distances.back() - trims[i + 1];
    for (std::size_t j = 1; j < path.positions.size(); ++j) {
      if (path.distances[j] > trims[i] && path.distances[j] < end) {
        append(path.positions[j]);
      }
    }
    const Eigen::VectorXd entry = path.at(end);
    append(entry);
    if (i + 1 == paths.size()) {break;}
    const Eigen::VectorXd & corner = path.positions.back();
    const Eigen::VectorXd exit = paths[i + 1].at(trims[i + 1]);
    // A near reversal would make the quadratic come to a stop inside the blend.
    const Eigen::VectorXd incoming = corner - entry;
    const Eigen::VectorXd outgoing = exit - corner;
    if (incoming.norm() < 1e-9 || outgoing.norm() < 1e-9 ||
      incoming.normalized().dot(outgoing.normalized()) < -.99)
    {
      return false;
    }
    for (int j = 1; j <= blend_samples; ++j) {
      const double t = static_cast<double>(j) / blend_samples;
      append((1. - t) * (1. - t) * entry + 2. * t * (1. - t) * corner + t * t * exit);
    }
  }
  trajectory.points.back().positions = segments.back().points.back().positions;
  result.joint_trajectory = std::move(trajectory);
  return true;
}
}  // namespace arm_cell_motion
