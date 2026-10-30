#include "arm_cell_motion_moveit2/trajectory_settling_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace arm_cell_motion_moveit2
{
namespace
{
std::string format_values(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << "[" << std::fixed << std::setprecision(9);
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ",";
    }
    stream << values[index];
  }
  stream << "]";
  return stream.str();
}
}

bool should_log_settling_snapshot(
  double last_logged_simulation_time,
  double simulation_time,
  double period_seconds)
{
  return std::isfinite(last_logged_simulation_time) &&
         std::isfinite(simulation_time) &&
         std::isfinite(period_seconds) && period_seconds > 0.0 &&
         simulation_time - last_logged_simulation_time >= period_seconds;
}

std::string format_trajectory_settling_snapshot(
  double simulation_time_since_trajectory_end,
  const std::vector<double> & commanded_positions,
  const std::vector<double> & actual_positions,
  const std::vector<std::string> & joint_names)
{
  const auto count = std::min(commanded_positions.size(), actual_positions.size());
  std::vector<double> absolute_error;
  absolute_error.reserve(count);
  double max_error = 0.0;
  std::size_t max_error_index = 0;
  for (std::size_t index = 0; index < count; ++index) {
    const auto error = std::abs(commanded_positions[index] - actual_positions[index]);
    absolute_error.push_back(error);
    if (error > max_error) {
      max_error = error;
      max_error_index = index;
    }
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9)
         << "simulation_time_since_trajectory_end=" << simulation_time_since_trajectory_end
         << " commanded=" << format_values(commanded_positions)
         << " actual=" << format_values(actual_positions)
         << " absolute_error=" << format_values(absolute_error);
  if (count == 0) {
    stream << " max_error_joint_index=none max_error_joint_name=none"
           << " max_tracking_error=0.000000000";
  } else {
    stream << " max_error_joint_index=" << max_error_index
           << " max_error_joint_name="
           << (max_error_index < joint_names.size() ? joint_names[max_error_index] : "unknown")
           << " max_tracking_error=" << max_error;
  }
  return stream.str();
}
}  // namespace arm_cell_motion_moveit2
