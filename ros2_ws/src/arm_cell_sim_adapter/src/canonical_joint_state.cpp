#include "arm_cell_sim_adapter/canonical_joint_state.hpp"
#include <cmath>
#include <unordered_map>

namespace arm_cell_sim_adapter
{
std::optional<sensor_msgs::msg::JointState> canonical_joint_state(
  const sensor_msgs::msg::JointState & raw,
  const std::vector<std::string> & independent_joints)
{
  const auto size = raw.name.size();
  if (raw.position.size() != size ||
    (!raw.velocity.empty() && raw.velocity.size() != size) ||
    (!raw.effort.empty() && raw.effort.size() != size))
  {
    return std::nullopt;
  }
  std::unordered_map<std::string, size_t> indices;
  for (size_t i = 0; i < size; ++i) {
    if (!indices.emplace(raw.name[i], i).second) {
      return std::nullopt;
    }
  }
  sensor_msgs::msg::JointState result;
  result.header.stamp = raw.header.stamp;
  for (const auto & name : independent_joints) {
    const auto found = indices.find(name);
    if (found == indices.end()) {
      return std::nullopt;
    }
    const auto i = found->second;
    if (!std::isfinite(raw.position[i]) ||
      (!raw.velocity.empty() && !std::isfinite(raw.velocity[i])) ||
      (!raw.effort.empty() && !std::isfinite(raw.effort[i])))
    {
      return std::nullopt;
    }
    result.name.push_back(name);
    result.position.push_back(raw.position[i]);
    if (!raw.velocity.empty()) {
      result.velocity.push_back(raw.velocity[i]);
    }
    if (!raw.effort.empty()) {
      result.effort.push_back(raw.effort[i]);
    }
  }
  return result;
}
}  // namespace arm_cell_sim_adapter
