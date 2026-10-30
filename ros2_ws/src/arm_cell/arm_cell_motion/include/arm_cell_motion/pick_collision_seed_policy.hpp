#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

namespace arm_cell_motion
{

struct PickCollisionContact
{
  std::string body_1;
  std::string body_2;
  std::string stage;
  bool sample_index_available{false};
  std::size_t sample_index{0U};
  std::size_t pose_candidate{0U};
  std::size_t seed_attempt{0U};
};

// The callback returns ancestor variable names for robot links and an empty
// list for obstacles. Variable order comes from the active MoveIt group.
inline std::vector<std::string> collision_influenced_variables(
  const std::vector<PickCollisionContact> & contacts,
  const std::vector<std::string> & group_variables,
  const std::function<std::vector<std::string>(const std::string &)> & ancestors)
{
  std::unordered_set<std::string> influenced;
  for (const auto & contact : contacts) {
    for (const auto * body : {&contact.body_1, &contact.body_2}) {
      for (const auto & variable : ancestors(*body)) {influenced.insert(variable);}
    }
  }
  std::vector<std::string> ordered;
  for (const auto & variable : group_variables) {
    if (influenced.count(variable) != 0U) {ordered.push_back(variable);}
  }
  return ordered;
}

inline std::vector<double> diversify_pick_ik_seed(
  const std::vector<double> & measured_seed,
  const std::vector<double> & randomized_seed,
  const std::vector<std::string> & group_variables,
  const std::vector<std::string> & collision_influenced)
{
  if (measured_seed.size() != randomized_seed.size() ||
    measured_seed.size() != group_variables.size() || collision_influenced.empty())
  {
    return randomized_seed;
  }
  const std::unordered_set<std::string> prioritized(
    collision_influenced.begin(), collision_influenced.end());
  auto seed = measured_seed;
  for (std::size_t index = 0U; index < group_variables.size(); ++index) {
    if (prioritized.count(group_variables[index]) != 0U) {
      seed[index] = randomized_seed[index];
    }
  }
  return seed;
}

}  // namespace arm_cell_motion
