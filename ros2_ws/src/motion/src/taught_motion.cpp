#include "arm_cell_motion/taught_motion.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace arm_cell_motion
{
TaughtMotions parse_taught_motions(
  const std::string & json, const std::vector<double> & home,
  const std::vector<double> & tray_return, std::size_t joint_count)
{
  const auto motions = nlohmann::json::parse(json);
  if (!motions.is_object() || motions.empty() || joint_count == 0U) {
    throw std::invalid_argument("Taught motions must be a nonempty named mapping");
  }
  TaughtMotions result;
  for (const auto & entry : motions.items()) {
    const auto & value = entry.value();
    if (!std::regex_match(entry.key(), std::regex("[a-z][a-z0-9_]*")) ||
      !value.is_object() || value.size() != (value.contains("joint_path") ? 4U + value.contains("plan_to_start") : 3U) ||
      !value.contains("velocity_scale") || !value.contains("acceleration_scale") ||
      (static_cast<int>(value.contains("target")) + static_cast<int>(value.contains("joint_positions")) +
      static_cast<int>(value.contains("joint_path"))) != 1)
    {
      throw std::invalid_argument("Invalid taught motion definition: " + entry.key());
    }
    TaughtMotion motion;
    motion.velocity_scale = value.at("velocity_scale").get<double>();
    motion.acceleration_scale = value.at("acceleration_scale").get<double>();
    if (value.contains("joint_path")) {
      motion.plan_to_start = value.value("plan_to_start", false);
      motion.joint_path = value.at("joint_path").get<std::vector<std::vector<double>>>();
      motion.duration_s = value.at("duration_s").get<double>();
      if (motion.joint_path.size() < 2 || !std::isfinite(motion.duration_s) || motion.duration_s <= 0 ||
        !std::all_of(motion.joint_path.begin(), motion.joint_path.end(), [joint_count](const auto & knot) {
          return knot.size() == joint_count && std::all_of(knot.begin(), knot.end(),
            [](double q) {return std::isfinite(q);});
        })) {throw std::invalid_argument("Invalid taught joint path: " + entry.key());}
      motion.joint_positions = motion.joint_path.back();
    } else if (value.contains("target")) {
      const auto target = value.at("target").get<std::string>();
      if (target != "home" && target != "tray_return") {
        throw std::invalid_argument("Unknown taught target: " + target);
      }
      motion.joint_positions = target == "home" ? home : tray_return;
    } else {
      motion.joint_positions = value.at("joint_positions").get<std::vector<double>>();
    }
    const auto valid_scale = [](double scale) {
        return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
      };
    if (!valid_scale(motion.velocity_scale) || !valid_scale(motion.acceleration_scale) ||
      motion.joint_positions.size() != joint_count ||
      !std::all_of(
        motion.joint_positions.begin(), motion.joint_positions.end(),
        [](double position) {return std::isfinite(position);}))
    {
      throw std::invalid_argument(
              "Invalid taught motion limits or joint positions: " +
              entry.key());
    }
    result.emplace(entry.key(), std::move(motion));
  }
  return result;
}
}  // namespace arm_cell_motion
