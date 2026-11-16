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
      !value.is_object() || value.size() != 3U ||
      !value.contains("velocity_scale") || !value.contains("acceleration_scale") ||
      value.contains("target") == value.contains("joint_positions"))
    {
      throw std::invalid_argument("Invalid taught motion definition: " + entry.key());
    }
    TaughtMotion motion;
    motion.velocity_scale = value.at("velocity_scale").get<double>();
    motion.acceleration_scale = value.at("acceleration_scale").get<double>();
    if (value.contains("target")) {
      const auto target = value.at("target").get<std::string>();
      if (target != "home" && target != "tray_return") {
        throw std::invalid_argument("Unknown taught target: " + target);
      }
      if (((entry.key() == "home" || entry.key() == "payload_home") && target != "home") ||
        (entry.key() == "return_material" && target != "tray_return"))
      {
        throw std::invalid_argument("Reserved taught motion target mismatch: " + entry.key());
      }
      motion.joint_positions = target == "home" ? home : tray_return;
    } else {
      if (entry.key() == "home" || entry.key() == "payload_home" ||
        entry.key() == "return_material")
      {
        throw std::invalid_argument(
                "Reserved taught motion requires its robot target: " + entry.key());
      }
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
