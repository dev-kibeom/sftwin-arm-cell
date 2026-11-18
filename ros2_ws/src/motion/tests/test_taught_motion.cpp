#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include "arm_cell_motion/taught_motion.hpp"

using namespace arm_cell_motion;

TEST(TaughtMotion, ResolvesRobotTargetsAndIndependentLimits)
{
  const auto motions = parse_taught_motions(
    R"({
    "home":{"target":"home","velocity_scale":0.3,"acceleration_scale":0.2},
    "return_material":{"target":"tray_return","velocity_scale":0.2,"acceleration_scale":0.15},
    "standby":{"joint_positions":[0.5,0.6],"velocity_scale":0.4,"acceleration_scale":0.1}
  })", {0.0, 0.1}, {0.2, 0.3},
    2U);
  EXPECT_EQ(motions.at("home").joint_positions, (std::vector<double>{0.0, 0.1}));
  EXPECT_EQ(motions.at("return_material").joint_positions, (std::vector<double>{0.2, 0.3}));
  EXPECT_DOUBLE_EQ(motions.at("home").velocity_scale, 0.3);
  EXPECT_DOUBLE_EQ(motions.at("return_material").acceleration_scale, 0.15);
  EXPECT_EQ(motions.at("standby").joint_positions, (std::vector<double>{0.5, 0.6}));
}

TEST(TaughtMotion, RejectsInvalidDefinitionsBeforeMovement)
{
  const nlohmann::json good = {{"home", {
      {"target", "home"}, {"velocity_scale", 0.3}, {"acceleration_scale", 0.2}}}};
  for (double scale : {0.0, -0.1, 1.01}) {
    for (const auto * key : {"velocity_scale", "acceleration_scale"}) {
      auto bad = good;
      bad["home"][key] = scale;
      EXPECT_THROW(parse_taught_motions(bad.dump(), {0.0}, {0.1}, 1U), std::invalid_argument);
    }
  }
  auto bad = good;
  bad["home"]["target"] = "missing";
  EXPECT_THROW(parse_taught_motions(bad.dump(), {0.0}, {0.1}, 1U), std::invalid_argument);
  bad = good;
  bad["home"]["joint_positions"] = {0.0};
  EXPECT_THROW(parse_taught_motions(bad.dump(), {0.0}, {0.1}, 1U), std::invalid_argument);
  EXPECT_THROW(parse_taught_motions(good.dump(), {0.0}, {0.1}, 2U), std::invalid_argument);
  EXPECT_THROW(parse_taught_motions("{}", {0.0}, {0.1}, 1U), std::invalid_argument);
}

TEST(TaughtMotion, RegistryNamesUseTheSameDataSchema)
{
  const auto motions = parse_taught_motions(
    R"({
    "return_material":{"joint_positions":[0.5],"velocity_scale":0.3,"acceleration_scale":0.2},
    "custom_pose":{"joint_positions":[0.5],"velocity_scale":0.3,"acceleration_scale":0.2}
  })", {0.0}, {0.1},
    1U);
  EXPECT_EQ(
    motions.at("return_material").joint_positions,
    motions.at("custom_pose").joint_positions);
}

TEST(TaughtMotion, PreservesConfiguredPathAndDuration)
{
  const auto motions = parse_taught_motions(
    R"({"insert":{"joint_path":[[0.0,0.1],[0.2,0.3],[0.4,0.5]],"duration_s":4.0,"velocity_scale":0.3,"acceleration_scale":0.2}})",
    {0.0,0.0}, {0.0,0.0}, 2);
  EXPECT_EQ(motions.at("insert").joint_path.size(), 3U);
  EXPECT_DOUBLE_EQ(motions.at("insert").duration_s, 4.0);
}

TEST(TaughtMotion, PlannedEntryIsExplicitAndOnlyAvailableForConfiguredPaths)
{
  const auto motions = parse_taught_motions(
    R"({"approach":{"joint_path":[[0.0],[0.1]],"duration_s":4.0,"velocity_scale":0.3,"acceleration_scale":0.2,"plan_to_start":true}})",
    {0.0}, {0.0}, 1);
  EXPECT_TRUE(motions.at("approach").plan_to_start);
  EXPECT_THROW(parse_taught_motions(
    R"({"home":{"target":"home","velocity_scale":0.3,"acceleration_scale":0.2,"plan_to_start":true}})",
    {0.0}, {0.0}, 1), std::invalid_argument);
}
