#include "arm_cell_orchestration_bt/recipe_repository.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace arm_cell_orchestration_bt
{
namespace
{

class RecipeRepositoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    directory_ = std::filesystem::temp_directory_path() /
      ("arm_cell_recipes_" + std::to_string(counter_++));
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(directory_);
  }

  void write(const std::string & name, const std::string & contents)
  {
    std::ofstream(directory_ / name) << contents;
  }

  static inline unsigned counter_{0};
  std::filesystem::path directory_;
};

constexpr char kValidRecipe[] = R"({
  "schema_version": 1,
  "recipe_id": "rawpart-placement-v1",
  "target_id": "RawPart",
  "grasp": {"width_mm": 70.0, "yaw_rad": 0.0},
  "place": {
    "orientation_constraint": "bounded",
    "desired_object_pose": {
      "frame": "base_link",
      "position": [0.50, 0.25, 0.3675],
      "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]
    },
    "position_tolerance_m": 0.01,
    "orientation_tolerance_rad": 0.034906585,
    "approach": {"direction_object": [0.0, 0.0, 1.0], "distance_m": 0.015},
    "retract_distance_m": 0.05
  }
})";

TEST_F(RecipeRepositoryTest, ParsesAValidRecipeIntoMissionSemantics)
{
  write("RawPart.json", kValidRecipe);

  const auto result = RecipeRepository(directory_).load("RawPart");

  ASSERT_TRUE(result.recipe);
  EXPECT_EQ(result.recipe->recipe_id, "rawpart-placement-v1");
  EXPECT_EQ(result.recipe->target_id, "RawPart");
  EXPECT_FLOAT_EQ(result.recipe->grasp_width_mm, 70.0F);
  EXPECT_DOUBLE_EQ(result.recipe->grasp_yaw_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.recipe->place_pose.pose.position.z, 0.3675);
  EXPECT_DOUBLE_EQ(result.recipe->place_position_tolerance_m, 0.01);
  EXPECT_EQ(result.recipe->place_orientation_constraint, PlaceOrientationConstraint::BOUNDED);
  EXPECT_DOUBLE_EQ(result.recipe->place_approach_distance_m, 0.015);
  EXPECT_DOUBLE_EQ(result.recipe->retract_distance_m, 0.05);
}

TEST_F(RecipeRepositoryTest, ParsesPreferredToolOrientationIndependentlyOfObjectPose)
{
  auto contents = std::string(kValidRecipe);
  contents.insert(
    contents.find("\"position_tolerance_m\""),
    "\"tool_orientation\": {\"mode\": \"preferred\", "
    "\"orientation_xyzw\": [1.0, 0.0, 0.0, 0.0]},\n    ");
  write("RawPart.json", contents);

  const auto result = RecipeRepository(directory_).load("RawPart");

  ASSERT_TRUE(result.recipe);
  EXPECT_DOUBLE_EQ(result.recipe->place_pose.pose.orientation.w, 1.0);
  ASSERT_TRUE(result.recipe->has_place_tool_orientation_preference);
  EXPECT_DOUBLE_EQ(result.recipe->place_tool_orientation_preference.x, 1.0);
  EXPECT_DOUBLE_EQ(result.recipe->place_tool_orientation_preference.w, 0.0);
}

TEST_F(RecipeRepositoryTest, ParsesFreeOrientationWithoutToolPreference)
{
  auto contents = std::string(kValidRecipe);
  contents.replace(contents.find("bounded"), std::string("bounded").size(), "free");
  write("RawPart.json", contents);

  const auto result = RecipeRepository(directory_).load("RawPart");

  ASSERT_TRUE(result.recipe);
  EXPECT_EQ(result.recipe->place_orientation_constraint, PlaceOrientationConstraint::FREE);
  EXPECT_FALSE(result.recipe->has_place_tool_orientation_preference);
}

TEST_F(RecipeRepositoryTest, RejectsUnknownOrientationConstraintAtJsonBoundary)
{
  auto contents = std::string(kValidRecipe);
  contents.replace(contents.find("bounded"), std::string("bounded").size(), "unbounded");
  write("RawPart.json", contents);

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
  EXPECT_EQ(result.reason, RecipeDiagnosticReason::INVALID_ORIENTATION_CONSTRAINT);
}

TEST_F(RecipeRepositoryTest, RejectsToolPreferenceForFreeOrientation)
{
  auto contents = std::string(kValidRecipe);
  contents.replace(contents.find("bounded"), std::string("bounded").size(), "free");
  contents.insert(
    contents.find("\"position_tolerance_m\""),
    "\"tool_orientation\": {\"mode\": \"preferred\", "
    "\"orientation_xyzw\": [0.0, 0.0, 0.0, 1.0]},\n    ");
  write("RawPart.json", contents);

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
  EXPECT_EQ(result.reason, RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION);
}

TEST_F(RecipeRepositoryTest, RejectsInvalidPreferredToolOrientationFailClosed)
{
  const std::vector<std::string> invalid_values{
    "\"tool_orientation\": {\"mode\": \"preferred\", "
    "\"orientation_xyzw\": [0.0, 0.0, 0.0, 0.0]}",
    "\"tool_orientation\": {\"mode\": \"required\", "
    "\"orientation_xyzw\": [0.0, 0.0, 0.0, 1.0]}",
    "\"tool_orientation\": {\"mode\": \"preferred\", "
    "\"orientation_xyzw\": [0.0, 0.0, 0.0, 2.0]}"};
  for (const auto & orientation : invalid_values) {
    auto contents = std::string(kValidRecipe);
    contents.insert(contents.find("\"position_tolerance_m\""), orientation + ",\n    ");
    write("RawPart.json", contents);

    const auto result = RecipeRepository(directory_).load("RawPart");

    EXPECT_FALSE(result.recipe);
    EXPECT_EQ(result.reason, RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION);
  }
}

TEST_F(RecipeRepositoryTest, RejectsNonFinitePreferredToolOrientation)
{
  auto contents = std::string(kValidRecipe);
  contents.insert(
    contents.find("\"position_tolerance_m\""),
    "\"tool_orientation\": {\"mode\": \"preferred\", "
    "\"orientation_xyzw\": [0.0, 0.0, 0.0, 1e309]},\n    ");
  write("RawPart.json", contents);

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
}

TEST_F(RecipeRepositoryTest, SelectsRecipeByRequestedTargetId)
{
  write("RawPart.json", kValidRecipe);
  write("Other.json", std::string(kValidRecipe).replace(
    std::string(kValidRecipe).find("RawPart"), 7, "Other"));

  const auto result = RecipeRepository(directory_).load("Other");

  ASSERT_TRUE(result.recipe);
  EXPECT_EQ(result.recipe->target_id, "Other");
}

TEST_F(RecipeRepositoryTest, RejectsMalformedJsonWithStructuredReason)
{
  write("RawPart.json", "{ malformed");

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
  EXPECT_EQ(result.reason, RecipeDiagnosticReason::JSON_MALFORMED);
}

TEST_F(RecipeRepositoryTest, RejectsMissingRequiredField)
{
  write("RawPart.json", R"({"schema_version":1})");

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
  EXPECT_EQ(result.reason, RecipeDiagnosticReason::REQUIRED_FIELD_MISSING);
}

TEST_F(RecipeRepositoryTest, DistinguishesRequiredFieldTypeErrorsByReasonToken)
{
  auto contents = std::string(kValidRecipe);
  contents.replace(contents.find("70.0"), 4, "\"wide\"");
  write("RawPart.json", contents);

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
  EXPECT_EQ(result.reason, RecipeDiagnosticReason::TYPE_MISMATCH);
  EXPECT_STREQ(recipe_reason_token(result.reason), "recipe_type_mismatch");
}

TEST_F(RecipeRepositoryTest, RejectsUnsupportedSchemaVersion)
{
  write("RawPart.json", std::string(kValidRecipe).replace(
    std::string(kValidRecipe).find("\"schema_version\": 1"), 18,
    "\"schema_version\": 2"));

  const auto result = RecipeRepository(directory_).load("RawPart");

  EXPECT_FALSE(result.recipe);
  EXPECT_EQ(result.reason, RecipeDiagnosticReason::SCHEMA_UNSUPPORTED);
}

TEST_F(RecipeRepositoryTest, RejectsSemanticViolationsWithDistinctReasons)
{
  struct InvalidCase
  {
    std::string from;
    std::string to;
    RecipeDiagnosticReason expected;
  };
  const InvalidCase cases[] = {
    {"0.0, 0.0, 0.0, 1.0", "0.0, 0.0, 0.0, 0.0", RecipeDiagnosticReason::INVALID_QUATERNION},
    {"\"direction_object\": [0.0, 0.0, 1.0]",
      "\"direction_object\": [0.0, 0.0, 2.0]",
      RecipeDiagnosticReason::INVALID_APPROACH_DIRECTION},
    {"\"distance_m\": 0.015", "\"distance_m\": 0.0", RecipeDiagnosticReason::INVALID_DISTANCE},
    {"\"position_tolerance_m\": 0.01", "\"position_tolerance_m\": -0.01", RecipeDiagnosticReason::INVALID_TOLERANCE},
    {"\"width_mm\": 70.0", "\"width_mm\": 86.0", RecipeDiagnosticReason::INVALID_GRASP_WIDTH},
  };
  for (const auto & invalid : cases) {
    auto contents = std::string(kValidRecipe);
    contents.replace(contents.find(invalid.from), invalid.from.size(), invalid.to);
    write("RawPart.json", contents);
    const auto result = RecipeRepository(directory_).load("RawPart");
    EXPECT_FALSE(result.recipe);
    EXPECT_EQ(result.reason, invalid.expected);
  }
}

TEST_F(RecipeRepositoryTest, DistinguishesTargetMismatchAndMissingFile)
{
  auto mismatch = std::string(kValidRecipe);
  mismatch.replace(mismatch.find("RawPart"), 7, "Other");
  write("RawPart.json", mismatch);
  EXPECT_EQ(
    RecipeRepository(directory_).load("RawPart").reason,
    RecipeDiagnosticReason::TARGET_MISMATCH);
  EXPECT_EQ(
    RecipeRepository(directory_).load("Missing").reason,
    RecipeDiagnosticReason::FILE_NOT_FOUND);
}

}  // namespace
}  // namespace arm_cell_orchestration_bt
