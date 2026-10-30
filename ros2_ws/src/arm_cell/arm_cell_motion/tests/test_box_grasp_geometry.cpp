#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <array>

#include "arm_cell_motion/box_grasp_geometry.hpp"

using arm_cell_motion::box_grasp_contact;
using arm_cell_motion::BoxFaceEvidence;

TEST(BoxGraspGeometry, SidePairAcceptsTiltAroundClosingAxis)
{
  const Eigen::Quaterniond object(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond tcp = object *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()));
  const auto result = box_grasp_contact(object, tcp, 0.07, 0.05, 0.08, 40.0, 85.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->face_evidence, BoxFaceEvidence::INFERRED_SIDE_PAIR);
  EXPECT_EQ(result->closing_axis, 0U);
  EXPECT_NEAR(result->width_mm, 70.0, 1e-6);
}

TEST(BoxGraspGeometry, TopBottomPairUsesMeasuredHeight)
{
  const Eigen::Quaterniond object = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond tcp(Eigen::AngleAxisd(
      1.5707963267948966, Eigen::Vector3d::UnitY()));
  const auto result = box_grasp_contact(object, tcp, 0.07, 0.05, 0.08, 40.0, 85.0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->face_evidence, BoxFaceEvidence::OBSERVED_TOP_INFERRED_BOTTOM);
  EXPECT_NEAR(result->width_mm, 80.0, 1e-6);
}

TEST(BoxGraspGeometry, OppositeClosingDirectionsAndTcpRollAreEquivalent)
{
  const Eigen::Quaterniond object(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const std::array<double, 3> expected_width_mm{70.0, 50.0, 80.0};
  for (std::size_t axis = 0; axis < expected_width_mm.size(); ++axis) {
    for (const double sign : {1.0, -1.0}) {
      const Eigen::Vector3d closing_axis = sign * Eigen::Vector3d::Unit(axis);
      const Eigen::Quaterniond alignment = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitX(), closing_axis);
      for (const double roll_rad : {0.0, 0.7}) {
        const Eigen::Quaterniond tcp = object * alignment *
          Eigen::Quaterniond(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
        const auto result = box_grasp_contact(
          object, tcp, 0.07, 0.05, 0.08, 40.0, 85.0);
        ASSERT_TRUE(result.has_value()) << "axis=" << axis << " sign=" << sign <<
          " roll=" << roll_rad;
        EXPECT_EQ(result->closing_axis, axis);
        EXPECT_NEAR(result->width_mm, expected_width_mm[axis], 1e-6);
        EXPECT_EQ(
          result->face_evidence,
          axis == 2 ? BoxFaceEvidence::OBSERVED_TOP_INFERRED_BOTTOM :
          BoxFaceEvidence::INFERRED_SIDE_PAIR);
      }
    }
  }
}

TEST(BoxGraspGeometry, RejectsCornerContactAndWidthBeyondGripper)
{
  const Eigen::Quaterniond object = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond diagonal(Eigen::AngleAxisd(
      0.7853981633974483, Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(box_grasp_contact(object, diagonal, 0.07, 0.05, 0.08, 40.0, 85.0));
  EXPECT_FALSE(box_grasp_contact(object, object, 0.09, 0.05, 0.08, 40.0, 85.0));
}

TEST(BoxGraspGeometry, OneDegreeAlignmentTolerancePreservesSignsAndWidths)
{
  constexpr double pi = 3.14159265358979323846;
  for (double sign : {1.0, -1.0}) {
    for (double degrees : {0.5, 1.0, 1.001}) {
      const Eigen::Quaterniond tcp(Eigen::AngleAxisd(
          (sign < 0.0 ? pi : 0.0) + degrees * pi / 180.0,
          Eigen::Vector3d::UnitZ()));
      const auto contact = box_grasp_contact(
        Eigen::Quaterniond::Identity(), tcp, 0.07, 0.05, 0.08, 60.0, 85.0);
      EXPECT_EQ(contact.has_value(), degrees <= 1.0);
      if (contact) {
        EXPECT_EQ(contact->closing_axis, 0U);
        EXPECT_DOUBLE_EQ(contact->width_mm, 70.0);
      }
      EXPECT_FALSE(box_grasp_contact(
          Eigen::Quaterniond::Identity(), tcp, 0.09, 0.05, 0.08, 60.0, 100.0));
      EXPECT_FALSE(box_grasp_contact(
          Eigen::Quaterniond::Identity(), tcp, 0.07, 0.05, 0.08, 71.0, 85.0));
      EXPECT_FALSE(box_grasp_contact(
          Eigen::Quaterniond::Identity(), tcp, 0.07, 0.05, 0.08, 60.0, 69.0));
    }
  }
}
