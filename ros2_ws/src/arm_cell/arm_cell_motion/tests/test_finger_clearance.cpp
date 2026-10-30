#include <gtest/gtest.h>
#include "arm_cell_motion/finger_clearance.hpp"

namespace arm_cell_motion
{
namespace
{
Eigen::Isometry3d pose(double x, double y, double z, double yaw = 0.0)
{
  auto result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, z);
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}
TEST(FingerClearance, OpposingFacesUseContactWidthAndObjectCenterRatherThanTcpX)
{
  const auto volumes = make_finger_protection_volumes(
    pose(0.003, 0.012, 0.04), Eigen::Vector3d(0, 0, 0.02),
    Eigen::Quaterniond::Identity(), BoxGraspContact{0U, 60.0},
    Eigen::Vector3d::UnitZ(), {});
  EXPECT_TRUE(volumes[0].frame.translation().isApprox(Eigen::Vector3d(-0.03, 0.012, 0.04)));
  EXPECT_TRUE(volumes[1].frame.translation().isApprox(Eigen::Vector3d(0.03, 0.012, 0.04)));
  EXPECT_TRUE(volumes[0].frame.linear().col(0).isApprox(-Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(volumes[1].frame.linear().col(0).isApprox(Eigen::Vector3d::UnitX()));
}
TEST(FingerClearance, CapsuleEndsAndOneSidedInteriorAreNotRectangular)
{
  FingerProtectionVolume v;
  const Eigen::Vector3d tiny(0.001, 0.001, 0.001);
  EXPECT_TRUE(finger_protection_intersects_box(v, pose(0.005, 0.0, -0.01), tiny));
  EXPECT_TRUE(finger_protection_intersects_box(v, pose(0.003, 0.018, -0.01), tiny));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.009, 0.019, -0.01), tiny));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(-0.005, 0.0, -0.01), tiny));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.005, 0.0, -0.03), tiny));
}
TEST(FingerClearance, BoundaryOnlyContactIsAllowedOnAllClippingPlanesAndRoundSide)
{
  FingerProtectionVolume v;
  const Eigen::Vector3d size(0.002, 0.002, 0.002);
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.011, 0, -0.01), size));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(-0.001, 0, -0.01), size));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.005, 0, 0.001), size));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.005, 0, -0.026), size));
  EXPECT_TRUE(finger_protection_intersects_box(v, pose(0.0109, 0, -0.01), size));
}
TEST(FingerClearance, RotatedBoxAvoidsAabbFalsePositiveAndRotationIsEquivariant)
{
  FingerProtectionVolume v;
  // Long diagonal box has an AABB overlapping the protection, but lies beyond its end.
  const auto box = pose(0.025, 0.025, -0.01, -0.7853981633974483);
  const Eigen::Vector3d size(0.04, 0.002, 0.01);
  EXPECT_FALSE(finger_protection_intersects_box(v, box, size));
  const auto rotation = pose(0.2, -0.1, 0.3, 1.1);
  v.frame = rotation;
  EXPECT_FALSE(finger_protection_intersects_box(v, rotation * box, size));
  EXPECT_TRUE(finger_protection_intersects_box(v, rotation * pose(0.005, 0, -0.01), size));
}
TEST(FingerClearance, ParametersChangeRadiusSpanAndDepthIndependently)
{
  FingerProtectionVolume v;
  const auto box = pose(0.014, 0, -0.01);
  const Eigen::Vector3d tiny(0.001, 0.001, 0.001);
  EXPECT_FALSE(finger_protection_intersects_box(v, box, tiny));
  v.dimensions.radius_m = 0.02;
  EXPECT_TRUE(finger_protection_intersects_box(v, box, tiny));
  v = FingerProtectionVolume{};
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.003, 0.027, -0.01), tiny));
  v.dimensions.contact_span_m = 0.04;
  EXPECT_TRUE(finger_protection_intersects_box(v, pose(0.003, 0.027, -0.01), tiny));
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.003, 0, -0.035), tiny));
  v.dimensions.depth_m = 0.04;
  EXPECT_TRUE(finger_protection_intersects_box(v, pose(0.003, 0, -0.035), tiny));
}
TEST(FingerClearance, ConvexTopUsesActualSlopeAndAllowsContact)
{
  FingerProtectionVolume v;
  std::vector<Eigen::Vector3d> top{{0.005, -0.005, -0.005},
    {0.025, -0.005, 0.005}, {0.025, 0.005, 0.005}, {0.005, 0.005, -0.005}};
  EXPECT_TRUE(finger_protection_intersects_convex_top(v, Eigen::Isometry3d::Identity(), top, -0.02));
  for (auto & p : top) {p.x() += 0.02;}
  EXPECT_FALSE(finger_protection_intersects_convex_top(v, Eigen::Isometry3d::Identity(), top, -0.02));
  for (auto & p : top) {p.x() -= 0.015; p.z() += 0.02;}
  EXPECT_FALSE(finger_protection_intersects_convex_top(v, Eigen::Isometry3d::Identity(), top, 0.0));
}
TEST(FingerClearance, ContactCentersAndVolumesRotateWithYaw)
{
  const auto rotation = pose(0.1, 0.2, 0.3, 0.8);
  const auto volumes = make_finger_protection_volumes(
    rotation, rotation.translation(), Eigen::Quaterniond(rotation.linear()),
    BoxGraspContact{0U, 60.0}, Eigen::Vector3d::UnitZ(), {});
  EXPECT_TRUE(volumes[1].frame.translation().isApprox(rotation * Eigen::Vector3d(0.03, 0, 0)));
  EXPECT_TRUE(
    finger_protection_intersects_box(
      volumes[1], rotation * pose(0.035, 0, -0.01), Eigen::Vector3d::Constant(0.002)));
  EXPECT_FALSE(
    finger_protection_intersects_box(
      volumes[0], rotation * pose(0.035, 0, -0.01), Eigen::Vector3d::Constant(0.002)));
}
TEST(FingerClearance, ArbitraryThreeDimensionalRotationAndRoundEndTangency)
{
  FingerProtectionVolume v;
  const Eigen::Vector3d size(0.002, 0.002, 0.002);
  EXPECT_FALSE(finger_protection_intersects_box(v, pose(0.0, 0.021, -0.01), size));
  const auto transform = pose(0.3, -0.2, 0.5, 0.7) *
    Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX());
  v.frame = transform;
  auto rotated_box = pose(0.005, 0, -0.01);
  rotated_box.linear() = Eigen::AngleAxisd(
    0.8,
    Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  EXPECT_TRUE(finger_protection_intersects_box(v, transform * rotated_box, size));
  EXPECT_FALSE(
    finger_protection_intersects_box(
      v, transform * pose(0.03, 0, -0.01), size));
}
TEST(FingerClearance, ClosingAxisSignAndChoiceDoNotReestimateContact)
{
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(
      1.5707963267948966,
      Eigen::Vector3d::UnitZ()));
  auto tcp = pose(0.01, 0, 0.04);
  tcp.linear() = yaw.toRotationMatrix();
  const auto volumes = make_finger_protection_volumes(
    tcp, Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(), BoxGraspContact{1U, 40.0},
    Eigen::Vector3d::UnitZ(), {});
  EXPECT_TRUE(volumes[0].frame.translation().isApprox(Eigen::Vector3d(0.01, -0.02, 0.04)));
  EXPECT_TRUE(volumes[1].frame.translation().isApprox(Eigen::Vector3d(0.01, 0.02, 0.04)));
}
TEST(FingerClearance, FingerIdentityFollowsTcpWhenClosingAxisSignReverses)
{
  const auto volumes = make_finger_protection_volumes(
    pose(0, 0, 0, 3.141592653589793), Eigen::Vector3d::Zero(),
    Eigen::Quaterniond::Identity(), BoxGraspContact{0U, 60.0},
    Eigen::Vector3d::UnitZ(), {});
  EXPECT_TRUE(volumes[0].frame.translation().isApprox(Eigen::Vector3d(0.03, 0, 0)));
  EXPECT_TRUE(volumes[1].frame.translation().isApprox(Eigen::Vector3d(-0.03, 0, 0)));
}
}  // namespace
}  // namespace arm_cell_motion
