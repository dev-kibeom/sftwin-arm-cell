#include <gtest/gtest.h>

#include "arm_cell_vision/vision_sensor_ingress.hpp"

namespace arm_cell_vision
{
namespace
{
constexpr int64_t kFlush = 1'000'000'000;
constexpr int64_t kSettle = 50'000'000;
constexpr int64_t kTimeout = 500'000'000;

SensorSample sample(int64_t stamp, int64_t receipt)
{
  return SensorSample{stamp, receipt, 1280, 720, "camera_color_optical_frame"};
}

SynchronizedObservationCandidate candidate(int64_t stamp, int64_t receipt)
{
  return SynchronizedObservationCandidate{
    sample(stamp, receipt), sample(stamp, receipt), sample(stamp, receipt), stamp};
}

SensorIngressPolicy make_policy(std::optional<int64_t> max_age = std::nullopt)
{
  return SensorIngressPolicy({kSettle, kTimeout, max_age});
}

void begin(SensorIngressPolicy & ingress, int64_t watermark = kFlush, int64_t now = 10)
{
  ASSERT_EQ(ingress.begin_acquisition(watermark, now).status, IngressStatus::kWaiting);
}

TEST(VisionSensorIngress, AcceptsExactCandidateAtSettlingBoundary)
{
  auto ingress = make_policy();
  begin(ingress);

  const auto result = ingress.evaluate(candidate(kFlush + kSettle, 20), 20);
  ASSERT_TRUE(result.observation);
  EXPECT_EQ(result.status, IngressStatus::kObservationReady);
  EXPECT_EQ(result.observation->stamp_ns(), kFlush + kSettle);
}

TEST(VisionSensorIngress, MonotonicSynchronizedProgressionDoesNotRollback)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + kSettle;
  const auto t2 = t1 + kSettle;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);

  begin(ingress, t1, 21);
  EXPECT_EQ(ingress.evaluate(candidate(t2, 21), 21).status, IngressStatus::kObservationReady);
  EXPECT_FALSE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, SameStampCandidateIsDuplicateAndDoesNotRollback)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + kSettle;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);

  begin(ingress, t1, 21);
  EXPECT_EQ(ingress.evaluate(candidate(t1, 21), 21).status, IngressStatus::kDuplicateObservation);
  EXPECT_FALSE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, GenuineSynchronizedRollbackInvalidatesEpoch)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + kSettle;
  const auto t2 = t1 + kSettle;
  const auto t0 = t1 - 1;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);
  begin(ingress, t1, 21);
  ASSERT_EQ(ingress.evaluate(candidate(t2, 21), 21).status, IngressStatus::kObservationReady);
  begin(ingress, t2, 22);

  EXPECT_EQ(ingress.evaluate(candidate(t0, 22), 22).status, IngressStatus::kTimeRollback);
  EXPECT_TRUE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, NewerCandidateIsRefusedAfterRollbackUntilLifecycleReset)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + kSettle;
  const auto t2 = t1 + kSettle;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);
  begin(ingress, t1, 21);
  ASSERT_EQ(ingress.evaluate(candidate(t2, 21), 21).status, IngressStatus::kObservationReady);
  begin(ingress, t2, 22);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 22), 22).status, IngressStatus::kTimeRollback);

  EXPECT_EQ(
    ingress.evaluate(
      candidate(
        t2 + kSettle,
        23), 23).status, IngressStatus::kEpochInvalidated);
}

TEST(VisionSensorIngress, ExplicitLifecycleResetAllowsNewEpoch)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + kSettle;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);
  begin(ingress, t1, 21);
  ASSERT_EQ(ingress.evaluate(candidate(kFlush, 21), 21).status, IngressStatus::kTimeRollback);

  EXPECT_EQ(ingress.reset_after_lifecycle().status, IngressStatus::kWaiting);
  begin(ingress, 10, 22);
  EXPECT_EQ(
    ingress.evaluate(
      candidate(
        10 + kSettle,
        22), 22).status, IngressStatus::kObservationReady);
}

TEST(VisionSensorIngress, WatermarkIsStrictAndSettlingUsesSensorTime)
{
  auto ingress = make_policy();
  begin(ingress);
  EXPECT_EQ(ingress.evaluate(candidate(kFlush, 20), 20).status, IngressStatus::kStaleObservation);
  EXPECT_EQ(
    ingress.evaluate(candidate(kFlush + kSettle - 1, 21), 21).status,
    IngressStatus::kUnsettledObservation);
  EXPECT_EQ(
    ingress.evaluate(
      candidate(
        kFlush + kSettle,
        22), 22).status, IngressStatus::kObservationReady);
}

TEST(VisionSensorIngress, WallTimeDoesNotSatisfySettlingButDoesExpireDeadline)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);
  EXPECT_EQ(
    ingress.evaluate(candidate(kFlush + kSettle - 1, 101), 100 + 100'000'000).status,
    IngressStatus::kUnsettledObservation);
  EXPECT_EQ(ingress.check_deadline(100 + kTimeout).status, IngressStatus::kRequestTimeout);
}

TEST(VisionSensorIngress, ReceiptFreshnessHasNoDefaultAndUsesExplicitConfiguration)
{
  auto ingress = make_policy(5);
  begin(ingress);
  EXPECT_EQ(
    ingress.evaluate(candidate(kFlush + kSettle, 20), 26).status,
    IngressStatus::kReceiptStaleObservation);
}

TEST(VisionSensorIngress, PreservesBoundedSkewMetadataAndRejectsMalformedMetadata)
{
  auto ingress = make_policy();
  begin(ingress);
  const auto stamp = kFlush + kSettle;
  auto bounded_skew = candidate(stamp, 20);
  bounded_skew.depth.stamp_ns = stamp + 1;
  EXPECT_EQ(ingress.evaluate(bounded_skew, 20).status, IngressStatus::kObservationReady);

  begin(ingress, stamp, 21);
  auto malformed = candidate(stamp + 1, 21);
  malformed.rgb.stamp_ns.reset();
  EXPECT_EQ(ingress.evaluate(malformed, 21).status, IngressStatus::kMalformedObservation);
}

TEST(VisionSensorIngress, CandidateDimensionsAndFramesMustAgree)
{
  auto ingress = make_policy();
  begin(ingress);
  auto inconsistent = candidate(kFlush + kSettle, 20);
  inconsistent.depth.width = 640;
  EXPECT_EQ(ingress.evaluate(inconsistent, 20).status, IngressStatus::kIncompatibleMetadata);
}

}  // namespace
}  // namespace arm_cell_vision
