#include <gtest/gtest.h>

#include "arm_cell_vision/vision_sensor_ingress.hpp"

namespace arm_cell_vision
{
namespace
{
constexpr int64_t kFlush = 1'000'000'000;
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
  return SensorIngressPolicy({kTimeout, max_age});
}

void begin(SensorIngressPolicy & ingress, int64_t watermark = kFlush, int64_t now = 10)
{
  ASSERT_EQ(ingress.begin_acquisition(watermark, now).status, IngressStatus::kWaiting);
}

TEST(VisionSensorIngress, AcceptsFreshPostRequestCandidateImmediately)
{
  auto ingress = make_policy();
  begin(ingress);

  const auto result = ingress.evaluate(candidate(kFlush + 1, 20), 20);
  ASSERT_TRUE(result.observation);
  EXPECT_EQ(result.status, IngressStatus::kObservationReady);
  EXPECT_EQ(result.observation->stamp_ns(), kFlush + 1);
}

TEST(VisionSensorIngress, MonotonicSynchronizedProgressionDoesNotRollback)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + 1;
  const auto t2 = t1 + 1;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);

  begin(ingress, t1, 21);
  EXPECT_EQ(ingress.evaluate(candidate(t2, 22), 22).status, IngressStatus::kObservationReady);
  EXPECT_FALSE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, SameStampCandidateIsDuplicateAndDoesNotRollback)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + 1;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);

  begin(ingress, t1, 21);
  EXPECT_EQ(ingress.evaluate(candidate(t1, 22), 22).status, IngressStatus::kDuplicateObservation);
  EXPECT_FALSE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, GenuineSynchronizedRollbackInvalidatesEpoch)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + 1;
  const auto t2 = t1 + 1;
  const auto t0 = t1 - 1;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);
  begin(ingress, t1, 21);
  ASSERT_EQ(ingress.evaluate(candidate(t2, 22), 22).status, IngressStatus::kObservationReady);
  begin(ingress, t2, 22);

  EXPECT_EQ(ingress.evaluate(candidate(t0, 23), 23).status, IngressStatus::kTimeRollback);
  EXPECT_TRUE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, NewerCandidateIsRefusedAfterRollbackUntilLifecycleReset)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + 1;
  const auto t2 = t1 + 1;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);
  begin(ingress, t1, 21);
  ASSERT_EQ(ingress.evaluate(candidate(t2, 22), 22).status, IngressStatus::kObservationReady);
  begin(ingress, t2, 22);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 23), 23).status, IngressStatus::kTimeRollback);

  EXPECT_EQ(
    ingress.evaluate(
      candidate(
        t2 + 1,
        23), 23).status, IngressStatus::kEpochInvalidated);
}

TEST(VisionSensorIngress, ExplicitLifecycleResetAllowsNewEpoch)
{
  auto ingress = make_policy();
  const auto t1 = kFlush + 1;
  begin(ingress);
  ASSERT_EQ(ingress.evaluate(candidate(t1, 20), 20).status, IngressStatus::kObservationReady);
  begin(ingress, t1, 21);
  ASSERT_EQ(ingress.evaluate(candidate(kFlush, 22), 22).status, IngressStatus::kTimeRollback);

  EXPECT_EQ(ingress.reset_after_lifecycle().status, IngressStatus::kWaiting);
  begin(ingress, 10, 22);
  EXPECT_EQ(
    ingress.evaluate(
      candidate(
        11,
        23), 23).status, IngressStatus::kObservationReady);
}

TEST(VisionSensorIngress, WatermarkIsStrictAndFreshPostRequestPairIsReady)
{
  auto ingress = make_policy();
  begin(ingress);
  EXPECT_EQ(ingress.evaluate(candidate(kFlush, 20), 20).status, IngressStatus::kStaleObservation);
  EXPECT_EQ(
    ingress.evaluate(candidate(kFlush + 1, 21), 21).status,
    IngressStatus::kObservationReady);
}

TEST(VisionSensorIngress, FreshCachedInputCanStartANewDetectionRequest)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);

  const auto result = ingress.evaluate_cached(candidate(kFlush, 90), 100, 20);
  ASSERT_EQ(result.status, IngressStatus::kObservationReady);
  ASSERT_TRUE(result.observation);
  EXPECT_EQ(result.observation->stamp_ns(), kFlush);
  EXPECT_EQ(ingress.check_deadline(101).status, IngressStatus::kNoActiveAcquisition);
}

TEST(VisionSensorIngress, CachedInputOlderThanReuseWindowIsRejected)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);

  const auto result = ingress.evaluate_cached(candidate(kFlush, 79), 100, 20);
  EXPECT_EQ(result.status, IngressStatus::kReceiptStaleObservation);
  EXPECT_EQ(ingress.check_deadline(101).status, IngressStatus::kWaiting);
}

TEST(VisionSensorIngress, CachedInputStillRequiresCompatibleSourceIdentity)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);
  auto source = candidate(kFlush, 100);
  source.depth.frame_id = "other_camera_optical_frame";

  EXPECT_EQ(ingress.evaluate_cached(source, 100, 20).status, IngressStatus::kIncompatibleMetadata);
}

TEST(VisionSensorIngress, CachedInputMustAlsoBeFreshInSensorTimeWhenClockIsAvailable)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);

  const auto result = ingress.evaluate_cached(
    candidate(kFlush, 100), 100, 20, kFlush + 21);
  EXPECT_EQ(result.status, IngressStatus::kReceiptStaleObservation);
}

TEST(VisionSensorIngress, PreRequestReceiptIsRejectedEvenWhenItsStampExceedsWatermark)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);

  EXPECT_EQ(
    ingress.evaluate(candidate(kFlush + 1, 99), 101).status,
    IngressStatus::kStaleObservation);
  EXPECT_FALSE(ingress.epoch_invalidated());
}

TEST(VisionSensorIngress, RequestTimeoutUsesMonotonicClock)
{
  auto ingress = make_policy();
  begin(ingress, kFlush, 100);
  EXPECT_EQ(ingress.check_deadline(100 + 100'000'000).status, IngressStatus::kWaiting);
  EXPECT_EQ(ingress.check_deadline(100 + kTimeout).status, IngressStatus::kRequestTimeout);
}

TEST(VisionSensorIngress, RequestTimeoutOverridesAdapterDefault)
{
  SensorIngressPolicy ingress({10, std::nullopt});
  ASSERT_EQ(ingress.begin_acquisition(kFlush, 100, 100).status, IngressStatus::kWaiting);
  EXPECT_EQ(ingress.check_deadline(199).status, IngressStatus::kWaiting);
  EXPECT_EQ(ingress.check_deadline(200).status, IngressStatus::kRequestTimeout);
}

TEST(VisionSensorIngress, ReceiptFreshnessHasNoDefaultAndUsesExplicitConfiguration)
{
  auto ingress = make_policy(5);
  begin(ingress);
  EXPECT_EQ(
    ingress.evaluate(candidate(kFlush + 1, 20), 26).status,
    IngressStatus::kReceiptStaleObservation);
}

TEST(VisionSensorIngress, PreservesBoundedSkewMetadataAndRejectsMalformedMetadata)
{
  auto ingress = make_policy();
  begin(ingress);
  const auto stamp = kFlush + 1;
  auto bounded_skew = candidate(stamp, 20);
  bounded_skew.depth.stamp_ns = stamp + 1;
  EXPECT_EQ(ingress.evaluate(bounded_skew, 20).status, IngressStatus::kObservationReady);

  begin(ingress, stamp, 21);
  auto malformed = candidate(stamp + 1, 22);
  malformed.rgb.stamp_ns.reset();
  EXPECT_EQ(ingress.evaluate(malformed, 21).status, IngressStatus::kMalformedObservation);
}

TEST(VisionSensorIngress, CandidateDimensionsAndFramesMustAgree)
{
  auto ingress = make_policy();
  begin(ingress);
  auto inconsistent = candidate(kFlush + 1, 20);
  inconsistent.depth.width = 640;
  EXPECT_EQ(ingress.evaluate(inconsistent, 20).status, IngressStatus::kIncompatibleMetadata);
}

}  // namespace
}  // namespace arm_cell_vision
