#include <gtest/gtest.h>
#include "arm_cell_orchestration/pnp_progress.hpp"

using arm_cell_orchestration::PnpProgress;

TEST(PnpProgress, TerminalObservationAloneDoesNotReportProgress)
{
  PnpProgress progress;
  progress.elapsed_s = 44.4;
  progress.completed = true;
  EXPECT_FALSE(progress.commit(0));
  const auto summary = progress.commit(1);
  ASSERT_TRUE(summary);
  EXPECT_DOUBLE_EQ(summary->elapsed_s, 44.4);
  EXPECT_TRUE(summary->completed);
  EXPECT_FALSE(progress.commit(1));
  EXPECT_FALSE(progress.commit(0));
}

TEST(PnpProgress, PlacedMaterialWithFailedHomeIsNotCompletedCycle)
{
  PnpProgress progress;
  progress.elapsed_s = 22.1;
  progress.completed = false;
  const auto summary = progress.commit(1);
  ASSERT_TRUE(summary);
  EXPECT_FALSE(summary->completed);
  progress.elapsed_s = 30.0;
  progress.completed = true;
  EXPECT_FALSE(progress.commit(1));
  const auto recovered = progress.commit(2);
  ASSERT_TRUE(recovered);
  EXPECT_TRUE(recovered->completed);
  EXPECT_DOUBLE_EQ(recovered->elapsed_s, 30.0);
}
