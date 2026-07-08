#include "hik_camera_ros2_driver/livox_timestamp_synchronizer.hpp"

#include <gtest/gtest.h>

using hik_camera_ros2_driver::LivoxTimestampSynchronizer;
using hik_camera_ros2_driver::LivoxTimestampSynchronizerConfig;

namespace
{

LivoxTimestampSynchronizerConfig makeConfig()
{
  LivoxTimestampSynchronizerConfig config;
  config.queue_size = 3;
  config.wait_timeout_ns = 1000000;
  config.max_pairing_host_delta_ns = 5000000;
  return config;
}

}  // namespace

TEST(LivoxTimestampSynchronizerTest, AcceptsNormalSamples)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(1000000000, 1000000000, 10));

  const auto stats = sync.stats();
  EXPECT_EQ(stats.accepted_livox_stamps, 1u);
  EXPECT_EQ(stats.invalid_livox_stamps, 0u);
  EXPECT_EQ(sync.queueSize(), 1u);
}

TEST(LivoxTimestampSynchronizerTest, RejectsZeroTimestamp)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_FALSE(sync.addSample(0, 0, 10));

  const auto stats = sync.stats();
  EXPECT_EQ(stats.invalid_livox_stamps, 1u);
  EXPECT_EQ(sync.queueSize(), 0u);
}

TEST(LivoxTimestampSynchronizerTest, RejectsDuplicateTimestamp)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(1000000000, 1000000000, 10));
  EXPECT_FALSE(sync.addSample(1000000000, 1000000000, 20));

  const auto stats = sync.stats();
  EXPECT_EQ(stats.duplicate_livox_stamps, 1u);
  EXPECT_EQ(sync.queueSize(), 1u);
}

TEST(LivoxTimestampSynchronizerTest, RejectsRollbackTimestamp)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(2000000000, 2000000000, 10));
  EXPECT_FALSE(sync.addSample(1000000000, 1000000000, 20));

  const auto stats = sync.stats();
  EXPECT_EQ(stats.rollback_livox_stamps, 1u);
  EXPECT_EQ(sync.queueSize(), 1u);
}

TEST(LivoxTimestampSynchronizerTest, CleansQueueOverflow)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(1000000000, 1000000000, 10));
  EXPECT_TRUE(sync.addSample(1100000000, 1100000000, 20));
  EXPECT_TRUE(sync.addSample(1200000000, 1200000000, 30));
  EXPECT_TRUE(sync.addSample(1300000000, 1300000000, 40));

  const auto stats = sync.stats();
  EXPECT_EQ(stats.queue_overflow_drops, 1u);
  EXPECT_EQ(sync.queueSize(), 3u);
}

TEST(LivoxTimestampSynchronizerTest, MatchesNearestImageTimestamp)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(1000000000, 1000000000, 100));
  EXPECT_TRUE(sync.addSample(1100000000, 1100000000, 200));

  const auto result = sync.matchImage(198);

  ASSERT_TRUE(result.matched);
  EXPECT_EQ(result.sample.stamp_ns, 1100000000);
  EXPECT_EQ(result.host_delta_ns, 2);
}

TEST(LivoxTimestampSynchronizerTest, DoesNotReuseConsumedTimestamp)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(1000000000, 1000000000, 100));
  EXPECT_TRUE(sync.matchImage(100).matched);

  const auto result = sync.matchImage(100);

  EXPECT_FALSE(result.matched);
  const auto stats = sync.stats();
  EXPECT_EQ(stats.dropped_unsynced_frames, 1u);
}

TEST(LivoxTimestampSynchronizerTest, RejectsHostDeltaTooLarge)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_TRUE(sync.addSample(1000000000, 1000000000, 100));

  const auto result = sync.matchImage(10000000);

  EXPECT_FALSE(result.matched);
  const auto stats = sync.stats();
  EXPECT_EQ(stats.dropped_unsynced_frames, 1u);
}

TEST(LivoxTimestampSynchronizerTest, RejectsHeaderMismatch)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  EXPECT_FALSE(sync.addSample(1000000000, 1000000100, 100));

  const auto stats = sync.stats();
  EXPECT_EQ(stats.header_mismatch_stamps, 1u);
  EXPECT_EQ(sync.queueSize(), 0u);
}

TEST(LivoxTimestampSynchronizerTest, ShutdownWakesWaitingMatch)
{
  LivoxTimestampSynchronizer sync(makeConfig());

  sync.shutdown();
  const auto result = sync.matchImage(100);

  EXPECT_FALSE(result.matched);
}
