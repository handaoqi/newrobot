#include <gtest/gtest.h>

#include <localization/point_cloud_scheduler.hpp>
#include <localization/correction_cooldown_gate.hpp>

namespace localization {
namespace {

TEST(PointCloudScheduler, StableLioUsesConfiguredStationaryStride) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;
  input.now_ns = 1000000000LL;
  input.frame_index = 4;

  auto decision = decidePointCloudWork(config, input);
  EXPECT_FALSE(decision.run_ndt);
  EXPECT_FALSE(decision.needs_heavy_cloud);

  input.frame_index = 5;
  decision = decidePointCloudWork(config, input);
  EXPECT_TRUE(decision.run_ndt);
  EXPECT_TRUE(decision.needs_heavy_cloud);
}

TEST(PointCloudScheduler, InitializationUsesDedicatedStride) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.lio_primary_enabled = true;
  input.now_ns = 1000000000LL;
  input.frame_index = 1;

  EXPECT_FALSE(decidePointCloudWork(config, input).run_ndt);
  input.frame_index = 2;
  EXPECT_TRUE(decidePointCloudWork(config, input).run_ndt);
}

TEST(PointCloudScheduler, RelocalizationAndLidarOdometryForceHeavyCloud) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;
  input.now_ns = 1000000000LL;
  input.frame_index = 1;
  input.global_relocalization_requested = true;
  EXPECT_TRUE(decidePointCloudWork(config, input).needs_heavy_cloud);

  input.global_relocalization_requested = false;
  input.lidar_odometry_required = true;
  EXPECT_TRUE(decidePointCloudWork(config, input).needs_heavy_cloud);
}

TEST(PointCloudScheduler, PausedRtkPrimarySkipsAllCloudWork) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;
  input.now_ns = 1000000000LL;
  input.frame_index = 5;
  input.lidar_matching_paused = true;
  input.rtk_primary = true;
  input.global_relocalization_requested = true;
  input.lidar_odometry_required = true;

  const auto decision = decidePointCloudWork(config, input);
  EXPECT_FALSE(decision.run_ndt);
  EXPECT_FALSE(decision.needs_heavy_cloud);
}

TEST(PointCloudScheduler, NdtPrimaryKeepsLegacyStationaryRate) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.frame_index = 1;
  EXPECT_TRUE(decidePointCloudWork(config, input).run_ndt);
}

TEST(PointCloudScheduler, StableLioRespectsWallClockRateOnStrideAlignedCloud) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;
  input.frame_index = 10;
  input.last_ndt_start_ns = 1000000000LL;
  input.now_ns = 1400000000LL;

  auto decision = decidePointCloudWork(config, input);
  EXPECT_TRUE(decision.stride_due);
  EXPECT_FALSE(decision.rate_due);
  EXPECT_FALSE(decision.run_ndt);
  EXPECT_EQ(decision.reason, "rate_limited");

  input.now_ns = 1500000000LL;
  decision = decidePointCloudWork(config, input);
  EXPECT_TRUE(decision.run_ndt);
}

TEST(PointCloudScheduler, StableLioRunsFirstCloudAfterRateDeadline) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;
  input.last_ndt_start_ns = 1000000000LL;

  // The stride-aligned cloud arrives just before the 500 ms deadline.
  input.frame_index = 10;
  input.now_ns = 1490000000LL;
  auto decision = decidePointCloudWork(config, input);
  EXPECT_TRUE(decision.stride_due);
  EXPECT_FALSE(decision.rate_due);
  EXPECT_FALSE(decision.run_ndt);

  // The next cloud must run immediately even though it is not stride-aligned.
  input.frame_index = 11;
  input.now_ns = 1590000000LL;
  decision = decidePointCloudWork(config, input);
  EXPECT_FALSE(decision.stride_due);
  EXPECT_TRUE(decision.rate_due);
  EXPECT_TRUE(decision.run_ndt);
  EXPECT_EQ(decision.reason, "stable_match");
}

TEST(PointCloudScheduler, StableLioMaintainsTwoHertzWithJitteredTenHertzClouds) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;

  constexpr std::int64_t millisecond = 1000000LL;
  const std::int64_t frame_times_ms[] = {
    100, 199, 301, 398, 497, 601, 699, 802, 899, 998,
    1101, 1198, 1302, 1401, 1497, 1600, 1699, 1801, 1898, 2002,
  };
  std::int64_t last_start_ns = 0;
  int match_count = 0;
  std::size_t last_match_frame = 0;
  const std::size_t frame_count = sizeof(frame_times_ms) / sizeof(frame_times_ms[0]);
  for (std::size_t index = 0; index < frame_count; ++index) {
    input.frame_index = index + 1;
    input.now_ns = frame_times_ms[index] * millisecond;
    input.last_ndt_start_ns = last_start_ns;
    const auto decision = decidePointCloudWork(config, input);
    if (decision.run_ndt) {
      ++match_count;
      last_match_frame = input.frame_index;
      last_start_ns = input.now_ns;
    }
  }

  // Bootstrap on frame 5, then run on the first cloud after each 500 ms
  // deadline: about 2 Hz without requiring a stride/rate phase coincidence.
  EXPECT_EQ(match_count, 3);
  EXPECT_EQ(last_match_frame, 16U);
}

TEST(PointCloudScheduler, RecoveryUsesFiveHertzRateAndInitializationStride) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = false;
  input.frame_index = 4;
  input.last_ndt_start_ns = 1000000000LL;
  input.now_ns = 1190000000LL;
  EXPECT_FALSE(decidePointCloudWork(config, input).run_ndt);
  input.now_ns = 1200000000LL;
  EXPECT_TRUE(decidePointCloudWork(config, input).run_ndt);
}

TEST(PointCloudScheduler, CorrectionSuppressionKeepsStableFrameLight) {
  PointCloudScheduleConfig config;
  PointCloudScheduleInput input;
  input.initialized = true;
  input.lio_primary_enabled = true;
  input.lio_stable = true;
  input.correction_suppressed = true;
  input.frame_index = 5;
  input.now_ns = 1000000000LL;
  const auto decision = decidePointCloudWork(config, input);
  EXPECT_FALSE(decision.run_ndt);
  EXPECT_FALSE(decision.needs_heavy_cloud);
  EXPECT_EQ(decision.reason, "correction_suppressed");
}

TEST(CorrectionCooldownGate, EnforcesStartIntervalAndPostCompletionSuppression) {
  CorrectionCooldownGate gate;
  gate.configure(3.0, 3.0);
  constexpr std::int64_t second = 1000000000LL;
  EXPECT_TRUE(gate.canStart(10 * second));
  gate.markStarted(10 * second);
  EXPECT_FALSE(gate.canStart(12 * second));
  EXPECT_TRUE(gate.canStart(13 * second));
  gate.markCompleted(14 * second);
  EXPECT_FALSE(gate.canStart(16 * second));
  EXPECT_DOUBLE_EQ(gate.remainingSeconds(16 * second), 1.0);
  EXPECT_TRUE(gate.canStart(17 * second));
}

TEST(PointCloudScheduler, StaleGuardOnlyDropsInitializedPastClouds) {
  constexpr std::int64_t second = 1000000000LL;
  EXPECT_FALSE(isPointCloudStale(10 * second, 9 * second, 0.2, false));
  EXPECT_TRUE(isPointCloudStale(10 * second, 9 * second, 0.2, true));
  EXPECT_FALSE(isPointCloudStale(10 * second, 11 * second, 0.2, true));
  EXPECT_FALSE(isPointCloudStale(10 * second, 9 * second, 0.0, true));
}

}  // namespace
}  // namespace localization
