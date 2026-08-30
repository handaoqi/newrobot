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

TEST(PointCloudScheduler, StableLioRequiresStrideAndWallClockRate) {
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
