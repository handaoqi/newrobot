#include <gtest/gtest.h>

#include <cmath>

#include "localization/lio_motion_guard.hpp"

namespace {

constexpr double kDeg = M_PI / 180.0;

TEST(LioMotionGuard, AcceptsNormalMaximumTurn)
{
  const auto result = localization::evaluateLioMotion(
    {0.0, 1000000000LL}, {2.0 * kDeg, 1100000000LL}, 12.0 * kDeg, 60.0 * kDeg);
  EXPECT_FALSE(result.anomaly);
  EXPECT_NEAR(result.yaw_rate_radps, 20.0 * kDeg, 1.0e-6);
}

TEST(LioMotionGuard, HandlesWrappedYaw)
{
  const auto result = localization::evaluateLioMotion(
    {179.0 * kDeg, 1000000000LL}, {-179.0 * kDeg, 1100000000LL},
    12.0 * kDeg, 60.0 * kDeg);
  EXPECT_FALSE(result.anomaly);
  EXPECT_NEAR(result.yaw_step_rad, 2.0 * kDeg, 1.0e-6);
}

TEST(LioMotionGuard, RejectsSingleFrameYawJump)
{
  const auto result = localization::evaluateLioMotion(
    {0.0, 1000000000LL}, {13.0 * kDeg, 1300000000LL}, 12.0 * kDeg, 60.0 * kDeg);
  EXPECT_TRUE(result.anomaly);
  EXPECT_EQ(result.reason, "yaw_step_exceeded");
}

TEST(LioMotionGuard, RejectsYawRateJump)
{
  const auto result = localization::evaluateLioMotion(
    {0.0, 1000000000LL}, {7.0 * kDeg, 1100000000LL}, 12.0 * kDeg, 60.0 * kDeg);
  EXPECT_TRUE(result.anomaly);
  EXPECT_EQ(result.reason, "yaw_rate_exceeded");
}

TEST(LioMotionGuard, DuplicateUnchangedSampleIsSafe)
{
  const auto result = localization::evaluateLioMotion(
    {0.5, 1000000000LL}, {0.5, 1000000000LL}, 12.0 * kDeg, 60.0 * kDeg);
  EXPECT_FALSE(result.anomaly);
  EXPECT_TRUE(result.duplicate);
}

TEST(LioMotionGuard, RejectsTimeRegression)
{
  const auto result = localization::evaluateLioMotion(
    {0.0, 1100000000LL}, {0.01, 1000000000LL}, 12.0 * kDeg, 60.0 * kDeg);
  EXPECT_TRUE(result.anomaly);
  EXPECT_EQ(result.reason, "non_monotonic_stamp");
}

}  // namespace
