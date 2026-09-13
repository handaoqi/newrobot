#include <gtest/gtest.h>

#include <localization/correction_policy.hpp>
#include <localization/online_anchor_correction_policy.hpp>

namespace localization {
namespace {

CorrectionCandidateSummary candidate(double x, double variance, std::int64_t stamp) {
  CorrectionCandidateSummary value;
  value.eligible = true;
  value.x = x;
  value.horizontal_variance = variance;
  value.orientation_variance = variance;
  value.stamp_ns = stamp;
  return value;
}

TEST(CorrectionPolicy, ParsesAndRoutesEveryWaypointMode) {
  EXPECT_EQ(parseCorrectionPolicyMode("NDT"), CorrectionPolicyMode::ndt);
  EXPECT_EQ(parseCorrectionPolicyMode("rtk"), CorrectionPolicyMode::rtk);
  EXPECT_EQ(parseCorrectionPolicyMode("ukf"), CorrectionPolicyMode::ukf);
  EXPECT_TRUE(correctionPolicyAllowsNdt(CorrectionPolicyMode::ndt));
  EXPECT_FALSE(correctionPolicyAllowsRtk(CorrectionPolicyMode::ndt));
  EXPECT_FALSE(correctionPolicyAllowsNdt(CorrectionPolicyMode::rtk));
  EXPECT_TRUE(correctionPolicyAllowsRtk(CorrectionPolicyMode::rtk));
  EXPECT_TRUE(correctionPolicyAllowsNdt(CorrectionPolicyMode::ukf));
  EXPECT_TRUE(correctionPolicyAllowsRtk(CorrectionPolicyMode::ukf));
}

TEST(CorrectionPolicy, NamesUkfFusedCorrectionSource) {
  EXPECT_STREQ(correctionSourceName(CorrectionSource::ukf_fused), "ukf_fused");
}

TEST(CorrectionPolicy, StrictModesNeverUseTheOtherSource) {
  const auto ndt = candidate(0.0, 0.02, 10);
  const auto rtk = candidate(0.0, 0.01, 20);
  EXPECT_EQ(selectCorrectionSource(
    CorrectionPolicyMode::ndt, ndt, rtk, 0.3, 0.1, 0.3, 0.1).source,
    CorrectionSource::ndt);
  EXPECT_EQ(selectCorrectionSource(
    CorrectionPolicyMode::rtk, ndt, rtk, 0.3, 0.1, 0.3, 0.1).source,
    CorrectionSource::rtk);
}

TEST(CorrectionPolicy, UkfSelectsLowerVarianceAndUsesFreshnessAsTieBreaker) {
  auto ndt = candidate(0.0, 0.02, 10);
  auto rtk = candidate(0.1, 0.01, 20);
  EXPECT_EQ(selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1).source,
    CorrectionSource::rtk);
  ndt.horizontal_variance = 0.01;
  EXPECT_EQ(selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1).source,
    CorrectionSource::rtk);
}

TEST(CorrectionPolicy, UkfKeepsUsingIndividuallyValidConflictingSources) {
  const auto ndt = candidate(0.0, 0.01, 10);
  const auto rtk = candidate(3.1, 0.02, 20);
  const auto selection = selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1);
  EXPECT_EQ(selection.source, CorrectionSource::ndt);
  EXPECT_EQ(selection.reason, "ukf_ndt_lower_variance");
}


TEST(CorrectionPolicy, PreferFixedRtkWinsEvenOnConflictOrLowerVariance) {
  auto ndt = candidate(0.0, 0.001, 10);
  auto rtk = candidate(1.0, 0.05, 20);
  const auto conflict_without = selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1, false);
  EXPECT_EQ(conflict_without.source, CorrectionSource::ndt);

  const auto preferred = selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1, true);
  EXPECT_EQ(preferred.source, CorrectionSource::rtk);
  EXPECT_EQ(preferred.reason, "ukf_prefer_fixed_rtk");

  ndt = candidate(0.0, 0.001, 30);
  rtk = candidate(0.05, 0.05, 40);
  const auto variance_without = selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1, false);
  EXPECT_EQ(variance_without.source, CorrectionSource::ndt);

  const auto variance_with = selectCorrectionSource(
    CorrectionPolicyMode::ukf, ndt, rtk, 0.3, 0.1, 0.3, 0.1, true);
  EXPECT_EQ(variance_with.source, CorrectionSource::rtk);
  EXPECT_EQ(variance_with.reason, "ukf_prefer_fixed_rtk");
}

TEST(CorrectionPolicy, FloatRtkResidualGateIncludesTheConfiguredBoundary) {
  EXPECT_TRUE(floatRtkResidualWithinGate(0.4000, 0.40));
  EXPECT_TRUE(floatRtkResidualWithinGate(0.0, 0.40));
  EXPECT_FALSE(floatRtkResidualWithinGate(0.4001, 0.40));
  EXPECT_FALSE(floatRtkResidualWithinGate(-0.01, 0.40));
}

TEST(CorrectionPolicy, UkfWaypointUsesFixedThenNdtBandsThenFloatFusion) {
  const auto fixed = selectWaypointCorrectionSource(
    CorrectionPolicyMode::ukf, true, 0.01, true, true, 0.10);
  EXPECT_EQ(fixed.source, CorrectionSource::rtk);
  EXPECT_EQ(fixed.reason, "ukf_prefer_fixed_rtk");

  const auto high_quality_ndt = selectWaypointCorrectionSource(
    CorrectionPolicyMode::ukf, true, 0.0999, false, true, 0.10);
  EXPECT_EQ(high_quality_ndt.source, CorrectionSource::ndt);
  EXPECT_EQ(high_quality_ndt.reason, "ukf_high_quality_ndt");

  const auto fused_at_lower_ndt_boundary = selectWaypointCorrectionSource(
    CorrectionPolicyMode::ukf, true, 0.1000, false, true, 0.10);
  EXPECT_EQ(fused_at_lower_ndt_boundary.source, CorrectionSource::ukf_fused);

  const auto fused_at_float_boundary = selectWaypointCorrectionSource(
    CorrectionPolicyMode::ukf, true, 0.3999, false,
    floatRtkResidualWithinGate(0.4000, 0.40), 0.10);
  EXPECT_EQ(fused_at_float_boundary.source, CorrectionSource::ukf_fused);

  const auto outside_float_gate_uses_ndt_only = selectWaypointCorrectionSource(
    CorrectionPolicyMode::ukf, true, 0.3999, false,
    floatRtkResidualWithinGate(0.4001, 0.40), 0.10, true, true);
  EXPECT_EQ(outside_float_gate_uses_ndt_only.source, CorrectionSource::ndt);
  EXPECT_EQ(
    outside_float_gate_uses_ndt_only.reason, "ukf_float_outside_gate_ndt_only");

  const auto no_ndt_at_upper_boundary = selectWaypointCorrectionSource(
    CorrectionPolicyMode::ukf, false, 0.40, false, false, 0.10);
  EXPECT_EQ(no_ndt_at_upper_boundary.source, CorrectionSource::none);
}

TEST(CorrectionPolicy, StrictWaypointModesDoNotFallbackAcrossSources) {
  EXPECT_EQ(selectWaypointCorrectionSource(
    CorrectionPolicyMode::rtk, true, 0.01, false, true, 0.10).source,
    CorrectionSource::none);
  EXPECT_EQ(selectWaypointCorrectionSource(
    CorrectionPolicyMode::ndt, false, 0.50, true, false, 0.10).source,
    CorrectionSource::none);
}

TEST(OnlineAnchorCorrectionPolicy, RequiresAnEligibleLowSpeedCruise) {
  OnlineAnchorCorrectionConfig config;
  OnlineAnchorCorrectionInput input;
  input.policy_allowed = true;
  input.moving = true;
  input.lio_fresh = true;
  input.nominal_profile = true;
  input.motion_valid = true;
  input.linear_speed_mps = 0.15;
  input.yaw_rate_radps = 0.10;

  EXPECT_TRUE(onlineAnchorCorrectionAllowed(config, input));
  EXPECT_EQ(onlineAnchorCorrectionRejectionReason(config, input), "allowed");

  input.linear_speed_mps = 0.151;
  EXPECT_FALSE(onlineAnchorCorrectionAllowed(config, input));
  EXPECT_EQ(onlineAnchorCorrectionRejectionReason(config, input), "linear_speed_exceeded");

  input.linear_speed_mps = 0.10;
  input.yaw_rate_radps = 0.101;
  EXPECT_FALSE(onlineAnchorCorrectionAllowed(config, input));
  EXPECT_EQ(onlineAnchorCorrectionRejectionReason(config, input), "yaw_rate_exceeded");

  input.yaw_rate_radps = 0.05;
  input.smoothing_active = true;
  EXPECT_FALSE(onlineAnchorCorrectionAllowed(config, input));
  EXPECT_EQ(onlineAnchorCorrectionRejectionReason(config, input), "correction_already_active");
}

TEST(OnlineAnchorCorrectionPolicy, KeepsResidualBandAndSafetyBoundaryDistinct) {
  OnlineAnchorCorrectionConfig config;
  EXPECT_TRUE(onlineAnchorCorrectionResidualInRange(config, 0.30, 0.0));
  EXPECT_TRUE(onlineAnchorCorrectionResidualInRange(config, 0.0, 5.0 * M_PI / 180.0));
  EXPECT_FALSE(onlineAnchorCorrectionResidualInRange(config, 0.299, 0.0));
  EXPECT_FALSE(onlineAnchorCorrectionResidualInRange(config, 1.00, 0.0));
  EXPECT_TRUE(onlineAnchorCorrectionResidualSevere(config, 1.00, 0.0));
  EXPECT_TRUE(onlineAnchorCorrectionResidualSevere(config, 0.0, 10.0 * M_PI / 180.0));
  EXPECT_FALSE(onlineAnchorCorrectionResidualSevere(config, 0.99, 9.9 * M_PI / 180.0));
}

}  // namespace
}  // namespace localization
