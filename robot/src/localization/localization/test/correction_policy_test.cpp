#include <gtest/gtest.h>

#include <localization/correction_policy.hpp>

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

}  // namespace
}  // namespace localization
