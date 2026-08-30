#include <gtest/gtest.h>

#include <localization/scan_match_policy.hpp>

namespace localization {
namespace {

TEST(ScanMatchPolicy, StableHighQualityNdtSkipsRefinement) {
  ScanMatchRefinePolicy policy;
  EXPECT_FALSE(shouldRunRefinement(true, true, 0.14f, policy));
  EXPECT_TRUE(shouldRunRefinement(false, true, 0.14f, policy));
  EXPECT_TRUE(shouldRunRefinement(true, true, 0.16f, policy));
  EXPECT_TRUE(shouldRunRefinement(true, false, 0.10f, policy));
}

TEST(ScanMatchPolicy, RefinementMustImproveAndRemainConsistent) {
  ScanMatchRefinePolicy policy;
  EXPECT_TRUE(shouldChooseRefinement(
    true, 0.20f, true, 0.18f, 0.10f, 0.03f, policy));
  EXPECT_FALSE(shouldChooseRefinement(
    true, 0.20f, true, 0.21f, 0.10f, 0.03f, policy));
  EXPECT_FALSE(shouldChooseRefinement(
    true, 0.20f, true, 0.18f, 0.50f, 0.03f, policy));
  EXPECT_FALSE(shouldChooseRefinement(
    true, 0.20f, true, 0.18f, 0.10f, 0.20f, policy));
}

TEST(ScanMatchPolicy, RefinementCanRecoverRejectedNdt) {
  ScanMatchRefinePolicy policy;
  EXPECT_TRUE(shouldChooseRefinement(
    false, 1.0f, true, 0.20f, 1.0f, 1.0f, policy));
  EXPECT_FALSE(shouldChooseRefinement(
    false, 1.0f, false, 0.20f, 0.0f, 0.0f, policy));
}

}  // namespace
}  // namespace localization
