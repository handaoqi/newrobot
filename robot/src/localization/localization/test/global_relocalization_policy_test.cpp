#include <gtest/gtest.h>

#include <localization/global_relocalization_policy.hpp>

namespace localization {
namespace {

TEST(GlobalRelocalizationPolicy, CurrentGenerationCanApplySuccessOrFailure) {
  EXPECT_EQ(
    decideGlobalRelocalizationResult(7, 7, true, 120.0, 10.0),
    GlobalRelocalizationResultDisposition::kApplySuccess);
  EXPECT_EQ(
    decideGlobalRelocalizationResult(7, 7, false, 120.0, 10.0),
    GlobalRelocalizationResultDisposition::kApplyFailure);
}

TEST(GlobalRelocalizationPolicy, OldGenerationNeverWritesBack) {
  EXPECT_EQ(
    decideGlobalRelocalizationResult(8, 7, true, 120.0, 10.0),
    GlobalRelocalizationResultDisposition::kDiscardStale);
}

TEST(GlobalRelocalizationPolicy, LateSuccessIsRejected) {
  EXPECT_EQ(
    decideGlobalRelocalizationResult(7, 7, true, 10001.0, 10.0),
    GlobalRelocalizationResultDisposition::kDiscardTimedOut);
  EXPECT_EQ(
    decideGlobalRelocalizationResult(7, 7, true, 10001.0, 0.0),
    GlobalRelocalizationResultDisposition::kApplySuccess);
}

TEST(GlobalRelocalizationPolicy, RequestWaitsForPoseSettleDeadline) {
  EXPECT_FALSE(globalRelocalizationRequestReady(999, 1000));
  EXPECT_TRUE(globalRelocalizationRequestReady(1000, 1000));
  EXPECT_TRUE(globalRelocalizationRequestReady(1, 0));
}

TEST(GlobalRelocalizationPolicy, ExplicitOperatorRequestMayApplyWhileAutomaticRecoveryStaysShadow) {
  EXPECT_TRUE(scanContextApplyAllowed("active", 7, 0));
  EXPECT_TRUE(scanContextApplyAllowed("shadow", 7, 7));
  EXPECT_FALSE(scanContextApplyAllowed("shadow", 8, 7));
  EXPECT_FALSE(scanContextApplyAllowed("shadow", 7, 0));
  EXPECT_FALSE(scanContextApplyAllowed("disabled", 7, 7));
}

}  // namespace
}  // namespace localization
