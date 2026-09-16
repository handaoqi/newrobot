#include <gtest/gtest.h>

#include <localization/rtk_primary_policy.hpp>

namespace localization {
namespace {

TEST(RtkPrimaryPolicy, PositionGoodDoesNotRequireHeading) {
  EXPECT_TRUE(rtkPositionGoodForNavigation(true, "fixed"));
  EXPECT_FALSE(rtkPositionGoodForNavigation(true, "float"));
  EXPECT_FALSE(rtkPositionGoodForNavigation(false, "fixed"));
  EXPECT_TRUE(rtkGoodForPrimaryDrive(true, true, "fixed"));
  EXPECT_FALSE(rtkGoodForPrimaryDrive(true, false, "fixed"));
}

TEST(RtkPrimaryPolicy, DoesNotLatchUntilPromoteSamples) {
  RtkPrimaryLatchState state;
  const RtkPrimaryLatchConfig config{2, 3};
  updateRtkPrimaryLatch(state, config, true);
  EXPECT_FALSE(state.latched);
  EXPECT_EQ(state.good_frames, 1);
  updateRtkPrimaryLatch(state, config, true);
  EXPECT_TRUE(state.latched);
  EXPECT_EQ(state.good_frames, 2);
}

TEST(RtkPrimaryPolicy, KeepsLatchThroughBriefRtkDrops) {
  RtkPrimaryLatchState state;
  const RtkPrimaryLatchConfig config{2, 3};
  updateRtkPrimaryLatch(state, config, true);
  updateRtkPrimaryLatch(state, config, true);
  ASSERT_TRUE(state.latched);
  updateRtkPrimaryLatch(state, config, false);
  updateRtkPrimaryLatch(state, config, false);
  EXPECT_TRUE(state.latched);
  updateRtkPrimaryLatch(state, config, false);
  EXPECT_FALSE(state.latched);
}

TEST(RtkPrimaryPolicy, DefaultConfigNeedsSustainedHeadingBeforePromote) {
  RtkPrimaryLatchState state;
  const RtkPrimaryLatchConfig config;
  for (int i = 0; i < config.promote_samples - 1; ++i) {
    updateRtkPrimaryLatch(state, config, true);
    EXPECT_FALSE(state.latched);
  }
  updateRtkPrimaryLatch(state, config, true);
  EXPECT_TRUE(state.latched);
}

TEST(RtkPrimaryPolicy, RelatchesAfterDemoteWhenRtkRecovers) {
  RtkPrimaryLatchState state;
  const RtkPrimaryLatchConfig config{2, 3};
  updateRtkPrimaryLatch(state, config, true);
  updateRtkPrimaryLatch(state, config, true);
  updateRtkPrimaryLatch(state, config, false);
  updateRtkPrimaryLatch(state, config, false);
  updateRtkPrimaryLatch(state, config, false);
  ASSERT_FALSE(state.latched);
  updateRtkPrimaryLatch(state, config, true);
  EXPECT_FALSE(state.latched);
  updateRtkPrimaryLatch(state, config, true);
  EXPECT_TRUE(state.latched);
}

TEST(RtkPrimaryPolicy, DrivesPoseWhenArbiterIsOnAndNotBridging) {
  EXPECT_TRUE(rtkPrimaryShouldDrive(true, false, true));
  EXPECT_FALSE(rtkPrimaryShouldDrive(false, false, true));
  EXPECT_FALSE(rtkPrimaryShouldDrive(true, true, true));
  EXPECT_FALSE(rtkPrimaryShouldDrive(true, false, false));
}


TEST(RtkPrimaryPolicy, HeadingTrustedWithinGateWithoutSelfStable) {
  EXPECT_TRUE(rtkHeadingTrustedForCorrection(true, 0.2f, 0.52f, false));
  EXPECT_FALSE(rtkHeadingTrustedForCorrection(true, 0.70f, 0.52f, false));
  EXPECT_FALSE(rtkHeadingTrustedForCorrection(false, 0.1f, 0.52f, false));
}

TEST(RtkPrimaryPolicy, SelfStableDoesNotBypassYawGate) {
  EXPECT_TRUE(rtkHeadingTrustedForCorrection(true, 0.20f, 0.52f, true));
  EXPECT_FALSE(rtkHeadingTrustedForCorrection(true, 1.05f, 0.52f, true));
  EXPECT_FALSE(rtkHeadingTrustedForCorrection(false, 0.20f, 0.52f, true));
}

TEST(RtkPrimaryPolicy, SelfStableDoesNotTrustHeadingFlip) {
  EXPECT_TRUE(rtkHeadingIsFlip(3.14f));
  EXPECT_FALSE(rtkHeadingIsFlip(1.05f));
  EXPECT_FALSE(rtkHeadingTrustedForCorrection(true, 3.14f, 0.52f, true));
  EXPECT_FALSE(rtkHeadingTrustedForCorrection(true, 2.0f, 0.52f, true));
}

TEST(RtkPrimaryPolicy, CruiseDoesNotTakeDualAntennaHeading) {
  EXPECT_TRUE(rtkHeadingAllowedForCorrection(false));
  EXPECT_FALSE(rtkHeadingAllowedForCorrection(true));
  EXPECT_TRUE(rtkHeadingTrustedForCorrection(true, 0.21f, 0.52f, true));
  EXPECT_FALSE(
    rtkHeadingAllowedForCorrection(true) &&
    rtkHeadingTrustedForCorrection(true, 0.21f, 0.52f, true));
}

TEST(RtkPrimaryPolicy, DoesNotPromoteWhileMoving) {
  RtkPrimaryLatchState state;
  const RtkPrimaryLatchConfig config{2, 3};
  updateRtkPrimaryLatch(state, config, true, true);
  updateRtkPrimaryLatch(state, config, true, true);
  EXPECT_FALSE(state.latched);
  EXPECT_EQ(state.good_frames, 2);
  updateRtkPrimaryLatch(state, config, true, false);
  EXPECT_TRUE(state.latched);
}

TEST(RtkPrimaryPolicy, DemotesWhileMovingWhenGpsDrops) {
  RtkPrimaryLatchState state;
  const RtkPrimaryLatchConfig config{2, 3};
  updateRtkPrimaryLatch(state, config, true, false);
  updateRtkPrimaryLatch(state, config, true, false);
  ASSERT_TRUE(state.latched);
  updateRtkPrimaryLatch(state, config, false, true);
  updateRtkPrimaryLatch(state, config, false, true);
  EXPECT_TRUE(state.latched);
  updateRtkPrimaryLatch(state, config, false, true);
  EXPECT_FALSE(state.latched);
}
}  // namespace
}  // namespace localization
