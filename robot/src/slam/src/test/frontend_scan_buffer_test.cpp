#include "frontend_scan_buffer.h"

#include <gtest/gtest.h>

#include <deque>

namespace robot::slam
{
    TEST(FrontendScanBufferTest, NavigationDropsAllPendingScansBeforeAddingLatest)
    {
        std::deque<TimedLidarScan<int>> scans {
            { 1.0, 1 },
            { 2.0, 2 },
            { 3.0, 3 },
        };

        discardSupersededNavigationScans(scans, false);
        scans.push_back({ 4.0, 4 });

        ASSERT_EQ(scans.size(), 1U);
        EXPECT_DOUBLE_EQ(scans.front().stamp, 4.0);
        EXPECT_EQ(scans.front().cloud, 4);
    }

    TEST(FrontendScanBufferTest, NavigationPreservesInFlightScanAndReplacesTrailingScans)
    {
        std::deque<TimedLidarScan<int>> scans {
            { 1.0, 1 },
            { 2.0, 2 },
            { 3.0, 3 },
        };

        discardSupersededNavigationScans(scans, true);
        scans.push_back({ 4.0, 4 });

        ASSERT_EQ(scans.size(), 2U);
        EXPECT_DOUBLE_EQ(scans.front().stamp, 1.0);
        EXPECT_EQ(scans.front().cloud, 1);
        EXPECT_DOUBLE_EQ(scans.back().stamp, 4.0);
        EXPECT_EQ(scans.back().cloud, 4);
    }

    TEST(FrontendScanBufferTest, WorldCloudIsBuiltOnlyForPublishOrKeyframeRecording)
    {
        EXPECT_FALSE(needsWorldPointCloud(false, false, false));
        EXPECT_FALSE(needsWorldPointCloud(false, true, false));
        EXPECT_FALSE(needsWorldPointCloud(false, false, true));
        EXPECT_TRUE(needsWorldPointCloud(false, true, true));
        EXPECT_TRUE(needsWorldPointCloud(true, false, false));
    }
}  // namespace robot::slam
