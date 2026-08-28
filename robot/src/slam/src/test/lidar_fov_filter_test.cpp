#include "process/lidar_process.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

livox_pcl::Point makePoint(float x, float y, float z, std::uint8_t line = 0)
{
    livox_pcl::Point point {};
    point.x         = x;
    point.y         = y;
    point.z         = z;
    point.intensity = 1.0f;
    point.timestamp = 0.0;
    point.line      = line;
    point.tag       = 0x10;
    return point;
}

pcl::PointCloud<livox_pcl::Point> cloudWith(const std::vector<livox_pcl::Point>& points)
{
    pcl::PointCloud<livox_pcl::Point> cloud;
    cloud.push_back(makePoint(0.0f, 0.0f, 0.0f));
    for (const auto& point : points)
    {
        cloud.push_back(point);
    }
    return cloud;
}

robot::slam::PointCloudType processCloud(Preprocess& preprocess, const std::vector<livox_pcl::Point>& points)
{
    robot::slam::CloudPtr output(new robot::slam::PointCloudType());
    preprocess.process(cloudWith(points), output);
    return *output;
}

bool hasPointNear(const robot::slam::PointCloudType& cloud, float x, float y, float z, float tol = 1e-4f)
{
    for (const auto& point : cloud.points)
    {
        if (std::hypot(point.x - x, point.y - y, point.z - z) <= tol)
        {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST(LidarFovFilterTest, Front180KeepsHemisphereAndDropsRear)
{
    Preprocess preprocess;
    preprocess.point_filter_num = 1;
    preprocess.blind            = 0.01;
    preprocess.setFovDegree(180.0);

    const auto kept = processCloud(preprocess, {
        makePoint(5.0f, 0.0f, 0.0f),
        makePoint(3.0f, 3.0f, 0.0f),
        makePoint(0.0f, 5.0f, 0.0f),
        makePoint(0.0f, -5.0f, 0.0f),
        makePoint(3.0f, 0.0f, 2.0f),
        makePoint(-3.0f, 3.0f, 0.0f),
        makePoint(-5.0f, 0.0f, 0.0f),
        makePoint(-1.0f, -4.0f, 0.0f),
    });

    EXPECT_EQ(kept.size(), 5u);
    EXPECT_TRUE(hasPointNear(kept, 5.0f, 0.0f, 0.0f));
    EXPECT_TRUE(hasPointNear(kept, 3.0f, 3.0f, 0.0f));
    EXPECT_TRUE(hasPointNear(kept, 0.0f, 5.0f, 0.0f));
    EXPECT_TRUE(hasPointNear(kept, 0.0f, -5.0f, 0.0f));
    EXPECT_TRUE(hasPointNear(kept, 3.0f, 0.0f, 2.0f));
    EXPECT_FALSE(hasPointNear(kept, -3.0f, 3.0f, 0.0f));
    EXPECT_FALSE(hasPointNear(kept, -5.0f, 0.0f, 0.0f));
    EXPECT_FALSE(hasPointNear(kept, -1.0f, -4.0f, 0.0f));
}

TEST(LidarFovFilterTest, Full360KeepsRearPoints)
{
    Preprocess preprocess;
    preprocess.point_filter_num = 1;
    preprocess.blind            = 0.01;
    preprocess.setFovDegree(360.0);

    const auto kept = processCloud(preprocess, {
        makePoint(5.0f, 0.0f, 0.0f),
        makePoint(-5.0f, 0.0f, 0.0f),
        makePoint(0.0f, -5.0f, 0.0f),
    });

    EXPECT_EQ(kept.size(), 3u);
    EXPECT_TRUE(hasPointNear(kept, -5.0f, 0.0f, 0.0f));
}

TEST(LidarFovFilterTest, Front240MasksOnlyRearSixtyDegrees)
{
    Preprocess preprocess;
    preprocess.point_filter_num = 1;
    preprocess.blind = 0.01;
    preprocess.setMaxRange(20.0);
    preprocess.setFovDegree(240.0);

    const auto kept = processCloud(preprocess, {
        makePoint(5.0f, 0.0f, 0.0f),
        makePoint(-2.49f, 4.33f, 0.0f),
        makePoint(-2.49f, -4.33f, 0.0f),
        makePoint(-5.0f, 0.0f, 0.0f),
    });

    EXPECT_EQ(kept.size(), 3u);
    EXPECT_FALSE(hasPointNear(kept, -5.0f, 0.0f, 0.0f));
}

TEST(LidarFovFilterTest, AppliesNearAndFarRangeClipping)
{
    Preprocess preprocess;
    preprocess.point_filter_num = 1;
    preprocess.blind = 0.5;
    preprocess.setMaxRange(10.0);
    preprocess.setFovDegree(360.0);

    const auto kept = processCloud(preprocess, {
        makePoint(0.49f, 0.0f, 0.0f),
        makePoint(0.51f, 0.0f, 0.0f),
        makePoint(9.99f, 0.0f, 0.0f),
        makePoint(10.01f, 0.0f, 0.0f),
    });

    EXPECT_EQ(kept.size(), 2u);
    EXPECT_TRUE(hasPointNear(kept, 0.51f, 0.0f, 0.0f));
    EXPECT_TRUE(hasPointNear(kept, 9.99f, 0.0f, 0.0f));
}
