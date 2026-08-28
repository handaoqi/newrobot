#include "process/ground_filter.h"

#include <gtest/gtest.h>

#include <cmath>

namespace robot::slam
{
    namespace
    {
        PointType point(float x, float y, float z)
        {
            PointType value;
            value.x = x;
            value.y = y;
            value.z = z;
            value.intensity = 1.0f;
            return value;
        }
    }

    TEST(GroundFilterTest, RemovesGroundAndLowLegButKeepsWall)
    {
        GroundFilterConfig config;
        config.min_inliers = 50;
        GroundFilter filter(config);
        CloudPtr input(new PointCloudType());
        for (int x = -5; x <= 5; ++x)
        {
            for (int y = -5; y <= 5; ++y)
                input->push_back(point(0.25f * x, 0.25f * y, -0.50f));
        }
        for (int z = -4; z <= 8; ++z)
            input->push_back(point(2.0f, 0.5f, 0.10f * z));
        input->push_back(point(1.0f, 0.0f, -0.40f));

        CloudPtr output(new PointCloudType());
        const auto result = filter.filter(input, Eigen::Vector3d::UnitZ(), output);

        ASSERT_TRUE(result.plane_found);
        EXPECT_GE(result.plane_inliers, 100u);
        EXPECT_GT(result.removed_points, 100u);
        EXPECT_LT(output->size(), input->size());
        bool wall_kept = false;
        bool low_leg_kept = false;
        for (const auto& value : output->points)
        {
            wall_kept = wall_kept || (std::abs(value.x - 2.0f) < 1e-4f && value.z > 0.0f);
            low_leg_kept = low_leg_kept || (std::abs(value.x - 1.0f) < 1e-4f
                && std::abs(value.z + 0.40f) < 1e-4f);
        }
        EXPECT_TRUE(wall_kept);
        EXPECT_FALSE(low_leg_kept);
    }

    TEST(GroundFilterTest, FailsOpenWithoutEnoughGround)
    {
        GroundFilterConfig config;
        config.min_inliers = 50;
        GroundFilter filter(config);
        CloudPtr input(new PointCloudType());
        for (int i = 0; i < 20; ++i)
            input->push_back(point(2.0f, 0.1f * i, 0.1f * i));

        CloudPtr output(new PointCloudType());
        const auto result = filter.filter(input, Eigen::Vector3d::UnitZ(), output);

        EXPECT_FALSE(result.plane_found);
        EXPECT_EQ(output->size(), input->size());
    }
}
