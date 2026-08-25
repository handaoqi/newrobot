#include "global_factor_graph.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace robot::slam
{
namespace
{

std::vector<GlobalGraphKeyframe> stationaryKeyframes()
{
    std::vector<GlobalGraphKeyframe> keyframes(2);
    for (std::size_t index = 0; index < keyframes.size(); ++index)
    {
        keyframes[index].index = index;
        keyframes[index].stamp = static_cast<double>(index);
        keyframes[index].initial_pose = gtsam::Pose3();
        keyframes[index].initial_velocity = gtsam::Vector3::Zero();
        keyframes[index].initial_bias = gtsam::imuBias::ConstantBias();
    }
    auto& interval = keyframes[1];
    interval.has_imu_preintegration = true;
    interval.imu_bias_hat = gtsam::imuBias::ConstantBias();
    interval.imu_measurements.reserve(100);
    for (int sample = 0; sample < 100; ++sample)
    {
        GlobalGraphKeyframe::ImuMeasurement measurement;
        measurement.delta_t = 0.01;
        measurement.acceleration = gtsam::Vector3(0.0, 0.0, 9.81);
        measurement.angular_velocity = gtsam::Vector3::Zero();
        interval.imu_measurements.push_back(measurement);
    }
    return keyframes;
}

TEST(GlobalFactorGraphTest, StationaryImuCompensatesGravity)
{
    GlobalFactorGraphConfig config;
    config.use_imu_factor = true;
    config.max_iterations = 30;
    GlobalFactorGraph graph(config);

    const auto result = graph.optimize(stationaryKeyframes(), {});

    ASSERT_TRUE(result.success) << result.error;
    ASSERT_EQ(result.optimized_poses.size(), 2U);
    EXPECT_EQ(result.ndt_factor_count, 1U);
    EXPECT_EQ(result.imu_factor_count, 1U);
    EXPECT_EQ(result.imu_bias_factor_count, 1U);
    EXPECT_EQ(result.loop_closure_factor_count, 0U);
    EXPECT_NEAR(result.optimized_poses.back().translation().norm(), 0.0, 1e-5);
    EXPECT_NEAR(result.final_velocity.norm(), 0.0, 1e-5);
    EXPECT_NEAR(result.final_bias.vector().norm(), 0.0, 1e-5);
}

TEST(GlobalFactorGraphTest, CanDisableImuStateExplicitly)
{
    GlobalFactorGraphConfig config;
    config.use_imu_factor = false;
    GlobalFactorGraph graph(config);

    const auto result = graph.optimize(stationaryKeyframes(), {});

    ASSERT_TRUE(result.success) << result.error;
    EXPECT_EQ(result.imu_factor_count, 0U);
    EXPECT_EQ(result.imu_bias_factor_count, 0U);
    EXPECT_EQ(result.imu_velocity_prior_factor_count, 0U);
    EXPECT_EQ(result.factor_count, 2U);
}

}  // namespace
}  // namespace robot::slam
