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
    EXPECT_EQ(result.lio_between_factor_count, 1U);
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

TEST(GlobalFactorGraphTest, CanDisableLoopFactors)
{
    GlobalFactorGraphConfig config;
    config.use_imu_factor = false;
    config.use_loop = false;
    GlobalFactorGraph graph(config);

    auto keyframes = stationaryKeyframes();
    keyframes[1].initial_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));

    GlobalGraphLoopClosure loop;
    loop.from = 0;
    loop.to = 1;
    loop.relative_pose = gtsam::Pose3();
    loop.covariance = gtsam::Matrix6::Identity() * 0.01;

    const auto disabled = graph.optimize(keyframes, {loop});
    ASSERT_TRUE(disabled.success) << disabled.error;
    EXPECT_EQ(disabled.loop_closure_factor_count, 0U);

    config.use_loop = true;
    GlobalFactorGraph enabled(config);
    const auto applied = enabled.optimize(keyframes, {loop});
    ASSERT_TRUE(applied.success) << applied.error;
    EXPECT_EQ(applied.loop_closure_factor_count, 1U);
}

TEST(GlobalFactorGraphTest, RejectsImuIntervalWithInconsistentEndpointVelocity)
{
    GlobalFactorGraphConfig config;
    config.use_imu_factor = true;
    config.imu_kinematic_gate_max_residual_mps = 0.25;
    GlobalFactorGraph graph(config);

    auto keyframes = stationaryKeyframes();
    keyframes[1].initial_velocity = gtsam::Vector3(1.0, 0.0, 0.0);

    const auto result = graph.optimize(keyframes, {});

    ASSERT_TRUE(result.success) << result.error;
    EXPECT_EQ(result.imu_factor_count, 0U);
    EXPECT_EQ(result.imu_kinematic_rejected_factor_count, 1U);
    EXPECT_EQ(result.imu_velocity_prior_factor_count, 2U);
    EXPECT_NEAR(result.max_imu_kinematic_residual_mps, 0.5, 1e-9);
    EXPECT_NEAR(result.optimized_poses.back().translation().norm(), 0.0, 1e-5);
}

TEST(GlobalFactorGraphTest, RtkXyPullKeepsLioZAndAcceptsMetreJump)
{
    GlobalFactorGraphConfig config;
    config.use_imu_factor = false;
    config.use_loop = false;
    config.max_pose_jump_m = 25.0;
    config.max_abs_z_change_m = 1.5;
    GlobalFactorGraph graph(config);

    const std::size_t count = 12;
    std::vector<GlobalGraphKeyframe> keyframes(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        const double t = static_cast<double>(i);
        const double frac = t / static_cast<double>(count - 1);
        keyframes[i].index = i;
        keyframes[i].stamp = t;
        const double lio_z = (i + 1 == count) ? 1.0 : 0.10 * t;
        keyframes[i].initial_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(t, 0.0, lio_z));
        keyframes[i].has_rtk_position = true;
        keyframes[i].rtk_position = gtsam::Point3(t + 2.20 * frac, -1.80 * frac, 10.0);
        keyframes[i].rtk_position_sigma = 0.05;
    }

    const auto result = graph.optimize(keyframes, {});

    ASSERT_TRUE(result.success) << result.error;
    EXPECT_EQ(result.rtk_position_factor_count, count);
    EXPECT_NEAR(result.optimized_poses.front().x(), 0.0, 0.05);
    EXPECT_NEAR(result.optimized_poses.front().y(), 0.0, 0.05);
    EXPECT_NEAR(result.optimized_poses.back().z(), 1.0, 1e-6);
    EXPECT_GT(result.optimized_poses.back().x(), 12.0);
    EXPECT_LT(result.optimized_poses.back().y(), -0.8);
}

}  // namespace
}  // namespace robot::slam
