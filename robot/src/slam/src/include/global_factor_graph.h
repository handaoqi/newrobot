#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include <cstddef>
#include <string>
#include <vector>

namespace robot::slam
{

struct GlobalGraphLoopClosure
{
    std::size_t from = 0;
    std::size_t to = 0;
    gtsam::Pose3 relative_pose;
    gtsam::Matrix6 covariance = gtsam::Matrix6::Identity();
    double score = 0.0;
};

struct GlobalGraphKeyframe
{
    std::size_t index = 0;
    double stamp = 0.0;
    gtsam::Pose3 initial_pose;

    bool has_rtk_position = false;
    gtsam::Point3 rtk_position = gtsam::Point3(0.0, 0.0, 0.0);
    double rtk_position_sigma = 1.0;
    gtsam::Point3 rtk_lever_arm = gtsam::Point3(0.0, 0.0, 0.0);

    bool has_rtk_heading = false;
    double rtk_heading_rad = 0.0;
    double rtk_heading_sigma_rad = 0.15;

    bool has_imu_delta = false;
    gtsam::Pose3 imu_delta;
    gtsam::Matrix6 imu_covariance = gtsam::Matrix6::Identity();
};

struct GlobalFactorGraphConfig
{
    bool enable = true;
    int max_iterations = 80;
    double prior_translation_sigma = 0.01;
    double prior_rotation_sigma_rad = 0.01;
    double ndt_translation_sigma = 0.15;
    double ndt_rotation_sigma_rad = 0.08;
    double imu_translation_sigma = 0.20;
    double imu_rotation_sigma_rad = 0.12;
    double rtk_position_sigma_floor = 0.20;
    double rtk_heading_sigma_floor_rad = 0.035;
    double loop_translation_sigma = 0.10;
    double loop_rotation_sigma_rad = 0.08;
    double robust_huber_k = 1.345;
};

struct GlobalFactorGraphResult
{
    bool success = false;
    std::string error;
    std::vector<gtsam::Pose3> optimized_poses;
    std::vector<gtsam::Matrix6> covariances;
    std::size_t factor_count = 0;
    std::size_t ndt_factor_count = 0;
    std::size_t imu_factor_count = 0;
    std::size_t rtk_position_factor_count = 0;
    std::size_t rtk_heading_factor_count = 0;
    std::size_t loop_closure_factor_count = 0;
    double error_before = 0.0;
    double error_after = 0.0;
};

class GlobalFactorGraph
{
public:
    explicit GlobalFactorGraph(GlobalFactorGraphConfig config = {});

    GlobalFactorGraphResult optimize(
        const std::vector<GlobalGraphKeyframe>& keyframes,
        const std::vector<GlobalGraphLoopClosure>& loop_closures) const;

private:
    GlobalFactorGraphConfig config_;
};

}  // namespace robot::slam
