#pragma once

#include "common.h"

#include <Eigen/Core>

namespace robot::slam
{
    struct GroundFilterConfig
    {
        bool   enable = true;
        double distance_threshold_m = 0.08;
        double max_tilt_deg = 20.0;
        int    min_inliers = 80;
        double min_sensor_height_m = 0.20;
        double max_sensor_height_m = 1.20;
        double clearance_m = 0.15;
    };

    struct GroundFilterResult
    {
        bool        plane_found = false;
        std::size_t input_points = 0;
        std::size_t candidate_points = 0;
        std::size_t plane_inliers = 0;
        std::size_t removed_points = 0;
        Eigen::Vector4d plane = Eigen::Vector4d::Zero();
    };

    class GroundFilter
    {
    public:
        explicit GroundFilter(const GroundFilterConfig& config = {});

        void setConfig(const GroundFilterConfig& config);

        GroundFilterResult filter(
            const CloudPtr& input,
            const Eigen::Vector3d& gravity_up_lidar,
            CloudPtr& output) const;

    private:
        GroundFilterConfig config_;
    };
}
