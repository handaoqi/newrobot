#include "process/ground_filter.h"

#include <algorithm>
#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace robot::slam
{
    GroundFilter::GroundFilter(const GroundFilterConfig& config)
    {
        setConfig(config);
    }

    void GroundFilter::setConfig(const GroundFilterConfig& config)
    {
        config_ = config;
        config_.distance_threshold_m = std::clamp(config_.distance_threshold_m, 0.01, 0.30);
        config_.max_tilt_deg = std::clamp(config_.max_tilt_deg, 1.0, 45.0);
        config_.min_inliers = std::max(config_.min_inliers, 20);
        config_.min_sensor_height_m = std::max(config_.min_sensor_height_m, 0.05);
        config_.max_sensor_height_m = std::max(
            config_.max_sensor_height_m, config_.min_sensor_height_m + 0.10);
        config_.clearance_m = std::clamp(config_.clearance_m, 0.0, 0.30);
    }

    GroundFilterResult GroundFilter::filter(
        const CloudPtr& input,
        const Eigen::Vector3d& gravity_up_lidar,
        CloudPtr& output) const
    {
        GroundFilterResult result;
        if (!output)
            output = CloudPtr(new PointCloudType());
        output->clear();
        if (!input)
            return result;
        result.input_points = input->size();
        if (!config_.enable || input->empty() || !gravity_up_lidar.allFinite()
            || gravity_up_lidar.norm() < 1e-6)
        {
            *output = *input;
            return result;
        }

        const Eigen::Vector3d up = gravity_up_lidar.normalized();
        CloudPtr candidates(new PointCloudType());
        candidates->reserve(input->size() / 2);
        for (const auto& point : input->points)
        {
            const Eigen::Vector3d p(point.x, point.y, point.z);
            const double vertical = up.dot(p);
            if (vertical <= -config_.min_sensor_height_m
                && vertical >= -config_.max_sensor_height_m)
            {
                candidates->push_back(point);
            }
        }
        result.candidate_points = candidates->size();
        if (static_cast<int>(candidates->size()) < config_.min_inliers)
        {
            *output = *input;
            return result;
        }

        pcl::SACSegmentation<PointType> segmentation;
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setAxis(up.cast<float>());
        segmentation.setEpsAngle(config_.max_tilt_deg * M_PI / 180.0);
        segmentation.setDistanceThreshold(config_.distance_threshold_m);
        segmentation.setMaxIterations(80);
        segmentation.setInputCloud(candidates);
        segmentation.segment(inliers, coefficients);
        result.plane_inliers = inliers.indices.size();
        if (static_cast<int>(result.plane_inliers) < config_.min_inliers
            || coefficients.values.size() < 4)
        {
            *output = *input;
            return result;
        }

        Eigen::Vector3d normal(
            coefficients.values[0], coefficients.values[1], coefficients.values[2]);
        double offset = coefficients.values[3];
        const double normal_norm = normal.norm();
        if (!normal.allFinite() || !std::isfinite(offset) || normal_norm < 1e-6)
        {
            *output = *input;
            return result;
        }
        normal /= normal_norm;
        offset /= normal_norm;
        if (normal.dot(up) < 0.0)
        {
            normal = -normal;
            offset = -offset;
        }
        const double alignment = normal.dot(up);
        const double sensor_height = offset;
        if (alignment < std::cos(config_.max_tilt_deg * M_PI / 180.0)
            || sensor_height < config_.min_sensor_height_m
            || sensor_height > config_.max_sensor_height_m)
        {
            *output = *input;
            return result;
        }

        output->reserve(input->size());
        for (const auto& point : input->points)
        {
            const double signed_height = normal.dot(Eigen::Vector3d(point.x, point.y, point.z)) + offset;
            if (signed_height >= -config_.distance_threshold_m
                && signed_height <= config_.clearance_m)
            {
                ++result.removed_points;
                continue;
            }
            output->push_back(point);
        }
        output->width = static_cast<std::uint32_t>(output->size());
        output->height = 1;
        output->is_dense = input->is_dense;
        result.plane_found = true;
        result.plane << normal, offset;
        return result;
    }
}
