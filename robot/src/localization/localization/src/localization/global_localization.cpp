#include "localization/global_localization.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cmath>

namespace localization {

    GlobalLocalization::GlobalLocalization() = default;

GlobalLocalization::~GlobalLocalization() {}

void GlobalLocalization::init(const Eigen::Matrix4d& initial_pose) {
    initial_pose_ = initial_pose;
}

bool GlobalLocalization::performGlobalLocalization(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& global_map,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& current_cloud,
    const Eigen::Matrix4d& initial_trans,  
    Eigen::Matrix4d& final_pose
) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_ds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(current_cloud);
    voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f); 
    voxel_filter.filter(*current_cloud_filtered);

    pcl::transformPointCloud(*current_cloud_filtered, *current_cloud_ds, initial_trans.cast<float>());

    // ICP 
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputTarget(global_map);
    icp.setInputSource(current_cloud_ds);
    // Global localization is a startup fallback, not a continuous tracker.
    // Keep the one-shot search bounded so a bad initial pose cannot block the
    // lidar callback for an extended period.
    icp.setMaximumIterations(30);
    icp.setTransformationEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    Eigen::Matrix4f T_corr_current;
    double fitness_score;
    icp.align(aligned_cloud);
    T_corr_current = icp.getFinalTransformation();
    fitness_score = icp.getFitnessScore();
    RCLCPP_INFO(logger_, "ICP fitness score 1: %.6f", fitness_score);

    pcl::transformPointCloud(*current_cloud_ds, *current_cloud_ds, T_corr_current);


    // This routine is invoked as a bounded one-shot initializer. Requiring a
    // second callback made successful matches impossible after the caller's
    // retry gate was consumed. Use the same fitness scale as local NDT and
    // reject non-finite or unconverged ICP results here.
    // The first pass is only a coarse seed for the second ICP pass. Keep this
    // gate looser than the final acceptance threshold so borderline seeds can
    // still be refined.
    if (!icp.hasConverged() || !std::isfinite(fitness_score) || fitness_score > 1.0) {
        return false;
    }

    // 第二次 ICP 
    icp.align(aligned_cloud);
    Eigen::Matrix4f T_corr_second = icp.getFinalTransformation();
    fitness_score = icp.getFitnessScore();
    RCLCPP_INFO(logger_, "ICP fitness score 2: %.6f", fitness_score);
    pcl::transformPointCloud(*current_cloud_ds, *current_cloud_ds, T_corr_second);

    if (!icp.hasConverged() || !std::isfinite(fitness_score) || fitness_score > 0.5) {
        return false;
    }

    // ICP aligns the cloud after it has already been transformed by the
    // supplied initial pose. Preserve that initial map-frame transform when
    // returning the refined pose.
    Eigen::Matrix4d T_corr_final =
        T_corr_second.cast<double>() * T_corr_current.cast<double>() * initial_trans;

    final_pose = T_corr_final;
    return true;
}
} // namespace localization
