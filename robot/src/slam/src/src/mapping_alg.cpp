/**
 * @file mapping_alg.cpp
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#include "mapping_alg.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <sys/statvfs.h>
#include <unordered_set>
#include <Eigen/Geometry>

namespace robot::slam
{
    namespace
    {
#pragma pack(push, 1)
        struct BinaryPcdPoint
        {
            float x;
            float y;
            float z;
            float intensity;
            float normal_x;
            float normal_y;
            float normal_z;
            float curvature;
        };
#pragma pack(pop)

        static_assert(sizeof(BinaryPcdPoint) == 32, "unexpected binary PCD point size");

#pragma pack(push, 1)
        struct ExportPointRecord
        {
            BinaryPcdPoint point;
            float reference_z;
        };
#pragma pack(pop)

        static_assert(sizeof(ExportPointRecord) == 36, "unexpected export point record size");

        BinaryPcdPoint packPoint(const PointType& point)
        {
            return BinaryPcdPoint{
                point.x,
                point.y,
                point.z,
                point.intensity,
                point.normal_x,
                point.normal_y,
                point.normal_z,
                point.curvature
            };
        }

        std::string jsonEscape(const std::string& value)
        {
            std::ostringstream escaped;
            for (const char ch : value)
            {
                switch (ch)
                {
                    case '\\': escaped << "\\\\"; break;
                    case '"': escaped << "\\\""; break;
                    case '\n': escaped << "\\n"; break;
                    case '\r': escaped << "\\r"; break;
                    case '\t': escaped << "\\t"; break;
                    default: escaped << ch; break;
                }
            }
            return escaped.str();
        }

        Eigen::Quaterniond normalizedXyzw(double x, double y, double z, double w)
        {
            Eigen::Quaterniond quaternion(w, x, y, z);
            const double norm = quaternion.norm();
            if (norm > 1e-12)
                quaternion.coeffs() /= norm;
            else
                quaternion = Eigen::Quaterniond::Identity();
            return quaternion;
        }

        Eigen::Quaterniond rotationToXyzw(const Mat3d& rotation)
        {
            const Eigen::Quaterniond quaternion(rotation);
            return normalizedXyzw(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        }

        Eigen::Isometry3d optimizedLidarPose(const MappingKeyframe& keyframe,
            const gtsam::Pose3& optimized_imu_pose)
        {
            const Eigen::Isometry3d raw_imu = Eigen::Translation3d(keyframe.world_origin)
                * normalizedXyzw(keyframe.world_qx, keyframe.world_qy, keyframe.world_qz, keyframe.world_qw);
            const Eigen::Isometry3d raw_lidar = Eigen::Translation3d(keyframe.lidar_origin)
                * normalizedXyzw(keyframe.lidar_qx, keyframe.lidar_qy, keyframe.lidar_qz, keyframe.lidar_qw);
            const auto q = optimized_imu_pose.rotation().toQuaternion();
            const Eigen::Isometry3d optimized_imu = Eigen::Translation3d(
                optimized_imu_pose.x(), optimized_imu_pose.y(), optimized_imu_pose.z())
                * Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
            return optimized_imu * raw_imu.inverse() * raw_lidar;
        }

        std::uint64_t residentSetBytes()
        {
            std::ifstream status("/proc/self/statm");
            std::uint64_t pages = 0;
            std::uint64_t resident = 0;
            if (status >> pages >> resident)
                return resident * static_cast<std::uint64_t>(::sysconf(_SC_PAGESIZE));
            return 0;
        }

        std::uint64_t freeDiskBytes(const std::string& path)
        {
            struct statvfs stats {};
            if (::statvfs(path.c_str(), &stats) == 0)
                return static_cast<std::uint64_t>(stats.f_bavail) * static_cast<std::uint64_t>(stats.f_frsize);
            return 0;
        }

        std::size_t binaryPcdPointCount(const std::string& path)
        {
            std::ifstream input(path, std::ios::binary);
            std::string line;
            while (std::getline(input, line))
            {
                if (line.rfind("POINTS ", 0) == 0)
                {
                    try
                    {
                        return static_cast<std::size_t>(std::stoull(line.substr(7)));
                    }
                    catch (const std::exception&)
                    {
                        return 0;
                    }
                }
                if (line.rfind("DATA ", 0) == 0)
                    break;
            }
            return 0;
        }

    }

    DynamicFilterVoxelKey makeDynamicFilterVoxelKey(const PointType& point, double voxel_size)
    {
        return DynamicFilterVoxelKey{
            static_cast<int>(std::floor(point.x / voxel_size)),
            static_cast<int>(std::floor(point.y / voxel_size)),
            static_cast<int>(std::floor(point.z / voxel_size))
        };
    }

    std::string makeMapSubdir(const std::string& data_path)
    {
        auto now = std::chrono::system_clock::now();
        auto now_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

        std::ostringstream timestamp_ss;
        timestamp_ss << std::put_time(std::localtime(&now_t), "%Y%m%d_%H%M%S")
                     << "_" << std::setw(3) << std::setfill('0') << ms;

        std::string base = data_path + "/" + timestamp_ss.str();
        std::string candidate = base;
        int suffix = 1;
        while (std::filesystem::exists(candidate))
        {
            candidate = base + "_" + std::to_string(suffix++);
        }
        return candidate;
    }

    MappingAlg::MappingAlg(const rclcpp::NodeOptions& options)
        : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.world_points_en", true);
        this->declare_parameter<bool>("publish.body_points_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<string>("common.gnss_topic", "/fix");
        this->declare_parameter<string>("common.rtk_pvh_topic", "/rtk_pvh");
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<int>("imu_init.sample_count", 600);
        this->declare_parameter<double>("imu_init.max_acc_variance", 0.5);
        this->declare_parameter<double>("imu_init.max_gyro_variance", 0.05);
        this->declare_parameter<bool>("gnss_fusion.enable", false);
        this->declare_parameter<string>("gnss_fusion.origin_file", "/home/dogrobot/runtime/nx-edge/data/jszr/map/gnss_origin.yaml");
        this->declare_parameter<double>("gnss_fusion.gain", 0.03);
        this->declare_parameter<double>("gnss_fusion.max_correction_step", 0.10);
        this->declare_parameter<double>("gnss_fusion.max_residual", 5.0);
        this->declare_parameter<double>("gnss_fusion.max_age", 1.5);
        this->declare_parameter<double>("gnss_fusion.max_horizontal_std", 1.5);
        this->declare_parameter<int>("gnss_fusion.min_status", 0);
        this->declare_parameter<bool>("gnss_fusion.use_elevation", false);
        this->declare_parameter<vector<double>>("gnss_fusion.lever_arm_base", vector<double>({ -0.05, 0.0, 0.15 }));
        this->declare_parameter<int>("gnss_fusion.alignment_min_samples", 20);
        this->declare_parameter<double>("gnss_fusion.alignment_min_baseline", 15.0);
        this->declare_parameter<double>("gnss_fusion.alignment_max_rms", 1.5);
        this->declare_parameter<double>("gnss_fusion.alignment_max_yaw_change_deg", 3.0);
        this->declare_parameter<int>("gnss_fusion.alignment_required_fits", 3);
        this->declare_parameter<bool>("gnss_fusion.use_heading", true);
        this->declare_parameter<double>("gnss_fusion.heading_offset_deg", 0.0);
        this->declare_parameter<double>("gnss_fusion.heading_min_baseline_m", 0.20);
        this->declare_parameter<double>("gnss_fusion.heading_max_std_deg", 5.0);
        this->declare_parameter<double>("gnss_fusion.heading_max_age", 1.5);
        this->declare_parameter<bool>("odom_guard.enable", false);
        this->declare_parameter<string>("odom_guard.topic", "/odom/mc_odom");
        this->declare_parameter<double>("odom_guard.max_speed", 2.0);
        this->declare_parameter<double>("odom_guard.max_vertical_speed", 0.10);
        this->declare_parameter<double>("odom_guard.max_lidar_correction", 0.35);
        this->declare_parameter<double>("odom_guard.max_lidar_z_correction", 0.03);
        this->declare_parameter<double>("odom_guard.max_lidar_rotation", 0.35);
        this->declare_parameter<double>("odom_guard.max_abs_z_from_start", 10.0);
        this->declare_parameter<double>("health_guard.max_frame_translation", 1.5);
        this->declare_parameter<double>("health_guard.max_speed", 3.0);
        this->declare_parameter<double>("health_guard.max_abs_z", 5.0);
        this->declare_parameter<double>("health_guard.max_frame_z", 1.0);
        this->declare_parameter<double>("health_guard.warn_speed", 1.8);
        this->declare_parameter<double>("health_guard.warn_abs_z", 0.5);
        this->declare_parameter<int>("health_guard.pose_guard_frames", 3);
        this->declare_parameter<int>("health_guard.no_effective_points_limit", 10);

        this->declare_parameter<string>("pcd2pgm.file_name", "map");
        this->declare_parameter<double>("pcd2pgm.thre_z_min", 0.2);
        this->declare_parameter<double>("pcd2pgm.thre_z_max", 2.0);
        this->declare_parameter<int>("pcd2pgm.flag_pass_through", 0);
        this->declare_parameter<double>("pcd2pgm.map_resolution", 0.05);
        this->declare_parameter<std::int64_t>("pcd2pgm.max_grid_cells", 200000000);
        this->declare_parameter<int>("pcd2pgm.min_points_per_cell", 1);
        this->declare_parameter<int>("pcd2pgm.support_radius_cells", 0);
        this->declare_parameter<double>("pcd2pgm.projection_padding_m", 25.0);
        this->declare_parameter<bool>("dynamic_filter.enable", false);
        this->declare_parameter<double>("dynamic_filter.voxel_size", 0.20);
        this->declare_parameter<int>("dynamic_filter.min_scan_observations", 1);
        this->declare_parameter<int>("dynamic_filter.shard_count", 64);
        this->declare_parameter<bool>("keyframe_record.enable", true);
        this->declare_parameter<double>("keyframe_record.min_distance_m", 0.8);
        this->declare_parameter<double>("keyframe_record.min_yaw_rad", 0.35);
        this->declare_parameter<double>("keyframe_record.max_interval_s", 2.0);
        this->declare_parameter<double>("keyframe_record.voxel_size_m", 0.25);
        this->declare_parameter<int>("keyframe_record.max_queue_size", 8);
        this->declare_parameter<bool>("global_optimization.enable", true);
        this->declare_parameter<int>("global_optimization.max_iterations", 80);
        this->declare_parameter<double>("global_optimization.prior_translation_sigma", 0.01);
        this->declare_parameter<double>("global_optimization.prior_rotation_sigma_rad", 0.01);
        this->declare_parameter<double>("global_optimization.ndt_translation_sigma", 0.15);
        this->declare_parameter<double>("global_optimization.ndt_rotation_sigma_rad", 0.08);
        this->declare_parameter<double>("global_optimization.imu_translation_sigma", 0.20);
        this->declare_parameter<double>("global_optimization.imu_rotation_sigma_rad", 0.12);
        this->declare_parameter<double>("global_optimization.rtk_position_sigma_floor", 0.20);
        this->declare_parameter<double>("global_optimization.rtk_heading_sigma_floor_rad", 0.035);
        this->declare_parameter<double>("global_optimization.loop_translation_sigma", 0.10);
        this->declare_parameter<double>("global_optimization.loop_rotation_sigma_rad", 0.08);
        this->declare_parameter<double>("global_optimization.robust_huber_k", 1.345);
        this->declare_parameter<string>("storage.data_path", "");

        this->get_parameter_or<string>("pcd2pgm.file_name", pcd2pgm_options_.file_name, "map");
        this->get_parameter_or<double>("pcd2pgm.thre_z_min", pcd2pgm_options_.thre_z_min, 0.2);
        this->get_parameter_or<double>("pcd2pgm.thre_z_max", pcd2pgm_options_.thre_z_max, 2.0);
        this->get_parameter_or<int>("pcd2pgm.flag_pass_through", pcd2pgm_options_.flag_pass_through, 0);
        this->get_parameter_or<double>("pcd2pgm.map_resolution", pcd2pgm_options_.map_resolution, 0.05);
        std::int64_t max_grid_cells = 200000000;
        this->get_parameter_or<std::int64_t>("pcd2pgm.max_grid_cells", max_grid_cells, 200000000);
        pcd2pgm_options_.max_grid_cells = static_cast<std::size_t>(std::max<std::int64_t>(1, max_grid_cells));
        int min_points_per_cell = 1;
        this->get_parameter_or<int>("pcd2pgm.min_points_per_cell", min_points_per_cell, 1);
        pcd2pgm_options_.min_points_per_cell = static_cast<std::uint8_t>(std::clamp(min_points_per_cell, 1, 255));
        int support_radius_cells = 0;
        this->get_parameter_or<int>("pcd2pgm.support_radius_cells", support_radius_cells, 0);
        pcd2pgm_options_.support_radius_cells = static_cast<std::uint8_t>(std::clamp(support_radius_cells, 0, 8));
        this->get_parameter_or<double>("pcd2pgm.projection_padding_m", pcd2pgm_projection_padding_m_, 25.0);
        this->get_parameter_or<bool>("dynamic_filter.enable", dynamic_filter_enable_, false);
        this->get_parameter_or<double>("dynamic_filter.voxel_size", dynamic_filter_voxel_size_, 0.20);
        this->get_parameter_or<int>("dynamic_filter.min_scan_observations", dynamic_filter_min_scan_observations_, 1);
        int dynamic_filter_shard_count = 64;
        this->get_parameter_or<int>("dynamic_filter.shard_count", dynamic_filter_shard_count, 64);
        dynamic_filter_shard_count_ = static_cast<std::size_t>(std::clamp(dynamic_filter_shard_count, 8, 512));
        this->get_parameter_or<bool>("keyframe_record.enable", keyframe_record_enable_, true);
        this->get_parameter_or<double>("keyframe_record.min_distance_m", keyframe_min_distance_m_, 0.8);
        this->get_parameter_or<double>("keyframe_record.min_yaw_rad", keyframe_min_yaw_rad_, 0.35);
        this->get_parameter_or<double>("keyframe_record.max_interval_s", keyframe_max_interval_s_, 2.0);
        this->get_parameter_or<double>("keyframe_record.voxel_size_m", keyframe_voxel_size_m_, 0.25);
        int keyframe_max_queue_size = 8;
        this->get_parameter_or<int>("keyframe_record.max_queue_size", keyframe_max_queue_size, 8);
        keyframe_max_queue_size_ = static_cast<std::size_t>(std::max(1, keyframe_max_queue_size));
        this->get_parameter_or<bool>("global_optimization.enable", global_factor_graph_config_.enable, true);
        this->get_parameter_or<int>("global_optimization.max_iterations", global_factor_graph_config_.max_iterations, 80);
        this->get_parameter_or<double>("global_optimization.prior_translation_sigma", global_factor_graph_config_.prior_translation_sigma, 0.01);
        this->get_parameter_or<double>("global_optimization.prior_rotation_sigma_rad", global_factor_graph_config_.prior_rotation_sigma_rad, 0.01);
        this->get_parameter_or<double>("global_optimization.ndt_translation_sigma", global_factor_graph_config_.ndt_translation_sigma, 0.15);
        this->get_parameter_or<double>("global_optimization.ndt_rotation_sigma_rad", global_factor_graph_config_.ndt_rotation_sigma_rad, 0.08);
        this->get_parameter_or<double>("global_optimization.imu_translation_sigma", global_factor_graph_config_.imu_translation_sigma, 0.20);
        this->get_parameter_or<double>("global_optimization.imu_rotation_sigma_rad", global_factor_graph_config_.imu_rotation_sigma_rad, 0.12);
        this->get_parameter_or<double>("global_optimization.rtk_position_sigma_floor", global_factor_graph_config_.rtk_position_sigma_floor, 0.20);
        this->get_parameter_or<double>("global_optimization.rtk_heading_sigma_floor_rad", global_factor_graph_config_.rtk_heading_sigma_floor_rad, 0.035);
        this->get_parameter_or<double>("global_optimization.loop_translation_sigma", global_factor_graph_config_.loop_translation_sigma, 0.10);
        this->get_parameter_or<double>("global_optimization.loop_rotation_sigma_rad", global_factor_graph_config_.loop_rotation_sigma_rad, 0.08);
        this->get_parameter_or<double>("global_optimization.robust_huber_k", global_factor_graph_config_.robust_huber_k, 1.345);
        global_factor_graph_ = std::make_unique<GlobalFactorGraph>(global_factor_graph_config_);
        std::string configured_data_path;
        this->get_parameter_or<string>("storage.data_path", configured_data_path, "");

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.world_points_en", pub_world_points_flag_, true);
        this->get_parameter_or<bool>("publish.body_points_en", pub_body_points_flag_, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
        this->get_parameter_or<string>("common.gnss_topic", gnss_topic, "/fix");
        this->get_parameter_or<string>("common.rtk_pvh_topic", rtk_pvh_topic, "/rtk_pvh");
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        int imu_init_sample_count = 600;
        double imu_init_max_acc_variance = 0.5;
        double imu_init_max_gyro_variance = 0.05;
        this->get_parameter_or<int>("imu_init.sample_count", imu_init_sample_count, 600);
        this->get_parameter_or<double>("imu_init.max_acc_variance", imu_init_max_acc_variance, 0.5);
        this->get_parameter_or<double>("imu_init.max_gyro_variance", imu_init_max_gyro_variance, 0.05);
        this->get_parameter_or<bool>("gnss_fusion.enable", use_gnss_fusion_, false);
        gnss_fusion_config_enabled_ = use_gnss_fusion_;
        this->get_parameter_or<string>("gnss_fusion.origin_file", gnss_origin_file_, "/home/dogrobot/runtime/nx-edge/data/jszr/map/gnss_origin.yaml");
        this->get_parameter_or<double>("gnss_fusion.gain", gnss_fusion_gain_, 0.03);
        this->get_parameter_or<double>("gnss_fusion.max_correction_step", gnss_max_correction_step_, 0.10);
        this->get_parameter_or<double>("gnss_fusion.max_residual", gnss_max_residual_, 5.0);
        this->get_parameter_or<double>("gnss_fusion.max_age", gnss_max_age_, 1.5);
        this->get_parameter_or<double>("gnss_fusion.max_horizontal_std", gnss_max_horizontal_std_, 1.5);
        this->get_parameter_or<int>("gnss_fusion.min_status", gnss_min_status_, 0);
        this->get_parameter_or<bool>("gnss_fusion.use_elevation", gnss_use_elevation_, false);
        std::vector<double> gnss_lever_arm;
        this->get_parameter_or<vector<double>>("gnss_fusion.lever_arm_base", gnss_lever_arm, vector<double>({ -0.05, 0.0, 0.15 }));
        if (gnss_lever_arm.size() >= 3)
        {
            gnss_lever_arm_base_ << gnss_lever_arm[0], gnss_lever_arm[1], gnss_lever_arm[2];
        }
        this->get_parameter_or<int>("gnss_fusion.alignment_min_samples", gnss_alignment_min_samples_, 20);
        this->get_parameter_or<double>("gnss_fusion.alignment_min_baseline", gnss_alignment_min_baseline_m_, 15.0);
        this->get_parameter_or<double>("gnss_fusion.alignment_max_rms", gnss_alignment_max_rms_m_, 1.5);
        double gnss_alignment_yaw_change_deg = 3.0;
        this->get_parameter_or<double>("gnss_fusion.alignment_max_yaw_change_deg", gnss_alignment_yaw_change_deg, 3.0);
        gnss_alignment_max_yaw_change_rad_ = gnss_alignment_yaw_change_deg * M_PI / 180.0;
        this->get_parameter_or<int>("gnss_fusion.alignment_required_fits", gnss_alignment_required_fits_, 3);
        this->get_parameter_or<bool>("gnss_fusion.use_heading", gnss_use_heading_, true);
        double gnss_heading_offset_deg = 0.0;
        this->get_parameter_or<double>("gnss_fusion.heading_offset_deg", gnss_heading_offset_deg, 0.0);
        gnss_heading_offset_rad_ = gnss_heading_offset_deg * M_PI / 180.0;
        this->get_parameter_or<double>("gnss_fusion.heading_min_baseline_m", gnss_heading_min_baseline_m_, 0.20);
        this->get_parameter_or<double>("gnss_fusion.heading_max_std_deg", gnss_heading_max_std_deg_, 5.0);
        this->get_parameter_or<double>("gnss_fusion.heading_max_age", gnss_heading_max_age_s_, 1.5);
        this->get_parameter_or<bool>("odom_guard.enable", odom_guard_enable_, false);
        this->get_parameter_or<string>("odom_guard.topic", odom_guard_topic, "/odom/mc_odom");
        this->get_parameter_or<double>("odom_guard.max_speed", odom_guard_max_speed_mps_, 2.0);
        this->get_parameter_or<double>("odom_guard.max_vertical_speed", odom_guard_max_vertical_speed_mps_, 0.10);
        this->get_parameter_or<double>("odom_guard.max_lidar_correction", odom_guard_max_lidar_correction_m_, 0.35);
        this->get_parameter_or<double>("odom_guard.max_lidar_z_correction", odom_guard_max_lidar_z_correction_m_, 0.03);
        this->get_parameter_or<double>("odom_guard.max_lidar_rotation", odom_guard_max_lidar_rotation_rad_, 0.35);
        this->get_parameter_or<double>("odom_guard.max_abs_z_from_start", odom_guard_max_abs_z_from_start_m_, 10.0);
        this->get_parameter_or<double>("health_guard.max_frame_translation", health_max_frame_translation_m_, 1.5);
        this->get_parameter_or<double>("health_guard.max_speed", health_max_speed_mps_, 3.0);
        this->get_parameter_or<double>("health_guard.max_abs_z", health_max_abs_z_m_, 5.0);
        this->get_parameter_or<double>("health_guard.max_frame_z", health_max_frame_z_m_, 1.0);
        this->get_parameter_or<double>("health_guard.warn_speed", health_warn_speed_mps_, 1.8);
        this->get_parameter_or<double>("health_guard.warn_abs_z", health_warn_abs_z_m_, 0.5);
        this->get_parameter_or<int>("health_guard.pose_guard_frames", health_pose_guard_frames_, 3);
        this->get_parameter_or<int>("health_guard.no_effective_points_limit", health_no_effective_limit_, 10);

#ifdef ROOT_DIR
        data_path_ = std::string(ROOT_DIR) + "/map";
#else
        RCLCPP_INFO(this->get_logger(), "There is no macro definition of ROOT_DIR");
        data_path_ = "/home/user_name/.jszr/map";
#endif
        if (!configured_data_path.empty())
            data_path_ = std::filesystem::path(configured_data_path).lexically_normal().string();
        if (!checkDirExist(data_path_))
        {
            RCLCPP_INFO(this->get_logger(), "Create map directory failed!!!!!!.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp    = this->get_clock()->now();
        path.header.frame_id = "map";

        FOV_DEG      = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudType());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(Vec3d(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu->set_init_requirements(imu_init_sample_count, imu_init_max_acc_variance, imu_init_max_gyro_variance);

        init();
        pcd2grid_ptr_ = std::make_shared<Pcd2Grid>(pcd2pgm_options_);

        sub_lidar_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::QoS(10).best_effort(), std::bind(&MappingAlg::lidarCallBack, this, std::placeholders::_1));

        sub_imu_ptr_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::QoS(200).best_effort(), std::bind(&MappingAlg::imuCallBack, this, std::placeholders::_1));
        sub_gnss_ptr_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            gnss_topic, 20, std::bind(&MappingAlg::gnssCallBack, this, std::placeholders::_1));
        sub_rtk_pvh_ptr_ = this->create_subscription<robots_dog_msgs::msg::UniRtkPvh>(
            rtk_pvh_topic, 20, std::bind(&MappingAlg::rtkPvhCallBack, this, std::placeholders::_1));
        if (odom_guard_enable_)
        {
            odom_guard_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_guard_topic, rclcpp::QoS(100).best_effort(),
                std::bind(&MappingAlg::odomGuardCallBack, this, std::placeholders::_1));
        }
        RCLCPP_INFO(this->get_logger(), "GNSS collection enabled on %s; pose correction=%s heading_lock=%s", gnss_topic.c_str(),
            use_gnss_fusion_ ? "enabled after alignment lock" : "disabled",
            gnss_use_heading_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Odometry replay guard=%s topic=%s", odom_guard_enable_ ? "enabled" : "disabled",
            odom_guard_topic.c_str());
        pubLaserCloudFull_      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_points", rclcpp::QoS(20).best_effort());
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/body_points", 20);
        pubLaserCloudMap_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 20);
        pubOdomAftMapped_       = this->create_publisher<nav_msgs::msg::Odometry>("/slam_odom", 20);
        pubPath_                = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        pubGlobalOptimizedOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("/slam/global_optimized_odom", 20);
        pubGlobalOptimizedPath_ = this->create_publisher<nav_msgs::msg::Path>("/slam/global_optimized_path", 10);
        pubGlobalOptimizationStatus_ = this->create_publisher<std_msgs::msg::String>(
            "/slam/global_optimization_status", rclcpp::QoS(1).transient_local());
        tf_broadcaster_         = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        state_service_ = this->create_service<robots_dog_msgs::srv::MapState>(
            "/slam_state_service", std::bind(&MappingAlg::stateCallBack, this, std::placeholders::_1, std::placeholders::_2));
        start_mapping_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/slam/start_mapping", std::bind(&MappingAlg::startMappingCallBack, this, std::placeholders::_1, std::placeholders::_2));
        save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/slam/save_map", std::bind(&MappingAlg::saveMapCallBack, this, std::placeholders::_1, std::placeholders::_2));
        global_optimize_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/slam/global_optimize", std::bind(&MappingAlg::globalOptimizeCallBack, this,
                std::placeholders::_1, std::placeholders::_2));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&MappingAlg::map_publish_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    MappingAlg ::~MappingAlg()
    {
        stopKeyframeWriter(false);
    }

    void MappingAlg::init()
    {
        std::vector<double> epsi(23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, std::bind(&MappingAlg::h_share_model, this, std::placeholders::_1, std::placeholders::_2),
            NUM_MAX_ITERATIONS, epsi.data());
    }

    void MappingAlg::stateCallBack(
        robots_dog_msgs::srv::MapState::Request::SharedPtr request, robots_dog_msgs::srv::MapState::Response::SharedPtr response)
    {
        uint8_t receive_message = request->data;
        switch (receive_message)
        {
            case 0:
                state_.store(SlamState::STABLE);
                response->success = true;
                response->message = "Set STABLE State!!!!!!";
                break;
            case 1:
                state_.store(SlamState::PASSIVE);
                response->success = true;
                response->message = "Set PASSIVE State!!!!!!";
                break;
            case 2:
                state_.store(SlamState::READY);
                response->success = true;
                response->message = "Set READY State!!!!!!";
                break;
            case 3:
                if (state_.load() == SlamState::WARMUP)
                {
                    mapping_capture_enabled_ = true;
                    has_last_keyframe_ = false;
                    resetImuPreintegration(lidar_end_time);
                    writeSaveProgress("waiting_first_keyframe", 0.0);
                }
                else
                {
                    reset();
                    mapping_capture_enabled_ = true;
                    // Legacy direct starts are indoor-compatible. Outdoor starts
                    // enter WARMUP first, preserving the prelocked ENU origin here.
                    use_gnss_fusion_ = false;
                    gnss_origin_initialized_ = false;
                    gnss_origin_prelocked_ = false;
                    gnss_alignment_locked_ = false;
                    gnss_alignment_source_.clear();
                }
                if (active_map_subdir_.empty())
                {
                    state_.store(SlamState::ERROR);
                    response->success = false;
                    response->message = "Failed to initialize mapping session directory.";
                }
                else
                {
                    state_.store(SlamState::ACTIVE);
                    writeSaveProgress(
                        p_imu->initialization_ready() ? "waiting_first_keyframe" : "initializing_imu", 0.0);
                    response->success = true;
                    response->message = "Set ACTIVE State!!!!!!";
                }
                break;
            case 6:
                reset();
                mapping_capture_enabled_ = false;
                use_gnss_fusion_ = gnss_fusion_config_enabled_;
                if (!gnss_origin_prelocked_)
                {
                    state_.store(SlamState::ERROR);
                    response->success = false;
                    response->message = "Outdoor SLAM warmup rejected: locked gnss_origin.yaml is missing or invalid.";
                }
                else if (active_map_subdir_.empty())
                {
                    state_.store(SlamState::ERROR);
                    response->success = false;
                    response->message = "Failed to initialize SLAM warmup session directory.";
                }
                else
                {
                    state_.store(SlamState::WARMUP);
                    response->success = true;
                    response->message = "Set WARMUP State; formal keyframe capture is disabled.";
                }
                break;
            case 7:
                reset();
                mapping_capture_enabled_ = false;
                use_gnss_fusion_ = false;
                gnss_origin_initialized_ = false;
                gnss_origin_prelocked_ = false;
                gnss_alignment_locked_ = false;
                gnss_alignment_source_.clear();
                if (active_map_subdir_.empty())
                {
                    state_.store(SlamState::ERROR);
                    response->success = false;
                    response->message = "Failed to initialize indoor SLAM warmup session directory.";
                }
                else
                {
                    state_.store(SlamState::WARMUP);
                    response->success = true;
                    response->message = "Set indoor WARMUP State; GNSS origin and formal capture are disabled.";
                }
                break;
            case 4:
                state_.store(SlamState::ERROR);
                response->success = true;
                response->message = "Set ERROR State!!!!!!";
                break;
            case 5:
                if (slam_diverged_)
                {
                    response->success = false;
                    response->message = "Map save rejected: SLAM_DIVERGED: " + slam_health_error_;
                    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
                    break;
                }
                if (mapping_keyframes_.empty()
                    && (state_.load() != SlamState::STABLE || !recoverLatestKeyframeSession()))
                {
                    response->success = false;
                    response->message = "Map save rejected: no active or recoverable keyframes were found.";
                    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
                    break;
                }
                if (state_.load() != SlamState::ACTIVE
                    && state_.load() != SlamState::ERROR
                    && state_.load() != SlamState::READY
                    && state_.load() != SlamState::STABLE)
                {
                    response->success = false;
                    response->message = "Map save rejected: SLAM is not mapping or recoverable.";
                    RCLCPP_ERROR(get_logger(), "Map save rejected because SLAM state is not recoverable");
                    break;
                }
                if (map_export_completed_)
                {
                    response->success = true;
                    response->message = "Map was already saved.";
                    break;
                }
                if (keyframe_writer_failed_)
                {
                    response->success = false;
                    response->message = "Map save rejected: keyframe writer failed: " + keyframe_writer_error_;
                    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
                    break;
                }
                state_.store(SlamState::SAVE);
                response->success = true;
                response->message = "Map save accepted.";
                RCLCPP_INFO(get_logger(), "Map save accepted: %zu keyframes", mapping_keyframes_.size());
                break;
            default:
                response->success = false;
                response->message = "SLAM not this State Fail !!!!!!";
                break;
        }
    }

    void MappingAlg::startMappingCallBack(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        auto map_request = std::make_shared<robots_dog_msgs::srv::MapState::Request>();
        auto map_response = std::make_shared<robots_dog_msgs::srv::MapState::Response>();
        map_request->data = 3;
        stateCallBack(map_request, map_response);
        response->success = map_response->success;
        response->message = map_response->message;
    }

    void MappingAlg::saveMapCallBack(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        auto map_request = std::make_shared<robots_dog_msgs::srv::MapState::Request>();
        auto map_response = std::make_shared<robots_dog_msgs::srv::MapState::Response>();
        map_request->data = 5;
        stateCallBack(map_request, map_response);
        response->success = map_response->success;
        response->message = map_response->message;
    }

    void MappingAlg::globalOptimizeCallBack(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        if (active_map_subdir_.empty() || mapping_keyframes_.empty())
        {
            response->success = false;
            response->message = "No active mapping session or keyframes.";
            return;
        }
        bool success = optimizeHistoricalTrajectory(active_map_subdir_);
        if (success && map_export_completed_ && global_optimization_applied_)
        {
            const auto map_pcd = std::filesystem::path(active_map_subdir_) / "map.pcd";
            const auto map_raw = std::filesystem::path(active_map_subdir_) / "map_raw.pcd";
            std::error_code filesystem_error;
            if (!std::filesystem::exists(map_raw) && std::filesystem::exists(map_pcd))
                std::filesystem::copy_file(map_pcd, map_raw,
                    std::filesystem::copy_options::overwrite_existing, filesystem_error);
            std::size_t written_points = 0;
            success = streamMapFromKeyframes(active_map_subdir_, written_points);
            if (success)
            {
                const auto grid_pcd = std::filesystem::path(active_map_subdir_) / ".map_grid_relative.pcd";
                std::string grid_error;
                Pcd2Grid grid_builder(pcd2pgm_options_);
                success = grid_builder.runFromBinaryPcd(
                    grid_pcd.string(), (std::filesystem::path(active_map_subdir_) / "map").string(),
                    nullptr, &grid_error);
                std::filesystem::remove(grid_pcd, filesystem_error);
                if (!success)
                    keyframe_writer_error_ = grid_error;
            }
            if (success)
                success = writeTrajectoryAndGnssMetadata(active_map_subdir_);
            if (!success && std::filesystem::exists(map_raw))
            {
                std::filesystem::copy_file(map_raw, map_pcd,
                    std::filesystem::copy_options::overwrite_existing, filesystem_error);
                RCLCPP_ERROR(get_logger(), "Optimized map rebuild failed; restored map_raw.pcd: %s",
                    keyframe_writer_error_.c_str());
            }
        }
        response->success = success;
        response->message = success
            ? "Historical GTSAM optimization and map rebuild completed."
            : keyframe_writer_error_;
    }

    void MappingAlg::reset()
    {
        stopKeyframeWriter(false);
        time_buffer.clear();
        lidar_buffer.clear();
        imu_buffer.clear();
        is_first_lidar      = true;
        flg_first_scan      = true;
        scan_num            = 0;
        lidar_mean_scantime = 0.0;
        memset(point_selected_surf, true, sizeof(point_selected_surf));

        p_imu->reset();
        {
            std::lock_guard<std::mutex> gnss_lock(gnss_mutex_);
            has_gnss_ = false;
            gnss_origin_initialized_ = false;
            gnss_correction_count_ = 0;
        }
        gnss_alignment_samples_.clear();
        gnss_alignment_locked_ = false;
        gnss_alignment_source_.clear();
        gnss_enu_to_map_yaw_ = 0.0;
        gnss_enu_to_map_translation_.setZero();
        gnss_alignment_rms_ = std::numeric_limits<double>::infinity();
        gnss_last_alignment_stamp_ = -1.0;
        gnss_alignment_stable_fits_ = 0;
        optimized_global_poses_.clear();
        global_pose_covariances_.clear();
        global_factor_graph_result_ = GlobalFactorGraphResult{};
        global_optimization_applied_ = false;
        scan_context_count_ = 0;
        loop_status_ = "skipped";
        {
            std::lock_guard<std::mutex> heading_lock(gnss_heading_mutex_);
            has_gnss_heading_ = false;
        }
        {
            std::lock_guard<std::mutex> odom_lock(odom_guard_mutex_);
            has_odom_guard_ = false;
        }
        odom_guard_initialized_ = false;
        odom_guard_position_.setZero();
        odom_guard_stamp_ = 0.0;
        odom_guard_initial_z_ = 0.0;
        odom_guard_rejected_updates_ = 0;
        odom_guard_clamped_z_updates_ = 0;
        slam_diverged_ = false;
        slam_health_state_ = "initializing";
        slam_health_error_code_.clear();
        slam_health_error_.clear();
        slam_health_warning_.clear();
        health_frame_delta_m_ = 0.0;
        health_speed_mps_ = 0.0;
        health_pose_z_m_ = 0.0;
        no_effective_points_streak_ = 0;
        pose_anomaly_streak_ = 0;
        has_last_health_pose_ = false;
        last_health_stamp_ = 0.0;
        mapping_started_stamp_ = 0.0;
        mapping_capture_enabled_ = false;
        slam_pose_ready_ = false;
        mapping_keyframes_.clear();
        keyframe_write_queue_.clear();
        keyframe_writer_failed_ = false;
        keyframe_writer_error_.clear();
        written_keyframes_ = 0;
        written_keyframe_points_ = 0;
        dropped_keyframes_ = 0;
        keyframe_trajectory_m_ = 0.0;
        active_map_subdir_.clear();
        map_export_completed_ = false;
        resetImuPreintegration(0.0);
        pcl_wait_pub->clear();
        pcl_wait_save->clear();
        has_last_keyframe_ = false;
        last_keyframe_stamp_ = 0.0;
        last_mapping_progress_stamp_ = 0.0;

        state_ikfom state_updated;
        state_updated.pos = Zero3d;
        state_updated.rot = Quatd(1.0, 0.0, 0.0, 0.0);
        state_point       = state_updated;  // 对state_point进行更新，state_point可视化用到
        kf.change_x(state_updated);
        Localmap_Initialized = false;

        if (!initializeKeyframeSession())
            active_map_subdir_.clear();
        loadLockedGnssOrigin();
    }

    bool MappingAlg::loadLockedGnssOrigin()
    {
        gnss_origin_prelocked_ = false;
        if (gnss_origin_file_.empty())
            return false;
        std::ifstream input(gnss_origin_file_);
        if (!input.is_open())
            return false;
        bool locked = false;
        bool has_lat = false;
        bool has_lon = false;
        bool has_alt = false;
        std::string line;
        try
        {
            while (std::getline(input, line))
            {
                const auto separator = line.find(':');
                if (separator == std::string::npos)
                    continue;
                const std::string key = line.substr(0, separator);
                const std::string value = line.substr(separator + 1);
                if (key == "origin_latitude")
                {
                    gnss_origin_lat_ = std::stod(value);
                    has_lat = true;
                }
                else if (key == "origin_longitude")
                {
                    gnss_origin_lon_ = std::stod(value);
                    has_lon = true;
                }
                else if (key == "origin_altitude")
                {
                    gnss_origin_alt_ = std::stod(value);
                    has_alt = true;
                }
                else if (key == "alignment_locked")
                    locked = value.find("true") != std::string::npos || value.find("True") != std::string::npos;
            }
        }
        catch (const std::exception& exc)
        {
            RCLCPP_ERROR(get_logger(), "Invalid locked GNSS origin %s: %s", gnss_origin_file_.c_str(), exc.what());
            return false;
        }
        gnss_origin_initialized_ = locked && has_lat && has_lon && has_alt;
        gnss_origin_prelocked_ = gnss_origin_initialized_;
        if (gnss_origin_prelocked_)
            RCLCPP_INFO(get_logger(), "Loaded locked ENU origin %.10f, %.10f, %.3f from %s",
                gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_, gnss_origin_file_.c_str());
        return gnss_origin_prelocked_;
    }


    double MappingAlg::get_time_sec(const builtin_interfaces::msg::Time& time)
    {
        return rclcpp::Time(time).seconds();
    }

    rclcpp::Time MappingAlg::get_ros_time(double timestamp)
    {
        int32_t  sec       = std::floor(timestamp);
        auto     nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
        uint32_t nanosec   = nanosec_d;
        return rclcpp::Time(sec, nanosec);
    }

    void MappingAlg::pointsBody2World(PointType const* const pi, PointType* const po)
    {
        Vec3d p_body(pi->x, pi->y, pi->z);
        Vec3d p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x         = p_global(0);
        po->y         = p_global(1);
        po->z         = p_global(2);
        po->intensity = pi->intensity;
    }

    void MappingAlg::pointsBody2Imu(PointType const* const pi, PointType* const po)
    {
        Vec3d p_body_lidar(pi->x, pi->y, pi->z);
        Vec3d p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

        po->x         = p_body_imu(0);
        po->y         = p_body_imu(1);
        po->z         = p_body_imu(2);
        po->intensity = pi->intensity;
    }

    void MappingAlg::points_cache_collect()
    {
        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);
    }

    void MappingAlg::lasermap_fov_segment()
    {
        cub_needrm.clear();
        int   kdtree_delete_counter = 0;
        Vec3d pos_LiD               = pos_lid;
        if (!Localmap_Initialized)
        {
            for (int i = 0; i < 3; i++)
            {
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }
        float dist_to_map_edge[3][2];
        bool  need_move = false;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
                need_move = true;
        }
        if (!need_move)
            return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist      = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
            else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        points_cache_collect();
        double delete_begin = omp_get_wtime();
        if (cub_needrm.size() > 0)
            kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }

    void MappingAlg::lidarCallBack(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        mtx_buffer.lock();
        double cur_time              = get_time_sec(msg->header.stamp);
        double preprocess_start_time = omp_get_wtime();
        scan_count++;
        if (!is_first_lidar && cur_time < last_timestamp_lidar)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            lidar_buffer.clear();
        }
        if (is_first_lidar)
        {
            is_first_lidar = false;
        }
        last_timestamp_lidar = cur_time;

        PointCloudType::Ptr ptr(new PointCloudType());

        pcl::PointCloud<livox_pcl::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);

        p_pre->process(pl_orig, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);

        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    void MappingAlg::gnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(gnss_mutex_);
        latest_gnss_ = *msg;
        has_gnss_    = true;
    }

    void MappingAlg::rtkPvhCallBack(const robots_dog_msgs::msg::UniRtkPvh::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(gnss_heading_mutex_);
        latest_gnss_heading_deg_ = static_cast<double>(msg->heading.heading_deg);
        latest_gnss_heading_std_deg_ = static_cast<double>(msg->heading.heading_std);
        latest_gnss_heading_baseline_m_ = static_cast<double>(msg->heading.base_line);
        latest_gnss_heading_status_ = static_cast<int>(msg->heading.sol_status);
        latest_gnss_heading_type_ = static_cast<int>(msg->heading.heading_type);
        latest_gnss_heading_receive_time_ = get_clock()->now();
        has_gnss_heading_ = true;
    }

    void MappingAlg::odomGuardCallBack(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_guard_mutex_);
        latest_odom_guard_ = *msg;
        has_odom_guard_ = true;
    }

    bool MappingAlg::applyOdomGuardPrediction(double lidar_time)
    {
        if (!odom_guard_enable_)
            return false;

        nav_msgs::msg::Odometry odom;
        {
            std::lock_guard<std::mutex> lock(odom_guard_mutex_);
            if (!has_odom_guard_)
                return false;
            odom = latest_odom_guard_;
        }

        state_ikfom guarded = kf.get_x();
        Vec3d body_velocity(
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            0.0);
        if (!body_velocity.allFinite())
            return false;
        const double speed = body_velocity.norm();
        if (speed > odom_guard_max_speed_mps_ && speed > 1e-6)
            body_velocity *= odom_guard_max_speed_mps_ / speed;
        Vec3d world_velocity = guarded.rot * body_velocity;
        world_velocity(2) = std::clamp(
            world_velocity(2), -odom_guard_max_vertical_speed_mps_, odom_guard_max_vertical_speed_mps_);

        if (!odom_guard_initialized_)
        {
            odom_guard_position_ = guarded.pos;
            odom_guard_stamp_ = lidar_time;
            odom_guard_initial_z_ = guarded.pos(2);
            odom_guard_initialized_ = true;
        }
        else
        {
            const double dt = lidar_time - odom_guard_stamp_;
            if (dt > 0.0 && dt < 0.5)
                guarded.pos = odom_guard_position_ + world_velocity * dt;
            else
                odom_guard_position_ = guarded.pos;
            odom_guard_stamp_ = lidar_time;
        }
        guarded.pos(2) = std::clamp(
            guarded.pos(2),
            odom_guard_initial_z_ - odom_guard_max_abs_z_from_start_m_,
            odom_guard_initial_z_ + odom_guard_max_abs_z_from_start_m_);
        guarded.vel = world_velocity;
        kf.change_x(guarded);
        state_point = guarded;
        return true;
    }

    bool MappingAlg::acceptOdomGuardCorrection(double lidar_time, const state_ikfom& prediction)
    {
        state_ikfom candidate = kf.get_x();
        const double translation = (candidate.pos - prediction.pos).norm();
        const double z_correction = candidate.pos(2) - prediction.pos(2);
        const Mat3d relative_rotation = prediction.rot.conjugate().toRotationMatrix()
            * candidate.rot.toRotationMatrix();
        const double cosine = std::clamp((relative_rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
        const double rotation = std::acos(cosine);
        const bool rejected = !candidate.pos.allFinite()
            || translation > odom_guard_max_lidar_correction_m_
            || rotation > odom_guard_max_lidar_rotation_rad_;

        if (rejected)
        {
            state_ikfom restored = prediction;
            kf.change_x(restored);
            state_point = restored;
            odom_guard_rejected_updates_++;
            slam_health_state_ = "degraded";
            std::ostringstream warning;
            warning << "odom guard rejected lidar update: translation=" << translation
                    << "m rotation=" << rotation << "rad";
            slam_health_warning_ = warning.str();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", slam_health_warning_.c_str());
        }
        else
        {
            if (std::fabs(z_correction) > odom_guard_max_lidar_z_correction_m_)
            {
                candidate.pos(2) = prediction.pos(2);
                odom_guard_clamped_z_updates_++;
            }
            candidate.pos(2) = std::clamp(
                candidate.pos(2),
                odom_guard_initial_z_ - odom_guard_max_abs_z_from_start_m_,
                odom_guard_initial_z_ + odom_guard_max_abs_z_from_start_m_);
            candidate.vel = prediction.vel;
            kf.change_x(candidate);
            state_point = candidate;
        }

        odom_guard_position_ = state_point.pos;
        odom_guard_stamp_ = lidar_time;
        return !rejected;
    }

    Vec3d MappingAlg::llaToEnu(double latitude_deg, double longitude_deg, double altitude_m) const
    {
        constexpr double kEarthRadiusM = 6378137.0;
        constexpr double kDegToRad     = M_PI / 180.0;
        const double     d_lat         = (latitude_deg - gnss_origin_lat_) * kDegToRad;
        const double     d_lon         = (longitude_deg - gnss_origin_lon_) * kDegToRad;
        const double     lat0          = gnss_origin_lat_ * kDegToRad;
        return Vec3d(d_lon * std::cos(lat0) * kEarthRadiusM, d_lat * kEarthRadiusM, altitude_m - gnss_origin_alt_);
    }

    bool MappingAlg::gnssToMap(const sensor_msgs::msg::NavSatFix& msg, Vec3d& map_pos)
    {
        if (msg.status.status < gnss_min_status_)
            return false;
        if (std::fabs(msg.latitude) < 1e-7 || std::fabs(msg.longitude) < 1e-7)
            return false;
        const double h_std = std::sqrt(std::max(msg.position_covariance[0], msg.position_covariance[4]));
        if (h_std > gnss_max_horizontal_std_)
            return false;

        if (!gnss_origin_initialized_ || !gnss_alignment_locked_)
            return false;

        const Vec3d enu = llaToEnu(msg.latitude, msg.longitude, msg.altitude);
        const double cosine = std::cos(gnss_enu_to_map_yaw_);
        const double sine = std::sin(gnss_enu_to_map_yaw_);
        map_pos(0) = cosine * enu(0) - sine * enu(1) + gnss_enu_to_map_translation_(0);
        map_pos(1) = sine * enu(0) + cosine * enu(1) + gnss_enu_to_map_translation_(1);
        map_pos(2) = enu(2) + gnss_map_offset_(2);
        return true;
    }

    void MappingAlg::collectGnssAlignment(double lidar_time)
    {
        if (!flg_EKF_inited || slam_diverged_)
            return;

        sensor_msgs::msg::NavSatFix gnss;
        {
            std::lock_guard<std::mutex> lock(gnss_mutex_);
            if (!has_gnss_)
                return;
            gnss = latest_gnss_;
        }
        const double gnss_time = get_time_sec(gnss.header.stamp);
        if (gnss_time <= gnss_last_alignment_stamp_ + 1e-6 || std::fabs(lidar_time - gnss_time) > gnss_max_age_)
            return;
        if (gnss.status.status < gnss_min_status_ || std::fabs(gnss.latitude) < 1e-7 || std::fabs(gnss.longitude) < 1e-7)
            return;
        const double h_std = std::sqrt(std::max(0.0,
            std::max(gnss.position_covariance[0], gnss.position_covariance[4])));
        if (!std::isfinite(h_std) || h_std > gnss_max_horizontal_std_)
            return;

        if (!gnss_origin_initialized_)
        {
            gnss_origin_lat_ = gnss.latitude;
            gnss_origin_lon_ = gnss.longitude;
            gnss_origin_alt_ = gnss.altitude;
            gnss_origin_initialized_ = true;
            RCLCPP_INFO(get_logger(), "GNSS origin collected lat=%.9f lon=%.9f alt=%.3f; waiting for heading or trajectory ENU-map alignment",
                gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_);
        }

        const Vec3d enu = llaToEnu(gnss.latitude, gnss.longitude, gnss.altitude);
        const Vec3d estimated_gps = state_point.pos + state_point.rot * gnss_lever_arm_base_;
        gnss_alignment_samples_.push_back({ enu.head<2>(), estimated_gps.head<2>(), gnss_time });
        gnss_last_alignment_stamp_ = gnss_time;
        while (gnss_alignment_samples_.size() > gnss_alignment_max_samples_)
            gnss_alignment_samples_.pop_front();
        if (!lockGnssAlignmentFromHeading(gnss))
            estimateGnssAlignment();
    }

    bool MappingAlg::gnssHeadingIsValid(double& heading_deg, double& heading_std_deg, double& age_s)
    {
        std::lock_guard<std::mutex> lock(gnss_heading_mutex_);
        age_s = has_gnss_heading_
            ? std::max(0.0, (get_clock()->now() - latest_gnss_heading_receive_time_).seconds())
            : std::numeric_limits<double>::infinity();
        heading_deg = latest_gnss_heading_deg_;
        heading_std_deg = latest_gnss_heading_std_deg_;
        return has_gnss_heading_
            && latest_gnss_heading_status_ == 0
            && latest_gnss_heading_type_ > 0
            && latest_gnss_heading_baseline_m_ >= gnss_heading_min_baseline_m_
            && latest_gnss_heading_std_deg_ <= gnss_heading_max_std_deg_
            && age_s <= gnss_heading_max_age_s_;
    }

    bool MappingAlg::lockGnssAlignmentFromHeading(const sensor_msgs::msg::NavSatFix& gnss)
    {
        if (gnss_alignment_locked_ || !gnss_use_heading_ || !gnss_origin_initialized_)
            return gnss_alignment_locked_;

        double heading_deg = 0.0;
        double heading_std_deg = 0.0;
        double heading_age = 0.0;
        if (!gnssHeadingIsValid(heading_deg, heading_std_deg, heading_age))
            return false;

        const auto rotation = state_point.rot.toRotationMatrix();
        const double slam_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        const double yaw_enu = M_PI / 2.0 - heading_deg * M_PI / 180.0;
        const double alignment_yaw = std::atan2(
            std::sin(slam_yaw - yaw_enu - gnss_heading_offset_rad_),
            std::cos(slam_yaw - yaw_enu - gnss_heading_offset_rad_));
        const Vec3d enu = llaToEnu(gnss.latitude, gnss.longitude, gnss.altitude);
        const Vec3d estimated_gps = state_point.pos + state_point.rot * gnss_lever_arm_base_;
        const double cosine = std::cos(alignment_yaw);
        const double sine = std::sin(alignment_yaw);
        const Eigen::Vector2d rotated(cosine * enu(0) - sine * enu(1), sine * enu(0) + cosine * enu(1));

        gnss_alignment_locked_ = true;
        gnss_alignment_source_ = "heading";
        gnss_enu_to_map_yaw_ = alignment_yaw;
        gnss_enu_to_map_translation_ = estimated_gps.head<2>() - rotated;
        gnss_map_offset_ << gnss_enu_to_map_translation_(0), gnss_enu_to_map_translation_(1), 0.0;
        gnss_alignment_rms_ = 0.0;
        RCLCPP_INFO(get_logger(),
            "GNSS ENU-map alignment locked from dual-antenna heading: heading=%.2fdeg slam_yaw=%.2fdeg "
            "enu_to_map_yaw=%.2fdeg offset=[%.2f, %.2f] heading_std=%.2fdeg age=%.2fs",
            heading_deg, slam_yaw * 180.0 / M_PI, alignment_yaw * 180.0 / M_PI,
            gnss_enu_to_map_translation_(0), gnss_enu_to_map_translation_(1), heading_std_deg, heading_age);
        return true;
    }

    bool MappingAlg::estimateGnssAlignment()
    {
        if (gnss_alignment_samples_.size() < static_cast<std::size_t>(gnss_alignment_min_samples_))
            return false;

        double baseline = 0.0;
        for (const auto& sample : gnss_alignment_samples_)
            baseline = std::max(baseline, (sample.enu - gnss_alignment_samples_.front().enu).norm());
        if (baseline < gnss_alignment_min_baseline_m_)
            return false;

        auto solve = [](const std::vector<const GnssAlignmentSample*>& samples,
                         Eigen::Matrix2d& rotation, Eigen::Vector2d& translation) {
            Eigen::Vector2d enu_mean = Eigen::Vector2d::Zero();
            Eigen::Vector2d map_mean = Eigen::Vector2d::Zero();
            for (const auto* sample : samples)
            {
                enu_mean += sample->enu;
                map_mean += sample->map;
            }
            enu_mean /= static_cast<double>(samples.size());
            map_mean /= static_cast<double>(samples.size());
            Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
            for (const auto* sample : samples)
                covariance += (sample->enu - enu_mean) * (sample->map - map_mean).transpose();
            Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            rotation = svd.matrixV() * svd.matrixU().transpose();
            if (rotation.determinant() < 0.0)
            {
                Eigen::Matrix2d fix = Eigen::Matrix2d::Identity();
                fix(1, 1) = -1.0;
                rotation = svd.matrixV() * fix * svd.matrixU().transpose();
            }
            translation = map_mean - rotation * enu_mean;
        };

        std::vector<const GnssAlignmentSample*> all;
        all.reserve(gnss_alignment_samples_.size());
        for (const auto& sample : gnss_alignment_samples_)
            all.push_back(&sample);
        Eigen::Matrix2d rotation;
        Eigen::Vector2d translation;
        solve(all, rotation, translation);

        std::vector<double> residuals;
        residuals.reserve(all.size());
        for (const auto* sample : all)
            residuals.push_back((rotation * sample->enu + translation - sample->map).norm());
        auto median_values = residuals;
        const auto middle = median_values.begin() + median_values.size() / 2;
        std::nth_element(median_values.begin(), middle, median_values.end());
        const double outlier_limit = std::max(0.5, 3.0 * *middle);
        std::vector<const GnssAlignmentSample*> inliers;
        for (std::size_t index = 0; index < all.size(); ++index)
            if (residuals[index] <= outlier_limit)
                inliers.push_back(all[index]);
        if (inliers.size() < static_cast<std::size_t>(gnss_alignment_min_samples_))
            return false;
        solve(inliers, rotation, translation);

        double squared_error = 0.0;
        for (const auto* sample : inliers)
            squared_error += (rotation * sample->enu + translation - sample->map).squaredNorm();
        const double rms = std::sqrt(squared_error / static_cast<double>(inliers.size()));
        const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        const double yaw_delta = std::fabs(std::atan2(std::sin(yaw - gnss_last_candidate_yaw_),
            std::cos(yaw - gnss_last_candidate_yaw_)));
        if (rms <= gnss_alignment_max_rms_m_)
            gnss_alignment_stable_fits_ = gnss_alignment_stable_fits_ == 0 || yaw_delta <= gnss_alignment_max_yaw_change_rad_
                ? gnss_alignment_stable_fits_ + 1 : 1;
        else
            gnss_alignment_stable_fits_ = 0;
        gnss_last_candidate_yaw_ = yaw;
        gnss_alignment_rms_ = rms;

        if (!gnss_alignment_locked_ && gnss_alignment_stable_fits_ >= gnss_alignment_required_fits_)
        {
            gnss_alignment_locked_ = true;
            gnss_alignment_source_ = "trajectory";
            gnss_enu_to_map_yaw_ = yaw;
            gnss_enu_to_map_translation_ = translation;
            gnss_map_offset_ << translation(0), translation(1), 0.0;
            RCLCPP_INFO(get_logger(), "GNSS ENU-map alignment locked from trajectory: yaw=%.2fdeg offset=[%.2f, %.2f] rms=%.2fm samples=%zu",
                yaw * 180.0 / M_PI, translation(0), translation(1), rms, inliers.size());
        }
        return gnss_alignment_locked_;
    }

    void MappingAlg::applyGnssCorrection(double lidar_time)
    {
        if (!use_gnss_fusion_ || !flg_EKF_inited)
            return;

        sensor_msgs::msg::NavSatFix gnss;
        {
            std::lock_guard<std::mutex> lock(gnss_mutex_);
            if (!has_gnss_)
                return;
            gnss = latest_gnss_;
        }

        const double gnss_time = get_time_sec(gnss.header.stamp);
        if (std::fabs(lidar_time - gnss_time) > gnss_max_age_)
            return;

        Vec3d gnss_gps_map;
        if (!gnssToMap(gnss, gnss_gps_map))
            return;

        const Vec3d estimated_gps = state_point.pos + state_point.rot * gnss_lever_arm_base_;
        Vec3d       residual      = gnss_gps_map - estimated_gps;
        if (!gnss_use_elevation_)
            residual(2) = 0.0;

        const double residual_norm = residual.norm();
        if (residual_norm > gnss_max_residual_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Reject GNSS correction: residual %.2fm exceeds %.2fm", residual_norm, gnss_max_residual_);
            return;
        }

        // Fixed RTK acts as the primary absolute-position constraint. Float
        // solutions remain useful, but are deliberately applied more softly.
        const double quality_gain = gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
            ? gnss_fusion_gain_ : gnss_fusion_gain_ * 0.25;
        Vec3d correction = residual * quality_gain;
        const double correction_norm = correction.norm();
        if (correction_norm > gnss_max_correction_step_)
        {
            correction *= gnss_max_correction_step_ / correction_norm;
        }

        if (correction.norm() < 1e-4)
            return;

        state_ikfom corrected = state_point;
        corrected.pos += correction;
        kf.change_x(corrected);
        state_point = corrected;
        gnss_correction_count_++;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GNSS correction #%d mode=%s residual=%.2fm step=%.3fm", gnss_correction_count_,
            gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX ? "rtk_primary" : "hybrid",
            residual_norm, correction.norm());
    }

    void MappingAlg::markSlamDiverged(const std::string& reason)
    {
        if (slam_diverged_)
            return;
        slam_diverged_ = true;
        slam_health_state_ = "diverged";
        slam_health_error_code_ = "SLAM_DIVERGED";
        slam_health_error_ = reason;
        state_.store(SlamState::ERROR);
        writeSaveProgress("failed", 0.0, reason);
        RCLCPP_ERROR(get_logger(), "SLAM_DIVERGED: %s; keyframe recording and map export disabled", reason.c_str());
    }

    void MappingAlg::updateSlamHealth(double lidar_time)
    {
        if (slam_diverged_)
            return;
        if (!p_imu->initialization_ready())
        {
            slam_health_state_ = "initializing";
            return;
        }
        if (!state_point.pos.allFinite())
        {
            markSlamDiverged("pose contains non-finite values");
            return;
        }
        if (mapping_started_stamp_ <= 0.0)
            mapping_started_stamp_ = lidar_time;

        no_effective_points_streak_ = effct_feat_num < 1 ? no_effective_points_streak_ + 1 : 0;
        if (no_effective_points_streak_ >= health_no_effective_limit_)
        {
            slam_health_state_ = "degraded";
            slam_health_warning_ = "scan matching produced no effective points for "
                + std::to_string(no_effective_points_streak_) + " consecutive scans";
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", slam_health_warning_.c_str());
            return;
        }

        health_pose_z_m_ = state_point.pos(2);
        bool hard_anomaly = false;
        if (has_last_health_pose_)
        {
            const double dt = lidar_time - last_health_stamp_;
            const Vec3d delta = state_point.pos - last_health_position_;
            const double speed = dt > 1e-3 ? delta.norm() / dt : 0.0;
            health_frame_delta_m_ = delta.norm();
            health_speed_mps_ = speed;
            hard_anomaly = delta.norm() > health_max_frame_translation_m_
                || std::fabs(delta(2)) > health_max_frame_z_m_
                || speed > health_max_speed_mps_;
            pose_anomaly_streak_ = hard_anomaly ? pose_anomaly_streak_ + 1 : 0;
            if (hard_anomaly)
            {
                std::ostringstream reason;
                reason << "pose anomaly detected: frame_delta=" << delta.norm() << "m speed=" << speed
                       << "m/s z=" << state_point.pos(2) << "m";
                slam_health_state_ = "degraded";
                slam_health_warning_ = reason.str();
                last_health_position_ = state_point.pos;
                last_health_stamp_ = lidar_time;
                has_last_health_pose_ = true;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", slam_health_warning_.c_str());
                return;
            }
        }
        has_last_health_pose_ = true;
        last_health_position_ = state_point.pos;
        last_health_stamp_ = lidar_time;
        slam_health_warning_.clear();
        if (hard_anomaly || health_speed_mps_ > health_warn_speed_mps_)
        {
            slam_health_state_ = "degraded";
            std::ostringstream warning;
            warning << "pose speed warning: " << health_speed_mps_ << "m/s";
            slam_health_warning_ = warning.str();
        }
        else if (health_warn_abs_z_m_ > 0.0 && std::fabs(health_pose_z_m_) > health_warn_abs_z_m_)
        {
            slam_health_state_ = "degraded";
            std::ostringstream warning;
            warning << "pose height drift warning: z=" << health_pose_z_m_ << "m";
            slam_health_warning_ = warning.str();
        }
        else if (no_effective_points_streak_ >= std::max(2, health_no_effective_limit_ / 3))
        {
            slam_health_state_ = "degraded";
            slam_health_warning_ = "scan matching effective points are intermittently missing";
        }
        else
        {
            slam_health_state_ = "healthy";
        }
    }

    bool MappingAlg::validateMappingSession(std::string& reason) const
    {
        if (slam_diverged_)
        {
            reason = slam_health_error_.empty() ? "SLAM divergence detected" : slam_health_error_;
            return false;
        }
        if (mapping_keyframes_.empty())
        {
            reason = "no persistent keyframes available";
            return false;
        }
        for (const auto& keyframe : mapping_keyframes_)
        {
            if (!keyframe.lidar_origin.allFinite())
            {
                reason = "keyframe trajectory contains invalid pose";
                return false;
            }
        }
        return true;
    }

    void MappingAlg::imuCallBack(const sensor_msgs::msg::Imu::UniquePtr msg_in)
    {
        sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

        msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);

        double timestamp = get_time_sec(msg->header.stamp);

        mtx_buffer.lock();

        if (timestamp < last_timestamp_imu)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        ImuMessagePtr imu_msg_ptr =
            std::make_shared<ImuMessage>(timestamp, Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));

        imu_buffer.push_back(imu_msg_ptr);
        mtx_buffer.unlock();
        accumulateImuPreintegration(timestamp, imu_msg_ptr->acc, imu_msg_ptr->gyr);
        sig_buffer.notify_all();
    }

    bool MappingAlg::syncData(MeasureGroup& meas)
    {
        if (lidar_buffer.empty() || imu_buffer.empty())
        {
            return false;
        }

        if (!lidar_pushed)
        {
            meas.lidar          = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 1)  // time too little
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                std::cerr << "Too few input point cloud!\n";
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            meas.lidar_end_time = lidar_end_time;

            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_time)
        {
            return false;
        }

        double imu_time = imu_buffer.front()->timestamp;
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_buffer.front()->timestamp;
            if (imu_time > lidar_end_time)
                break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    void MappingAlg::map_incremental()
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        PointToAdd.reserve(feats_down_size);
        PointNoNeedDownsample.reserve(feats_down_size);
        for (int i = 0; i < feats_down_size; i++)
        {
            pointsBody2World(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            if (!Nearest_Points[i].empty() && flg_EKF_inited)
            {
                const PointVector& points_near = Nearest_Points[i];
                bool               need_add    = true;
                BoxPointType       Box_of_Point;
                PointType          downsample_result, mid_point;
                mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
                float dist  = calc_dist(feats_down_world->points[i], mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min
                    && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min
                    && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
                {
                    PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                    continue;
                }
                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
                {
                    if (points_near.size() < NUM_MATCH_POINTS)
                        break;
                    if (calc_dist(points_near[readd_i], mid_point) < dist)
                    {
                        need_add = false;
                        break;
                    }
                }
                if (need_add)
                    PointToAdd.push_back(feats_down_world->points[i]);
            }
            else
            {
                PointToAdd.push_back(feats_down_world->points[i]);
            }
        }

        double st_time        = omp_get_wtime();
        int    add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    }

    void MappingAlg::pubWorldPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
    {

        PointCloudType::Ptr laserCloudFullRes(feats_undistort);
        int                 size = laserCloudFullRes->points.size();
        PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointsBody2World(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        recordKeyframe(laserCloudWorld);
        if (pub_world_points_flag_)
        {
            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
            laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time);
            laserCloudmsg.header.frame_id = "map";
            pubLaserCloudFull->publish(laserCloudmsg);
        }
    }

    void MappingAlg::recordKeyframe(const CloudPtr& cloud_world)
    {
        if (slam_diverged_ || !mapping_capture_enabled_ || !keyframe_record_enable_ || !cloud_world || cloud_world->empty() || active_map_subdir_.empty())
            return;
        const Vec3d lidar_origin = state_point.rot * state_point.offset_T_L_I + state_point.pos;
        const Mat3d world_rotation = state_point.rot.toRotationMatrix();
        const Mat3d lidar_rotation = (state_point.rot * state_point.offset_R_L_I).toRotationMatrix();
        const Eigen::Quaterniond world_quat = rotationToXyzw(world_rotation);
        const Eigen::Quaterniond lidar_quat = rotationToXyzw(lidar_rotation);
        const double yaw = std::atan2(lidar_rotation(1, 0), lidar_rotation(0, 0));
        const double distance = has_last_keyframe_ ? (lidar_origin - last_keyframe_origin_).norm() : std::numeric_limits<double>::infinity();
        const double elapsed = has_last_keyframe_ ? lidar_end_time - last_keyframe_stamp_ : std::numeric_limits<double>::infinity();
        double yaw_delta = std::fabs(yaw - last_keyframe_yaw_);
        yaw_delta = std::min(yaw_delta, 2.0 * M_PI - yaw_delta);
        if (has_last_keyframe_ && distance < keyframe_min_distance_m_ && yaw_delta < keyframe_min_yaw_rad_ && elapsed < keyframe_max_interval_s_)
            return;

        auto keyframe_cloud = CloudPtr(new PointCloudType());
        pcl::VoxelGrid<PointType> downsample;
        downsample.setInputCloud(cloud_world);
        downsample.setLeafSize(keyframe_voxel_size_m_, keyframe_voxel_size_m_, keyframe_voxel_size_m_);
        downsample.filter(*keyframe_cloud);

        MappingKeyframe metadata;
        metadata.index = mapping_keyframes_.size();
        metadata.stamp = lidar_end_time;
        metadata.world_origin = state_point.pos;
        metadata.world_qx = world_quat.x();
        metadata.world_qy = world_quat.y();
        metadata.world_qz = world_quat.z();
        metadata.world_qw = world_quat.w();
        metadata.lidar_origin = lidar_origin;
        metadata.lidar_qx = lidar_quat.x();
        metadata.lidar_qy = lidar_quat.y();
        metadata.lidar_qz = lidar_quat.z();
        metadata.lidar_qw = lidar_quat.w();
        metadata.yaw = yaw;
        metadata.point_count = keyframe_cloud->size();
        metadata.scan_context_index = metadata.index;
        metadata.preint = captureImuPreintegration(lidar_end_time);
        std::ostringstream file_name;
        file_name << active_map_subdir_ << "/keyframes/scan_" << std::setw(5) << std::setfill('0')
                  << metadata.index << ".pcd";
        metadata.file_path = file_name.str();
        std::ostringstream preint_name;
        preint_name << active_map_subdir_ << "/imu_preintegration/preint_" << std::setw(5) << std::setfill('0')
                    << metadata.index << ".json";
        metadata.preintegration_file = preint_name.str();
        {
            std::lock_guard<std::mutex> gnss_lock(gnss_mutex_);
            if (has_gnss_)
            {
                metadata.rtk_status = latest_gnss_.status.status;
                metadata.rtk_latitude = latest_gnss_.latitude;
                metadata.rtk_longitude = latest_gnss_.longitude;
                metadata.rtk_altitude = latest_gnss_.altitude;
                metadata.rtk_horizontal_std = std::sqrt(std::max(
                    0.0, std::max(latest_gnss_.position_covariance[0], latest_gnss_.position_covariance[4])));
                metadata.rtk_age_seconds = std::fabs(
                    lidar_end_time - get_time_sec(latest_gnss_.header.stamp));
                metadata.rtk_valid = metadata.rtk_status >= gnss_min_status_
                    && metadata.rtk_age_seconds <= gnss_max_age_
                    && metadata.rtk_horizontal_std <= gnss_max_horizontal_std_;
            }
        }
        {
            double heading_deg = 0.0;
            double heading_std_deg = 0.0;
            double heading_age = 0.0;
            metadata.rtk_heading_valid = gnssHeadingIsValid(heading_deg, heading_std_deg, heading_age);
            metadata.rtk_heading_deg = heading_deg;
            metadata.rtk_heading_std_deg = heading_std_deg;
            metadata.rtk_heading_age_seconds = heading_age;
        }

        {
            std::lock_guard<std::mutex> writer_lock(keyframe_writer_mutex_);
            if (keyframe_writer_failed_ || keyframe_write_queue_.size() >= keyframe_max_queue_size_)
            {
                ++dropped_keyframes_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Keyframe disk queue unavailable (queued=%zu, limit=%zu, failed=%s); retaining next candidate",
                    keyframe_write_queue_.size(), keyframe_max_queue_size_, keyframe_writer_failed_ ? "true" : "false");
                return;
            }
            mapping_keyframes_.push_back(metadata);
            keyframe_write_queue_.push_back(PendingKeyframe{ metadata, keyframe_cloud });
            if (has_last_keyframe_)
                keyframe_trajectory_m_ += distance;
        }
        keyframe_writer_cv_.notify_one();
        last_keyframe_origin_ = lidar_origin;
        last_keyframe_yaw_ = yaw;
        last_keyframe_stamp_ = lidar_end_time;
        has_last_keyframe_ = true;
        writeSaveProgress("mapping", 0.0);
    }

    bool MappingAlg::initializeKeyframeSession()
    {
        try
        {
            active_map_subdir_ = makeMapSubdir(data_path_);
            const auto keyframe_dir = std::filesystem::path(active_map_subdir_) / "keyframes";
            std::filesystem::create_directories(keyframe_dir);
            std::filesystem::create_directories(std::filesystem::path(active_map_subdir_) / "imu_preintegration");
            std::ofstream poses(keyframe_dir / "keyframes.csv", std::ios::out | std::ios::trunc);
            if (!poses.is_open())
                throw std::runtime_error("cannot create keyframes.csv");
            poses << "index,stamp,x,y,z,yaw,world_x,world_y,world_z,world_qx,world_qy,world_qz,world_qw,"
                     "lidar_x,lidar_y,lidar_z,lidar_qx,lidar_qy,lidar_qz,lidar_qw,point_count,preintegration_file,"
                     "scan_context_index,rtk_valid,rtk_status,rtk_latitude,rtk_longitude,rtk_altitude,"
                     "rtk_horizontal_std,rtk_age_seconds,rtk_heading_valid,rtk_heading_deg,rtk_heading_std_deg,"
                     "rtk_heading_age_seconds\n";
            poses.close();
            resetImuPreintegration(0.0);
            startKeyframeWriter();
            writeSaveProgress("mapping", 0.0);
            RCLCPP_INFO(get_logger(), "Mapping session initialized at %s", active_map_subdir_.c_str());
            return true;
        }
        catch (const std::exception& exc)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize mapping session: %s", exc.what());
            return false;
        }
    }

    bool MappingAlg::recoverLatestKeyframeSession()
    {
        std::filesystem::path latest_dir;
        std::filesystem::file_time_type latest_time {};
        std::error_code filesystem_error;
        for (const auto& entry : std::filesystem::directory_iterator(data_path_, filesystem_error))
        {
            if (filesystem_error || !entry.is_directory())
                continue;
            const auto keyframe_csv = entry.path() / "keyframes" / "keyframes.csv";
            if (!std::filesystem::is_regular_file(keyframe_csv))
                continue;
            if (std::filesystem::exists(entry.path() / "map.yaml")
                && std::filesystem::exists(entry.path() / "map.pgm"))
                continue;
            const auto modified = std::filesystem::last_write_time(keyframe_csv, filesystem_error);
            if (filesystem_error)
            {
                filesystem_error.clear();
                continue;
            }
            if (latest_dir.empty() || modified > latest_time)
            {
                latest_dir = entry.path();
                latest_time = modified;
            }
        }
        if (latest_dir.empty())
            return false;

        std::ifstream input(latest_dir / "keyframes" / "keyframes.csv");
        if (!input.is_open())
            return false;
        auto split = [](const std::string& line) {
            std::vector<std::string> values;
            std::stringstream stream(line);
            std::string value;
            while (std::getline(stream, value, ','))
                values.push_back(value);
            return values;
        };

        std::string line;
        if (!std::getline(input, line))
            return false;
        const auto header = split(line);
        std::map<std::string, std::size_t> columns;
        for (std::size_t index = 0; index < header.size(); ++index)
            columns.emplace(header[index], index);
        const auto has_columns = [&columns](std::initializer_list<const char*> names) {
            return std::all_of(names.begin(), names.end(), [&columns](const char* name) {
                return columns.find(name) != columns.end();
            });
        };
        if (!has_columns({ "index", "stamp", "x", "y", "z" }))
            return false;

        std::vector<MappingKeyframe> recovered;
        double recovered_trajectory = 0.0;
        while (std::getline(input, line))
        {
            if (line.empty())
                continue;
            const auto values = split(line);
            const auto field = [&values, &columns](const char* name, const std::string& fallback = "0") -> std::string {
                const auto column = columns.find(name);
                return column != columns.end() && column->second < values.size()
                    ? values[column->second]
                    : fallback;
            };
            try
            {
                MappingKeyframe keyframe;
                keyframe.index = static_cast<std::size_t>(std::stoull(field("index")));
                keyframe.stamp = std::stod(field("stamp"));
                keyframe.lidar_origin << std::stod(field("lidar_x", field("x"))),
                    std::stod(field("lidar_y", field("y"))), std::stod(field("lidar_z", field("z")));
                keyframe.yaw = std::stod(field("yaw"));
                const Eigen::Quaterniond recovered_yaw = normalizedXyzw(
                    0.0, 0.0, std::sin(keyframe.yaw * 0.5), std::cos(keyframe.yaw * 0.5));
                keyframe.world_origin << std::stod(field("world_x", field("lidar_x", field("x")))),
                    std::stod(field("world_y", field("lidar_y", field("y")))),
                    std::stod(field("world_z", field("lidar_z", field("z"))));
                keyframe.world_qx = std::stod(field("world_qx", "0"));
                keyframe.world_qy = std::stod(field("world_qy", "0"));
                keyframe.world_qz = std::stod(field("world_qz", std::to_string(recovered_yaw.z())));
                keyframe.world_qw = std::stod(field("world_qw", std::to_string(recovered_yaw.w())));
                keyframe.lidar_qx = std::stod(field("lidar_qx", "0"));
                keyframe.lidar_qy = std::stod(field("lidar_qy", "0"));
                keyframe.lidar_qz = std::stod(field("lidar_qz", std::to_string(recovered_yaw.z())));
                keyframe.lidar_qw = std::stod(field("lidar_qw", std::to_string(recovered_yaw.w())));
                const Eigen::Quaterniond world_quat = normalizedXyzw(
                    keyframe.world_qx, keyframe.world_qy, keyframe.world_qz, keyframe.world_qw);
                const Eigen::Quaterniond lidar_quat = normalizedXyzw(
                    keyframe.lidar_qx, keyframe.lidar_qy, keyframe.lidar_qz, keyframe.lidar_qw);
                keyframe.world_qx = world_quat.x();
                keyframe.world_qy = world_quat.y();
                keyframe.world_qz = world_quat.z();
                keyframe.world_qw = world_quat.w();
                keyframe.lidar_qx = lidar_quat.x();
                keyframe.lidar_qy = lidar_quat.y();
                keyframe.lidar_qz = lidar_quat.z();
                keyframe.lidar_qw = lidar_quat.w();
                keyframe.point_count = static_cast<std::size_t>(std::stoull(field("point_count")));
                keyframe.scan_context_index = static_cast<std::size_t>(std::stoull(field("scan_context_index", field("index"))));
                keyframe.preintegration_file = field("preintegration_file", "");
                keyframe.rtk_valid = std::stoi(field("rtk_valid")) != 0;
                keyframe.rtk_status = std::stoi(field("rtk_status", "-1"));
                keyframe.rtk_latitude = std::stod(field("rtk_latitude"));
                keyframe.rtk_longitude = std::stod(field("rtk_longitude"));
                keyframe.rtk_altitude = std::stod(field("rtk_altitude"));
                keyframe.rtk_horizontal_std = std::stod(field("rtk_horizontal_std"));
                keyframe.rtk_age_seconds = std::stod(field("rtk_age_seconds"));
                keyframe.rtk_heading_valid = std::stoi(field("rtk_heading_valid")) != 0;
                keyframe.rtk_heading_deg = std::stod(field("rtk_heading_deg"));
                keyframe.rtk_heading_std_deg = std::stod(field("rtk_heading_std_deg"));
                keyframe.rtk_heading_age_seconds = std::stod(field("rtk_heading_age_seconds"));
                std::ostringstream file_name;
                file_name << (latest_dir / "keyframes" / "scan_").string()
                          << std::setw(5) << std::setfill('0') << keyframe.index << ".pcd";
                keyframe.file_path = file_name.str();
                if (!std::filesystem::is_regular_file(keyframe.file_path))
                    continue;
                if (keyframe.point_count == 0)
                    keyframe.point_count = binaryPcdPointCount(keyframe.file_path);
                if (keyframe.point_count == 0)
                    continue;
                if (!recovered.empty())
                    recovered_trajectory += (keyframe.lidar_origin - recovered.back().lidar_origin).norm();
                recovered.push_back(std::move(keyframe));
            }
            catch (const std::exception& exc)
            {
                RCLCPP_WARN(get_logger(), "Skipping malformed recovered keyframe row: %s", exc.what());
            }
        }
        if (recovered.empty())
            return false;

        stopKeyframeWriter(false);
        {
            std::lock_guard<std::mutex> lock(keyframe_writer_mutex_);
            mapping_keyframes_ = std::move(recovered);
            keyframe_write_queue_.clear();
            written_keyframes_ = mapping_keyframes_.size();
            written_keyframe_points_ = 0;
            for (const auto& keyframe : mapping_keyframes_)
                written_keyframe_points_ += keyframe.point_count;
            keyframe_trajectory_m_ = recovered_trajectory;
            keyframe_writer_failed_ = false;
            keyframe_writer_error_.clear();
        }
        active_map_subdir_ = latest_dir.string();
        map_export_completed_ = false;
        writeSaveProgress("recovering", 1.0);
        RCLCPP_WARN(get_logger(), "Recovered %zu persistent keyframes from %s",
            mapping_keyframes_.size(), active_map_subdir_.c_str());
        return true;
    }

    void MappingAlg::startKeyframeWriter()
    {
        stopKeyframeWriter(false);
        {
            std::lock_guard<std::mutex> lock(keyframe_writer_mutex_);
            keyframe_writer_stop_ = false;
            keyframe_writer_active_ = false;
        }
        keyframe_writer_thread_ = std::thread(&MappingAlg::keyframeWriterLoop, this);
    }

    void MappingAlg::stopKeyframeWriter(bool drain)
    {
        if (!keyframe_writer_thread_.joinable())
            return;
        if (drain)
            flushKeyframeWriter();
        {
            std::lock_guard<std::mutex> lock(keyframe_writer_mutex_);
            if (!drain)
                keyframe_write_queue_.clear();
            keyframe_writer_stop_ = true;
        }
        keyframe_writer_cv_.notify_all();
        keyframe_writer_thread_.join();
    }

    bool MappingAlg::flushKeyframeWriter()
    {
        std::unique_lock<std::mutex> lock(keyframe_writer_mutex_);
        keyframe_writer_cv_.wait(lock, [this]() {
            return keyframe_writer_failed_ || (keyframe_write_queue_.empty() && !keyframe_writer_active_);
        });
        return !keyframe_writer_failed_;
    }

    void MappingAlg::keyframeWriterLoop()
    {
        pcl::PCDWriter writer;
        while (true)
        {
            PendingKeyframe pending;
            {
                std::unique_lock<std::mutex> lock(keyframe_writer_mutex_);
                keyframe_writer_cv_.wait(lock, [this]() {
                    return keyframe_writer_stop_ || !keyframe_write_queue_.empty();
                });
                if (keyframe_write_queue_.empty())
                {
                    if (keyframe_writer_stop_)
                        break;
                    continue;
                }
                pending = std::move(keyframe_write_queue_.front());
                keyframe_write_queue_.pop_front();
                keyframe_writer_active_ = true;
            }

            std::string failure;
            try
            {
                const std::string temporary = pending.metadata.file_path + ".tmp";
                if (writer.writeBinary(temporary, *pending.cloud_world) != 0)
                    throw std::runtime_error("PCL failed to write " + temporary);
                std::filesystem::rename(temporary, pending.metadata.file_path);

                std::ofstream poses(
                    std::filesystem::path(active_map_subdir_) / "keyframes" / "keyframes.csv",
                    std::ios::out | std::ios::app);
                if (!poses.is_open())
                    throw std::runtime_error("cannot append keyframes.csv");
                const auto& keyframe = pending.metadata;
                poses << keyframe.index << ',' << std::fixed << std::setprecision(6) << keyframe.stamp << ','
                      << keyframe.lidar_origin(0) << ',' << keyframe.lidar_origin(1) << ',' << keyframe.lidar_origin(2) << ','
                      << keyframe.yaw << ','
                      << keyframe.world_origin(0) << ',' << keyframe.world_origin(1) << ',' << keyframe.world_origin(2) << ','
                      << std::setprecision(7) << keyframe.world_qx << ',' << keyframe.world_qy << ','
                      << keyframe.world_qz << ',' << keyframe.world_qw << ','
                      << std::setprecision(6) << keyframe.lidar_origin(0) << ',' << keyframe.lidar_origin(1) << ','
                      << keyframe.lidar_origin(2) << ','
                      << std::setprecision(7) << keyframe.lidar_qx << ',' << keyframe.lidar_qy << ','
                      << keyframe.lidar_qz << ',' << keyframe.lidar_qw << ','
                      << keyframe.point_count << ',' << keyframe.preintegration_file << ','
                      << keyframe.scan_context_index << ','
                      << (keyframe.rtk_valid ? 1 : 0) << ',' << keyframe.rtk_status << ','
                      << std::setprecision(10) << keyframe.rtk_latitude << ',' << keyframe.rtk_longitude << ','
                      << std::setprecision(4) << keyframe.rtk_altitude << ',' << keyframe.rtk_horizontal_std << ','
                      << keyframe.rtk_age_seconds << ',' << (keyframe.rtk_heading_valid ? 1 : 0) << ','
                      << keyframe.rtk_heading_deg << ',' << keyframe.rtk_heading_std_deg << ','
                      << keyframe.rtk_heading_age_seconds << '\n';
                poses.close();
                if (!writeImuPreintegrationFile(keyframe))
                    throw std::runtime_error("cannot write " + keyframe.preintegration_file);
            }
            catch (const std::exception& exc)
            {
                failure = exc.what();
            }

            {
                std::lock_guard<std::mutex> lock(keyframe_writer_mutex_);
                if (!failure.empty())
                {
                    keyframe_writer_failed_ = true;
                    keyframe_writer_error_ = failure;
                    keyframe_write_queue_.clear();
                }
                else
                {
                    ++written_keyframes_;
                    written_keyframe_points_ += pending.metadata.point_count;
                }
                keyframe_writer_active_ = false;
            }
            keyframe_writer_cv_.notify_all();
            writeSaveProgress(failure.empty() ? "mapping" : "failed", 0.0, failure);
            if (!failure.empty())
                RCLCPP_ERROR(get_logger(), "Keyframe writer failed: %s", failure.c_str());
        }
    }

    void MappingAlg::pubBodyPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
    {
        int                 size = feats_undistort->points.size();
        PointCloudType::Ptr laserCloudIMUBody(new PointCloudType(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointsBody2Imu(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
        laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "body";
        pubLaserCloudFull_body->publish(laserCloudmsg);
    }

    void MappingAlg::pubMapPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
    {
        // PointCloudType::Ptr laserCloudFullRes(feats_down_body);
        // int                 size = laserCloudFullRes->points.size();
        // PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));

        // for (int i = 0; i < size; i++)
        // {
        //     pointsBody2World(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        // }
        // *pcl_wait_pub += *laserCloudWorld;

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
        laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "map";
        pubLaserCloudMap->publish(laserCloudmsg);
    }

    void MappingAlg::publish_odometry(
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster>& tf_br)
    {
        odomAftMapped.header.frame_id = "map";
        odomAftMapped.child_frame_id  = "body";
        odomAftMapped.header.stamp    = get_ros_time(lidar_end_time);
        set_posestamp(odomAftMapped.pose);
        pubOdomAftMapped->publish(odomAftMapped);
        auto P = kf.get_P();
        for (int i = 0; i < 6; i++)
        {
            int k                                    = i < 3 ? i + 3 : i - 3;
            odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
            odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
            odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
            odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
            odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
            odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
        }

        geometry_msgs::msg::TransformStamped trans;
        trans.header.frame_id         = "map";
        trans.child_frame_id          = "body";
        trans.header.stamp            = get_ros_time(lidar_end_time);
        trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
        trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
        trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
        trans.transform.rotation.w    = odomAftMapped.pose.pose.orientation.w;
        trans.transform.rotation.x    = odomAftMapped.pose.pose.orientation.x;
        trans.transform.rotation.y    = odomAftMapped.pose.pose.orientation.y;
        trans.transform.rotation.z    = odomAftMapped.pose.pose.orientation.z;
        tf_br->sendTransform(trans);
    }

    void MappingAlg::publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
    {
        set_posestamp(msg_body_pose);
        msg_body_pose.header.stamp    = get_ros_time(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
        msg_body_pose.header.frame_id = "map";

        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0)
        {
            path.poses.push_back(msg_body_pose);
            pubPath->publish(path);
        }
    }

    void MappingAlg::h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
    {
        laserCloudOri->clear();
        corr_normvect->clear();

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < feats_down_size; i++)
        {
            PointType& point_body = feats_down_body->points[i];
            PointType  point_world;

            Vec3d p_body(point_body.x, point_body.y, point_body.z);
            Vec3d p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
            point_world.x         = p_global(0);
            point_world.y         = p_global(1);
            point_world.z         = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto& points_near = Nearest_Points[i];

            if (ekfom_data.converge)
            {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                         : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                      : true;
            }

            if (!point_selected_surf[i])
                continue;

            // 添加额外的安全检查
            if (points_near.size() < NUM_MATCH_POINTS)
            {
                point_selected_surf[i] = false;
                continue;
            }

            VF(4) pabcd;
            point_selected_surf[i] = false;
            if (esti_plane<float>(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                float s   = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9)
                {
                    point_selected_surf[i]       = true;
                    normvec->points[i].x         = pabcd(0);
                    normvec->points[i].y         = pabcd(1);
                    normvec->points[i].z         = pabcd(2);
                    normvec->points[i].intensity = pd2;
                    res_last[i]                  = abs(pd2);
                }
            }
        }

        effct_feat_num = 0;

        for (int i = 0; i < feats_down_size; i++)
        {
            if (point_selected_surf[i])
            {
                laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                corr_normvect->points[effct_feat_num] = normvec->points[i];
                effct_feat_num++;
            }
        }

        if (effct_feat_num < 1)
        {
            ekfom_data.valid = false;
            std::cerr << "No Effective Points!" << std::endl;
            // ROS_WARN("No Effective Points! \n");
            return;
        }

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);  // 23
        ekfom_data.h.resize(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++)
        {
            const PointType& laser_p = laserCloudOri->points[i];
            Vec3d            point_this_be(laser_p.x, laser_p.y, laser_p.z);
            Mat3d            point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            Vec3d point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            Mat3d point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            const PointType& norm_p = corr_normvect->points[i];
            Vec3d            norm_vec(norm_p.x, norm_p.y, norm_p.z);

            Vec3d C(s.rot.conjugate() * norm_vec);
            Vec3d A(point_crossmat * C);
            if (extrinsic_est_en)
            {
                Vec3d B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);  // s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            }
            else
            {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            ekfom_data.h(i) = -norm_p.intensity;
        }
    }

    void MappingAlg::run()
    {
        if (state_.load() == SlamState::ACTIVE || state_.load() == SlamState::WARMUP)
        {

            if (syncData(Measures))
            {
                if (flg_first_scan)
                {
                    first_lidar_time        = Measures.lidar_beg_time;
                    p_imu->first_lidar_time = first_lidar_time;
                    flg_first_scan          = false;
                    return;
                }

                p_imu->Process(Measures, kf, feats_undistort);
                const bool odom_guard_active = applyOdomGuardPrediction(Measures.lidar_end_time);
                state_point = kf.get_x();
                pos_lid     = state_point.pos + state_point.rot * state_point.offset_T_L_I;

                if (last_mapping_progress_stamp_ <= 0.0
                    || Measures.lidar_end_time - last_mapping_progress_stamp_ >= 1.0)
                {
                    const char* stage = !p_imu->initialization_ready()
                        ? "initializing_imu"
                        : (!slam_pose_ready_ ? "waiting_first_keyframe"
                            : (!mapping_capture_enabled_ ? "ready_to_map"
                                : (mapping_keyframes_.empty() ? "waiting_first_keyframe" : "mapping")));
                    writeSaveProgress(stage, 0.0);
                    last_mapping_progress_stamp_ = Measures.lidar_end_time;
                }

                if (feats_undistort->empty())
                {
                    RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                    return;
                }

                flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
                lasermap_fov_segment();

                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                feats_down_size = feats_down_body->points.size();
                if (ikdtree.Root_Node == nullptr)
                {
                    RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                    if (feats_down_size > 5)
                    {
                        ikdtree.set_downsample_param(filter_size_map_min);
                        feats_down_world->resize(feats_down_size);
                        for (int i = 0; i < feats_down_size; i++)
                        {
                            pointsBody2World(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                        }
                        ikdtree.Build(feats_down_world->points);
                    }
                    return;
                }
                if (feats_down_size < 5)
                {
                    RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                    return;
                }

                normvec->resize(feats_down_size);
                feats_down_world->resize(feats_down_size);

                Vec3d ext_euler = SO3ToEuler(state_point.offset_R_L_I);

                if (0)  // If you need to see map point, change to "if(1)"
                {
                    PointVector().swap(ikdtree.PCL_Storage);
                    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;
                }

                pointSearchInd_surf.resize(feats_down_size);
                Nearest_Points.resize(feats_down_size);
                int  rematch_num       = 0;
                bool nearest_search_en = true;  //

                const state_ikfom guarded_prediction = kf.get_x();
                double solve_H_time = 0;
                kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
                state_point = kf.get_x();
                const bool lidar_update_accepted = !odom_guard_active
                    || acceptOdomGuardCorrection(Measures.lidar_end_time, guarded_prediction);
                collectGnssAlignment(Measures.lidar_end_time);
                applyGnssCorrection(Measures.lidar_end_time);
                state_point = kf.get_x();
                updateSlamHealth(Measures.lidar_end_time);
                if (slam_diverged_)
                    return;
                euler_cur   = SO3ToEuler(state_point.rot);
                pos_lid     = state_point.pos + state_point.rot * state_point.offset_T_L_I;
                geoQuat.x   = state_point.rot.coeffs()[0];
                geoQuat.y   = state_point.rot.coeffs()[1];
                geoQuat.z   = state_point.rot.coeffs()[2];
                geoQuat.w   = state_point.rot.coeffs()[3];
                if (!lidar_update_accepted)
                {
                    publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                    if (path_en)
                        publish_path(pubPath_);
                    return;
                }
                map_incremental();
                slam_pose_ready_ = p_imu->initialization_ready() && ikdtree.Root_Node != nullptr;

                publish_odometry(pubOdomAftMapped_, tf_broadcaster_);
                pubWorldPoints(pubLaserCloudFull_);

                if (path_en)
                    publish_path(pubPath_);
                if (pub_body_points_flag_)
                    pubBodyPoints(pubLaserCloudFull_body_);
            }
        }
        else if (state_.load() == SlamState::SAVE)
        {
            RCLCPP_INFO(get_logger(), "Map export started: %zu keyframes", mapping_keyframes_.size());
            const bool saved = finish();
            state_.store(saved ? SlamState::READY : SlamState::ERROR);
            RCLCPP_INFO(get_logger(), "Map export %s", saved ? "completed" : "failed");
        }
        else
        {
            return;
        }
    }

    void MappingAlg::map_publish_callback()
    {
        if (map_pub_en)
            pubMapPoints(pubLaserCloudMap_);
    }

    bool MappingAlg::streamMapFromKeyframes(const std::string& map_subdir, std::size_t& written_points)
    {
        const bool filter_enabled = dynamic_filter_enable_ && dynamic_filter_voxel_size_ > 0.0
            && dynamic_filter_min_scan_observations_ > 1;
        const std::size_t shard_count = filter_enabled ? dynamic_filter_shard_count_ : 1;
        const auto shard_dir = std::filesystem::path(map_subdir) / ".dynamic_filter_shards.tmp";
        std::error_code filesystem_error;
        std::filesystem::remove_all(shard_dir, filesystem_error);
        filesystem_error.clear();
        std::filesystem::create_directories(shard_dir, filesystem_error);
        if (filesystem_error)
        {
            keyframe_writer_error_ = "cannot create map filter shard directory: " + filesystem_error.message();
            return false;
        }
        const auto fail = [this, &shard_dir](const std::string& message) {
            keyframe_writer_error_ = message;
            std::error_code cleanup_error;
            std::filesystem::remove_all(shard_dir, cleanup_error);
            return false;
        };

        std::uint64_t estimated_source_bytes = 0;
        for (const auto& keyframe : mapping_keyframes_)
            estimated_source_bytes += keyframe.point_count * sizeof(ExportPointRecord);
        // Temporary shards plus the 3D localization PCD and slope-normalized
        // 2D projection PCD coexist briefly during export.
        const std::uint64_t required_free_bytes = estimated_source_bytes * 3
            + std::max<std::uint64_t>(estimated_source_bytes / 4, 512ULL * 1024ULL * 1024ULL);
        const std::uint64_t free_bytes = freeDiskBytes(map_subdir);
        if (free_bytes > 0 && free_bytes < required_free_bytes)
        {
            std::ostringstream message;
            message << "insufficient disk space for streaming map export: free=" << free_bytes
                    << " required=" << required_free_bytes;
            return fail(message.str());
        }

        std::vector<std::ofstream> point_shards;
        std::vector<std::ofstream> hit_shards;
        point_shards.reserve(shard_count);
        hit_shards.reserve(shard_count);
        for (std::size_t shard = 0; shard < shard_count; ++shard)
        {
            point_shards.emplace_back(
                shard_dir / ("points_" + std::to_string(shard) + ".bin"),
                std::ios::binary | std::ios::trunc);
            if (!point_shards.back().is_open())
                return fail("cannot create point filter shard");
            if (filter_enabled)
            {
                hit_shards.emplace_back(
                    shard_dir / ("hits_" + std::to_string(shard) + ".bin"),
                    std::ios::binary | std::ios::trunc);
                if (!hit_shards.back().is_open())
                    return fail("cannot create voxel evidence shard");
            }
        }

        const DynamicFilterVoxelKeyHash key_hash;
        std::size_t source_points = 0;
        for (std::size_t index = 0; index < mapping_keyframes_.size(); ++index)
        {
            const auto& keyframe = mapping_keyframes_[index];
            PointCloudType cloud;
            if (pcl::io::loadPCDFile<PointType>(keyframe.file_path, cloud) != 0)
                return fail("cannot read keyframe: " + keyframe.file_path);

            Eigen::Isometry3d raw_lidar = Eigen::Translation3d(keyframe.lidar_origin)
                * normalizedXyzw(keyframe.lidar_qx, keyframe.lidar_qy, keyframe.lidar_qz, keyframe.lidar_qw);
            Eigen::Isometry3d raw_to_optimized = Eigen::Isometry3d::Identity();
            double optimized_reference_z = keyframe.lidar_origin.z();
            if (global_optimization_applied_ && index < optimized_global_poses_.size())
            {
                const Eigen::Isometry3d optimized_lidar = optimizedLidarPose(keyframe, optimized_global_poses_[index]);
                raw_to_optimized = optimized_lidar * raw_lidar.inverse();
                optimized_reference_z = optimized_lidar.translation().z();
            }

            std::unordered_set<DynamicFilterVoxelKey, DynamicFilterVoxelKeyHash> observed_in_keyframe;
            if (filter_enabled)
                observed_in_keyframe.reserve(cloud.size());
            for (const auto& point : cloud.points)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                    continue;
                const Eigen::Vector3d corrected = raw_to_optimized * Eigen::Vector3d(point.x, point.y, point.z);
                BinaryPcdPoint corrected_point = packPoint(point);
                corrected_point.x = static_cast<float>(corrected.x());
                corrected_point.y = static_cast<float>(corrected.y());
                corrected_point.z = static_cast<float>(corrected.z());
                const PointType corrected_pcl = [&]() {
                    PointType value = point;
                    value.x = corrected_point.x;
                    value.y = corrected_point.y;
                    value.z = corrected_point.z;
                    return value;
                }();
                const auto key = makeDynamicFilterVoxelKey(corrected_pcl, dynamic_filter_voxel_size_);
                const std::size_t shard = filter_enabled ? key_hash(key) % shard_count : 0;
                const ExportPointRecord packed{ corrected_point, static_cast<float>(optimized_reference_z) };
                point_shards[shard].write(
                    reinterpret_cast<const char*>(&packed), sizeof(ExportPointRecord));
                if (!point_shards[shard].good())
                    return fail("failed while writing point filter shard");
                if (filter_enabled)
                    observed_in_keyframe.insert(key);
                ++source_points;
            }
            if (filter_enabled)
            {
                for (const auto& key : observed_in_keyframe)
                {
                    const std::size_t shard = key_hash(key) % shard_count;
                    hit_shards[shard].write(
                        reinterpret_cast<const char*>(&key), sizeof(DynamicFilterVoxelKey));
                    if (!hit_shards[shard].good())
                        return fail("failed while writing voxel evidence shard");
                }
            }
            writeSaveProgress("partitioning_filter",
                8.0 + 27.0 * static_cast<double>(index + 1) / static_cast<double>(mapping_keyframes_.size()));
        }
        for (auto& stream : point_shards)
            stream.close();
        for (auto& stream : hit_shards)
            stream.close();

        const std::string pcd_file = map_subdir + "/map.pcd";
        const std::string pcd_tmp = pcd_file + ".tmp";
        const std::string grid_pcd_file = map_subdir + "/.map_grid_relative.pcd";
        const std::string grid_pcd_tmp = grid_pcd_file + ".tmp";
        std::ofstream pcd(pcd_tmp, std::ios::binary | std::ios::trunc);
        std::ofstream grid_pcd(grid_pcd_tmp, std::ios::binary | std::ios::trunc);
        if (!pcd.is_open() || !grid_pcd.is_open())
            return fail("cannot create map PCD outputs");
        const auto write_pcd_header = [](std::ofstream& stream, std::streampos& width, std::streampos& points) {
            stream << "# .PCD v0.7 - Point Cloud Data file format\n"
                   << "VERSION 0.7\n"
                   << "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
                   << "SIZE 4 4 4 4 4 4 4 4\n"
                   << "TYPE F F F F F F F F\n"
                   << "COUNT 1 1 1 1 1 1 1 1\n"
                   << "WIDTH ";
            width = stream.tellp();
            stream << "00000000000000000000\n"
                   << "HEIGHT 1\n"
                   << "VIEWPOINT 0 0 0 1 0 0 0\n"
                   << "POINTS ";
            points = stream.tellp();
            stream << "00000000000000000000\n"
                   << "DATA binary\n";
        };
        std::streampos width_position, points_position, grid_width_position, grid_points_position;
        write_pcd_header(pcd, width_position, points_position);
        write_pcd_header(grid_pcd, grid_width_position, grid_points_position);

        constexpr std::size_t chunk_records = 65536;
        std::vector<DynamicFilterVoxelKey> hit_buffer(chunk_records);
        std::vector<ExportPointRecord> point_buffer(chunk_records);
        written_points = 0;
        const auto minimum_hits = static_cast<std::uint8_t>(
            std::clamp(dynamic_filter_min_scan_observations_, 1, 255));
        for (std::size_t shard = 0; shard < shard_count; ++shard)
        {
            std::unordered_map<DynamicFilterVoxelKey, std::uint8_t, DynamicFilterVoxelKeyHash> hit_counts;
            const auto hit_path = shard_dir / ("hits_" + std::to_string(shard) + ".bin");
            if (filter_enabled)
            {
                std::ifstream hits(hit_path, std::ios::binary);
                if (!hits.is_open())
                    return fail("cannot read voxel evidence shard");
                const auto hit_bytes = std::filesystem::file_size(hit_path, filesystem_error);
                if (!filesystem_error)
                    hit_counts.reserve(static_cast<std::size_t>(hit_bytes / sizeof(DynamicFilterVoxelKey)));
                filesystem_error.clear();
                while (hits.good())
                {
                    hits.read(reinterpret_cast<char*>(hit_buffer.data()),
                        static_cast<std::streamsize>(hit_buffer.size() * sizeof(DynamicFilterVoxelKey)));
                    const auto received = static_cast<std::size_t>(hits.gcount()) / sizeof(DynamicFilterVoxelKey);
                    for (std::size_t index = 0; index < received; ++index)
                    {
                        auto& count = hit_counts[hit_buffer[index]];
                        if (count < minimum_hits)
                            ++count;
                    }
                }
            }

            const auto point_path = shard_dir / ("points_" + std::to_string(shard) + ".bin");
            std::ifstream points(point_path, std::ios::binary);
            if (!points.is_open())
                return fail("cannot read point filter shard");
            while (points.good())
            {
                points.read(reinterpret_cast<char*>(point_buffer.data()),
                    static_cast<std::streamsize>(point_buffer.size() * sizeof(ExportPointRecord)));
                const auto received = static_cast<std::size_t>(points.gcount()) / sizeof(ExportPointRecord);
                for (std::size_t index = 0; index < received; ++index)
                {
                    const auto& record = point_buffer[index];
                    const auto& point = record.point;
                    bool keep = true;
                    if (filter_enabled)
                    {
                        PointType pcl_point;
                        pcl_point.x = point.x;
                        pcl_point.y = point.y;
                        pcl_point.z = point.z;
                        const auto count = hit_counts.find(
                            makeDynamicFilterVoxelKey(pcl_point, dynamic_filter_voxel_size_));
                        keep = count != hit_counts.end() && count->second >= minimum_hits;
                    }
                    if (!keep)
                        continue;
                    pcd.write(reinterpret_cast<const char*>(&point), sizeof(BinaryPcdPoint));
                    auto grid_point = point;
                    grid_point.z -= record.reference_z;
                    grid_pcd.write(reinterpret_cast<const char*>(&grid_point), sizeof(BinaryPcdPoint));
                    ++written_points;
                }
                if (!pcd.good() || !grid_pcd.good())
                    return fail("failed while writing map PCD outputs");
            }
            points.close();
            hit_counts.clear();
            std::filesystem::remove(point_path, filesystem_error);
            filesystem_error.clear();
            if (filter_enabled)
            {
                std::filesystem::remove(hit_path, filesystem_error);
                filesystem_error.clear();
            }
            writeSaveProgress("writing_pcd",
                35.0 + 30.0 * static_cast<double>(shard + 1) / static_cast<double>(shard_count));
        }
        if (written_points == 0)
        {
            pcd.close();
            grid_pcd.close();
            std::filesystem::remove(pcd_tmp, filesystem_error);
            std::filesystem::remove(grid_pcd_tmp, filesystem_error);
            return fail("dynamic filter removed every map point");
        }
        const auto finalize_pcd = [written_points](std::ofstream& stream, std::streampos width, std::streampos points) {
            stream.seekp(width);
            stream << std::setw(20) << std::setfill('0') << written_points;
            stream.seekp(points);
            stream << std::setw(20) << std::setfill('0') << written_points;
            stream.flush();
            stream.close();
            return stream.good();
        };
        if (!finalize_pcd(pcd, width_position, points_position)
            || !finalize_pcd(grid_pcd, grid_width_position, grid_points_position))
            return fail("failed while closing map PCD outputs");
        std::filesystem::rename(pcd_tmp, pcd_file, filesystem_error);
        if (filesystem_error)
            return fail("cannot publish final map.pcd: " + filesystem_error.message());
        filesystem_error.clear();
        std::filesystem::rename(grid_pcd_tmp, grid_pcd_file, filesystem_error);
        if (filesystem_error)
            return fail("cannot publish slope-normalized grid PCD: " + filesystem_error.message());
        std::filesystem::remove_all(shard_dir, filesystem_error);

        const double keep_ratio = source_points == 0
            ? 0.0
            : static_cast<double>(written_points) / static_cast<double>(source_points);
        RCLCPP_INFO(get_logger(),
            "Disk-sharded map export: keyframes=%zu source=%zu filtered=%zu keep_ratio=%.3f shards=%zu rss=%llu",
            mapping_keyframes_.size(), source_points, written_points, keep_ratio, shard_count,
            static_cast<unsigned long long>(residentSetBytes()));
        return true;
    }

    bool MappingAlg::loadLoopClosures(const std::string& map_subdir,
        std::vector<GlobalGraphLoopClosure>& loop_closures)
    {
        loop_closures.clear();
        scan_context_count_ = 0;
        loop_status_ = "no_valid_loop";
        const auto file = std::filesystem::path(map_subdir) / "loop_closures.csv";
        std::ifstream input;
        bool converting_scan_context_candidates = false;
        std::ofstream converted;
        if (std::filesystem::exists(file))
        {
            input.open(file);
        }
        else
        {
            const auto candidates_file = std::filesystem::path(map_subdir)
                / "scan_context" / "loop_candidates.csv";
            if (!std::filesystem::exists(candidates_file))
                return true;
            input.open(candidates_file);
            converting_scan_context_candidates = true;
            converted.open(file, std::ios::out | std::ios::trunc);
            if (!converted.is_open())
            {
                keyframe_writer_error_ = "cannot create loop_closures.csv";
                return false;
            }
            converted << "from,to,tx,ty,tz,qx,qy,qz,qw,sigma_translation,sigma_rotation,score\n";
            scan_context_count_ = mapping_keyframes_.size();
        }
        if (!input.is_open())
            return false;
        std::string line;
        std::size_t line_number = 0;
        while (std::getline(input, line))
        {
            ++line_number;
            if (line.empty() || line[0] == '#')
                continue;
            std::vector<std::string> fields;
            std::stringstream stream(line);
            std::string field;
            while (std::getline(stream, field, ','))
                fields.push_back(field);
            if (fields.empty() || fields.front() == "from"
                || fields.front() == "query_index")
                continue;
            if (fields.size() < 10)
            {
                RCLCPP_WARN(get_logger(), "Ignoring malformed loop closure line %zu", line_number);
                continue;
            }
            try
            {
                GlobalGraphLoopClosure loop;
                const double translation_sigma = converting_scan_context_candidates ? 0.50
                    : std::max(0.01, std::stod(fields[9]));
                const double rotation_sigma = converting_scan_context_candidates ? 0.15
                    : (fields.size() > 10
                        ? std::max(0.005, std::stod(fields[10]))
                        : global_factor_graph_config_.loop_rotation_sigma_rad);
                if (converting_scan_context_candidates)
                {
                    const auto accepted = fields.size() > 6 &&
                        (fields[6] == "true" || fields[6] == "True" || fields[6] == "1");
                    const auto geometrically_verified = fields.size() > 5 &&
                        (fields[5] == "true" || fields[5] == "True" || fields[5] == "1");
                    if (!accepted || !geometrically_verified)
                        continue;
                    loop.from = static_cast<std::size_t>(std::stoull(fields[0]));
                    loop.to = static_cast<std::size_t>(std::stoull(fields[1]));
                    const double tx = std::stod(fields[7]);
                    const double ty = std::stod(fields[8]);
                    const double yaw = std::stod(fields[4]);
                    loop.relative_pose = gtsam::Pose3(
                        gtsam::Rot3::Yaw(yaw), gtsam::Point3(tx, ty, 0.0));
                    loop.score = 1.0 - std::clamp(std::stod(fields[3]), 0.0, 1.0);
                    converted << loop.from << ',' << loop.to << ',' << tx << ',' << ty << ",0,"
                              << "0,0," << std::sin(yaw / 2.0) << ',' << std::cos(yaw / 2.0) << ','
                              << translation_sigma << ',' << rotation_sigma << ',' << loop.score << '\n';
                }
                else
                {
                    loop.from = static_cast<std::size_t>(std::stoull(fields[0]));
                    loop.to = static_cast<std::size_t>(std::stoull(fields[1]));
                    const double tx = std::stod(fields[2]);
                    const double ty = std::stod(fields[3]);
                    const double tz = std::stod(fields[4]);
                    const double qx = std::stod(fields[5]);
                    const double qy = std::stod(fields[6]);
                    const double qz = std::stod(fields[7]);
                    const double qw = std::stod(fields[8]);
                    loop.relative_pose = gtsam::Pose3(
                        gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
                    loop.score = fields.size() > 11 ? std::stod(fields[11]) : 0.0;
                }
                loop.covariance.setZero();
                loop.covariance.block<3, 3>(0, 0) = gtsam::Matrix3::Identity() * (rotation_sigma * rotation_sigma);
                loop.covariance.block<3, 3>(3, 3) = gtsam::Matrix3::Identity() * (translation_sigma * translation_sigma);
                loop_closures.push_back(loop);
            }
            catch (const std::exception& exception)
            {
                RCLCPP_WARN(get_logger(), "Ignoring loop_closures.csv line %zu: %s", line_number, exception.what());
            }
        }
        if (converting_scan_context_candidates)
        {
            converted.close();
            loop_status_ = loop_closures.empty() ? "no_valid_loop" : "accepted";
        }
        else if (!loop_closures.empty())
        {
            loop_status_ = "accepted";
            scan_context_count_ = std::filesystem::exists(
                std::filesystem::path(map_subdir) / "scan_context" / "index.json")
                ? mapping_keyframes_.size() : 0;
        }
        return true;
    }

    bool MappingAlg::writeGlobalOptimizationOutputs(const std::string& map_subdir,
        const std::vector<GlobalGraphLoopClosure>& loop_closures)
    {
        (void)loop_closures;
        if (!global_optimization_applied_ || optimized_global_poses_.size() != mapping_keyframes_.size())
            return true;

        std::ofstream trajectory(map_subdir + "/trajectory_optimized.csv", std::ios::out | std::ios::trunc);
        std::ofstream covariance(map_subdir + "/trajectory_covariance.json", std::ios::out | std::ios::trunc);
        if (!trajectory.is_open() || !covariance.is_open())
        {
            keyframe_writer_error_ = "cannot create global optimization outputs";
            return false;
        }
        trajectory << "index,timestamp,world_x,world_y,world_z,world_qx,world_qy,world_qz,world_qw\n";
        covariance << "{\n  \"schema_version\": 1,\n  \"ordering\": \"rotation_xyz,translation_xyz\",\n  \"poses\": [\n";
        nav_msgs::msg::Path optimized_path;
        optimized_path.header.frame_id = "map";
        optimized_path.header.stamp = get_clock()->now();
        for (std::size_t index = 0; index < optimized_global_poses_.size(); ++index)
        {
            const auto& pose = optimized_global_poses_[index];
            const auto quaternion = pose.rotation().toQuaternion();
            trajectory << mapping_keyframes_[index].index << ',' << std::fixed << std::setprecision(6)
                       << mapping_keyframes_[index].stamp << ',' << pose.x() << ',' << pose.y() << ',' << pose.z()
                       << ',' << std::setprecision(8) << quaternion.x() << ',' << quaternion.y() << ','
                       << quaternion.z() << ',' << quaternion.w() << '\n';
            const auto& matrix = global_pose_covariances_[index];
            covariance << "    {\"index\": " << mapping_keyframes_[index].index << ", \"covariance\": [";
            for (int row = 0; row < 6; ++row)
                for (int col = 0; col < 6; ++col)
                    covariance << (row || col ? ", " : "") << std::setprecision(10) << matrix(row, col);
            covariance << "]}" << (index + 1 == optimized_global_poses_.size() ? "\n" : ",\n");

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = optimized_path.header;
            pose_stamped.header.stamp = get_ros_time(mapping_keyframes_[index].stamp);
            pose_stamped.pose.position.x = pose.x();
            pose_stamped.pose.position.y = pose.y();
            pose_stamped.pose.position.z = pose.z();
            pose_stamped.pose.orientation.x = quaternion.x();
            pose_stamped.pose.orientation.y = quaternion.y();
            pose_stamped.pose.orientation.z = quaternion.z();
            pose_stamped.pose.orientation.w = quaternion.w();
            optimized_path.poses.push_back(pose_stamped);
        }
        covariance << "  ],\n  \"factor_count\": " << global_factor_graph_result_.factor_count
                   << ",\n  \"ndt_factor_count\": " << global_factor_graph_result_.ndt_factor_count
                   << ",\n  \"imu_factor_count\": " << global_factor_graph_result_.imu_factor_count
                   << ",\n  \"rtk_position_factor_count\": " << global_factor_graph_result_.rtk_position_factor_count
                   << ",\n  \"rtk_heading_factor_count\": " << global_factor_graph_result_.rtk_heading_factor_count
                   << ",\n  \"loop_closure_factor_count\": " << global_factor_graph_result_.loop_closure_factor_count
                   << ",\n  \"error_before\": " << global_factor_graph_result_.error_before
                   << ",\n  \"error_after\": " << global_factor_graph_result_.error_after << "\n}\n";

        if (pubGlobalOptimizedPath_)
            pubGlobalOptimizedPath_->publish(optimized_path);
        if (pubGlobalOptimizedOdom_ && !optimized_global_poses_.empty())
        {
            const std::size_t last = optimized_global_poses_.size() - 1;
            const auto& pose = optimized_global_poses_.back();
            const auto quaternion = pose.rotation().toQuaternion();
            nav_msgs::msg::Odometry odom;
            odom.header = optimized_path.header;
            odom.header.stamp = get_ros_time(mapping_keyframes_.back().stamp);
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = pose.x();
            odom.pose.pose.position.y = pose.y();
            odom.pose.pose.position.z = pose.z();
            odom.pose.pose.orientation.x = quaternion.x();
            odom.pose.pose.orientation.y = quaternion.y();
            odom.pose.pose.orientation.z = quaternion.z();
            odom.pose.pose.orientation.w = quaternion.w();
            for (int row = 0; row < 6; ++row)
                for (int col = 0; col < 6; ++col)
                {
                    const int ros_row = row < 3 ? row + 3 : row - 3;
                    const int ros_col = col < 3 ? col + 3 : col - 3;
                    odom.pose.covariance[ros_row * 6 + ros_col] = global_pose_covariances_[last](row, col);
                }
            pubGlobalOptimizedOdom_->publish(odom);
        }
        if (pubGlobalOptimizationStatus_)
        {
            std_msgs::msg::String status;
            std::ostringstream json;
            json << "{\"success\":true,\"keyframe_count\":" << mapping_keyframes_.size()
                 << ",\"factor_count\":" << global_factor_graph_result_.factor_count
                 << ",\"loop_closure_count\":" << global_factor_graph_result_.loop_closure_factor_count
                 << ",\"error_before\":" << global_factor_graph_result_.error_before
                 << ",\"error_after\":" << global_factor_graph_result_.error_after << '}';
            status.data = json.str();
            pubGlobalOptimizationStatus_->publish(status);
        }
        return true;
    }

    bool MappingAlg::optimizeHistoricalTrajectory(const std::string& map_subdir)
    {
        optimized_global_poses_.clear();
        global_pose_covariances_.clear();
        global_optimization_applied_ = false;
        if (!global_factor_graph_config_.enable || mapping_keyframes_.empty())
            return true;

        std::vector<GlobalGraphLoopClosure> loop_closures;
        if (!loadLoopClosures(map_subdir, loop_closures))
        {
            keyframe_writer_error_ = "cannot read loop_closures.csv";
            return false;
        }
        std::vector<GlobalGraphKeyframe> graph_keyframes;
        graph_keyframes.reserve(mapping_keyframes_.size());
        for (const auto& keyframe : mapping_keyframes_)
        {
            GlobalGraphKeyframe frame;
            frame.index = graph_keyframes.size();
            frame.stamp = keyframe.stamp;
            frame.initial_pose = gtsam::Pose3(
                gtsam::Rot3::Quaternion(keyframe.world_qw, keyframe.world_qx, keyframe.world_qy, keyframe.world_qz),
                gtsam::Point3(keyframe.world_origin(0), keyframe.world_origin(1), keyframe.world_origin(2)));
            if (keyframe.rtk_valid && gnss_origin_initialized_ && gnss_alignment_locked_)
            {
                sensor_msgs::msg::NavSatFix fix;
                fix.status.status = keyframe.rtk_status;
                fix.latitude = keyframe.rtk_latitude;
                fix.longitude = keyframe.rtk_longitude;
                fix.altitude = keyframe.rtk_altitude;
                fix.position_covariance[0] = keyframe.rtk_horizontal_std * keyframe.rtk_horizontal_std;
                fix.position_covariance[4] = fix.position_covariance[0];
                Vec3d map_position;
                if (gnssToMap(fix, map_position))
                {
                    frame.has_rtk_position = true;
                    frame.rtk_position = gtsam::Point3(map_position(0), map_position(1), map_position(2));
                    frame.rtk_position_sigma = std::max(0.05, keyframe.rtk_horizontal_std);
                    frame.rtk_lever_arm = gtsam::Point3(gnss_lever_arm_base_(0), gnss_lever_arm_base_(1), gnss_lever_arm_base_(2));
                }
            }
            if (keyframe.rtk_heading_valid && gnss_use_heading_)
            {
                frame.has_rtk_heading = true;
                frame.rtk_heading_rad = M_PI / 2.0 - keyframe.rtk_heading_deg * M_PI / 180.0
                    + gnss_enu_to_map_yaw_ + gnss_heading_offset_rad_;
                frame.rtk_heading_sigma_rad = std::max(0.005, keyframe.rtk_heading_std_deg * M_PI / 180.0);
            }
            const auto& preint = keyframe.preint;
            if (preint.imu_sample_count > 0 && preint.error_status.empty())
            {
                frame.has_imu_delta = true;
                frame.imu_delta = gtsam::Pose3(
                    gtsam::Rot3::Quaternion(preint.delta_rotation_xyzw[3], preint.delta_rotation_xyzw[0],
                        preint.delta_rotation_xyzw[1], preint.delta_rotation_xyzw[2]),
                    gtsam::Point3(preint.delta_position(0), preint.delta_position(1), preint.delta_position(2)));
                frame.imu_covariance.setZero();
                for (int row = 0; row < 3; ++row)
                    for (int col = 0; col < 3; ++col)
                    {
                        frame.imu_covariance(row, col) = preint.covariance[static_cast<std::size_t>(row * 15 + col)];
                        frame.imu_covariance(row + 3, col + 3) = preint.covariance[
                            static_cast<std::size_t>((row + 6) * 15 + col + 6)];
                    }
            }
            graph_keyframes.push_back(frame);
        }
        global_factor_graph_result_ = global_factor_graph_->optimize(graph_keyframes, loop_closures);
        if (!global_factor_graph_result_.success)
        {
            keyframe_writer_error_ = "global factor graph failed: " + global_factor_graph_result_.error;
            RCLCPP_ERROR(get_logger(), "%s", keyframe_writer_error_.c_str());
            if (pubGlobalOptimizationStatus_)
            {
                std_msgs::msg::String status;
                status.data = std::string("{\"success\":false,\"error\":\"")
                    + jsonEscape(global_factor_graph_result_.error) + "\"}";
                pubGlobalOptimizationStatus_->publish(status);
            }
            return false;
        }
        optimized_global_poses_ = global_factor_graph_result_.optimized_poses;
        global_pose_covariances_ = global_factor_graph_result_.covariances;
        global_optimization_applied_ = true;
        RCLCPP_INFO(get_logger(),
            "Historical GTSAM optimization completed: keyframes=%zu factors=%zu ndt=%zu imu=%zu rtk_xy=%zu rtk_heading=%zu loops=%zu error %.3f -> %.3f",
            mapping_keyframes_.size(), global_factor_graph_result_.factor_count,
            global_factor_graph_result_.ndt_factor_count, global_factor_graph_result_.imu_factor_count,
            global_factor_graph_result_.rtk_position_factor_count, global_factor_graph_result_.rtk_heading_factor_count,
            global_factor_graph_result_.loop_closure_factor_count,
            global_factor_graph_result_.error_before, global_factor_graph_result_.error_after);
        return writeGlobalOptimizationOutputs(map_subdir, loop_closures);
    }

    bool MappingAlg::writeTrajectoryAndGnssMetadata(const std::string& map_subdir)
    {
        std::ofstream trajectory(map_subdir + "/map.txt", std::ios::out | std::ios::trunc);
        if (!trajectory.is_open())
        {
            keyframe_writer_error_ = "cannot create map.txt";
            return false;
        }
        trajectory << "# path\n";
        if (global_optimization_applied_ && !optimized_global_poses_.empty())
        {
            for (const auto& pose : optimized_global_poses_)
                trajectory << std::fixed << std::setprecision(2) << pose.x() << ' '
                           << pose.y() << ' ' << pose.rotation().yaw() << '\n';
        }
        else if (!path.poses.empty())
        {
            for (const auto& pose : path.poses)
            {
                const double theta = QuaternionToYaw(pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w);
                trajectory << std::fixed << std::setprecision(2) << pose.pose.position.x << ' '
                           << pose.pose.position.y << ' ' << theta << '\n';
            }
        }
        else
        {
            for (const auto& keyframe : mapping_keyframes_)
            {
                trajectory << std::fixed << std::setprecision(2) << keyframe.lidar_origin(0) << ' '
                           << keyframe.lidar_origin(1) << ' ' << keyframe.yaw << '\n';
            }
        }
        trajectory.close();

        std::ofstream raw_trajectory(map_subdir + "/trajectory_raw.csv", std::ios::out | std::ios::trunc);
        if (!raw_trajectory.is_open())
        {
            keyframe_writer_error_ = "cannot create trajectory_raw.csv";
            return false;
        }
        raw_trajectory << "index,timestamp,world_x,world_y,world_z,world_qx,world_qy,world_qz,world_qw,"
                          "lidar_x,lidar_y,lidar_z,lidar_qx,lidar_qy,lidar_qz,lidar_qw\n";
        for (const auto& keyframe : mapping_keyframes_)
        {
            raw_trajectory << keyframe.index << ',' << std::fixed << std::setprecision(6) << keyframe.stamp << ','
                           << keyframe.world_origin(0) << ',' << keyframe.world_origin(1) << ',' << keyframe.world_origin(2) << ','
                           << std::setprecision(7) << keyframe.world_qx << ',' << keyframe.world_qy << ','
                           << keyframe.world_qz << ',' << keyframe.world_qw << ','
                           << std::setprecision(6) << keyframe.lidar_origin(0) << ',' << keyframe.lidar_origin(1) << ','
                           << keyframe.lidar_origin(2) << ','
                           << std::setprecision(7) << keyframe.lidar_qx << ',' << keyframe.lidar_qy << ','
                           << keyframe.lidar_qz << ',' << keyframe.lidar_qw << '\n';
        }
        raw_trajectory.close();
        std::error_code copy_error;
        if (!global_optimization_applied_)
        {
            std::filesystem::copy_file(map_subdir + "/trajectory_raw.csv", map_subdir + "/trajectory_optimized.csv",
                std::filesystem::copy_options::overwrite_existing, copy_error);
        }
        if (copy_error)
        {
            keyframe_writer_error_ = "cannot create trajectory_optimized.csv";
            return false;
        }
        const auto map_pcd = std::filesystem::path(map_subdir) / "map.pcd";
        if (std::filesystem::exists(map_pcd)
            && !std::filesystem::exists(std::filesystem::path(map_subdir) / "map_raw.pcd"))
        {
            std::filesystem::copy_file(map_pcd, std::filesystem::path(map_subdir) / "map_raw.pcd",
                std::filesystem::copy_options::overwrite_existing, copy_error);
        }

        if (gnss_origin_initialized_)
        {
            std::ofstream meta(map_subdir + "/gnss_origin.yaml", std::ios::out | std::ios::trunc);
            if (!meta.is_open())
            {
                keyframe_writer_error_ = "cannot create gnss_origin.yaml";
                return false;
            }
            meta << "rtk_enabled: " << (gnss_alignment_locked_ ? "true" : "false") << "\n";
            meta << "datum: CGCS2000\n";
            meta << std::fixed << std::setprecision(10);
            meta << "origin_latitude: " << gnss_origin_lat_ << "\n";
            meta << "origin_longitude: " << gnss_origin_lon_ << "\n";
            meta << std::setprecision(4);
            meta << "origin_altitude: " << gnss_origin_alt_ << "\n";
            meta << "alignment_locked: " << (gnss_alignment_locked_ ? 1 : 0) << "\n";
            meta << "alignment_source: " << (gnss_alignment_source_.empty() ? "none" : gnss_alignment_source_) << "\n";
            meta << "enu_to_map_yaw: " << gnss_enu_to_map_yaw_ << "\n";
            meta << "alignment_rms: " << (std::isfinite(gnss_alignment_rms_) ? gnss_alignment_rms_ : -1.0) << "\n";
            meta << "alignment_samples: " << gnss_alignment_samples_.size() << "\n";
            meta << "map_offset_x: " << gnss_map_offset_(0) << "\n";
            meta << "map_offset_y: " << gnss_map_offset_(1) << "\n";
            meta << "map_offset_z: " << gnss_map_offset_(2) << "\n";
            meta << "map_offset:\n";
            meta << "  x: " << gnss_map_offset_(0) << "\n";
            meta << "  y: " << gnss_map_offset_(1) << "\n";
            meta << "  z: " << gnss_map_offset_(2) << "\n";
            meta << "gps_link:\n";
            meta << "  x: " << gnss_lever_arm_base_(0) << "\n";
            meta << "  y: " << gnss_lever_arm_base_(1) << "\n";
            meta << "  z: " << gnss_lever_arm_base_(2) << "\n";
            meta << "gnss_corrections: " << gnss_correction_count_ << "\n";
        }
        return writeMapManifest(map_subdir);
    }

    bool MappingAlg::writeImuPreintegrationFile(const MappingKeyframe& keyframe) const
    {
        if (keyframe.preintegration_file.empty())
            return true;
        std::filesystem::create_directories(std::filesystem::path(keyframe.preintegration_file).parent_path());
        std::ofstream output(keyframe.preintegration_file, std::ios::out | std::ios::trunc);
        if (!output.is_open())
            return false;
        const auto& preint = keyframe.preint;
        output << std::fixed;
        output << "{\n"
               << "  \"schema_version\": 1,\n"
               << "  \"keyframe_index\": " << keyframe.index << ",\n"
               << "  \"start_timestamp\": " << std::setprecision(6) << preint.start_timestamp << ",\n"
               << "  \"end_timestamp\": " << preint.end_timestamp << ",\n"
               << "  \"delta_t\": " << preint.delta_t << ",\n"
               << "  \"delta_rotation_xyzw\": ["
               << std::setprecision(7) << preint.delta_rotation_xyzw[0] << ", "
               << preint.delta_rotation_xyzw[1] << ", " << preint.delta_rotation_xyzw[2] << ", "
               << preint.delta_rotation_xyzw[3] << "],\n"
               << "  \"delta_velocity\": [" << std::setprecision(6) << preint.delta_velocity(0) << ", "
               << preint.delta_velocity(1) << ", " << preint.delta_velocity(2) << "],\n"
               << "  \"delta_position\": [" << preint.delta_position(0) << ", "
               << preint.delta_position(1) << ", " << preint.delta_position(2) << "],\n"
               << "  \"linearized_accel_bias\": [" << preint.linearized_accel_bias(0) << ", "
               << preint.linearized_accel_bias(1) << ", " << preint.linearized_accel_bias(2) << "],\n"
               << "  \"linearized_gyro_bias\": [" << preint.linearized_gyro_bias(0) << ", "
               << preint.linearized_gyro_bias(1) << ", " << preint.linearized_gyro_bias(2) << "],\n"
               << "  \"covariance_15x15\": [";
        for (std::size_t index = 0; index < preint.covariance.size(); ++index)
        {
            if (index)
                output << ", ";
            output << std::setprecision(8) << preint.covariance[index];
        }
        output << "],\n"
               << "  \"imu_sample_count\": " << preint.imu_sample_count << ",\n"
               << "  \"error_status\": \"" << jsonEscape(preint.error_status) << "\",\n"
               << "  \"state_order\": \"dtheta,dv,dp,dba,dbg\"\n"
               << "}\n";
        return true;
    }

    bool MappingAlg::writeMapManifest(const std::string& map_subdir)
    {
        const bool rtk_fixed = gnss_origin_initialized_ && gnss_alignment_locked_;
        std::size_t preintegration_count = 0;
        std::error_code filesystem_error;
        const auto preint_dir = std::filesystem::path(map_subdir) / "imu_preintegration";
        if (std::filesystem::is_directory(preint_dir, filesystem_error))
        {
            for (const auto& entry : std::filesystem::directory_iterator(preint_dir, filesystem_error))
            {
                if (!filesystem_error && entry.is_regular_file() && entry.path().extension() == ".json")
                    ++preintegration_count;
            }
        }
        std::ofstream output(map_subdir + "/map_manifest.json", std::ios::out | std::ios::trunc);
        if (!output.is_open())
        {
            keyframe_writer_error_ = "cannot create map_manifest.json";
            return false;
        }
        output << "{\n"
               << "  \"schema_version\": 2,\n"
               << "  \"coordinate_mode\": \"" << (rtk_fixed ? "rtk_fixed" : "local_only") << "\",\n"
               << "  \"scene_scope\": \"" << (rtk_fixed ? "" : "indoor") << "\",\n"
               << "  \"localization_mode\": \"" << (rtk_fixed ? "rtk_ndt" : "ndt") << "\",\n"
               << "  \"origin_status\": \"" << (rtk_fixed ? "fixed" : "local_only") << "\",\n"
               << "  \"rtk_origin_required\": " << (rtk_fixed ? "true" : "false") << ",\n"
               << "  \"completeness\": \"complete\",\n"
               << "  \"frame_id\": \"map\",\n"
               << "  \"quaternion_order\": \"xyzw\",\n"
               << "  \"keyframe_count\": " << mapping_keyframes_.size() << ",\n"
               << "  \"point_cloud_count\": " << mapping_keyframes_.size() << ",\n"
               << "  \"preintegration_count\": " << preintegration_count << ",\n"
               << "  \"scan_context_count\": " << scan_context_count_ << ",\n"
               << "  \"loop_closure_count\": " << global_factor_graph_result_.loop_closure_factor_count << ",\n"
               << "  \"global_factor_count\": " << global_factor_graph_result_.factor_count << ",\n"
               << "  \"trajectory_source\": \"" << (global_optimization_applied_ ? "gtsam_global_optimized" : "raw") << "\",\n"
               << "  \"loop_status\": \"" << loop_status_ << "\"\n"
               << "}\n";
        return true;
    }

    void MappingAlg::accumulateImuPreintegration(double timestamp, const Vec3d& acc, const Vec3d& gyro)
    {
        std::lock_guard<std::mutex> lock(imu_preint_mutex_);
        if (timestamp < imu_preint_last_t_ && imu_preint_has_sample_)
        {
            imu_preint_error_ = "time_rollback";
            imu_preint_has_sample_ = false;
            imu_preint_samples_ = 0;
            imu_preint_dR_ = Eye3d;
            imu_preint_dv_ = Zero3d;
            imu_preint_dp_ = Zero3d;
            imu_preint_cov_.setZero();
            imu_preint_last_t_ = timestamp;
            imu_preint_last_acc_ = acc;
            imu_preint_last_gyro_ = gyro;
            imu_preint_has_sample_ = true;
            return;
        }
        if (!imu_preint_has_sample_)
        {
            imu_preint_start_ = imu_preint_start_ > 0.0 ? imu_preint_start_ : timestamp;
            imu_preint_last_t_ = timestamp;
            imu_preint_last_acc_ = acc;
            imu_preint_last_gyro_ = gyro;
            imu_preint_has_sample_ = true;
            imu_preint_ba_ = Vec3d(state_point.ba(0), state_point.ba(1), state_point.ba(2));
            imu_preint_bg_ = Vec3d(state_point.bg(0), state_point.bg(1), state_point.bg(2));
            return;
        }
        const double dt = timestamp - imu_preint_last_t_;
        if (dt <= 0.0)
            return;
        if (dt > 0.25)
            imu_preint_error_ = imu_preint_error_.empty() ? "imu_gap" : imu_preint_error_;
        const Vec3d unbiased_acc = imu_preint_last_acc_ - imu_preint_ba_;
        const Vec3d unbiased_gyro = imu_preint_last_gyro_ - imu_preint_bg_;
        imu_preint_dp_ += imu_preint_dv_ * dt + 0.5 * imu_preint_dR_ * unbiased_acc * dt * dt;
        imu_preint_dv_ += imu_preint_dR_ * unbiased_acc * dt;
        imu_preint_dR_ = imu_preint_dR_ * Exp(unbiased_gyro, dt);
        const double acc_var = acc_cov * acc_cov * dt;
        const double gyro_var = gyr_cov * gyr_cov * dt;
        const double ba_var = b_acc_cov * b_acc_cov * dt;
        const double bg_var = b_gyr_cov * b_gyr_cov * dt;
        imu_preint_cov_.block<3, 3>(0, 0) += Mat3d::Identity() * gyro_var;
        imu_preint_cov_.block<3, 3>(3, 3) += Mat3d::Identity() * acc_var;
        imu_preint_cov_.block<3, 3>(6, 6) += Mat3d::Identity() * acc_var * dt * dt;
        imu_preint_cov_.block<3, 3>(9, 9) += Mat3d::Identity() * ba_var;
        imu_preint_cov_.block<3, 3>(12, 12) += Mat3d::Identity() * bg_var;
        ++imu_preint_samples_;
        imu_preint_last_t_ = timestamp;
        imu_preint_last_acc_ = acc;
        imu_preint_last_gyro_ = gyro;
    }

    ImuPreintegrationSnapshot MappingAlg::captureImuPreintegration(double keyframe_stamp)
    {
        std::lock_guard<std::mutex> lock(imu_preint_mutex_);
        ImuPreintegrationSnapshot snapshot;
        snapshot.start_timestamp = imu_preint_start_;
        snapshot.end_timestamp = keyframe_stamp;
        snapshot.delta_t = keyframe_stamp - imu_preint_start_;
        const Eigen::Quaterniond delta_q = rotationToXyzw(imu_preint_dR_);
        snapshot.delta_rotation_xyzw[0] = delta_q.x();
        snapshot.delta_rotation_xyzw[1] = delta_q.y();
        snapshot.delta_rotation_xyzw[2] = delta_q.z();
        snapshot.delta_rotation_xyzw[3] = delta_q.w();
        snapshot.delta_velocity = imu_preint_dv_;
        snapshot.delta_position = imu_preint_dp_;
        snapshot.linearized_accel_bias = imu_preint_ba_;
        snapshot.linearized_gyro_bias = imu_preint_bg_;
        snapshot.imu_sample_count = imu_preint_samples_;
        for (int row = 0; row < 15; ++row)
        {
            for (int col = 0; col < 15; ++col)
                snapshot.covariance[static_cast<std::size_t>(row * 15 + col)] = imu_preint_cov_(row, col);
        }
        if (!imu_preint_has_prev_keyframe_)
            snapshot.error_status = "no_previous_keyframe";
        else if (!imu_preint_error_.empty())
            snapshot.error_status = imu_preint_error_;
        else if (imu_preint_samples_ <= 0)
            snapshot.error_status = "empty_interval";
        imu_preint_has_prev_keyframe_ = true;
        imu_preint_start_ = keyframe_stamp;
        imu_preint_dR_ = Eye3d;
        imu_preint_dv_ = Zero3d;
        imu_preint_dp_ = Zero3d;
        imu_preint_cov_.setZero();
        imu_preint_samples_ = 0;
        imu_preint_error_.clear();
        imu_preint_ba_ = Vec3d(state_point.ba(0), state_point.ba(1), state_point.ba(2));
        imu_preint_bg_ = Vec3d(state_point.bg(0), state_point.bg(1), state_point.bg(2));
        return snapshot;
    }

    void MappingAlg::resetImuPreintegration(double start_timestamp)
    {
        std::lock_guard<std::mutex> lock(imu_preint_mutex_);
        imu_preint_has_prev_keyframe_ = false;
        imu_preint_has_sample_ = false;
        imu_preint_start_ = start_timestamp;
        imu_preint_last_t_ = start_timestamp;
        imu_preint_last_acc_ = Zero3d;
        imu_preint_last_gyro_ = Zero3d;
        imu_preint_dR_ = Eye3d;
        imu_preint_dv_ = Zero3d;
        imu_preint_dp_ = Zero3d;
        imu_preint_ba_ = Zero3d;
        imu_preint_bg_ = Zero3d;
        imu_preint_cov_.setZero();
        imu_preint_samples_ = 0;
        imu_preint_error_.clear();
    }

    void MappingAlg::writeSaveProgress(
        const std::string& stage, double progress_percent, const std::string& error) const
    {
        std::lock_guard<std::mutex> progress_lock(progress_file_mutex_);
        if (active_map_subdir_.empty())
            return;
        std::size_t keyframe_count = 0;
        std::size_t queued_keyframes = 0;
        std::size_t written_keyframes = 0;
        std::size_t written_points = 0;
        std::size_t dropped_keyframes = 0;
        double trajectory_m = 0.0;
        MappingKeyframe latest;
        {
            std::lock_guard<std::mutex> lock(keyframe_writer_mutex_);
            keyframe_count = mapping_keyframes_.size();
            queued_keyframes = keyframe_write_queue_.size() + (keyframe_writer_active_ ? 1 : 0);
            written_keyframes = written_keyframes_;
            written_points = written_keyframe_points_;
            dropped_keyframes = dropped_keyframes_;
            trajectory_m = keyframe_trajectory_m_;
            if (!mapping_keyframes_.empty())
                latest = mapping_keyframes_.back();
        }

        const std::string progress_file = active_map_subdir_ + "/save_progress.json";
        const std::string temporary = progress_file + ".tmp";
        std::ofstream output(temporary, std::ios::out | std::ios::trunc);
        if (!output.is_open())
            return;
        output << "{\n"
               << "  \"format\": \"roamerx.streaming-map-progress.v1\",\n"
               << "  \"stage\": \"" << jsonEscape(stage) << "\",\n"
               << "  \"progress_percent\": " << std::fixed << std::setprecision(2)
               << std::clamp(progress_percent, 0.0, 100.0) << ",\n"
               << "  \"keyframe_count\": " << keyframe_count << ",\n"
               << "  \"written_keyframes\": " << written_keyframes << ",\n"
               << "  \"queued_keyframes\": " << queued_keyframes << ",\n"
               << "  \"dropped_keyframes\": " << dropped_keyframes << ",\n"
               << "  \"written_points\": " << written_points << ",\n"
               << "  \"estimated_output_bytes\": " << written_points * sizeof(BinaryPcdPoint) << ",\n"
               << "  \"trajectory_m\": " << std::setprecision(3) << trajectory_m << ",\n"
               << "  \"mapping_capture_enabled\": " << (mapping_capture_enabled_ ? "true" : "false") << ",\n"
               << "  \"slam_pose_ready\": " << (slam_pose_ready_ ? "true" : "false") << ",\n"
               << "  \"rss_bytes\": " << residentSetBytes() << ",\n"
               << "  \"disk_free_bytes\": " << freeDiskBytes(active_map_subdir_) << ",\n"
               << "  \"rtk_quality\": {\"valid\": " << (latest.rtk_valid ? "true" : "false")
               << ", \"status\": " << latest.rtk_status
               << ", \"horizontal_std\": " << latest.rtk_horizontal_std
               << ", \"age_seconds\": " << latest.rtk_age_seconds << "},\n"
               << "  \"rtk_alignment\": {\"locked\": " << (gnss_alignment_locked_ ? "true" : "false")
               << ", \"source\": \"" << jsonEscape(gnss_alignment_source_) << "\""
               << ", \"samples\": " << gnss_alignment_samples_.size()
               << ", \"rms\": " << (std::isfinite(gnss_alignment_rms_) ? gnss_alignment_rms_ : -1.0)
               << ", \"yaw_deg\": " << gnss_enu_to_map_yaw_ * 180.0 / M_PI
               << ", \"fusion_enabled\": " << (use_gnss_fusion_ ? "true" : "false") << "},\n"
               << "  \"slam_health\": {\"state\": \"" << jsonEscape(slam_health_state_)
               << "\", \"imu_initialized\": " << (p_imu->initialization_ready() ? "true" : "false")
               << ", \"slam_pose_ready\": " << (slam_pose_ready_ ? "true" : "false")
               << ", \"imu_samples\": " << p_imu->initialization_samples()
               << ", \"imu_required_samples\": " << p_imu->initialization_required_samples()
               << ", \"no_effective_points_streak\": " << no_effective_points_streak_
               << ", \"pose_anomaly_streak\": " << pose_anomaly_streak_
               << ", \"odom_guard_rejected_updates\": " << odom_guard_rejected_updates_
               << ", \"odom_guard_clamped_z_updates\": " << odom_guard_clamped_z_updates_
               << ", \"frame_delta_m\": " << health_frame_delta_m_
               << ", \"speed_mps\": " << health_speed_mps_
               << ", \"pose_z_m\": " << health_pose_z_m_
               << ", \"warning\": \"" << jsonEscape(slam_health_warning_) << "\"},\n"
               << "  \"updated_at_unix\": " << std::time(nullptr) << ",\n"
               << "  \"recoverable\": "
               << (written_keyframes > 0 && stage != "completed" && !slam_diverged_ ? "true" : "false") << ",\n"
               << "  \"error_code\": \"" << jsonEscape(slam_health_error_code_) << "\",\n"
               << "  \"error\": \"" << jsonEscape(error) << "\"\n"
               << "}\n";
        output.close();
        std::error_code rename_error;
        std::filesystem::rename(temporary, progress_file, rename_error);
        if (rename_error)
            std::filesystem::remove(temporary);
    }

    bool MappingAlg::finish()
    {
        if (map_export_completed_)
            return true;
        std::string validation_error;
        if (active_map_subdir_.empty() || !validateMappingSession(validation_error))
        {
            keyframe_writer_error_ = validation_error.empty() ? "mapping session is invalid" : validation_error;
            if (slam_health_error_code_.empty())
                slam_health_error_code_ = "MAP_SANITY_CHECK_FAILED";
            writeSaveProgress("failed", 0.0, keyframe_writer_error_);
            RCLCPP_ERROR(get_logger(), "Map export rejected: %s", keyframe_writer_error_.c_str());
            return false;
        }

        writeSaveProgress("flushing_keyframes", 2.0);
        if (!flushKeyframeWriter())
        {
            writeSaveProgress("failed", 2.0, keyframe_writer_error_);
            return false;
        }
        stopKeyframeWriter(true);
        writeSaveProgress("global_optimization", 8.0);
        if (!optimizeHistoricalTrajectory(active_map_subdir_))
        {
            // Keep the map export recoverable when an external loop file or graph
            // is invalid. Raw keyframes remain available for a later batch retry.
            RCLCPP_WARN(get_logger(), "Global optimization unavailable; exporting raw trajectory: %s",
                keyframe_writer_error_.c_str());
            optimized_global_poses_.clear();
            global_pose_covariances_.clear();
            global_optimization_applied_ = false;
        }
        writeSaveProgress("filtering", 12.0);

        std::size_t written_points = 0;
        if (!streamMapFromKeyframes(active_map_subdir_, written_points))
        {
            writeSaveProgress("failed", 65.0, keyframe_writer_error_);
            return false;
        }

        writeSaveProgress("building_grid", 68.0);
        std::string grid_error;
        auto grid_options = pcd2pgm_options_;
        if (!mapping_keyframes_.empty() && pcd2pgm_projection_padding_m_ > 0.0)
        {
            grid_options.use_xy_bounds = true;
            const auto projected_lidar_position = [this](std::size_t index) -> Vec3d {
                if (global_optimization_applied_ && index < optimized_global_poses_.size())
                    return optimizedLidarPose(mapping_keyframes_[index], optimized_global_poses_[index]).translation().eval();
                return mapping_keyframes_[index].lidar_origin;
            };
            const auto first_position = projected_lidar_position(0);
            grid_options.x_min = grid_options.x_max = first_position.x();
            grid_options.y_min = grid_options.y_max = first_position.y();
            for (std::size_t index = 0; index < mapping_keyframes_.size(); ++index)
            {
                const auto position = projected_lidar_position(index);
                grid_options.x_min = std::min(grid_options.x_min, position.x());
                grid_options.x_max = std::max(grid_options.x_max, position.x());
                grid_options.y_min = std::min(grid_options.y_min, position.y());
                grid_options.y_max = std::max(grid_options.y_max, position.y());
            }
            grid_options.x_min -= pcd2pgm_projection_padding_m_;
            grid_options.x_max += pcd2pgm_projection_padding_m_;
            grid_options.y_min -= pcd2pgm_projection_padding_m_;
            grid_options.y_max += pcd2pgm_projection_padding_m_;
        }
        Pcd2Grid grid_builder(grid_options);
        const bool grid_saved = grid_builder.runFromBinaryPcd(
            active_map_subdir_ + "/.map_grid_relative.pcd",
            active_map_subdir_ + "/map",
            [this](double progress) {
                writeSaveProgress("building_grid", 68.0 + 24.0 * progress);
            },
            &grid_error);
        std::error_code grid_cleanup_error;
        std::filesystem::remove(active_map_subdir_ + "/.map_grid_relative.pcd", grid_cleanup_error);
        if (!grid_saved)
        {
            keyframe_writer_error_ = grid_error;
            writeSaveProgress("failed", 92.0, grid_error);
            return false;
        }

        writeSaveProgress("writing_metadata", 95.0);
        if (!writeTrajectoryAndGnssMetadata(active_map_subdir_))
        {
            writeSaveProgress("failed", 95.0, keyframe_writer_error_);
            return false;
        }

        written_keyframe_points_ = written_points;
        map_export_completed_ = true;
        writeSaveProgress("completed", 100.0);
        RCLCPP_INFO(get_logger(), "Save Map Success to %s (keyframes=%zu, points=%zu)",
            active_map_subdir_.c_str(), mapping_keyframes_.size(), written_points);
        return true;
    }
}  // namespace robot::slam
