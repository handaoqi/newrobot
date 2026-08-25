/**
 * @file mapping_alg.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once
#include "common/state_mode.h"
#include "global_factor_graph.h"
#include "ikd_tree/ikd_tree.h"
#include "pcd2grid.h"
#include "process/imu_process.h"
#include "process/lidar_process.h"
#include "so3_math.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <functional>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <mtk_iekf/esekfom/esekfom.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <robots_dog_msgs/msg/uni_rtk_pvh.hpp>
#include <robots_dog_msgs/srv/map_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
namespace robot::slam
{
    enum class HealthDecision
    {
        ACCEPT,
        REJECT_FRAME,
        SAFE_HOLD,
        FATAL,
    };

    struct DynamicFilterVoxelKey
    {
        int x;
        int y;
        int z;

        bool operator==(const DynamicFilterVoxelKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct DynamicFilterVoxelKeyHash
    {
        std::size_t operator()(const DynamicFilterVoxelKey& key) const
        {
            std::size_t h = 1469598103934665603ULL;
            auto mix = [&h](int value) {
                h ^= static_cast<std::size_t>(value) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            };
            mix(key.x);
            mix(key.y);
            mix(key.z);
            return h;
        }
    };

    struct ImuPreintegrationMeasurement
    {
        double delta_t = 0.0;
        Vec3d  acceleration = Zero3d;
        Vec3d  angular_velocity = Zero3d;
    };

    struct ImuPreintegrationSnapshot
    {
        double      start_timestamp = 0.0;
        double      end_timestamp = 0.0;
        double      delta_t = 0.0;
        double      delta_rotation_xyzw[4] = { 0.0, 0.0, 0.0, 1.0 };
        Vec3d       delta_velocity = Zero3d;
        Vec3d       delta_position = Zero3d;
        Vec3d       linearized_accel_bias = Zero3d;
        Vec3d       linearized_gyro_bias = Zero3d;
        std::array<double, 225> covariance {};
        int         imu_sample_count = 0;
        std::string error_status;
        std::vector<ImuPreintegrationMeasurement> measurements;
    };

    struct MappingKeyframe
    {
        std::size_t index = 0;
        double      stamp = 0.0;
        Vec3d       world_origin = Zero3d;
        double      world_qx = 0.0;
        double      world_qy = 0.0;
        double      world_qz = 0.0;
        double      world_qw = 1.0;
        Vec3d       lidar_origin = Zero3d;
        double      lidar_qx = 0.0;
        double      lidar_qy = 0.0;
        double      lidar_qz = 0.0;
        double      lidar_qw = 1.0;
        double      yaw = 0.0;
        Vec3d       velocity = Zero3d;
        Vec3d       accel_bias = Zero3d;
        Vec3d       gyro_bias = Zero3d;
        Vec3d       gravity = Vec3d(0.0, 0.0, -G_m_s2);
        std::string file_path;
        std::string preintegration_file;
        std::size_t point_count = 0;
        std::size_t scan_context_index = 0;
        bool        rtk_valid = false;
        int         rtk_status = -1;
        double      rtk_latitude = 0.0;
        double      rtk_longitude = 0.0;
        double      rtk_altitude = 0.0;
        double      rtk_horizontal_std = 0.0;
        double      rtk_age_seconds = 0.0;
        bool        rtk_heading_valid = false;
        double      rtk_heading_deg = 0.0;
        double      rtk_heading_std_deg = 0.0;
        double      rtk_heading_age_seconds = 0.0;
        bool        enu_valid = false;
        Vec3d       enu_position = Zero3d;
        double      enu_yaw = 0.0;
        double      enu_stamp = 0.0;
        double      enu_age_seconds = 0.0;
        double      enu_covariance_x = 0.0;
        double      enu_covariance_y = 0.0;
        double      enu_covariance_z = 0.0;
        std::string enu_origin_session_id;
        std::string enu_origin_sha256;
        ImuPreintegrationSnapshot preint;
    };

    struct PendingKeyframe
    {
        MappingKeyframe metadata;
        CloudPtr        cloud_world;
    };

    struct GnssAlignmentSample
    {
        Eigen::Vector2d enu = Eigen::Vector2d::Zero();
        Eigen::Vector2d map = Eigen::Vector2d::Zero();
        double stamp = 0.0;
    };

    class MappingAlg : public rclcpp::Node
    {
    public:
        MappingAlg(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~MappingAlg();

        void run();

        void reset();

    private:
        double get_time_sec(const builtin_interfaces::msg::Time& time);

        rclcpp::Time get_ros_time(double timestamp);

        void init();

        void pointsBody2World(PointType const* const pi, PointType* const po);

        void pointsBody2Imu(PointType const* const pi, PointType* const po);

        void points_cache_collect();

        void lasermap_fov_segment();

        void lidarCallBack(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

        void imuCallBack(const sensor_msgs::msg::Imu::UniquePtr msg_in);

        void gnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

        void enuOdometryCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);

        void rtkPvhCallBack(const robots_dog_msgs::msg::UniRtkPvh::SharedPtr msg);

        void odomGuardCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);

        bool applyOdomGuardPrediction(double lidar_time);

        bool acceptOdomGuardCorrection(double lidar_time, const state_ikfom& prediction);

        bool syncData(MeasureGroup& meas);

        void map_incremental();

        void pubWorldPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull);

        void recordKeyframe(const CloudPtr& cloud_world);

        bool initializeKeyframeSession();

        bool recoverLatestKeyframeSession();

        void startKeyframeWriter();

        void stopKeyframeWriter(bool drain);

        bool flushKeyframeWriter(double timeout_seconds = 0.0);

        void keyframeWriterLoop();

        bool streamMapFromKeyframes(const std::string& map_subdir, std::size_t& written_points);

        bool writeTrajectoryAndGnssMetadata(const std::string& map_subdir);

        bool optimizeHistoricalTrajectory(const std::string& map_subdir);

        bool loadLoopClosures(const std::string& map_subdir,
            std::vector<GlobalGraphLoopClosure>& loop_closures);

        bool writeGlobalOptimizationOutputs(const std::string& map_subdir,
            const std::vector<GlobalGraphLoopClosure>& loop_closures);

        bool writeImuPreintegrationFile(const MappingKeyframe& keyframe) const;

        bool readImuPreintegrationFile(MappingKeyframe& keyframe) const;

        bool writeMapManifest(const std::string& map_subdir);

        void accumulateImuPreintegration(double timestamp, const Vec3d& acc, const Vec3d& gyro);

        ImuPreintegrationSnapshot captureImuPreintegration(double keyframe_stamp);

        void resetImuPreintegration(double start_timestamp);

        bool loadLockedGnssOrigin();

        void writeSaveProgress(const std::string& stage, double progress_percent, const std::string& error = "") const;

        void pubBodyPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);

        void pubMapPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);

        void stateCallBack(
            robots_dog_msgs::srv::MapState::Request::SharedPtr request, robots_dog_msgs::srv::MapState::Response::SharedPtr response);

        void startMappingCallBack(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void saveMapCallBack(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void globalOptimizeCallBack(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
            std::unique_ptr<tf2_ros::TransformBroadcaster>&                               tf_br);

        void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);

        void map_publish_callback();

        void collectGnssAlignment(double lidar_time);

        bool estimateGnssAlignment();

        bool lockGnssAlignmentFromHeading(const nav_msgs::msg::Odometry& enu_odometry);

        void tryLockGnssAlignmentOnCaptureStart();

        bool gnssHeadingIsValid(double& heading_deg, double& heading_std_deg, double& age_s);

        bool enuToMap(const nav_msgs::msg::Odometry& msg, Vec3d& map_pos) const;

        HealthDecision updateSlamHealth(double lidar_time);

        void markSlamDiverged(const std::string& reason);

        void enterSafeHold(const std::string& reason, const std::string& trigger_type, HealthDecision decision);

        void writeDivergenceEvent(const std::string& reason, const std::string& trigger_type);

        bool validateMappingSession(std::string& reason) const;

        void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

        template <typename T>
        void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi, Eigen::Matrix<T, 3, 1>& po)
        {
            Vec3d p_body(pi[0], pi[1], pi[2]);
            Vec3d p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

            po[0] = p_global(0);
            po[1] = p_global(1);
            po[2] = p_global(2);
        }

        template <typename T>
        void set_posestamp(T& out)
        {
            out.pose.position.x    = state_point.pos(0);
            out.pose.position.y    = state_point.pos(1);
            out.pose.position.z    = state_point.pos(2);
            out.pose.orientation.x = geoQuat.x;
            out.pose.orientation.y = geoQuat.y;
            out.pose.orientation.z = geoQuat.z;
            out.pose.orientation.w = geoQuat.w;
        }

        inline double QuaternionToYaw(double x, double y, double z, double w)
        {
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }


    private:
        bool extrinsic_est_en = true, path_en = true;

        float       res_last[100000]       = { 0.0 };
        float       DET_RANGE              = 300.0f;
        const float MOV_THRESHOLD          = 1.5f;
        double      time_diff_lidar_to_imu = 0.0;

        std::mutex              mtx_buffer;
        std::condition_variable sig_buffer;
        std::string             root_dir_ = ROOT_DIR;
        std::string             lid_topic, imu_topic, gnss_topic, enu_odom_topic, rtk_pvh_topic, odom_guard_topic;
        std::string             data_path_;

        double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
        double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
        double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
        double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
        int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
        int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
        bool   point_selected_surf[100000] = { 0 };
        bool   lidar_pushed, flg_first_scan = true, flg_EKF_inited;
        bool   pub_world_points_flag_ = false, pub_body_points_flag_ = false;
        bool   is_first_lidar = true;

        Pcd2GridOptions           pcd2pgm_options_;
        double                    pcd2pgm_projection_padding_m_ = 25.0;
        std::shared_ptr<Pcd2Grid> pcd2grid_ptr_;

        std::vector<vector<int>>  pointSearchInd_surf;
        std::vector<BoxPointType> cub_needrm;
        std::vector<PointVector>  Nearest_Points;
        std::vector<double>       extrinT;
        std::vector<double>       extrinR;
        std::deque<double>        time_buffer;
        std::deque<CloudPtr>      lidar_buffer;
        std::deque<ImuMessagePtr> imu_buffer;

        std::mutex                         gnss_mutex_;
        sensor_msgs::msg::NavSatFix        latest_gnss_;
        bool                               has_gnss_ = false;
        std::mutex                         enu_mutex_;
        nav_msgs::msg::Odometry            latest_enu_odometry_;
        bool                               has_enu_odometry_ = false;
        std::mutex                         gnss_heading_mutex_;
        bool                               has_gnss_heading_ = false;
        double                             latest_gnss_heading_deg_ = 0.0;
        double                             latest_gnss_heading_std_deg_ = 0.0;
        double                             latest_gnss_heading_baseline_m_ = 0.0;
        int                                latest_gnss_heading_status_ = -1;
        int                                latest_gnss_heading_type_ = 0;
        rclcpp::Time                       latest_gnss_heading_receive_time_ { 0, 0, RCL_ROS_TIME };
        bool                               gnss_use_heading_ = true;
        double                             gnss_heading_offset_rad_ = 0.0;
        double                             gnss_heading_min_baseline_m_ = 0.20;
        double                             gnss_heading_max_std_deg_ = 5.0;
        double                             gnss_heading_max_age_s_ = 1.5;
        bool                               gnss_origin_initialized_ = false;
        double                             gnss_origin_lat_ = 0.0;
        double                             gnss_origin_lon_ = 0.0;
        double                             gnss_origin_alt_ = 0.0;
        Vec3d                              gnss_map_offset_ = Zero3d;
        Vec3d                              gnss_lever_arm_base_ = Zero3d;
        bool                               use_gnss_fusion_ = false;
        bool                               gnss_fusion_config_enabled_ = false;
        bool                               use_gps_config_enabled_ = true;
        double                             gnss_max_age_ = 1.5;
        double                             gnss_max_horizontal_std_ = 1.5;
        int                                gnss_min_status_ = 0;
        int                                gnss_correction_count_ = 0;
        std::deque<GnssAlignmentSample>    gnss_alignment_samples_;
        bool                               gnss_alignment_locked_ = false;
        std::string                        gnss_alignment_source_;
        std::string                        gnss_origin_file_;
        std::string                        gnss_origin_session_id_;
        std::string                        gnss_origin_sha256_;
        bool                               gnss_origin_prelocked_ = false;
        double                             gnss_enu_to_map_yaw_ = 0.0;
        Eigen::Vector2d                    gnss_enu_to_map_translation_ = Eigen::Vector2d::Zero();
        double                             gnss_alignment_rms_ = std::numeric_limits<double>::infinity();
        double                             gnss_last_alignment_stamp_ = -1.0;
        double                             gnss_last_candidate_yaw_ = 0.0;
        int                                gnss_alignment_stable_fits_ = 0;
        int                                gnss_alignment_min_samples_ = 20;
        int                                gnss_alignment_required_fits_ = 3;
        double                             gnss_alignment_min_baseline_m_ = 15.0;
        double                             gnss_alignment_max_rms_m_ = 1.5;
        double                             gnss_alignment_max_yaw_change_rad_ = 3.0 * M_PI / 180.0;
        std::size_t                        gnss_alignment_max_samples_ = 300;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_guard_sub_;
        std::mutex                         odom_guard_mutex_;
        nav_msgs::msg::Odometry            latest_odom_guard_;
        bool                               odom_guard_enable_ = false;
        bool                               has_odom_guard_ = false;
        bool                               odom_guard_initialized_ = false;
        Vec3d                              odom_guard_position_ = Zero3d;
        double                             odom_guard_stamp_ = 0.0;
        double                             odom_guard_max_speed_mps_ = 2.0;
        double                             odom_guard_max_vertical_speed_mps_ = 0.10;
        double                             odom_guard_max_lidar_correction_m_ = 0.35;
        double                             odom_guard_max_lidar_z_correction_m_ = 0.03;
        double                             odom_guard_max_lidar_rotation_rad_ = 0.35;
        double                             odom_guard_max_abs_z_from_start_m_ = 10.0;
        double                             odom_guard_initial_z_ = 0.0;
        std::size_t                        odom_guard_rejected_updates_ = 0;
        std::size_t                        odom_guard_clamped_z_updates_ = 0;
        bool                               slam_diverged_ = false;
        std::string                        slam_health_state_ = "initializing";
        std::string                        slam_health_error_code_;
        std::string                        slam_health_error_;
        std::string                        slam_health_warning_;
        int                                no_effective_points_streak_ = 0;
        int                                pose_anomaly_streak_ = 0;
        bool                               has_last_health_pose_ = false;
        Vec3d                              last_health_position_ = Zero3d;
        double                             last_health_yaw_ = 0.0;
        double                             last_health_stamp_ = 0.0;
        Vec3d                              health_prediction_position_ = Zero3d;
        double                             health_prediction_yaw_ = 0.0;
        double                             mapping_started_stamp_ = 0.0;
        double                             health_start_z_m_ = 0.0;
        bool                               has_health_start_z_ = false;
        long long                          last_healthy_keyframe_ = -1;
        std::string                        divergence_trigger_type_;
        double                             health_max_frame_translation_m_ = 1.5;
        double                             health_max_speed_mps_ = 3.0;
        double                             health_max_abs_z_m_ = 5.0;
        double                             health_max_frame_z_m_ = 1.0;
        double                             health_warn_speed_mps_ = 1.8;
        double                             health_warn_abs_z_m_ = 0.5;
        double                             health_frame_delta_m_ = 0.0;
        double                             health_speed_mps_ = 0.0;
        double                             health_pose_z_m_ = 0.0;
        int                                health_pose_guard_frames_ = 3;
        int                                health_no_effective_limit_ = 10;
        bool                               dynamic_filter_enable_ = false;
        double                             dynamic_filter_voxel_size_ = 0.20;
        int                                dynamic_filter_min_scan_observations_ = 1;
        std::size_t                        dynamic_filter_shard_count_ = 64;
        bool                               keyframe_record_enable_ = true;
        bool                               mapping_capture_enabled_ = false;
        bool                               slam_pose_ready_ = false;
        double                             keyframe_min_distance_m_ = 0.8;
        double                             keyframe_min_yaw_rad_ = 0.35;
        double                             keyframe_max_interval_s_ = 2.0;
        double                             keyframe_voxel_size_m_ = 0.25;
        std::size_t                        keyframe_max_queue_size_ = 8;
        GlobalFactorGraphConfig            global_factor_graph_config_;
        std::unique_ptr<GlobalFactorGraph> global_factor_graph_;
        std::vector<gtsam::Pose3>           optimized_global_poses_;
        std::vector<gtsam::Matrix6>         global_pose_covariances_;
        GlobalFactorGraphResult             global_factor_graph_result_;
        bool                               global_optimization_applied_ = false;
        std::size_t                        scan_context_count_ = 0;
        std::string                        loop_status_ = "skipped";
        std::vector<MappingKeyframe>       mapping_keyframes_;
        std::deque<PendingKeyframe>        keyframe_write_queue_;
        mutable std::mutex                 keyframe_writer_mutex_;
        mutable std::mutex                 progress_file_mutex_;
        std::condition_variable            keyframe_writer_cv_;
        std::thread                        keyframe_writer_thread_;
        bool                               keyframe_writer_stop_ = false;
        bool                               keyframe_writer_active_ = false;
        bool                               keyframe_writer_failed_ = false;
        std::string                        keyframe_writer_error_;
        std::size_t                        written_keyframes_ = 0;
        std::size_t                        written_keyframe_points_ = 0;
        std::size_t                        dropped_keyframes_ = 0;
        double                             keyframe_trajectory_m_ = 0.0;
        std::string                        active_map_subdir_;
        bool                               map_export_completed_ = false;
        Vec3d                              last_keyframe_origin_ = Zero3d;
        double                             last_keyframe_yaw_ = 0.0;
        double                             last_keyframe_stamp_ = 0.0;
        bool                               has_last_keyframe_ = false;
        double                             last_mapping_progress_stamp_ = 0.0;
        mutable std::mutex                 imu_preint_mutex_;
        bool                               imu_preint_has_prev_keyframe_ = false;
        bool                               imu_preint_has_sample_ = false;
        double                             imu_preint_start_ = 0.0;
        double                             imu_preint_last_t_ = 0.0;
        Vec3d                              imu_preint_last_acc_ = Zero3d;
        Vec3d                              imu_preint_last_gyro_ = Zero3d;
        Mat3d                              imu_preint_dR_ = Eye3d;
        Vec3d                              imu_preint_dv_ = Zero3d;
        Vec3d                              imu_preint_dp_ = Zero3d;
        Vec3d                              imu_preint_ba_ = Zero3d;
        Vec3d                              imu_preint_bg_ = Zero3d;
        Eigen::Matrix<double, 15, 15>      imu_preint_cov_ = Eigen::Matrix<double, 15, 15>::Zero();
        std::vector<ImuPreintegrationMeasurement> imu_preint_measurements_;
        int                                imu_preint_samples_ = 0;
        std::string                        imu_preint_error_;

        CloudPtr featsFromMap     = CloudPtr(new PointCloudType());
        CloudPtr feats_undistort  = CloudPtr(new PointCloudType());
        CloudPtr feats_down_body  = CloudPtr(new PointCloudType());
        CloudPtr feats_down_world = CloudPtr(new PointCloudType());
        CloudPtr normvec          = CloudPtr(new PointCloudType(100000, 1));
        CloudPtr laserCloudOri    = CloudPtr(new PointCloudType(100000, 1));
        CloudPtr corr_normvect    = CloudPtr(new PointCloudType(100000, 1));
        CloudPtr _featsArray      = CloudPtr(new PointCloudType());

        pcl::VoxelGrid<PointType> downSizeFilterSurf;
        pcl::VoxelGrid<PointType> downSizeFilterMap;

        KD_TREE<PointType> ikdtree;

        Vec3d euler_cur;
        Vec3d position_last   = Zero3d;
        Vec3d Lidar_T_wrt_IMU = Zero3d;
        Mat3d Lidar_R_wrt_IMU = Eye3d;

        MeasureGroup                                 Measures;
        esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
        state_ikfom                                  state_point;
        vect3                                        pos_lid;

    public:
        bool finish();

    private:
        std::shared_ptr<Preprocess> p_pre = std::make_shared<Preprocess>();
        std::shared_ptr<ImuProcess> p_imu = std::make_shared<ImuProcess>();

    private:
        nav_msgs::msg::Path             path;
        nav_msgs::msg::Odometry         odomAftMapped;
        geometry_msgs::msg::Quaternion  geoQuat;
        geometry_msgs::msg::PoseStamped msg_body_pose;

        BoxPointType LocalMap_Points;
        bool         Localmap_Initialized = false;

        double lidar_mean_scantime = 0.0;
        int    scan_num            = 0;

        PointCloudType::Ptr pcl_wait_pub  = PointCloudType::Ptr(new PointCloudType());
        PointCloudType::Ptr pcl_wait_save = PointCloudType::Ptr(new PointCloudType());

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudFull_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudFull_body_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudMap_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pubOdomAftMapped_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pubLocalizationOdom_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pubPath_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pubGlobalOptimizedOdom_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pubGlobalOptimizedPath_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             pubGlobalOptimizationStatus_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             pubDivergenceEvent_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr   sub_gnss_ptr_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_enu_odom_ptr_;
        rclcpp::Subscription<robots_dog_msgs::msg::UniRtkPvh>::SharedPtr sub_rtk_pvh_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_ptr_;

        rclcpp::Service<robots_dog_msgs::srv::MapState>::SharedPtr state_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_optimize_service_;

        std::atomic<SlamState> state_{ SlamState::STABLE };

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr                   map_pub_timer_;

        bool effect_pub_en = false, map_pub_en = false;
    };

}  // namespace robot::slam
