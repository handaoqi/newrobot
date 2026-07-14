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
#include "ikd_tree/ikd_tree.h"
#include "pcd2grid.h"
#include "process/imu_process.h"
#include "process/lidar_process.h"
#include "so3_math.h"

#include <Eigen/Core>
#include <chrono>
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
#include <robots_dog_msgs/srv/map_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
namespace robot::slam
{
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

    struct MappingKeyframe
    {
        double   stamp = 0.0;
        Vec3d    lidar_origin = Zero3d;
        CloudPtr cloud_world = CloudPtr(new PointCloudType());
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

        bool syncData(MeasureGroup& meas);

        void map_incremental();

        void pubWorldPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull);

        void recordKeyframe(const CloudPtr& cloud_world);

        void saveKeyframes(const std::string& map_subdir) const;

        void pubBodyPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);

        void pubMapPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);

        void stateCallBack(
            robots_dog_msgs::srv::MapState::Request::SharedPtr request, robots_dog_msgs::srv::MapState::Response::SharedPtr response);

        void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
            std::unique_ptr<tf2_ros::TransformBroadcaster>&                               tf_br);

        void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);

        void map_publish_callback();

        void applyGnssCorrection(double lidar_time);

        bool gnssToMap(const sensor_msgs::msg::NavSatFix& msg, Vec3d& map_pos);

        Vec3d llaToEnu(double latitude_deg, double longitude_deg, double altitude_m) const;

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
        std::string             lid_topic, imu_topic, gnss_topic;
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
        bool                               gnss_origin_initialized_ = false;
        double                             gnss_origin_lat_ = 0.0;
        double                             gnss_origin_lon_ = 0.0;
        double                             gnss_origin_alt_ = 0.0;
        Vec3d                              gnss_map_offset_ = Zero3d;
        Vec3d                              gnss_lever_arm_base_ = Zero3d;
        bool                               use_gnss_fusion_ = false;
        double                             gnss_fusion_gain_ = 0.03;
        double                             gnss_max_correction_step_ = 0.25;
        double                             gnss_max_residual_ = 8.0;
        double                             gnss_max_age_ = 2.5;
        double                             gnss_max_horizontal_std_ = 2.0;
        int                                gnss_min_status_ = 0;
        bool                               gnss_use_elevation_ = false;
        int                                gnss_correction_count_ = 0;
        bool                               dynamic_filter_enable_ = true;
        double                             dynamic_filter_voxel_size_ = 0.20;
        int                                dynamic_filter_min_scan_observations_ = 3;
        std::unordered_map<DynamicFilterVoxelKey, int, DynamicFilterVoxelKeyHash> dynamic_filter_scan_observations_;
        bool                               keyframe_record_enable_ = true;
        double                             keyframe_min_distance_m_ = 0.8;
        double                             keyframe_min_yaw_rad_ = 0.35;
        double                             keyframe_max_interval_s_ = 2.0;
        double                             keyframe_voxel_size_m_ = 0.25;
        std::vector<MappingKeyframe>       mapping_keyframes_;
        Vec3d                              last_keyframe_origin_ = Zero3d;
        double                             last_keyframe_yaw_ = 0.0;
        double                             last_keyframe_stamp_ = 0.0;
        bool                               has_last_keyframe_ = false;

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
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pubPath_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr   sub_gnss_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_ptr_;

        rclcpp::Service<robots_dog_msgs::srv::MapState>::SharedPtr state_service_;

        std::atomic<SlamState> state_{ SlamState::STABLE };

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr                   map_pub_timer_;

        bool effect_pub_en = false, map_pub_en = false;
    };

}  // namespace robot::slam
