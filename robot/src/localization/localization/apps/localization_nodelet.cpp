// hdl localizaton 
#include <Eigen/Geometry>
#include <mutex>
#include <memory>
#include <iostream>
#include <atomic>
#include <filesystem>
#include <deque>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <localization/pose_estimator.hpp>
#include <localization/static_imu_init.hpp>
#include <localization/mode_state.h>
#include <localization/global_localization.hpp>
#include <localization/scan_context_db.hpp>

#include <localization/msg/scan_matching_status.hpp>
#include <robots_dog_msgs/srv/load_map.hpp>
#include <robots_dog_msgs/srv/localization_state.hpp>
#include <robots_dog_msgs/msg/localization.hpp>
#include <robots_dog_msgs/msg/uni_rtk_pvh.hpp>

using namespace std;

namespace localization {

class HdlLocalizationNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNode(const rclcpp::NodeOptions& options) : Node("localization", options) {
    tf_buffer      = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buffer->setUsingDedicatedThread(true);
    tf_listener    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    robot_odom_frame_id              = declare_parameter<std::string>("robot_odom_frame_id", "map");
    odom_child_frame_id              = declare_parameter<std::string>("odom_child_frame_id", "livox_frame");
    send_tf_transforms               = declare_parameter<bool>("send_tf_transforms", false);
    tf_use_current_time              = declare_parameter<bool>("tf_use_current_time", true);
    tf_future_offset_                = declare_parameter<double>("tf_future_offset", 0.0);
    cool_time_duration               = declare_parameter<double>("cool_time_duration", 0.5);
    reg_method                       = declare_parameter<std::string>("reg_method", "NDT_OMP");
    ndt_neighbor_search_method       = declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    ndt_neighbor_search_radius       = declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    ndt_resolution                   = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_max_fitness_score_ = static_cast<float>(std::max(
      0.001, declare_parameter<double>("ndt_max_fitness_score", 0.50)));
    scan_matching_refine_enable_ = declare_parameter<bool>("scan_matching.refine_enable", true);
    scan_matching_local_map_xy_radius_ = static_cast<float>(std::max(
      3.0, declare_parameter<double>("scan_matching.local_map_xy_radius", 18.0)));
    scan_matching_local_map_z_radius_ = static_cast<float>(std::max(
      1.0, declare_parameter<double>("scan_matching.local_map_z_radius", 4.0)));
    scan_matching_min_local_map_points_ = static_cast<int>(std::max<int64_t>(
      50, declare_parameter<int>("scan_matching.min_local_map_points", 400)));
    scan_matching_vgicp_resolution_ = std::max(
      0.15, declare_parameter<double>("scan_matching.vgicp_resolution", 0.50));
    scan_matching_max_correspondence_distance_ = static_cast<float>(std::max(
      0.20, declare_parameter<double>("scan_matching.max_correspondence_distance", 1.00)));
    scan_matching_max_iterations_ = static_cast<int>(std::max<int64_t>(
      5, declare_parameter<int>("scan_matching.max_iterations", 20)));
    scan_matching_num_threads_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("scan_matching.num_threads", 2)));
    scan_matching_coarse_max_fitness_score_ = static_cast<float>(std::max(
      static_cast<double>(ndt_max_fitness_score_),
      declare_parameter<double>("scan_matching.coarse_max_fitness_score", 2.00)));
    enable_robot_odometry_prediction = declare_parameter<bool>("enable_robot_odometry_prediction", false);
    enable_lidar_odometry_prediction_ = declare_parameter<bool>("lidar_odometry_prediction.enable", false);
    lidar_odom_voxel_size_ = static_cast<float>(std::max(
      0.05, declare_parameter<double>("lidar_odometry_prediction.voxel_size", 0.40)));
    lidar_odom_max_correspondence_distance_ = static_cast<float>(std::max(
      0.05, declare_parameter<double>("lidar_odometry_prediction.max_correspondence_distance", 1.00)));
    lidar_odom_max_fitness_score_ = static_cast<float>(std::max(
      0.001, declare_parameter<double>("lidar_odometry_prediction.max_fitness_score", 0.50)));
    lidar_odom_max_translation_per_scan_ = static_cast<float>(std::max(
      0.05, declare_parameter<double>("lidar_odometry_prediction.max_translation_per_scan", 0.80)));
    lidar_odom_max_rotation_per_scan_rad_ = static_cast<float>(std::max(
      0.01, declare_parameter<double>("lidar_odometry_prediction.max_rotation_per_scan_rad", 0.70)));
    lidar_odom_min_points_ = static_cast<size_t>(std::max<int64_t>(
      20, declare_parameter<int>("lidar_odometry_prediction.min_points", 200)));
    lidar_odom_num_threads_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("lidar_odometry_prediction.num_threads", 2)));
    enable_lio_primary_ = declare_parameter<bool>("lio_primary.enable", false);
    lio_odom_topic_ = declare_parameter<std::string>("lio_primary.topic", "/odom/lio_odom");
    lio_max_age_s_ = std::max(0.05, declare_parameter<double>("lio_primary.max_age", 0.30));
    lio_max_step_m_ = static_cast<float>(std::max(
      0.10, declare_parameter<double>("lio_primary.max_step_m", 1.50)));
    lio_xy_variance_ = static_cast<float>(std::max(
      1e-4, declare_parameter<double>("lio_primary.xy_variance", 0.0025)));
    lio_z_variance_ = static_cast<float>(std::max(
      1e-4, declare_parameter<double>("lio_primary.z_variance", 0.010)));
    lio_orientation_variance_ = static_cast<float>(std::max(
      1e-5, declare_parameter<double>("lio_primary.orientation_variance", 0.0004)));
    lio_drift_xy_m_ = static_cast<float>(std::max(
      0.05, declare_parameter<double>("lio_primary.drift_xy_m", 0.30)));
    lio_drift_yaw_rad_ = static_cast<float>(std::max(
      0.01, declare_parameter<double>("lio_primary.drift_yaw_deg", 5.0) * M_PI / 180.0));
    lio_drift_hysteresis_frames_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("lio_primary.drift_hysteresis_frames", 3)));
    lio_stable_xy_tolerance_m_ = static_cast<float>(std::max(
      0.01, declare_parameter<double>("lio_primary.stable_xy_tolerance_m", 0.10)));
    lio_stable_yaw_tolerance_rad_ = static_cast<float>(std::max(
      0.001, declare_parameter<double>("lio_primary.stable_yaw_tolerance_deg", 2.0) * M_PI / 180.0));
    lio_rearm_xy_m_ = static_cast<float>(std::clamp(
      declare_parameter<double>("lio_primary.rearm_xy_m", 0.20), 0.01,
      static_cast<double>(lio_drift_xy_m_)));
    lio_suppressed_covariance_scale_ = static_cast<float>(std::max(
      100.0, declare_parameter<double>("lio_primary.suppressed_covariance_scale", 100.0)));
    lio_max_correction_jump_m_ = static_cast<float>(std::max(
      0.10, declare_parameter<double>("lio_primary.max_correction_jump_m", 1.50)));
    lio_correct_xy_variance_ = static_cast<float>(std::max(
      1e-4, declare_parameter<double>("lio_primary.correct_xy_variance", 0.010)));
    lio_correct_z_variance_ = static_cast<float>(std::max(
      1e-4, declare_parameter<double>("lio_primary.correct_z_variance", 0.020)));
    lio_correct_orientation_variance_ = static_cast<float>(std::max(
      1e-5, declare_parameter<double>("lio_primary.correct_orientation_variance", 0.001)));

	    use_imu     = declare_parameter<bool>("use_imu", true);
	    if (enable_lio_primary_ && use_imu) {
	      RCLCPP_WARN(get_logger(),
	        "lio_primary is enabled; disabling raw IMU prediction in the localization UKF "
	        "to avoid fusing the Mid-360 IMU twice");
	      use_imu = false;
	    }
	    invert_acc  = declare_parameter<bool>("invert_acc", false);
	    invert_gyro = declare_parameter<bool>("invert_gyro", false);
	    imu_acc_scale_ = declare_parameter<double>("imu_acc_scale", 9.81);
    // imu static init params
    imu_init_time_         = static_cast<float>(declare_parameter<double>("imu_init_time", 3.0));
    imu_init_queue_size_   = declare_parameter<int>("imu_init_queue_size", 600);
    imu_init_max_gyro_var_ = static_cast<float>(declare_parameter<double>("imu_init_max_gyro_var", 0.05));
    imu_init_max_acce_var_ = static_cast<float>(declare_parameter<double>("imu_init_max_acce_var", 0.2));
    // UKF gyro bias tuning. Exposed as parameters so an offline bag replay can sweep
    // them without a rebuild; see docs/IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md.
    gyro_bias_process_noise_ = declare_parameter<double>(
      "gyro_bias_process_noise", localization::PoseEstimator::kDefaultGyroBiasProcessNoise);
    gyro_bias_initial_cov_ = declare_parameter<double>(
      "gyro_bias_initial_cov", localization::PoseEstimator::kDefaultGyroBiasInitialCov);

    std::string imu_topic               = declare_parameter<std::string>("imu_topic", "/livox/imu");
    std::string points_topic            = declare_parameter<std::string>("points_topic", "/livox/lidar");
    std::string odom_topic              = declare_parameter<std::string>("odom_topic", "/odom/localization_odom");
    robot_odom_topic_                   = declare_parameter<std::string>("robot_odom_topic", "/odom/mc_odom");
    lidar_odom_topic_                   = declare_parameter<std::string>("lidar_odometry_prediction.topic", "/odom/lidar_odom");
    std::string aligned_points_topic    = declare_parameter<std::string>("aligned_points_topic", "/aligned_points");
    std::string status_topic            = declare_parameter<std::string>("status_topic", "/status");
    // Absolute NDT/VGICP result. /status carries only the frame-to-frame delta.
    std::string scan_match_pose_topic   = declare_parameter<std::string>("scan_match_pose_topic", "/localization/scan_match_pose");
    std::string localization_info_topic = declare_parameter<std::string>("localization_info_topic", "/localization_info");
    std::string global_map_points_topic = declare_parameter<std::string>("global_map_points_topic", "/global_map_points");
    std::string gnss_topic              = declare_parameter<std::string>("gnss_topic", "/fix");
    std::string rtk_pvh_topic           = declare_parameter<std::string>("gnss_fusion.rtk_pvh_topic", "/rtk_pvh");

    // Load numeric parameters
    imu_data_filter_num_      = declare_parameter<int>("imu_data_filter_num", 5);
    globalmap_voxel_size_     = static_cast<float>(declare_parameter<double>("globalmap_voxel_size", 0.3));
    points_voxel_filter_size_ = static_cast<float>(declare_parameter<double>("points_voxel_filter_size", 0.2));
    
    min_valid_count_           = declare_parameter<int>("min_valid_count", 5);
    buffer_size_               = declare_parameter<int>("buffer_size", 10);
    localization_odom_frame_id = declare_parameter<std::string>("localization_odom_frame_id", "base_link");
    use_gnss_fusion_           = declare_parameter<bool>("gnss_fusion.enable", false);
    gnss_fusion_gain_          = declare_parameter<double>("gnss_fusion.gain", 0.03);
    gnss_max_correction_step_  = declare_parameter<double>("gnss_fusion.max_correction_step", 0.10);
    gnss_max_residual_         = declare_parameter<double>("gnss_fusion.max_residual", 5.0);
    gnss_max_age_              = declare_parameter<double>("gnss_fusion.max_age", 1.5);
    gnss_max_horizontal_std_   = declare_parameter<double>("gnss_fusion.max_horizontal_std", 1.5);
    gnss_min_status_           = declare_parameter<int>("gnss_fusion.min_status", 0);
    gnss_use_elevation_        = declare_parameter<bool>("gnss_fusion.use_elevation", false);
    gnss_auto_recovery_enable_ = declare_parameter<bool>("gnss_fusion.auto_recovery_enable", true);
    gnss_auto_recovery_retry_seconds_ = std::max(1.0,
      declare_parameter<double>("gnss_fusion.auto_recovery_retry_seconds", 5.0));
    gnss_use_heading_ = declare_parameter<bool>("gnss_fusion.use_heading", false);
    gnss_heading_offset_param_rad_ =
      declare_parameter<double>("gnss_fusion.heading_offset_deg", 0.0) * M_PI / 180.0;
    gnss_heading_offset_rad_ = gnss_heading_offset_param_rad_;
    gnss_heading_max_std_deg_ = declare_parameter<double>("gnss_fusion.heading_max_std_deg", 5.0);
    gnss_heading_min_baseline_m_ = declare_parameter<double>("gnss_fusion.heading_min_baseline_m", 0.20);
    gnss_heading_max_age_ = declare_parameter<double>("gnss_fusion.heading_max_age", 1.5);
    gnss_heading_filter_tau_s_ = std::max(0.05,
      declare_parameter<double>("gnss_fusion.heading_filter_tau_s", 0.15));
    source_arbiter_enable_ = declare_parameter<bool>("source_arbiter.enable", true);
    bridge_max_distance_m_ = declare_parameter<double>("source_arbiter.bridge_max_distance_m", 10.0);
    bridge_max_seconds_ = declare_parameter<double>("source_arbiter.bridge_max_seconds", 20.0);
    bridge_max_horizontal_sigma_m_ = declare_parameter<double>("source_arbiter.max_horizontal_sigma_m", 0.8);
    bridge_max_yaw_sigma_rad_ = declare_parameter<double>("source_arbiter.max_yaw_sigma_deg", 15.0) * M_PI / 180.0;
    bridge_translation_variance_per_m_ = declare_parameter<double>("source_arbiter.odom_translation_variance_per_m", 0.0025);
    bridge_max_odom_speed_mps_ = declare_parameter<double>("source_arbiter.max_odom_speed_mps", 1.5);
    bridge_max_odom_yaw_rate_rps_ = declare_parameter<double>("source_arbiter.max_odom_yaw_rate_rps", 2.0);
    absolute_recovery_samples_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.absolute_recovery_samples", 3)));
    moving_ndt_stride_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.moving_ndt_stride", 5)));
    ndt_failure_hysteresis_frames_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.ndt_failure_hysteresis_frames", 3)));
    prefer_fixed_rtk_ = declare_parameter<bool>("source_arbiter.prefer_fixed_rtk", true);
    rtk_primary_promote_samples_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.rtk_primary_promote_samples", 2)));
    rtk_primary_demote_samples_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.rtk_primary_demote_samples", 3)));
    std::vector<double> gnss_lever_arm = declare_parameter<std::vector<double>>("gnss_fusion.lever_arm_base", {-0.05, 0.0, 0.15});
    if (gnss_lever_arm.size() >= 3) {
      gnss_lever_arm_base_ << gnss_lever_arm[0], gnss_lever_arm[1], gnss_lever_arm[2];
      gnss_map_offset_ = gnss_lever_arm_base_;
    }

    // IMU rotation matrix parameters 
    std::vector<double> init_imu_R;
    init_imu_R.reserve(9);
    declare_parameter<std::vector<double>>("init_R", std::vector<double>{});
    get_parameter("init_R", init_imu_R);
    
    // Gravity transform matrix parameters 
    std::vector<double> init_T;
    init_T.reserve(16);
    declare_parameter<std::vector<double>>("init_T", std::vector<double>{});
    get_parameter("init_T", init_T);
    
    // Initial pose initialization parameters 
    declare_parameter<int>("init_match_count_threshold", 5);
    get_parameter("init_match_count_threshold", init_match_count_threshold_);
    declare_parameter<float>("init_match_score_threshold", 0.15);
    get_parameter("init_match_score_threshold", init_match_score_threshold_);
    
    // Global localization parameters
    declare_parameter<bool>("use_global_localization_init", true);
    get_parameter("use_global_localization_init", use_global_localization_init_);
    
    declare_parameter<float>("init_pose_change_threshold", 0.01);
    get_parameter("init_pose_change_threshold", init_pose_change_threshold_);
    
    declare_parameter<float>("init_quat_change_threshold", 0.01);
    get_parameter("init_quat_change_threshold", init_quat_change_threshold_);
    
    declare_parameter<float>("global_localization_timeout", 10.0);
    get_parameter("global_localization_timeout", global_localization_timeout_);

    declare_parameter<int>("runtime_relocalization_failure_threshold", 10);
    get_parameter("runtime_relocalization_failure_threshold", runtime_relocalization_failure_threshold_);
    runtime_relocalization_failure_threshold_ =
      std::max(1, runtime_relocalization_failure_threshold_);
    declare_parameter<double>("runtime_relocalization_retry_seconds", 5.0);
    get_parameter("runtime_relocalization_retry_seconds", runtime_relocalization_retry_seconds_);
    runtime_relocalization_retry_seconds_ =
      std::max(1.0, runtime_relocalization_retry_seconds_);

    // Scan context relocalization. Default off: this package carries no unit tests, so
    // the new retrieval path only turns on where replay or an on-robot run has backed it.
    declare_parameter<bool>("relocalization.use_scan_context", false);
    get_parameter("relocalization.use_scan_context", use_scan_context_);
    declare_parameter<int>("relocalization.top_k", 5);
    get_parameter("relocalization.top_k", scan_context_top_k_);
    scan_context_top_k_ = std::max(1, scan_context_top_k_);
    declare_parameter<int>("relocalization.max_seeds_per_attempt", 1);
    get_parameter("relocalization.max_seeds_per_attempt", scan_context_max_seeds_);
    scan_context_max_seeds_ = std::max(1, scan_context_max_seeds_);
    // Offline leave-one-out over 103 recorded maps put the descriptor distance of correct
    // retrievals (p90 0.33) on top of that of wrong ones (median 0.26), so a cutoff here
    // discards good candidates without excluding bad ones. 0 disables it and leaves the
    // ICP verification as the only gate, which is the one that actually discriminates.
    declare_parameter<double>("relocalization.max_descriptor_distance", 0.0);
    get_parameter("relocalization.max_descriptor_distance", scan_context_max_distance_);

    static_imu_init_.SetParam(imu_init_time_, imu_init_queue_size_, imu_init_max_gyro_var_, imu_init_max_acce_var_);
    
    if (!init_imu_R.empty()) {
      init_rotation_matrix_ << init_imu_R[0], init_imu_R[1], init_imu_R[2], 
                               init_imu_R[3], init_imu_R[4], init_imu_R[5], 
                               init_imu_R[6], init_imu_R[7], init_imu_R[8];
      RCLCPP_INFO(get_logger(), "IMU rotation matrix loaded from config");
    }
    if (!init_T.empty()) {
      gravity_transform_ << init_T[0], init_T[1], init_T[2], init_T[3], 
                           init_T[4], init_T[5], init_T[6], init_T[7], 
                           init_T[8], init_T[9], init_T[10], init_T[11], 
                           init_T[12], init_T[13], init_T[14], init_T[15];
      RCLCPP_INFO(get_logger(), "Gravity transform matrix loaded from config");
    }
    // Log loaded parameters for verification
    RCLCPP_INFO(get_logger(),
                "Loaded parameters:\n"
                "  robot_odom_frame_id: %s\n"
                "  odom_child_frame_id: %s\n"
                "  localization_odom_frame_id: %s\n"
	                "  use_imu: %s\n"
	                "  invert_acc: %s\n"
	                "  invert_gyro: %s\n"
	                "  imu_acc_scale: %.3f\n"
	                "  imu_topic: %s\n"
                "  points_topic: %s\n"
                "  odom_topic: %s\n"
                "  aligned_points_topic: %s\n"
                "  status_topic: %s\n"
                "  localization_info_topic: %s\n"
                "  global_map_points_topic: %s\n"
                "  send_tf_transforms: %s\n"
                "  enable_robot_odometry_prediction: %s\n"
                "  reg_method: %s\n"
                "  ndt_neighbor_search_method: %s\n"
                "  ndt_neighbor_search_radius: %.3f\n"
                "  ndt_resolution: %.3f\n"
                "  scan_matching_refine: %s\n"
                "  imu_data_filter_num: %d\n"
                "  globalmap_voxel_size: %.3f\n"
                "  points_voxel_filter_size: %.3f\n"
                "  min_valid_count: %d\n"
                "  buffer_size: %d\n"
                "  imu_init_time: %.1f\n"
                "  imu_init_queue_size: %d\n"
                "  imu_init_max_gyro_var: %.3f\n"
                "  imu_init_max_acce_var: %.3f",
                robot_odom_frame_id.c_str(),
                odom_child_frame_id.c_str(),
                localization_odom_frame_id.c_str(),
	                use_imu ? "true" : "false",
	                invert_acc ? "true" : "false",
	                invert_gyro ? "true" : "false",
	                imu_acc_scale_,
	                imu_topic.c_str(),
                points_topic.c_str(),
                odom_topic.c_str(),
                aligned_points_topic.c_str(),
                status_topic.c_str(),
                localization_info_topic.c_str(),
                global_map_points_topic.c_str(),
                send_tf_transforms ? "true" : "false",
                enable_robot_odometry_prediction ? "true" : "false",
                reg_method.c_str(),
                ndt_neighbor_search_method.c_str(),
                ndt_neighbor_search_radius,
                ndt_resolution,
                scan_matching_refine_enable_ ? "ndt+vgicp" : "ndt",
                imu_data_filter_num_,
                globalmap_voxel_size_,
                points_voxel_filter_size_,
                min_valid_count_,
                buffer_size_,
                imu_init_time_,
                imu_init_queue_size_,
                imu_init_max_gyro_var_,
                imu_init_max_acce_var_);

    global_map_points_ptr_.reset(new pcl::PointCloud<PointT>());
    if (use_imu) {
      RCLCPP_INFO(get_logger(), "enable imu-based prediction");
      correct_imu_data_ptr_ = std::make_shared<sensor_msgs::msg::Imu>();
      imu_sub               = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&HdlLocalizationNode::imu_callback, this, std::placeholders::_1));
    }
    points_sub      = create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, 5, std::bind(&HdlLocalizationNode::points_callback, this, std::placeholders::_1));
    // Subscribe to controller odometry only when it is explicitly needed for
    // NDT prediction or this node owns the map->odom TF.  In the real
    // navigation configuration both are disabled: Mid360 point matching and
    // its built-in IMU are the sole source for the Nav2 odometry chain.
    if (enable_robot_odometry_prediction || send_tf_transforms) {
      robot_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        robot_odom_topic_, rclcpp::QoS(100),
        std::bind(&HdlLocalizationNode::robot_odom_callback, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "Robot odometry subscribed; NDT prediction %s, TF publishing %s, topic=%s",
        enable_robot_odometry_prediction ? "enabled" : "disabled",
        send_tf_transforms ? "enabled" : "disabled", robot_odom_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "Controller odometry disabled for localization and Nav2 pose chains");
    }
    if (enable_lio_primary_) {
      lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        lio_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&HdlLocalizationNode::lio_odom_callback, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(),
        "Indoor LIO primary enabled: topic=%s max_age=%.2fs drift_xy=%.2fm "
        "drift_yaw=%.1fdeg hysteresis=%d jump=%.2fm (NDT/RTK gated auxiliaries)",
        lio_odom_topic_.c_str(), lio_max_age_s_, lio_drift_xy_m_,
        lio_drift_yaw_rad_ * 180.0 / M_PI, lio_drift_hysteresis_frames_,
        lio_max_correction_jump_m_);
    }
    if (use_gnss_fusion_) {
      gnss_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic, rclcpp::SensorDataQoS(),
        std::bind(&HdlLocalizationNode::gnss_callback, this, std::placeholders::_1));
      rtk_pvh_sub_ = create_subscription<robots_dog_msgs::msg::UniRtkPvh>(
        rtk_pvh_topic, rclcpp::SensorDataQoS(),
        std::bind(&HdlLocalizationNode::rtk_pvh_callback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(),
        "GNSS fusion enabled, fix=%s pvh=%s gain=%.3f auto_recovery=%s heading=%s",
        gnss_topic.c_str(), rtk_pvh_topic.c_str(), gnss_fusion_gain_,
        gnss_auto_recovery_enable_ ? "true" : "false", gnss_use_heading_ ? "true" : "false");
    }
    initialpose_sub =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, std::bind(&HdlLocalizationNode::initialpose_callback, this, std::placeholders::_1));
    rtk_initial_pose_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/seed_from_rtk",
      std::bind(&HdlLocalizationNode::rtk_initial_pose_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    // Name fixed by the navigation behaviour tree, which addresses it as
    // /reinitialize_global_localization; see the callback for the contract.
    reinitialize_global_localization_service_ = create_service<std_srvs::srv::Empty>(
      "/reinitialize_global_localization",
      std::bind(&HdlLocalizationNode::reinitialize_global_localization_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    localization_policy_sub_ = create_subscription<std_msgs::msg::String>(
      "/localization/policy", 10,
      std::bind(&HdlLocalizationNode::localization_policy_callback, this, std::placeholders::_1));
    localization_decision_pub_ = create_publisher<std_msgs::msg::String>("/localization/decision", 10);

    localization_lidar_info_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), // 10Hz 
                std::bind(&HdlLocalizationNode::PublishLidarLocalizationInfo, this));
    odom_publish_timer_            = this->create_wall_timer(
                std::chrono::milliseconds(50), // 20Hz
                std::bind(&HdlLocalizationNode::PublishOdomTimer, this));
    pose_pub    = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
    lidar_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(lidar_odom_topic_, 5);
    aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_points_topic, 5);
    status_pub  = create_publisher<localization::msg::ScanMatchingStatus>(status_topic, 5);
    scan_match_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(scan_match_pose_topic, 5);

    localization_state_srv_ = this->create_service<robots_dog_msgs::srv::LocalizationState>(
            "/localization_state/service",
            std::bind(&HdlLocalizationNode::LocalizationStateCallback, this, std::placeholders::_1, std::placeholders::_2));
    load_map_service_ptr_   = create_service<robots_dog_msgs::srv::LoadMap>(
            "/load_map_service", std::bind(&HdlLocalizationNode::LoadMapCallBack, this, std::placeholders::_1, std::placeholders::_2));

    localization_info_pub_ = this->create_publisher<robots_dog_msgs::msg::Localization>(localization_info_topic, 10);
    global_map_pub_        = this->create_publisher<sensor_msgs::msg::PointCloud2>(global_map_points_topic, 1);

    initialize_params();
    raw_points_ptr_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    if (enable_lidar_odometry_prediction_) {
      lidar_odom_registration_ = std::make_unique<fast_gicp::FastGICP<PointT, PointT>>();
      lidar_odom_registration_->setNumThreads(lidar_odom_num_threads_);
      lidar_odom_registration_->setCorrespondenceRandomness(20);
      lidar_odom_registration_->setMaximumIterations(20);
      lidar_odom_registration_->setTransformationEpsilon(0.01);
      lidar_odom_registration_->setMaxCorrespondenceDistance(lidar_odom_max_correspondence_distance_);
      RCLCPP_INFO(
        get_logger(),
        "LiDAR odometry prediction enabled: topic=%s voxel=%.2fm max_score=%.2f max_delta=[%.2fm, %.1fdeg] threads=%d",
        lidar_odom_topic_.c_str(), lidar_odom_voxel_size_, lidar_odom_max_fitness_score_,
        lidar_odom_max_translation_per_scan_,
        lidar_odom_max_rotation_per_scan_rad_ * 180.0 / M_PI, lidar_odom_num_threads_);
    }
    // Initialize sensor data validity tracking
    last_lidar_data_time_ = get_clock()->now();
    last_imu_data_time_   = get_clock()->now();
    // Initialize buffer, mark all as invalid initially
    for (int i = 0; i < buffer_size_; i++) {
      lidar_status_buffer_.push_back(false);
      imu_status_buffer_.push_back(false);
    }
    // Initialize pose history
    last_valid_pose_time_   = get_clock()->now();
    has_valid_pose_history_ = false;
    
    // Initialize confidence management
    last_confidence_update_time_ = get_clock()->now();
    current_confidence_          = 0.0;
    is_extrapolating_            = false;
    
    log_counter_  = 0;
    log_interval_ = 10;
    
    RCLCPP_INFO(get_logger(), "Sensor data validity tracking initialized with buffer size: %d, min valid count: %d", 
                buffer_size_, min_valid_count_);
  }

private:
  localization::StaticIMUInit static_imu_init_;
  float imu_init_time_ = 3.0f;
  int imu_init_queue_size_ = 300;
  float imu_init_max_gyro_var_ = 0.05f;
  float imu_init_max_acce_var_ = 0.2f;
  double gyro_bias_process_noise_ = localization::PoseEstimator::kDefaultGyroBiasProcessNoise;
  double gyro_bias_initial_cov_ = localization::PoseEstimator::kDefaultGyroBiasInitialCov;
  // Latches the one-shot bias write-back for the estimator built before StaticIMUInit
  // finished. Only touched from points_callback, which holds pose_estimator_mutex.
  bool imu_bias_seeded_ = false;
  // Initial pose initialization parameters
  int init_match_count_threshold_ = 5;
  float init_match_score_threshold_ = 0.2f;
  // Initial pose initialization state variables
  int init_match_count_ = 0;

  /**
   * @brief Build a PoseEstimator seeded with the calibrated gyro bias.
   *
   * Centralises what used to be six near-identical constructor calls so the bias
   * and the tuning parameters cannot drift apart between them.
   *
   * The accelerometer bias is deliberately left at zero: StaticIMUInit::GetInitBa()
   * still contains gravity because the gravity estimation in static_imu_init.cpp is
   * commented out, and PoseSystem::f() already removes gravity in the world frame.
   * Passing it here would subtract gravity twice. See pose_estimator.cpp.
   */
  std::unique_ptr<localization::PoseEstimator> createPoseEstimator(
    const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat) {
    const bool calibrated = static_imu_init_.InitSuccess();
    const Eigen::Vector3d bias_gyro =
      calibrated ? static_imu_init_.GetInitBg() : Eigen::Vector3d::Zero();
    // Estimators built before the calibration completes still need the write-back.
    imu_bias_seeded_ = calibrated;
    if (calibrated) {
      RCLCPP_INFO(get_logger(),
        "PoseEstimator seeded with gyro bias [%.6f, %.6f, %.6f] rad/s (|bg| = %.4f deg/s)",
        bias_gyro.x(), bias_gyro.y(), bias_gyro.z(), bias_gyro.norm() * 180.0 / M_PI);
    } else {
      RCLCPP_WARN(get_logger(),
        "PoseEstimator built before static IMU calibration finished; gyro bias starts at zero "
        "and will be written back once calibration succeeds");
    }
    auto estimator = std::make_unique<localization::PoseEstimator>(
      registration, get_clock()->now(), pos, quat, cool_time_duration,
      Eigen::Vector3d::Zero(), bias_gyro,
      gyro_bias_process_noise_, gyro_bias_initial_cov_);
    estimator->configure_scan_matching(
      refine_registration_,
      global_map_points_ptr_,
      scan_matching_local_map_xy_radius_,
      scan_matching_local_map_z_radius_,
      scan_matching_min_local_map_points_,
      ndt_max_fitness_score_,
      scan_matching_coarse_max_fitness_score_);
    return estimator;
  }

  /**
   * @brief One-shot write-back of the calibrated gyro bias into a running estimator.
   *
   * Only estimators built before StaticIMUInit converged can be missing it - notably
   * the one created in initialize_params(), which runs in the node constructor before
   * a single IMU sample has arrived.
   *
   * MUST be called with pose_estimator_mutex held. imu_callback() does not take that
   * mutex, so this may never be called from there - only from points_callback().
   */
  void seedImuBiasesOnce() {
    if (imu_bias_seeded_ || !pose_estimator || !static_imu_init_.InitSuccess()) {
      return;
    }
    const Eigen::Vector3d bg = static_imu_init_.GetInitBg();
    // Accelerometer bias stays zero - same gravity contamination reason as above.
    pose_estimator->set_initial_biases(Eigen::Vector3f::Zero(), bg.cast<float>());
    imu_bias_seeded_ = true;
    RCLCPP_INFO(get_logger(),
      "Gyro bias written back into the running PoseEstimator: [%.6f, %.6f, %.6f] rad/s "
      "(|bg| = %.4f deg/s)",
      bg.x(), bg.y(), bg.z(), bg.norm() * 180.0 / M_PI);
  }

  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(get_logger(), "invalid search method was given");
          RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }
    RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  pcl::Registration<PointT, PointT>::Ptr create_refine_registration() {
    if (!scan_matching_refine_enable_) {
      RCLCPP_INFO(get_logger(), "Indoor VGICP refine disabled; scan matching is NDT only");
      return nullptr;
    }
    fast_gicp::FastVGICP<PointT, PointT>::Ptr vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
    vgicp->setResolution(scan_matching_vgicp_resolution_);
    vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    vgicp->setRegularizationMethod(fast_gicp::RegularizationMethod::PLANE);
    vgicp->setNumThreads(scan_matching_num_threads_);
    vgicp->setMaximumIterations(scan_matching_max_iterations_);
    vgicp->setTransformationEpsilon(0.01);
    vgicp->setMaxCorrespondenceDistance(scan_matching_max_correspondence_distance_);
    RCLCPP_INFO(
      get_logger(),
      "Indoor scan matching: NDT coarse + local FastVGICP refine (res=%.2fm corr=%.2fm iter=%d threads=%d)",
      scan_matching_vgicp_resolution_,
      scan_matching_max_correspondence_distance_,
      scan_matching_max_iterations_,
      scan_matching_num_threads_);
    return vgicp;
  }

  void initialize_params() {
    voxel_filter_ptr_->setLeafSize(points_voxel_filter_size_, points_voxel_filter_size_, points_voxel_filter_size_);
    registration = create_registration();
    refine_registration_ = create_refine_registration();

    // Initialize global localization
    if (use_global_localization_init_) {
      try {
        global_localization_ptr_ = std::make_shared<GlobalLocalization>();
        RCLCPP_INFO(get_logger(), "Global localization initialized successfully");
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to initialize global localization: %s", e.what());
        use_global_localization_init_ = false;
      }
    }
    // initialize pose estimator
    specify_init_pose_ = declare_parameter<bool>("specify_init_pose", true);
    if (specify_init_pose_) {
      RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters!!"); 
      init_pos_x_ = declare_parameter<double>("init_pos_x", 0.0);
      init_pos_y_ = declare_parameter<double>("init_pos_y", 0.0);
      init_pos_z_ = declare_parameter<double>("init_pos_z", 0.0);
      init_ori_w_ = declare_parameter<double>("init_ori_w", 1.0);
      init_ori_x_ = declare_parameter<double>("init_ori_x", 0.0);
      init_ori_y_ = declare_parameter<double>("init_ori_y", 0.0);
      init_ori_z_ = declare_parameter<double>("init_ori_z", 0.0);
  
      Eigen::Vector3f config_pos(init_pos_x_, init_pos_y_, init_pos_z_);
      Eigen::Quaternionf config_quat(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_);
      last_init_pos_ = config_pos;
      last_init_quat_ = config_quat;
      has_set_init_pose_ = true;
      last_pose_source_ = "config";
      // Runs inside the node constructor, so StaticIMUInit has not seen a single
      // sample yet and the gyro bias is necessarily unknown. seedImuBiasesOnce()
      // writes it back from points_callback once the calibration succeeds.
      pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
      RCLCPP_INFO(get_logger(), "Initial pose estimator created with config pose - Position: [%.3f, %.3f, %.3f]",
                   last_init_pos_.x(), last_init_pos_.y(), last_init_pos_.z());
    }
  }

private:
  struct RtkObservation {
    bool usable = false;
    bool heading_usable = false;
    std::string quality = "invalid";
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    double horizontal_std_m = std::numeric_limits<double>::infinity();
    double heading_std_rad = std::numeric_limits<double>::infinity();
    double age_s = std::numeric_limits<double>::infinity();
    double heading_age_s = std::numeric_limits<double>::infinity();
    int64_t stamp_ns = 0;
    int64_t heading_stamp_ns = 0;
  };

  enum class AuxiliaryGateStatus { reject, pending, accept };

  // Shared 0.30 m / 3-frame / no-jump gate for NDT/VGICP and RTK while LIO is
  // the indoor primary observation. Neither source is fused every UKF frame.
  struct AuxiliaryDriftGate {
    int consecutive = 0;
    bool has_prev_source = false;
    Eigen::Vector3f prev_source_xy = Eigen::Vector3f::Zero();
    Eigen::Vector2f prev_correction_xy = Eigen::Vector2f::Zero();
    float prev_residual_xy = 0.0f;
    float prev_residual_yaw = 0.0f;
    int64_t last_stamp_ns = 0;
    bool correction_latched = false;
    std::string last_decision = "idle";

    void reset() {
      consecutive = 0;
      has_prev_source = false;
      prev_correction_xy.setZero();
      prev_residual_xy = 0.0f;
      prev_residual_yaw = 0.0f;
      last_stamp_ns = 0;
      correction_latched = false;
      last_decision = "idle";
    }

    void resetConsecutive() {
      consecutive = 0;
      has_prev_source = false;
      prev_correction_xy.setZero();
      prev_residual_xy = 0.0f;
      prev_residual_yaw = 0.0f;
    }

    void markCorrected() {
      resetConsecutive();
      correction_latched = true;
      last_decision = "corrected_once";
    }
  };

  static Eigen::Isometry3f poseFromOdometry(const nav_msgs::msg::Odometry& msg) {
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.translation() = Eigen::Vector3f(
      static_cast<float>(msg.pose.pose.position.x),
      static_cast<float>(msg.pose.pose.position.y),
      static_cast<float>(msg.pose.pose.position.z));
    Eigen::Quaternionf orientation(
      static_cast<float>(msg.pose.pose.orientation.w),
      static_cast<float>(msg.pose.pose.orientation.x),
      static_cast<float>(msg.pose.pose.orientation.y),
      static_cast<float>(msg.pose.pose.orientation.z));
    if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-6f) {
      orientation = Eigen::Quaternionf::Identity();
    } else {
      orientation.normalize();
    }
    pose.linear() = orientation.toRotationMatrix();
    return pose;
  }

  void lio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(lio_odom_mutex_);
    latest_lio_odom_ = *msg;
    latest_lio_odom_stamp_ = rclcpp::Time(msg->header.stamp);
    has_lio_odom_ = true;
  }

  bool lioOdomFresh(const rclcpp::Time& stamp) const {
    std::lock_guard<std::mutex> lock(lio_odom_mutex_);
    if (!has_lio_odom_) {
      return false;
    }
    // LIO stamps can lead the current lidar stamp by a few milliseconds.
    // Treating that as "stale" drops LIO-primary and falls back to every-frame
    // NDT UKF, which is exactly the indoor flicker we observed.
    const double age = (stamp - latest_lio_odom_stamp_).seconds();
    return std::isfinite(age) && std::fabs(age) <= lio_max_age_s_;
  }

  void resetLioAnchor() {
    lio_anchor_valid_ = false;
    lio_has_previous_pose_ = false;
    lio_corrected_this_frame_ = false;
    ndt_drift_gate_.reset();
    rtk_drift_gate_.reset();
  }

  bool currentLioPose(
      Eigen::Isometry3f& pose,
      float* horizontal_variance = nullptr,
      float* vertical_variance = nullptr,
      float* orientation_variance = nullptr) const {
    std::lock_guard<std::mutex> lock(lio_odom_mutex_);
    if (!has_lio_odom_) {
      return false;
    }
    pose = poseFromOdometry(latest_lio_odom_);
    if (horizontal_variance) {
      const double candidate = std::max(
        latest_lio_odom_.pose.covariance[0], latest_lio_odom_.pose.covariance[7]);
      *horizontal_variance = std::isfinite(candidate) && candidate > 0.0
        ? static_cast<float>(std::clamp(candidate, 1.0e-4, 1.0)) : lio_xy_variance_;
    }
    if (vertical_variance) {
      const double candidate = latest_lio_odom_.pose.covariance[14];
      *vertical_variance = std::isfinite(candidate) && candidate > 0.0
        ? static_cast<float>(std::clamp(candidate, 1.0e-4, 4.0)) : lio_z_variance_;
    }
    if (orientation_variance) {
      const double candidate = std::max({
        latest_lio_odom_.pose.covariance[21],
        latest_lio_odom_.pose.covariance[28],
        latest_lio_odom_.pose.covariance[35]});
      *orientation_variance = std::isfinite(candidate) && candidate > 0.0
        ? static_cast<float>(std::clamp(candidate, 1.0e-5, 1.0)) : lio_orientation_variance_;
    }
    return pose.matrix().allFinite();
  }

  void reanchorLioToUkf() {
    Eigen::Isometry3f T_lio = Eigen::Isometry3f::Identity();
    if (!pose_estimator || !currentLioPose(T_lio)) {
      resetLioAnchor();
      return;
    }
    lio_map_T_lio_ = Eigen::Isometry3f(pose_estimator->matrix()) * T_lio.inverse();
    previous_lio_pose_ = T_lio;
    lio_anchor_valid_ = lio_map_T_lio_.matrix().allFinite();
    lio_has_previous_pose_ = lio_anchor_valid_;
  }

  bool applyLioPrimaryObservation(const rclcpp::Time& stamp) {
    lio_corrected_this_frame_ = false;
    if (!pose_estimator) {
      return false;
    }
    Eigen::Isometry3f T_lio = Eigen::Isometry3f::Identity();
    float horizontal_variance = lio_xy_variance_;
    float vertical_variance = lio_z_variance_;
    float orientation_variance = lio_orientation_variance_;
    if (!currentLioPose(
          T_lio, &horizontal_variance, &vertical_variance, &orientation_variance) ||
        !lioOdomFresh(stamp)) {
      return false;
    }
    if (lio_has_previous_pose_) {
      const Eigen::Isometry3f delta = previous_lio_pose_.inverse() * T_lio;
      if (!delta.matrix().allFinite() || delta.translation().norm() > lio_max_step_m_) {
        RCLCPP_WARN(get_logger(),
          "Rejecting FAST-LIO2 step of %.2fm; waiting to re-anchor on the next healthy scan",
          delta.translation().norm());
        resetLioAnchor();
        return false;
      }
    }
    if (!lio_anchor_valid_) {
      reanchorLioToUkf();
      if (!lio_anchor_valid_) {
        return false;
      }
      RCLCPP_INFO(get_logger(),
        "FAST-LIO2 anchored to the map at [%.3f, %.3f, %.3f]",
        pose_estimator->pos().x(), pose_estimator->pos().y(), pose_estimator->pos().z());
    }
    const Eigen::Isometry3f T_map = lio_map_T_lio_ * T_lio;
    if (!T_map.matrix().allFinite()) {
      resetLioAnchor();
      return false;
    }
    Eigen::Quaternionf orientation(T_map.rotation());
    orientation.normalize();
    pose_estimator->correct_absolute_pose(
      T_map.translation(), orientation,
      horizontal_variance, vertical_variance, orientation_variance);
    previous_lio_pose_ = T_lio;
    lio_has_previous_pose_ = true;
    last_lio_observation_stamp_ = stamp;
    return true;
  }

  AuxiliaryGateStatus evaluateAuxiliaryDriftGate(
      AuxiliaryDriftGate& gate,
      const char* source,
      bool quality_ok,
      bool drifted,
      const Eigen::Vector3f& source_xy,
      const Eigen::Vector2f& correction_xy,
      float residual_xy,
      float residual_yaw,
      int64_t sample_stamp_ns) {
    if (sample_stamp_ns != 0 && sample_stamp_ns == gate.last_stamp_ns) {
      if (gate.consecutive > 0 && gate.consecutive < lio_drift_hysteresis_frames_) {
        return AuxiliaryGateStatus::pending;
      }
      return AuxiliaryGateStatus::reject;
    }

    if (!quality_ok || !source_xy.allFinite() || !std::isfinite(residual_xy)) {
      gate.resetConsecutive();
      gate.last_decision = "quality_rejected";
      return AuxiliaryGateStatus::reject;
    }
    if (residual_xy > static_cast<float>(gnss_max_residual_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s residual %.2fm exceeds %.2fm; rejecting as a jump",
        source, residual_xy, gnss_max_residual_);
      gate.reset();
      gate.last_decision = "jump_rejected";
      return AuxiliaryGateStatus::reject;
    }

    if (gate.has_prev_source) {
      const float source_step =
        (source_xy.head<2>() - gate.prev_source_xy.head<2>()).norm();
      const float residual_delta = std::fabs(residual_xy - gate.prev_residual_xy);
      if (source_step > lio_max_correction_jump_m_ ||
          (gate.consecutive > 0 && residual_delta > lio_max_correction_jump_m_)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "%s jumped (source=%.2fm residual_delta=%.2fm); restarting 3-frame gate",
          source, source_step, residual_delta);
        gate.resetConsecutive();
        gate.has_prev_source = true;
        gate.prev_source_xy = source_xy;
        gate.prev_correction_xy = correction_xy;
        gate.prev_residual_xy = residual_xy;
        gate.prev_residual_yaw = residual_yaw;
        gate.last_stamp_ns = sample_stamp_ns;
        gate.last_decision = "jump_rejected";
        return AuxiliaryGateStatus::reject;
      }
    }

    if (!drifted) {
      gate.resetConsecutive();
      if (residual_xy <= lio_rearm_xy_m_ &&
          residual_yaw <= 0.5f * lio_drift_yaw_rad_) {
        gate.correction_latched = false;
      }
      gate.last_decision = "suppressed_low_drift";
      return AuxiliaryGateStatus::reject;
    }

    if (gate.correction_latched) {
      gate.last_decision = "single_correction_latched";
      return AuxiliaryGateStatus::reject;
    }

    const bool residual_stable = !gate.has_prev_source ||
      ((correction_xy - gate.prev_correction_xy).norm() <= lio_stable_xy_tolerance_m_ &&
       std::fabs(residual_yaw - gate.prev_residual_yaw) <= lio_stable_yaw_tolerance_rad_);
    gate.has_prev_source = true;
    gate.prev_source_xy = source_xy;
    gate.prev_correction_xy = correction_xy;
    gate.prev_residual_xy = residual_xy;
    gate.prev_residual_yaw = residual_yaw;
    gate.last_stamp_ns = sample_stamp_ns;

    if (!residual_stable) {
      gate.consecutive = 1;
      gate.last_decision = "unstable_restart";
      return AuxiliaryGateStatus::pending;
    }

    ++gate.consecutive;
    if (gate.consecutive < lio_drift_hysteresis_frames_) {
      gate.last_decision = "stable_pending";
      return AuxiliaryGateStatus::pending;
    }
    gate.last_decision = "stable_accept";
    return AuxiliaryGateStatus::accept;
  }

  bool maybeCorrectLioDrift(
      const PoseEstimator::MatchResult& match, const rclcpp::Time& stamp) {
    lio_corrected_this_frame_ = false;
    if (!pose_estimator || !match.is_converged_ || !match.transform_.allFinite()) {
      ndt_drift_gate_.resetConsecutive();
      return false;
    }
    const Eigen::Vector3f ndt_position = match.transform_.block<3, 1>(0, 3);
    const Eigen::Vector3f ukf_position = pose_estimator->pos();
    const float drift_xy = (ndt_position.head<2>() - ukf_position.head<2>()).norm();
    Eigen::Quaternionf ndt_orientation(match.transform_.block<3, 3>(0, 0));
    ndt_orientation.normalize();
    const float drift_yaw = pose_estimator->quat().angularDistance(ndt_orientation);
    const bool drifted = drift_xy >= lio_drift_xy_m_ || drift_yaw >= lio_drift_yaw_rad_;
    const Eigen::Vector2f correction_xy =
      ndt_position.head<2>() - ukf_position.head<2>();
    const AuxiliaryGateStatus status = evaluateAuxiliaryDriftGate(
      ndt_drift_gate_, "NDT/VGICP", true, drifted, ndt_position,
      correction_xy, drift_xy, drift_yaw,
      stamp.nanoseconds());
    if (status == AuxiliaryGateStatus::pending) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "LIO/map drift pending correction: xy=%.3fm yaw=%.1fdeg frames=%d/%d score=%.3f",
        drift_xy, drift_yaw * 180.0 / M_PI,
        ndt_drift_gate_.consecutive, lio_drift_hysteresis_frames_, match.fitness_score_);
      return false;
    }
    if (status != AuxiliaryGateStatus::accept) {
      return false;
    }
    pose_estimator->correct_absolute_pose(
      ndt_position, ndt_orientation,
      lio_correct_xy_variance_, lio_correct_z_variance_,
      lio_correct_orientation_variance_);
    reanchorLioToUkf();
    ndt_drift_gate_.markCorrected();
    rtk_drift_gate_.resetConsecutive();
    lio_corrected_this_frame_ = true;
    RCLCPP_WARN(get_logger(),
      "NDT/VGICP corrected LIO drift: xy=%.3fm yaw=%.1fdeg score=%.3f method=%s",
      drift_xy, drift_yaw * 180.0 / M_PI, match.fitness_score_, match.method_.c_str());
    return true;
  }

  bool maybeCorrectRtkDrift(const RtkObservation& observation) {
    if (!pose_estimator || lio_corrected_this_frame_) {
      if (lio_corrected_this_frame_) {
        rtk_drift_gate_.resetConsecutive();
      }
      return false;
    }
    const bool quality_ok = observation.usable && observation.quality == "fixed";
    const Eigen::Vector3f ukf_position = pose_estimator->pos();
    Eigen::Vector3f rtk_position = observation.position;
    if (!gnss_use_elevation_) {
      rtk_position.z() = ukf_position.z();
    }
    const float drift_xy = (rtk_position.head<2>() - ukf_position.head<2>()).norm();
    float drift_yaw = 0.0f;
    if (observation.heading_usable) {
      drift_yaw = pose_estimator->quat().angularDistance(observation.orientation);
    }
    const bool drifted = drift_xy >= lio_drift_xy_m_ ||
      (observation.heading_usable && drift_yaw >= lio_drift_yaw_rad_);
    const Eigen::Vector2f correction_xy =
      rtk_position.head<2>() - ukf_position.head<2>();
    const AuxiliaryGateStatus status = evaluateAuxiliaryDriftGate(
      rtk_drift_gate_, "RTK", quality_ok, drifted, rtk_position,
      correction_xy, drift_xy, drift_yaw,
      observation.stamp_ns);
    if (status == AuxiliaryGateStatus::pending) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "LIO/RTK drift pending correction: xy=%.3fm yaw=%.1fdeg frames=%d/%d quality=%s",
        drift_xy, drift_yaw * 180.0 / M_PI,
        rtk_drift_gate_.consecutive, lio_drift_hysteresis_frames_,
        observation.quality.c_str());
      return false;
    }
    if (status != AuxiliaryGateStatus::accept) {
      return false;
    }

    Eigen::Quaternionf orientation = pose_estimator->quat();
    float orientation_variance = 1.0e6f;
    if (gnss_use_heading_ && observation.heading_usable) {
      orientation = observation.orientation;
      orientation_variance = lio_correct_orientation_variance_;
      last_rtk_heading_fused_stamp_ns_ = observation.heading_stamp_ns;
      rtk_heading_fused_this_frame_ = true;
    }
    pose_estimator->correct_absolute_pose(
      rtk_position, orientation,
      lio_correct_xy_variance_, lio_correct_z_variance_, orientation_variance);
    reanchorLioToUkf();
    rtk_drift_gate_.markCorrected();
    ndt_drift_gate_.resetConsecutive();
    rtk_position_fused_this_frame_ = true;
    lio_corrected_this_frame_ = true;
    last_rtk_map_position_ = rtk_position;
    RCLCPP_WARN(get_logger(),
      "RTK corrected LIO drift: xy=%.3fm yaw=%.1fdeg quality=%s heading=%s",
      drift_xy, drift_yaw * 180.0 / M_PI, observation.quality.c_str(),
      observation.heading_usable ? "yes" : "no");
    return true;
  }

  void localization_policy_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string command = msg->data;
    if (command.find("rtk") != std::string::npos) {
      preferred_source_ = "rtk";
    } else if (command.find("ndt") != std::string::npos) {
      preferred_source_ = "ndt";
    }
    motion_phase_ = command.find("moving") != std::string::npos ? "moving" : "stationary";
    if (motion_phase_ == "stationary") {
      bridge_active_ = false;
      bridge_distance_m_ = 0.0;
      absolute_stable_count_ = 0;
      absolute_stable_ = false;
      stable_source_.clear();
    }
  }

  bool rtkGoodForNavigation(const RtkObservation& observation) const {
    return observation.usable &&
      observation.heading_usable &&
      observation.quality == "fixed";
  }

  void updateRtkAutoPrimary(const RtkObservation& observation, const rclcpp::Time& stamp) {
    if (!prefer_fixed_rtk_ || !source_arbiter_enable_) {
      rtk_auto_primary_latched_ = false;
      rtk_auto_primary_good_frames_ = 0;
      rtk_auto_primary_bad_frames_ = 0;
      return;
    }
    // Indoor LIO-primary: RTK is an auxiliary gated source, never the pose owner
    // while FAST-LIO odom is healthy. Outdoor RTK-primary still latches after LIO drops.
    const bool lio_holds_primary = enable_lio_primary_ && is_init_success_ &&
      has_trusted_ndt_pose_ && lioOdomFresh(stamp);
    if (lio_holds_primary) {
      if (rtk_auto_primary_latched_) {
        rtk_auto_primary_latched_ = false;
        RCLCPP_INFO(get_logger(),
          "Indoor LIO primary is healthy; RTK demoted to a gated auxiliary");
      }
      rtk_auto_primary_good_frames_ = 0;
      rtk_auto_primary_bad_frames_ = 0;
      return;
    }
    if (rtkGoodForNavigation(observation)) {
      rtk_auto_primary_bad_frames_ = 0;
      rtk_auto_primary_good_frames_ = std::min(
        rtk_auto_primary_good_frames_ + 1, rtk_primary_promote_samples_);
      if (!rtk_auto_primary_latched_ &&
          rtk_auto_primary_good_frames_ >= rtk_primary_promote_samples_) {
        rtk_auto_primary_latched_ = true;
        RCLCPP_INFO(get_logger(),
          "Outdoor RTK is fixed with a valid heading; overriding route preference '%s' for RTK-primary navigation",
          preferred_source_.c_str());
      }
      return;
    }
    rtk_auto_primary_good_frames_ = 0;
    rtk_auto_primary_bad_frames_ = std::min(
      rtk_auto_primary_bad_frames_ + 1, rtk_primary_demote_samples_);
    if (rtk_auto_primary_latched_ &&
        rtk_auto_primary_bad_frames_ >= rtk_primary_demote_samples_) {
      rtk_auto_primary_latched_ = false;
      RCLCPP_INFO(get_logger(),
        "RTK is no longer good for primary navigation (quality=%s heading=%s usable=%s); falling back to NDT",
        observation.quality.c_str(),
        observation.heading_usable ? "yes" : "no",
        observation.usable ? "yes" : "no");
    }
  }

  rclcpp::Time timeOnStampClock(int64_t nanoseconds, const rclcpp::Time& stamp) const {
    return rclcpp::Time(nanoseconds, stamp.get_clock_type());
  }

  rclcpp::Time timeOnStampClock(
      const builtin_interfaces::msg::Time& msg, const rclcpp::Time& stamp) const {
    return rclcpp::Time(msg, stamp.get_clock_type());
  }

  RtkObservation currentRtkObservation(const rclcpp::Time& stamp) {
    RtkObservation observation;
    if (!use_gnss_fusion_ || !gnss_map_origin_loaded_) {
      return observation;
    }
    sensor_msgs::msg::NavSatFix gnss;
    bool have_gnss = false;
    {
      std::lock_guard<std::mutex> lock(gnss_mutex_);
      if (has_gnss_) {
        gnss = latest_gnss_;
        have_gnss = true;
      }
    }

    observation.orientation = pose_estimator ? pose_estimator->quat() : last_init_quat_;
    {
      std::lock_guard<std::mutex> lock(gnss_heading_mutex_);
      // int64_t Time() defaults to RCL_SYSTEM_TIME, while lidar stamps are
      // RCL_ROS_TIME. Mixing them throws and aborts every outdoor frame that
      // has a dual-antenna heading.
      const rclcpp::Time heading_stamp = latest_gnss_heading_stamp_ns_ > 0
        ? timeOnStampClock(latest_gnss_heading_stamp_ns_, stamp)
        : timeOnStampClock(latest_gnss_heading_receive_time_.nanoseconds(), stamp);
      // Lidar stamps lag the RTK heading clock (scan + NDT often 0.1-0.6s).
      // A signed (lidar - heading) age is then negative and was treated as
      // "unavailable", so XY fused from RTK while yaw ran on IMU only.
      // Use absolute freshness, matching GNSS position age.
      const double heading_age = has_gnss_heading_
        ? std::fabs((stamp - heading_stamp).seconds())
        : std::numeric_limits<double>::infinity();
      observation.heading_age_s = heading_age;
      observation.heading_usable = has_gnss_heading_ && latest_gnss_heading_status_ == 0 &&
        latest_gnss_heading_type_ > 0 &&
        latest_gnss_heading_baseline_m_ >= gnss_heading_min_baseline_m_ &&
        latest_gnss_heading_std_deg_ <= gnss_heading_max_std_deg_ &&
        heading_age <= gnss_heading_max_age_;
      observation.heading_stamp_ns = heading_stamp.nanoseconds();
      if (observation.heading_usable) {
        const double yaw_enu = M_PI / 2.0 - latest_gnss_heading_deg_ * M_PI / 180.0;
        const double yaw_map = std::atan2(
          std::sin(yaw_enu + gnss_enu_to_map_yaw_ + gnss_heading_offset_rad_),
          std::cos(yaw_enu + gnss_enu_to_map_yaw_ + gnss_heading_offset_rad_));
        observation.orientation = Eigen::AngleAxisf(
          static_cast<float>(yaw_map), Eigen::Vector3f::UnitZ());
        observation.heading_std_rad = latest_gnss_heading_std_deg_ * M_PI / 180.0;
      }
    }

    if (!have_gnss) {
      return observation;
    }
    observation.age_s = std::fabs((stamp - timeOnStampClock(gnss.header.stamp, stamp)).seconds());
    observation.stamp_ns = timeOnStampClock(gnss.header.stamp, stamp).nanoseconds();
    observation.horizontal_std_m = std::sqrt(std::max(
      0.0, std::max(gnss.position_covariance[0], gnss.position_covariance[4])));
    if (gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      observation.quality = "fixed";
    } else if (gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
      observation.quality = "float";
    } else if (gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      observation.quality = "standalone";
    }
    observation.usable = observation.quality != "standalone" && observation.quality != "invalid" &&
      std::isfinite(observation.horizontal_std_m) &&
      observation.horizontal_std_m <= gnss_max_horizontal_std_ &&
      observation.age_s <= gnss_max_age_ &&
      std::fabs(gnss.latitude) > 1e-7 && std::fabs(gnss.longitude) > 1e-7;

    const Eigen::Vector3f gps_map = llaToMap(gnss.latitude, gnss.longitude, gnss.altitude);
    observation.position = gps_map - observation.orientation.toRotationMatrix() * gnss_lever_arm_base_;
    if (!gnss_use_elevation_ && pose_estimator) {
      observation.position.z() = pose_estimator->pos().z();
    }
    return observation;
  }

  float currentEstimatorYaw() const {
    if (!pose_estimator) {
      return 0.0f;
    }
    const Eigen::Matrix3f rotation = pose_estimator->quat().toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
  }

  float slewRtkYaw(float target_yaw) {
    // Low-pass the RTK heading, then inject that filtered yaw fully.
    // Slewing the *pose* toward RTK at 30 deg/s lagged 15-30 deg while
    // turning, so Nav2 steered the wrong way from the first meter.
    if (!rtk_yaw_slew_initialized_) {
      rtk_yaw_slew_initialized_ = true;
      filtered_rtk_yaw_ = target_yaw;
      last_rtk_yaw_slew_time_ = get_clock()->now();
      return target_yaw;
    }
    const rclcpp::Time now = get_clock()->now();
    double dt = (now - last_rtk_yaw_slew_time_).seconds();
    last_rtk_yaw_slew_time_ = now;
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 0.10;
    }
    dt = std::min(std::max(dt, 0.02), 0.25);
    const float err = std::atan2(
      std::sin(target_yaw - filtered_rtk_yaw_),
      std::cos(target_yaw - filtered_rtk_yaw_));
    if (std::fabs(err) > 1.2f) {
      filtered_rtk_yaw_ = target_yaw;
      return target_yaw;
    }
    const double tau = std::max(0.05, gnss_heading_filter_tau_s_);
    const float alpha = static_cast<float>(1.0 - std::exp(-dt / tau));
    filtered_rtk_yaw_ = std::atan2(
      std::sin(filtered_rtk_yaw_ + alpha * err),
      std::cos(filtered_rtk_yaw_ + alpha * err));
    return filtered_rtk_yaw_;
  }

  bool applyRtkObservation(const RtkObservation& observation) {
    if (!pose_estimator || !observation.usable) {
      return false;
    }
    Eigen::Vector3f position = observation.position;
    if (!gnss_use_elevation_) {
      position.z() = 0.0f;
    }
    const float rtk_yaw = std::atan2(
      observation.orientation.toRotationMatrix()(1, 0),
      observation.orientation.toRotationMatrix()(0, 0));
    const float injected_yaw = observation.heading_usable
      ? slewRtkYaw(rtk_yaw) : rtk_yaw;
    const float horizontal_variance = static_cast<float>(std::max(
      observation.horizontal_std_m * observation.horizontal_std_m, 0.20 * 0.20));
    const float vertical_variance = gnss_use_elevation_ ? horizontal_variance : 1.0f;
    const float heading_variance = observation.heading_usable
      ? static_cast<float>(std::max(
          observation.heading_std_rad * observation.heading_std_rad, 0.035 * 0.035))
      : 1.0e6f;
    pose_estimator->inject_rtk_xy_yaw(
      position, observation.heading_usable, injected_yaw, !gnss_use_elevation_,
      horizontal_variance, vertical_variance, heading_variance);
    is_init_success_ = true;
    init_match_count_ = 0;
    last_rtk_map_position_ = position;
    last_rtk_map_yaw_ = rtk_yaw;
    rtk_position_fused_this_frame_ = true;
    if (observation.heading_usable) {
      last_rtk_heading_fused_stamp_ns_ = observation.heading_stamp_ns;
      rtk_heading_fused_this_frame_ = true;
    }
    return true;
  }

  bool applyRtkHeadingObservation(const RtkObservation& observation) {
    if (!pose_estimator || !observation.heading_usable ||
        observation.heading_stamp_ns <= 0 ||
        observation.heading_stamp_ns == last_rtk_heading_fused_stamp_ns_) {
      return false;
    }

    // Keep the current IMU roll/pitch and fuse only the absolute RTK yaw.
    // The large position variances make this a heading-only observation while
    // still using the existing 7D PoseEstimator UKF measurement path.
    const Eigen::Vector3f current_rpy = quaternionToNormalizedRPY(pose_estimator->quat());
    const float rtk_yaw = std::atan2(
      observation.orientation.toRotationMatrix()(1, 0),
      observation.orientation.toRotationMatrix()(0, 0));
    const Eigen::Quaternionf heading_orientation =
      Eigen::AngleAxisf(rtk_yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(current_rpy.y(), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(current_rpy.x(), Eigen::Vector3f::UnitX());
    const float heading_variance = static_cast<float>(
      std::max(observation.heading_std_rad * observation.heading_std_rad, 1e-5));
    pose_estimator->correct_absolute_pose(
      pose_estimator->pos(), heading_orientation, 1.0e6f, 1.0e6f, heading_variance);
    last_rtk_heading_fused_stamp_ns_ = observation.heading_stamp_ns;
    last_rtk_map_yaw_ = rtk_yaw;
    rtk_heading_fused_this_frame_ = true;
    return true;
  }

  bool startBridge(const rclcpp::Time& stamp) {
    if (!source_arbiter_enable_ || !enable_robot_odometry_prediction ||
        !pose_estimator || !has_valid_pose_history_) {
      return false;
    }
    double odom_age = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(robot_odom_mutex_);
      odom_age = (get_clock()->now() - latest_robot_odom_receive_time_).seconds();
    }
    if (odom_age < 0.0 || odom_age > 0.5) {
      return false;
    }
    bridge_active_ = true;
    bridge_start_time_ = stamp;
    bridge_distance_m_ = 0.0;
    active_source_ = "imu_odom_bridge";
    bridge_rejection_reason_.clear();
    absolute_stable_count_ = 0;
    absolute_stable_ = false;
    stable_source_.clear();
    pose_estimator->begin_dead_reckoning_bridge();
    return true;
  }

  bool applyBridgeDelta(
      const Eigen::Matrix4f& odom_delta,
      double odom_dt_s,
      bool odom_time_monotonic,
      const rclcpp::Time& stamp) {
    if (!bridge_active_ || !pose_estimator || !odom_delta.allFinite()) {
      return false;
    }
    Eigen::Vector3f body_translation = odom_delta.block<3, 1>(0, 3);
    body_translation.z() = 0.0f;
    const double distance = body_translation.norm();
    Eigen::Quaternionf delta_q(odom_delta.block<3, 3>(0, 0));
    delta_q.normalize();
    const double angle = Eigen::Quaternionf::Identity().angularDistance(delta_q);
    const double speed = odom_dt_s > 1e-4 ? distance / odom_dt_s : std::numeric_limits<double>::infinity();
    const double yaw_rate = odom_dt_s > 1e-4 ? angle / odom_dt_s : std::numeric_limits<double>::infinity();
    if (!odom_time_monotonic || odom_dt_s > 0.5 || speed > bridge_max_odom_speed_mps_ ||
        yaw_rate > bridge_max_odom_yaw_rate_rps_) {
      bridge_active_ = false;
      active_source_ = "unavailable";
      bridge_rejection_reason_ = !odom_time_monotonic ? "odom_time_discontinuity" :
        (speed > bridge_max_odom_speed_mps_ ? "odom_speed_jump" :
        (yaw_rate > bridge_max_odom_yaw_rate_rps_ ? "odom_yaw_rate_jump" : "odom_stale"));
      RCLCPP_ERROR(get_logger(),
        "Reject bridge odometry: reason=%s dt=%.3fs speed=%.2fm/s yaw_rate=%.2frad/s",
        bridge_rejection_reason_.c_str(), odom_dt_s, speed, yaw_rate);
      return false;
    }
    bridge_distance_m_ += distance;
    pose_estimator->apply_body_odom_translation(
      odom_delta,
      static_cast<float>(bridge_translation_variance_per_m_ * std::max(distance, 0.001)));
    const double elapsed = (stamp - bridge_start_time_).seconds();
    const bool within_limits = bridge_distance_m_ <= bridge_max_distance_m_ &&
      elapsed <= bridge_max_seconds_ &&
      pose_estimator->horizontal_position_sigma() <= bridge_max_horizontal_sigma_m_ &&
      pose_estimator->yaw_sigma() <= bridge_max_yaw_sigma_rad_;
    if (!within_limits) {
      bridge_active_ = false;
      active_source_ = "unavailable";
      RCLCPP_ERROR(get_logger(),
        "IMU+odometry bridge safety limit reached: distance=%.2fm elapsed=%.1fs sigma_xy=%.2fm sigma_yaw=%.1fdeg",
        bridge_distance_m_, elapsed, pose_estimator->horizontal_position_sigma(),
        pose_estimator->yaw_sigma() * 180.0 / M_PI);
    }
    return within_limits;
  }

  void publishLocalizationDecision(const rclcpp::Time& stamp) {
    if (!localization_decision_pub_) {
      return;
    }
    const RtkObservation rtk = currentRtkObservation(stamp);
    // Expose the current RTK map observation independently from whether the
    // source arbiter selected RTK for pose fusion.  In particular, a healthy
    // NDT source may remain active while a fixed RTK solution is still valid.
    // Never serialize unavailable coordinates as zero: zero is a valid map
    // coordinate and would make stale/uninitialized data look trustworthy.
    const bool rtk_position_available = rtk.usable &&
      std::isfinite(rtk.position.x()) && std::isfinite(rtk.position.y());
    const double rtk_map_yaw = std::atan2(
      rtk.orientation.toRotationMatrix()(1, 0),
      rtk.orientation.toRotationMatrix()(0, 0));
    const bool rtk_heading_available = rtk_position_available && rtk.heading_usable &&
      std::isfinite(rtk_map_yaw);
    std::string rtk_blocked_reason = "none";
    if (!use_gnss_fusion_) {
      rtk_blocked_reason = "gnss_fusion_disabled";
    } else if (!gnss_map_origin_loaded_) {
      rtk_blocked_reason = "gnss_origin_not_loaded";
    } else if (!rtk.usable) {
      if (rtk.quality == "invalid") {
        rtk_blocked_reason = "no_fix";
      } else if (rtk.quality == "standalone") {
        rtk_blocked_reason = "quality_insufficient";
      } else if (!std::isfinite(rtk.horizontal_std_m) ||
                 rtk.horizontal_std_m > gnss_max_horizontal_std_) {
        rtk_blocked_reason = "horizontal_error_exceeded";
      } else if (rtk.age_s > gnss_max_age_) {
        rtk_blocked_reason = "position_stale";
      } else {
        rtk_blocked_reason = "quality_insufficient";
      }
    } else if (!rtk.heading_usable) {
      rtk_blocked_reason = "heading_unavailable";
    }
    const bool odom_time_valid = odom_time_source_ == "ros_reception_monotonic" ||
      odom_time_source_ == "duplicate_pose_ignored";
    std_msgs::msg::String message;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"active_source\":\"" << active_source_
        << "\",\"preferred_source\":\"" << preferred_source_
        << "\",\"rtk_auto_primary\":" << (rtk_auto_primary_latched_ ? "true" : "false")
        << ",\"rtk_good_for_navigation\":" << (rtkGoodForNavigation(rtk) ? "true" : "false")
        << ",\"phase\":\"" << motion_phase_
        << "\",\"ndt_healthy\":" << (last_ndt_healthy_ ? "true" : "false")
        << ",\"ndt_score\":" << last_ndt_score_
        << ",\"lio_primary\":" << (enable_lio_primary_ ? "true" : "false")
        << ",\"lio_healthy\":" << (lioOdomFresh(stamp) ? "true" : "false")
        << ",\"lio_anchored\":" << (lio_anchor_valid_ ? "true" : "false")
        << ",\"lio_corrected\":" << (lio_corrected_this_frame_ ? "true" : "false")
        << ",\"ndt_drift_frames\":" << ndt_drift_gate_.consecutive
        << ",\"rtk_drift_frames\":" << rtk_drift_gate_.consecutive
        << ",\"ndt_drift_decision\":\"" << ndt_drift_gate_.last_decision << "\""
        << ",\"rtk_drift_decision\":\"" << rtk_drift_gate_.last_decision << "\""
        << ",\"ndt_correction_latched\":" << (ndt_drift_gate_.correction_latched ? "true" : "false")
        << ",\"rtk_correction_latched\":" << (rtk_drift_gate_.correction_latched ? "true" : "false")
        << ",\"suppressed_covariance_scale\":" << lio_suppressed_covariance_scale_
        << ",\"raw_imu_in_localization_ukf\":" << (use_imu ? "true" : "false")
        << ",\"absolute_stable\":" << (absolute_stable_ ? "true" : "false")
        << ",\"absolute_stable_samples\":" << absolute_stable_count_
        << ",\"rtk_quality\":\"" << rtk.quality
        << "\",\"rtk_usable\":" << (rtk.usable ? "true" : "false")
        << ",\"rtk_heading_usable\":" << (rtk.heading_usable ? "true" : "false")
        << ",\"rtk_heading_age_s\":";
    if (std::isfinite(rtk.heading_age_s)) {
      out << rtk.heading_age_s;
    } else {
      out << "null";
    }
    out << ",\"rtk_blocked_reason\":\"" << rtk_blocked_reason << "\""
        << ",\"rtk_heading_fused\":" << (rtk_heading_fused_this_frame_ ? "true" : "false")
        << ",\"rtk_position_fused\":" << (rtk_position_fused_this_frame_ ? "true" : "false")
        << ",\"heading_offset_deg\":" << (gnss_heading_offset_rad_ * 180.0 / M_PI)
        << ",\"rtk_x\":";
    if (rtk_position_available) {
      out << rtk.position.x();
    } else {
      out << "null";
    }
    out << ",\"rtk_y\":";
    if (rtk_position_available) {
      out << rtk.position.y();
    } else {
      out << "null";
    }
    out << ",\"rtk_yaw\":";
    if (rtk_heading_available) {
      out << rtk_map_yaw;
    } else {
      out << "null";
    }
    out
        << ",\"bridge_distance_m\":" << bridge_distance_m_
        << ",\"bridge_elapsed_s\":"
        << (bridge_active_ ? std::max(0.0, (stamp - bridge_start_time_).seconds()) : 0.0)
        << ",\"bridge_rejection_reason\":\"" << bridge_rejection_reason_ << "\""
        << ",\"odom_time_source\":\"" << odom_time_source_ << "\""
        << ",\"odom_time_valid\":";
    if (!enable_robot_odometry_prediction) {
      out << "null";
    } else {
      out << (odom_time_valid ? "true" : "false");
    }
    out
        << ",\"position_sigma_m\":"
        << (pose_estimator ? pose_estimator->horizontal_position_sigma() : -1.0f)
        << ",\"yaw_sigma_deg\":"
        << (pose_estimator ? pose_estimator->yaw_sigma() * 180.0 / M_PI : -1.0)
        << "}";
    message.data = out.str();
    localization_decision_pub_->publish(message);
  }

  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(gnss_mutex_);
    latest_gnss_ = *msg;
    has_gnss_ = true;
  }

  static bool readYamlScalar(const std::string& path, const std::string& key, double& value) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      return false;
    }
    std::string line;
    const std::string prefix = key + ":";
    while (std::getline(ifs, line)) {
      auto first = line.find_first_not_of(" \t");
      if (first == std::string::npos) {
        continue;
      }
      line = line.substr(first);
      if (line.rfind(prefix, 0) != 0) {
        continue;
      }
      try {
        value = std::stod(line.substr(prefix.size()));
        return true;
      } catch (const std::exception&) {
        return false;
      }
    }
    return false;
  }

  bool loadGnssOriginForMap(const std::string& map_path) {
    gnss_map_origin_loaded_ = false;
    gnss_correction_count_ = 0;

    const std::filesystem::path meta_path = std::filesystem::path(map_path).parent_path() / "gnss_origin.yaml";
    if (!std::filesystem::exists(meta_path)) {
      RCLCPP_WARN(get_logger(), "GNSS map origin not found beside map: %s", meta_path.c_str());
      return false;
    }

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    if (!readYamlScalar(meta_path.string(), "origin_latitude", lat) ||
        !readYamlScalar(meta_path.string(), "origin_longitude", lon) ||
        !readYamlScalar(meta_path.string(), "origin_altitude", alt)) {
      RCLCPP_WARN(get_logger(), "GNSS map origin file is incomplete: %s", meta_path.c_str());
      return false;
    }

    double offset_x = gnss_lever_arm_base_.x();
    double offset_y = gnss_lever_arm_base_.y();
    double offset_z = gnss_lever_arm_base_.z();
    if (!readYamlScalar(meta_path.string(), "map_offset_x", offset_x)) {
      readYamlScalar(meta_path.string(), "x", offset_x);
    }
    if (!readYamlScalar(meta_path.string(), "map_offset_y", offset_y)) {
      readYamlScalar(meta_path.string(), "y", offset_y);
    }
    if (!readYamlScalar(meta_path.string(), "map_offset_z", offset_z)) {
      readYamlScalar(meta_path.string(), "z", offset_z);
    }
    double alignment_locked = 0.0;
    if (!readYamlScalar(meta_path.string(), "alignment_locked", alignment_locked) || alignment_locked < 0.5) {
      RCLCPP_WARN(get_logger(), "GNSS fusion disabled for map without a locked ENU-map alignment: %s", meta_path.c_str());
      return false;
    }
    readYamlScalar(meta_path.string(), "enu_to_map_yaw", gnss_enu_to_map_yaw_);
    gnss_heading_offset_rad_ = gnss_heading_offset_param_rad_;
    double heading_offset_deg = gnss_heading_offset_param_rad_ * 180.0 / M_PI;
    const bool heading_offset_from_origin =
      readYamlScalar(meta_path.string(), "heading_offset_deg", heading_offset_deg);
    if (heading_offset_from_origin) {
      gnss_heading_offset_rad_ = heading_offset_deg * M_PI / 180.0;
    }

    gnss_origin_lat_ = lat;
    gnss_origin_lon_ = lon;
    gnss_origin_alt_ = alt;
    gnss_map_offset_ << static_cast<float>(offset_x), static_cast<float>(offset_y), static_cast<float>(offset_z);
    gnss_map_origin_loaded_ = true;
    rtk_yaw_slew_initialized_ = false;
    RCLCPP_INFO(get_logger(),
      "Loaded GNSS map origin lat=%.9f lon=%.9f alt=%.3f offset=[%.3f, %.3f, %.3f] yaw=%.2fdeg heading_offset=%.2fdeg from=%s",
      gnss_origin_lat_, gnss_origin_lon_, gnss_origin_alt_, gnss_map_offset_.x(), gnss_map_offset_.y(), gnss_map_offset_.z(),
      gnss_enu_to_map_yaw_ * 180.0 / M_PI, gnss_heading_offset_rad_ * 180.0 / M_PI,
      heading_offset_from_origin ? "gnss_origin.yaml" : "config");
    return true;
  }

  Eigen::Vector3f llaToMap(double latitude_deg, double longitude_deg, double altitude_m) const {
    constexpr double kEarthRadiusM = 6378137.0;
    constexpr double kDegToRad = M_PI / 180.0;
    const double d_lat = (latitude_deg - gnss_origin_lat_) * kDegToRad;
    const double d_lon = (longitude_deg - gnss_origin_lon_) * kDegToRad;
    const double lat0 = gnss_origin_lat_ * kDegToRad;
    Eigen::Vector3f enu;
    enu << static_cast<float>(d_lon * std::cos(lat0) * kEarthRadiusM),
           static_cast<float>(d_lat * kEarthRadiusM),
           static_cast<float>(altitude_m - gnss_origin_alt_);
    const float cosine = static_cast<float>(std::cos(gnss_enu_to_map_yaw_));
    const float sine = static_cast<float>(std::sin(gnss_enu_to_map_yaw_));
    Eigen::Vector3f map;
    map.x() = cosine * enu.x() - sine * enu.y() + gnss_map_offset_.x();
    map.y() = sine * enu.x() + cosine * enu.y() + gnss_map_offset_.y();
    map.z() = enu.z() + gnss_map_offset_.z();
    return map;
  }

  void rtk_pvh_callback(const robots_dog_msgs::msg::UniRtkPvh::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(gnss_heading_mutex_);
    const auto& heading = msg->heading;
    latest_gnss_heading_deg_ = static_cast<double>(heading.heading_deg);
    latest_gnss_heading_std_deg_ = static_cast<double>(heading.heading_std);
    latest_gnss_heading_baseline_m_ = static_cast<double>(heading.base_line);
    latest_gnss_heading_status_ = static_cast<int>(heading.sol_status);
    latest_gnss_heading_type_ = static_cast<int>(heading.heading_type);
    latest_gnss_heading_receive_time_ = get_clock()->now();
    latest_gnss_heading_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (latest_gnss_heading_stamp_ns_ <= 0) {
      latest_gnss_heading_stamp_ns_ = latest_gnss_heading_receive_time_.nanoseconds();
    }
    has_gnss_heading_ = true;
  }

  bool seedPositionFromGnss(
    const rclcpp::Time& stamp,
    const char* reason,
    bool require_fixed = false,
    bool require_heading = false) {
    if (!use_gnss_fusion_ || !gnss_map_origin_loaded_) {
      return false;
    }
    sensor_msgs::msg::NavSatFix gnss;
    {
      std::lock_guard<std::mutex> lock(gnss_mutex_);
      if (!has_gnss_) return false;
      gnss = latest_gnss_;
    }
    const double h_std = std::sqrt(std::max(0.0,
      std::max(gnss.position_covariance[0], gnss.position_covariance[4])));
    const double age = std::fabs((stamp - timeOnStampClock(gnss.header.stamp, stamp)).seconds());
    if (gnss.status.status < gnss_min_status_ ||
        (require_fixed && gnss.status.status < sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) ||
        !std::isfinite(h_std) ||
        h_std > gnss_max_horizontal_std_ || age > gnss_max_age_) {
      return false;
    }
    bool heading_applied = false;
    if (gnss_use_heading_) {
      std::lock_guard<std::mutex> lock(gnss_heading_mutex_);
      const double heading_age = has_gnss_heading_
        ? (get_clock()->now() - latest_gnss_heading_receive_time_).seconds()
        : std::numeric_limits<double>::infinity();
      if (has_gnss_heading_ && latest_gnss_heading_status_ == 0 && latest_gnss_heading_type_ > 0 &&
          latest_gnss_heading_baseline_m_ >= gnss_heading_min_baseline_m_ &&
          latest_gnss_heading_std_deg_ <= gnss_heading_max_std_deg_ &&
          heading_age >= 0.0 && heading_age <= gnss_heading_max_age_) {
        const double yaw_enu = M_PI / 2.0 - latest_gnss_heading_deg_ * M_PI / 180.0;
        const double yaw_map = std::atan2(
          std::sin(yaw_enu + gnss_enu_to_map_yaw_ + gnss_heading_offset_rad_),
          std::cos(yaw_enu + gnss_enu_to_map_yaw_ + gnss_heading_offset_rad_));
        last_init_quat_ = Eigen::AngleAxisf(static_cast<float>(yaw_map), Eigen::Vector3f::UnitZ());
        heading_applied = true;
      }
    }
    if (require_heading && !heading_applied) {
      return false;
    }
    const Eigen::Vector3f gps_map = llaToMap(gnss.latitude, gnss.longitude, gnss.altitude);
    const Eigen::Matrix3f rotation = last_init_quat_.normalized().toRotationMatrix();
    const Eigen::Vector3f base_map = gps_map - rotation * gnss_lever_arm_base_;
    last_init_pos_.x() = base_map.x();
    last_init_pos_.y() = base_map.y();
    has_set_init_pose_ = true;
    last_pose_source_ = gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
      ? "rtk_fixed" : "rtk_float";
    RCLCPP_INFO(get_logger(),
      "GNSS seed for %s: mode=%s map_xy=[%.3f, %.3f] std=%.3fm age=%.2fs heading=%s",
      reason, last_pose_source_.c_str(), last_init_pos_.x(), last_init_pos_.y(), h_std, age,
      heading_applied ? "rtk" : "preserved");
    return true;
  }

  bool resetPoseEstimatorFromFixedRtk(const char* reason) {
    if (!seedPositionFromGnss(get_clock()->now(), reason, true, true)) {
      return false;
    }
    pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
    has_trusted_ndt_pose_ = false;
    resetLioAnchor();
    is_init_success_ = false;
    init_match_count_ = 0;
    localization_state_ = 1;
    is_extrapolating_ = false;
    gl_once_gate_ = false;
    consecutive_match_failures_ = 0;
    runtime_relocalization_attempted_ = true;
    gnss_recovery_seed_pending_ = true;
    last_gnss_recovery_attempt_ = std::chrono::steady_clock::now();
    last_global_localization_attempt_ = last_gnss_recovery_attempt_;
    {
      std::lock_guard<std::mutex> imu_lock(imu_data_mutex);
      imu_data.clear();
    }
    {
      std::lock_guard<std::mutex> odom_lock(robot_odom_mutex_);
      odom_prediction_initialized_ = false;
      consumed_robot_odom_sequence_ = latest_robot_odom_sequence_;
    }
    return true;
  }

  /**
   * @brief Arm one global relocalization pass, on request.
   *
   * The navigation behaviour tree's ReinitializeGlobalLocalization node is a
   * BtServiceNode<std_srvs::srv::Empty>, and BtServiceNode::on_configure throws
   * if the server is missing - so the tree cannot reference that node at all
   * until this service exists. This is the server side of that contract.
   *
   * It mirrors the runtime self-healing path (see the NDT-failure branch in
   * points_callback): re-seed from the last trusted pose, drop out of the
   * converged state so the gate below is reachable, and open gl_once_gate_.
   * The expensive ICP itself still runs inside points_callback, so this
   * callback returns immediately instead of blocking the service thread.
   */
  void reinitialize_global_localization_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    if (!has_valid_pose_history_) {
      // Nothing to re-seed from. std_srvs::Empty has no way to report this, so
      // the log is the only channel; the caller gets success either way.
      RCLCPP_WARN(get_logger(),
        "Global relocalization requested but no trusted pose history exists yet; ignoring");
      return;
    }
    last_init_pos_ = last_pose_.block<3, 1>(0, 3);
    last_init_quat_ = Eigen::Quaternionf(last_pose_.block<3, 3>(0, 0));
    last_init_quat_.normalize();
    has_set_init_pose_ = true;
    last_pose_source_ = "service_relocalization";
    is_init_success_ = false;
    init_match_count_ = 0;
    localization_state_ = 1;
    gl_once_gate_ = true;
    // Marked as attempted so the retry rearm at the top of points_callback owns
    // any follow-up passes instead of this service being re-driven by the tree.
    runtime_relocalization_attempted_ = true;
    consecutive_match_failures_ = 0;
    {
      std::lock_guard<std::mutex> imu_lock(imu_data_mutex);
      imu_data.clear();
    }
    {
      std::lock_guard<std::mutex> odom_lock(robot_odom_mutex_);
      odom_prediction_initialized_ = false;
      consumed_robot_odom_sequence_ = latest_robot_odom_sequence_;
    }
    RCLCPP_WARN(get_logger(),
      "Global relocalization armed by service request from last trusted pose x=%.3f y=%.3f",
      last_init_pos_.x(), last_init_pos_.y());
  }

  void rtk_initial_pose_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    response->success = resetPoseEstimatorFromFixedRtk("operator initialization");
    if (response->success) {
      std::ostringstream message;
      message << std::fixed << std::setprecision(3)
              << "fixed RTK initial pose accepted x=" << last_init_pos_.x()
              << " y=" << last_init_pos_.y();
      response->message = message.str();
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      response->message =
        "fixed RTK pose unavailable: require fresh GBAS fix, valid map GNSS alignment, "
        "horizontal std within limit, and valid dual-antenna heading";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  }

  bool applyGnssCorrection(const rclcpp::Time& stamp) {
    if (!use_gnss_fusion_ || !pose_estimator || !gnss_map_origin_loaded_) {
      return false;
    }

    sensor_msgs::msg::NavSatFix gnss;
    {
      std::lock_guard<std::mutex> lock(gnss_mutex_);
      if (!has_gnss_) {
        return false;
      }
      gnss = latest_gnss_;
    }

    if (gnss.status.status < gnss_min_status_) {
      return false;
    }
    if (std::fabs(gnss.latitude) < 1e-7 || std::fabs(gnss.longitude) < 1e-7) {
      return false;
    }
    const double h_std = std::sqrt(std::max(gnss.position_covariance[0], gnss.position_covariance[4]));
    const int64_t gnss_stamp_ns = rclcpp::Time(gnss.header.stamp).nanoseconds();
    if (!std::isfinite(h_std) || h_std > gnss_max_horizontal_std_ || gnss_stamp_ns <= 0 ||
        gnss_stamp_ns == last_gnss_position_fused_stamp_ns_) {
      return false;
    }
    const double age = std::fabs((stamp - timeOnStampClock(gnss.header.stamp, stamp)).seconds());
    if (age > gnss_max_age_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skip GNSS correction: age %.2fs exceeds %.2fs", age, gnss_max_age_);
      return false;
    }
    last_gnss_position_fused_stamp_ns_ = gnss_stamp_ns;

    const Eigen::Matrix4f pose = pose_estimator->matrix();
    const Eigen::Vector3f estimated_gps = pose.block<3, 1>(0, 3) + pose.block<3, 3>(0, 0) * gnss_lever_arm_base_;
    Eigen::Vector3f residual = llaToMap(gnss.latitude, gnss.longitude, gnss.altitude) - estimated_gps;
    if (!gnss_use_elevation_) {
      residual.z() = 0.0f;
    }

    const double residual_norm = residual.norm();
    if (residual_norm > gnss_max_residual_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Reject GNSS correction: residual %.2fm exceeds %.2fm", residual_norm, gnss_max_residual_);
      return false;
    }

    const double quality_gain = gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
      ? gnss_fusion_gain_ : gnss_fusion_gain_ * 0.25;
    Eigen::Vector3f correction = residual * static_cast<float>(quality_gain);
    const double correction_norm = correction.norm();
    if (correction_norm > gnss_max_correction_step_) {
      correction *= static_cast<float>(gnss_max_correction_step_ / correction_norm);
    }
    if (correction.norm() < 1e-4f) {
      return false;
    }

    pose_estimator->apply_position_correction(correction);
    gnss_correction_count_++;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "GNSS localization correction #%d mode=%s residual=%.2fm step=%.3fm",
      gnss_correction_count_,
      gnss.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX ? "rtk_primary" : "hybrid",
      residual_norm, correction.norm());
    return true;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    correct_imu_data_ptr_ = imu_msg;
	    Eigen::Vector3f acceleration(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
	    // Apply rotation matrix and gravity compensation 
	    acceleration = init_rotation_matrix_ * acceleration * imu_acc_scale_;
    correct_imu_data_ptr_->linear_acceleration.x = acceleration.x();
    correct_imu_data_ptr_->linear_acceleration.y = acceleration.y();
    correct_imu_data_ptr_->linear_acceleration.z = acceleration.z();
    Eigen::Vector3f angular_velocity(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    // Apply rotation matrix to angular velocity
    angular_velocity = init_rotation_matrix_ * angular_velocity;
    correct_imu_data_ptr_->angular_velocity.x = angular_velocity.x();
    correct_imu_data_ptr_->angular_velocity.y = angular_velocity.y();
    correct_imu_data_ptr_->angular_velocity.z = angular_velocity.z();
    // Update latest IMU angular velocity for extrapolation
    latest_angular_velocity_ = angular_velocity;
    updateImuStatus(true);
    if (!static_imu_init_.InitSuccess()) {
      static_imu_init_.AddIMUData(correct_imu_data_ptr_);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      imu_data.push_back(correct_imu_data_ptr_);
    }
  }

  void robot_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    Eigen::Quaterniond orientation(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
      return;
    }
    pose.linear() = orientation.normalized().toRotationMatrix();
    std::lock_guard<std::mutex> lock(robot_odom_mutex_);
    latest_robot_odom_ = pose.cast<float>().matrix();
    latest_robot_odom_receive_time_ = get_clock()->now();
    latest_robot_odom_source_stamp_ = rclcpp::Time(msg->header.stamp);
    ++latest_robot_odom_sequence_;
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    auto start = std::chrono::high_resolution_clock::now();
    updateLidarStatus(true);
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex); 
    rtk_heading_fused_this_frame_ = false;
    rtk_position_fused_this_frame_ = false;
    if (use_imu && !static_imu_init_.InitSuccess()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "Radar CallBack Waiting for IMU Initial !!!");
      publishExtrapolatedOdom(points_msg->header.stamp);
      return;
    }
    if (!pose_estimator) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "Radar CallBack Waiting for Initial Pose Input!!");
      pubDefaultLocalizationOdom(points_msg->header.stamp);
      return;
    }
    // The estimator may have been built before StaticIMUInit converged. Past the gate
    // above the calibration is known good, so push the gyro bias in exactly once.
    // Safe here because this callback holds pose_estimator_mutex; imu_callback does not.
    seedImuBiasesOnce();
    // Use the previous frame's RTK-primary latch so this callback can skip
    // cloud conversion before the expensive PCL work. Promotion/demotion still
    // runs later in the same callback from the latest RTK sample.
    const bool skip_lidar_matching =
      is_init_success_ && source_arbiter_enable_ && !bridge_active_ &&
      rtk_auto_primary_latched_;
    if (skip_lidar_matching != lidar_matching_paused_for_rtk_) {
      lidar_matching_paused_for_rtk_ = skip_lidar_matching;
      if (skip_lidar_matching) {
        resetLidarOdometryState();
        RCLCPP_INFO(get_logger(),
          "Outdoor RTK is good for navigation; pausing NDT, VGICP, lidar-odometry, and cloud conversion until RTK drops");
      } else {
        RCLCPP_INFO(get_logger(),
          "RTK is no longer good for navigation; resuming NDT matching from the current RTK pose");
      }
    }
    if (!skip_lidar_matching &&
        (!global_map_points_ptr_ || global_map_points_ptr_->empty())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "Radar CallBack Waiting for Globalmap Input!!");
      pubDefaultLocalizationOdom(points_msg->header.stamp);
      return;
    }
    if (!isSensorDataValid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, "Sensor data invalid, using extrapolation!");
      publishExtrapolatedOdom(points_msg->header.stamp);
      return;
    }

    const builtin_interfaces::msg::Time stamp = skip_lidar_matching
      ? static_cast<builtin_interfaces::msg::Time>(get_clock()->now())
      : points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    if (!skip_lidar_matching) {
      raw_points_ptr_->clear();
      pcl::fromROSMsg(*points_msg, *raw_points_ptr_);
      if (raw_points_ptr_->empty()) {
        RCLCPP_ERROR(get_logger(), "cloud is empty!!");
        return;
      }
      auto filtered = downsample(raw_points_ptr_);
      TransformPoints(filtered, raw_points_ptr_);
    } else {
      raw_points_ptr_->clear();
    }

    try {
    bool gnss_seeded_this_frame = false;
    if (gnss_auto_recovery_enable_ && !gnss_recovery_seed_pending_ &&
        !is_init_success_ && gnss_map_origin_loaded_) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = last_gnss_recovery_attempt_ == std::chrono::steady_clock::time_point{}
        ? std::numeric_limits<double>::infinity()
        : std::chrono::duration<double>(now - last_gnss_recovery_attempt_).count();
      if (elapsed >= gnss_auto_recovery_retry_seconds_ &&
          resetPoseEstimatorFromFixedRtk("automatic recovery")) {
        gnss_seeded_this_frame = true;
        RCLCPP_INFO(get_logger(), "Pose estimator reseeded from fixed RTK XY and heading; validating with local NDT");
      }
    }

    if (gnss_recovery_seed_pending_ && !is_init_success_) {
      const double seed_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_gnss_recovery_attempt_).count();
      if (seed_age >= gnss_auto_recovery_retry_seconds_) {
        gnss_recovery_seed_pending_ = false;
        gl_once_gate_ = true;
        RCLCPP_INFO(get_logger(),
          "RTK local NDT validation did not converge after %.1fs; escalating to bounded global localization",
          seed_age);
      }
    }

    if (!gnss_seeded_this_frame && runtime_relocalization_attempted_ && !is_init_success_ && !gl_once_gate_) {
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_global_localization_attempt_).count();
      if (elapsed >= runtime_relocalization_retry_seconds_) {
        gl_once_gate_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Rearming global relocalization after %.1fs at the last trusted pose",
          elapsed);
      }
    }
    if (use_global_localization_init_ && gl_once_gate_ && global_localization_ptr_ && !is_init_success_) {
      // Consume the one-shot gate before doing the expensive synchronous ICP.
      // A failed attempt must not be retried for every incoming lidar frame.
      gl_once_gate_ = false;
      last_global_localization_attempt_ = std::chrono::steady_clock::now();
      seedPositionFromGnss(rclcpp::Time(points_msg->header.stamp), "global relocalization");
      RCLCPP_INFO(get_logger(), "Attempting global localization for better initial pose...");
      bool relocalized = false;
      if (use_scan_context_ && !scan_context_db_.empty()) {
        relocalized = tryScanContextRelocalization(raw_points_ptr_);
      }
      if (relocalized || performGlobalLocalization(raw_points_ptr_)) {
        RCLCPP_INFO(get_logger(), "Global localization successful! Using new initial pose.");
        pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
        has_trusted_ndt_pose_ = false;
        resetLioAnchor();
        is_init_success_ = false;
        init_match_count_ = 0;
        localization_state_ = 1;
        RCLCPP_INFO(get_logger(), "Pose estimator recreated with global localization result");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Global localization failed once; keeping the last trusted pose and waiting for operator initialization.");
        if (runtime_relocalization_attempted_ && has_valid_pose_history_) {
          pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
          is_extrapolating_ = true;
        }
      }
    }
    
    bool initialized_this_frame = false;
    if (!is_init_success_) {
      localization_state_ = 1;
      PoseEstimator::MatchResult init_result = pose_estimator->GetMatchState();
      RCLCPP_INFO(get_logger(), "init_result.is_converged_ : %d", init_result.is_converged_);
      RCLCPP_INFO(get_logger(), "init_result.fitness_score_ : %f", init_result.fitness_score_);
      // Check if current frame meets initialization criteria
      if (init_result.is_converged_ && init_result.fitness_score_ < init_match_score_threshold_) {
        init_match_count_++;
        RCLCPP_INFO(get_logger(), "Init match count: %d/%d (score: %.6f)", init_match_count_, init_match_count_threshold_, init_result.fitness_score_);
        // Check if we have enough consecutive successful matches
        if (init_match_count_ >= init_match_count_threshold_) {
          is_init_success_ = true;
          initialized_this_frame = true;
          localization_state_ = 2;
          RCLCPP_INFO(get_logger(), "Init Pose Successful!!!");
          init_match_count_ = 0;  // Reset counter
        }
      } else {
        init_match_count_ = 0;
        RCLCPP_INFO(get_logger(), "Init match criteria not met, resetting counter");
      }
      RCLCPP_INFO(get_logger(), "Wait Init Pose!!! Current count: %d/%d", init_match_count_, init_match_count_threshold_);
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());  
    // Do not integrate IMU translation while scan matching is lost or the
    // initial pose is still being verified. Discard samples from that period
    // so a later recovery cannot replay a stale backlog into the UKF.
    // The same-frame IMU dump after Init Pose Successful previously fed a
    // measurement-only covariance into the first predict and aborted the node.
    if (!is_extrapolating_ && !use_imu) {
      pose_estimator->predict(stamp);
    } else if (!is_extrapolating_ && is_init_success_ && !initialized_this_frame) {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      int num = 0;
      for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if (rclcpp::Time(stamp) < rclcpp::Time((*imu_iter)->header.stamp)) {
          break;
        }
        num++;
        if (!(num % imu_data_filter_num_)) {
          const auto& acc = (*imu_iter)->linear_acceleration;
          const auto& gyro = (*imu_iter)->angular_velocity;
          double acc_sign = invert_acc ? -1.0 : 1.0;
          double gyro_sign = invert_gyro ? -1.0 : 1.0;
          Eigen::Vector3f bridge_acceleration = Eigen::Vector3f::Zero();
          if (!bridge_active_) {
            bridge_acceleration =
              static_cast<float>(acc_sign) * Eigen::Vector3f(acc.x, acc.y, acc.z);
          }
          pose_estimator->predict(
            (*imu_iter)->header.stamp,
            bridge_acceleration,
            gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        }
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    } else if (use_imu) {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      imu_data.clear();
    }

    if (!skip_lidar_matching && enable_lidar_odometry_prediction_ && pose_estimator &&
        !(enable_lio_primary_ && is_init_success_ && lioOdomFresh(rclcpp::Time(stamp)) &&
          !rtk_auto_primary_latched_)) {
      pose_estimator->enable_lidar_odometry_prediction();
      updateLidarOdometryPrediction(raw_points_ptr_, rclcpp::Time(stamp));
    }

    // Use adjacent received odometry poses directly. The device header carries
    // controller uptime rather than ROS epoch time, so TF time queries are not
    // a reliable source of motion deltas on this platform.
    Eigen::Matrix4f odom_delta = Eigen::Matrix4f::Identity();
    bool has_odom_delta = false;
    bool robot_odom_fresh = false;
    bool odom_time_monotonic = false;
    double odom_delta_dt_s = 0.0;
    if (enable_robot_odometry_prediction) {
      {
        std::lock_guard<std::mutex> lock(robot_odom_mutex_);
        const double age = (get_clock()->now() - latest_robot_odom_receive_time_).seconds();
        if (latest_robot_odom_sequence_ > 0 && age >= 0.0 && age < 0.5) {
          robot_odom_fresh = true;
          if (!odom_prediction_initialized_) {
            previous_robot_odom_ = latest_robot_odom_;
            previous_robot_odom_receive_time_ = latest_robot_odom_receive_time_;
            previous_robot_odom_source_stamp_ = latest_robot_odom_source_stamp_;
            odom_prediction_initialized_ = true;
          } else if (latest_robot_odom_sequence_ != consumed_robot_odom_sequence_) {
            const double receive_dt =
              (latest_robot_odom_receive_time_ - previous_robot_odom_receive_time_).seconds();
            odom_delta = previous_robot_odom_.inverse() * latest_robot_odom_;
            const Eigen::Vector3f translation = odom_delta.block<3, 1>(0, 3);
            Eigen::Quaternionf rotation(odom_delta.block<3, 3>(0, 0));
            rotation.normalize();
            const bool pose_changed = translation.norm() > 1e-4f ||
              Eigen::Quaternionf::Identity().angularDistance(rotation) > 1e-4f;

            // The factory dog_task header stamp is a non-monotonic controller
            // telemetry counter, not a usable ROS time base. It can repeat or
            // roll back while the same state is republished. Integrate only
            // changed poses, and measure their interval from ROS reception.
            if (!pose_changed) {
              odom_time_source_ = "duplicate_pose_ignored";
            } else {
              has_odom_delta = odom_delta.allFinite();
              odom_delta_dt_s = receive_dt;
              odom_time_monotonic = receive_dt > 0.0 && receive_dt <= 0.5;
              odom_time_source_ = "ros_reception_monotonic";

              previous_robot_odom_ = latest_robot_odom_;
              previous_robot_odom_receive_time_ = latest_robot_odom_receive_time_;
              previous_robot_odom_source_stamp_ = latest_robot_odom_source_stamp_;
            }
          }
          consumed_robot_odom_sequence_ = latest_robot_odom_sequence_;
        }
      }
      // Wheel yaw remains useful for recovering from a transient NDT loss, but
      // must not perturb the operator-provided pose before initialization.
      if (has_odom_delta && is_init_success_ && !bridge_active_) {
        pose_estimator->predict_odom(odom_delta);
      }
    }
    const RtkObservation rtk_observation = currentRtkObservation(rclcpp::Time(stamp));
    updateRtkAutoPrimary(rtk_observation, rclcpp::Time(stamp));
    // Only a latched fixed-RTK + heading solution pauses LiDAR matching.
    // A route waypoint that merely prefers rtk, or a float/usable position
    // without a valid heading, must keep NDT available as the fallback.
    const bool rtk_primary = source_arbiter_enable_ && !bridge_active_ &&
      rtk_auto_primary_latched_;
    if (rtk_primary) {
      resetLioAnchor();
    }
    const bool lio_primary = enable_lio_primary_ && is_init_success_ &&
      has_trusted_ndt_pose_ && !rtk_primary && !bridge_active_ &&
      lioOdomFresh(rclcpp::Time(stamp));
    bool lio_observation_applied = false;
    if (lio_primary) {
      lio_observation_applied = applyLioPrimaryObservation(rclcpp::Time(stamp));
    }
    // Obstacle extraction remains on the independent 10 Hz laser scan chain.
    // Scan matching is reduced to 2 Hz while moving and restored to every
    // LiDAR frame while stationary or during initialization.
    ++ndt_frame_counter_;
    const bool run_ndt = !skip_lidar_matching && !rtk_primary &&
      (motion_phase_ != "moving" || !is_init_success_ ||
      (ndt_frame_counter_ % static_cast<uint64_t>(moving_ndt_stride_) == 0));
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    if (run_ndt) {
      aligned = pose_estimator->correct(
        stamp, raw_points_ptr_,
        is_init_success_ && !rtk_primary && !bridge_active_ && !lio_observation_applied);
      publish_scan_matching_status(points_msg->header, aligned);
      const PoseEstimator::MatchResult match_result = pose_estimator->GetMatchState();
      last_ndt_score_ = match_result.fitness_score_;
      // NDT is an absolute observation. Its health must be decided by the
      // registration result, not by the uncertainty limits reserved for the
      // IMU+wheel-odometry fallback bridge. Otherwise a good NDT correction
      // can be discarded solely because the fallback yaw covariance is high.
      const bool ndt_sample_healthy = match_result.is_converged_ &&
        match_result.fitness_score_ < ndt_max_fitness_score_ && last_ndt_status_healthy_;
      if (ndt_sample_healthy) {
        ndt_unhealthy_frame_count_ = 0;
        last_ndt_healthy_ = true;
      } else {
        ++ndt_unhealthy_frame_count_;
        if (ndt_unhealthy_frame_count_ >= ndt_failure_hysteresis_frames_) {
          last_ndt_healthy_ = false;
        }
      }
      last_ndt_update_time_ = rclcpp::Time(stamp);
      if (lio_observation_applied && ndt_sample_healthy) {
        maybeCorrectLioDrift(match_result, rclcpp::Time(stamp));
      } else if (lio_observation_applied) {
        ndt_drift_gate_.resetConsecutive();
      }
    } else if ((rclcpp::Time(stamp) - last_ndt_update_time_).seconds() > 1.0) {
      last_ndt_healthy_ = false;
      ndt_unhealthy_frame_count_ = ndt_failure_hysteresis_frames_;
      ndt_drift_gate_.resetConsecutive();
    }
    bool absolute_pose_valid = false;
    bool bridge_pose_valid = false;
    bool absolute_observation_updated = false;
    if (bridge_active_) {
      // Repeated controller poses are normal while the robot is stationary.
      // Keep the bridge alive while the stream is fresh; only integrate an
      // actual pose delta. A true source outage still fails robot_odom_fresh.
      bridge_pose_valid = robot_odom_fresh && (!has_odom_delta || applyBridgeDelta(
        odom_delta, odom_delta_dt_s, odom_time_monotonic, rclcpp::Time(stamp)));
    } else if (rtk_primary) {
      absolute_pose_valid = applyRtkObservation(rtk_observation);
      active_source_ = absolute_pose_valid ? "rtk_imu" : "unavailable";
      absolute_observation_updated = absolute_pose_valid &&
        rtk_observation.stamp_ns != last_absolute_observation_stamp_ns_;
      last_absolute_observation_stamp_ns_ = rtk_observation.stamp_ns;
    } else if (lio_observation_applied) {
      absolute_pose_valid = true;
      active_source_ = "lio_imu";
      absolute_observation_updated = true;
      maybeCorrectRtkDrift(rtk_observation);
    } else if (last_ndt_healthy_) {
      absolute_pose_valid = true;
      active_source_ = "ndt_imu";
      absolute_observation_updated = run_ndt;
      if (enable_lio_primary_) {
        maybeCorrectRtkDrift(rtk_observation);
      } else {
        applyRtkHeadingObservation(rtk_observation);
        rtk_position_fused_this_frame_ = applyGnssCorrection(rclcpp::Time(stamp));
      }
    } else if (rtk_observation.usable) {
      absolute_pose_valid = applyRtkObservation(rtk_observation);
      active_source_ = absolute_pose_valid ? "rtk_imu" : "unavailable";
      absolute_observation_updated = absolute_pose_valid &&
        rtk_observation.stamp_ns != last_absolute_observation_stamp_ns_;
      last_absolute_observation_stamp_ns_ = rtk_observation.stamp_ns;
    } else if (startBridge(rclcpp::Time(stamp))) {
      bridge_pose_valid = !has_odom_delta || applyBridgeDelta(
        odom_delta, odom_delta_dt_s, odom_time_monotonic, rclcpp::Time(stamp));
    } else {
      active_source_ = "unavailable";
    }

    if (absolute_pose_valid) {
      if (stable_source_ != active_source_) {
        stable_source_ = active_source_;
        absolute_stable_count_ = 0;
      }
      if (absolute_observation_updated) {
        absolute_stable_count_++;
      }
      absolute_stable_ = absolute_stable_count_ >= absolute_recovery_samples_;
    } else if (bridge_pose_valid || active_source_ == "unavailable") {
      absolute_stable_count_ = 0;
      absolute_stable_ = false;
      stable_source_.clear();
    }

    if (absolute_pose_valid || bridge_pose_valid) {
      localization_state_ = 3;
      consecutive_match_failures_ = 0;
      if (absolute_pose_valid) {
        runtime_relocalization_attempted_ = false;
        gnss_recovery_seed_pending_ = false;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Localization source=%s match=%s score=%.3f ndt=%.3f vgicp=%.3f rtk=%s bridge=%.2fm",
        active_source_.c_str(),
        skip_lidar_matching ? "rtk_paused"
          : (pose_estimator ? pose_estimator->GetMatchState().method_.c_str() : "none"),
        last_ndt_score_,
        pose_estimator ? pose_estimator->GetMatchState().ndt_score_ : last_ndt_score_,
        pose_estimator ? pose_estimator->GetMatchState().refine_score_ : -1.0f,
        rtk_observation.quality.c_str(), bridge_distance_m_);
    } else {
      localization_state_ = 4;
      consecutive_match_failures_++;
      // Keep the estimator and its odometry anchor alive. Recreating it here
      // freezes the seed at the last NDT yaw, making recovery impossible while
      // the robot is still turning.
      if (has_valid_pose_history_) {
        last_velocity_.setZero();
        last_angular_velocity_.setZero();
      }
      is_extrapolating_ = true;
      current_confidence_ = 0.0;
      if (!bridge_active_ && !rtk_observation.usable && !lio_observation_applied &&
          is_init_success_ && has_valid_pose_history_ &&
          !runtime_relocalization_attempted_ &&
          consecutive_match_failures_ >= runtime_relocalization_failure_threshold_) {
        last_init_pos_ = last_pose_.block<3, 1>(0, 3);
        last_init_quat_ = Eigen::Quaternionf(last_pose_.block<3, 3>(0, 0));
        last_init_quat_.normalize();
        has_set_init_pose_ = true;
        last_pose_source_ = "runtime_last_valid";
        is_init_success_ = false;
        init_match_count_ = 0;
        gl_once_gate_ = true;
        runtime_relocalization_attempted_ = true;
        {
          std::lock_guard<std::mutex> imu_lock(imu_data_mutex);
          imu_data.clear();
        }
        {
          std::lock_guard<std::mutex> odom_lock(robot_odom_mutex_);
          odom_prediction_initialized_ = false;
          consumed_robot_odom_sequence_ = latest_robot_odom_sequence_;
        }
        RCLCPP_WARN(
          get_logger(),
          "NDT failed for %d consecutive frames; scheduling one global relocalization from the last trusted pose",
          consecutive_match_failures_);
      }
      RCLCPP_INFO(get_logger(), "Continuous Localization may not good!!!");
    }

    auto end = std::chrono::high_resolution_clock::now();
    last_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (last_timeout_ > 80) {
      RCLCPP_INFO(get_logger(), "!!!point cloud callback time cost > 80ms, = %d ms\n", last_timeout_);
    }

    if (run_ndt && aligned_pub->get_subscription_count()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_pub->publish(aligned_msg);
    }
    // Update pose history for extrapolation
    if (absolute_pose_valid || bridge_pose_valid) {
      Eigen::Matrix4f current_pose = pose_estimator->matrix();
      Eigen::Vector3f current_velocity = getCurrentVelocity(current_pose, points_msg->header.stamp);
      Eigen::Vector3f current_angular_velocity = getCurrentAngularVelocity(current_pose, points_msg->header.stamp);
      updatePoseHistory(current_pose, current_velocity, current_angular_velocity, points_msg->header.stamp);
      is_extrapolating_ = false;
      current_confidence_ = bridge_pose_valid
        ? std::max(0.0, 1.0 - bridge_distance_m_ / bridge_max_distance_m_)
        : 1.0;
      last_confidence_update_time_ = points_msg->header.stamp;
    }
    publishLocalizationDecision(rclcpp::Time(stamp));
    publish_odometry(
      points_msg->header.stamp,
      (absolute_pose_valid || bridge_pose_valid)
        ? pose_estimator->matrix()
        : (has_valid_pose_history_ ? last_pose_ : pose_estimator->matrix()));
    syncLidarOdometryImuAnchor();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Localization points_callback aborted: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Localization points_callback aborted with unknown exception");
    }
  }

  /**
   * @brief callback for initial pose input 
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
  
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    Eigen::Vector3f new_pos(p.x, p.y, p.z);
    Eigen::Quaternionf new_quat(q.w, q.x, q.y, q.z);
    
    bool pose_changed = false;
    if (!has_set_init_pose_) {
      pose_changed = true;
    } else {
      float pos_change = (new_pos - last_init_pos_).norm();
      float quat_change = std::abs(1.0f - std::abs(last_init_quat_.dot(new_quat))); 
      if (pos_change > init_pose_change_threshold_ || quat_change > init_quat_change_threshold_) {
        pose_changed = true;
        RCLCPP_INFO(get_logger(), "Pose change detected - Position: %.3f m, Orientation: %.3f", pos_change, quat_change);
      }
    }
    
    if (pose_changed) {
      last_init_pos_ = new_pos;
      last_init_quat_ = new_quat;
      has_set_init_pose_ = true;
      last_pose_source_ = "Callback";   
      pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
      has_trusted_ndt_pose_ = false;
      resetLioAnchor();
      // Restart verification and allow one bounded ICP refinement from the
      // operator-provided pose. Local NDT alone can only recover small pose
      // errors, while the manual pose is often only approximate.
      is_init_success_ = false;
      init_match_count_ = 0;
      localization_state_ = 1;
      gl_once_gate_ = true;
      is_extrapolating_ = false;
      consecutive_match_failures_ = 0;
      runtime_relocalization_attempted_ = false;
      {
        std::lock_guard<std::mutex> imu_lock(imu_data_mutex);
        imu_data.clear();
      }
      {
        std::lock_guard<std::mutex> odom_lock(robot_odom_mutex_);
        odom_prediction_initialized_ = false;
        consumed_robot_odom_sequence_ = latest_robot_odom_sequence_;
      }
      RCLCPP_INFO(get_logger(), "New initial pose set from RViz - Position: [%.3f, %.3f, %.3f], Quaternion: [%.3f, %.3f, %.3f, %.3f]",
                   last_init_pos_.x(), last_init_pos_.y(), last_init_pos_.z(), last_init_quat_.w(), last_init_quat_.x(), last_init_quat_.y(), last_init_quat_.z());
      RCLCPP_INFO(get_logger(), "Localization will restart with new pose");
    } else {
      RCLCPP_INFO(get_logger(), "Pose unchanged, no action needed");
    }
  }

  pcl::PointCloud<PointT>::Ptr downsample(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    if (!voxel_filter_ptr_) {
      return cloud;
    }
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxel_filter_ptr_->setInputCloud(cloud);
    voxel_filter_ptr_->filter(*filtered);
    filtered->header = cloud->header;
    return filtered;
  }

  void resetLidarOdometryState() {
    previous_lidar_odom_cloud_.reset();
    previous_lidar_imu_pose_.setIdentity();
    lidar_odom_pose_.setIdentity();
    lidar_odom_estimator_instance_ = pose_estimator.get();
    if (pose_estimator) {
      pose_estimator->invalidate_lidar_odometry_prediction();
    }
  }

  void syncLidarOdometryImuAnchor() {
    if (!enable_lidar_odometry_prediction_ || !pose_estimator ||
        !previous_lidar_odom_cloud_) {
      return;
    }
    previous_lidar_imu_pose_ = pose_estimator->matrix();
  }

  bool updateLidarOdometryPrediction(
    const pcl::PointCloud<PointT>::ConstPtr& input_cloud,
    const rclcpp::Time& stamp) {
    if (!enable_lidar_odometry_prediction_ || !pose_estimator ||
        !lidar_odom_registration_ || !input_cloud) {
      return false;
    }
    if (lidar_odom_estimator_instance_ != pose_estimator.get()) {
      resetLidarOdometryState();
    }

    lidar_odom_voxel_filter_.setLeafSize(
      lidar_odom_voxel_size_, lidar_odom_voxel_size_, lidar_odom_voxel_size_);
    lidar_odom_voxel_filter_.setInputCloud(input_cloud);
    pcl::PointCloud<PointT>::Ptr current_cloud(new pcl::PointCloud<PointT>());
    lidar_odom_voxel_filter_.filter(*current_cloud);
    current_cloud->header = input_cloud->header;
    if (current_cloud->size() < lidar_odom_min_points_) {
      pose_estimator->invalidate_lidar_odometry_prediction();
      previous_lidar_odom_cloud_ = current_cloud;
      previous_lidar_imu_pose_ = pose_estimator->matrix();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2.0,
        "LiDAR odometry rejected: only %zu points after %.2fm voxel filter (need %zu)",
        current_cloud->size(), lidar_odom_voxel_size_, lidar_odom_min_points_);
      return false;
    }
    if (!previous_lidar_odom_cloud_ ||
        previous_lidar_odom_cloud_->size() < lidar_odom_min_points_) {
      previous_lidar_odom_cloud_ = current_cloud;
      previous_lidar_imu_pose_ = pose_estimator->matrix();
      return false;
    }

    const Eigen::Matrix4f current_imu_pose = pose_estimator->matrix();
    const Eigen::Matrix4f imu_delta =
      previous_lidar_imu_pose_.inverse() * current_imu_pose;
    pcl::PointCloud<PointT> aligned;
    try {
      lidar_odom_registration_->setInputTarget(previous_lidar_odom_cloud_);
      lidar_odom_registration_->setInputSource(current_cloud);
      lidar_odom_registration_->align(aligned, imu_delta);
    } catch (const std::exception& e) {
      pose_estimator->invalidate_lidar_odometry_prediction();
      previous_lidar_odom_cloud_ = current_cloud;
      previous_lidar_imu_pose_ = current_imu_pose;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1.0,
        "LiDAR odometry align threw: %s", e.what());
      return false;
    }
    const Eigen::Matrix4f lidar_delta = lidar_odom_registration_->getFinalTransformation();
    const float fitness_score = static_cast<float>(lidar_odom_registration_->getFitnessScore());
    const bool finite = lidar_delta.allFinite() && std::isfinite(fitness_score);
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    if (finite) {
      rotation = Eigen::Quaternionf(lidar_delta.block<3, 3>(0, 0));
      rotation.normalize();
    }
    const float translation_m = finite ? lidar_delta.block<3, 1>(0, 3).norm()
                                       : std::numeric_limits<float>::infinity();
    const float rotation_rad = finite
      ? Eigen::Quaternionf::Identity().angularDistance(rotation)
      : std::numeric_limits<float>::infinity();
    const bool valid = finite && lidar_odom_registration_->hasConverged() &&
      fitness_score <= lidar_odom_max_fitness_score_ &&
      translation_m <= lidar_odom_max_translation_per_scan_ &&
      rotation_rad <= lidar_odom_max_rotation_per_scan_rad_;

    previous_lidar_odom_cloud_ = current_cloud;
    previous_lidar_imu_pose_ = current_imu_pose;
    if (!valid) {
      pose_estimator->invalidate_lidar_odometry_prediction();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1.0,
        "LiDAR odometry rejected: converged=%d score=%.3f delta=[%.3fm, %.1fdeg]",
        lidar_odom_registration_->hasConverged(), fitness_score, translation_m,
        rotation_rad * 180.0 / M_PI);
      return false;
    }

    pose_estimator->predict_lidar_odometry(lidar_delta);
    lidar_odom_pose_ = lidar_odom_pose_ * lidar_delta;
    nav_msgs::msg::Odometry lidar_odom;
    lidar_odom.header.stamp = stamp;
    lidar_odom.header.frame_id = "lidar_odom";
    lidar_odom.child_frame_id = "livox_frame";
    lidar_odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(lidar_odom_pose_.cast<double>()));
    lidar_odom.pose.covariance[0] = std::max(1e-4f, fitness_score);
    lidar_odom.pose.covariance[7] = std::max(1e-4f, fitness_score);
    lidar_odom.pose.covariance[35] = std::max(1e-4f, fitness_score);
    lidar_odom_pub_->publish(lidar_odom);
    return true;
  }

  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    const rclcpp::Time tf_stamp =
      (tf_use_current_time ? get_clock()->now() : stamp) +
      rclcpp::Duration::from_seconds(tf_future_offset_);
    RCLCPP_DEBUG(
      get_logger(),
      "[publish_odometry] stamp_ns=%ld tf_stamp_ns=%ld now_ns=%ld send_tf_transforms=%s frame_id=%s child_frame_id=%s pose_xyz=[%.3f, %.3f, %.3f]",
      static_cast<long>(stamp.nanoseconds()),
      static_cast<long>(tf_stamp.nanoseconds()),
      static_cast<long>(get_clock()->now().nanoseconds()),
      send_tf_transforms ? "true" : "false",
      robot_odom_frame_id.c_str(),
      localization_odom_frame_id.c_str(),
      pose(0, 3), pose(1, 3), pose(2, 3));
    if (send_tf_transforms) {
      Eigen::Matrix4f robot_odom = Eigen::Matrix4f::Identity();
      bool has_fresh_robot_odom = false;
      {
        std::lock_guard<std::mutex> lock(robot_odom_mutex_);
        const double age = (get_clock()->now() - latest_robot_odom_receive_time_).seconds();
        if (latest_robot_odom_sequence_ > 0 && age >= 0.0 && age < 0.5) {
          robot_odom = latest_robot_odom_;
          has_fresh_robot_odom = true;
        }
      }
      if (has_fresh_robot_odom) {
        // map_T_odom = map_T_base * inverse(odom_T_base). Computing this from
        // the subscribed odometry avoids a startup cycle through the TF buffer.
        const Eigen::Matrix4f map_to_odom = pose * robot_odom.inverse();
        geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(
          Eigen::Isometry3d(map_to_odom.cast<double>()));
        odom_trans.header.stamp = tf_stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = robot_odom_frame_id;

        tf_broadcaster->sendTransform(odom_trans);
        RCLCPP_DEBUG(
          get_logger(),
          "[publish_odometry] broadcast TF map -> %s at stamp_ns=%ld",
          robot_odom_frame_id.c_str(), static_cast<long>(tf_stamp.nanoseconds()));
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "[publish_odometry] robot odometry unavailable, fallback broadcast map -> %s directly",
          odom_child_frame_id.c_str());
        geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
        odom_trans.header.stamp = tf_stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = odom_child_frame_id;
        tf_broadcaster->sendTransform(odom_trans);
        RCLCPP_DEBUG(
          get_logger(),
          "[publish_odometry] broadcast fallback TF map -> %s at stamp_ns=%ld",
          odom_child_frame_id.c_str(), static_cast<long>(tf_stamp.nanoseconds()));
      }
    }
    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.child_frame_id = localization_odom_frame_id;
    if (pose_estimator) {
      odom.pose.covariance = pose_estimator->pose_covariance();
    } else {
      odom.pose.covariance.fill(0.0);
      odom.pose.covariance[0] = 1.0e6;
      odom.pose.covariance[7] = 1.0e6;
      odom.pose.covariance[14] = 1.0e6;
      odom.pose.covariance[21] = 1.0e6;
      odom.pose.covariance[28] = 1.0e6;
      odom.pose.covariance[35] = 1.0e6;
    }
    // The custom estimator does not publish a measured twist. Mark that fact
    // explicitly instead of exposing a zero covariance for the zero values.
    odom.twist.covariance.fill(0.0);
    odom.twist.covariance[0] = 1.0e6;
    odom.twist.covariance[7] = 1.0e6;
    odom.twist.covariance[14] = 1.0e6;
    odom.twist.covariance[21] = 1.0e6;
    odom.twist.covariance[28] = 1.0e6;
    odom.twist.covariance[35] = 1.0e6;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    pose_pub->publish(odom);
    RCLCPP_DEBUG(
      get_logger(),
      "[publish_odometry] published odom header_ns=%ld frame_id=%s child_frame_id=%s",
      static_cast<long>(static_cast<long long>(odom.header.stamp.sec) * 1000000000LL + odom.header.stamp.nanosec),
      odom.header.frame_id.c_str(),
      odom.child_frame_id.c_str());
  }

  /**
   * @brief Publish a fallback localization odom. Prefer the last trusted pose
   * so a lost/reinit cycle does not teleport Nav2 to the map origin.
   */
  void pubDefaultLocalizationOdom(const rclcpp::Time& stamp) {
    is_extrapolating_ = false;
    current_confidence_ = 0.0;
    if (has_valid_pose_history_) {
      publish_odometry(stamp, last_pose_);
      return;
    }
    Eigen::Matrix4f default_pose = Eigen::Matrix4f::Identity();
    publish_odometry(stamp, default_pose);
  }

  /**
   * @brief Seed global localization from scan context place recognition.
   *
   * The stock path seeds ICP from the last trusted pose and searches no yaw at all, so a
   * robot that has drifted far enough in heading can never recover: every retry re-seeds
   * from a pose that is a little more wrong than the last. Retrieval answers the question
   * directly instead, proposing a map keyframe and a heading from the descriptor.
   *
   * Only scan_context_max_seeds_ candidates are verified per firing, so the per-frame ICP
   * cost matches the stock path - which matters because points_callback runs on a single
   * threaded executor and already warns past 80 ms. The existing
   * runtime_relocalization_retry_seconds_ rearm walks the cursor down the rest of the list.
   *
   * @return true when a candidate passed verification and last_init_pos_ / last_init_quat_
   *         now hold the recovered pose.
   */
  bool tryScanContextRelocalization(const pcl::PointCloud<PointT>::Ptr& current_cloud) {
    if (!current_cloud || current_cloud->empty()) {
      return false;
    }
    const auto candidates = scan_context_db_.query(
      *current_cloud, scan_context_top_k_, scan_context_max_distance_);
    if (candidates.empty()) {
      RCLCPP_WARN(get_logger(), "Scan context returned no candidate; falling back to the last trusted pose");
      return false;
    }
    if (scan_context_cursor_ >= candidates.size()) {
      scan_context_cursor_ = 0;
    }
    const int attempts = std::min<int>(scan_context_max_seeds_, candidates.size());
    for (int attempt = 0; attempt < attempts; ++attempt) {
      const auto candidate = candidates[scan_context_cursor_];
      scan_context_cursor_ = (scan_context_cursor_ + 1) % candidates.size();
      RCLCPP_INFO(
        get_logger(),
        "Scan context candidate %d/%zu: keyframe %d, distance %.3f, yaw %+.1f deg, seed [%.2f, %.2f, %.2f]",
        attempt + 1, candidates.size(), candidate.keyframe_index, candidate.distance,
        candidate.yaw_offset_rad * 180.0 / M_PI,
        candidate.seed_pose(0, 3), candidate.seed_pose(1, 3), candidate.seed_pose(2, 3));
      if (performGlobalLocalization(current_cloud, &candidate.seed_pose)) {
        RCLCPP_INFO(get_logger(), "Scan context seed verified against the map");
        return true;
      }
    }
    RCLCPP_WARN(
      get_logger(),
      "No scan context seed verified this round; next attempt resumes at candidate %zu",
      scan_context_cursor_ + 1);
    return false;
  }

  /**
   * @brief perform global localization
   * @param current_cloud current point cloud
   * @param seed_override optional map-frame seed, e.g. from scan context retrieval;
   *        nullptr keeps the historical last-trusted-pose behaviour
   * @return true if global localization is successful
   */
  bool performGlobalLocalization(const pcl::PointCloud<PointT>::Ptr& current_cloud,
                                 const Eigen::Matrix4d* seed_override = nullptr) {
    if (!global_localization_ptr_ || !global_map_points_ptr_) {
      RCLCPP_WARN(get_logger(), "Global localization not available");
      return false;
    }
    global_localization_in_progress_ = true;
    global_localization_start_time_ = get_clock()->now();  
    RCLCPP_INFO(get_logger(), "Starting global localization..."); 
    try {
      Eigen::Matrix4d initial_trans = Eigen::Matrix4d::Identity();
      if (seed_override != nullptr) {
        // Scan context already answered where and at what heading, so the last trusted
        // pose - the one that has just been shown wrong - must not override it.
        initial_trans = *seed_override;
      } else {
        Eigen::Vector3f init_pos;
        Eigen::Quaternionf init_quat;
        if (!getCurrentInitPose(init_pos, init_quat)) {
          RCLCPP_WARN(get_logger(), "Failed to get initial pose for global localization");
          global_localization_in_progress_ = false;
          return false;
        }
        initial_trans.block<3, 1>(0, 3) = init_pos.cast<double>();
        initial_trans.block<3, 3>(0, 0) = init_quat.toRotationMatrix().cast<double>();
      }
      
      Eigen::Matrix4d final_pose;
      bool localization_success = global_localization_ptr_->performGlobalLocalization(
        global_map_points_ptr_, current_cloud, initial_trans, final_pose);
      
      if (localization_success) {
        Eigen::Vector3f new_pos = final_pose.block<3, 1>(0, 3).cast<float>();
        Eigen::Quaternionf new_quat(final_pose.block<3, 3>(0, 0).cast<float>());
        if (pose_estimator) {
          last_init_pos_ = new_pos;
          last_init_quat_ = new_quat;
          has_set_init_pose_ = true;
          last_pose_source_ = "global_localization";
          RCLCPP_INFO(get_logger(), "Global localization successful! New pose: pos[%.3f, %.3f, %.3f], quat[%.3f, %.3f, %.3f, %.3f]",
                      new_pos.x(), new_pos.y(), new_pos.z(), new_quat.w(), new_quat.x(), new_quat.y(), new_quat.z());
          global_localization_in_progress_ = false;
          return true;
        }
      } else {
        RCLCPP_WARN(get_logger(), "Global localization failed");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Global localization exception: %s", e.what());
    }
    global_localization_in_progress_ = false;
    return false;
  }
  
  /**
   * @brief get current initial pose
   * @param pos output position
   * @param quat output orientation
   * @return true if success
   */
  bool getCurrentInitPose(Eigen::Vector3f& pos, Eigen::Quaternionf& quat) {
    if (has_set_init_pose_) {
      pos = last_init_pos_;
      quat = last_init_quat_;
      return true;
    }
    if (specify_init_pose_) {
      pos = Eigen::Vector3f(init_pos_x_, init_pos_y_, init_pos_z_);
      quat = Eigen::Quaternionf(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_);
      return true;
    }
    pos = Eigen::Vector3f::Zero();
    quat = Eigen::Quaternionf::Identity();
    return true;
  }
  
  /**
   * @brief Check if lidar data is valid
   * @return true: lidar data is valid, false: lidar data is invalid
   */
  bool isLidarDataValid() {
    if (lidar_status_buffer_.size() < min_valid_count_) { return false; }
    int valid_count = std::count(lidar_status_buffer_.begin(), lidar_status_buffer_.end(), true);
    return valid_count >= min_valid_count_;
  }

  /**
   * @brief Check if IMU data is valid
   * @return true: IMU data is valid, false: IMU data is invalid
   */
  bool isImuDataValid() {
    if (!use_imu) { return false; }
    if (imu_status_buffer_.size() < min_valid_count_) { return false; }
    int valid_count = std::count(imu_status_buffer_.begin(), imu_status_buffer_.end(), true);
    return valid_count >= min_valid_count_;
  }

  /**
   * @brief Check if sensor data is valid (at least one of lidar or IMU is valid)
   * @return true: at least one sensor data is valid, false: all sensor data are invalid
   */
  bool isSensorDataValid() { return isLidarDataValid() || isImuDataValid(); }

  /**
   * @brief Update lidar data status
   * @param is_valid whether data is valid
   */
  void updateLidarStatus(bool is_valid) {
    lidar_status_buffer_.push_back(is_valid);
    if (lidar_status_buffer_.size() > buffer_size_) {
      lidar_status_buffer_.pop_front();
    }
    if (is_valid) {
      last_lidar_data_time_ = get_clock()->now();
    }
  }

  /**
   * @brief Update IMU data status
   * @param is_valid whether data is valid
   */
  void updateImuStatus(bool is_valid) {
    imu_status_buffer_.push_back(is_valid);
    if (imu_status_buffer_.size() > buffer_size_) {
      imu_status_buffer_.pop_front();
    }
    if (is_valid) {
      last_imu_data_time_ = get_clock()->now();
    }
  }
  
  // Transform points using gravity transformation matrix 
  void TransformPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out) {
      cloud_out->clear();
      int point_size = cloud_in->points.size();
      cloud_out->resize(point_size);
  #pragma omp parallel for
      for (int i = 0; i < cloud_in->points.size(); ++i) {
          pcl::PointXYZI  point;
          Eigen::Vector4f p_start(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1.0);
          Eigen::Vector4f p_result(gravity_transform_ * p_start);
          point.x              = p_result(0);
          point.y              = p_result(1);
          point.z              = p_result(2);
          point.intensity      = cloud_in->points[i].intensity;
          cloud_out->points[i] = point;
      }
  }

  /**
   * @brief Update pose history for extrapolation
   * @param pose current pose matrix
   * @param velocity current velocity
   * @param angular_velocity current angular velocity
   * @param current_time current time
   */
  void updatePoseHistory(const Eigen::Matrix4f& pose, const Eigen::Vector3f& velocity, 
                         const Eigen::Vector3f& angular_velocity, const rclcpp::Time& current_time) {
    last_pose_ = pose;
    last_velocity_ = velocity;
    last_angular_velocity_ = angular_velocity;
    last_valid_pose_time_ = current_time;
    has_valid_pose_history_ = true;
  }

  /**
   * @brief Get current velocity (prioritize UKF state, extrapolation as backup)
   * @param current_pose current pose
   * @param current_time current time
   * @return current velocity
   */
  Eigen::Vector3f getCurrentVelocity(const Eigen::Matrix4f& current_pose, const rclcpp::Time& current_time) {
    if (pose_estimator) {
      return pose_estimator->vel();
    }
    if (has_valid_pose_history_) {
      double dt = (current_time - last_valid_pose_time_).seconds();
      if (dt > 0.0) {
        Eigen::Vector3f current_position = current_pose.block<3, 1>(0, 3);
        Eigen::Vector3f last_position = last_pose_.block<3, 1>(0, 3);
        Eigen::Vector3f position_delta = current_position - last_position;
        return position_delta / dt;
      }
    }
    return last_velocity_;
  }

  /**
   * @brief Get current angular velocity (prioritize IMU data, extrapolation as backup)
   * @param current_pose current pose
   * @param current_time current time
   * @return current angular velocity
   */
  Eigen::Vector3f getCurrentAngularVelocity(const Eigen::Matrix4f& current_pose, const rclcpp::Time& current_time) {
    if (latest_angular_velocity_.norm() > 0.0) {
      return latest_angular_velocity_;
    }
    if (has_valid_pose_history_) {
      double dt = (current_time - last_valid_pose_time_).seconds();
      if (dt > 0.0) {
        Eigen::Matrix3f current_rotation = current_pose.block<3, 3>(0, 0);
        Eigen::Matrix3f last_rotation = last_pose_.block<3, 3>(0, 0);
        Eigen::Matrix3f relative_rotation = current_rotation * last_rotation.transpose();
        Eigen::AngleAxisf angle_axis(relative_rotation);
        Eigen::Vector3f angular_velocity = angle_axis.axis() * angle_axis.angle() / dt;
        return angular_velocity;
      }
    }
    return last_angular_velocity_;
  }

  /**
   * @brief Get sensor status statistics information
   * @return string containing sensor status statistics information
   */
  std::string getSensorStatusInfo() const {
    std::stringstream ss;
    // Lidar status statistics
    int lidar_valid_count = std::count(lidar_status_buffer_.begin(), lidar_status_buffer_.end(), true);
    int lidar_total_count = lidar_status_buffer_.size();
    double lidar_valid_ratio = lidar_total_count > 0 ? (double)lidar_valid_count / lidar_total_count : 0.0; 
    // IMU status statistics
    int imu_valid_count = std::count(imu_status_buffer_.begin(), imu_status_buffer_.end(), true);
    int imu_total_count = imu_status_buffer_.size();
    double imu_valid_ratio = imu_total_count > 0 ? (double)imu_valid_count / imu_total_count : 0.0;
    
    ss << "Lidar: " << lidar_valid_count << "/" << lidar_total_count 
       << " (" << std::fixed << std::setprecision(1) << (lidar_valid_ratio * 100.0) << "%)"
       << " | IMU: " << imu_valid_count << "/" << imu_total_count 
       << " (" << std::fixed << std::setprecision(1) << (imu_valid_ratio * 100.0) << "%)"
       << " | Pose History: " << (has_valid_pose_history_ ? "Available" : "Not Available")
       << " | Confidence: " << std::fixed << std::setprecision(2) << current_confidence_; 
    return ss.str();
  }

  /**
   * @brief Update confidence
   * @param current_time current time
   */
  void updateConfidence(const rclcpp::Time& current_time) {
    if (!is_init_success_) {
      current_confidence_ = 0.0;
    } else if (is_extrapolating_) {
      // When extrapolating, confidence decays exponentially
      double dt = (current_time - last_confidence_update_time_).seconds();
      if (dt > 0.0) {
        current_confidence_ *= std::exp(-confidence_decay_rate_ * dt);
        if (current_confidence_ < 0.01) {
          current_confidence_ = 0.01;
        }
      }
    } else {
      current_confidence_ = 1.0;
    } 
    last_confidence_update_time_ = current_time;
  }

  /**
   * @brief Get current confidence
   * @return current confidence value
   */
  double getCurrentConfidence() const { return current_confidence_; }

  /**
   * @brief Control log printing frequency
   * @param message message to print
   * @param force_print whether to force print (ignore counter)
   */
  void controlledLogInfo(const std::string& message, bool force_print = false) {
    log_counter_++;
    if (force_print || log_counter_ % log_interval_ == 0) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      log_counter_ = 0;  // Reset counter
    }
  }

  /**
   * @brief Convert quaternion to normalized Euler angles (ZYX order, consistent with Eigen)
   * @param quat quaternion (w, x, y, z)
   * @return normalized Euler angles [roll, pitch, yaw] (ZYX order)
   */
  Eigen::Vector3f quaternionToNormalizedRPY(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalized_quat = quat.normalized();
    float w = normalized_quat.w();
    float x = normalized_quat.x();
    float y = normalized_quat.y();
    float z = normalized_quat.z();
    // 0=Z-axis(yaw), 1=Y-axis(pitch), 2=X-axis(roll)
    float roll, pitch, yaw;
    // Calculate Roll 
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    // Calculate Pitch 
    float sinp = 2.0f * (w * y - x * z);
    if (std::abs(sinp) >= 1.0f) {
      // Handle gimbal lock case (pitch = ±90°)
      pitch = std::copysign(M_PI / 2.0f, sinp);
      roll = 0.0f;
      yaw = 2.0f * std::atan2(x, w);
    } else {
      pitch = std::asin(sinp);
      // Calculate Yaw 
      float siny_cosp = 2.0f * (w * z + x * y);
      float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
      yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    if (roll > M_PI / 2.0f) {
      roll -= M_PI;
      pitch = M_PI - pitch;
      yaw += M_PI;
    } else if (roll < -M_PI / 2.0f) {
      roll += M_PI;
      pitch = -M_PI - pitch;
      yaw += M_PI;
    }
    if (pitch > M_PI / 2.0f) {
      pitch = M_PI - pitch;
      roll += M_PI;
      yaw += M_PI;
    } else if (pitch < -M_PI / 2.0f) {
      pitch = -M_PI - pitch;
      roll += M_PI;
      yaw += M_PI;
    }
    if (yaw > M_PI) {
      yaw -= 2.0f * M_PI;
    } else if (yaw < -M_PI) {
      yaw += 2.0f * M_PI;
    }
    return Eigen::Vector3f(roll, pitch, yaw);
  }

  /**
   * @brief Extrapolate pose based on previous state
   * @param current_time current time
   * @return extrapolated pose matrix
   */
  Eigen::Matrix4f extrapolatePose(const rclcpp::Time& current_time) {
    if (!has_valid_pose_history_) {
      return Eigen::Matrix4f::Identity();
    }
    // Use fixed time step 0.05 seconds (20Hz)
    const double dt = 0.05;
    const double max_velocity = 1.0;        // Maximum velocity
    const double max_angular_velocity = 0.5; // Maximum angular velocity
    Eigen::Vector3f limited_velocity = last_velocity_;
    double velocity_norm = limited_velocity.norm();
    if (velocity_norm > max_velocity) {
      limited_velocity = limited_velocity.normalized() * max_velocity;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, 
                           "Velocity limited from %.2f to %.2f m/s", velocity_norm, max_velocity);
    }
    
    // Limit angular velocity range
    Eigen::Vector3f limited_angular_velocity = last_angular_velocity_;
    double angular_velocity_norm = limited_angular_velocity.norm();
    if (angular_velocity_norm > max_angular_velocity) {
      limited_angular_velocity = limited_angular_velocity.normalized() * max_angular_velocity;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, 
                           "Angular velocity limited from %.2f to %.2f rad/s", angular_velocity_norm, max_angular_velocity);
    }
    Eigen::Vector3f position_delta = limited_velocity * dt;
    Eigen::Vector3f last_position = last_pose_.block<3, 1>(0, 3);
    Eigen::Matrix3f last_rotation = last_pose_.block<3, 3>(0, 0);
    Eigen::Vector3f new_position = last_position + position_delta;
    Eigen::Vector3f angle_delta = limited_angular_velocity * dt;
    Eigen::Matrix3f delta_rotation = Eigen::Matrix3f::Identity();
    delta_rotation = Eigen::AngleAxisf(angle_delta.z(), Eigen::Vector3f::UnitZ()) *
                     Eigen::AngleAxisf(angle_delta.y(), Eigen::Vector3f::UnitY()) *
                     Eigen::AngleAxisf(angle_delta.x(), Eigen::Vector3f::UnitX());
    Eigen::Matrix3f new_rotation = last_rotation * delta_rotation;
    Eigen::Matrix4f extrapolated_pose = Eigen::Matrix4f::Identity();
    extrapolated_pose.block<3, 3>(0, 0) = new_rotation;
    extrapolated_pose.block<3, 1>(0, 3) = new_position;
    
    // Debug information
    RCLCPP_DEBUG(get_logger(), 
                 "Extrapolation: dt=%.3fs, vel=[%.2f,%.2f,%.2f], pos_delta=[%.2f,%.2f,%.2f], "
                 "new_pos=[%.2f,%.2f,%.2f]", 
                 dt, 
                 limited_velocity.x(), limited_velocity.y(), limited_velocity.z(),
                 position_delta.x(), position_delta.y(), position_delta.z(),
                 new_position.x(), new_position.y(), new_position.z());
    
    return extrapolated_pose;
  }

  /**
   * @brief Publish extrapolated localization information
   * @param stamp timestamp
   */
  void publishExtrapolatedOdom(const rclcpp::Time& stamp) {
    if (!has_valid_pose_history_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1.0, 
                           "No pose history available, publishing default pose");
      pubDefaultLocalizationOdom(stamp);
      return;
    }
    // Set extrapolation state to true, start confidence decay
    is_extrapolating_ = true;
    Eigen::Matrix4f extrapolated_pose = extrapolatePose(stamp);
    publish_odometry(stamp, extrapolated_pose);
    last_pose_ = extrapolated_pose;
    last_valid_pose_time_ = stamp; 
    double extrapolation_time = (stamp - last_valid_pose_time_).seconds();  
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1.0, 
                         "Published extrapolated pose and updated history (extrapolation time: %.2fs, "
                         "last velocity: [%.2f, %.2f, %.2f], "
                         "last angular velocity: [%.2f, %.2f, %.2f], "
                         "confidence: %.3f)", 
                         extrapolation_time,
                         last_velocity_.x(), last_velocity_.y(), last_velocity_.z(),
                         last_angular_velocity_.x(), last_angular_velocity_.y(), last_angular_velocity_.z(),
                         getCurrentConfidence());
  }

  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    localization::msg::ScanMatchingStatus status;
    status.header = header;
    const PoseEstimator::MatchResult match = pose_estimator->GetMatchState();
    status.matching_error = std::isfinite(match.fitness_score_)
      ? match.fitness_score_
      : registration->getFitnessScore();
    Eigen::Matrix4f final_transform = match.transform_;
    if (!final_transform.allFinite()) {
      final_transform = registration->getFinalTransformation();
    }
    // Scan matching returns an absolute map pose. Report and validate the frame-to-frame
    // delta; using the absolute translation makes every pose beyond 20 m look
    // like a localization jump.
    Eigen::Matrix4f relative_transform = Eigen::Matrix4f::Identity();
    if (has_valid_pose_history_) {
      relative_transform = last_pose_.inverse() * final_transform;
    }
    const Eigen::Vector3f relative_translation = relative_transform.block<3, 1>(0, 3);
    const double relative_translation_m = static_cast<double>(relative_translation.norm());
    status.relative_pose = tf2::eigenToTransform(
      Eigen::Isometry3d(relative_transform.cast<double>())).transform;
    if (!aligned || aligned->empty()) {
      last_ndt_status_healthy_ = false;
      status.has_converged = false;
      status.inlier_fraction = 0.0f;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2.0,
                           "Scan match status invalid: method=%s aligned cloud is empty, score=%.3f, relative_translation=%.3f",
                           match.method_.c_str(),
                           status.matching_error,
                           relative_translation_m);
      status_pub->publish(status);
      return;
    }
    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    auto target_tree = registration->getSearchMethodTarget();
    if (!target_tree) {
      status.inlier_fraction = match.is_converged_ ? 1.0f : 0.0f;
    } else {
    for (int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      k_indices.clear();
      k_sq_dists.clear();
      if (target_tree->nearestKSearch(pt, 1, k_indices, k_sq_dists) <= 0 || k_sq_dists.empty()) {
        continue;
      }
      if (k_sq_dists.front() < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();
    }
    const bool score_valid = std::isfinite(status.matching_error) &&
      status.matching_error < ndt_max_fitness_score_;
    const bool inliers_valid = status.inlier_fraction >= 0.05f;
    // The first NDT observation after a new initial pose is an acquisition,
    // not a frame-to-frame motion. Comparing it to stale/extrapolated history
    // can permanently reject an otherwise excellent match: history is then
    // never advanced and all later scans see the same large delta. Once NDT
    // has anchored this cycle, retain the 1 m continuity guard.
    const bool transform_valid = std::isfinite(relative_translation_m) &&
      (!has_trusted_ndt_pose_ || relative_translation_m < 1.0);
    status.has_converged = match.is_converged_ && score_valid && inliers_valid && transform_valid;
    last_ndt_status_healthy_ = status.has_converged;
    if (status.has_converged) {
      has_trusted_ndt_pose_ = true;
    }
    if (!status.has_converged) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2.0,
                           "Scan match rejected: method=%s score=%.3f ndt=%.3f vgicp=%.3f inlier=%.3f relative_translation=%.3f",
                           match.method_.c_str(),
                           status.matching_error,
                           match.ndt_score_,
                           match.refine_score_,
                           status.inlier_fraction,
                           relative_translation_m);
    }
    status.prediction_labels.reserve(3);
    status.prediction_errors.reserve(3);
    std::vector<double> errors(6, 0.0);
    if (pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }
    if (pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }
    if (pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }
    if (pose_estimator->lidar_odometry_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "lidar_odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->lidar_odometry_prediction_error().get().cast<double>())).transform);
    }
    // NDT/VGICP produced an absolute map pose in final_transform, but the
    // ScanMatchingStatus above deliberately reduces it to a frame-to-frame delta
    // for its continuity gate, so the absolute match cannot be recovered from a
    // recorded bag. Publish it on its own topic rather than extending
    // ScanMatchingStatus: the Edge Agent subscribes to that message
    // (ros_adapter.py scan_matching_status_topic), and a new field would change
    // its type hash and force a lockstep restart of both sides.
    if (final_transform.allFinite()) {
      const geometry_msgs::msg::Transform matched =
        tf2::eigenToTransform(Eigen::Isometry3d(final_transform.cast<double>())).transform;
      geometry_msgs::msg::PoseStamped scan_match_pose;
      scan_match_pose.header.stamp = header.stamp;
      scan_match_pose.header.frame_id = "map";
      scan_match_pose.pose.position.x = matched.translation.x;
      scan_match_pose.pose.position.y = matched.translation.y;
      scan_match_pose.pose.position.z = matched.translation.z;
      scan_match_pose.pose.orientation = matched.rotation;
      scan_match_pose_pub_->publish(scan_match_pose);
    }
    status_pub->publish(status);
  }

  void PublishLidarLocalizationInfo() {
        auto current_time = this->get_clock()->now();
        updateConfidence(current_time);
        if (lidar_status_buffer_.size() > 0) {
            double lidar_time_diff = (current_time - last_lidar_data_time_).seconds();
            if (lidar_time_diff > sensor_timeout_threshold_) {
                updateLidarStatus(false);
            }
        }  
        if (imu_status_buffer_.size() > 0) {
            double imu_time_diff = (current_time - last_imu_data_time_).seconds();
            if (imu_time_diff > sensor_timeout_threshold_) {
                updateImuStatus(false);
            }
        }
        robots_dog_msgs::msg::Localization msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.type = "loc_state";

        if (!is_init_success_) {
            msg.coord_type = is_use_map_coord_ ? 0 : 1;
            msg.vel.x = msg.vel.y = msg.vel.z = 0.0;
            msg.acc.x = msg.acc.y = msg.acc.z = 0.0;
            msg.gyro.x = msg.gyro.y = msg.gyro.z = 0.0;
            msg.speed = 0.0;
            current_confidence_ = 0.0;
            if (has_valid_pose_history_) {
              // Runtime relocalization must not yank Nav2 to the map origin.
              // Keep the last trusted pose and report Lost so motion is held.
              msg.status = 4;
              const Eigen::Vector3f position = last_pose_.block<3, 1>(0, 3);
              if (msg.coord_type == 0) {
                msg.pos.x = position.x();
                msg.pos.y = position.y();
                msg.pos.z = position.z();
              }
              Eigen::Quaternionf quat(last_pose_.block<3, 3>(0, 0));
              Eigen::Vector3f rpy = quaternionToNormalizedRPY(quat);
              msg.rpy.x = rpy(0);
              msg.rpy.y = rpy(1);
              msg.rpy.z = rpy(2);
            } else {
              msg.status = 0;
              msg.pos.x = msg.pos.y = msg.pos.z = 0.0;
              msg.rpy.x = msg.rpy.y = msg.rpy.z = 0.0;
            }
            localization_info_pub_->publish(msg);
            controlledLogInfo("Sensor Status: " + getSensorStatusInfo());
            return;
        }
        if (!isSensorDataValid()) {
            // Sensor data invalid, use extrapolated pose, set status to 4 (localization failed)
            msg.status = 4;
            msg.coord_type = is_use_map_coord_ ? 0 : 1;          
            if (has_valid_pose_history_) {
                Eigen::Matrix4f extrapolated_pose = extrapolatePose(current_time);
                Eigen::Vector3f position = extrapolated_pose.block<3, 1>(0, 3);
                Eigen::Matrix3f rotation = extrapolated_pose.block<3, 3>(0, 0);        
                if (msg.coord_type == 0) {
                    msg.pos.x = position.x();
                    msg.pos.y = position.y();
                    msg.pos.z = position.z();
                }
                Eigen::Quaternionf quat(rotation);
                Eigen::Vector3f rpy = quaternionToNormalizedRPY(quat);
                msg.rpy.x = rpy(0); // roll
                msg.rpy.y = rpy(1); // pitch
                msg.rpy.z = rpy(2); // yaw
            } else {
                msg.pos.x = msg.pos.y = msg.pos.z = 0.0;
                msg.rpy.y = msg.rpy.z = 0.0;
            }  
            msg.vel.x = msg.vel.y = msg.vel.z = 0.0;
            msg.acc.x = msg.acc.y = msg.acc.z = 0.0;
            msg.gyro.x = msg.gyro.y = msg.gyro.z = 0.0;
            msg.speed = 0.0;
            
            localization_info_pub_->publish(msg);
            controlledLogInfo("Sensor Status: " + getSensorStatusInfo() + " | Publishing extrapolated pose with status 4");
            return;
        }
        if (localization_state_ == 0) {
            msg.status = 0;
        } else if (localization_state_ == 1) {
            msg.status = 1;
        } else if (localization_state_ == 2) {
            msg.status = 2;
        } else if (localization_state_ == 4) {
            msg.status = 4;
        } else {
            msg.status = 3;
        }
        msg.coord_type = is_use_map_coord_ ? 0 : 1; // 0 map coordinate, 1 latitude and longitude coordinate

        Eigen::VectorXf state = pose_estimator->GetCurrentUkfState();
        if (msg.coord_type == 0) {
            msg.pos.x = state(0); // pos.x
            msg.pos.y = state(1); // pos.y
            msg.pos.z = state(2); // pos.z
        }
        {
            Eigen::Matrix3f rotation = pose_estimator->matrix().block<3, 3>(0, 0);
            Eigen::Quaternionf quat(rotation);
            Eigen::Vector3f rpy = quaternionToNormalizedRPY(quat);
            msg.rpy.x = rpy(0); // roll
            msg.rpy.y = rpy(1); // pitch
            msg.rpy.z = rpy(2); // yaw
        }
        if (correct_imu_data_ptr_) {
            const auto &imu = *correct_imu_data_ptr_;
            msg.acc.x = imu.linear_acceleration.x;
            msg.acc.y = imu.linear_acceleration.y;
            msg.acc.z = imu.linear_acceleration.z;

            msg.gyro.x = imu.angular_velocity.x;
            msg.gyro.y = imu.angular_velocity.y;
            msg.gyro.z = imu.angular_velocity.z;
        } else {
            msg.acc.x = msg.acc.y = msg.acc.z = 0.0;
            msg.gyro.x = msg.gyro.y = msg.gyro.z = 0.0;
        }
        const Eigen::Vector3f vel = pose_estimator->vel();
        msg.vel.x = vel(0);
        msg.vel.y = vel(1);
        msg.vel.z = vel(2);
        msg.speed = vel.norm();
        localization_info_pub_->publish(msg);
        controlledLogInfo("Sensor Status: " + getSensorStatusInfo());
    }

    /**
     * @brief Odometry publishing timer callback function
     * Only provides supplementary odometry publishing when sensors fail or not initialized, does not interfere with original logic
     */
    void PublishOdomTimer() {
        if (!is_init_success_) {
            RCLCPP_DEBUG(get_logger(), "Localization not initialized, publishing default pose");
            pubDefaultLocalizationOdom(this->get_clock()->now());
            return;
        }  

        if (is_extrapolating_) {
            if (has_valid_pose_history_) {
                // A matching failure is different from missing sensor data: keep
                // TF fixed at the verified pose until NDT recovers.
                publish_odometry(this->get_clock()->now(), last_pose_);
            } else {
                pubDefaultLocalizationOdom(this->get_clock()->now());
            }
            return;
        }

        if (!isSensorDataValid()) {
            RCLCPP_DEBUG(get_logger(), "Sensor data invalid, checking pose history...");
            auto current_time = this->get_clock()->now();
            if (has_valid_pose_history_) {
                double time_since_last_pose = (current_time - last_valid_pose_time_).seconds();
                RCLCPP_DEBUG(get_logger(), "Time since last pose: %.2fs, using extrapolation", time_since_last_pose);
                
                publishExtrapolatedOdom(current_time);
                return;
            } else {
                RCLCPP_DEBUG(get_logger(), "No valid pose history available, using default pose");
                pubDefaultLocalizationOdom(current_time);
                return;
            }
        }
        // Sensor availability and scan-match validity are independent. Only a
        // valid NDT correction may leave extrapolation mode.
        if (pose_estimator) {
            publish_odometry(this->get_clock()->now(), pose_estimator->matrix());
        }
        RCLCPP_DEBUG(get_logger(), "Sensor data valid, republished odometry/TF from latest pose");
    }

    void LocalizationStateCallback(const std::shared_ptr<robots_dog_msgs::srv::LocalizationState::Request> request,
            std::shared_ptr<robots_dog_msgs::srv::LocalizationState::Response> response) {
         uint8_t receive_message = request->data;
        switch (receive_message) {
        case 0: 
            mode_state_.store(ModeState::INIT);
            response->success = true;
            response->message = "Initialized (Mode: INIT)";
            break;
        case 2:
            mode_state_.store(ModeState::READY);
            response->success = true;
            response->message = "Set READY State (will auto switch to ACTIVE when ready)";
            break;
        case 4: 
            mode_state_.store(ModeState::SUCCESS);
            response->success = true;
            response->message = "Localization stopped (Mode: SUCCESS)";
            break;
        default:
            response->success = false;
            response->message = "Invalid State";
            break;
        }
    }

    /**
     * @brief Load the scan context keyframe database shipped beside the map.
     *
     * Same convention loadGnssOriginForMap already uses: the map session directory is
     * map.pcd's parent. A missing or unreadable database is not an error - relocalization
     * simply keeps the stock last-trusted-pose seed.
     */
    void loadScanContextForMap(const std::string& map_path) {
        scan_context_db_.clear();
        scan_context_cursor_ = 0;
        if (!use_scan_context_) {
            return;
        }
        const std::string map_dir = std::filesystem::path(map_path).parent_path().string();
        std::string error;
        if (!scan_context_db_.load(map_dir, &error)) {
            RCLCPP_WARN(get_logger(), "Scan context relocalization unavailable for %s: %s",
                        map_dir.c_str(), error.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Scan context database loaded: %zu keyframes from %s",
                    scan_context_db_.size(), map_dir.c_str());
    }

    void LoadMapCallBack(robots_dog_msgs::srv::LoadMap::Request::SharedPtr request, 
            robots_dog_msgs::srv::LoadMap::Response::SharedPtr response) {
        update_map_flag_.store(true);
        std::string map_path = request->pcd_path;

        if (!std::filesystem::exists(std::filesystem::path(map_path))) {
            RCLCPP_ERROR(get_logger(), "Map file does not exist: %s", map_path.c_str());
            response->success = false;
            response->message = "Map file does not exist.";
            return;
        }
        global_map_points_ptr_->clear();
        pcl::io::loadPCDFile(map_path, *global_map_points_ptr_);
        RCLCPP_INFO(get_logger(), "Global map points size==: %zu", global_map_points_ptr_->points.size());
        if (!global_map_points_ptr_->empty()) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(global_map_points_ptr_);
            voxel.setLeafSize(globalmap_voxel_size_, globalmap_voxel_size_, globalmap_voxel_size_);
            voxel.filter(*global_map_points_ptr_);
            if (global_map_points_ptr_->points.size() < 1000) {
                RCLCPP_ERROR(get_logger(), "Global map points size is too small: %zu", global_map_points_ptr_->points.size());
                response->success = false;
                response->message = "Global map points size is too small.";
                return;
            } else {
                if (global_map_pub_->get_subscription_count()) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::VoxelGrid<pcl::PointXYZI> publish_voxel;
                    publish_voxel.setInputCloud(global_map_points_ptr_);
                    publish_voxel.setLeafSize(0.5f, 0.5f, 0.5f); // Only for publishing
                    publish_voxel.filter(*global_map_downsampled);
                    sensor_msgs::msg::PointCloud2 global_map_msg;
                    pcl::toROSMsg(*global_map_downsampled, global_map_msg);
                    global_map_msg.header.frame_id = "map";
                    global_map_msg.header.stamp = this->get_clock()->now();
                    global_map_pub_->publish(global_map_msg);
                    RCLCPP_INFO(get_logger(), "Published downsampled global map to /global_map");
                }
                response->success = true;
                response->message = "Map update successfully.";
                RCLCPP_INFO(get_logger(), "Global map updated!!!!!!");
                registration->setInputTarget(global_map_points_ptr_);;
                if (use_gnss_fusion_) {
                    loadGnssOriginForMap(map_path);
                }
                loadScanContextForMap(map_path);
                Reset();
                update_map_flag_.store(false);
            }
        }
        else {
            RCLCPP_ERROR(get_logger(), "Loaded map is empty: %s", map_path.c_str());
            response->success = false;
            response->message = "Loaded map is empty.";
            update_map_flag_.store(false);
            return;
        }
        update_map_flag_.store(false);
    }

    void Reset() {
        is_init_success_ = false;
        localization_state_ = 0; 
        init_match_count_ = 0;
        gl_once_gate_ = true;
        scan_context_cursor_ = 0;
        consecutive_match_failures_ = 0;
        runtime_relocalization_attempted_ = false;
        gnss_recovery_seed_pending_ = false;
        last_gnss_recovery_attempt_ = std::chrono::steady_clock::time_point{};
        has_set_init_pose_ = false;
        last_init_pos_ = Eigen::Vector3f(init_pos_x_, init_pos_y_, init_pos_z_);
        last_init_quat_ = Eigen::Quaternionf(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_);
        seedPositionFromGnss(get_clock()->now(), "map initialization");
        pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
        has_trusted_ndt_pose_ = false;
        resetLidarOdometryState();
        resetLioAnchor();
        is_extrapolating_ = false;
        {
          std::lock_guard<std::mutex> imu_lock(imu_data_mutex);
          imu_data.clear();
        }
        {
          std::lock_guard<std::mutex> odom_lock(robot_odom_mutex_);
          odom_prediction_initialized_ = false;
          consumed_robot_odom_sequence_ = latest_robot_odom_sequence_;
        }
        lidar_status_buffer_.clear();
        imu_status_buffer_.clear();
        for (int i = 0; i < buffer_size_; i++) {
          lidar_status_buffer_.push_back(false);
          imu_status_buffer_.push_back(false);
        }
        has_valid_pose_history_ = false;
    }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  std::string robot_odom_topic_;
  std::string lidar_odom_topic_;
  std::string localization_odom_frame_id;
  bool send_tf_transforms;
  bool tf_use_current_time;
  double tf_future_offset_ = 0.0;

	  bool use_imu;
	  bool invert_acc;
	  bool invert_gyro;
	  double imu_acc_scale_ = 9.81;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                         imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 points_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                   gnss_sub;
  rclcpp::Subscription<robots_dog_msgs::msg::UniRtkPvh>::SharedPtr              rtk_pvh_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr                 globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                              rtk_initial_pose_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                reinitialize_global_localization_service_;
  rclcpp::TimerBase::SharedPtr localization_lidar_info_timer_;
  rclcpp::TimerBase::SharedPtr odom_publish_timer_; 

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               lidar_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            robot_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            lio_odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         aligned_pub;
  rclcpp::Publisher<localization::msg::ScanMatchingStatus>::SharedPtr status_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_match_pose_pub_;
  rclcpp::Publisher<robots_dog_msgs::msg::Localization>::SharedPtr    localization_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         global_map_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr              localization_policy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 localization_decision_pub_;

  std::unique_ptr<tf2_ros::Buffer>               tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data;

  std::mutex gnss_mutex_;
  sensor_msgs::msg::NavSatFix latest_gnss_;
  bool has_gnss_ = false;
  bool gnss_map_origin_loaded_ = false;
  double gnss_origin_lat_ = 0.0;
  double gnss_origin_lon_ = 0.0;
  double gnss_origin_alt_ = 0.0;
  Eigen::Vector3f gnss_map_offset_{0.0f, 0.0f, 0.0f};
  double gnss_enu_to_map_yaw_ = 0.0;
  Eigen::Vector3f gnss_lever_arm_base_{-0.05f, 0.0f, 0.15f};
  bool use_gnss_fusion_ = false;
  double gnss_fusion_gain_ = 0.03;
  double gnss_max_correction_step_ = 0.25;
  double gnss_max_residual_ = 8.0;
  double gnss_max_age_ = 2.5;
  double gnss_max_horizontal_std_ = 2.0;
  int gnss_min_status_ = 0;
  bool gnss_use_elevation_ = false;
  int gnss_correction_count_ = 0;
  bool gnss_auto_recovery_enable_ = true;
  double gnss_auto_recovery_retry_seconds_ = 5.0;
  bool gnss_use_heading_ = false;
  double gnss_heading_offset_param_rad_ = 0.0;
  double gnss_heading_offset_rad_ = 0.0;
  double gnss_heading_max_std_deg_ = 5.0;
  double gnss_heading_min_baseline_m_ = 0.20;
  double gnss_heading_max_age_ = 1.5;
  double gnss_heading_filter_tau_s_ = 0.15;
  float filtered_rtk_yaw_ = 0.0f;
  bool rtk_yaw_slew_initialized_ = false;
  rclcpp::Time last_rtk_yaw_slew_time_{0, 0, RCL_ROS_TIME};
  std::mutex gnss_heading_mutex_;
  bool has_gnss_heading_ = false;
  double latest_gnss_heading_deg_ = 0.0;
  double latest_gnss_heading_std_deg_ = std::numeric_limits<double>::infinity();
  double latest_gnss_heading_baseline_m_ = 0.0;
  int latest_gnss_heading_status_ = -1;
  int latest_gnss_heading_type_ = 0;
  rclcpp::Time latest_gnss_heading_receive_time_{0, 0, RCL_ROS_TIME};
  int64_t latest_gnss_heading_stamp_ns_ = 0;
  int64_t last_rtk_heading_fused_stamp_ns_ = 0;
  int64_t last_gnss_position_fused_stamp_ns_ = 0;
  bool rtk_heading_fused_this_frame_ = false;
  bool rtk_position_fused_this_frame_ = false;
  std::chrono::steady_clock::time_point last_gnss_recovery_attempt_{};
  bool gnss_recovery_seed_pending_ = false;
  bool source_arbiter_enable_ = true;
  bool prefer_fixed_rtk_ = true;
  int rtk_primary_promote_samples_ = 2;
  int rtk_primary_demote_samples_ = 3;
  int rtk_auto_primary_good_frames_ = 0;
  int rtk_auto_primary_bad_frames_ = 0;
  bool rtk_auto_primary_latched_ = false;
  bool lidar_matching_paused_for_rtk_ = false;
  std::string preferred_source_ = "ndt";
  std::string active_source_ = "unavailable";
  std::string motion_phase_ = "stationary";
  bool bridge_active_ = false;
  rclcpp::Time bridge_start_time_{0, 0, RCL_ROS_TIME};
  double bridge_distance_m_ = 0.0;
  double bridge_max_distance_m_ = 10.0;
  double bridge_max_seconds_ = 20.0;
  double bridge_max_horizontal_sigma_m_ = 0.8;
  double bridge_max_yaw_sigma_rad_ = 15.0 * M_PI / 180.0;
  double bridge_translation_variance_per_m_ = 0.0025;
  double bridge_max_odom_speed_mps_ = 1.5;
  double bridge_max_odom_yaw_rate_rps_ = 2.0;
  int absolute_recovery_samples_ = 3;
  int moving_ndt_stride_ = 5;
  int ndt_failure_hysteresis_frames_ = 3;
  int ndt_unhealthy_frame_count_ = 0;
  uint64_t ndt_frame_counter_ = 0;
  bool last_ndt_healthy_ = false;
  bool last_ndt_status_healthy_ = false;
  double last_ndt_score_ = std::numeric_limits<double>::infinity();
  rclcpp::Time last_ndt_update_time_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3f last_rtk_map_position_ = Eigen::Vector3f::Zero();
  double last_rtk_map_yaw_ = 0.0;
  int absolute_stable_count_ = 0;
  bool absolute_stable_ = false;
  std::string stable_source_;
  std::string bridge_rejection_reason_;
  std::string odom_time_source_ = "not_used";
  int64_t last_absolute_observation_stamp_ns_ = 0;
  
  // transformation matrices 
  Eigen::Matrix3f init_rotation_matrix_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f gravity_transform_    = Eigen::Matrix4f::Identity();
  
  // Global localization and pose management
  std::shared_ptr<GlobalLocalization> global_localization_ptr_;
  ScanContextDatabase scan_context_db_;
  bool use_scan_context_ = false;
  int scan_context_top_k_ = 5;
  int scan_context_max_seeds_ = 1;
  double scan_context_max_distance_ = 0.0;
  /// Candidate the next gate firing starts from. Advancing it across firings is what
  /// makes the existing rearm walk the list instead of re-verifying a rejected seed.
  std::size_t scan_context_cursor_ = 0;
  // Pose caching and change detection
  Eigen::Vector3f last_init_pos_     = Eigen::Vector3f::Zero();
  Eigen::Quaternionf last_init_quat_ = Eigen::Quaternionf::Identity();
  bool has_set_init_pose_            = false;
  std::string last_pose_source_      = "none";
  // Configuration parameters for initial pose
  bool specify_init_pose_ = true;
  double init_pos_x_ = 0.0;
  double init_pos_y_ = 0.0;
  double init_pos_z_ = 0.0;
  double init_ori_w_ = 1.0;
  double init_ori_x_ = 0.0;
  double init_ori_y_ = 0.0;
  double init_ori_z_ = 0.0;
  // Global localization parameters
  bool use_global_localization_init_ = true;
  float init_pose_change_threshold_ = 0.01f;      
  float init_quat_change_threshold_ = 0.01f;      
  float global_localization_timeout_ = 10.0f;     
  // Global localization state
  bool global_localization_in_progress_ = false;
  rclcpp::Time global_localization_start_time_;
  bool gl_once_gate_ = true;
  int runtime_relocalization_failure_threshold_ = 10;
  double runtime_relocalization_retry_seconds_ = 5.0;
  std::chrono::steady_clock::time_point last_global_localization_attempt_{};
  int consecutive_match_failures_ = 0;
  bool runtime_relocalization_attempted_ = false;
  // Sensor data validity tracking
  rclcpp::Time last_lidar_data_time_;
  rclcpp::Time last_imu_data_time_;
  std::deque<bool> lidar_status_buffer_;
  std::deque<bool> imu_status_buffer_;
  int min_valid_count_;      
  int buffer_size_;          
  double sensor_timeout_threshold_ = 1.0; 

  // Store previous state for extrapolation
  Eigen::Vector3f last_velocity_{0.0, 0.0, 0.0};           // Previous velocity
  Eigen::Vector3f last_angular_velocity_{0.0, 0.0, 0.0};   // Previous angular velocity
  Eigen::Matrix4f last_pose_{Eigen::Matrix4f::Identity()};  // Previous pose matrix
  rclcpp::Time last_valid_pose_time_;                        // Time of last valid pose
  bool has_valid_pose_history_ = false;                      // Whether valid pose history exists
  
  // Store latest IMU data for extrapolation
  Eigen::Vector3f latest_angular_velocity_{0.0, 0.0, 0.0}; // Latest IMU angular velocity
  
  // Confidence management
  double current_confidence_ = 1.0;                    // Current confidence
  rclcpp::Time last_confidence_update_time_;           // Last confidence update time
  double confidence_decay_rate_ = 0.1;                 // Confidence decay rate (per second)
  bool is_extrapolating_ = false;                      // Whether currently extrapolating
  
  int log_counter_ = 0;                                // Log counter
  int log_interval_ = 10;                              // Log printing interval (print every 10 times)

  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::Registration<PointT, PointT>::Ptr refine_registration_;
  pcl::PointCloud<PointT>::Ptr global_map_points_ptr_;
  pcl::VoxelGrid<PointT>::Ptr  voxel_filter_ptr_ = pcl::VoxelGrid<PointT>::Ptr(new pcl::VoxelGrid<PointT>());
  pcl::PointCloud<PointT>::Ptr raw_points_ptr_ = nullptr;  ///< Raw point cloud pointer.
  std::unique_ptr<fast_gicp::FastGICP<PointT, PointT>> lidar_odom_registration_;
  pcl::VoxelGrid<PointT> lidar_odom_voxel_filter_;
  pcl::PointCloud<PointT>::Ptr previous_lidar_odom_cloud_;
  Eigen::Matrix4f previous_lidar_imu_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f lidar_odom_pose_ = Eigen::Matrix4f::Identity();
  const localization::PoseEstimator* lidar_odom_estimator_instance_ = nullptr;
  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<localization::PoseEstimator> pose_estimator;

  rclcpp::Service<robots_dog_msgs::srv::LocalizationState>::SharedPtr localization_state_srv_;
  rclcpp::Service<robots_dog_msgs::srv::LoadMap>::SharedPtr load_map_service_ptr_;
  // Parameters
  double cool_time_duration;
  std::string reg_method;
  std::string ndt_neighbor_search_method;
  double ndt_neighbor_search_radius;
  double ndt_resolution;
  bool enable_robot_odometry_prediction;
  bool enable_lidar_odometry_prediction_ = false;
  bool enable_lio_primary_ = false;
  std::string lio_odom_topic_ = "/odom/lio_odom";
  double lio_max_age_s_ = 0.30;
  float lio_max_step_m_ = 1.50f;
  float lio_xy_variance_ = 0.0025f;
  float lio_z_variance_ = 0.010f;
  float lio_orientation_variance_ = 0.0004f;
  float lio_drift_xy_m_ = 0.30f;
  float lio_drift_yaw_rad_ = 5.0f * static_cast<float>(M_PI) / 180.0f;
  int lio_drift_hysteresis_frames_ = 3;
  float lio_stable_xy_tolerance_m_ = 0.10f;
  float lio_stable_yaw_tolerance_rad_ = 2.0f * static_cast<float>(M_PI) / 180.0f;
  float lio_rearm_xy_m_ = 0.20f;
  float lio_suppressed_covariance_scale_ = 100.0f;
  float lio_max_correction_jump_m_ = 1.50f;
  float lio_correct_xy_variance_ = 0.010f;
  float lio_correct_z_variance_ = 0.020f;
  float lio_correct_orientation_variance_ = 0.001f;
  mutable std::mutex lio_odom_mutex_;
  nav_msgs::msg::Odometry latest_lio_odom_;
  rclcpp::Time latest_lio_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool has_lio_odom_ = false;
  bool lio_anchor_valid_ = false;
  bool lio_has_previous_pose_ = false;
  bool lio_corrected_this_frame_ = false;
  AuxiliaryDriftGate ndt_drift_gate_;
  AuxiliaryDriftGate rtk_drift_gate_;
  Eigen::Isometry3f lio_map_T_lio_ = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f previous_lio_pose_ = Eigen::Isometry3f::Identity();
  rclcpp::Time last_lio_observation_stamp_{0, 0, RCL_ROS_TIME};
  float lidar_odom_voxel_size_ = 0.40f;
  float lidar_odom_max_correspondence_distance_ = 1.00f;
  float lidar_odom_max_fitness_score_ = 0.50f;
  float ndt_max_fitness_score_ = 0.50f;
  bool scan_matching_refine_enable_ = true;
  float scan_matching_local_map_xy_radius_ = 18.0f;
  float scan_matching_local_map_z_radius_ = 4.0f;
  int scan_matching_min_local_map_points_ = 400;
  double scan_matching_vgicp_resolution_ = 0.50;
  float scan_matching_max_correspondence_distance_ = 1.00f;
  int scan_matching_max_iterations_ = 20;
  int scan_matching_num_threads_ = 2;
  float scan_matching_coarse_max_fitness_score_ = 2.00f;
  float lidar_odom_max_translation_per_scan_ = 0.80f;
  float lidar_odom_max_rotation_per_scan_rad_ = 0.70f;
  size_t lidar_odom_min_points_ = 200;
  int lidar_odom_num_threads_ = 2;
  std::mutex robot_odom_mutex_;
  Eigen::Matrix4f latest_robot_odom_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f previous_robot_odom_ = Eigen::Matrix4f::Identity();
  rclcpp::Time latest_robot_odom_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_robot_odom_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_robot_odom_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_robot_odom_source_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t latest_robot_odom_sequence_ = 0;
  uint64_t consumed_robot_odom_sequence_ = 0;
  bool odom_prediction_initialized_ = false;
  int imu_data_filter_num_ = 5;  // Number of IMU data points to filter.

  bool is_init_success_ = false;
  // Tracks a verified NDT anchor for the current initialization cycle. Generic
  // pose history may contain an extrapolated/stale pose and is not suitable
  // for deciding whether the first NDT acquisition is a discontinuity.
  bool has_trusted_ndt_pose_ = false;
  bool is_use_map_coord_ = true;
  int localization_state_ = 0;   // 0: not init, 1: initing, 2: init success, 3: continuous localization, 4: continuous localization failed
  sensor_msgs::msg::Imu::SharedPtr correct_imu_data_ptr_;
  std::atomic<bool> update_map_flag_{false};
  float globalmap_voxel_size_ = 0.3;
  float points_voxel_filter_size_ = 0.2;
  int last_timeout_;
  std::atomic<ModeState>  mode_state_ = ModeState::INIT;
};
}  // namespace localization

int main(int argc, char** argv) {
  omp_set_num_threads(6);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<localization::HdlLocalizationNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
