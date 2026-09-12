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
#include <array>
#include <chrono>
#include <functional>
#include <vector>
#include <utility>
#include <numeric>
#include <condition_variable>
#include <cstdint>
#include <optional>
#include <iterator>
#include <thread>

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
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <localization/pose_estimator.hpp>
#include <localization/static_imu_init.hpp>
#include <localization/mode_state.h>
#include <localization/global_localization.hpp>
#include <localization/scan_context_db.hpp>
#include <localization/relocalization_geometry.hpp>
#include <localization/point_cloud_scheduler.hpp>
#include <localization/correction_cooldown_gate.hpp>
#include <localization/correction_policy.hpp>
#include <localization/global_relocalization_policy.hpp>
#include <localization/lio_motion_guard.hpp>
#include <localization/rtk_primary_policy.hpp>
#include <localization/anchor_ukf.hpp>

#include <localization/msg/scan_matching_status.hpp>
#include <robots_dog_msgs/srv/load_map.hpp>
#include <robots_dog_msgs/srv/localization_state.hpp>
#include <robots_dog_msgs/srv/control_localization_correction.hpp>
#include <robots_dog_msgs/srv/set_localization_fusion_profile.hpp>
#include <robots_dog_msgs/msg/localization.hpp>
#include <robots_dog_msgs/msg/uni_rtk_pvh.hpp>

using namespace std;

namespace localization {

namespace {

using SteadyClock = std::chrono::steady_clock;

std::int64_t steadyNowNanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    SteadyClock::now().time_since_epoch()).count();
}

enum PointCloudPerfStage : std::size_t {
  kLockWait = 0,
  kRosConversion,
  kPreprocess,
  kVoxelTransform,
  kGlobalRelocalization,
  kLidarOdometry,
  kLioObservation,
  kNdt,
  kLocalMap,
  kVgicp,
  kOutput,
  kTotal,
  kPointCloudPerfStageCount
};

struct PointCloudPerfSample {
  std::array<double, kPointCloudPerfStageCount> stage_ms{};
  double cloud_age_ms = 0.0;
  std::string phase = "stationary";
  bool heavy = false;
  bool run_ndt = false;
  bool stale = false;
};

struct PointCloudPerfWindow {
  std::vector<PointCloudPerfSample> samples;
  std::uint64_t heavy_count = 0;
  std::uint64_t light_count = 0;
  std::uint64_t stale_count = 0;
};

template<typename Callable>
class ScopeExit {
public:
  explicit ScopeExit(Callable&& callable) : callable_(std::forward<Callable>(callable)) {}
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
  ~ScopeExit() { callable_(); }

private:
  Callable callable_;
};

template<typename Callable>
ScopeExit<Callable> makeScopeExit(Callable&& callable) {
  return ScopeExit<Callable>(std::forward<Callable>(callable));
}

double quantile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = std::clamp(fraction, 0.0, 1.0) * (values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

float normalizedYaw(float yaw) {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

float yawFromRotation(const Eigen::Matrix3f& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

float yawDifference(float lhs, float rhs) {
  return normalizedYaw(lhs - rhs);
}

}  // namespace

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
      0.001, declare_parameter<double>("ndt_max_fitness_score", 0.40)));
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
    scan_matching_refine_policy_.skip_ndt_score = static_cast<float>(std::clamp(
      declare_parameter<double>("scan_matching.refine_skip_ndt_score", 0.15),
      0.001, static_cast<double>(ndt_max_fitness_score_)));
    scan_matching_refine_policy_.zero_inlier_skip_count = static_cast<int>(std::max<int64_t>(
      0, declare_parameter<int>("scan_matching.refine_zero_inlier_skip_count", 2)));
    scan_matching_refine_policy_.min_improvement_ratio = static_cast<float>(std::clamp(
      declare_parameter<double>("scan_matching.refine_min_improvement_ratio", 0.05),
      0.0, 0.90));
    scan_matching_refine_policy_.max_translation_disagreement_m = static_cast<float>(std::max(
      0.05, declare_parameter<double>(
        "scan_matching.refine_max_translation_disagreement_m", 0.30)));
    scan_matching_refine_policy_.max_rotation_disagreement_rad = static_cast<float>(std::max(
      0.01, declare_parameter<double>(
        "scan_matching.refine_max_rotation_disagreement_deg", 5.0) * M_PI / 180.0));
    scan_preprocess_min_range_m_ = std::max(
      0.0, declare_parameter<double>("scan_preprocessing.min_range", 0.50));
    scan_preprocess_max_range_m_ = std::max(
      scan_preprocess_min_range_m_ + 0.10,
      declare_parameter<double>("scan_preprocessing.max_range", 10.0));
    scan_preprocess_fov_degree_ = std::clamp(
      declare_parameter<double>("scan_preprocessing.fov_degree", 240.0), 1.0, 360.0);
    scan_ground_filter_enable_ = declare_parameter<bool>(
      "scan_preprocessing.ground_filter.enable", true);
    scan_ground_distance_threshold_m_ = std::clamp(
      declare_parameter<double>("scan_preprocessing.ground_filter.distance_threshold_m", 0.08),
      0.01, 0.30);
    scan_ground_max_tilt_deg_ = std::clamp(
      declare_parameter<double>("scan_preprocessing.ground_filter.max_tilt_deg", 20.0),
      1.0, 45.0);
    scan_ground_min_inliers_ = static_cast<int>(std::max<int64_t>(
      20, declare_parameter<int>("scan_preprocessing.ground_filter.min_inliers", 80)));
    scan_ground_min_sensor_height_m_ = std::max(
      0.05, declare_parameter<double>("scan_preprocessing.ground_filter.min_sensor_height_m", 0.20));
    scan_ground_max_sensor_height_m_ = std::max(
      scan_ground_min_sensor_height_m_ + 0.10,
      declare_parameter<double>("scan_preprocessing.ground_filter.max_sensor_height_m", 1.20));
    scan_ground_clearance_m_ = std::clamp(
      declare_parameter<double>("scan_preprocessing.ground_filter.clearance_m", 0.15),
      0.0, 0.30);
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
    lio_max_yaw_step_rad_ = static_cast<float>(std::max(
      1.0, declare_parameter<double>("lio_primary.max_yaw_step_deg", 30.0))
      * M_PI / 180.0);
    lio_max_yaw_rate_radps_ = static_cast<float>(std::max(
      1.0, declare_parameter<double>("lio_primary.max_yaw_rate_degps", 60.0))
      * M_PI / 180.0);
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
    lio_stable_confirmation_frames_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("lio_primary.stable_confirmation_frames", 3)));
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
    lio_max_correction_yaw_rad_ = static_cast<float>(std::max(
      1.0, declare_parameter<double>("lio_primary.max_correction_yaw_deg", 30.0))
        * M_PI / 180.0);
    lio_correct_xy_variance_ = static_cast<float>(std::max(
      1e-4, declare_parameter<double>("lio_primary.correct_xy_variance", 0.010)));
    lio_correct_z_variance_ = static_cast<float>(std::max(
      1e-4, declare_parameter<double>("lio_primary.correct_z_variance", 0.020)));
    lio_correct_orientation_variance_ = static_cast<float>(std::max(
      1e-5, declare_parameter<double>("lio_primary.correct_orientation_variance", 0.001)));
    lio_dynamic_covariance_enable_ = declare_parameter<bool>(
      "lio_primary.dynamic_covariance.enable", true);
    lio_dynamic_ndt_good_score_ = static_cast<float>(std::clamp(
      declare_parameter<double>("lio_primary.dynamic_covariance.ndt_good_score", 0.10),
      0.0, static_cast<double>(ndt_max_fitness_score_)));
    lio_dynamic_vgicp_good_score_ = static_cast<float>(std::clamp(
      declare_parameter<double>("lio_primary.dynamic_covariance.vgicp_good_score", 0.05),
      0.0, static_cast<double>(ndt_max_fitness_score_)));
    lio_dynamic_good_inlier_fraction_ = static_cast<float>(std::clamp(
      declare_parameter<double>("lio_primary.dynamic_covariance.good_inlier_fraction", 0.50),
      0.05, 1.0));
    lio_dynamic_xy_variance_max_ = static_cast<float>(std::max(
      static_cast<double>(lio_correct_xy_variance_),
      declare_parameter<double>("lio_primary.dynamic_covariance.xy_variance_max", 0.25)));
    lio_dynamic_z_variance_max_ = static_cast<float>(std::max(
      static_cast<double>(lio_correct_z_variance_),
      declare_parameter<double>("lio_primary.dynamic_covariance.z_variance_max", 0.50)));
    lio_dynamic_orientation_variance_max_ = static_cast<float>(std::max(
      static_cast<double>(lio_correct_orientation_variance_),
      declare_parameter<double>("lio_primary.dynamic_covariance.orientation_variance_max", 0.05)));
    lio_correction_translation_rate_mps_ = static_cast<float>(std::max(
      0.01, declare_parameter<double>("lio_primary.correction_smoothing.translation_rate_mps", 0.25)));
    lio_correction_rotation_rate_radps_ = static_cast<float>(std::max(
      0.01, declare_parameter<double>("lio_primary.correction_smoothing.rotation_rate_degps", 8.0)
        * M_PI / 180.0));
    lio_correction_completion_translation_m_ = static_cast<float>(std::max(
      0.001, declare_parameter<double>("lio_primary.correction_smoothing.completion_translation_m", 0.01)));
    lio_correction_completion_rotation_rad_ = static_cast<float>(std::max(
      0.001, declare_parameter<double>("lio_primary.correction_smoothing.completion_rotation_deg", 0.25)
        * M_PI / 180.0));
    lio_correction_min_interval_s_ = std::max(
      0.0, declare_parameter<double>("lio_primary.correction_gate.min_interval_seconds", 3.0));
    lio_correction_post_suppression_s_ = std::max(
      0.0, declare_parameter<double>("lio_primary.correction_gate.post_suppression_seconds", 3.0));
    lio_correction_cooldown_gate_.configure(
      lio_correction_min_interval_s_, lio_correction_post_suppression_s_);
    // Fixed RTK that stays self-consistent for this window is fully trusted for
    // LIO anchor correction, even when LIO has drifted farther than
    // max_correction_jump_m. Instantaneous RTK source jumps still reject.
    rtk_trust_stable_window_s_ = std::max(
      0.5, declare_parameter<double>("lio_primary.rtk_trust.stable_window_seconds", 1.0));
    rtk_trust_stable_span_m_ = static_cast<float>(std::max(
      0.05, declare_parameter<double>("lio_primary.rtk_trust.stable_span_m", 0.35)));
    rtk_trusted_correction_translation_rate_mps_ = static_cast<float>(std::max(
      static_cast<double>(lio_correction_translation_rate_mps_),
      declare_parameter<double>(
        "lio_primary.rtk_trust.trusted_correction_translation_rate_mps", 1.0)));
    rtk_trusted_correction_rotation_rate_radps_ = static_cast<float>(std::max(
      static_cast<double>(lio_correction_rotation_rate_radps_),
      declare_parameter<double>(
        "lio_primary.rtk_trust.trusted_correction_rotation_rate_degps", 30.0)
        * M_PI / 180.0));
    prefer_fixed_rtk_for_correction_ = declare_parameter<bool>(
      "lio_primary.rtk_trust.prefer_fixed_for_correction", true);
    ukf_high_quality_ndt_score_ = static_cast<float>(std::clamp(
      declare_parameter<double>("lio_primary.ukf_fusion.high_quality_ndt_score", 0.10),
      0.0, static_cast<double>(ndt_max_fitness_score_)));
    ukf_float_max_residual_m_ = static_cast<float>(std::max(
      0.0, declare_parameter<double>("lio_primary.ukf_fusion.float_max_residual_m", 0.40)));

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
    anchor_yaw_variance_per_rad_ = std::max(
      0.0, declare_parameter<double>("lio_primary.anchor_ukf.yaw_variance_per_rad", 0.01));
    lio_hold_covariance_scale_ = std::max(
      1.0, declare_parameter<double>("lio_primary.self_healing.lio_hold_covariance_scale", 4.0));
    balanced_observation_variance_scale_ = std::max(
      1.0,
      declare_parameter<double>(
        "lio_primary.self_healing.balanced_observation_variance_scale", 4.0));
    lio_pose_history_seconds_ = std::max(
      0.5, declare_parameter<double>("lio_primary.anchor_ukf.pose_history_seconds", 2.0));
    lio_observation_sync_tolerance_s_ = std::max(
      0.01, declare_parameter<double>("lio_primary.anchor_ukf.sync_tolerance_seconds", 0.20));
    bridge_max_odom_speed_mps_ = declare_parameter<double>("source_arbiter.max_odom_speed_mps", 1.5);
    bridge_max_odom_yaw_rate_rps_ = declare_parameter<double>("source_arbiter.max_odom_yaw_rate_rps", 2.0);
    absolute_recovery_samples_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.absolute_recovery_samples", 3)));
    moving_ndt_stride_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.moving_ndt_stride", 5)));
    stationary_ndt_stride_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.stationary_ndt_stride", 5)));
    initialization_ndt_stride_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.initialization_ndt_stride", 2)));
    stable_ndt_max_rate_hz_ = std::max(
      0.0, declare_parameter<double>("source_arbiter.stable_ndt_max_rate_hz", 2.0));
    recovery_ndt_max_rate_hz_ = std::max(
      0.0, declare_parameter<double>("source_arbiter.recovery_ndt_max_rate_hz", 5.0));
    point_cloud_max_age_s_ = std::max(
      0.0, declare_parameter<double>("point_cloud_processing.max_age_seconds", 0.20));
    point_cloud_latest_only_ = declare_parameter<bool>(
      "point_cloud_processing.latest_only", true);
    point_cloud_perf_log_interval_s_ = std::max(
      1.0, declare_parameter<double>("point_cloud_processing.performance_log_interval_seconds", 10.0));
    point_cloud_slow_callback_ms_ = std::max(
      1.0, declare_parameter<double>("point_cloud_processing.slow_callback_ms", 80.0));
    ndt_failure_hysteresis_frames_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.ndt_failure_hysteresis_frames", 3)));
    // Outdoor default: FAST-LIO stays the continuous pose; fixed RTK XY (and
    // optional heading) correct intermittently. Enabling prefer_fixed_rtk still
    // requires sustained dual-antenna heading before GPS drives the pose.
    const bool requested_rtk_primary =
      declare_parameter<bool>("source_arbiter.prefer_fixed_rtk", false);
    prefer_fixed_rtk_ = false;
    if (requested_rtk_primary) {
      RCLCPP_WARN(
        get_logger(),
        "source_arbiter.prefer_fixed_rtk is deprecated and ignored; "
        "FAST-LIO remains the only continuous navigation source");
    }
    rtk_primary_promote_samples_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.rtk_primary_promote_samples", 20)));
    rtk_primary_demote_samples_ = static_cast<int>(std::max<int64_t>(
      1, declare_parameter<int>("source_arbiter.rtk_primary_demote_samples", 8)));
    rtk_primary_handoff_suppress_s_ = std::max(
      0.0, declare_parameter<double>("source_arbiter.rtk_primary_handoff_suppress_seconds", 2.5));
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
    declare_parameter<int>("init_match_count_threshold", 3);
    get_parameter("init_match_count_threshold", init_match_count_threshold_);
    declare_parameter<float>("init_match_score_threshold", 0.15);
    get_parameter("init_match_score_threshold", init_match_score_threshold_);
    declare_parameter<float>("init_match_optimal_score_threshold", 0.01f);
    get_parameter("init_match_optimal_score_threshold", init_match_optimal_score_threshold_);
    init_match_optimal_score_threshold_ = std::clamp(
      init_match_optimal_score_threshold_, 0.0001f, init_match_score_threshold_);
    init_match_min_inlier_fraction_ = static_cast<float>(std::clamp(
      declare_parameter<double>("init_match_min_inlier_fraction", 0.50), 0.0, 1.0));
    init_match_stable_xy_m_ = static_cast<float>(std::max(
      0.01, declare_parameter<double>("init_match_stable_xy_m", 0.10)));
    init_match_stable_yaw_rad_ = static_cast<float>(std::max(
      0.1, declare_parameter<double>("init_match_stable_yaw_deg", 2.0)) * M_PI / 180.0);
    init_match_max_seed_xy_m_ = static_cast<float>(std::max(
      0.10, declare_parameter<double>("init_match_max_seed_xy_m", 1.50)));
    init_match_max_seed_yaw_rad_ = static_cast<float>(std::max(
      1.0, declare_parameter<double>("init_match_max_seed_yaw_deg", 30.0)) * M_PI / 180.0);
    
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
    initial_pose_relocalization_settle_s_ = std::max(
      0.0, declare_parameter<double>("relocalization.initial_pose_settle_seconds", 0.75));
    map_relocalization_settle_s_ = std::max(
      0.0, declare_parameter<double>("relocalization.map_settle_seconds", 2.0));

    // Scan Context relocalization has an explicit rollout state. The legacy boolean is
    // retained as a compatibility alias for active, but a non-disabled mode wins.
    declare_parameter<bool>("relocalization.use_scan_context", false);
    get_parameter("relocalization.use_scan_context", use_scan_context_);
    scan_context_runtime_mode_ = declare_parameter<std::string>(
      "relocalization.scan_context_runtime_mode", "disabled");
    if (scan_context_runtime_mode_ != "disabled" &&
        scan_context_runtime_mode_ != "shadow" &&
        scan_context_runtime_mode_ != "active") {
      RCLCPP_WARN(get_logger(),
        "Unknown relocalization.scan_context_runtime_mode=%s; forcing disabled",
        scan_context_runtime_mode_.c_str());
      scan_context_runtime_mode_ = "disabled";
    }
    if (scan_context_runtime_mode_ == "disabled" && use_scan_context_) {
      scan_context_runtime_mode_ = "active";
      RCLCPP_WARN(get_logger(),
        "relocalization.use_scan_context is deprecated; treating true as runtime_mode=active");
    }
    use_scan_context_ = scan_context_runtime_mode_ != "disabled";
    scan_context_active_map_paths_ = declare_parameter<std::vector<std::string>>(
      "relocalization.active_map_paths", std::vector<std::string>{});
    declare_parameter<int>("relocalization.top_k", 5);
    get_parameter("relocalization.top_k", scan_context_top_k_);
    scan_context_top_k_ = std::max(1, scan_context_top_k_);
    declare_parameter<int>("relocalization.max_seeds_per_attempt", 1);
    get_parameter("relocalization.max_seeds_per_attempt", scan_context_max_seeds_);
    scan_context_max_seeds_ = std::max(1, scan_context_max_seeds_);
    scan_context_prefilter_candidates_ = static_cast<int>(std::max<std::int64_t>(
      0, declare_parameter<int>("relocalization.prefilter_candidates", 60)));
    scan_context_target_half_window_ = static_cast<int>(std::max<std::int64_t>(
      0, declare_parameter<int>("relocalization.target_half_window", 1)));
    scan_context_yaw_neighbors_ = static_cast<int>(std::max<std::int64_t>(
      0, declare_parameter<int>("relocalization.yaw_neighbors", 2)));
    // Offline leave-one-out over 103 recorded maps put the descriptor distance of correct
    // retrievals (p90 0.33) on top of that of wrong ones (median 0.26), so a cutoff here
    // discards good candidates without excluding bad ones. 0 disables it and leaves the
    // ICP verification as the only gate, which is the one that actually discriminates.
    declare_parameter<double>("relocalization.max_descriptor_distance", 0.0);
    get_parameter("relocalization.max_descriptor_distance", scan_context_max_distance_);
    relocalization_geometry_config_.voxel_size_m = static_cast<float>(std::max(
      0.05, declare_parameter<double>("relocalization.geometry.voxel_size_m", 0.20)));
    relocalization_geometry_config_.max_correspondence_distance_m = static_cast<float>(
      std::max(0.10, declare_parameter<double>(
        "relocalization.geometry.max_correspondence_distance_m", 1.0)));
    relocalization_geometry_config_.max_iterations = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<int>("relocalization.geometry.max_iterations", 30)));
    relocalization_geometry_config_.num_threads = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<int>("relocalization.geometry.num_threads", 2)));
    relocalization_geometry_config_.min_points = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<int>("relocalization.geometry.min_points", 200)));
    relocalization_geometry_config_.min_inliers = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<int>("relocalization.geometry.min_inliers", 500)));
    relocalization_geometry_config_.min_bidirectional_overlap = std::clamp(
      declare_parameter<double>("relocalization.geometry.min_bidirectional_overlap", 0.45),
      0.0, 1.0);
    relocalization_geometry_config_.max_rmse_m = std::max(
      0.01, declare_parameter<double>("relocalization.geometry.max_rmse_m", 0.25));

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
    rclcpp::QoS point_cloud_qos = point_cloud_latest_only_
      ? rclcpp::QoS(rclcpp::SensorDataQoS().keep_last(1))
      : rclcpp::QoS(5);
    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, point_cloud_qos,
      std::bind(&HdlLocalizationNode::points_callback, this, std::placeholders::_1));
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
      // NDT/VGICP can occupy the point-cloud callback for longer than the
      // 300 ms LIO freshness gate during recovery. Keep LIO reception in a
      // separate callback group so a heavy registration cannot make a healthy
      // 10 Hz odometry stream look stale.
      lio_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions lio_options;
      lio_options.callback_group = lio_callback_group_;
      lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        lio_odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&HdlLocalizationNode::lio_odom_callback, this, std::placeholders::_1),
        lio_options);
      RCLCPP_INFO(
        get_logger(),
        "FAST-LIO continuous source enabled (RTK fixed+heading overrides to GPS): topic=%s "
        "max_age=%.2fs drift_xy=%.2fm "
        "drift_yaw=%.1fdeg motion_guard_10hz=[%.1fdeg/sample, %.1fdeg/s] hysteresis=%d "
        "absolute_cap=%.2fm smoothing=[%.2fm/s, %.1fdeg/s] "
        "dynamic_covariance=%s (NDT/RTK intermittent corrections while RTK is poor)",
        lio_odom_topic_.c_str(), lio_max_age_s_, lio_drift_xy_m_,
        lio_drift_yaw_rad_ * 180.0 / M_PI,
        lio_max_yaw_step_rad_ * 180.0 / M_PI,
        lio_max_yaw_rate_radps_ * 180.0 / M_PI, lio_drift_hysteresis_frames_,
        lio_max_correction_jump_m_, lio_correction_translation_rate_mps_,
        lio_correction_rotation_rate_radps_ * 180.0 / M_PI,
        lio_dynamic_covariance_enable_ ? "true" : "false");
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
    global_relocalize_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/global_relocalize",
      std::bind(&HdlLocalizationNode::global_relocalize_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    localization_policy_sub_ = create_subscription<std_msgs::msg::String>(
      "/localization/policy", 10,
      std::bind(&HdlLocalizationNode::localization_policy_callback, this, std::placeholders::_1));
    control_localization_correction_service_ =
      create_service<robots_dog_msgs::srv::ControlLocalizationCorrection>(
        "/localization/control_correction",
        std::bind(&HdlLocalizationNode::control_localization_correction_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    set_localization_fusion_profile_service_ =
      create_service<robots_dog_msgs::srv::SetLocalizationFusionProfile>(
        "/localization/set_fusion_profile",
        std::bind(&HdlLocalizationNode::set_localization_fusion_profile_callback, this,
          std::placeholders::_1, std::placeholders::_2));
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
    RCLCPP_INFO(
      get_logger(),
      "Localization scan preprocessing: range=[%.2f, %.2f]m fov=%.1fdeg ground=%s "
      "threshold=%.2fm clearance=%.2fm",
      scan_preprocess_min_range_m_, scan_preprocess_max_range_m_,
      scan_preprocess_fov_degree_, scan_ground_filter_enable_ ? "enabled" : "disabled",
      scan_ground_distance_threshold_m_, scan_ground_clearance_m_);
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
    global_relocalization_worker_ = std::thread(
      &HdlLocalizationNode::globalRelocalizationWorkerLoop, this);
  }

  ~HdlLocalizationNode() override {
    {
      std::lock_guard<std::mutex> lock(global_relocalization_job_mutex_);
      global_relocalization_worker_stop_ = true;
      pending_global_relocalization_job_.reset();
    }
    global_relocalization_job_cv_.notify_all();
    if (global_relocalization_worker_.joinable()) {
      global_relocalization_worker_.join();
    }
  }

private:
  struct GlobalRelocalizationJob {
    std::uint64_t generation = 0;
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<PointT>::ConstPtr map;
    Eigen::Matrix4d fallback_seed = Eigen::Matrix4d::Identity();
    std::vector<std::pair<std::string, Eigen::Matrix4d>> progressive_seeds;
    bool use_scan_context = false;
    bool apply_scan_context = false;
    bool allow_fallback = true;
  };

  struct GlobalRelocalizationResult {
    std::uint64_t generation = 0;
    bool success = false;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    double elapsed_ms = 0.0;
    std::string source = "none";
    bool candidate_accepted = false;
    int candidate_keyframe = -1;
    double candidate_distance = 0.0;
    double candidate_yaw_deg = 0.0;
    double candidate_rmse_m = 0.0;
    double candidate_overlap = 0.0;
    std::string rejection_reason = "no_candidate";
    std::string seed_source = "none";
    int attempts = 0;
  };

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
  float init_match_optimal_score_threshold_ = 0.01f;
  // Initial pose initialization state variables
  int init_match_count_ = 0;
  // A decimated initialization must never count the same previous NDT result
  // more than once on intervening lightweight callbacks.
  bool init_match_result_pending_ = false;
  float init_match_min_inlier_fraction_ = 0.50f;
  float init_match_stable_xy_m_ = 0.10f;
  float init_match_stable_yaw_rad_ = 2.0f * M_PI / 180.0f;
  float init_match_max_seed_xy_m_ = 1.50f;
  float init_match_max_seed_yaw_rad_ = 30.0f * M_PI / 180.0f;
  bool has_previous_init_match_pose_ = false;
  Eigen::Vector2f previous_init_match_xy_ = Eigen::Vector2f::Zero();
  float previous_init_match_yaw_ = 0.0f;
  float initialization_seed_yaw_ = 0.0f;
  float initialization_match_yaw_ = 0.0f;
  float initialization_yaw_correction_rad_ = 0.0f;
  float initialization_position_correction_m_ = 0.0f;
  bool initialization_verified_ = false;
  std::string initialization_state_ = "uninitialized";

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
    init_match_result_pending_ = false;
    const bool calibrated = static_imu_init_.InitSuccess();
    const Eigen::Vector3d bias_gyro =
      calibrated ? static_imu_init_.GetInitBg() : Eigen::Vector3d::Zero();
    // Estimators built before the calibration completes still need the write-back.
    imu_bias_seeded_ = calibrated;
    if (calibrated) {
      RCLCPP_INFO(get_logger(),
        "PoseEstimator seeded with gyro bias [%.6f, %.6f, %.6f] rad/s (|bg| = %.4f deg/s)",
        bias_gyro.x(), bias_gyro.y(), bias_gyro.z(), bias_gyro.norm() * 180.0 / M_PI);
    } else if (use_imu) {
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
    estimator->configure_refine_policy(scan_matching_refine_policy_);
    return estimator;
  }

  void resetInitializationValidation(const std::string& state = "validating") {
    init_match_count_ = 0;
    init_match_result_pending_ = false;
    has_previous_init_match_pose_ = false;
    initialization_verified_ = false;
    initialization_state_ = state;
    initialization_seed_yaw_ = yawFromRotation(last_init_quat_.toRotationMatrix());
    initialization_match_yaw_ = initialization_seed_yaw_;
    initialization_yaw_correction_rad_ = 0.0f;
    initialization_position_correction_m_ = 0.0f;
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
      "Indoor scan matching: NDT coarse + local FastVGICP refine "
      "(res=%.2fm corr=%.2fm iter=%d threads=%d stable_skip<=%.3f zero_inlier_skip=%d improve>=%.0f%%)",
      scan_matching_vgicp_resolution_,
      scan_matching_max_correspondence_distance_,
      scan_matching_max_iterations_,
      scan_matching_num_threads_, scan_matching_refine_policy_.skip_ndt_score,
      scan_matching_refine_policy_.zero_inlier_skip_count,
      scan_matching_refine_policy_.min_improvement_ratio * 100.0f);
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

  struct CorrectionNoise {
    float horizontal_variance = 0.01f;
    float vertical_variance = 0.02f;
    float orientation_variance = 0.001f;
    float quality_penalty = 0.0f;
  };

  struct PendingLioCorrection {
    bool active = false;
    Eigen::Isometry3f target_map_T_lio = Eigen::Isometry3f::Identity();
    CorrectionNoise noise;
    std::string source = "none";
    int64_t last_update_stamp_ns = 0;
    float initial_translation_m = 0.0f;
    float initial_rotation_rad = 0.0f;
    float remaining_translation_m = 0.0f;
    float remaining_rotation_rad = 0.0f;
    float translation_rate_mps = 0.25f;
    float rotation_rate_radps = 8.0f * static_cast<float>(M_PI) / 180.0f;

    void reset() {
      active = false;
      target_map_T_lio.setIdentity();
      noise = CorrectionNoise{};
      source = "none";
      last_update_stamp_ns = 0;
      initial_translation_m = 0.0f;
      initial_rotation_rad = 0.0f;
      remaining_translation_m = 0.0f;
      remaining_rotation_rad = 0.0f;
      translation_rate_mps = 0.25f;
      rotation_rate_radps = 8.0f * static_cast<float>(M_PI) / 180.0f;
    }
  };

  struct OneShotCorrection {
    bool active = false;
    std::string transaction_id;
    CorrectionPolicyMode mode = CorrectionPolicyMode::ndt;
    std::string status = "idle";
    std::string selected_source = "none";
    std::string selection_reason = "none";
    std::string reason = "none";
    int64_t started_steady_ns = 0;
    int64_t completed_steady_ns = 0;
  };

  // Shared 0.30 m / 3-frame / no-jump gate for NDT/VGICP and RTK while LIO is
  // the indoor primary observation. Neither source is fused every UKF frame.
  // RTK may additionally use trust_stable_source so a fixed, self-consistent
  // GPS solution can correct large LIO drift; only instantaneous RTK jumps reject.
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

  struct RtkStabilitySample {
    int64_t stamp_ns = 0;
    Eigen::Vector2f xy = Eigen::Vector2f::Zero();
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
    const Eigen::Isometry3f next_pose = poseFromOdometry(*msg);
    double translation_m = 0.0;
    double rotation_rad = 0.0;
    {
      std::lock_guard<std::mutex> lock(lio_odom_mutex_);
      if (has_anchor_prediction_lio_pose_) {
        const Eigen::Isometry3f delta = anchor_prediction_lio_pose_.inverse() * next_pose;
        translation_m = delta.translation().head<2>().norm();
        rotation_rad = Eigen::Quaternionf::Identity().angularDistance(
          Eigen::Quaternionf(delta.rotation()).normalized());
      }
      anchor_prediction_lio_pose_ = next_pose;
      has_anchor_prediction_lio_pose_ = next_pose.matrix().allFinite();
      latest_lio_odom_ = *msg;
      latest_lio_odom_stamp_ = rclcpp::Time(msg->header.stamp);
      has_lio_odom_ = true;
      ++lio_odom_sequence_;
      lio_pose_history_.push_back({latest_lio_odom_stamp_.nanoseconds(), next_pose});
      const int64_t horizon = latest_lio_odom_stamp_.nanoseconds() -
        static_cast<int64_t>(lio_pose_history_seconds_ * 1.0e9);
      while (lio_pose_history_.size() > 2 && lio_pose_history_.front().stamp_ns < horizon) {
        lio_pose_history_.pop_front();
      }
    }
    // Check the native 10 Hz LIO stream here, independently of NDT/VGICP.
    // The point-cloud callback can take 300-400 ms during a heavy match; using
    // its cadence made a normal continuous turn accumulate into a false
    // 12-degree "single-frame" jump. Keep sampling even while disarmed so a
    // newly anchored source starts from the latest LIO pose.
    std::optional<LioMotionGuardResult> motion_anomaly;
    const LioMotionSample current_motion{
      std::atan2(next_pose.rotation()(1, 0), next_pose.rotation()(0, 0)),
      rclcpp::Time(msg->header.stamp).nanoseconds()};
    {
      std::lock_guard<std::mutex> guard_lock(lio_motion_guard_mutex_);
      if (previous_lio_motion_sample_ && lio_anchor_valid_.load() &&
          !lio_motion_anomaly_active_.load()) {
        const auto result = evaluateLioMotion(
          *previous_lio_motion_sample_, current_motion,
          lio_max_yaw_step_rad_, lio_max_yaw_rate_radps_);
        if (result.anomaly) {
          motion_anomaly = result;
        }
      }
      previous_lio_motion_sample_ = current_motion;
    }
    if (motion_anomaly) {
      latchLioMotionAnomaly(*motion_anomaly);
    }
    if (translation_m > 0.0 || rotation_rad > 0.0) {
      std::lock_guard<std::mutex> anchor_lock(anchor_ukf_mutex_);
      anchor_ukf_.predict(
        translation_m, rotation_rad,
        bridge_translation_variance_per_m_, anchor_yaw_variance_per_rad_);
    }
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

  void beginLioHandoff(const char* source) {
    if (!enable_lio_primary_) {
      return;
    }
    std::uint64_t required_sequence = 0;
    std::uint64_t generation = 0;
    const char* handoff_source = source ? source : "absolute_pose";
    {
      std::lock_guard<std::mutex> lock(lio_odom_mutex_);
      required_sequence = lio_odom_sequence_;
    }
    {
      std::lock_guard<std::mutex> lock(lio_handoff_mutex_);
      generation = ++lio_handoff_generation_;
      lio_handoff_required_sequence_ = required_sequence;
      lio_handoff_source_ = handoff_source;
      lio_handoff_failure_reason_ = "waiting_for_fresh_lio";
      lio_handoff_pending_ = true;
    }
    active_source_ = "unavailable";
    policy_source_ready_ = false;
    absolute_stable_ = false;
    absolute_stable_count_ = 0;
    stable_source_.clear();
    RCLCPP_INFO(
      get_logger(), "FAST-LIO handoff pending generation=%llu source=%s after_lio_sequence=%llu",
      static_cast<unsigned long long>(generation), handoff_source,
      static_cast<unsigned long long>(required_sequence));
  }

  bool lioHandoffHasFreshFrame() const {
    std::uint64_t current_sequence = 0;
    {
      std::lock_guard<std::mutex> lock(lio_odom_mutex_);
      current_sequence = lio_odom_sequence_;
    }
    std::lock_guard<std::mutex> lock(lio_handoff_mutex_);
    return !lio_handoff_pending_ || current_sequence > lio_handoff_required_sequence_;
  }

  void completeLioHandoff() {
    std::lock_guard<std::mutex> lock(lio_handoff_mutex_);
    if (!lio_handoff_pending_) {
      return;
    }
    lio_handoff_pending_ = false;
    lio_handoff_failure_reason_ = "none";
    RCLCPP_INFO(
      get_logger(), "FAST-LIO handoff ready generation=%llu source=%s",
      static_cast<unsigned long long>(lio_handoff_generation_), lio_handoff_source_.c_str());
  }

  struct LioHandoffSnapshot {
    bool pending = false;
    std::uint64_t generation = 0;
    std::uint64_t required_sequence = 0;
    std::string source = "none";
    std::string failure_reason = "none";
  };

  LioHandoffSnapshot lioHandoffSnapshot() const {
    std::lock_guard<std::mutex> lock(lio_handoff_mutex_);
    return {
      lio_handoff_pending_, lio_handoff_generation_, lio_handoff_required_sequence_,
      lio_handoff_source_, lio_handoff_failure_reason_
    };
  }

  void resetLioAnchor() {
    lio_anchor_valid_.store(false);
    lio_has_previous_pose_ = false;
    lio_stable_frame_count_ = 0;
    lio_corrected_this_frame_ = false;
    pending_lio_correction_.reset();
    lio_correction_cooldown_gate_.reset();
    ndt_drift_gate_.reset();
    rtk_drift_gate_.reset();
    {
      std::lock_guard<std::mutex> anchor_lock(anchor_ukf_mutex_);
      anchor_ukf_.clear();
    }
  }

  bool currentLioPose(
      Eigen::Isometry3f& pose,
      float* horizontal_variance = nullptr,
      float* vertical_variance = nullptr,
      float* orientation_variance = nullptr,
      rclcpp::Time* odom_stamp = nullptr) const {
    std::lock_guard<std::mutex> lock(lio_odom_mutex_);
    if (!has_lio_odom_) {
      return false;
    }
    pose = poseFromOdometry(latest_lio_odom_);
    if (odom_stamp) {
      *odom_stamp = latest_lio_odom_stamp_;
    }
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

  bool lioPoseAt(const rclcpp::Time& stamp, Eigen::Isometry3f& pose) const {
    std::lock_guard<std::mutex> lock(lio_odom_mutex_);
    if (lio_pose_history_.empty()) {
      return false;
    }
    const int64_t target = stamp.nanoseconds();
    if (target <= 0) {
      pose = lio_pose_history_.back().pose;
      return pose.matrix().allFinite();
    }
    auto upper = std::lower_bound(
      lio_pose_history_.begin(), lio_pose_history_.end(), target,
      [](const StampedLioPose& item, int64_t value) { return item.stamp_ns < value; });
    if (upper == lio_pose_history_.begin()) {
      if (std::fabs(static_cast<double>(upper->stamp_ns - target)) * 1.0e-9 >
          lio_observation_sync_tolerance_s_) {
        return false;
      }
      pose = upper->pose;
      return pose.matrix().allFinite();
    }
    if (upper == lio_pose_history_.end()) {
      const auto& latest = lio_pose_history_.back();
      if (std::fabs(static_cast<double>(target - latest.stamp_ns)) * 1.0e-9 >
          lio_observation_sync_tolerance_s_) {
        return false;
      }
      pose = latest.pose;
      return pose.matrix().allFinite();
    }
    const auto& after = *upper;
    const auto& before = *std::prev(upper);
    const double span = static_cast<double>(after.stamp_ns - before.stamp_ns);
    if (span <= 0.0) {
      pose = before.pose;
      return pose.matrix().allFinite();
    }
    const double ratio = std::clamp(
      static_cast<double>(target - before.stamp_ns) / span, 0.0, 1.0);
    pose = Eigen::Isometry3f::Identity();
    pose.translation() = before.pose.translation() + static_cast<float>(ratio) *
      (after.pose.translation() - before.pose.translation());
    Eigen::Quaternionf before_q(before.pose.rotation());
    Eigen::Quaternionf after_q(after.pose.rotation());
    before_q.normalize();
    after_q.normalize();
    if (before_q.coeffs().dot(after_q.coeffs()) < 0.0f) {
      after_q.coeffs() *= -1.0f;
    }
    pose.linear() = before_q.slerp(static_cast<float>(ratio), after_q).normalized().toRotationMatrix();
    return pose.matrix().allFinite();
  }

  void reanchorLioToUkf() {
    Eigen::Isometry3f T_lio = Eigen::Isometry3f::Identity();
    if (!pose_estimator ||
        !currentLioPose(T_lio)) {
      resetLioAnchor();
      return;
    }
    const Eigen::Isometry3f anchor =
      Eigen::Isometry3f(pose_estimator->matrix()) * T_lio.inverse();
    {
      std::lock_guard<std::mutex> anchor_lock(lio_anchor_mutex_);
      lio_map_T_lio_ = anchor;
    }
    previous_lio_pose_ = T_lio;
    lio_anchor_valid_.store(anchor.matrix().allFinite());
    lio_has_previous_pose_ = lio_anchor_valid_.load();
    if (lio_anchor_valid_.load()) {
      const double yaw = std::atan2(anchor.rotation()(1, 0), anchor.rotation()(0, 0));
      std::lock_guard<std::mutex> anchor_filter_lock(anchor_ukf_mutex_);
      anchor_ukf_.reset(
        Eigen::Vector3d(anchor.translation().x(), anchor.translation().y(), yaw),
        Eigen::Vector3d(
          lio_correct_xy_variance_, lio_correct_xy_variance_,
          lio_correct_orientation_variance_));
    }
  }

  Eigen::Isometry3f mapToLioAnchorSnapshot() const {
    std::lock_guard<std::mutex> lock(lio_anchor_mutex_);
    return lio_map_T_lio_;
  }

  Eigen::Isometry3f fusePlanarAnchorObservation(
      const Eigen::Isometry3f& measured_map_T_lio,
      const CorrectionNoise& noise,
      const char* source) {
    const Eigen::Isometry3f current = mapToLioAnchorSnapshot();
    const double measured_yaw = std::atan2(
      measured_map_T_lio.rotation()(1, 0), measured_map_T_lio.rotation()(0, 0));
    AnchorObservation observation;
    observation.value = Eigen::Vector3d(
      measured_map_T_lio.translation().x(), measured_map_T_lio.translation().y(),
      measured_yaw);
    observation.variance = Eigen::Vector3d(
      std::max(1.0e-9f, noise.horizontal_variance),
      std::max(1.0e-9f, noise.horizontal_variance),
      std::max(1.0e-9f, noise.orientation_variance));
    observation.source = source ? source : "unknown";

    Eigen::Vector3d fused;
    {
      std::lock_guard<std::mutex> lock(anchor_ukf_mutex_);
      if (!anchor_ukf_.initialized()) {
        const double current_yaw = std::atan2(
          current.rotation()(1, 0), current.rotation()(0, 0));
        anchor_ukf_.reset(
          Eigen::Vector3d(current.translation().x(), current.translation().y(), current_yaw),
          Eigen::Vector3d(
            lio_correct_xy_variance_, lio_correct_xy_variance_,
            lio_correct_orientation_variance_));
      }
      const AnchorUpdateResult update = anchor_ukf_.update(observation);
      if (!update.accepted) {
        return current;
      }
      last_anchor_innovation_ = update.innovation;
      last_anchor_gain_ = update.gain;
      last_anchor_observation_source_ = observation.source;
      fused = anchor_ukf_.value();
    }

    Eigen::Isometry3f target = current;
    target.translation().x() = static_cast<float>(fused.x());
    target.translation().y() = static_cast<float>(fused.y());
    const float current_yaw = std::atan2(current.rotation()(1, 0), current.rotation()(0, 0));
    const float yaw_delta = static_cast<float>(wrapAnchorYaw(fused.z() - current_yaw));
    target.linear() =
      Eigen::AngleAxisf(yaw_delta, Eigen::Vector3f::UnitZ()).toRotationMatrix() *
      current.rotation();
    return target;
  }

  bool rtkPositionGoodForNavigation(const RtkObservation& observation) const {
    return localization::rtkPositionGoodForNavigation(
      observation.usable, observation.quality);
  }

  bool withinRtkPrimaryHandoffSuppress() const {
    if (rtk_primary_handoff_suppress_until_ns_ <= 0) {
      return false;
    }
    return steadyNowNanoseconds() < rtk_primary_handoff_suppress_until_ns_;
  }

  void suppressLioMotionAnomalyForRtkHandoff(const char* reason) {
    const std::int64_t now_ns = steadyNowNanoseconds();
    const std::int64_t suppress_ns = static_cast<std::int64_t>(
      rtk_primary_handoff_suppress_s_ * 1e9);
    rtk_primary_handoff_suppress_until_ns_ = now_ns + std::max<std::int64_t>(0, suppress_ns);
    if (lio_motion_anomaly_active_) {
      clearLioMotionAnomaly(reason);
    } else {
      resetLioAnchor();
    }
    RCLCPP_WARN(
      get_logger(),
      "Suppressing FAST-LIO motion anomaly for %.1fs after %s (RTK XY remains navigation safety net)",
      rtk_primary_handoff_suppress_s_, reason);
  }

  void latchLioMotionAnomaly(const LioMotionGuardResult& result) {
    {
      std::lock_guard<std::mutex> lock(lio_motion_anomaly_mutex_);
      if (lio_motion_anomaly_active_.load()) {
        return;
      }
      lio_motion_anomaly_reason_ = result.reason;
      lio_motion_anomaly_yaw_step_rad_ = result.yaw_step_rad;
      lio_motion_anomaly_yaw_rate_radps_ = result.yaw_rate_radps;
      lio_motion_anomaly_active_.store(true);
    }
    // This function now runs from the independent LIO callback group. Only
    // invalidate atomic state here; the point-cloud callback performs the full
    // anchor reset during verified recovery. Publishing timers observe the
    // anomaly flag immediately without racing PoseEstimator access.
    lio_anchor_valid_.store(false);
    RCLCPP_ERROR(
      get_logger(),
      "FAST-LIO motion anomaly latched: reason=%s yaw_step=%.1fdeg yaw_rate=%.1fdeg/s; holding last trusted pose",
      result.reason.c_str(), result.yaw_step_rad * 180.0 / M_PI,
      result.yaw_rate_radps * 180.0 / M_PI);
  }

  void clearLioMotionAnomaly(const char* source) {
    if (!lio_motion_anomaly_active_.load()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "FAST-LIO motion anomaly cleared after %s", source);
    resetLioAnchor();
    {
      std::lock_guard<std::mutex> lock(lio_motion_anomaly_mutex_);
      lio_motion_anomaly_reason_ = "none";
      lio_motion_anomaly_yaw_step_rad_ = 0.0;
      lio_motion_anomaly_yaw_rate_radps_ = 0.0;
      lio_motion_anomaly_active_.store(false);
    }
  }

  void applyFusionProfileObservationScale(CorrectionNoise& noise) const {
    if (fusion_profile_.load() != kFusionProfileBalanced) {
      return;
    }
    const float scale = static_cast<float>(balanced_observation_variance_scale_);
    noise.horizontal_variance *= scale;
    noise.vertical_variance *= scale;
    noise.orientation_variance *= scale;
  }

  CorrectionNoise scanMatchCorrectionNoise(
      const PoseEstimator::MatchResult& match) const {
    CorrectionNoise noise;
    noise.horizontal_variance = lio_correct_xy_variance_;
    noise.vertical_variance = lio_correct_z_variance_;
    noise.orientation_variance = lio_correct_orientation_variance_;
    if (!lio_dynamic_covariance_enable_) {
      applyFusionProfileObservationScale(noise);
      return noise;
    }

    const bool vgicp = match.method_.find("vgicp") != std::string::npos;
    const float good_score = vgicp
      ? lio_dynamic_vgicp_good_score_ : lio_dynamic_ndt_good_score_;
    const float score_span = std::max(1.0e-4f, ndt_max_fitness_score_ - good_score);
    const float score_penalty = std::clamp(
      (match.fitness_score_ - good_score) / score_span, 0.0f, 1.0f);
    const float inlier_span = std::max(
      1.0e-4f, lio_dynamic_good_inlier_fraction_ - 0.05f);
    const float inlier_penalty = std::isfinite(last_ndt_inlier_fraction_)
      ? std::clamp(
          (lio_dynamic_good_inlier_fraction_ - last_ndt_inlier_fraction_) / inlier_span,
          0.0f, 1.0f)
      : 1.0f;
    // Fitness and overlap describe different failure modes. Let the weaker
    // one dominate, then square it so high-quality matches stay near the
    // minimum noise while borderline matches are strongly de-weighted.
    const float penalty = std::pow(std::max(score_penalty, inlier_penalty), 2.0f);
    noise.quality_penalty = penalty;
    noise.horizontal_variance = lio_correct_xy_variance_ + penalty *
      (lio_dynamic_xy_variance_max_ - lio_correct_xy_variance_);
    noise.vertical_variance = lio_correct_z_variance_ + penalty *
      (lio_dynamic_z_variance_max_ - lio_correct_z_variance_);
    noise.orientation_variance = lio_correct_orientation_variance_ + penalty *
      (lio_dynamic_orientation_variance_max_ - lio_correct_orientation_variance_);
    applyFusionProfileObservationScale(noise);
    return noise;
  }

  CorrectionNoise rtkCorrectionNoise(const RtkObservation& observation) const {
    CorrectionNoise noise;
    noise.horizontal_variance = static_cast<float>(std::max(
      observation.horizontal_std_m * observation.horizontal_std_m,
      static_cast<double>(lio_correct_xy_variance_)));
    noise.vertical_variance = gnss_use_elevation_
      ? noise.horizontal_variance : lio_z_variance_;
    noise.orientation_variance = observation.heading_usable
      ? static_cast<float>(std::max(
          observation.heading_std_rad * observation.heading_std_rad,
          static_cast<double>(lio_correct_orientation_variance_)))
      : lio_orientation_variance_;
    noise.quality_penalty = static_cast<float>(std::clamp(
      observation.horizontal_std_m / std::max(gnss_max_horizontal_std_, 1.0e-3),
      0.0, 1.0));
    applyFusionProfileObservationScale(noise);
    return noise;
  }

  bool scheduleLioAnchorCorrection(
      const Eigen::Isometry3f& target_map_T_base,
      const CorrectionNoise& noise,
      const char* source,
      const rclcpp::Time& stamp,
      float translation_rate_mps = -1.0f,
      float rotation_rate_radps = -1.0f,
      bool bypass_cooldown = false) {
    const std::int64_t steady_now_ns = steadyNowNanoseconds();
    if (!lio_anchor_valid_.load() || pending_lio_correction_.active ||
        (!bypass_cooldown && !lio_correction_cooldown_gate_.canStart(steady_now_ns)) ||
        !target_map_T_base.matrix().allFinite()) {
      return false;
    }
    Eigen::Isometry3f current_lio_T_base = Eigen::Isometry3f::Identity();
    if (!lioPoseAt(stamp, current_lio_T_base)) {
      return false;
    }
    const Eigen::Isometry3f measured_map_T_lio =
      target_map_T_base * current_lio_T_base.inverse();
    if (!measured_map_T_lio.matrix().allFinite()) {
      return false;
    }
    const Eigen::Isometry3f target_map_T_lio =
      fusePlanarAnchorObservation(measured_map_T_lio, noise, source);
    const Eigen::Isometry3f current_map_T_lio = mapToLioAnchorSnapshot();
    const Eigen::Vector3f translation_delta =
      target_map_T_lio.translation() - current_map_T_lio.translation();
    Eigen::Quaternionf current_orientation(current_map_T_lio.rotation());
    Eigen::Quaternionf target_orientation(target_map_T_lio.rotation());
    current_orientation.normalize();
    target_orientation.normalize();
    const float rotation_delta = current_orientation.angularDistance(target_orientation);
    const float translation_m = translation_delta.norm();
    pending_lio_correction_.active = true;
    pending_lio_correction_.target_map_T_lio = target_map_T_lio;
    pending_lio_correction_.noise = noise;
    pending_lio_correction_.source = source;
    pending_lio_correction_.last_update_stamp_ns = stamp.nanoseconds();
    pending_lio_correction_.initial_translation_m = translation_m;
    pending_lio_correction_.initial_rotation_rad = rotation_delta;
    pending_lio_correction_.remaining_translation_m = translation_m;
    pending_lio_correction_.remaining_rotation_rad = rotation_delta;
    pending_lio_correction_.translation_rate_mps =
      translation_rate_mps > 0.0f ? translation_rate_mps : lio_correction_translation_rate_mps_;
    pending_lio_correction_.rotation_rate_radps =
      rotation_rate_radps > 0.0f ? rotation_rate_radps : lio_correction_rotation_rate_radps_;
    last_correction_xy_variance_ = noise.horizontal_variance;
    last_correction_z_variance_ = noise.vertical_variance;
    last_correction_orientation_variance_ = noise.orientation_variance;
    last_correction_quality_penalty_ = noise.quality_penalty;
    last_correction_progress_ = 0.0f;
    lio_correction_cooldown_gate_.markStarted(steady_now_ns);
    last_correction_started_steady_ns_ = steady_now_ns;
    last_correction_source_ = source;
    return true;
  }

  bool advancePendingLioCorrection(const rclcpp::Time& stamp) {
    if (!pending_lio_correction_.active || !lio_anchor_valid_.load()) {
      return false;
    }
    double dt = 0.10;
    if (pending_lio_correction_.last_update_stamp_ns > 0) {
      dt = static_cast<double>(
        stamp.nanoseconds() - pending_lio_correction_.last_update_stamp_ns) * 1.0e-9;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = 0.10;
    }
    dt = std::clamp(dt, 0.02, 0.25);
    pending_lio_correction_.last_update_stamp_ns = stamp.nanoseconds();

    std::lock_guard<std::mutex> anchor_lock(lio_anchor_mutex_);
    const Eigen::Vector3f current_translation = lio_map_T_lio_.translation();
    const Eigen::Vector3f target_translation =
      pending_lio_correction_.target_map_T_lio.translation();
    const Eigen::Vector3f translation_delta = target_translation - current_translation;
    const float translation_m = translation_delta.norm();
    const float translation_step =
      pending_lio_correction_.translation_rate_mps * static_cast<float>(dt);
    const float translation_ratio = translation_m > 1.0e-6f
      ? std::min(1.0f, translation_step / translation_m) : 1.0f;
    lio_map_T_lio_.translation() =
      current_translation + translation_ratio * translation_delta;

    Eigen::Quaternionf current_orientation(lio_map_T_lio_.rotation());
    Eigen::Quaternionf target_orientation(
      pending_lio_correction_.target_map_T_lio.rotation());
    current_orientation.normalize();
    target_orientation.normalize();
    if (current_orientation.coeffs().dot(target_orientation.coeffs()) < 0.0f) {
      target_orientation.coeffs() *= -1.0f;
    }
    const float rotation_rad = current_orientation.angularDistance(target_orientation);
    const float rotation_step =
      pending_lio_correction_.rotation_rate_radps * static_cast<float>(dt);
    const float rotation_ratio = rotation_rad > 1.0e-6f
      ? std::min(1.0f, rotation_step / rotation_rad) : 1.0f;
    const Eigen::Quaternionf next_orientation =
      current_orientation.slerp(rotation_ratio, target_orientation).normalized();
    lio_map_T_lio_.linear() = next_orientation.toRotationMatrix();

    pending_lio_correction_.remaining_translation_m =
      (target_translation - lio_map_T_lio_.translation()).norm();
    pending_lio_correction_.remaining_rotation_rad =
      next_orientation.angularDistance(target_orientation);
    const float translation_fraction = pending_lio_correction_.initial_translation_m > 1.0e-6f
      ? pending_lio_correction_.remaining_translation_m /
        pending_lio_correction_.initial_translation_m : 0.0f;
    const float rotation_fraction = pending_lio_correction_.initial_rotation_rad > 1.0e-6f
      ? pending_lio_correction_.remaining_rotation_rad /
        pending_lio_correction_.initial_rotation_rad : 0.0f;
    last_correction_progress_ = std::clamp(
      1.0f - std::max(translation_fraction, rotation_fraction), 0.0f, 1.0f);

    if (pending_lio_correction_.remaining_translation_m <=
          lio_correction_completion_translation_m_ &&
        pending_lio_correction_.remaining_rotation_rad <=
          lio_correction_completion_rotation_rad_) {
      lio_map_T_lio_ = pending_lio_correction_.target_map_T_lio;
      last_correction_progress_ = 1.0f;
      RCLCPP_INFO(get_logger(),
        "%s global correction smoothing completed", pending_lio_correction_.source.c_str());
      pending_lio_correction_.active = false;
      last_correction_completed_steady_ns_ = steadyNowNanoseconds();
      lio_correction_cooldown_gate_.markCompleted(last_correction_completed_steady_ns_);
      if (one_shot_correction_.active && one_shot_correction_.status == "smoothing") {
        one_shot_correction_.active = false;
        one_shot_correction_.status = "completed";
        one_shot_correction_.reason = one_shot_correction_.selected_source == "ndt_vgicp" &&
            one_shot_correction_.selection_reason == "ukf_high_quality_ndt"
          ? "ukf_high_quality_ndt"
          : one_shot_correction_.selected_source == "ndt_vgicp" &&
              one_shot_correction_.selection_reason == "ukf_float_outside_gate_ndt_only"
            ? "ukf_float_outside_gate_ndt_only"
          : one_shot_correction_.selected_source == "ukf_fused"
          ? "corrected_ukf_fused"
          : one_shot_correction_.selected_source == "rtk"
            ? "corrected_rtk"
            : one_shot_correction_.selected_source == "ndt_vgicp"
              ? "corrected_ndt" : "anchor_update_completed";
        one_shot_correction_.completed_steady_ns = last_correction_completed_steady_ns_;
      }
    }
    // The current frame contains part of the queued global correction even if
    // this step completed it. Its dynamic covariance must still be applied.
    return true;
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
    if (lio_motion_anomaly_active_) {
      return false;
    }
    // The absolute RTK/NDT pose may be based on the most recently received
    // LIO frame, but it cannot authorize that old frame as the new continuous
    // source. Require one subsequent independent FAST-LIO callback.
    if (!lioHandoffHasFreshFrame()) {
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
    if (!lio_anchor_valid_.load()) {
      reanchorLioToUkf();
      if (!lio_anchor_valid_.load()) {
        return false;
      }
      RCLCPP_INFO(get_logger(),
        "FAST-LIO2 anchored to the map at [%.3f, %.3f, %.3f]",
        pose_estimator->pos().x(), pose_estimator->pos().y(), pose_estimator->pos().z());
    }
    const bool correction_step_applied = advancePendingLioCorrection(stamp);
    if (correction_step_applied) {
      horizontal_variance = std::max(
        horizontal_variance, pending_lio_correction_.noise.horizontal_variance);
      vertical_variance = std::max(
        vertical_variance, pending_lio_correction_.noise.vertical_variance);
      orientation_variance = std::max(
        orientation_variance, pending_lio_correction_.noise.orientation_variance);
    }
    const Eigen::Isometry3f T_map = mapToLioAnchorSnapshot() * T_lio;
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
    completeLioHandoff();
    return true;
  }

  bool updateRtkSelfStability(
      const Eigen::Vector3f& source_xy, int64_t sample_stamp_ns, float source_step_m) {
    if (sample_stamp_ns <= 0 || !source_xy.head<2>().allFinite()) {
      rtk_stability_window_.clear();
      rtk_self_stable_ = false;
      return false;
    }
    // Instantaneous RTK jumps invalidate the trust window immediately.
    if (source_step_m > lio_max_correction_jump_m_) {
      rtk_stability_window_.clear();
      rtk_self_stable_ = false;
      RtkStabilitySample sample;
      sample.stamp_ns = sample_stamp_ns;
      sample.xy = source_xy.head<2>();
      rtk_stability_window_.push_back(sample);
      return false;
    }
    if (!rtk_stability_window_.empty() &&
        sample_stamp_ns == rtk_stability_window_.back().stamp_ns) {
      return rtk_self_stable_;
    }
    RtkStabilitySample sample;
    sample.stamp_ns = sample_stamp_ns;
    sample.xy = source_xy.head<2>();
    rtk_stability_window_.push_back(sample);
    const int64_t window_ns =
      static_cast<int64_t>(rtk_trust_stable_window_s_ * 1.0e9);
    while (!rtk_stability_window_.empty() &&
           sample_stamp_ns - rtk_stability_window_.front().stamp_ns > window_ns) {
      rtk_stability_window_.pop_front();
    }
    if (rtk_stability_window_.size() < 2) {
      rtk_self_stable_ = false;
      return false;
    }
    const double span_s = static_cast<double>(
      rtk_stability_window_.back().stamp_ns - rtk_stability_window_.front().stamp_ns) *
      1.0e-9;
    if (span_s + 1.0e-3 < rtk_trust_stable_window_s_) {
      rtk_self_stable_ = false;
      return false;
    }
    Eigen::Vector2f min_xy = rtk_stability_window_.front().xy;
    Eigen::Vector2f max_xy = min_xy;
    for (const auto& item : rtk_stability_window_) {
      min_xy = min_xy.cwiseMin(item.xy);
      max_xy = max_xy.cwiseMax(item.xy);
    }
    const float span_m = (max_xy - min_xy).norm();
    rtk_self_stable_ = span_m <= rtk_trust_stable_span_m_;
    return rtk_self_stable_;
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
      int64_t sample_stamp_ns,
      bool trust_stable_source = false,
      bool ignore_lio_residual_cap = false,
      bool force_correction = false) {
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

    float source_step = 0.0f;
    if (gate.has_prev_source) {
      source_step = (source_xy.head<2>() - gate.prev_source_xy.head<2>()).norm();
    }

    // Hard-reject on LIO↔source residual size only for sources that are not a
    // fixed, self-checked RTK stream. Fixed RTK may sit several metres from a
    // drifted LIO pose; that disagreement is the reason to correct, not reject.
    // Instantaneous RTK source jumps are handled below via source_step.
    if (!trust_stable_source && !ignore_lio_residual_cap) {
      if (residual_xy > static_cast<float>(gnss_max_residual_)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "%s residual %.2fm exceeds %.2fm; rejecting as a jump",
          source, residual_xy, gnss_max_residual_);
        gate.reset();
        gate.last_decision = "jump_rejected";
        return AuxiliaryGateStatus::reject;
      }
      if (residual_xy > lio_max_correction_jump_m_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "%s absolute correction rejected: xy=%.2fm/%.2fm yaw=%.1fdeg",
          source, residual_xy, lio_max_correction_jump_m_,
          residual_yaw * 180.0 / M_PI);
        gate.reset();
        gate.last_decision = "absolute_correction_rejected";
        return AuxiliaryGateStatus::reject;
      }
    } else if (residual_xy > 50.0f) {
      // Sanity only: ignore pathological residuals while trusting RTK.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s residual %.2fm exceeds sanity cap 50.0m while RTK quality is fixed",
        source, residual_xy);
      gate.resetConsecutive();
      gate.last_decision = "sanity_rejected";
      return AuxiliaryGateStatus::reject;
    }

    // Oversized yaw is untrusted (dual-antenna flicker, bad match yaw). Keep
    // the XY gate alive and let the correction caller discard yaw.
    if (residual_yaw > lio_max_correction_yaw_rad_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s yaw residual %.1fdeg exceeds %.1fdeg; XY correction may still proceed",
        source, residual_yaw * 180.0 / M_PI,
        lio_max_correction_yaw_rad_ * 180.0 / M_PI);
      residual_yaw = 0.0f;
    }

    if (gate.has_prev_source) {
      const float residual_delta = std::fabs(residual_xy - gate.prev_residual_xy);
      const bool source_jumped = source_step > lio_max_correction_jump_m_;
      // Growing LIO↔RTK residual is expected while LIO drifts; only the RTK
      // source jump itself is untrusted when ignore_lio_residual_cap is set.
      const bool residual_jumped =
        !trust_stable_source &&
        !ignore_lio_residual_cap &&
        gate.consecutive > 0 &&
        residual_delta > lio_max_correction_jump_m_;
      if (source_jumped || residual_jumped) {
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

    if (!drifted && !force_correction) {
      gate.resetConsecutive();
      if (residual_xy <= lio_rearm_xy_m_ &&
          residual_yaw <= 0.5f * lio_drift_yaw_rad_) {
        gate.correction_latched = false;
      }
      gate.last_decision = "suppressed_low_drift";
      return AuxiliaryGateStatus::reject;
    }

    if (gate.correction_latched && !force_correction) {
      gate.last_decision = "single_correction_latched";
      return AuxiliaryGateStatus::reject;
    }

    // Large LIO↔RTK residual may only be applied after the RTK stream itself
    // is self-stable (~1s). While waiting, do not hard-reject — LIO residual
    // will keep changing and must not poison the gate.
    if (ignore_lio_residual_cap &&
        !trust_stable_source &&
        residual_xy > lio_max_correction_jump_m_) {
      gate.has_prev_source = true;
      gate.prev_source_xy = source_xy;
      gate.prev_correction_xy = correction_xy;
      gate.prev_residual_xy = residual_xy;
      gate.prev_residual_yaw = residual_yaw;
      gate.last_stamp_ns = sample_stamp_ns;
      gate.consecutive = 0;
      gate.last_decision = "awaiting_rtk_self_stable";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%s waiting for %.1fs self-stable window before trusting xy=%.2fm "
        "(source_step=%.2fm)",
        source, rtk_trust_stable_window_s_, residual_xy, source_step);
      return AuxiliaryGateStatus::pending;
    }

    const bool residual_stable = !gate.has_prev_source ||
      ((correction_xy - gate.prev_correction_xy).norm() <= lio_stable_xy_tolerance_m_ &&
       std::fabs(residual_yaw - gate.prev_residual_yaw) <= lio_stable_yaw_tolerance_rad_) ||
      // When trusting RTK, the filter/LIO residual may keep growing while RTK
      // itself is steady. Require only the RTK source to stay put.
      ((trust_stable_source || ignore_lio_residual_cap) &&
       source_step <= rtk_trust_stable_span_m_);
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
    const int required_frames = trust_stable_source ? 1 : lio_drift_hysteresis_frames_;
    if (gate.consecutive < required_frames) {
      gate.last_decision = trust_stable_source ? "rtk_stable_pending" : "stable_pending";
      return AuxiliaryGateStatus::pending;
    }
    gate.last_decision = trust_stable_source ? "rtk_stable_accept" : "stable_accept";
    return AuxiliaryGateStatus::accept;
  }

  bool maybeCorrectLioDrift(
      const PoseEstimator::MatchResult& match, const rclcpp::Time& stamp,
      bool force_correction = false) {
    lio_corrected_this_frame_ = false;
    if (motion_phase_ != "stationary") {
      ndt_drift_gate_.resetConsecutive();
      ndt_drift_gate_.last_decision = "correction_requires_stationary";
      return false;
    }
    if (!pose_estimator || pending_lio_correction_.active ||
        !match.is_converged_ || !match.transform_.allFinite() ||
        !std::isfinite(match.fitness_score_) ||
        match.fitness_score_ >= ndt_max_fitness_score_ ||
        !last_ndt_status_healthy_ ||
        !std::isfinite(last_ndt_inlier_fraction_) ||
        last_ndt_inlier_fraction_ < 0.05f) {
      ndt_drift_gate_.resetConsecutive();
      ndt_drift_gate_.last_decision = "quality_rejected";
      return false;
    }
    const Eigen::Vector3f ndt_position = match.transform_.block<3, 1>(0, 3);
    const Eigen::Vector3f ukf_position = pose_estimator->pos();
    const Eigen::Vector2f correction_xy =
      ndt_position.head<2>() - ukf_position.head<2>();
    const float drift_xy = correction_xy.norm();
    Eigen::Quaternionf ndt_orientation(match.transform_.block<3, 3>(0, 0));
    ndt_orientation.normalize();
    const float ndt_yaw = yawFromRotation(ndt_orientation.toRotationMatrix());
    const float lio_yaw = yawFromRotation(pose_estimator->quat().toRotationMatrix());
    const float drift_yaw = std::fabs(yawDifference(ndt_yaw, lio_yaw));
    last_ndt_drift_x_m_ = correction_xy.x();
    last_ndt_drift_y_m_ = correction_xy.y();
    last_ndt_drift_xy_m_ = drift_xy;
    last_ndt_drift_yaw_rad_ = drift_yaw;
    last_ndt_orientation_delta_rad_ = pose_estimator->quat().angularDistance(ndt_orientation);
    const bool yaw_trusted = drift_yaw <= lio_max_correction_yaw_rad_;
    if (!yaw_trusted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "NDT/VGICP yaw residual %.1fdeg exceeds %.1fdeg; discarding yaw, XY-only",
        drift_yaw * 180.0 / M_PI,
        lio_max_correction_yaw_rad_ * 180.0 / M_PI);
    }
    const bool drifted = force_correction || drift_xy >= lio_drift_xy_m_ ||
      (yaw_trusted && drift_yaw >= lio_drift_yaw_rad_);
    const AuxiliaryGateStatus status = evaluateAuxiliaryDriftGate(
      ndt_drift_gate_, "NDT/VGICP", true, drifted, ndt_position,
      correction_xy, drift_xy, yaw_trusted ? drift_yaw : 0.0f,
      stamp.nanoseconds(), false, false, force_correction);
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
    Eigen::Isometry3f target_map_T_base = Eigen::Isometry3f::Identity();
    target_map_T_base.translation() = ndt_position;
    // Keep current LIO yaw when the match yaw is untrustworthy.
    target_map_T_base.linear() = yaw_trusted
      ? ndt_orientation.toRotationMatrix()
      : pose_estimator->quat().normalized().toRotationMatrix();
    CorrectionNoise noise = scanMatchCorrectionNoise(match);
    if (!yaw_trusted) {
      noise.orientation_variance = lio_orientation_variance_;
    }
    if (!scheduleLioAnchorCorrection(
          target_map_T_base, noise, "NDT/VGICP", stamp,
          -1.0f, -1.0f, force_correction)) {
      ndt_drift_gate_.resetConsecutive();
      ndt_drift_gate_.last_decision = "correction_schedule_rejected";
      return false;
    }
    ndt_drift_gate_.markCorrected();
    rtk_drift_gate_.resetConsecutive();
    lio_corrected_this_frame_ = true;
    if (force_correction && one_shot_correction_.active) {
      one_shot_correction_.status = "smoothing";
      one_shot_correction_.selected_source = "ndt_vgicp";
      one_shot_correction_.reason = "accepted_ndt_anchor_observation";
    }
    RCLCPP_WARN(get_logger(),
      "NDT/VGICP queued smooth LIO correction: xy=%.3fm yaw=%.1fdeg score=%.3f "
      "inlier=%.3f Rxy=%.4f Ryaw=%.5f method=%s heading=%s",
      drift_xy, yaw_trusted ? drift_yaw * 180.0 / M_PI : 0.0, match.fitness_score_,
      last_ndt_inlier_fraction_, noise.horizontal_variance,
      noise.orientation_variance, match.method_.c_str(),
      yaw_trusted ? "yes" : "discarded");
    return true;
  }

  bool maybeCorrectRtkDrift(
      const RtkObservation& observation, bool force_correction = false) {
    if (motion_phase_ != "stationary") {
      rtk_drift_gate_.resetConsecutive();
      rtk_drift_gate_.last_decision = "correction_requires_stationary";
      return false;
    }
    if (observation.stamp_ns <= 0 ||
        observation.stamp_ns == last_rtk_aux_observation_stamp_ns_) {
      return false;
    }
    last_rtk_aux_observation_stamp_ns_ = observation.stamp_ns;
    const bool fixed_quality_ok = rtkCorrectionQualityOk(observation);
    if (!fixed_quality_ok) {
      rtk_stability_window_.clear();
      rtk_self_stable_ = false;
      rtk_drift_gate_.resetConsecutive();
      rtk_drift_gate_.last_decision = "fixed_rtk_required";
      return false;
    }
    // Keep the RTK self-stability window alive even while a smooth correction
    // is in flight. Otherwise the 3s window expires mid-correction and the
    // next large residual is treated as untrusted again.
    Eigen::Vector3f rtk_position_for_stability = observation.position;
    float source_step_for_stability = 0.0f;
    if (rtk_drift_gate_.has_prev_source) {
      source_step_for_stability =
        (rtk_position_for_stability.head<2>() -
         rtk_drift_gate_.prev_source_xy.head<2>()).norm();
    }
    const bool rtk_self_stable = fixed_quality_ok && updateRtkSelfStability(
      rtk_position_for_stability, observation.stamp_ns, source_step_for_stability);
    const bool trust_rtk = rtk_self_stable ||
      (!force_correction && motion_phase_ == "moving");
    if (force_correction && !rtk_self_stable) {
      rtk_drift_gate_.last_decision = "awaiting_rtk_self_stable";
      if (force_correction && one_shot_correction_.active) {
        one_shot_correction_.status = "waiting_source";
        one_shot_correction_.reason = "waiting_for_fixed_rtk_stability_window";
      }
      return false;
    }
    if (!pose_estimator || lio_corrected_this_frame_ ||
        pending_lio_correction_.active) {
      if (lio_corrected_this_frame_) {
        rtk_drift_gate_.resetConsecutive();
      }
      return false;
    }
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
    // Self-stable fixed RTK trusts dual-antenna heading the same way it trusts
    // XY. The normal 30 deg gate still protects against heading flicker while
    // the RTK stream is not yet self-stable.
    const bool yaw_trusted = localization::rtkHeadingTrustedForCorrection(
      observation.heading_usable, drift_yaw, lio_max_correction_yaw_rad_,
      trust_rtk);
    if (observation.heading_usable && !yaw_trusted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "RTK heading residual %.1fdeg exceeds %.1fdeg; discarding yaw, XY-only "
        "(rtk_self_stable=no)",
        drift_yaw * 180.0 / M_PI,
        lio_max_correction_yaw_rad_ * 180.0 / M_PI);
    } else if (observation.heading_usable && trust_rtk &&
               drift_yaw > lio_max_correction_yaw_rad_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "RTK heading residual %.1fdeg exceeds %.1fdeg; trusting yaw because "
        "fixed RTK is self_stable",
        drift_yaw * 180.0 / M_PI,
        lio_max_correction_yaw_rad_ * 180.0 / M_PI);
    }
    const bool drifted = force_correction || drift_xy >= lio_drift_xy_m_ ||
      (yaw_trusted && drift_yaw >= lio_drift_yaw_rad_);
    const Eigen::Vector2f correction_xy =
      rtk_position.head<2>() - ukf_position.head<2>();
    // Fixed RTK: never hard-reject because LIO drifted far. Only wait for the
    // RTK self-stability window before applying oversized corrections.
    const AuxiliaryGateStatus status = evaluateAuxiliaryDriftGate(
      rtk_drift_gate_, "RTK", fixed_quality_ok, drifted, rtk_position,
      correction_xy, drift_xy, yaw_trusted ? drift_yaw : 0.0f,
      observation.stamp_ns, trust_rtk, /*ignore_lio_residual_cap=*/fixed_quality_ok,
      force_correction);
    if (status == AuxiliaryGateStatus::pending) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "LIO/RTK drift pending correction: xy=%.3fm yaw=%.1fdeg frames=%d/%d "
        "quality=%s rtk_self_stable=%s trust_rtk=%s",
        drift_xy, drift_yaw * 180.0 / M_PI,
        rtk_drift_gate_.consecutive, lio_drift_hysteresis_frames_,
        observation.quality.c_str(), rtk_self_stable ? "yes" : "no",
        trust_rtk ? "yes" : "no");
      return false;
    }
    if (status != AuxiliaryGateStatus::accept) {
      return false;
    }

    Eigen::Quaternionf orientation = pose_estimator->quat();
    float orientation_variance = lio_orientation_variance_;
    if (gnss_use_heading_ && yaw_trusted) {
      orientation = observation.orientation;
      orientation_variance = static_cast<float>(std::max(
        observation.heading_std_rad * observation.heading_std_rad,
        static_cast<double>(lio_correct_orientation_variance_)));
      last_rtk_heading_fused_stamp_ns_ = observation.heading_stamp_ns;
      rtk_heading_fused_this_frame_ = true;
    }
    CorrectionNoise noise = rtkCorrectionNoise(observation);
    noise.orientation_variance = orientation_variance;
    Eigen::Isometry3f target_map_T_base = Eigen::Isometry3f::Identity();
    target_map_T_base.translation() = rtk_position;
    target_map_T_base.linear() = orientation.normalized().toRotationMatrix();
    const float translation_rate = trust_rtk
      ? rtk_trusted_correction_translation_rate_mps_
      : lio_correction_translation_rate_mps_;
    const float rotation_rate = (trust_rtk && yaw_trusted)
      ? rtk_trusted_correction_rotation_rate_radps_
      : lio_correction_rotation_rate_radps_;
    if (!scheduleLioAnchorCorrection(
          target_map_T_base, noise, "RTK",
          timeOnStampClock(observation.stamp_ns, get_clock()->now()),
          translation_rate, rotation_rate,
          /*bypass_cooldown=*/trust_rtk || force_correction)) {
      rtk_drift_gate_.resetConsecutive();
      rtk_drift_gate_.last_decision = "correction_schedule_rejected";
      return false;
    }
    rtk_drift_gate_.markCorrected();
    ndt_drift_gate_.resetConsecutive();
    rtk_position_fused_this_frame_ = true;
    lio_corrected_this_frame_ = true;
    if (force_correction && one_shot_correction_.active) {
      one_shot_correction_.status = "smoothing";
      one_shot_correction_.selected_source = "rtk";
      one_shot_correction_.reason = "accepted_fixed_rtk_anchor_observation";
    }
    last_rtk_map_position_ = rtk_position;
    RCLCPP_WARN(get_logger(),
      "RTK queued smooth LIO correction: xy=%.3fm yaw=%.1fdeg quality=%s "
      "heading=%s self_stable=%s rate=%.2fm/s Rxy=%.4f Ryaw=%.5f",
      drift_xy, yaw_trusted ? drift_yaw * 180.0 / M_PI : 0.0,
      observation.quality.c_str(),
      yaw_trusted ? "yes" : (observation.heading_usable ? "discarded" : "no"),
      rtk_self_stable ? "yes" : "no",
      translation_rate,
      noise.horizontal_variance, noise.orientation_variance);
    return true;
  }

  bool maybeCorrectUkfFused(
      const PoseEstimator::MatchResult& match,
      const RtkObservation& observation,
      const CorrectionCandidateSummary& float_rtk,
      const rclcpp::Time& stamp) {
    if (motion_phase_ != "stationary") {
      rtk_drift_gate_.resetConsecutive();
      rtk_drift_gate_.last_decision = "correction_requires_stationary";
      return false;
    }
    if (!pose_estimator || !match.transform_.allFinite() ||
        !std::isfinite(match.fitness_score_) ||
        match.fitness_score_ < ukf_high_quality_ndt_score_ ||
        match.fitness_score_ >= ndt_max_fitness_score_ ||
        !rtkFloatQualityOk(observation) || !float_rtk.eligible ||
        !floatRtkResidualWithinGate(
          float_rtk.residual_xy, ukf_float_max_residual_m_)) {
      return false;
    }
    const Eigen::Vector3f ndt_position = match.transform_.block<3, 1>(0, 3);
    const Eigen::Vector3f rtk_position = observation.position;
    const CorrectionNoise ndt_noise = scanMatchCorrectionNoise(match);
    const CorrectionNoise rtk_noise = rtkCorrectionNoise(observation);
    const double ndt_weight = 1.0 / std::max(1.0e-6f, ndt_noise.horizontal_variance);
    const double rtk_weight = 1.0 / std::max(1.0e-6f, rtk_noise.horizontal_variance);
    const float weight_sum = static_cast<float>(ndt_weight + rtk_weight);
    Eigen::Isometry3f target = Eigen::Isometry3f::Identity();
    target.translation().head<2>() = static_cast<float>(ndt_weight / weight_sum) *
      ndt_position.head<2>() + static_cast<float>(rtk_weight / weight_sum) *
      rtk_position.head<2>();
    target.translation().z() = ndt_position.z();
    Eigen::Quaternionf ndt_orientation(match.transform_.block<3, 3>(0, 0));
    ndt_orientation.normalize();
    target.linear() = ndt_orientation.toRotationMatrix();
    const float fused_variance = static_cast<float>(1.0 / (ndt_weight + rtk_weight));
    CorrectionNoise fused_noise = ndt_noise;
    fused_noise.horizontal_variance = fused_variance;
    fused_noise.quality_penalty = std::max(ndt_noise.quality_penalty, rtk_noise.quality_penalty);
    if (!scheduleLioAnchorCorrection(
          target, fused_noise, "NDT+RTK_FLOAT(UKF)", stamp,
          -1.0f, -1.0f, true)) {
      return false;
    }
    if (one_shot_correction_.active) {
      one_shot_correction_.status = "smoothing";
      one_shot_correction_.selected_source = "ukf_fused";
      one_shot_correction_.reason = "corrected_ukf_fused";
    }
    return true;
  }

  CorrectionCandidateSummary ndtCorrectionCandidate(
      const PoseEstimator::MatchResult& match,
      bool quality_ok,
      const rclcpp::Time& stamp) const {
    CorrectionCandidateSummary candidate;
    candidate.eligible = quality_ok && pose_estimator && match.is_converged_ &&
      match.transform_.allFinite() && std::isfinite(match.fitness_score_) &&
      match.fitness_score_ < ndt_max_fitness_score_ &&
      std::isfinite(last_ndt_inlier_fraction_) && last_ndt_inlier_fraction_ >= 0.05f;
    candidate.stamp_ns = stamp.nanoseconds();
    if (!candidate.eligible) {
      return candidate;
    }
    const Eigen::Vector3f position = match.transform_.block<3, 1>(0, 3);
    Eigen::Quaternionf orientation(match.transform_.block<3, 3>(0, 0));
    orientation.normalize();
    const CorrectionNoise noise = scanMatchCorrectionNoise(match);
    candidate.x = position.x();
    candidate.y = position.y();
    candidate.yaw = std::atan2(
      orientation.toRotationMatrix()(1, 0), orientation.toRotationMatrix()(0, 0));
    candidate.yaw_valid = true;
    candidate.horizontal_variance = noise.horizontal_variance;
    candidate.orientation_variance = noise.orientation_variance;
    candidate.residual_xy =
      (position.head<2>() - pose_estimator->pos().head<2>()).norm();
    candidate.residual_yaw = pose_estimator->quat().angularDistance(orientation);
    return candidate;
  }

  CorrectionCandidateSummary rtkCorrectionCandidate(
      const RtkObservation& observation) const {
    CorrectionCandidateSummary candidate;
    candidate.eligible = pose_estimator &&
      (rtkCorrectionQualityOk(observation) || rtkFloatQualityOk(observation));
    candidate.stamp_ns = observation.stamp_ns;
    if (!candidate.eligible) {
      return candidate;
    }
    const CorrectionNoise noise = rtkCorrectionNoise(observation);
    candidate.x = observation.position.x();
    candidate.y = observation.position.y();
    candidate.yaw = std::atan2(
      observation.orientation.toRotationMatrix()(1, 0),
      observation.orientation.toRotationMatrix()(0, 0));
    candidate.yaw_valid = observation.heading_usable;
    candidate.horizontal_variance = noise.horizontal_variance;
    candidate.orientation_variance = noise.orientation_variance;
    candidate.residual_xy =
      (observation.position.head<2>() - pose_estimator->pos().head<2>()).norm();
    if (observation.quality == "float" &&
        !floatRtkResidualWithinGate(candidate.residual_xy, ukf_float_max_residual_m_)) {
      candidate.eligible = false;
      return candidate;
    }
    candidate.residual_yaw = observation.heading_usable
      ? pose_estimator->quat().angularDistance(observation.orientation) : 0.0;
    return candidate;
  }

  bool rtkCorrectionQualityOk(const RtkObservation& observation) const {
    // A correction source must be independently valid at the point where it
    // is consumed.  Do not rely only on the upstream `usable` flag: this
    // keeps future callers from accidentally scheduling a stale or partial
    // RTK sample while the robot is at a waypoint.
    return observation.usable && observation.quality == "fixed" &&
      observation.heading_usable &&
      observation.position.allFinite() &&
      std::isfinite(observation.horizontal_std_m) &&
      observation.horizontal_std_m <= gnss_max_horizontal_std_ &&
      std::isfinite(observation.age_s) && observation.age_s <= gnss_max_age_ &&
      observation.stamp_ns > 0;
  }

  bool rtkFloatQualityOk(const RtkObservation& observation) const {
    return observation.usable && observation.quality == "float" &&
      observation.position.allFinite() &&
      std::isfinite(observation.horizontal_std_m) &&
      observation.horizontal_std_m <= gnss_max_horizontal_std_ * 2.0 &&
      std::isfinite(observation.age_s) && observation.age_s <= gnss_max_age_ &&
      observation.stamp_ns > 0;
  }

  void evaluateAuxiliaryCorrections(
      const PoseEstimator::MatchResult* match,
      bool ndt_quality_ok,
      const RtkObservation& observation,
      const rclcpp::Time& stamp) {
    const bool ndt_fresh = last_ndt_healthy_ && last_ndt_update_time_.nanoseconds() > 0 &&
      std::fabs((stamp - last_ndt_update_time_).seconds()) <= 1.0;
    const bool rtk_ready = rtkCorrectionQualityOk(observation);
    const bool rtk_float_ready = rtkFloatQualityOk(observation);
    float float_rtk_residual_xy_m = -1.0f;
    bool rtk_float_within_gate = false;
    std::string float_rtk_gate_reason = "not_float_or_quality_rejected";
    if (rtk_float_ready && pose_estimator) {
      float_rtk_residual_xy_m =
        (observation.position.head<2>() - pose_estimator->pos().head<2>()).norm();
      rtk_float_within_gate = floatRtkResidualWithinGate(
        float_rtk_residual_xy_m, ukf_float_max_residual_m_);
      float_rtk_gate_reason = rtk_float_within_gate
        ? "within_gate" : "residual_exceeds_max";
    } else if (rtk_float_ready) {
      float_rtk_gate_reason = "lio_pose_unavailable";
    }
    const float ndt_score = match && std::isfinite(match->fitness_score_)
      ? match->fitness_score_ : std::numeric_limits<float>::infinity();
    last_float_rtk_residual_xy_m_ = float_rtk_residual_xy_m;
    last_float_rtk_within_gate_ = rtk_float_within_gate;
    last_float_rtk_gate_reason_ = float_rtk_gate_reason;
    // A decimated FAST-LIO callback with no NDT result means "not sampled in
    // this frame", not that the last healthy NDT match disappeared.
    if (match) {
      last_ndt_score_band_ = !std::isfinite(ndt_score)
        ? "unavailable"
        : ndt_score < ukf_high_quality_ndt_score_
          ? "high_quality"
          : ndt_score < ndt_max_fitness_score_ ? "eligible" : "poor";
    }
    const bool force_correction = one_shot_correction_.active;
    const CorrectionPolicyMode effective_mode = force_correction
      ? one_shot_correction_.mode : preferred_correction_mode_;
    policy_source_ready_ = effective_mode == CorrectionPolicyMode::ndt
      ? ndt_fresh
      : effective_mode == CorrectionPolicyMode::rtk
        ? rtk_ready
        : (ndt_fresh || rtk_ready || rtk_float_within_gate);

    // The scheduler requests a fresh NDT result for an active NDT waypoint
    // transaction. Until that scan arrives, this is a pending measurement —
    // never an exhausted source attempt eligible for no-correction fallback.
    if (force_correction && effective_mode == CorrectionPolicyMode::ndt && !match) {
      one_shot_correction_.status = "waiting_source";
      one_shot_correction_.selected_source = "none";
      one_shot_correction_.selection_reason = "waiting_for_fresh_ndt_measurement";
      one_shot_correction_.reason = "waiting_for_fresh_ndt_measurement";
      last_correction_candidate_source_ = "none";
      last_correction_selection_reason_ = "waiting_for_fresh_ndt_measurement";
      return;
    }

    // In LIO-hold mode both absolute observers have already failed their
    // quality gates.  Keep propagating the high-rate FAST-LIO pose and reject
    // automatic anchor updates; increasing anchor process noise here would
    // have the opposite effect by increasing the next absolute-observation
    // gain. Explicit operator one-shot corrections remain allowed.
    if (fusion_profile_.load() == kFusionProfileLioHold && !force_correction) {
      policy_source_ready_ = lioOdomFresh(stamp);
      last_correction_candidate_source_ = "none";
      last_correction_selection_reason_ = "lio_hold_external_observations_suppressed";
      ndt_drift_gate_.resetConsecutive();
      rtk_drift_gate_.resetConsecutive();
      return;
    }

    if (!lio_anchor_valid_.load() || pending_lio_correction_.active) {
      last_correction_candidate_source_ = "none";
      last_correction_selection_reason_ = pending_lio_correction_.active
        ? "correction_already_active" : "lio_anchor_unavailable";
      return;
    }

    CorrectionCandidateSummary ndt;
    if (match && correctionPolicyAllowsNdt(effective_mode)) {
      ndt = ndtCorrectionCandidate(*match, ndt_quality_ok, stamp);
    }
    CorrectionCandidateSummary rtk;
    if (correctionPolicyAllowsRtk(effective_mode)) {
      rtk = rtkCorrectionCandidate(observation);
      // RTK mode is an absolute fixed-RTK policy.  Floating samples may be
      // considered only by the explicit UKF fusion policy; they must never
      // become a standalone RTK anchor correction.
      if (effective_mode == CorrectionPolicyMode::rtk && !rtk_ready) {
        rtk.eligible = false;
      }
    }

    const bool ndt_drifted = ndt.eligible &&
      (ndt.residual_xy >= lio_drift_xy_m_ || ndt.residual_yaw >= lio_drift_yaw_rad_);
    const bool rtk_drifted = rtk.eligible &&
      (rtk.residual_xy >= lio_drift_xy_m_ ||
       (rtk.yaw_valid && rtk.residual_yaw >= lio_drift_yaw_rad_));

    // Low-drift observations still pass through the existing gate so a prior
    // one-shot correction can re-arm only after returning to the safe band.
    if (ndt.eligible && !ndt_drifted && !force_correction) {
      maybeCorrectLioDrift(*match, stamp);
    }
    if (rtk_ready && rtk.eligible && !rtk_drifted && !force_correction) {
      maybeCorrectRtkDrift(observation);
    }

    ndt.eligible = ndt.eligible && (force_correction || ndt_drifted) &&
      (force_correction || !ndt_drift_gate_.correction_latched);
    rtk.eligible = rtk.eligible && (force_correction || rtk_drifted) &&
      (force_correction || !rtk_drift_gate_.correction_latched);
    const bool fixed_rtk_eligible = rtk_ready && rtk.eligible;
    const bool float_rtk_eligible = !rtk_ready && rtk_float_within_gate && rtk.eligible;
    CorrectionSelection selection = selectWaypointCorrectionSource(
      effective_mode, ndt.eligible, ndt_score, fixed_rtk_eligible,
      float_rtk_eligible, ukf_high_quality_ndt_score_,
      prefer_fixed_rtk_for_correction_, rtk_float_ready && !rtk_float_within_gate);
    const bool ukf_float_fusion = selection.source == CorrectionSource::ukf_fused;
    last_correction_candidate_source_ = correctionSourceName(selection.source);
    last_correction_selection_reason_ = selection.reason;
    if (force_correction) {
      one_shot_correction_.selection_reason = selection.reason;
    }

    bool corrected = false;
    if (ukf_float_fusion && match) {
      ndt_drift_gate_.resetConsecutive();
      rtk_drift_gate_.resetConsecutive();
      corrected = maybeCorrectUkfFused(*match, observation, rtk, stamp);
    } else if (selection.source == CorrectionSource::ndt && match) {
      rtk_drift_gate_.resetConsecutive();
      corrected = maybeCorrectLioDrift(*match, stamp, force_correction);
    } else if (selection.source == CorrectionSource::rtk) {
      ndt_drift_gate_.resetConsecutive();
      corrected = maybeCorrectRtkDrift(observation, force_correction);
    } else {
      if (match && !ndt_quality_ok) {
        ndt_drift_gate_.resetConsecutive();
        ndt_drift_gate_.last_decision = "quality_rejected";
      }
      if (!rtk_ready) {
        rtk_drift_gate_.resetConsecutive();
        rtk_drift_gate_.last_decision = "quality_rejected";
      }
      if (force_correction) {
        if (motion_phase_ != "stationary") {
          one_shot_correction_.status = "waiting_source";
          one_shot_correction_.reason = "waiting_stationary";
        } else {
          // An exhausted absolute-source attempt is an explicit outcome, not
          // a perpetual wait. The Edge execution layer separately requires a
          // fresh healthy LIO sample and a confirmed physical stop before it
          // departs on any of these no-correction completions.
          one_shot_correction_.active = false;
          one_shot_correction_.status = "completed";
          one_shot_correction_.selected_source = "none";
          one_shot_correction_.completed_steady_ns = steadyNowNanoseconds();
          if (effective_mode == CorrectionPolicyMode::ndt) {
            one_shot_correction_.reason = "ndt_no_correction_continue";
          } else if (effective_mode == CorrectionPolicyMode::rtk) {
            one_shot_correction_.reason = "rtk_no_correction_continue";
          } else {
            one_shot_correction_.reason = "ukf_no_correction_sources_meet_gate";
          }
          RCLCPP_WARN(get_logger(),
            "Waypoint %s correction skipped without blocking: ndt_band=%s "
            "float_rtk=%.3fm/%0.3fm gate=%s; continuing on LIO/UKF",
            correctionPolicyModeName(effective_mode), last_ndt_score_band_.c_str(),
            last_float_rtk_residual_xy_m_, ukf_float_max_residual_m_,
            last_float_rtk_gate_reason_.c_str());
        }
      }
    }
    if (corrected) {
      // Each source owns its own drift-episode latch. In UKF mode this lets a
      // second independently valid source update the same anchor on a later
      // low-rate cycle; the anchor covariance provides the adaptive blend.
      ndt_drift_gate_.resetConsecutive();
      rtk_drift_gate_.resetConsecutive();
    }
  }

  void control_localization_correction_callback(
      const std::shared_ptr<robots_dog_msgs::srv::ControlLocalizationCorrection::Request> request,
      std::shared_ptr<robots_dog_msgs::srv::ControlLocalizationCorrection::Response> response) {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    response->transaction_id = request ? request->transaction_id : "";
    if (!request || request->transaction_id.empty()) {
      response->accepted = false;
      response->status = "failed";
      response->message = "transaction_id is required";
      return;
    }
    using Request = robots_dog_msgs::srv::ControlLocalizationCorrection::Request;
    if (request->command == Request::COMMAND_CANCEL) {
      if (one_shot_correction_.transaction_id != request->transaction_id) {
        response->accepted = false;
        response->status = "not_found";
        response->message = "correction transaction does not exist";
        return;
      }
      one_shot_correction_.active = false;
      one_shot_correction_.status = "cancelled";
      one_shot_correction_.reason = "cancelled_by_owner";
      if (pending_lio_correction_.active) {
        pending_lio_correction_.reset();
        const Eigen::Isometry3f current_anchor = mapToLioAnchorSnapshot();
        const double current_yaw = std::atan2(
          current_anchor.rotation()(1, 0), current_anchor.rotation()(0, 0));
        std::lock_guard<std::mutex> anchor_filter_lock(anchor_ukf_mutex_);
        anchor_ukf_.reset(
          Eigen::Vector3d(
            current_anchor.translation().x(), current_anchor.translation().y(), current_yaw),
          Eigen::Vector3d(
            lio_correct_xy_variance_, lio_correct_xy_variance_,
            lio_correct_orientation_variance_));
      }
      response->accepted = true;
      response->status = one_shot_correction_.status;
      response->message = "correction transaction cancelled";
      return;
    }
    if (request->command != Request::COMMAND_START) {
      response->accepted = false;
      response->status = "failed";
      response->message = "unsupported correction command";
      return;
    }
    if (one_shot_correction_.transaction_id == request->transaction_id) {
      response->accepted = true;
      response->status = one_shot_correction_.status;
      response->message = "idempotent correction transaction replay";
      return;
    }
    if (one_shot_correction_.active || pending_lio_correction_.active) {
      response->accepted = false;
      response->status = "busy";
      response->message = "another correction transaction is active";
      return;
    }
    CorrectionPolicyMode mode;
    if (request->mode == Request::MODE_NDT) {
      mode = CorrectionPolicyMode::ndt;
    } else if (request->mode == Request::MODE_RTK) {
      mode = CorrectionPolicyMode::rtk;
    } else if (request->mode == Request::MODE_UKF) {
      mode = CorrectionPolicyMode::ukf;
    } else {
      response->accepted = false;
      response->status = "failed";
      response->message = "mode must be NDT, RTK, or UKF";
      return;
    }
    one_shot_correction_ = OneShotCorrection{};
    one_shot_correction_.active = true;
    one_shot_correction_.transaction_id = request->transaction_id;
    one_shot_correction_.mode = mode;
    one_shot_correction_.status = "waiting_source";
    one_shot_correction_.reason = "waiting_for_eligible_anchor_observation";
    one_shot_correction_.started_steady_ns = steadyNowNanoseconds();
    ndt_drift_gate_.reset();
    rtk_drift_gate_.reset();
    response->accepted = true;
    response->status = one_shot_correction_.status;
    response->message = "one-shot correction transaction started";
  }

  static const char* fusionProfileName(uint8_t profile) {
    switch (profile) {
      case kFusionProfileLioHold:
        return "lio_hold";
      case kFusionProfileBalanced:
        return "balanced";
      default:
        return "nominal";
    }
  }

  void maybeExpireFusionProfile() {
    const auto expiry_ns = fusion_profile_expires_steady_ns_.load();
    if (expiry_ns <= 0 || steadyNowNanoseconds() < expiry_ns) {
      return;
    }
    uint8_t previous = kFusionProfileNominal;
    uint64_t generation = fusion_profile_generation_.load();
    {
      // Serialize expiry with service updates. Without this lock an expired
      // generation could overwrite a newer profile between the first expiry
      // check and the exchange below.
      std::lock_guard<std::mutex> lock(pose_estimator_mutex);
      const auto current_expiry_ns = fusion_profile_expires_steady_ns_.load();
      if (current_expiry_ns <= 0 || steadyNowNanoseconds() < current_expiry_ns) {
        return;
      }
      previous = fusion_profile_.exchange(kFusionProfileNominal);
      fusion_profile_expires_steady_ns_.store(0);
      if (previous != kFusionProfileNominal) {
        generation = fusion_profile_generation_.fetch_add(1) + 1;
      }
    }
    if (previous != kFusionProfileNominal) {
      RCLCPP_INFO(
        get_logger(),
        "temporary localization fusion profile %s expired; restored nominal generation=%lu",
        fusionProfileName(previous), static_cast<unsigned long>(generation));
    }
  }

  void set_localization_fusion_profile_callback(
      const std::shared_ptr<robots_dog_msgs::srv::SetLocalizationFusionProfile::Request> request,
      std::shared_ptr<robots_dog_msgs::srv::SetLocalizationFusionProfile::Response> response) {
    using Request = robots_dog_msgs::srv::SetLocalizationFusionProfile::Request;
    if (!request || (request->profile != Request::PROFILE_NOMINAL &&
        request->profile != Request::PROFILE_LIO_HOLD &&
        request->profile != Request::PROFILE_BALANCED)) {
      response->accepted = false;
      response->applied_profile = fusion_profile_.load();
      response->generation = fusion_profile_generation_.load();
      response->profile_name = fusionProfileName(response->applied_profile);
      response->message = "unsupported localization fusion profile";
      return;
    }
    const double duration_seconds = request->profile == Request::PROFILE_NOMINAL ? 0.0 :
      std::clamp(
        request->duration_seconds > 0.0F ?
        static_cast<double>(request->duration_seconds) : 10.0,
        1.0, 180.0);
    uint64_t applied_generation = 0;
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex);
      const auto current_generation = fusion_profile_generation_.load();
      if (request->expected_generation != 0 &&
          request->expected_generation != current_generation) {
        response->accepted = false;
        response->applied_profile = fusion_profile_.load();
        response->generation = current_generation;
        response->profile_name = fusionProfileName(response->applied_profile);
        response->message = "stale localization fusion profile generation";
        return;
      }
      applied_generation = fusion_profile_generation_.fetch_add(1) + 1;
      fusion_profile_.store(request->profile);
      fusion_profile_expires_steady_ns_.store(
        duration_seconds > 0.0 ?
        steadyNowNanoseconds() + static_cast<std::int64_t>(duration_seconds * 1.0e9) : 0);
      if (request->profile == Request::PROFILE_LIO_HOLD) {
        ndt_drift_gate_.resetConsecutive();
        rtk_drift_gate_.resetConsecutive();
        last_correction_candidate_source_ = "none";
        last_correction_selection_reason_ = "lio_hold_requested";
      }
    }
    response->accepted = true;
    response->applied_profile = request->profile;
    response->generation = applied_generation;
    response->profile_name = fusionProfileName(request->profile);
    response->message = std::string("localization fusion profile applied: ") +
      response->profile_name +
      (duration_seconds > 0.0 ? " for " + std::to_string(duration_seconds) + "s" : "") +
      (request->reason.empty() ? "" : " (" + request->reason + ")");
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void localization_policy_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string command = msg ? msg->data : "";
    const auto separator = command.find(':');
    const auto mode_end = separator == std::string::npos
      ? std::string::npos : command.find(':', separator + 1);
    const std::string requested_mode = separator == std::string::npos
      ? command : command.substr(separator + 1, mode_end - separator - 1);
    const CorrectionPolicyMode next_mode = parseCorrectionPolicyMode(requested_mode);
    const auto anchor_end = mode_end == std::string::npos
      ? std::string::npos : command.find(':', mode_end + 1);
    std::string anchor_preference = "balanced";
    if (mode_end != std::string::npos) {
      anchor_preference = command.substr(mode_end + 1, anchor_end - mode_end - 1);
      if (anchor_preference != "ndt" && anchor_preference != "rtk") {
        anchor_preference = "balanced";
      }
    }
    if (next_mode != preferred_correction_mode_) {
      preferred_correction_mode_ = next_mode;
      preferred_source_ = correctionPolicyModeName(next_mode);
      // A waypoint policy change starts a new correction epoch. Do not carry
      // the previous source's one-shot latch into a different requested
      // source, but still require three fresh stable observations.
      ndt_drift_gate_.reset();
      rtk_drift_gate_.reset();
      last_correction_candidate_source_ = "none";
      last_correction_selection_reason_ = "policy_changed";
      RCLCPP_INFO(get_logger(), "Localization correction policy changed to %s",
        preferred_source_.c_str());
    }
    motion_phase_ = command.find("moving") != std::string::npos ? "moving" : "stationary";
    bool rtk_primary_allowed = false;
    if (mode_end != std::string::npos) {
      const auto primary_begin = command.find(':', mode_end + 1);
      if (primary_begin != std::string::npos) {
        const std::string primary_value = command.substr(primary_begin + 1);
        rtk_primary_allowed = primary_value == "1" || primary_value == "true";
      }
    }
    // RTK may become the continuous source only at an explicitly opted-in
    // outdoor RTK waypoint. It still has to pass the existing Fixed + heading
    // latch; losing that evidence immediately returns to FAST-LIO.
    rtk_primary_allowed_by_policy_ =
      rtk_primary_allowed && next_mode == CorrectionPolicyMode::rtk && motion_phase_ == "moving";
    ukf_anchor_preference_ = next_mode == CorrectionPolicyMode::ukf
      ? anchor_preference : "balanced";
    if (motion_phase_ == "stationary") {
      bridge_active_ = false;
      bridge_distance_m_ = 0.0;
      absolute_stable_count_ = 0;
      absolute_stable_ = false;
      stable_source_.clear();
    }
  }

  bool rtkGoodForNavigation(const RtkObservation& observation) const {
    // Continuous GPS drive still needs dual-antenna heading. Fixed XY alone is
    // exposed separately as rtk_position_good_for_navigation.
    return localization::rtkGoodForPrimaryDrive(
      observation.usable, observation.heading_usable, observation.quality);
  }

  void updateRtkAutoPrimary(const RtkObservation& observation, const rclcpp::Time& stamp) {
    (void)stamp;
    if (!(prefer_fixed_rtk_ || rtk_primary_allowed_by_policy_) || !source_arbiter_enable_) {
      if (rtk_auto_primary_latched_) {
        suppressLioMotionAnomalyForRtkHandoff("prefer_fixed_rtk disabled");
      }
      rtk_auto_primary_latched_ = false;
      rtk_auto_primary_good_frames_ = 0;
      rtk_auto_primary_bad_frames_ = 0;
      return;
    }
    RtkPrimaryLatchState latch_state;
    latch_state.latched = rtk_auto_primary_latched_;
    latch_state.good_frames = rtk_auto_primary_good_frames_;
    latch_state.bad_frames = rtk_auto_primary_bad_frames_;
    const bool was_latched = latch_state.latched;
    updateRtkPrimaryLatch(
      latch_state,
      RtkPrimaryLatchConfig{rtk_primary_promote_samples_, rtk_primary_demote_samples_},
      rtkGoodForNavigation(observation));
    rtk_auto_primary_latched_ = latch_state.latched;
    rtk_auto_primary_good_frames_ = latch_state.good_frames;
    rtk_auto_primary_bad_frames_ = latch_state.bad_frames;
    if (!was_latched && rtk_auto_primary_latched_) {
      RCLCPP_INFO(get_logger(),
        "Outdoor RTK is fixed with a valid heading; GPS is the navigation pose (FAST-LIO/NDT paused)");
      return;
    }
    if (was_latched && !rtk_auto_primary_latched_) {
      suppressLioMotionAnomalyForRtkHandoff("RTK primary demoted");
      RCLCPP_INFO(get_logger(),
        "RTK is no longer good for primary navigation (quality=%s heading=%s usable=%s); falling back to FAST-LIO/NDT",
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

  rclcpp::Time latestGnssStamp(const rclcpp::Time& fallback) {
    std::lock_guard<std::mutex> lock(gnss_mutex_);
    if (!has_gnss_) {
      return fallback;
    }
    return timeOnStampClock(latest_gnss_.header.stamp, fallback);
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
    if (observation.stamp_ns <= 0 ||
        observation.stamp_ns < last_rtk_primary_applied_stamp_ns_) {
      return false;
    }
    if (observation.stamp_ns == last_rtk_primary_applied_stamp_ns_) {
      // Keep the source valid between GNSS updates without correcting the UKF
      // repeatedly with the same statistical sample.
      return true;
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
    last_rtk_primary_applied_stamp_ns_ = observation.stamp_ns;
    is_init_success_ = true;
    clearLioMotionAnomaly("verified RTK relocalization");
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
    const LioHandoffSnapshot lio_handoff = lioHandoffSnapshot();
    Eigen::Vector3d anchor_covariance = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchor_innovation = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchor_gain = Eigen::Vector3d::Zero();
    std::string anchor_observation_source = "none";
    bool anchor_filter_initialized = false;
    {
      std::lock_guard<std::mutex> lock(anchor_ukf_mutex_);
      anchor_filter_initialized = anchor_ukf_.initialized();
      if (anchor_filter_initialized) {
        anchor_covariance = anchor_ukf_.covariance().diagonal();
      }
      anchor_innovation = last_anchor_innovation_;
      anchor_gain = last_anchor_gain_;
      anchor_observation_source = last_anchor_observation_source_;
    }
    Eigen::Vector3f lio_map_position = Eigen::Vector3f::Zero();
    bool lio_map_position_available = false;
    if (lio_anchor_valid_.load()) {
      Eigen::Isometry3f lio_T_base = Eigen::Isometry3f::Identity();
      if (currentLioPose(lio_T_base)) {
        const Eigen::Isometry3f map_T_base = mapToLioAnchorSnapshot() * lio_T_base;
        if (map_T_base.matrix().allFinite()) {
          lio_map_position = map_T_base.translation();
          lio_map_position_available = true;
        }
      }
    }
    const bool rtk_lio_drift_available = rtk_position_available && lio_map_position_available;
    const float rtk_lio_dx = rtk_lio_drift_available
      ? rtk.position.x() - lio_map_position.x() : 0.0f;
    const float rtk_lio_dy = rtk_lio_drift_available
      ? rtk.position.y() - lio_map_position.y() : 0.0f;
    const float rtk_lio_drift_xy = rtk_lio_drift_available
      ? std::hypot(rtk_lio_dx, rtk_lio_dy) : 0.0f;
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
    bool lio_motion_anomaly_active = false;
    std::string lio_motion_anomaly_reason;
    double lio_motion_anomaly_yaw_step_rad = 0.0;
    double lio_motion_anomaly_yaw_rate_radps = 0.0;
    {
      std::lock_guard<std::mutex> lock(lio_motion_anomaly_mutex_);
      lio_motion_anomaly_active = lio_motion_anomaly_active_.load();
      lio_motion_anomaly_reason = lio_motion_anomaly_reason_;
      lio_motion_anomaly_yaw_step_rad = lio_motion_anomaly_yaw_step_rad_;
      lio_motion_anomaly_yaw_rate_radps = lio_motion_anomaly_yaw_rate_radps_;
    }
    std_msgs::msg::String message;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"active_source\":\"" << active_source_
        << "\",\"preferred_source\":\"" << preferred_source_
        << "\",\"correction_policy\":\"" << preferred_source_
        << "\",\"anchor_preference\":\"" << ukf_anchor_preference_
        << "\",\"fusion_profile\":\"" << fusionProfileName(fusion_profile_.load())
        << "\",\"fusion_profile_generation\":" << fusion_profile_generation_.load()
        << ",\"allowed_correction_sources\":\""
        << (preferred_correction_mode_ == CorrectionPolicyMode::ndt
          ? "ndt_vgicp"
          : preferred_correction_mode_ == CorrectionPolicyMode::rtk
            ? "rtk" : "ndt_vgicp,rtk")
        << "\",\"policy_source_ready\":" << (policy_source_ready_ ? "true" : "false")
        << ",\"correction_candidate_source\":\"" << last_correction_candidate_source_
        << "\",\"correction_selection_reason\":\"" << last_correction_selection_reason_
        << "\",\"correction_gates\":{\"ndt_score_band\":\""
        << last_ndt_score_band_
        << "\",\"ukf_high_quality_ndt_score\":" << ukf_high_quality_ndt_score_
        << ",\"ndt_max_fitness_score\":" << ndt_max_fitness_score_
        << ",\"float_rtk_residual_xy_m\":" << last_float_rtk_residual_xy_m_
        << ",\"float_rtk_max_residual_m\":" << ukf_float_max_residual_m_
        << ",\"float_rtk_within_gate\":"
        << (last_float_rtk_within_gate_ ? "true" : "false")
        << ",\"float_rtk_gate_reason\":\"" << last_float_rtk_gate_reason_ << "\"}"
        << ",\"anchor_filter\":{\"initialized\":"
        << (anchor_filter_initialized ? "true" : "false")
        << ",\"source\":\"" << anchor_observation_source
        << "\",\"variance_x\":" << anchor_covariance.x()
        << ",\"variance_y\":" << anchor_covariance.y()
        << ",\"variance_yaw\":" << anchor_covariance.z()
        << ",\"innovation_x\":" << anchor_innovation.x()
        << ",\"innovation_y\":" << anchor_innovation.y()
        << ",\"innovation_yaw\":" << anchor_innovation.z()
        << ",\"gain_x\":" << anchor_gain.x()
        << ",\"gain_y\":" << anchor_gain.y()
        << ",\"gain_yaw\":" << anchor_gain.z() << "}"
        << ",\"one_shot_correction\":{\"active\":"
        << (one_shot_correction_.active ? "true" : "false")
        << ",\"transaction_id\":\"" << one_shot_correction_.transaction_id
        << "\",\"mode\":\"" << correctionPolicyModeName(one_shot_correction_.mode)
        << "\",\"status\":\"" << one_shot_correction_.status
        << "\",\"selected_source\":\"" << one_shot_correction_.selected_source
        << "\",\"selection_reason\":\"" << one_shot_correction_.selection_reason
        << "\",\"reason\":\"" << one_shot_correction_.reason << "\"}"
        << ",\"rtk_auto_primary\":" << (rtk_auto_primary_latched_ ? "true" : "false")
        << ",\"rtk_primary_allowed_by_policy\":"
        << (rtk_primary_allowed_by_policy_ ? "true" : "false")
        << ",\"rtk_good_for_navigation\":" << (rtkGoodForNavigation(rtk) ? "true" : "false")
        << ",\"rtk_position_good_for_navigation\":"
        << (rtkPositionGoodForNavigation(rtk) ? "true" : "false")
        << ",\"phase\":\"" << motion_phase_
        << "\",\"ndt_healthy\":" << (last_ndt_healthy_ ? "true" : "false")
        << ",\"ndt_score\":" << last_ndt_score_
        << ",\"lio_primary\":" << (enable_lio_primary_ ? "true" : "false")
        << ",\"handoff_state\":\""
        << (lio_handoff.pending ? "lio_handoff_pending" : "ready")
        << "\",\"handoff_anchor_generation\":" << lio_handoff.generation
        << ",\"handoff_required_lio_sequence\":" << lio_handoff.required_sequence
        << ",\"handoff_source\":\"" << lio_handoff.source
        << "\",\"handoff_failure_reason\":\"" << lio_handoff.failure_reason << "\""
        << ",\"single_continuous_source_enforced\":"
        << ((enable_lio_primary_ && !rtk_auto_primary_latched_) ? "true" : "false")
        << ",\"lio_healthy\":"
        << (lioOdomFresh(stamp) && !lio_motion_anomaly_active ? "true" : "false")
        << ",\"lio_anchored\":" << (lio_anchor_valid_.load() ? "true" : "false")
        << ",\"lio_motion_anomaly\":" << (lio_motion_anomaly_active ? "true" : "false")
        << ",\"lio_motion_anomaly_reason\":\"" << lio_motion_anomaly_reason << "\""
        << ",\"lio_yaw_step_deg\":" << lio_motion_anomaly_yaw_step_rad * 180.0 / M_PI
        << ",\"lio_yaw_rate_degps\":" << lio_motion_anomaly_yaw_rate_radps * 180.0 / M_PI
        << ",\"lio_max_yaw_step_deg\":" << lio_max_yaw_step_rad_ * 180.0 / M_PI
        << ",\"lio_max_yaw_rate_degps\":" << lio_max_yaw_rate_radps_ * 180.0 / M_PI
        << ",\"lio_corrected\":" << (lio_corrected_this_frame_ ? "true" : "false")
        << ",\"lio_stable_frames\":" << lio_stable_frame_count_
        << ",\"lio_schedule_stable\":"
        << (lio_stable_frame_count_ >= lio_stable_confirmation_frames_ ? "true" : "false")
        << ",\"correction_smoothing_active\":" << (pending_lio_correction_.active ? "true" : "false")
        << ",\"correction_source\":\"" << pending_lio_correction_.source << "\""
        << ",\"last_correction_source\":\"" << last_correction_source_ << "\""
        << ",\"correction_cooldown_remaining_s\":"
        << lio_correction_cooldown_gate_.remainingSeconds(steadyNowNanoseconds())
        << ",\"ndt_schedule_reason\":\"" << last_point_cloud_schedule_reason_ << "\""
        << ",\"correction_progress\":" << last_correction_progress_
        << ",\"correction_xy_variance\":" << last_correction_xy_variance_
        << ",\"correction_z_variance\":" << last_correction_z_variance_
        << ",\"correction_orientation_variance\":" << last_correction_orientation_variance_
        << ",\"correction_quality_penalty\":" << last_correction_quality_penalty_
        << ",\"ndt_drift_frames\":" << ndt_drift_gate_.consecutive
        << ",\"rtk_drift_frames\":" << rtk_drift_gate_.consecutive
        << ",\"ndt_drift_decision\":\"" << ndt_drift_gate_.last_decision << "\""
        << ",\"rtk_drift_decision\":\"" << rtk_drift_gate_.last_decision << "\""
        << ",\"ndt_correction_latched\":" << (ndt_drift_gate_.correction_latched ? "true" : "false")
        << ",\"rtk_correction_latched\":" << (rtk_drift_gate_.correction_latched ? "true" : "false")
        << ",\"ndt_drift\":{\"dx_m\":" << last_ndt_drift_x_m_
        << ",\"dy_m\":" << last_ndt_drift_y_m_
        << ",\"xy_m\":" << last_ndt_drift_xy_m_
        << ",\"yaw_deg\":" << last_ndt_drift_yaw_rad_ * 180.0 / M_PI
        << ",\"orientation_delta_deg\":" << last_ndt_orientation_delta_rad_ * 180.0 / M_PI
        << ",\"threshold_xy_m\":" << lio_drift_xy_m_
        << ",\"threshold_yaw_deg\":" << lio_drift_yaw_rad_ * 180.0 / M_PI
        << ",\"stable_frames\":" << ndt_drift_gate_.consecutive
        << ",\"required_stable_frames\":" << lio_drift_hysteresis_frames_
        << ",\"decision\":\"" << ndt_drift_gate_.last_decision << "\"}"
        << ",\"initialization\":{\"state\":\"" << initialization_state_
        << "\",\"verified\":" << (initialization_verified_ ? "true" : "false")
        << ",\"stable_frames\":" << init_match_count_
        << ",\"required_stable_frames\":" << init_match_count_threshold_
        << ",\"score\":" << last_ndt_score_
        << ",\"inlier_fraction\":" << last_ndt_inlier_fraction_
        << ",\"seed_yaw_deg\":" << initialization_seed_yaw_ * 180.0 / M_PI
        << ",\"matched_yaw_deg\":" << initialization_match_yaw_ * 180.0 / M_PI
        << ",\"yaw_correction_deg\":" << initialization_yaw_correction_rad_ * 180.0 / M_PI
        << ",\"position_correction_m\":" << initialization_position_correction_m_
        << ",\"within_seed_gate\":"
        << ((initialization_position_correction_m_ <= init_match_max_seed_xy_m_ &&
             std::fabs(initialization_yaw_correction_rad_) <= init_match_max_seed_yaw_rad_)
          ? "true" : "false")
        << "}"
        << ",\"global_relocalization\":{\"mode\":\"" << scan_context_effective_runtime_mode_
        << "\",\"state\":\"" << global_relocalization_state_
        << "\",\"attempt_phase\":\"" << relocalization_attempt_phase_
        << "\",\"attempt_index\":" << relocalization_attempt_index_
        << ",\"attempt_total\":" << relocalization_attempt_total_
        << ",\"best_source\":\"" << relocalization_best_source_
        << "\",\"best_score\":" << relocalization_best_score_
        << ",\"required\":" << (global_search_required_ ? "true" : "false")
        << ",\"candidate_applied\":" << (global_candidate_applied_ ? "true" : "false")
        << ",\"keyframe\":" << global_candidate_keyframe_
        << ",\"descriptor_distance\":" << global_candidate_distance_
        << ",\"yaw_deg\":" << global_candidate_yaw_deg_
        << ",\"rmse_m\":" << global_candidate_rmse_m_
        << ",\"overlap\":" << global_candidate_overlap_
        << ",\"rejection_reason\":\"" << global_candidate_rejection_reason_ << "\"}"
        << ",\"suppressed_covariance_scale\":" << lio_suppressed_covariance_scale_
        << ",\"ndt_inlier_fraction\":" << last_ndt_inlier_fraction_
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
    out << ",\"rtk_drift\":{\"sample_stamp_ns\":" << rtk.stamp_ns
        << ",\"source\":\"aligned_fast_lio\""
        << ",\"threshold_xy_m\":" << lio_drift_xy_m_
        << ",\"dx_m\":";
    if (rtk_lio_drift_available) {
      out << rtk_lio_dx;
    } else {
      out << "null";
    }
    out << ",\"dy_m\":";
    if (rtk_lio_drift_available) {
      out << rtk_lio_dy;
    } else {
      out << "null";
    }
    out << ",\"xy_m\":";
    if (rtk_lio_drift_available) {
      out << rtk_lio_drift_xy;
    } else {
      out << "null";
    }
    out << ",\"self_stable\":" << (rtk_self_stable_ ? "true" : "false")
        << ",\"decision\":\"" << rtk_drift_gate_.last_decision << "\"}";
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

    // map.pcd is often the compatibility symlink map/map.pcd -> current/map.pcd.
    // Resolve it so gnss_origin.yaml is loaded from the session directory, not
    // from the map root that may no longer have a sibling origin pointer.
    const std::filesystem::path pcd_path(map_path);
    std::error_code canonical_error;
    const std::filesystem::path canonical_pcd =
      std::filesystem::weakly_canonical(pcd_path, canonical_error);
    const std::filesystem::path map_dir = canonical_error
      ? pcd_path.parent_path()
      : canonical_pcd.parent_path();
    const std::filesystem::path meta_path = map_dir / "gnss_origin.yaml";
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
    advanceGlobalRelocalizationGeneration(reason);
    pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
    resetLioAnchor();
    global_search_required_ = false;
    global_candidate_applied_ = false;
    resetInitializationValidation();
    is_extrapolating_ = false;
    gl_once_gate_ = false;
    consecutive_match_failures_ = 0;
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
    const RtkObservation rtk = currentRtkObservation(get_clock()->now());
    if (rtkGoodForNavigation(rtk) && applyRtkObservation(rtk)) {
      // Only latch GPS as the continuous driver when prefer_fixed_rtk is on.
      // Otherwise keep FAST-LIO primary and treat this as an absolute seed/correct.
      // Do not bypass the normal RTK-primary promotion window here. Keeping
      // LIO active for the first samples lets the edge verify the freshly
      // established map<-lio anchor against fixed RTK before declaring the
      // initialization complete.
      rtk_auto_primary_latched_ = false;
      rtk_auto_primary_good_frames_ = 0;
      rtk_auto_primary_bad_frames_ = 0;
      has_trusted_ndt_pose_ = true;
      gnss_recovery_seed_pending_ = false;
      runtime_relocalization_attempted_ = false;
      localization_state_ = 2;
      initialization_verified_ = true;
      initialization_state_ = "localized";
      // Align the continuous LIO map←odom bridge to the RTK XY+yaw seed now so
      // the next LIO frame does not reintroduce a large yaw residual.
      reanchorLioToUkf();
      beginLioHandoff("rtk_fixed");
      const float seeded_yaw = yawFromRotation(last_init_quat_.toRotationMatrix());
      RCLCPP_INFO(get_logger(),
        "Fixed RTK accepted as %s for %s without NDT validation "
        "x=%.3f y=%.3f yaw=%.1fdeg heading=%s lio_anchor=%s",
        prefer_fixed_rtk_ ? "absolute seed pending RTK-primary promotion" : "absolute seed (LIO remains continuous)",
        reason, last_init_pos_.x(), last_init_pos_.y(), seeded_yaw * 180.0 / M_PI,
        rtk.heading_usable ? "rtk" : "missing",
        lio_anchor_valid_.load() ? "aligned" : "pending");
      return true;
    }
    has_trusted_ndt_pose_ = false;
    is_init_success_ = false;
    rtk_auto_primary_latched_ = false;
    rtk_auto_primary_good_frames_ = 0;
    rtk_auto_primary_bad_frames_ = 0;
    localization_state_ = 1;
    runtime_relocalization_attempted_ = true;
    gnss_recovery_seed_pending_ = true;
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
   * The expensive ICP runs on the background relocalization worker, so this
   * callback only advances the recovery generation and arms the next cloud.
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
    advanceGlobalRelocalizationGeneration("service relocalization");
    is_init_success_ = false;
    global_search_required_ = false;
    global_candidate_applied_ = false;
    resetInitializationValidation();
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

  void global_relocalize_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    bool database_ready = false;
    {
      std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
      database_ready = !scan_context_db_.empty();
    }
    if (!use_global_localization_init_ ||
        scan_context_effective_runtime_mode_ == "disabled" || !database_ready) {
      response->success = false;
      response->message = "Scan Context global relocalization is unavailable for the active map";
      initialization_state_ = "global_unavailable";
      return;
    }

    advanceGlobalRelocalizationGeneration("explicit global relocalization");
    // Keep automatic recovery in the configured shadow rollout, but allow an
    // operator's explicit initialization command to apply a candidate that
    // passes every Scan Context + geometry gate.  The generation scope makes
    // the permission expire as soon as another pose/relocalization operation
    // supersedes this request.
    explicit_global_relocalization_generation_ = global_relocalization_generation_;
    is_init_success_ = false;
    resetInitializationValidation("global_searching");
    localization_state_ = 1;
    global_search_required_ = true;
    global_candidate_applied_ = false;
    gl_once_gate_ = true;
    runtime_relocalization_attempted_ = false;
    consecutive_match_failures_ = 0;
    last_pose_source_ = "global_search";
    response->success = true;
    response->message = scan_context_effective_runtime_mode_ == "active"
      ? "Global position and 360-degree yaw search armed"
      : "Operator global search armed; only a fully geometry-verified candidate may be applied";
    RCLCPP_WARN(get_logger(),
      "Explicit global relocalization armed (mode=%s); mapping-start fallback is disabled",
      scan_context_effective_runtime_mode_.c_str());
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

  void recordPointCloudPerformance(const PointCloudPerfSample& sample) {
    const std::size_t phase_index = sample.phase == "initialization" ? 0
      : sample.phase == "relocalization" ? 1
      : sample.phase == "moving" ? 2 : 3;
    auto& window = point_cloud_perf_windows_[phase_index];
    window.samples.push_back(sample);
    window.heavy_count += sample.heavy ? 1 : 0;
    window.light_count += sample.heavy ? 0 : 1;
    window.stale_count += sample.stale ? 1 : 0;

    last_timeout_ = static_cast<int>(std::lround(sample.stage_ms[kTotal]));
    if (sample.stage_ms[kTotal] > point_cloud_slow_callback_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Slow point cloud callback: total=%.1fms age=%.1fms phase=%s heavy=%s "
        "lock=%.1f convert=%.1f preprocess=%.1f voxel_tf=%.1f global=%.1f "
        "lidar_odom=%.1f lio=%.1f ndt=%.1f local_map=%.1f vgicp=%.1f output=%.1f",
        sample.stage_ms[kTotal], sample.cloud_age_ms, sample.phase.c_str(),
        sample.heavy ? "yes" : "no", sample.stage_ms[kLockWait],
        sample.stage_ms[kRosConversion], sample.stage_ms[kPreprocess],
        sample.stage_ms[kVoxelTransform], sample.stage_ms[kGlobalRelocalization],
        sample.stage_ms[kLidarOdometry], sample.stage_ms[kLioObservation],
        sample.stage_ms[kNdt], sample.stage_ms[kLocalMap], sample.stage_ms[kVgicp],
        sample.stage_ms[kOutput]);
    }

    const auto now = SteadyClock::now();
    if (last_point_cloud_perf_log_ == SteadyClock::time_point{}) {
      last_point_cloud_perf_log_ = now;
      return;
    }
    if (std::chrono::duration<double>(now - last_point_cloud_perf_log_).count() <
        point_cloud_perf_log_interval_s_) {
      return;
    }

    static constexpr std::array<const char*, 4> phase_names{
      "initialization", "relocalization", "moving", "stationary"};
    for (std::size_t index = 0; index < point_cloud_perf_windows_.size(); ++index) {
      auto& phase_window = point_cloud_perf_windows_[index];
      if (phase_window.samples.empty()) {
        continue;
      }
      auto stage_values = [&phase_window](PointCloudPerfStage stage) {
        std::vector<double> values;
        values.reserve(phase_window.samples.size());
        for (const auto& entry : phase_window.samples) {
          values.push_back(entry.stage_ms[stage]);
        }
        return values;
      };
      std::vector<double> ages;
      ages.reserve(phase_window.samples.size());
      for (const auto& entry : phase_window.samples) {
        ages.push_back(entry.cloud_age_ms);
      }
      const auto total = stage_values(kTotal);
      const auto conversion = stage_values(kRosConversion);
      const auto preprocess = stage_values(kPreprocess);
      const auto voxel_tf = stage_values(kVoxelTransform);
      const auto ndt = stage_values(kNdt);
      const auto local_map = stage_values(kLocalMap);
      const auto vgicp = stage_values(kVgicp);
      const auto output = stage_values(kOutput);
      RCLCPP_INFO(
        get_logger(),
        "Point cloud perf phase=%s n=%zu heavy=%llu light=%llu stale=%llu "
        "total[mean/p50/p90/p99/max]=[%.1f/%.1f/%.1f/%.1f/%.1f]ms age_p99=%.1fms "
        "p90[convert/preprocess/voxel_tf/ndt/local_map/vgicp/output]="
        "[%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f]ms",
        phase_names[index], phase_window.samples.size(),
        static_cast<unsigned long long>(phase_window.heavy_count),
        static_cast<unsigned long long>(phase_window.light_count),
        static_cast<unsigned long long>(phase_window.stale_count),
        std::accumulate(total.begin(), total.end(), 0.0) / total.size(),
        quantile(total, 0.50), quantile(total, 0.90), quantile(total, 0.99),
        *std::max_element(total.begin(), total.end()), quantile(ages, 0.99),
        quantile(conversion, 0.90), quantile(preprocess, 0.90),
        quantile(voxel_tf, 0.90), quantile(ndt, 0.90), quantile(local_map, 0.90),
        quantile(vgicp, 0.90), quantile(output, 0.90));
      phase_window = PointCloudPerfWindow{};
    }
    last_point_cloud_perf_log_ = now;
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    const auto callback_start = SteadyClock::now();
    PointCloudPerfSample perf;
    auto perf_scope = makeScopeExit([this, &perf, callback_start]() {
      perf.stage_ms[kTotal] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - callback_start).count();
      recordPointCloudPerformance(perf);
    });
    const auto lock_start = SteadyClock::now();
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    perf.stage_ms[kLockWait] = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - lock_start).count();
    applyPendingGlobalRelocalizationResult();
    perf.phase = !is_init_success_ ? (gl_once_gate_ ? "relocalization" : "initialization")
      : motion_phase_;
    const rclcpp::Time cloud_stamp(points_msg->header.stamp, get_clock()->get_clock_type());
    perf.cloud_age_ms = std::max(0.0, (get_clock()->now() - cloud_stamp).seconds() * 1000.0);
    // Receipt health and processing freshness are separate. A stale queued
    // frame still proves the sensor stream is alive; it simply has no value
    // for a real-time localization correction.
    updateLidarStatus(true);
    if (isPointCloudStale(
          get_clock()->now().nanoseconds(), cloud_stamp.nanoseconds(),
          point_cloud_max_age_s_, is_init_success_)) {
      perf.stale = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping stale localization cloud: age=%.1fms limit=%.1fms",
        perf.cloud_age_ms, point_cloud_max_age_s_ * 1000.0);
      return;
    }
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
    const bool skip_lidar_matching = rtkPrimaryShouldDrive(
      source_arbiter_enable_, bridge_active_, rtk_auto_primary_latched_);
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
    ++ndt_frame_counter_;
    const bool lio_fresh_for_schedule = enable_lio_primary_ && is_init_success_ &&
      lioOdomFresh(rclcpp::Time(stamp));
    const bool lidar_odometry_required = enable_lidar_odometry_prediction_ &&
      !(lio_fresh_for_schedule && !rtk_auto_primary_latched_);
    PointCloudScheduleConfig schedule_config;
    schedule_config.initialization_stride = initialization_ndt_stride_;
    schedule_config.stationary_stride = stationary_ndt_stride_;
    schedule_config.moving_stride = moving_ndt_stride_;
    schedule_config.stable_max_rate_hz = stable_ndt_max_rate_hz_;
    schedule_config.recovery_max_rate_hz = recovery_ndt_max_rate_hz_;
    PointCloudScheduleInput schedule_input;
    schedule_input.initialized = is_init_success_;
    schedule_input.lidar_matching_paused = skip_lidar_matching;
    schedule_input.rtk_primary = skip_lidar_matching;
    const std::int64_t schedule_now_ns = steadyNowNanoseconds();
    schedule_input.global_relocalization_requested =
      use_global_localization_init_ && gl_once_gate_ && global_localization_ptr_ &&
      !is_init_success_ && globalRelocalizationRequestReady(
        schedule_now_ns, global_relocalization_earliest_start_ns_);
    schedule_input.lidar_odometry_required = lidar_odometry_required;
    schedule_input.lio_primary_enabled = enable_lio_primary_;
    schedule_input.lio_stable = lio_fresh_for_schedule &&
      lio_stable_frame_count_ >= lio_stable_confirmation_frames_;
    schedule_input.correction_suppressed = pending_lio_correction_.active ||
      !lio_correction_cooldown_gate_.canStart(steadyNowNanoseconds());
    // A waypoint NDT transaction needs a fresh registration result. Do not
    // let a rate-limited FAST-LIO callback be interpreted as a failed match.
    schedule_input.force_ndt_match = one_shot_correction_.active &&
      one_shot_correction_.status == "waiting_source" &&
      one_shot_correction_.mode == CorrectionPolicyMode::ndt &&
      motion_phase_ == "stationary";
    schedule_input.motion_phase = motion_phase_;
    schedule_input.frame_index = ndt_frame_counter_;
    schedule_input.now_ns = schedule_now_ns;
    schedule_input.last_ndt_start_ns = last_ndt_match_start_steady_ns_;
    const PointCloudWorkDecision work_decision =
      decidePointCloudWork(schedule_config, schedule_input);
    last_point_cloud_schedule_reason_ = work_decision.reason;
    if (work_decision.run_ndt) {
      last_ndt_match_start_steady_ns_ = schedule_input.now_ns;
    }
    perf.heavy = work_decision.needs_heavy_cloud;
    perf.run_ndt = work_decision.run_ndt;

    raw_points_ptr_->clear();
    if (work_decision.needs_heavy_cloud) {
      auto stage_start = SteadyClock::now();
      raw_points_ptr_->clear();
      pcl::fromROSMsg(*points_msg, *raw_points_ptr_);
      perf.stage_ms[kRosConversion] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - stage_start).count();
      if (raw_points_ptr_->empty()) {
        RCLCPP_ERROR(get_logger(), "cloud is empty!!");
        return;
      }
      stage_start = SteadyClock::now();
      auto preprocessed = preprocessScanForMapMatching(raw_points_ptr_);
      perf.stage_ms[kPreprocess] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - stage_start).count();
      stage_start = SteadyClock::now();
      auto filtered = downsample(preprocessed);
      TransformPoints(filtered, raw_points_ptr_);
      perf.stage_ms[kVoxelTransform] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - stage_start).count();
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
        if (is_init_success_) {
          RCLCPP_INFO(get_logger(),
            "Pose estimator set from fixed RTK; GPS is the navigation source");
        } else {
          RCLCPP_INFO(get_logger(),
            "Pose estimator reseeded from fixed RTK XY and heading; validating with local NDT");
        }
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
    if (schedule_input.global_relocalization_requested && work_decision.needs_heavy_cloud) {
      // Consume the one-shot gate before publishing to the capacity-one worker
      // mailbox. A failed attempt must not be retried for every lidar frame.
      gl_once_gate_ = false;
      last_global_localization_attempt_ = std::chrono::steady_clock::now();
      seedPositionFromGnss(rclcpp::Time(points_msg->header.stamp), "global relocalization");
      const auto global_start = SteadyClock::now();
      scheduleGlobalRelocalization(raw_points_ptr_);
      perf.stage_ms[kGlobalRelocalization] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - global_start).count();
      RCLCPP_INFO(
        get_logger(), "Queued background global localization generation=%llu",
        static_cast<unsigned long long>(global_relocalization_generation_));
    }
    
    bool initialized_this_frame = false;
    if (!is_init_success_) {
      localization_state_ = 1;
      if (init_match_result_pending_) {
        init_match_result_pending_ = false;
      PoseEstimator::MatchResult init_result = pose_estimator->GetMatchState();
      RCLCPP_INFO(get_logger(), "init_result.is_converged_ : %d", init_result.is_converged_);
      RCLCPP_INFO(get_logger(), "init_result.fitness_score_ : %f", init_result.fitness_score_);
      const Eigen::Vector2f init_match_xy = init_result.transform_.block<2, 1>(0, 3);
      initialization_match_yaw_ = yawFromRotation(init_result.transform_.block<3, 3>(0, 0));
      initialization_yaw_correction_rad_ = yawDifference(
        initialization_match_yaw_, initialization_seed_yaw_);
      initialization_position_correction_m_ =
        (init_match_xy - last_init_pos_.head<2>()).norm();
      const bool seed_correction_ok =
        initialization_position_correction_m_ <= init_match_max_seed_xy_m_ &&
        std::fabs(initialization_yaw_correction_rad_) <= init_match_max_seed_yaw_rad_;
      const bool pose_stable = !has_previous_init_match_pose_ ||
        ((init_match_xy - previous_init_match_xy_).norm() <= init_match_stable_xy_m_ &&
         std::fabs(yawDifference(initialization_match_yaw_, previous_init_match_yaw_)) <=
           init_match_stable_yaw_rad_);
      const bool global_seed_ready = !global_search_required_ || global_candidate_applied_;
      const bool quality_ok = init_result.is_converged_ && init_result.transform_.allFinite() &&
        init_result.fitness_score_ < init_match_score_threshold_ &&
        last_ndt_inlier_fraction_ >= init_match_min_inlier_fraction_ && global_seed_ready &&
        seed_correction_ok;
      const bool optimal_match = init_result.is_converged_ && init_result.transform_.allFinite() &&
        init_result.fitness_score_ <= init_match_optimal_score_threshold_ &&
        last_ndt_inlier_fraction_ >= init_match_min_inlier_fraction_ && global_seed_ready;
      if (optimal_match) {
        // A very strong NDT observation is already the best available pose;
        // stop the progressive search immediately and promote this transform
        // instead of waiting for the normal multi-frame acquisition window.
        last_init_pos_ = init_result.transform_.block<3, 1>(0, 3);
        last_init_quat_ = Eigen::Quaternionf(init_result.transform_.block<3, 3>(0, 0));
        last_init_quat_.normalize();
        has_set_init_pose_ = true;
        last_pose_source_ = "ndt_optimal";
        relocalization_best_source_ = "ndt_optimal";
        relocalization_best_score_ = init_result.fitness_score_;
        relocalization_attempt_phase_ = "optimal_ndt_applied";
        is_init_success_ = true;
        initialization_verified_ = true;
        initialization_state_ = "localized";
        global_search_required_ = false;
        global_candidate_applied_ = false;
        init_match_count_ = init_match_count_threshold_;
        advanceGlobalRelocalizationGeneration("optimal NDT initialization");
        clearLioMotionAnomaly("optimal NDT initialization");
        beginLioHandoff("ndt_optimal");
        initialized_this_frame = true;
        localization_state_ = 2;
        RCLCPP_INFO(get_logger(),
          "Optimal NDT initialization accepted immediately score=%.6f threshold=%.6f",
          init_result.fitness_score_, init_match_optimal_score_threshold_);
      } else if (quality_ok && pose_stable) {
        init_match_count_++;
        initialization_state_ = "validating";
        RCLCPP_INFO(get_logger(),
          "Init match count: %d/%d score=%.6f inlier=%.3f stable_xy<=%.2fm stable_yaw<=%.1fdeg yaw_correction=%+.2fdeg",
          init_match_count_, init_match_count_threshold_, init_result.fitness_score_,
          last_ndt_inlier_fraction_, init_match_stable_xy_m_,
          init_match_stable_yaw_rad_ * 180.0 / M_PI,
          initialization_yaw_correction_rad_ * 180.0 / M_PI);
        // Check if we have enough consecutive successful matches
        if (init_match_count_ >= init_match_count_threshold_) {
          advanceGlobalRelocalizationGeneration("local initialization success");
          is_init_success_ = true;
          initialization_verified_ = true;
          initialization_state_ = "localized";
          global_search_required_ = false;
          clearLioMotionAnomaly("verified NDT relocalization");
          beginLioHandoff("ndt_verified");
          initialized_this_frame = true;
          localization_state_ = 2;
          RCLCPP_INFO(get_logger(), "Init Pose Successful!!!");
          init_match_count_ = init_match_count_threshold_;
        }
      } else {
        init_match_count_ = quality_ok ? 1 : 0;
        initialization_state_ = global_seed_ready ? "validating" : "global_search_required";
        RCLCPP_INFO(get_logger(),
          "Init match rejected: quality=%s stable=%s global_seed_ready=%s seed_gate=%s "
          "correction_xy=%.2f/%.2fm correction_yaw=%.1f/%.1fdeg score=%.3f inlier=%.3f",
          quality_ok ? "true" : "false", pose_stable ? "true" : "false",
          global_seed_ready ? "true" : "false", seed_correction_ok ? "true" : "false",
          initialization_position_correction_m_, init_match_max_seed_xy_m_,
          initialization_yaw_correction_rad_ * 180.0 / M_PI,
          init_match_max_seed_yaw_rad_ * 180.0 / M_PI, init_result.fitness_score_,
          last_ndt_inlier_fraction_);
      }
      if (quality_ok) {
        previous_init_match_xy_ = init_match_xy;
        previous_init_match_yaw_ = initialization_match_yaw_;
        has_previous_init_match_pose_ = true;
      } else {
        has_previous_init_match_pose_ = false;
      }
      RCLCPP_INFO(get_logger(), "Wait Init Pose!!! Current count: %d/%d", init_match_count_, init_match_count_threshold_);
      }
    }

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

    if (!skip_lidar_matching && lidar_odometry_required && pose_estimator) {
      const auto lidar_odom_start = SteadyClock::now();
      pose_estimator->enable_lidar_odometry_prediction();
      updateLidarOdometryPrediction(raw_points_ptr_, rclcpp::Time(stamp));
      perf.stage_ms[kLidarOdometry] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - lidar_odom_start).count();
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
    const rclcpp::Time frame_stamp(stamp);
    const RtkObservation rtk_observation = currentRtkObservation(frame_stamp);
    // Latch GPS quality from the GNSS sample itself. A queued lidar stamp can
    // make a centimetre-grade fix look stale (|lidar-gnss| > max_age), unlatch
    // skip, run open-sky NDT, and pause the patrol task.
    updateRtkAutoPrimary(currentRtkObservation(latestGnssStamp(frame_stamp)), frame_stamp);
    const bool rtk_primary = false;
    const bool lio_primary = enable_lio_primary_ && is_init_success_ &&
      has_trusted_ndt_pose_ && !rtk_primary && !bridge_active_ &&
      !lio_motion_anomaly_active_ &&
      lioOdomFresh(rclcpp::Time(stamp));
    bool lio_observation_applied = false;
    if (lio_primary) {
      const auto lio_start = SteadyClock::now();
      lio_observation_applied = applyLioPrimaryObservation(rclcpp::Time(stamp));
      if (lio_observation_applied) {
        lio_stable_frame_count_ = std::min(
          lio_stable_frame_count_ + 1, lio_stable_confirmation_frames_);
      } else {
        lio_stable_frame_count_ = 0;
      }
      perf.stage_ms[kLioObservation] = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - lio_start).count();
    } else if (enable_lio_primary_ && is_init_success_) {
      lio_stable_frame_count_ = 0;
    }
    // Obstacle extraction remains on the independent 10 Hz laser scan chain.
    // Scan matching is reduced to 2 Hz while moving and restored to every
    // LiDAR frame while stationary or during initialization.
    // NDT remains the low-rate map-consistency observation even when RTK is
    // fixed. RTK never pauses FAST-LIO or suppresses this independent check.
    const bool run_ndt = work_decision.run_ndt && !rtk_primary;
    perf.run_ndt = run_ndt;
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    std::optional<PoseEstimator::MatchResult> current_match_result;
    bool current_ndt_sample_healthy = false;
    if (run_ndt) {
      aligned = pose_estimator->correct(
        stamp, raw_points_ptr_,
        is_init_success_ && !enable_lio_primary_ &&
          !rtk_primary && !bridge_active_ && !lio_observation_applied,
        enable_lio_primary_ && schedule_input.lio_stable);
      const PoseEstimator::MatchTiming match_timing = pose_estimator->GetMatchTiming();
      perf.stage_ms[kNdt] = match_timing.ndt_ms;
      perf.stage_ms[kLocalMap] = match_timing.local_map_ms;
      perf.stage_ms[kVgicp] = match_timing.refine_ms;
      if (!is_init_success_) {
        init_match_result_pending_ = true;
      }
      publish_scan_matching_status(points_msg->header, aligned);
      const PoseEstimator::MatchResult match_result = pose_estimator->GetMatchState();
      last_ndt_score_ = match_result.fitness_score_;
      // NDT is an absolute observation. Its health must be decided by the
      // registration result, not by the uncertainty limits reserved for the
      // IMU+wheel-odometry fallback bridge. Otherwise a good NDT correction
      // can be discarded solely because the fallback yaw covariance is high.
      const bool ndt_sample_healthy = match_result.is_converged_ &&
        match_result.fitness_score_ < ndt_max_fitness_score_ && last_ndt_status_healthy_;
      current_match_result = match_result;
      current_ndt_sample_healthy = ndt_sample_healthy;
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
    } else if (work_decision.reason != "correction_suppressed" &&
        (rclcpp::Time(stamp) - last_ndt_update_time_).seconds() > 1.0) {
      last_ndt_healthy_ = false;
      ndt_unhealthy_frame_count_ = ndt_failure_hysteresis_frames_;
      ndt_drift_gate_.resetConsecutive();
    }
    if (lio_observation_applied) {
      evaluateAuxiliaryCorrections(
        current_match_result ? &*current_match_result : nullptr,
        current_ndt_sample_healthy,
        rtk_observation,
        rclcpp::Time(stamp));
    } else if (rtk_primary) {
      policy_source_ready_ = true;
    } else if (enable_lio_primary_ && is_init_success_) {
      policy_source_ready_ = false;
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
      policy_source_ready_ = absolute_pose_valid;
      active_source_ = absolute_pose_valid ? "rtk_imu" : "unavailable";
      absolute_observation_updated = absolute_pose_valid &&
        rtk_observation.stamp_ns != last_absolute_observation_stamp_ns_;
      last_absolute_observation_stamp_ns_ = rtk_observation.stamp_ns;
    } else if (lio_observation_applied) {
      absolute_pose_valid = true;
      active_source_ = "lio_imu";
      absolute_observation_updated = true;
    } else if (enable_lio_primary_ && is_init_success_) {
      // A stale FAST-LIO stream is a localization outage, not permission for
      // NDT or RTK to become a second continuous odometry source. Keep scan
      // matching alive for diagnostics/relocalization, but do not drive UKF.
      active_source_ = "unavailable";
    } else if (last_ndt_healthy_) {
      absolute_pose_valid = true;
      active_source_ = "ndt_imu";
      absolute_observation_updated = run_ndt;
      applyRtkHeadingObservation(rtk_observation);
      rtk_position_fused_this_frame_ = applyGnssCorrection(rclcpp::Time(stamp));
    } else if (rtkGoodForNavigation(rtk_observation)) {
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

    // Registration may report a plausible single frame before the initialization
    // gate has accumulated its three stable observations. Never expose that as a
    // normal navigation pose; doing so lets Nav2 start on an unverified map/yaw.
    if (!is_init_success_) {
      absolute_pose_valid = false;
      bridge_pose_valid = false;
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
        last_rtk_primary_applied_stamp_ns_ = 0;
        last_rtk_aux_observation_stamp_ns_ = 0;
        last_absolute_observation_stamp_ns_ = 0;
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
      localization_state_ = is_init_success_ ? 4 : 1;
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
      if (!bridge_active_ && !rtkCorrectionQualityOk(rtk_observation) && !lio_observation_applied &&
          is_init_success_ && has_valid_pose_history_ &&
          !runtime_relocalization_attempted_ &&
          consecutive_match_failures_ >= runtime_relocalization_failure_threshold_) {
        advanceGlobalRelocalizationGeneration("runtime localization loss");
        last_init_pos_ = last_pose_.block<3, 1>(0, 3);
        last_init_quat_ = Eigen::Quaternionf(last_pose_.block<3, 3>(0, 0));
        last_init_quat_.normalize();
        has_set_init_pose_ = true;
        last_pose_source_ = "runtime_last_valid";
        relocalization_attempt_phase_ = "trusted_pose";
        relocalization_attempt_index_ = 0;
        relocalization_attempt_total_ = 0;
        relocalization_best_source_ = "trusted_pose";
        relocalization_best_score_ = -1.0;
        is_init_success_ = false;
        global_search_required_ = false;
        global_candidate_applied_ = false;
        resetInitializationValidation();
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

    const auto output_start = SteadyClock::now();
    if (run_ndt && aligned_pub->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header = points_msg->header;
      aligned_msg.header.frame_id = "map";
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
    // In FAST-LIO-primary mode the timer is the sole continuous output path:
    // map_T_base(t) = map_T_lio(anchor) * lio_T_base(t).  Publishing the NDT
    // estimator here as well would interleave a second pose stream on the same
    // topic and reintroduce jumps at scan-matching frequency.  Legacy mode
    // retains the callback-driven publication behavior.
    if (!enable_lio_primary_) {
      publish_odometry(
        points_msg->header.stamp,
        (absolute_pose_valid || bridge_pose_valid)
          ? pose_estimator->matrix()
          : (has_valid_pose_history_ ? last_pose_ : pose_estimator->matrix()));
    }
    syncLidarOdometryImuAnchor();
    perf.stage_ms[kOutput] = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - output_start).count();
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
    if (!pose_changed && localization_state_ != 3) {
      // An operator may deliberately retry the same map pose after a failed
      // LIO handoff. Treat it as a new acquisition while localization is not
      // normal; silently ignoring it leaves Edge waiting on stale state.
      pose_changed = true;
      RCLCPP_INFO(
        get_logger(),
        "Initial pose unchanged but localization state=%d; restarting verification",
        localization_state_);
    }
    
    if (pose_changed) {
      advanceGlobalRelocalizationGeneration(
        "initial pose update", initial_pose_relocalization_settle_s_);
      last_init_pos_ = new_pos;
      last_init_quat_ = new_quat;
      has_set_init_pose_ = true;
      last_pose_source_ = "Callback";
      relocalization_attempt_phase_ = "manual_point";
      relocalization_attempt_index_ = 0;
      relocalization_attempt_total_ = 0;
      relocalization_best_source_ = "manual_point";
      relocalization_best_score_ = -1.0;
      pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
      has_trusted_ndt_pose_ = false;
      resetLioAnchor();
      // Restart verification and allow one bounded ICP refinement from the
      // operator-provided pose. Local NDT alone can only recover small pose
      // errors, while the manual pose is often only approximate.
      is_init_success_ = false;
      global_search_required_ = false;
      global_candidate_applied_ = false;
      resetInitializationValidation();
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

  pcl::PointCloud<PointT>::Ptr preprocessScanForMapMatching(
      const pcl::PointCloud<PointT>::Ptr& cloud) const {
    pcl::PointCloud<PointT>::Ptr cropped(new pcl::PointCloud<PointT>());
    if (!cloud) {
      return cropped;
    }
    cropped->reserve(cloud->size());
    const double min_range_sq = scan_preprocess_min_range_m_ * scan_preprocess_min_range_m_;
    const double max_range_sq = scan_preprocess_max_range_m_ * scan_preprocess_max_range_m_;
    const double half_fov_rad = scan_preprocess_fov_degree_ * 0.5 * M_PI / 180.0;
    const bool mask_fov = scan_preprocess_fov_degree_ < 359.9;
    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double range_sq = static_cast<double>(point.x) * point.x
        + static_cast<double>(point.y) * point.y
        + static_cast<double>(point.z) * point.z;
      if (range_sq <= min_range_sq || range_sq > max_range_sq) {
        continue;
      }
      if (mask_fov && std::fabs(std::atan2(
          static_cast<double>(point.y), static_cast<double>(point.x))) > half_fov_rad) {
        continue;
      }
      cropped->push_back(point);
    }
    cropped->width = static_cast<std::uint32_t>(cropped->size());
    cropped->height = 1;
    cropped->is_dense = cloud->is_dense;
    if (!scan_ground_filter_enable_ || cropped->empty()) {
      return cropped;
    }

    Eigen::Vector3d gravity_up_lidar = Eigen::Vector3d::UnitZ();
    {
      std::lock_guard<std::mutex> lock(lio_odom_mutex_);
      if (has_lio_odom_) {
        const auto& value = latest_lio_odom_.pose.pose.orientation;
        Eigen::Quaterniond orientation(value.w, value.x, value.y, value.z);
        if (orientation.coeffs().allFinite() && orientation.norm() > 1e-6) {
          orientation.normalize();
          gravity_up_lidar = orientation.toRotationMatrix().transpose()
            * Eigen::Vector3d::UnitZ();
        }
      }
    }
    gravity_up_lidar.normalize();
    pcl::PointCloud<PointT>::Ptr candidates(new pcl::PointCloud<PointT>());
    candidates->reserve(cropped->size() / 2);
    for (const auto& point : cropped->points) {
      const double vertical = gravity_up_lidar.dot(
        Eigen::Vector3d(point.x, point.y, point.z));
      if (vertical <= -scan_ground_min_sensor_height_m_
          && vertical >= -scan_ground_max_sensor_height_m_) {
        candidates->push_back(point);
      }
    }
    if (static_cast<int>(candidates->size()) < scan_ground_min_inliers_) {
      return cropped;
    }

    pcl::SACSegmentation<PointT> segmentation;
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(gravity_up_lidar.cast<float>());
    segmentation.setEpsAngle(scan_ground_max_tilt_deg_ * M_PI / 180.0);
    segmentation.setDistanceThreshold(scan_ground_distance_threshold_m_);
    segmentation.setMaxIterations(80);
    segmentation.setInputCloud(candidates);
    segmentation.segment(inliers, coefficients);
    if (static_cast<int>(inliers.indices.size()) < scan_ground_min_inliers_
        || coefficients.values.size() < 4) {
      return cropped;
    }

    Eigen::Vector3d normal(
      coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    double offset = coefficients.values[3];
    const double normal_norm = normal.norm();
    if (!normal.allFinite() || !std::isfinite(offset) || normal_norm < 1e-6) {
      return cropped;
    }
    normal /= normal_norm;
    offset /= normal_norm;
    if (normal.dot(gravity_up_lidar) < 0.0) {
      normal = -normal;
      offset = -offset;
    }
    const double sensor_height = offset;
    if (normal.dot(gravity_up_lidar) < std::cos(scan_ground_max_tilt_deg_ * M_PI / 180.0)
        || sensor_height < scan_ground_min_sensor_height_m_
        || sensor_height > scan_ground_max_sensor_height_m_) {
      return cropped;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cropped->size());
    for (const auto& point : cropped->points) {
      const double signed_height = normal.dot(Eigen::Vector3d(point.x, point.y, point.z)) + offset;
      if (signed_height >= -scan_ground_distance_threshold_m_
          && signed_height <= scan_ground_clearance_m_) {
        continue;
      }
      filtered->push_back(point);
    }
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = cropped->is_dense;
    return filtered->size() >= 20 ? filtered : cropped;
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
      if (fusion_profile_.load() == kFusionProfileLioHold) {
        for (auto & value : odom.pose.covariance) {
          value *= lio_hold_covariance_scale_;
        }
      }
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

  bool runGlobalLocalizationIcp(
    const GlobalRelocalizationJob& job,
    const Eigen::Matrix4d& seed,
    Eigen::Matrix4d& final_pose) {
    if (!global_localization_ptr_ || !job.map || job.map->empty() ||
        !job.cloud || job.cloud->empty()) {
      return false;
    }
    try {
      return global_localization_ptr_->performGlobalLocalization(
        job.map, job.cloud, seed, final_pose);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Global localization worker exception: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Global localization worker aborted with unknown exception");
    }
    return false;
  }

  GlobalRelocalizationResult executeGlobalRelocalizationJob(
    const GlobalRelocalizationJob& job) {
    const auto started = SteadyClock::now();
    GlobalRelocalizationResult result;
    result.generation = job.generation;
    RCLCPP_INFO(
      get_logger(), "Starting background global localization generation=%llu",
      static_cast<unsigned long long>(job.generation));

    // Deterministic local progressive search. Operator/manual seed (or the
    // current recovery seed) wins first, followed by the last trusted pose,
    // map origin and a small origin neighbourhood. Scan-context route/keyframe
    // candidates are evaluated only after these bounded seeds fail.
    relocalization_attempt_phase_ = "progressive_search";
    relocalization_attempt_total_ = static_cast<int>(job.progressive_seeds.size());
    relocalization_attempt_index_ = 0;
    for (const auto & seed : job.progressive_seeds) {
      ++relocalization_attempt_index_;
      relocalization_attempt_phase_ = seed.first;
      RCLCPP_INFO(
        get_logger(), "Relocalization attempt %d/%d phase=%s seed=[%.3f, %.3f]",
        relocalization_attempt_index_, relocalization_attempt_total_, seed.first.c_str(),
        seed.second(0, 3), seed.second(1, 3));
      Eigen::Matrix4d candidate_pose = Eigen::Matrix4d::Identity();
      if (runGlobalLocalizationIcp(job, seed.second, candidate_pose)) {
        result.success = true;
        result.pose = candidate_pose;
        result.source = "progressive_" + seed.first;
        result.seed_source = seed.first;
        result.attempts = relocalization_attempt_index_;
        relocalization_best_source_ = seed.first;
        relocalization_attempt_phase_ = "progressive_success";
        break;
      }
    }

    if (!result.success && job.use_scan_context) {
      std::vector<ScanContextCandidate> candidates;
      {
        std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
        if (!scan_context_db_.empty()) {
          const auto descriptor = scan_context_db_.describe(*job.cloud);
          candidates = scan_context_db_.queryDescriptor(
            descriptor, scan_context_top_k_, scan_context_max_distance_, -1, 0,
            ScanContextDistanceMetric::kMeanAbsoluteHeight,
            scan_context_prefilter_candidates_);
        }
      }
      if (candidates.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Scan context returned no candidate; falling back to the last trusted pose");
      } else {
        std::size_t cursor = 0;
        {
          std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
          if (scan_context_cursor_ >= candidates.size()) {
            scan_context_cursor_ = 0;
          }
          cursor = scan_context_cursor_;
        }
        const int attempts = std::min<int>(scan_context_max_seeds_, candidates.size());
        RelocalizationGeometryVerifier geometry_verifier(relocalization_geometry_config_);
        for (int attempt = 0; attempt < attempts; ++attempt) {
          const auto candidate = candidates[cursor];
          cursor = (cursor + 1) % candidates.size();
          RCLCPP_INFO(
            get_logger(),
            "Scan context candidate %d/%zu: keyframe %d, distance %.3f, yaw %+.1f deg, "
            "seed [%.2f, %.2f, %.2f]",
            attempt + 1, candidates.size(), candidate.keyframe_index, candidate.distance,
            candidate.yaw_offset_rad * 180.0 / M_PI,
            candidate.seed_pose(0, 3), candidate.seed_pose(1, 3), candidate.seed_pose(2, 3));
          pcl::PointCloud<PointT> candidate_scan;
          Eigen::Matrix4d candidate_map_pose = Eigen::Matrix4d::Identity();
          bool candidate_loaded = false;
          std::string load_error;
          {
            std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
            std::size_t candidate_slot = 0;
            if (scan_context_db_.findKeyframeSlot(candidate.keyframe_index, candidate_slot)) {
              candidate_map_pose = scan_context_db_.keyframePose(candidate_slot);
              candidate_loaded = scan_context_db_.loadKeyframeSubmap(
                candidate_slot, scan_context_target_half_window_, candidate_scan, &load_error);
            } else {
              load_error = "candidate keyframe index is absent from the database";
            }
          }
          if (!candidate_loaded) {
            RCLCPP_WARN(get_logger(),
              "Scan context geometry candidate %d could not load: %s",
              candidate.keyframe_index, load_error.c_str());
            continue;
          }

          RelocalizationGeometryResult geometry;
          Eigen::Matrix4d accepted_seed = candidate.seed_pose;
          bool geometry_accepted = false;
          const double sector_yaw = 2.0 * M_PI /
            static_cast<double>(std::max(1, scan_context_db_.params().sectors));
          for (int yaw_neighbor = -scan_context_yaw_neighbors_;
               yaw_neighbor <= scan_context_yaw_neighbors_; ++yaw_neighbor) {
            Eigen::Matrix4d yaw_adjusted_seed = candidate.seed_pose;
            const double adjusted_yaw = candidate.yaw_offset_rad + yaw_neighbor * sector_yaw;
            yaw_adjusted_seed.block<3, 3>(0, 0) =
              Eigen::AngleAxisd(adjusted_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
              candidate_map_pose.block<3, 3>(0, 0);
            const Eigen::Matrix4f initial_source_to_target =
              (candidate_map_pose.inverse() * yaw_adjusted_seed).cast<float>();
            geometry = geometry_verifier.verify(
              *job.cloud, candidate_scan, initial_source_to_target);
            if (geometry.accepted) {
              accepted_seed = candidate_map_pose * geometry.source_to_target.cast<double>();
              geometry_accepted = true;
              break;
            }
          }
          result.candidate_keyframe = candidate.keyframe_index;
          result.candidate_distance = candidate.distance;
          result.candidate_yaw_deg = candidate.yaw_offset_rad * 180.0 / M_PI;
          result.candidate_rmse_m = geometry.rmse_m;
          result.candidate_overlap = geometry.bidirectional_overlap;
          result.rejection_reason = geometry.rejection_reason.empty()
            ? "none" : geometry.rejection_reason;
          RCLCPP_INFO(get_logger(),
            "Scan context geometry keyframe=%d accepted=%s reason=%s "
            "rmse=%.3fm inliers=%d overlap=%.3f hessian=%.1f "
            "icp_delta=%.3fm/%.2fdeg elapsed=%.1fms mode=%s",
            candidate.keyframe_index, geometry_accepted ? "true" : "false",
            geometry.rejection_reason.empty() ? "none" : geometry.rejection_reason.c_str(),
            geometry.rmse_m, geometry.inlier_count, geometry.bidirectional_overlap,
            geometry.hessian_condition, geometry.icp_translation_disagreement_m,
            geometry.icp_rotation_disagreement_rad * 180.0 / M_PI,
            geometry.elapsed_ms, scan_context_effective_runtime_mode_.c_str());
          if (!geometry_accepted) {
            continue;
          }
          result.candidate_accepted = true;
          result.seed_source = "route_keyframe";
          result.attempts = relocalization_attempt_index_ + attempt + 1;
          relocalization_attempt_phase_ = "route_progressive";
          result.rejection_reason = job.apply_scan_context ? "none" : "shadow_not_applied";
          if (!job.apply_scan_context) {
            break;
          }
          if (runGlobalLocalizationIcp(job, accepted_seed, result.pose)) {
            result.success = true;
            result.source = "scan_context_gicp_verified";
            break;
          }
        }
        {
          std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
          scan_context_cursor_ = cursor;
        }
      }
    }

    if (!result.success && job.allow_fallback &&
        runGlobalLocalizationIcp(job, job.fallback_seed, result.pose)) {
      result.success = true;
      result.source = "last_trusted_pose";
    }
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      SteadyClock::now() - started).count();
    return result;
  }

  void globalRelocalizationWorkerLoop() {
    while (true) {
      GlobalRelocalizationJob job;
      {
        std::unique_lock<std::mutex> lock(global_relocalization_job_mutex_);
        global_relocalization_job_cv_.wait(lock, [this]() {
          return global_relocalization_worker_stop_ || pending_global_relocalization_job_.has_value();
        });
        if (global_relocalization_worker_stop_) {
          return;
        }
        job = std::move(*pending_global_relocalization_job_);
        pending_global_relocalization_job_.reset();
      }

      auto result = executeGlobalRelocalizationJob(job);
      RCLCPP_INFO(
        get_logger(),
        "Background global localization finished generation=%llu success=%s source=%s elapsed=%.1fms",
        static_cast<unsigned long long>(result.generation),
        result.success ? "true" : "false", result.source.c_str(), result.elapsed_ms);
      {
        std::lock_guard<std::mutex> lock(global_relocalization_result_mutex_);
        pending_global_relocalization_result_ = std::move(result);
      }
      {
        std::lock_guard<std::mutex> lock(global_relocalization_job_mutex_);
        if (!pending_global_relocalization_job_) {
          global_localization_in_progress_.store(false);
        }
      }
    }
  }

  void scheduleGlobalRelocalization(const pcl::PointCloud<PointT>::Ptr& current_cloud) {
    GlobalRelocalizationJob job;
    job.generation = global_relocalization_generation_;
    job.cloud.reset(new pcl::PointCloud<PointT>(*current_cloud));
    job.map = global_map_points_ptr_;
    job.fallback_seed.block<3, 1>(0, 3) = last_init_pos_.cast<double>();
    job.fallback_seed.block<3, 3>(0, 0) = last_init_quat_.toRotationMatrix().cast<double>();
    auto append_seed = [&job](const std::string & source, const Eigen::Matrix4d & pose) {
        for (const auto & existing : job.progressive_seeds) {
          if ((existing.second.block<3, 1>(0, 3) - pose.block<3, 1>(0, 3)).norm() < 0.05) {
            return;
          }
        }
        job.progressive_seeds.emplace_back(source, pose);
      };
    // Priority is intentional: the latest operator seed is the strongest
    // semantic hint, then a trusted live pose, then the map origin and nearby
    // offsets before route/keyframe global matching.
    const bool manual_seed = last_pose_source_ == "Callback";
    if (manual_seed) {
      append_seed("manual_point", job.fallback_seed);
    } else if (has_valid_pose_history_) {
      append_seed("trusted_pose", job.fallback_seed);
    } else {
      append_seed("map_origin", job.fallback_seed);
    }
    if (has_valid_pose_history_ && manual_seed) {
      Eigen::Matrix4d trusted = last_pose_.cast<double>();
      append_seed("trusted_pose", trusted);
    }
    Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
    origin.block<3, 1>(0, 3) = Eigen::Vector3d(init_pos_x_, init_pos_y_, init_pos_z_);
    origin.block<3, 3>(0, 0) = Eigen::Quaterniond(
      init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_).normalized().toRotationMatrix();
    append_seed("map_origin", origin);
    for (const auto & offset : std::array<std::pair<double, double>, 4>{
      std::pair<double, double>{1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}}) {
      Eigen::Matrix4d nearby = origin;
      nearby(0, 3) += offset.first;
      nearby(1, 3) += offset.second;
      append_seed("origin_nearby", nearby);
    }
    relocalization_attempt_phase_ = "queued_progressive_search";
    relocalization_attempt_index_ = 0;
    relocalization_attempt_total_ = static_cast<int>(job.progressive_seeds.size());
    job.use_scan_context = scan_context_effective_runtime_mode_ != "disabled";
    job.apply_scan_context = scanContextApplyAllowed(
      scan_context_effective_runtime_mode_, job.generation,
      explicit_global_relocalization_generation_);
    job.allow_fallback = !global_search_required_;
    global_relocalization_state_ = "searching";
    {
      std::lock_guard<std::mutex> lock(global_relocalization_job_mutex_);
      // Capacity-one mailbox: a newer request replaces a queued request. The
      // active job may finish, but its generation is checked before write-back.
      pending_global_relocalization_job_ = std::move(job);
      global_localization_in_progress_.store(true);
      global_localization_start_time_ = get_clock()->now();
    }
    global_relocalization_job_cv_.notify_one();
  }

  void advanceGlobalRelocalizationGeneration(
    const char* reason, double settle_seconds = 0.0) {
    ++global_relocalization_generation_;
    explicit_global_relocalization_generation_ = 0;
    global_relocalization_earliest_start_ns_ = steadyNowNanoseconds() +
      static_cast<std::int64_t>(std::max(0.0, settle_seconds) * 1e9);
    {
      std::lock_guard<std::mutex> lock(global_relocalization_job_mutex_);
      if (pending_global_relocalization_job_ &&
          pending_global_relocalization_job_->generation != global_relocalization_generation_) {
        pending_global_relocalization_job_.reset();
      }
    }
    {
      std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
      scan_context_cursor_ = 0;
    }
    RCLCPP_INFO(
      get_logger(), "Global localization generation advanced to %llu (%s, settle=%.2fs)",
      static_cast<unsigned long long>(global_relocalization_generation_), reason,
      std::max(0.0, settle_seconds));
  }

  void applyPendingGlobalRelocalizationResult() {
    std::optional<GlobalRelocalizationResult> result;
    {
      std::lock_guard<std::mutex> lock(global_relocalization_result_mutex_);
      result.swap(pending_global_relocalization_result_);
    }
    if (!result) {
      return;
    }
    global_candidate_keyframe_ = result->candidate_keyframe;
    global_candidate_distance_ = result->candidate_distance;
    global_candidate_yaw_deg_ = result->candidate_yaw_deg;
    global_candidate_rmse_m_ = result->candidate_rmse_m;
    global_candidate_overlap_ = result->candidate_overlap;
    global_candidate_rejection_reason_ = result->rejection_reason;
    const bool explicit_operator_result =
      result->generation == explicit_global_relocalization_generation_;
    if (result->candidate_accepted && scan_context_effective_runtime_mode_ == "shadow" &&
        !explicit_operator_result) {
      global_relocalization_state_ = "shadow_candidate_verified";
      initialization_state_ = global_search_required_ ? "shadow_blocked" : initialization_state_;
    }
    const auto disposition = decideGlobalRelocalizationResult(
      global_relocalization_generation_, result->generation, result->success,
      result->elapsed_ms, global_localization_timeout_);
    if (disposition == GlobalRelocalizationResultDisposition::kDiscardStale) {
      RCLCPP_WARN(
        get_logger(),
        "Discarding stale global localization result generation=%llu current=%llu",
        static_cast<unsigned long long>(result->generation),
        static_cast<unsigned long long>(global_relocalization_generation_));
      return;
    }
    if (disposition == GlobalRelocalizationResultDisposition::kDiscardTimedOut) {
      RCLCPP_WARN(
        get_logger(), "Discarding timed-out global localization result elapsed=%.1fms limit=%.1fms",
        result->elapsed_ms, global_localization_timeout_ * 1000.0f);
      return;
    }
    if (disposition == GlobalRelocalizationResultDisposition::kApplyFailure) {
      RCLCPP_WARN(
        get_logger(),
        "Background global localization failed once; keeping the last trusted pose");
      if (!result->candidate_accepted) {
        global_relocalization_state_ = "failed";
      }
      if (!global_search_required_ && runtime_relocalization_attempted_ && has_valid_pose_history_) {
        pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
        is_extrapolating_ = true;
      }
      return;
    }

    last_init_pos_ = result->pose.block<3, 1>(0, 3).cast<float>();
    last_init_quat_ = Eigen::Quaternionf(result->pose.block<3, 3>(0, 0).cast<float>());
    last_init_quat_.normalize();
    has_set_init_pose_ = true;
    last_pose_source_ = "global_localization_" + result->source;
    pose_estimator = createPoseEstimator(last_init_pos_, last_init_quat_);
    has_trusted_ndt_pose_ = false;
    resetLioAnchor();
    is_init_success_ = false;
    resetInitializationValidation("validating_global_candidate");
    global_candidate_applied_ = true;
    global_relocalization_state_ = "candidate_applied";
    relocalization_attempt_phase_ = result->seed_source.empty()
      ? "route_candidate_applied" : result->seed_source + "_applied";
    relocalization_best_source_ = result->seed_source.empty() ? result->source : result->seed_source;
    localization_state_ = 1;
    RCLCPP_INFO(
      get_logger(),
      "Applied background global localization generation=%llu source=%s pose=[%.3f, %.3f, %.3f]",
      static_cast<unsigned long long>(result->generation), result->source.c_str(),
      last_init_pos_.x(), last_init_pos_.y(), last_init_pos_.z());
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
       << " (" << std::fixed << std::setprecision(1) << (lidar_valid_ratio * 100.0) << "%)";
    if (!use_imu) {
      // Raw IMU is intentionally not subscribed under lio_primary; FAST-LIO
      // already couples Mid-360 IMU into /odom/lio_odom.
      ss << " | IMU: n/a (via LIO)";
    } else {
      ss << " | IMU: " << imu_valid_count << "/" << imu_total_count 
         << " (" << std::fixed << std::setprecision(1) << (imu_valid_ratio * 100.0) << "%)";
    }
    ss << " | Pose History: " << (has_valid_pose_history_ ? "Available" : "Not Available")
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
      last_ndt_inlier_fraction_ = 0.0f;
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
      // A converged optimizer result without a target search tree has no
      // geometric inlier evidence.  Treat it as invalid instead of promoting
      // it to 100% inliers, otherwise a stale/empty map could authorize an
      // NDT drift correction.
      status.inlier_fraction = 0.0f;
      last_ndt_status_healthy_ = false;
      last_ndt_inlier_fraction_ = 0.0f;
      status.has_converged = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000.0,
        "Scan match rejected: target search tree unavailable, score=%.3f",
        status.matching_error);
      status_pub->publish(status);
      return;
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
    last_ndt_inlier_fraction_ = status.inlier_fraction;
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
        maybeExpireFusionProfile();
        auto current_time = this->get_clock()->now();
        updateConfidence(current_time);
        if (lidar_status_buffer_.size() > 0) {
            double lidar_time_diff = (current_time - last_lidar_data_time_).seconds();
            if (lidar_time_diff > sensor_timeout_threshold_) {
                updateLidarStatus(false);
            }
        }  
        if (use_imu && imu_status_buffer_.size() > 0) {
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
        if (lio_motion_anomaly_active_.load()) {
            msg.status = 4;
        } else if (localization_state_ == 0) {
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

        // FAST-LIO is the only continuous motion source. Publish every timer
        // tick from the latest local pose and the slowly changing map->lio
        // anchor, independently of the lower-rate NDT point-cloud callback.
        if (enable_lio_primary_ && !lioHandoffSnapshot().pending && lio_anchor_valid_.load() &&
            !lio_motion_anomaly_active_) {
            const rclcpp::Time now = this->get_clock()->now();
            Eigen::Isometry3f lio_pose = Eigen::Isometry3f::Identity();
            if (lioOdomFresh(now) && currentLioPose(lio_pose)) {
                const Eigen::Matrix4f map_pose =
                    (mapToLioAnchorSnapshot() * lio_pose).matrix();
                publish_odometry(now, map_pose);
                active_source_ = "lio_imu";
                return;
            }
            active_source_ = "unavailable";
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
     * Same convention loadGnssOriginForMap already uses: resolve map.pcd so the
     * session directory is the canonical parent, not the map-root symlink.
     * A missing or unreadable database is not an error - relocalization
     * simply keeps the stock last-trusted-pose seed.
     */
    void loadScanContextForMap(const std::string& map_path) {
        if (!use_scan_context_) {
            std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
            scan_context_db_.clear();
            scan_context_cursor_ = 0;
            scan_context_effective_runtime_mode_ = "disabled";
            return;
        }
        std::error_code canonical_error;
        const std::string map_dir = std::filesystem::weakly_canonical(
          std::filesystem::path(map_path), canonical_error).parent_path().string();
        const bool map_allowed = scan_context_active_map_paths_.empty() || std::any_of(
          scan_context_active_map_paths_.begin(), scan_context_active_map_paths_.end(),
          [&](const std::string& allowed_path) {
            std::error_code allowed_error;
            return std::filesystem::weakly_canonical(allowed_path, allowed_error).string() == map_dir;
          });
        if (!map_allowed) {
            std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
            scan_context_db_.clear();
            scan_context_cursor_ = 0;
            scan_context_effective_runtime_mode_ = "disabled";
            RCLCPP_INFO(get_logger(),
              "Scan context rollout disabled for non-allowlisted map %s", map_dir.c_str());
            return;
        }
        scan_context_effective_runtime_mode_ = scan_context_runtime_mode_;
        ScanContextDatabase loaded_database;
        std::string error;
        if (!loaded_database.load(map_dir, &error)) {
            {
                std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
                scan_context_db_.clear();
                scan_context_cursor_ = 0;
                scan_context_effective_runtime_mode_ = "disabled";
            }
            RCLCPP_WARN(get_logger(), "Scan context relocalization unavailable for %s: %s",
                        map_dir.c_str(), error.c_str());
            return;
        }
        const std::size_t loaded_size = loaded_database.size();
        const std::string seed_source = loaded_database.seed_pose_source();
        {
            std::lock_guard<std::mutex> resource_lock(global_relocalization_resource_mutex_);
            scan_context_db_ = std::move(loaded_database);
            scan_context_cursor_ = 0;
        }
        RCLCPP_INFO(get_logger(),
                    "Scan context database loaded: %zu keyframes from %s (seed poses: %s)",
                    loaded_size, map_dir.c_str(), seed_source.c_str());
    }

    void LoadMapCallBack(robots_dog_msgs::srv::LoadMap::Request::SharedPtr request, 
            robots_dog_msgs::srv::LoadMap::Response::SharedPtr response) {
        update_map_flag_.store(true);
        std::string map_path = request->pcd_path;

        if (!std::filesystem::exists(std::filesystem::path(map_path))) {
            RCLCPP_ERROR(get_logger(), "Map file does not exist: %s", map_path.c_str());
            response->success = false;
            response->message = "Map file does not exist.";
            update_map_flag_.store(false);
            return;
        }
        pcl::PointCloud<PointT>::Ptr loaded_map(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(map_path, *loaded_map);
        RCLCPP_INFO(get_logger(), "Global map points size==: %zu", loaded_map->points.size());
        if (!loaded_map->empty()) {
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(loaded_map);
            voxel.setLeafSize(globalmap_voxel_size_, globalmap_voxel_size_, globalmap_voxel_size_);
            voxel.filter(*loaded_map);
            if (loaded_map->points.size() < 1000) {
                RCLCPP_ERROR(get_logger(), "Global map points size is too small: %zu", loaded_map->points.size());
                response->success = false;
                response->message = "Global map points size is too small.";
                update_map_flag_.store(false);
                return;
            } else {
                if (global_map_pub_->get_subscription_count()) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::VoxelGrid<pcl::PointXYZI> publish_voxel;
                    publish_voxel.setInputCloud(loaded_map);
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
                // Publish the new immutable snapshot atomically at the callback
                // boundary. A running background ICP retains the old shared_ptr
                // and its generation prevents that old result from being applied.
                global_map_points_ptr_ = loaded_map;
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
        advanceGlobalRelocalizationGeneration("map reset", map_relocalization_settle_s_);
        is_init_success_ = false;
        localization_state_ = 0; 
        global_candidate_applied_ = false;
        gl_once_gate_ = true;
        consecutive_match_failures_ = 0;
        runtime_relocalization_attempted_ = false;
        gnss_recovery_seed_pending_ = false;
        last_gnss_recovery_attempt_ = std::chrono::steady_clock::time_point{};
        has_set_init_pose_ = false;
        last_init_pos_ = Eigen::Vector3f(init_pos_x_, init_pos_y_, init_pos_z_);
        last_init_quat_ = Eigen::Quaternionf(init_ori_w_, init_ori_x_, init_ori_y_, init_ori_z_);
        relocalization_attempt_phase_ = "map_origin";
        relocalization_attempt_index_ = 0;
        relocalization_attempt_total_ = 0;
        relocalization_best_source_ = "map_origin";
        relocalization_best_score_ = -1.0;
        const bool seeded_from_rtk = seedPositionFromGnss(
          get_clock()->now(), "map initialization", true, true);
        global_search_required_ = scan_context_effective_runtime_mode_ != "disabled" &&
          !seeded_from_rtk;
        resetInitializationValidation(
          global_search_required_ ? "global_search_required" : "validating");
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                              global_relocalize_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                reinitialize_global_localization_service_;
  rclcpp::Service<robots_dog_msgs::srv::ControlLocalizationCorrection>::SharedPtr control_localization_correction_service_;
  rclcpp::Service<robots_dog_msgs::srv::SetLocalizationFusionProfile>::SharedPtr set_localization_fusion_profile_service_;
  rclcpp::TimerBase::SharedPtr localization_lidar_info_timer_;
  rclcpp::TimerBase::SharedPtr odom_publish_timer_; 

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr               lidar_odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            robot_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            lio_odom_sub_;
  rclcpp::CallbackGroup::SharedPtr                                    lio_callback_group_;
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
  bool prefer_fixed_rtk_ = false;
  bool rtk_primary_allowed_by_policy_ = false;
  int rtk_primary_promote_samples_ = 20;
  int rtk_primary_demote_samples_ = 8;
  double rtk_primary_handoff_suppress_s_ = 2.5;
  std::int64_t rtk_primary_handoff_suppress_until_ns_ = 0;
  int rtk_auto_primary_good_frames_ = 0;
  int rtk_auto_primary_bad_frames_ = 0;
  bool rtk_auto_primary_latched_ = false;
  bool lidar_matching_paused_for_rtk_ = false;
  std::string preferred_source_ = "ndt";
  CorrectionPolicyMode preferred_correction_mode_ = CorrectionPolicyMode::ndt;
  std::string ukf_anchor_preference_ = "balanced";
  bool policy_source_ready_ = false;
  std::string last_correction_candidate_source_ = "none";
  std::string last_correction_selection_reason_ = "idle";
  std::string last_ndt_score_band_ = "unavailable";
  float last_float_rtk_residual_xy_m_ = -1.0f;
  bool last_float_rtk_within_gate_ = false;
  std::string last_float_rtk_gate_reason_ = "not_evaluated";
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
  double anchor_yaw_variance_per_rad_ = 0.01;
  static constexpr uint8_t kFusionProfileNominal = 0;
  static constexpr uint8_t kFusionProfileLioHold = 1;
  static constexpr uint8_t kFusionProfileBalanced = 2;
  std::atomic<uint8_t> fusion_profile_{kFusionProfileNominal};
  std::atomic<std::int64_t> fusion_profile_expires_steady_ns_{0};
  std::atomic<uint64_t> fusion_profile_generation_{0};
  double lio_hold_covariance_scale_ = 4.0;
  double balanced_observation_variance_scale_ = 4.0;
  double bridge_max_odom_speed_mps_ = 1.5;
  double bridge_max_odom_yaw_rate_rps_ = 2.0;
  int absolute_recovery_samples_ = 3;
  int moving_ndt_stride_ = 5;
  int stationary_ndt_stride_ = 5;
  int initialization_ndt_stride_ = 2;
  double stable_ndt_max_rate_hz_ = 2.0;
  double recovery_ndt_max_rate_hz_ = 5.0;
  int ndt_failure_hysteresis_frames_ = 3;
  int ndt_unhealthy_frame_count_ = 0;
  uint64_t ndt_frame_counter_ = 0;
  bool last_ndt_healthy_ = false;
  bool last_ndt_status_healthy_ = false;
  double last_ndt_score_ = -1.0;
  float last_ndt_inlier_fraction_ = 0.0f;
  float last_ndt_drift_x_m_ = 0.0f;
  float last_ndt_drift_y_m_ = 0.0f;
  float last_ndt_drift_xy_m_ = 0.0f;
  float last_ndt_drift_yaw_rad_ = 0.0f;
  float last_ndt_orientation_delta_rad_ = 0.0f;
  rclcpp::Time last_ndt_update_time_{0, 0, RCL_ROS_TIME};
  std::int64_t last_ndt_match_start_steady_ns_ = 0;
  std::string last_point_cloud_schedule_reason_ = "idle";
  Eigen::Vector3f last_rtk_map_position_ = Eigen::Vector3f::Zero();
  double last_rtk_map_yaw_ = 0.0;
  int absolute_stable_count_ = 0;
  bool absolute_stable_ = false;
  std::string stable_source_;
  std::string bridge_rejection_reason_;
  std::string odom_time_source_ = "not_used";
  int64_t last_absolute_observation_stamp_ns_ = 0;
  int64_t last_rtk_primary_applied_stamp_ns_ = 0;
  int64_t last_rtk_aux_observation_stamp_ns_ = 0;
  
  // transformation matrices 
  Eigen::Matrix3f init_rotation_matrix_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f gravity_transform_    = Eigen::Matrix4f::Identity();
  
  // Global localization and pose management
  std::shared_ptr<GlobalLocalization> global_localization_ptr_;
  ScanContextDatabase scan_context_db_;
  bool use_scan_context_ = false;
  std::string scan_context_runtime_mode_ = "disabled";
  std::string scan_context_effective_runtime_mode_ = "disabled";
  std::vector<std::string> scan_context_active_map_paths_;
  RelocalizationGeometryConfig relocalization_geometry_config_;
  int scan_context_top_k_ = 5;
  int scan_context_max_seeds_ = 1;
  int scan_context_prefilter_candidates_ = 60;
  int scan_context_target_half_window_ = 1;
  int scan_context_yaw_neighbors_ = 2;
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
  std::atomic<bool> global_localization_in_progress_{false};
  rclcpp::Time global_localization_start_time_;
  std::uint64_t global_relocalization_generation_ = 1;
  // A nonzero matching generation grants apply permission only to a manual
  // /localization/global_relocalize request while automatic recovery remains shadow-only.
  std::uint64_t explicit_global_relocalization_generation_ = 0;
  std::mutex global_relocalization_job_mutex_;
  std::condition_variable global_relocalization_job_cv_;
  std::optional<GlobalRelocalizationJob> pending_global_relocalization_job_;
  bool global_relocalization_worker_stop_ = false;
  std::thread global_relocalization_worker_;
  std::mutex global_relocalization_result_mutex_;
  std::optional<GlobalRelocalizationResult> pending_global_relocalization_result_;
  std::mutex global_relocalization_resource_mutex_;
  bool gl_once_gate_ = true;
  int runtime_relocalization_failure_threshold_ = 10;
  double runtime_relocalization_retry_seconds_ = 5.0;
  double initial_pose_relocalization_settle_s_ = 0.75;
  double map_relocalization_settle_s_ = 2.0;
  std::int64_t global_relocalization_earliest_start_ns_ = 0;
  std::chrono::steady_clock::time_point last_global_localization_attempt_{};
  int consecutive_match_failures_ = 0;
  bool runtime_relocalization_attempted_ = false;
  bool global_search_required_ = false;
  bool global_candidate_applied_ = false;
  std::string global_relocalization_state_ = "idle";
  std::string relocalization_attempt_phase_ = "idle";
  std::string relocalization_best_source_ = "none";
  int relocalization_attempt_index_ = 0;
  int relocalization_attempt_total_ = 0;
  double relocalization_best_score_ = -1.0;
  int global_candidate_keyframe_ = -1;
  double global_candidate_distance_ = 0.0;
  double global_candidate_yaw_deg_ = 0.0;
  double global_candidate_rmse_m_ = 0.0;
  double global_candidate_overlap_ = 0.0;
  std::string global_candidate_rejection_reason_ = "none";
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
  float lio_max_yaw_step_rad_ = 30.0f * static_cast<float>(M_PI) / 180.0f;
  float lio_max_yaw_rate_radps_ = 60.0f * static_cast<float>(M_PI) / 180.0f;
  float lio_xy_variance_ = 0.0025f;
  float lio_z_variance_ = 0.010f;
  float lio_orientation_variance_ = 0.0004f;
  float lio_drift_xy_m_ = 0.30f;
  float lio_drift_yaw_rad_ = 5.0f * static_cast<float>(M_PI) / 180.0f;
  int lio_drift_hysteresis_frames_ = 3;
  int lio_stable_confirmation_frames_ = 3;
  float lio_stable_xy_tolerance_m_ = 0.10f;
  float lio_stable_yaw_tolerance_rad_ = 2.0f * static_cast<float>(M_PI) / 180.0f;
  float lio_rearm_xy_m_ = 0.20f;
  float lio_suppressed_covariance_scale_ = 100.0f;
  float lio_max_correction_jump_m_ = 1.50f;
  float lio_max_correction_yaw_rad_ = 30.0f * static_cast<float>(M_PI) / 180.0f;
  double rtk_trust_stable_window_s_ = 1.0;
  float rtk_trust_stable_span_m_ = 0.35f;
  float rtk_trusted_correction_translation_rate_mps_ = 1.0f;
  float rtk_trusted_correction_rotation_rate_radps_ =
    30.0f * static_cast<float>(M_PI) / 180.0f;
  bool rtk_self_stable_ = false;
  bool prefer_fixed_rtk_for_correction_ = false;
  std::deque<RtkStabilitySample> rtk_stability_window_;
  float lio_correct_xy_variance_ = 0.010f;
  float lio_correct_z_variance_ = 0.020f;
  float lio_correct_orientation_variance_ = 0.001f;
  bool lio_dynamic_covariance_enable_ = true;
  float lio_dynamic_ndt_good_score_ = 0.10f;
  float lio_dynamic_vgicp_good_score_ = 0.05f;
  float lio_dynamic_good_inlier_fraction_ = 0.50f;
  float lio_dynamic_xy_variance_max_ = 0.25f;
  float lio_dynamic_z_variance_max_ = 0.50f;
  float lio_dynamic_orientation_variance_max_ = 0.05f;
  float lio_correction_translation_rate_mps_ = 0.25f;
  float lio_correction_rotation_rate_radps_ = 8.0f * static_cast<float>(M_PI) / 180.0f;
  float lio_correction_completion_translation_m_ = 0.01f;
  float lio_correction_completion_rotation_rad_ = 0.25f * static_cast<float>(M_PI) / 180.0f;
  double lio_correction_min_interval_s_ = 3.0;
  double lio_correction_post_suppression_s_ = 3.0;
  int lio_stable_frame_count_ = 0;
  CorrectionCooldownGate lio_correction_cooldown_gate_;
  std::int64_t last_correction_started_steady_ns_ = 0;
  std::int64_t last_correction_completed_steady_ns_ = 0;
  std::string last_correction_source_ = "none";
  struct StampedLioPose {
    int64_t stamp_ns = 0;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  };
  mutable std::mutex lio_odom_mutex_;
  nav_msgs::msg::Odometry latest_lio_odom_;
  rclcpp::Time latest_lio_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool has_lio_odom_ = false;
  std::uint64_t lio_odom_sequence_ = 0;
  std::deque<StampedLioPose> lio_pose_history_;
  double lio_pose_history_seconds_ = 2.0;
  double lio_observation_sync_tolerance_s_ = 0.20;
  Eigen::Isometry3f anchor_prediction_lio_pose_ = Eigen::Isometry3f::Identity();
  bool has_anchor_prediction_lio_pose_ = false;
  std::atomic<bool> lio_anchor_valid_{false};
  mutable std::mutex lio_handoff_mutex_;
  bool lio_handoff_pending_ = false;
  std::uint64_t lio_handoff_generation_ = 0;
  std::uint64_t lio_handoff_required_sequence_ = 0;
  std::string lio_handoff_source_ = "none";
  std::string lio_handoff_failure_reason_ = "none";
  bool lio_has_previous_pose_ = false;
  bool lio_corrected_this_frame_ = false;
  PendingLioCorrection pending_lio_correction_;
  float last_correction_progress_ = 0.0f;
  float last_correction_xy_variance_ = 0.0f;
  float last_correction_z_variance_ = 0.0f;
  float last_correction_orientation_variance_ = 0.0f;
  float last_correction_quality_penalty_ = 0.0f;
  AuxiliaryDriftGate ndt_drift_gate_;
  AuxiliaryDriftGate rtk_drift_gate_;
  mutable std::mutex lio_anchor_mutex_;
  mutable std::mutex anchor_ukf_mutex_;
  AnchorUkf anchor_ukf_;
  Eigen::Vector3d last_anchor_innovation_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_anchor_gain_ = Eigen::Vector3d::Zero();
  std::string last_anchor_observation_source_ = "none";
  OneShotCorrection one_shot_correction_;
  Eigen::Isometry3f lio_map_T_lio_ = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f previous_lio_pose_ = Eigen::Isometry3f::Identity();
  mutable std::mutex lio_motion_guard_mutex_;
  std::optional<LioMotionSample> previous_lio_motion_sample_;
  std::atomic<bool> lio_motion_anomaly_active_{false};
  mutable std::mutex lio_motion_anomaly_mutex_;
  std::string lio_motion_anomaly_reason_ = "none";
  double lio_motion_anomaly_yaw_step_rad_ = 0.0;
  double lio_motion_anomaly_yaw_rate_radps_ = 0.0;
  rclcpp::Time last_lio_observation_stamp_{0, 0, RCL_ROS_TIME};
  float lidar_odom_voxel_size_ = 0.40f;
  float lidar_odom_max_correspondence_distance_ = 1.00f;
  float lidar_odom_max_fitness_score_ = 0.50f;
  float ndt_max_fitness_score_ = 0.40f;
  float ukf_high_quality_ndt_score_ = 0.10f;
  float ukf_float_max_residual_m_ = 0.40f;
  bool scan_matching_refine_enable_ = true;
  float scan_matching_local_map_xy_radius_ = 18.0f;
  float scan_matching_local_map_z_radius_ = 4.0f;
  int scan_matching_min_local_map_points_ = 400;
  double scan_matching_vgicp_resolution_ = 0.50;
  float scan_matching_max_correspondence_distance_ = 1.00f;
  int scan_matching_max_iterations_ = 20;
  int scan_matching_num_threads_ = 2;
  float scan_matching_coarse_max_fitness_score_ = 2.00f;
  ScanMatchRefinePolicy scan_matching_refine_policy_;
  double scan_preprocess_min_range_m_ = 0.50;
  double scan_preprocess_max_range_m_ = 10.0;
  double scan_preprocess_fov_degree_ = 240.0;
  bool scan_ground_filter_enable_ = true;
  double scan_ground_distance_threshold_m_ = 0.08;
  double scan_ground_max_tilt_deg_ = 20.0;
  int scan_ground_min_inliers_ = 80;
  double scan_ground_min_sensor_height_m_ = 0.20;
  double scan_ground_max_sensor_height_m_ = 1.20;
  double scan_ground_clearance_m_ = 0.15;
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

  double point_cloud_max_age_s_ = 0.20;
  bool point_cloud_latest_only_ = true;
  double point_cloud_perf_log_interval_s_ = 10.0;
  double point_cloud_slow_callback_ms_ = 80.0;
  std::array<PointCloudPerfWindow, 4> point_cloud_perf_windows_;
  SteadyClock::time_point last_point_cloud_perf_log_{};

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
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
