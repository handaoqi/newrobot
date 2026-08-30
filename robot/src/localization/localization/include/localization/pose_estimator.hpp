#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <array>
#include <memory>
#include <limits>
#include <string>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <Eigen/Geometry>
#include <localization/scan_match_policy.hpp>

#include <rclcpp/rclcpp.hpp>

// struct MatchResult{
//     bool  is_converged_;   ///< Indicates whether the matching operation converged.
//     float fitness_score_;  ///< The fitness score of the matching operation.
//   };

namespace kkl {
  namespace alg {
template<typename T, class System> class UnscentedKalmanFilterX;
  }
}

namespace localization {

class PoseSystem;
class OdomSystem;

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  struct MatchResult{
    bool  is_converged_ = false;   ///< Indicates whether the matching operation converged.
    float fitness_score_ = std::numeric_limits<float>::infinity();  ///< Accepted (or best rejected) fitness.
    float ndt_score_ = std::numeric_limits<float>::infinity();
    float refine_score_ = std::numeric_limits<float>::infinity();
    Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
    std::string method_ = "none";
  };

  struct MatchTiming {
    double ndt_ms = 0.0;
    double local_map_ms = 0.0;
    double refine_ms = 0.0;
    double total_ms = 0.0;
  };

  /// Default process noise for the gyro bias states (rad/s)^2 per second.
  /// 1e-6 effectively freezes the bias; the mapping side uses b_gyr_cov = 1e-4
  /// (see robot/src/slam/src/config/config.yaml), which is the reference point here.
  static constexpr double kDefaultGyroBiasProcessNoise = 1e-4;
  /// Default initial covariance for the gyro bias states. Sized to the residual
  /// uncertainty that remains after StaticIMUInit has averaged a static window,
  /// not to the raw per-sample gyro noise.
  static constexpr double kDefaultGyroBiasInitialCov = 1e-4;

  /**
   * @brief constructor
   * @param registration        registration method
   * @param stamp               timestamp
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   * @param bias_acc            initial acceleration bias. MUST stay zero unless the
   *                            caller has a gravity-free estimate; see the warning in
   *                            pose_estimator.cpp.
   * @param bias_gyro           initial gyro bias, normally StaticIMUInit::GetInitBg()
   * @param gyro_bias_process_noise  process noise of the gyro bias states
   * @param gyro_bias_initial_cov    initial covariance of the gyro bias states
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const rclcpp::Time& stamp, const Eigen::Vector3f& pos,
    const Eigen::Quaternionf& quat, double cool_time_duration, Eigen::Vector3d bias_acc = Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d bias_gyro = Eigen::Vector3d(0.0, 0.0, 0.0),
    double gyro_bias_process_noise = kDefaultGyroBiasProcessNoise,
    double gyro_bias_initial_cov = kDefaultGyroBiasInitialCov);
  ~PoseEstimator();

  /**
   * @brief predict
   * @param stamp    timestamp
   */
  void predict(const rclcpp::Time& stamp);

  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const rclcpp::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

  /**
   * @brief update the state of the odomety-based pose estimation
   */
  void predict_odom(const Eigen::Matrix4f& odom_delta);

  /**
   * @brief Enable a scan-to-scan LiDAR odometry prior for NDT initialization.
   *
   * The node feeds this method only quality-gated relative LiDAR transforms.
   * It is intentionally separate from controller odometry: controller odometry
   * supplies a yaw-only prior while LiDAR odometry supplies a full SE(3) prior.
   */
  void enable_lidar_odometry_prediction();
  void predict_lidar_odometry(const Eigen::Matrix4f& lidar_delta);
  void invalidate_lidar_odometry_prediction();

  /**
   * @brief Attach the indoor refine matcher and the voxelized global map.
   *
   * NDT remains the coarse full-map lock. FastVGICP then snaps the scan to
   * local planar structure (walls/floor) which NDT voxels tend to smear.
   */
  void configure_scan_matching(
    pcl::Registration<PointT, PointT>::Ptr refine,
    pcl::PointCloud<PointT>::ConstPtr global_map,
    float local_map_xy_radius,
    float local_map_z_radius,
    int min_local_map_points,
    float max_fitness_score,
    float coarse_max_fitness_score);

  void configure_refine_policy(const ScanMatchRefinePolicy& policy);

  /**
   * @brief correct
   * @param cloud   input cloud
   * @return cloud aligned to the globalmap
   */
  pcl::PointCloud<PointT>::Ptr correct(
    const rclcpp::Time& stamp,
    const pcl::PointCloud<PointT>::ConstPtr& cloud,
    bool apply_observation = true,
    bool allow_high_quality_refine_skip = false);

  void correct_absolute_pose(
    const Eigen::Vector3f& position,
    const Eigen::Quaternionf& orientation,
    float horizontal_variance,
    float vertical_variance,
    float orientation_variance);

  void begin_dead_reckoning_bridge();

  void apply_body_odom_translation(
    const Eigen::Matrix4f& odom_delta,
    float translation_variance);

  float horizontal_position_sigma() const;
  float yaw_sigma() const;
  std::array<double, 36> pose_covariance() const;

  /* getters */
  rclcpp::Time last_correction_time() const;

  Eigen::Vector3f pos() const;
  Eigen::Vector3f vel() const;
  Eigen::Quaternionf quat() const;
  Eigen::Matrix4f matrix() const;

  Eigen::Vector3f odom_pos() const;
  Eigen::Quaternionf odom_quat() const;
  Eigen::Matrix4f odom_matrix() const;

  const boost::optional<Eigen::Matrix4f>& wo_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& imu_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& odom_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& lidar_odometry_prediction_error() const;

  MatchResult GetMatchState() const; 
  MatchTiming GetMatchTiming() const;
  Eigen::VectorXf GetCurrentUkfState(); 

  void apply_position_correction(const Eigen::Vector3f& correction);

  // Outdoor RTK primary: write map XY (and optional yaw) directly. The 7D
  // quaternion UKF update does not track dual-antenna heading tightly enough
  // for Nav2, which then drives the body the wrong way on the map.
  void inject_rtk_xy_yaw(
    const Eigen::Vector3f& position,
    bool set_yaw,
    float yaw,
    bool freeze_z,
    float horizontal_variance,
    float vertical_variance,
    float yaw_variance);

  // IMU bias setters (used after static IMU initialization)
  void set_initial_biases(const Eigen::Vector3f& acc_bias, const Eigen::Vector3f& gyro_bias);

private:
  void normalizeAndGuardUkf();

  rclcpp::Time init_stamp;             // when the estimator was initialized
  rclcpp::Time prev_stamp;             // when the estimator was updated last time
  rclcpp::Time last_correction_stamp;  // when the estimator performed the correction step
  double cool_time_duration;        //

  Eigen::MatrixXf process_noise;
  MatchResult     match_result_;
  MatchTiming     match_timing_;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;
  bool odom_orientation_initialized_ = false;
  Eigen::Quaternionf odom_orientation_prediction_ = Eigen::Quaternionf::Identity();
  bool lidar_odometry_prediction_enabled_ = false;
  bool lidar_odometry_prediction_initialized_ = false;
  Eigen::Matrix4f lidar_odometry_prediction_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f last_observation;
  boost::optional<Eigen::Matrix4f> wo_pred_error;
  boost::optional<Eigen::Matrix4f> imu_pred_error;
  boost::optional<Eigen::Matrix4f> odom_pred_error;
  boost::optional<Eigen::Matrix4f> lidar_odom_pred_error;

  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::Registration<PointT, PointT>::Ptr refine_registration_;
  pcl::PointCloud<PointT>::ConstPtr global_map_;
  float local_map_xy_radius_ = 18.0f;
  float local_map_z_radius_ = 4.0f;
  int min_local_map_points_ = 400;
  float max_fitness_score_ = 0.50f;
  float coarse_max_fitness_score_ = 2.00f;
  ScanMatchRefinePolicy refine_policy_;

  pcl::PointCloud<PointT>::Ptr cropLocalMap(const Eigen::Vector3f& center) const;

  rclcpp::Logger logger_ = rclcpp::get_logger("pose_estimator");  ///< The logger instance.
  };

}  // namespace localization

#endif  // POSE_ESTIMATOR_HPP
