#include <localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <localization/pose_system.hpp>
#include <localization/odom_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace localization {
/**
 * @brief constructor
 * @param registration        registration method
 * @param stamp               timestamp
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 * @param bias_acc            initial acceleration bias
 * @param bias_gyro           initial gyro bias
 * @param gyro_bias_process_noise  process noise of the gyro bias states
 * @param gyro_bias_initial_cov    initial covariance of the gyro bias states
 *
 * WARNING - do not feed StaticIMUInit::GetInitBa() into bias_acc.
 * static_imu_init.cpp assigns init_bias_acce_ = mean_acce_ with the gravity
 * estimation commented out, so GetInitBa() is the raw accelerometer mean and its
 * norm is ~9.81 m/s^2, not an accelerometer bias. PoseSystem::f() already removes
 * gravity in the world frame, so passing it here subtracts gravity twice and the
 * filter diverges within seconds. Fix static_imu_init.cpp:66-68 first if the
 * accelerometer bias is ever needed.
 */
PoseEstimator::PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const rclcpp::Time& stamp,
    const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration, Eigen::Vector3d bias_acc, Eigen::Vector3d bias_gyro,
    double gyro_bias_process_noise, double gyro_bias_initial_cov)
    : init_stamp(stamp), registration(registration), cool_time_duration(cool_time_duration) {

  prev_stamp = rclcpp::Time((int64_t)0, init_stamp.get_clock_type());
  last_observation = Eigen::Matrix4f::Identity();
  last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = pos;

  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3) *= 0.5;     // 1.0
  process_noise.middleRows(3, 3) *= 1.0;
  process_noise.middleRows(6, 4) *= 0.5;
  // Accelerometer bias stays frozen on purpose: it is barely observable here.
  // The dead-reckoning bridge zeroes the acceleration input, imu_data_filter_num_
  // keeps only 1/5 of the samples, and PoseSystem already notes that acceleration
  // contributes little because of its noise. Releasing it just lets the filter
  // trade accelerometer bias against gravity and pitch.
  process_noise.middleRows(10, 3) *= 1e-6;
  process_noise.middleRows(13, 3) *= gyro_bias_process_noise;

  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.01;
  measurement_noise.middleRows(3, 4) *= 0.001;

  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
  // bias_acc is deliberately ignored - see the WARNING above the constructor.
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3) = bias_gyro.cast<float>();
  // mean.middleRows(13, 3).setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;
  // The gyro bias now starts from a measured value instead of zero, so its prior
  // must be tightened accordingly; 0.01 means sigma = 0.1 rad/s = 5.7 deg/s, which
  // is an order of magnitude looser than the bias it is describing.
  cov.block<3, 3>(13, 13) = Eigen::Matrix3f::Identity() * static_cast<float>(gyro_bias_initial_cov);

  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator() {}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const rclcpp::Time& stamp) {
  if ((stamp - init_stamp).seconds() < cool_time_duration || prev_stamp == rclcpp::Time((int64_t)0, prev_stamp.get_clock_type()) || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).seconds();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  ukf->predict();
  normalizeAndGuardUkf();
}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const rclcpp::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  if (/*(stamp - init_stamp).seconds() < cool_time_duration || */prev_stamp == rclcpp::Time((int64_t)0, prev_stamp.get_clock_type()) || prev_stamp == stamp) {
    prev_stamp = stamp;
    RCLCPP_INFO(rclcpp::get_logger("PoseEstimator"), "Some ploblems with prev_stamp, not predict!");
    return;
  }

  double dt = (stamp - prev_stamp).seconds();
  if (dt > 0.1){
    dt = 0.1;
  } else if (dt < 0.0) {
    RCLCPP_INFO(rclcpp::get_logger("PoseEstimator"), "dt < 0.0, not predict!");
    return;
  }
  if (!acc.allFinite() || !gyro.allFinite()) {
    prev_stamp = stamp;
    return;
  }
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;
  
  ukf->predict(control);
  normalizeAndGuardUkf();
}

void PoseEstimator::set_initial_biases(const Eigen::Vector3f& acc_bias, const Eigen::Vector3f& gyro_bias){
  if(!ukf){
    return;
  }
  // state layout: [px,py,pz, vx,vy,vz, qw,qx,qy,qz, bax,bay,baz, bgx,bgy,bgz]
  ukf->mean.middleRows(10, 3) = acc_bias;
  ukf->mean.middleRows(13, 3) = gyro_bias;
  normalizeAndGuardUkf();
}

void PoseEstimator::normalizeAndGuardUkf() {
  if (!ukf || ukf->mean.size() < 10) {
    return;
  }
  Eigen::Quaternionf q(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6f) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }
  ukf->mean[6] = q.w();
  ukf->mean[7] = q.x();
  ukf->mean[8] = q.y();
  ukf->mean[9] = q.z();
  if (!ukf->mean.allFinite() || !ukf->cov.allFinite()) {
    RCLCPP_ERROR(logger_, "UKF state became non-finite; resetting covariance");
    for (int i = 0; i < ukf->mean.size(); ++i) {
      if (!std::isfinite(ukf->mean(i))) {
        ukf->mean(i) = 0.0f;
      }
    }
    ukf->cov = Eigen::MatrixXf::Identity(ukf->cov.rows(), ukf->cov.cols()) * 0.01f;
  }
}

/**
 * @brief update the state of the odomety-based pose estimation
 */
void PoseEstimator::predict_odom(const Eigen::Matrix4f& odom_delta) {
  Eigen::Quaternionf delta_orientation(odom_delta.block<3, 3>(0, 0));
  delta_orientation.normalize();
  if (!odom_orientation_initialized_) {
    odom_orientation_prediction_ = quat();
    odom_orientation_initialized_ = true;
  } else {
    odom_orientation_prediction_ =
      (odom_orientation_prediction_ * delta_orientation).normalized();
  }

  if(!odom_ukf) {
    Eigen::MatrixXf odom_process_noise = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf odom_measurement_noise = Eigen::MatrixXf::Identity(7, 7) * 1e-3;

    Eigen::VectorXf odom_mean(7);
    odom_mean.block<3, 1>(0, 0) = Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
    odom_mean.block<4, 1>(3, 0) = Eigen::Vector4f(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
    Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity(7, 7) * 1e-2;

    OdomSystem odom_system;
    odom_ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>(odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
  }

  // invert quaternion if the rotation axis is flipped
  Eigen::Quaternionf quat(odom_delta.block<3, 3>(0, 0));
  if(odom_quat().coeffs().dot(quat.coeffs()) < 0.0) {
    quat.coeffs() *= -1.0f;
  }

  Eigen::VectorXf control(7);
  control.middleRows(0, 3) = odom_delta.block<3, 1>(0, 3);
  control.middleRows(3, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(7, 7);
  process_noise.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity() * odom_delta.block<3, 1>(0, 3).norm() + Eigen::Matrix3f::Identity() * 1e-3;
  process_noise.bottomRightCorner(4, 4) = Eigen::Matrix4f::Identity() * (1 - std::abs(quat.w())) + Eigen::Matrix4f::Identity() * 1e-3;

  odom_ukf->setProcessNoiseCov(process_noise);
  odom_ukf->predict(control);
}

void PoseEstimator::enable_lidar_odometry_prediction() {
  lidar_odometry_prediction_enabled_ = true;
}

void PoseEstimator::predict_lidar_odometry(const Eigen::Matrix4f& lidar_delta) {
  if (!lidar_odometry_prediction_enabled_ || !lidar_odometry_prediction_initialized_ ||
      !lidar_delta.allFinite()) {
    return;
  }

  lidar_odometry_prediction_ = lidar_odometry_prediction_ * lidar_delta;
  if (!lidar_odometry_prediction_.allFinite()) {
    invalidate_lidar_odometry_prediction();
  }
}

void PoseEstimator::invalidate_lidar_odometry_prediction() {
  lidar_odometry_prediction_initialized_ = false;
  lidar_odom_pred_error = boost::none;
}

void PoseEstimator::configure_scan_matching(
    pcl::Registration<PointT, PointT>::Ptr refine,
    pcl::PointCloud<PointT>::ConstPtr global_map,
    float local_map_xy_radius,
    float local_map_z_radius,
    int min_local_map_points,
    float max_fitness_score,
    float coarse_max_fitness_score) {
  refine_registration_ = refine;
  global_map_ = global_map;
  local_map_xy_radius_ = std::max(3.0f, local_map_xy_radius);
  local_map_z_radius_ = std::max(1.0f, local_map_z_radius);
  min_local_map_points_ = std::max(50, min_local_map_points);
  max_fitness_score_ = std::max(0.001f, max_fitness_score);
  coarse_max_fitness_score_ = std::max(max_fitness_score_, coarse_max_fitness_score);
}

void PoseEstimator::configure_refine_policy(const ScanMatchRefinePolicy& policy) {
  refine_policy_.skip_ndt_score = std::clamp(
    policy.skip_ndt_score, 0.001f, max_fitness_score_);
  refine_policy_.min_improvement_ratio = std::clamp(
    policy.min_improvement_ratio, 0.0f, 0.90f);
  refine_policy_.max_translation_disagreement_m = std::max(
    0.05f, policy.max_translation_disagreement_m);
  refine_policy_.max_rotation_disagreement_rad = std::max(
    0.01f, policy.max_rotation_disagreement_rad);
}

pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::cropLocalMap(
    const Eigen::Vector3f& center) const {
  pcl::PointCloud<PointT>::Ptr local(new pcl::PointCloud<PointT>());
  if (!global_map_ || global_map_->empty()) {
    return local;
  }
  const float xy_r2 = local_map_xy_radius_ * local_map_xy_radius_;
  local->points.reserve(global_map_->points.size() / 4);
  for (const auto& point : global_map_->points) {
    const float dx = point.x - center.x();
    const float dy = point.y - center.y();
    if (dx * dx + dy * dy > xy_r2) {
      continue;
    }
    if (std::abs(point.z - center.z()) > local_map_z_radius_) {
      continue;
    }
    local->points.push_back(point);
  }
  local->width = static_cast<uint32_t>(local->points.size());
  local->height = 1;
  local->is_dense = false;
  return local;
}

namespace {
struct AlignAttempt {
  bool usable_as_seed = false;
  bool acceptable = false;
  bool converged = false;
  double fitness = std::numeric_limits<double>::infinity();
  float correction_m = std::numeric_limits<float>::infinity();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned;
};

AlignAttempt runAlign(
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr& reg,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
    const Eigen::Matrix4f& guess,
    double seed_max_fitness,
    double accept_max_fitness,
    float max_correction_m) {
  AlignAttempt attempt;
  attempt.aligned.reset(new pcl::PointCloud<pcl::PointXYZI>());
  if (!reg || !cloud || cloud->empty() || !guess.allFinite()) {
    return attempt;
  }
  try {
    reg->setInputSource(cloud);
    reg->align(*attempt.aligned, guess);
    attempt.fitness = reg->getFitnessScore();
    attempt.transform = reg->getFinalTransformation();
    attempt.converged = reg->hasConverged();
  } catch (const std::exception&) {
    return attempt;
  }
  const Eigen::Matrix4f correction = guess.inverse() * attempt.transform;
  attempt.correction_m = correction.block<3, 1>(0, 3).norm();
  const bool finite = attempt.transform.allFinite() &&
    std::isfinite(attempt.fitness) && std::isfinite(attempt.correction_m);
  attempt.usable_as_seed = attempt.converged && finite &&
    attempt.fitness < seed_max_fitness && attempt.correction_m < max_correction_m;
  attempt.acceptable = attempt.converged && finite &&
    attempt.fitness < accept_max_fitness && attempt.correction_m < max_correction_m;
  if (!finite && attempt.aligned) {
    attempt.aligned->clear();
  }
  return attempt;
}
}  // namespace

/**
 * @brief correct
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(
    const rclcpp::Time& stamp,
    const pcl::PointCloud<PointT>::ConstPtr& cloud,
    bool apply_observation,
    bool allow_high_quality_refine_skip) {
  const auto match_start = std::chrono::steady_clock::now();
  match_timing_ = MatchTiming{};
  Eigen::Matrix4f imu_guess = matrix();
  Eigen::Matrix4f init_guess = imu_guess;
  Eigen::Matrix4f odom_guess = imu_guess;

  // The chassis orientation is reliable during in-place turns, while its
  // translation scale does not agree closely enough with lidar localization.
  // Use only the odometry rotation as the NDT seed and retain the IMU/NDT
  // position estimate.
  const bool lidar_odometry_available =
    lidar_odometry_prediction_enabled_ && lidar_odometry_prediction_initialized_;
  if (lidar_odometry_available) {
    init_guess = lidar_odometry_prediction_;
  } else if (odom_orientation_initialized_) {
    odom_guess = odom_matrix();
    if (odom_orientation_prediction_.coeffs().allFinite()) {
      init_guess.block<3, 3>(0, 0) = odom_orientation_prediction_.toRotationMatrix();
    }
  }

  constexpr float kMaxInitCorrectionM = 5.0f;
  constexpr float kMaxRefineCorrectionM = 1.5f;

  const auto ndt_start = std::chrono::steady_clock::now();
  AlignAttempt ndt = runAlign(
    registration, cloud, init_guess,
    coarse_max_fitness_score_, max_fitness_score_, kMaxInitCorrectionM);
  match_timing_.ndt_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - ndt_start).count();

  match_result_.ndt_score_ = static_cast<float>(ndt.fitness);
  match_result_.refine_score_ = std::numeric_limits<float>::infinity();
  match_result_.transform_ = ndt.transform;
  match_result_.fitness_score_ = static_cast<float>(ndt.fitness);
  match_result_.method_ = "ndt";

  AlignAttempt refine;
  const bool run_refine = refine_registration_ && shouldRunRefinement(
    allow_high_quality_refine_skip, ndt.acceptable,
    static_cast<float>(ndt.fitness), refine_policy_);
  if (run_refine) {
    const Eigen::Matrix4f refine_guess = ndt.usable_as_seed ? ndt.transform : init_guess;
    const auto local_map_start = std::chrono::steady_clock::now();
    auto local_map = cropLocalMap(refine_guess.block<3, 1>(0, 3));
    match_timing_.local_map_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - local_map_start).count();
    if (static_cast<int>(local_map->size()) >= min_local_map_points_) {
      refine_registration_->setInputTarget(local_map);
      const auto refine_start = std::chrono::steady_clock::now();
      refine = runAlign(
        refine_registration_, cloud, refine_guess,
        coarse_max_fitness_score_, max_fitness_score_, kMaxRefineCorrectionM);
      match_timing_.refine_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - refine_start).count();
      const Eigen::Matrix4f from_init = init_guess.inverse() * refine.transform;
      const float from_init_m = from_init.block<3, 1>(0, 3).norm();
      const bool bounded_from_init = refine.transform.allFinite() &&
        std::isfinite(from_init_m) && from_init_m < kMaxInitCorrectionM;
      refine.usable_as_seed = refine.usable_as_seed && bounded_from_init;
      refine.acceptable = refine.acceptable && bounded_from_init;
      match_result_.refine_score_ = static_cast<float>(refine.fitness);
    } else {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock, 2000,
        "Skipping VGICP refine: local map has %zu points (need %d) around [%.2f, %.2f, %.2f]",
        local_map ? local_map->size() : 0, min_local_map_points_,
        refine_guess(0, 3), refine_guess(1, 3), refine_guess(2, 3));
    }
  }

  AlignAttempt chosen;
  float refine_disagreement_m = std::numeric_limits<float>::infinity();
  float refine_disagreement_rad = std::numeric_limits<float>::infinity();
  if (ndt.transform.allFinite() && refine.transform.allFinite()) {
    const Eigen::Matrix4f disagreement = ndt.transform.inverse() * refine.transform;
    refine_disagreement_m = disagreement.block<3, 1>(0, 3).norm();
    Eigen::Quaternionf disagreement_rotation(disagreement.block<3, 3>(0, 0));
    if (disagreement_rotation.coeffs().allFinite() && disagreement_rotation.norm() > 1e-6f) {
      disagreement_rotation.normalize();
      refine_disagreement_rad = Eigen::Quaternionf::Identity().angularDistance(
        disagreement_rotation);
    }
  }
  if (shouldChooseRefinement(
        ndt.acceptable, static_cast<float>(ndt.fitness),
        refine.acceptable, static_cast<float>(refine.fitness),
        refine_disagreement_m, refine_disagreement_rad, refine_policy_)) {
    chosen = refine;
    match_result_.method_ = ndt.usable_as_seed ? "ndt_vgicp" : "vgicp";
  } else if (ndt.acceptable) {
    chosen = ndt;
    match_result_.method_ = run_refine ? "ndt" : "ndt_high_quality";
  }

  match_result_.is_converged_ = chosen.acceptable;
  if (chosen.acceptable) {
    match_result_.fitness_score_ = static_cast<float>(chosen.fitness);
    match_result_.transform_ = chosen.transform;
  } else if (refine.converged && std::isfinite(refine.fitness) &&
             (!ndt.converged || refine.fitness < ndt.fitness)) {
    match_result_.fitness_score_ = static_cast<float>(refine.fitness);
    match_result_.transform_ = refine.transform;
    match_result_.method_ = "vgicp_rejected";
  } else {
    match_result_.method_ = "ndt_rejected";
  }

  if (!chosen.acceptable) {
    match_timing_.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - match_start).count();
    RCLCPP_WARN(logger_,
                "Rejecting scan match: ndt(conv=%d score=%.3f corr=%.2fm) "
                "vgicp(conv=%d score=%.3f corr=%.2fm)",
                ndt.converged, ndt.fitness, ndt.correction_m,
                refine.converged, refine.fitness, refine.correction_m);
    pcl::PointCloud<PointT>::Ptr empty(new pcl::PointCloud<PointT>());
    return empty;
  }

  pcl::PointCloud<PointT>::Ptr aligned = chosen.aligned;
  if (!aligned) {
    aligned.reset(new pcl::PointCloud<PointT>());
  }
  const Eigen::Matrix4f trans = chosen.transform;

  last_correction_stamp = stamp;

  Eigen::Vector3f p = trans.block<3, 1>(0, 3);
  Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

  if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }

  Eigen::VectorXf observation(7);
  observation.middleRows(0, 3) = p;
  observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
  last_observation = trans;

  if (!apply_observation) {
    match_timing_.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - match_start).count();
    return aligned;
  }

  ukf->correct(observation);
  normalizeAndGuardUkf();
  imu_pred_error = imu_guess.inverse() * trans;

  if (lidar_odometry_prediction_enabled_) {
    if (lidar_odometry_available) {
      lidar_odom_pred_error = lidar_odometry_prediction_.inverse() * trans;
    } else {
      lidar_odom_pred_error = boost::none;
    }
    // An accepted map match is the new absolute anchor. Subsequent accepted
    // scan-to-scan deltas advance this anchor until the next NDT correction.
    lidar_odometry_prediction_ = trans;
    lidar_odometry_prediction_initialized_ = true;
  }

  if (odom_orientation_initialized_) {
    if (odom_orientation_prediction_.coeffs().dot(q.coeffs()) < 0.0f) {
      q.coeffs() *= -1.0f;
    }
    const float disagreement = odom_orientation_prediction_.angularDistance(q);
    const float correction_gain = disagreement < 0.15f ? 0.20f : 0.02f;
    odom_orientation_prediction_ =
      odom_orientation_prediction_.slerp(correction_gain, q).normalized();
  }

  if(odom_ukf) {
    if (observation.tail<4>().dot(odom_ukf->mean.tail<4>()) < 0.0) {
      odom_ukf->mean.tail<4>() *= -1.0;
    }

    odom_ukf->correct(observation);
    odom_pred_error = odom_guess.inverse() * trans;
  }

  match_timing_.total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - match_start).count();

  return aligned;
}

/* getters */
rclcpp::Time PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  return m;
}

Eigen::Vector3f PoseEstimator::odom_pos() const {
  return Eigen::Vector3f(odom_ukf->mean[0], odom_ukf->mean[1], odom_ukf->mean[2]);
}

Eigen::Quaternionf PoseEstimator::odom_quat() const {
  return Eigen::Quaternionf(odom_ukf->mean[3], odom_ukf->mean[4], odom_ukf->mean[5], odom_ukf->mean[6]).normalized();
}

Eigen::Matrix4f PoseEstimator::odom_matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = odom_quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = odom_pos();
  return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error() const {
  return odom_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::lidar_odometry_prediction_error() const {
  return lidar_odom_pred_error;
}

PoseEstimator::MatchResult PoseEstimator::GetMatchState() const {
    return match_result_;
}

PoseEstimator::MatchTiming PoseEstimator::GetMatchTiming() const {
    return match_timing_;
}

Eigen::VectorXf PoseEstimator::GetCurrentUkfState() {
    Eigen::Matrix4f state_matrix = matrix();
    Eigen::Vector3f t_state = state_matrix.block<3, 1>(0, 3);
    Eigen::Matrix3f rot = state_matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf q_state(rot);
    q_state.normalize();
    Eigen::VectorXf state(7);
    state.head<3>() = t_state;
    state.tail<4>() << q_state.w(), q_state.x(), q_state.y(), q_state.z();
    return state;
}

void PoseEstimator::apply_position_correction(const Eigen::Vector3f& correction) {
  if(!ukf) {
    return;
  }
  ukf->mean.middleRows(0, 3) += correction;
  if(odom_ukf) {
    odom_ukf->mean.middleRows(0, 3) += correction;
  }
}

void PoseEstimator::inject_rtk_xy_yaw(
    const Eigen::Vector3f& position,
    bool set_yaw,
    float yaw,
    bool freeze_z,
    float horizontal_variance,
    float vertical_variance,
    float yaw_variance) {
  if (!ukf) {
    return;
  }
  ukf->mean[0] = position.x();
  ukf->mean[1] = position.y();
  ukf->mean[3] = 0.0f;
  ukf->mean[4] = 0.0f;
  if (freeze_z) {
    ukf->mean[2] = position.z();
    ukf->mean[5] = 0.0f;
  }
  Eigen::MatrixXf covariance = ukf->getCov();
  covariance(0, 0) = std::max(horizontal_variance, 1e-4f);
  covariance(1, 1) = std::max(horizontal_variance, 1e-4f);
  covariance(3, 3) = std::max(horizontal_variance, 1e-4f);
  covariance(4, 4) = std::max(horizontal_variance, 1e-4f);
  if (freeze_z) {
    covariance(2, 2) = std::max(vertical_variance, 1e-4f);
    covariance(5, 5) = std::max(vertical_variance, 1e-4f);
  }
  ukf->setCov(covariance);
  if (set_yaw) {
    const Eigen::Quaternionf current = quat();
    const Eigen::Matrix3f rotation = current.toRotationMatrix();
    const float current_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const float dyaw = std::atan2(std::sin(yaw - current_yaw), std::cos(yaw - current_yaw));
    Eigen::Quaternionf q = Eigen::AngleAxisf(dyaw, Eigen::Vector3f::UnitZ()) * current;
    q.normalize();
    if (q.coeffs().dot(current.coeffs()) < 0.0f) {
      q.coeffs() *= -1.0f;
    }
    ukf->mean[6] = q.w();
    ukf->mean[7] = q.x();
    ukf->mean[8] = q.y();
    ukf->mean[9] = q.z();
    // For small yaw errors qz is approximately yaw/2.
    covariance(9, 9) = std::max(yaw_variance * 0.25f, 1e-5f);
  }
  ukf->setCov(covariance);
  if (odom_ukf) {
    odom_ukf->mean[0] = ukf->mean[0];
    odom_ukf->mean[1] = ukf->mean[1];
    if (freeze_z) {
      odom_ukf->mean[2] = ukf->mean[2];
    }
  }
  normalizeAndGuardUkf();
  last_observation = matrix();
}

void PoseEstimator::correct_absolute_pose(
    const Eigen::Vector3f& position,
    const Eigen::Quaternionf& orientation,
    float horizontal_variance,
    float vertical_variance,
    float orientation_variance) {
  if (!ukf) {
    return;
  }
  Eigen::Quaternionf q = orientation.normalized();
  if (quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }
  Eigen::VectorXf observation(7);
  observation.head<3>() = position;
  observation.tail<4>() << q.w(), q.x(), q.y(), q.z();

  const Eigen::MatrixXf previous_noise = ukf->getMeasurementNoiseCov();
  Eigen::MatrixXf noise = Eigen::MatrixXf::Identity(7, 7);
  noise(0, 0) = std::max(horizontal_variance, 1e-4f);
  noise(1, 1) = std::max(horizontal_variance, 1e-4f);
  noise(2, 2) = std::max(vertical_variance, 1e-4f);
  noise.bottomRightCorner(4, 4) *= std::max(orientation_variance, 1e-5f);
  ukf->setMeasurementNoiseCov(noise);
  ukf->correct(observation);
  ukf->setMeasurementNoiseCov(previous_noise);
  normalizeAndGuardUkf();
  last_observation = matrix();
}

void PoseEstimator::begin_dead_reckoning_bridge() {
  if (ukf) {
    ukf->mean.segment<3>(3).setZero();
  }
}

void PoseEstimator::apply_body_odom_translation(
    const Eigen::Matrix4f& odom_delta,
    float translation_variance) {
  if (!ukf || !odom_delta.allFinite()) {
    return;
  }
  Eigen::Vector3f body_translation = odom_delta.block<3, 1>(0, 3);
  body_translation.z() = 0.0f;
  ukf->mean.head<3>() += quat().toRotationMatrix() * body_translation;
  // Position is propagated exclusively by body odometry during the bridge.
  // Keeping velocity at zero prevents IMU acceleration integration from
  // adding a second translation estimate on the next prediction cycle.
  ukf->mean.segment<3>(3).setZero();
  Eigen::MatrixXf covariance = ukf->getCov();
  const float translation_noise = std::max(translation_variance, 1e-6f);
  covariance.block<3, 3>(0, 0).diagonal().array() += translation_noise;
  ukf->setCov(covariance);
}

float PoseEstimator::horizontal_position_sigma() const {
  if (!ukf) {
    return std::numeric_limits<float>::infinity();
  }
  const auto& covariance = ukf->getCov();
  return std::sqrt(std::max(0.0f, std::max(covariance(0, 0), covariance(1, 1))));
}

float PoseEstimator::yaw_sigma() const {
  if (!ukf) {
    return std::numeric_limits<float>::infinity();
  }
  const auto& covariance = ukf->getCov();
  return 2.0f * std::sqrt(std::max(0.0f, covariance(9, 9)));
}

std::array<double, 36> PoseEstimator::pose_covariance() const {
  std::array<double, 36> output{};
  if (!ukf) {
    output[0] = output[7] = output[14] = 1.0e6;
    output[21] = output[28] = output[35] = 1.0e6;
    return output;
  }
  const auto& covariance = ukf->getCov();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      output[row * 6 + col] = std::isfinite(covariance(row, col))
        ? static_cast<double>(covariance(row, col)) : 0.0;
    }
  }
  const double roll_variance = std::max(1e-5, 4.0 * static_cast<double>(covariance(7, 7)));
  const double pitch_variance = std::max(1e-5, 4.0 * static_cast<double>(covariance(8, 8)));
  const double yaw_variance = std::max(1e-5, 4.0 * static_cast<double>(covariance(9, 9)));
  output[21] = std::isfinite(roll_variance) ? roll_variance : 1.0e6;
  output[28] = std::isfinite(pitch_variance) ? pitch_variance : 1.0e6;
  output[35] = std::isfinite(yaw_variance) ? yaw_variance : 1.0e6;
  return output;
}

} 
