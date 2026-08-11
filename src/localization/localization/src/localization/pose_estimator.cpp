#include <localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <cmath>
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
 */
PoseEstimator::PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const rclcpp::Time& stamp, 
    const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration, Eigen::Vector3d bias_acc, Eigen::Vector3d bias_gyro)
    : init_stamp(stamp), registration(registration), cool_time_duration(cool_time_duration) {

  prev_stamp = rclcpp::Time((int64_t)0, init_stamp.get_clock_type());
  last_observation = Eigen::Matrix4f::Identity();
  last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = pos;

  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3) *= 0.5;     // 1.0
  process_noise.middleRows(3, 3) *= 1.0;
  process_noise.middleRows(6, 4) *= 0.5;
  process_noise.middleRows(10, 3) *= 1e-6;
  process_noise.middleRows(13, 3) *= 1e-6;

  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.01;
  measurement_noise.middleRows(3, 4) *= 0.001;

  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3) = bias_gyro.cast<float>();
  // mean.middleRows(13, 3).setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

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
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;
  
  ukf->predict(control);
}

void PoseEstimator::set_initial_biases(const Eigen::Vector3f& acc_bias, const Eigen::Vector3f& gyro_bias){
  if(!ukf){
    return;
  }
  // state layout: [px,py,pz, vx,vy,vz, qw,qx,qy,qz, bax,bay,baz, bgx,bgy,bgz]
  ukf->mean.middleRows(10, 3) = acc_bias;
  ukf->mean.middleRows(13, 3) = gyro_bias;
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

/**
 * @brief correct
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(const rclcpp::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  Eigen::Matrix4f imu_guess = matrix();
  Eigen::Matrix4f init_guess = imu_guess;
  // Eigen::Matrix4f no_guess = last_observation;
  Eigen::Matrix4f odom_guess = imu_guess;
  // Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  // The chassis orientation is reliable during in-place turns, while its
  // translation scale does not agree closely enough with lidar localization.
  // Use only the odometry rotation as the NDT seed and retain the IMU/NDT
  // position estimate.
  if (odom_orientation_initialized_) {
    odom_guess = odom_matrix();
    if (odom_orientation_prediction_.coeffs().allFinite()) {
      init_guess.block<3, 3>(0, 0) = odom_orientation_prediction_.toRotationMatrix();
    }
  }

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration->setInputSource(cloud);
  registration->align(*aligned, init_guess);
  double ndt_score = registration->getFitnessScore();

  if (ndt_score > 0.5) {
    RCLCPP_INFO(rclcpp::get_logger("PoseEstimator"), "ndt_score > 0.5, ndt_score: %f", ndt_score);
  }

  Eigen::Matrix4f trans = registration->getFinalTransformation();
  match_result_.fitness_score_ = ndt_score;

  const Eigen::Matrix4f correction = init_guess.inverse() * trans;
  const float correction_distance = correction.block<3, 1>(0, 3).norm();
  const bool transform_valid = trans.allFinite() && std::isfinite(correction_distance);
  const bool match_valid = registration->hasConverged() && std::isfinite(ndt_score) &&
                           ndt_score < 0.5 && transform_valid && correction_distance < 5.0f;
  match_result_.is_converged_ = match_valid;

  if (!match_valid) {
    RCLCPP_WARN(rclcpp::get_logger("PoseEstimator"),
                "Rejecting NDT correction: converged=%d score=%.6f correction_distance=%.3f finite=%d",
                registration->hasConverged(), ndt_score, correction_distance, transform_valid);
    // The registration output may contain NaN points. Returning an empty cloud
    // prevents downstream nearest-neighbor checks and TF publication from
    // consuming an invalid transform.
    aligned->clear();
    return aligned;
  }

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

  // wo_pred_error = no_guess.inverse() * registration->getFinalTransformation();

  ukf->correct(observation);
  imu_pred_error = imu_guess.inverse() * registration->getFinalTransformation();

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
    odom_pred_error = odom_guess.inverse() * registration->getFinalTransformation();
  }

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

PoseEstimator::MatchResult PoseEstimator::GetMatchState() const {
    return match_result_;
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

} 
