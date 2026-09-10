#ifndef LOCALIZATION_ANCHOR_UKF_HPP
#define LOCALIZATION_ANCHOR_UKF_HPP

#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace localization {

inline double wrapAnchorYaw(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

struct AnchorObservation {
  Eigen::Vector3d value = Eigen::Vector3d::Zero();  // map->lio x, y, yaw
  Eigen::Vector3d variance = Eigen::Vector3d::Ones();
  std::string source = "none";
};

struct AnchorUpdateResult {
  bool accepted = false;
  Eigen::Vector3d innovation = Eigen::Vector3d::Zero();
  Eigen::Vector3d gain = Eigen::Vector3d::Zero();
  std::string reason = "uninitialized";
};

// The motion source is FAST-LIO. This filter deliberately estimates only the
// slowly-changing planar map->lio anchor. With an identity observation model,
// the UKF correction is the ordinary Kalman update; yaw wrapping is the only
// non-linear part and is handled explicitly.
class AnchorUkf {
public:
  void reset(
      const Eigen::Vector3d& value,
      const Eigen::Vector3d& variance = Eigen::Vector3d(0.01, 0.01, 0.001)) {
    value_ = value;
    value_.z() = wrapAnchorYaw(value_.z());
    covariance_ = variance.cwiseMax(Eigen::Vector3d::Constant(1.0e-9)).asDiagonal();
    initialized_ = value_.allFinite() && covariance_.allFinite();
    last_source_ = "seed";
  }

  void clear() {
    initialized_ = false;
    value_.setZero();
    covariance_.setIdentity();
    last_source_ = "none";
  }

  bool initialized() const { return initialized_; }
  const Eigen::Vector3d& value() const { return value_; }
  const Eigen::Matrix3d& covariance() const { return covariance_; }
  const std::string& lastSource() const { return last_source_; }

  void predict(double translation_m, double rotation_rad,
      double translation_variance_per_m, double yaw_variance_per_rad) {
    if (!initialized_) {
      return;
    }
    const double xy_noise = std::max(0.0, std::fabs(translation_m)) *
      std::max(0.0, translation_variance_per_m);
    const double yaw_noise = std::max(0.0, std::fabs(rotation_rad)) *
      std::max(0.0, yaw_variance_per_rad);
    covariance_(0, 0) += xy_noise;
    covariance_(1, 1) += xy_noise;
    covariance_(2, 2) += yaw_noise;
  }

  AnchorUpdateResult update(const AnchorObservation& observation) {
    AnchorUpdateResult result;
    if (!observation.value.allFinite() || !observation.variance.allFinite() ||
        (observation.variance.array() <= 0.0).any()) {
      result.reason = "invalid_observation";
      return result;
    }
    if (!initialized_) {
      reset(observation.value, observation.variance);
      last_source_ = observation.source;
      result.accepted = initialized_;
      result.gain = Eigen::Vector3d::Ones();
      result.reason = initialized_ ? "initialized" : "invalid_observation";
      return result;
    }

    Eigen::Vector3d innovation = observation.value - value_;
    innovation.z() = wrapAnchorYaw(innovation.z());
    const Eigen::Matrix3d noise = observation.variance.asDiagonal();
    const Eigen::Matrix3d innovation_covariance = covariance_ + noise;
    if (!innovation_covariance.allFinite() ||
        std::fabs(innovation_covariance.determinant()) < 1.0e-18) {
      result.reason = "singular_covariance";
      return result;
    }
    const Eigen::Matrix3d gain = covariance_ * innovation_covariance.inverse();
    value_ += gain * innovation;
    value_.z() = wrapAnchorYaw(value_.z());
    covariance_ = (Eigen::Matrix3d::Identity() - gain) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    last_source_ = observation.source;
    result.accepted = value_.allFinite() && covariance_.allFinite();
    result.innovation = innovation;
    result.gain = gain.diagonal();
    result.reason = result.accepted ? "accepted" : "non_finite_result";
    return result;
  }

private:
  bool initialized_ = false;
  Eigen::Vector3d value_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Identity();
  std::string last_source_ = "none";
};

}  // namespace localization

#endif  // LOCALIZATION_ANCHOR_UKF_HPP
