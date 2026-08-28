#include "global_factor_graph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/numericalDerivative.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace robot::slam
{
namespace
{
using gtsam::Key;
using gtsam::Pose3;

double wrapAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

gtsam::SharedNoiseModel robustDiagonal(const gtsam::Vector& sigmas, double huber_k)
{
    const auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(std::max(0.1, huber_k)), gaussian);
}

gtsam::SharedNoiseModel robustCovariance(const gtsam::Matrix6& covariance, double huber_k)
{
    gtsam::Matrix6 safe = covariance;
    for (int i = 0; i < 6; ++i)
    {
        if (!std::isfinite(safe(i, i)) || safe(i, i) < 1e-8)
            safe(i, i) = 1e-8;
    }
    safe = 0.5 * (safe + safe.transpose());
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(safe);
    if (solver.info() != Eigen::Success)
        return robustDiagonal((gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2).finished(), huber_k);
    const auto eigenvalues = solver.eigenvalues().cwiseMax(1e-8);
    safe = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(std::max(0.1, huber_k)),
        gtsam::noiseModel::Gaussian::Covariance(safe));
}

class LeverArmPositionFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
    using This = LeverArmPositionFactor;

    LeverArmPositionFactor() = default;
    LeverArmPositionFactor(Key key, const gtsam::Point3& measurement,
        const gtsam::Point3& lever_arm, const gtsam::SharedNoiseModel& model)
        : Base(model, key), measurement_(measurement), lever_arm_(lever_arm) {}

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const auto* other = dynamic_cast<const This*>(&expected);
        return other != nullptr && Base::equals(*other, tol)
            && measurement_.isApprox(other->measurement_, tol)
            && lever_arm_.isApprox(other->lever_arm_, tol);
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        const auto error = pose.translation() + pose.rotation().rotate(lever_arm_) - measurement_;
        if (H)
        {
            *H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
                [this](const gtsam::Pose3& value) {
                    return value.translation() + value.rotation().rotate(lever_arm_) - measurement_;
                }, pose);
        }
        return error;
    }

private:
    gtsam::Point3 measurement_ = gtsam::Point3(0.0, 0.0, 0.0);
    gtsam::Point3 lever_arm_ = gtsam::Point3(0.0, 0.0, 0.0);
};

class HeadingFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
    using This = HeadingFactor;

    HeadingFactor() = default;
    HeadingFactor(Key key, double heading, const gtsam::SharedNoiseModel& model)
        : Base(model, key), heading_(heading) {}

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const auto* other = dynamic_cast<const This*>(&expected);
        return other != nullptr && Base::equals(*other, tol)
            && std::fabs(wrapAngle(heading_ - other->heading_)) <= tol;
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        gtsam::Vector1 error;
        error << wrapAngle(pose.rotation().yaw() - heading_);
        if (H)
        {
            *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
                [this](const gtsam::Pose3& value) {
                    gtsam::Vector1 result;
                    result << wrapAngle(value.rotation().yaw() - heading_);
                    return result;
                }, pose);
        }
        return error;
    }

private:
    double heading_ = 0.0;
};

gtsam::Matrix6 diagonalCovariance(double translation_sigma, double rotation_sigma)
{
    gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
    covariance.block<3, 3>(0, 0) = gtsam::Matrix3::Identity() * std::max(1e-6, rotation_sigma * rotation_sigma);
    covariance.block<3, 3>(3, 3) = gtsam::Matrix3::Identity() * std::max(1e-6, translation_sigma * translation_sigma);
    return covariance;
}

}  // namespace

GlobalFactorGraph::GlobalFactorGraph(GlobalFactorGraphConfig config) : config_(std::move(config)) {}

GlobalFactorGraphResult GlobalFactorGraph::optimize(
    const std::vector<GlobalGraphKeyframe>& keyframes,
    const std::vector<GlobalGraphLoopClosure>& loop_closures) const
{
    GlobalFactorGraphResult result;
    if (!config_.enable)
    {
        result.error = "global factor graph is disabled";
        return result;
    }
    if (keyframes.empty())
    {
        result.error = "no keyframes";
        return result;
    }
    if (keyframes.size() == 1)
    {
        result.success = true;
        result.optimized_poses.push_back(keyframes.front().initial_pose);
        result.covariances.push_back(diagonalCovariance(1.0, 0.5));
        result.factor_count = 1;
        result.error_before = result.error_after = 0.0;
        return result;
    }

    try
    {
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;
        for (std::size_t i = 0; i < keyframes.size(); ++i)
        {
            initial.insert(gtsam::Symbol('x', i), keyframes[i].initial_pose);
            if (config_.use_imu_factor)
            {
                initial.insert(gtsam::Symbol('v', i), keyframes[i].initial_velocity);
                initial.insert(gtsam::Symbol('b', i), keyframes[i].initial_bias);
            }
        }

        const auto prior_sigmas = (gtsam::Vector(6) <<
            config_.prior_rotation_sigma_rad, config_.prior_rotation_sigma_rad, config_.prior_rotation_sigma_rad,
            config_.prior_translation_sigma, config_.prior_translation_sigma, config_.prior_translation_sigma).finished();
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            gtsam::Symbol('x', 0), keyframes.front().initial_pose,
            robustDiagonal(prior_sigmas, config_.robust_huber_k)));

        boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params;
        if (config_.use_imu_factor)
        {
            imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(
                std::max(1e-6, config_.gravity_magnitude));
            imu_params->setAccelerometerCovariance(gtsam::Matrix3::Identity()
                * std::pow(std::max(1e-8, config_.imu_accelerometer_noise_sigma), 2));
            imu_params->setGyroscopeCovariance(gtsam::Matrix3::Identity()
                * std::pow(std::max(1e-8, config_.imu_gyroscope_noise_sigma), 2));
            imu_params->setIntegrationCovariance(gtsam::Matrix3::Identity()
                * std::pow(std::max(1e-10, config_.imu_integration_noise_sigma), 2));
            imu_params->setBiasAccCovariance(gtsam::Matrix3::Identity()
                * std::pow(std::max(1e-10, config_.imu_accel_bias_random_walk_sigma), 2));
            imu_params->setBiasOmegaCovariance(gtsam::Matrix3::Identity()
                * std::pow(std::max(1e-10, config_.imu_gyro_bias_random_walk_sigma), 2));
            imu_params->setBiasAccOmegaInit(gtsam::Matrix6::Identity()
                * std::pow(std::max(1e-8, config_.imu_bias_prior_sigma), 2));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(
                gtsam::Symbol('v', 0), keyframes.front().initial_velocity,
                gtsam::noiseModel::Isotropic::Sigma(
                    3, std::max(1e-6, config_.imu_velocity_prior_sigma))));
            graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
                gtsam::Symbol('b', 0), keyframes.front().initial_bias,
                gtsam::noiseModel::Isotropic::Sigma(
                    6, std::max(1e-6, config_.imu_bias_prior_sigma))));
            result.imu_velocity_prior_factor_count = 1;
        }

        for (std::size_t i = 1; i < keyframes.size(); ++i)
        {
            const Key previous = gtsam::Symbol('x', i - 1);
            const Key current = gtsam::Symbol('x', i);
            const Pose3 lio_delta = keyframes[i - 1].initial_pose.between(keyframes[i].initial_pose);
            // This relative pose comes from the tightly-coupled FAST-LIO2 ESKF
            // trajectory. The ndt_* noise parameter names are legacy configuration
            // keys and do not imply that an NDT registration is performed here.
            const auto lio_sigmas = (gtsam::Vector(6) <<
                config_.ndt_rotation_sigma_rad, config_.ndt_rotation_sigma_rad, config_.ndt_rotation_sigma_rad,
                config_.ndt_translation_sigma, config_.ndt_translation_sigma, config_.ndt_translation_sigma).finished();
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                previous, current, lio_delta, robustDiagonal(lio_sigmas, config_.robust_huber_k)));
            ++result.lio_between_factor_count;
            // Deprecated telemetry alias used by older edge/frontend versions.
            ++result.ndt_factor_count;

            const auto& frame = keyframes[i];
            bool imu_kinematics_valid = true;
            if (config_.use_imu_factor && frame.has_imu_preintegration)
            {
                const auto& previous_frame = keyframes[i - 1];
                const double pose_dt = frame.stamp - previous_frame.stamp;
                if (std::isfinite(pose_dt) && pose_dt > 1e-6
                    && frame.initial_velocity.allFinite()
                    && previous_frame.initial_velocity.allFinite())
                {
                    const gtsam::Vector3 pose_displacement =
                        frame.initial_pose.translation() - previous_frame.initial_pose.translation();
                    const gtsam::Vector3 velocity_displacement =
                        0.5 * (previous_frame.initial_velocity + frame.initial_velocity) * pose_dt;
                    const double residual_mps =
                        (pose_displacement - velocity_displacement).norm() / pose_dt;
                    if (std::isfinite(residual_mps))
                    {
                        result.max_imu_kinematic_residual_mps = std::max(
                            result.max_imu_kinematic_residual_mps, residual_mps);
                        imu_kinematics_valid =
                            config_.imu_kinematic_gate_max_residual_mps <= 0.0
                            || residual_mps <= config_.imu_kinematic_gate_max_residual_mps;
                    }
                }
            }
            if (config_.use_imu_factor && frame.has_imu_preintegration && imu_kinematics_valid)
            {
                gtsam::PreintegratedCombinedMeasurements preintegrated(imu_params, frame.imu_bias_hat);
                double integrated_time = 0.0;
                for (const auto& measurement : frame.imu_measurements)
                {
                    if (!std::isfinite(measurement.delta_t) || measurement.delta_t <= 0.0
                        || measurement.delta_t > 0.25 || !measurement.acceleration.allFinite()
                        || !measurement.angular_velocity.allFinite())
                        continue;
                    preintegrated.integrateMeasurement(
                        measurement.acceleration, measurement.angular_velocity, measurement.delta_t);
                    integrated_time += measurement.delta_t;
                }
                if (integrated_time > 1e-6)
                {
                    graph.add(gtsam::CombinedImuFactor(
                        previous, gtsam::Symbol('v', i - 1), current, gtsam::Symbol('v', i),
                        gtsam::Symbol('b', i - 1), gtsam::Symbol('b', i), preintegrated));
                    ++result.imu_factor_count;
                    // CombinedImuFactor embeds the bias random-walk transition.
                    ++result.imu_bias_factor_count;
                }
                else
                {
                    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
                        gtsam::Symbol('v', i), frame.initial_velocity,
                        gtsam::noiseModel::Isotropic::Sigma(
                            3, std::max(1e-6, config_.imu_velocity_prior_sigma))));
                    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
                        gtsam::Symbol('b', i), frame.initial_bias,
                        gtsam::noiseModel::Isotropic::Sigma(
                            6, std::max(1e-6, config_.imu_bias_prior_sigma))));
                    ++result.imu_velocity_prior_factor_count;
                }
            }
            else if (config_.use_imu_factor)
            {
                if (frame.has_imu_preintegration && !imu_kinematics_valid)
                    ++result.imu_kinematic_rejected_factor_count;
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(
                    gtsam::Symbol('v', i), frame.initial_velocity,
                    gtsam::noiseModel::Isotropic::Sigma(
                        3, std::max(1e-6, config_.imu_velocity_prior_sigma))));
                graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
                    gtsam::Symbol('b', i), frame.initial_bias,
                    gtsam::noiseModel::Isotropic::Sigma(
                        6, std::max(1e-6, config_.imu_bias_prior_sigma))));
                ++result.imu_velocity_prior_factor_count;
            }
        }

        for (const auto& frame : keyframes)
        {
            const Key key = gtsam::Symbol('x', frame.index);
            if (frame.has_rtk_position)
            {
                const double sigma = std::max(config_.rtk_position_sigma_floor, frame.rtk_position_sigma);
                const auto sigmas = (gtsam::Vector(3) << sigma, sigma, sigma).finished();
                graph.add(LeverArmPositionFactor(key, frame.rtk_position, frame.rtk_lever_arm,
                    robustDiagonal(sigmas, config_.robust_huber_k)));
                ++result.rtk_position_factor_count;
            }
            if (frame.has_rtk_heading)
            {
                const double sigma = std::max(config_.rtk_heading_sigma_floor_rad, frame.rtk_heading_sigma_rad);
                graph.add(HeadingFactor(key, frame.rtk_heading_rad,
                    robustDiagonal((gtsam::Vector(1) << sigma).finished(), config_.robust_huber_k)));
                ++result.rtk_heading_factor_count;
            }
        }

        if (config_.use_loop)
        {
            for (const auto& loop : loop_closures)
            {
                if (loop.from >= keyframes.size() || loop.to >= keyframes.size() || loop.from == loop.to)
                    continue;
                gtsam::Matrix6 covariance = loop.covariance;
                if (!covariance.allFinite() || (covariance.diagonal().array() <= 0.0).any())
                    covariance = diagonalCovariance(config_.loop_translation_sigma, config_.loop_rotation_sigma_rad);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    gtsam::Symbol('x', loop.from), gtsam::Symbol('x', loop.to), loop.relative_pose,
                    robustCovariance(covariance, config_.robust_huber_k)));
                ++result.loop_closure_factor_count;
            }
        }

        result.factor_count = graph.size();
        result.error_before = graph.error(initial);
        gtsam::LevenbergMarquardtParams params;
        params.maxIterations = std::max(1, config_.max_iterations);
        params.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
        const gtsam::Values optimized = optimizer.optimize();
        result.error_after = graph.error(optimized);
        if (config_.use_imu_factor)
        {
            result.final_velocity = optimized.at<gtsam::Vector3>(
                gtsam::Symbol('v', keyframes.size() - 1));
            result.final_bias = optimized.at<gtsam::imuBias::ConstantBias>(
                gtsam::Symbol('b', keyframes.size() - 1));
        }
        result.optimized_poses.reserve(keyframes.size());
        result.covariances.reserve(keyframes.size());
        gtsam::Marginals marginals(graph, optimized, gtsam::Marginals::QR);
        double max_jump = 0.0;
        double max_abs_z = 0.0;
        double initial_z_min = keyframes.front().initial_pose.z();
        double initial_z_max = initial_z_min;
        double optimized_z_min = std::numeric_limits<double>::infinity();
        double optimized_z_max = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < keyframes.size(); ++i)
        {
            const Key key = gtsam::Symbol('x', i);
            const Pose3 pose = optimized.at<Pose3>(key);
            const auto& initial = keyframes[i].initial_pose;
            result.optimized_poses.push_back(pose);
            const gtsam::Matrix covariance = marginals.marginalCovariance(key);
            gtsam::Matrix6 covariance6 = gtsam::Matrix6::Identity();
            if (covariance.rows() == 6 && covariance.cols() == 6 && covariance.allFinite())
                covariance6 = covariance;
            result.covariances.push_back(covariance6);
            max_jump = std::max(max_jump, (pose.translation() - initial.translation()).norm());
            max_abs_z = std::max(max_abs_z, std::fabs(pose.z() - initial.z()));
            initial_z_min = std::min(initial_z_min, initial.z());
            initial_z_max = std::max(initial_z_max, initial.z());
            optimized_z_min = std::min(optimized_z_min, pose.z());
            optimized_z_max = std::max(optimized_z_max, pose.z());
        }
        const double z_span_growth = (optimized_z_max - optimized_z_min) - (initial_z_max - initial_z_min);
        if (max_jump > config_.max_pose_jump_m || max_abs_z > config_.max_abs_z_change_m
            || z_span_growth > config_.max_abs_z_change_m)
        {
            result.success = false;
            result.optimized_poses.clear();
            result.covariances.clear();
            std::ostringstream message;
            message << "optimization rejected: max_jump=" << max_jump
                    << "m max_abs_z=" << max_abs_z << "m z_span_growth=" << z_span_growth << "m";
            result.error = message.str();
            return result;
        }
        result.success = true;
    }
    catch (const std::exception& exception)
    {
        result.error = exception.what();
    }
    catch (...)
    {
        result.error = "unknown GTSAM exception";
    }
    return result;
}

}  // namespace robot::slam
