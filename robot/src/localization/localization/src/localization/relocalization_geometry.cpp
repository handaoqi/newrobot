#include "localization/relocalization_geometry.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace localization {
namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

CloudT::Ptr finiteVoxelCloud(const CloudT& input, float leaf_size) {
  auto finite = pcl::make_shared<CloudT>();
  finite->reserve(input.size());
  for (const auto& point : input) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite->push_back(point);
    }
  }
  finite->width = static_cast<std::uint32_t>(finite->size());
  finite->height = 1;
  finite->is_dense = true;
  if (leaf_size <= 0.0f || finite->empty()) {
    return finite;
  }

  auto filtered = pcl::make_shared<CloudT>();
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(finite);
  voxel.filter(*filtered);
  return filtered;
}

double rotationDistance(const Eigen::Matrix3f& lhs, const Eigen::Matrix3f& rhs) {
  const Eigen::Matrix3f delta = lhs.transpose() * rhs;
  const double cosine = std::clamp(
    (static_cast<double>(delta.trace()) - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

std::pair<int, double> directedOverlap(
    const CloudT::ConstPtr& source,
    const CloudT::ConstPtr& target,
    float max_distance) {
  if (source->empty() || target->empty()) {
    return {0, 0.0};
  }
  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  const float max_squared = max_distance * max_distance;
  int inliers = 0;
  for (const auto& point : *source) {
    if (tree.nearestKSearch(point, 1, indices, distances) > 0 &&
        distances.front() <= max_squared) {
      ++inliers;
    }
  }
  return {inliers, static_cast<double>(inliers) / source->size()};
}

bool finiteTransform(const Eigen::Matrix4f& transform) {
  return transform.array().isFinite().all();
}

}  // namespace

bool acceptRelocalizationGeometry(
    const RelocalizationGeometryConfig& config,
    RelocalizationGeometryResult& result) {
  result.accepted = false;
  result.rejection_reason.clear();
  const auto reject = [&result](const char* reason) {
    result.rejection_reason = reason;
    return false;
  };

  if (result.source_points < config.min_points || result.target_points < config.min_points) {
    return reject("insufficient_points");
  }
  if (!result.gicp_converged || !finiteTransform(result.source_to_target)) {
    return reject("no_convergence");
  }
  if (!std::isfinite(result.rmse_m) || result.rmse_m > config.max_rmse_m) {
    return reject("rmse_too_high");
  }
  if (result.inlier_count < config.min_inliers) {
    return reject("insufficient_inliers");
  }
  if (!std::isfinite(result.bidirectional_overlap) ||
      result.bidirectional_overlap < config.min_bidirectional_overlap) {
    return reject("low_overlap");
  }
  if (!std::isfinite(result.seed_translation_delta_m) ||
      !std::isfinite(result.seed_rotation_delta_rad) ||
      result.seed_translation_delta_m > config.max_seed_translation_delta_m ||
      result.seed_rotation_delta_rad > config.max_seed_rotation_delta_rad) {
    return reject("pose_inconsistent");
  }
  if (!std::isfinite(result.vertical_delta_m) ||
      result.vertical_delta_m > config.max_vertical_delta_m) {
    return reject("vertical_inconsistent");
  }
  if (!result.hessian_positive_definite || !std::isfinite(result.hessian_condition) ||
      result.hessian_condition > config.max_hessian_condition) {
    return reject("degenerate_hessian");
  }
  if (!result.icp_converged || !finiteTransform(result.icp_source_to_target) ||
      !std::isfinite(result.icp_translation_disagreement_m) ||
      !std::isfinite(result.icp_rotation_disagreement_rad) ||
      result.icp_translation_disagreement_m > config.max_icp_translation_disagreement_m ||
      result.icp_rotation_disagreement_rad > config.max_icp_rotation_disagreement_rad) {
    return reject("icp_disagreement");
  }

  result.accepted = true;
  return true;
}

RelocalizationGeometryVerifier::RelocalizationGeometryVerifier(
    RelocalizationGeometryConfig config)
: config_(std::move(config)) {}

RelocalizationGeometryResult RelocalizationGeometryVerifier::verify(
    const CloudT& source,
    const CloudT& target,
    const Eigen::Matrix4f& initial_source_to_target) const {
  RelocalizationGeometryResult result;
  const auto start = std::chrono::steady_clock::now();
  const auto finish = [&result, start]() {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  };

  const auto source_filtered = finiteVoxelCloud(source, config_.voxel_size_m);
  const auto target_filtered = finiteVoxelCloud(target, config_.voxel_size_m);
  result.source_points = static_cast<int>(source_filtered->size());
  result.target_points = static_cast<int>(target_filtered->size());
  if (result.source_points < config_.min_points || result.target_points < config_.min_points ||
      !finiteTransform(initial_source_to_target)) {
    acceptRelocalizationGeometry(config_, result);
    finish();
    return result;
  }

  CloudT aligned;
  fast_gicp::FastGICP<PointT, PointT> gicp;
  gicp.setNumThreads(std::max(1, config_.num_threads));
  gicp.setCorrespondenceRandomness(std::max(5, config_.correspondence_randomness));
  gicp.setMaximumIterations(std::max(1, config_.max_iterations));
  gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance_m);
  gicp.setTransformationEpsilon(1e-4);
  gicp.setRotationEpsilon(1e-4);
  gicp.setInputSource(source_filtered);
  gicp.setInputTarget(target_filtered);
  gicp.align(aligned, initial_source_to_target);
  result.gicp_converged = gicp.hasConverged();
  result.source_to_target = gicp.getFinalTransformation();
  const double fitness = gicp.getFitnessScore(config_.max_correspondence_distance_m);
  result.rmse_m = fitness >= 0.0 ? std::sqrt(fitness) :
    std::numeric_limits<double>::infinity();

  if (result.gicp_converged && finiteTransform(result.source_to_target)) {
    const Eigen::Matrix4f seed_delta = initial_source_to_target.inverse() *
      result.source_to_target;
    result.seed_translation_delta_m = seed_delta.block<3, 1>(0, 3).norm();
    result.seed_rotation_delta_rad = rotationDistance(
      initial_source_to_target.block<3, 3>(0, 0),
      result.source_to_target.block<3, 3>(0, 0));
    result.vertical_delta_m = std::abs(
      result.source_to_target(2, 3) - initial_source_to_target(2, 3));

    const Eigen::Matrix<double, 6, 6> hessian = gicp.getFinalHessian();
    if (hessian.array().isFinite().all()) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(hessian);
      if (solver.info() == Eigen::Success) {
        const auto eigenvalues = solver.eigenvalues();
        const double minimum = eigenvalues.minCoeff();
        const double maximum = eigenvalues.maxCoeff();
        result.hessian_positive_definite = minimum > 1e-9 && maximum > 0.0;
        result.hessian_condition = result.hessian_positive_definite ?
          maximum / minimum : std::numeric_limits<double>::infinity();
      }
    }

    auto aligned_ptr = pcl::make_shared<CloudT>();
    pcl::transformPointCloud(*source_filtered, *aligned_ptr, result.source_to_target);
    const auto source_overlap = directedOverlap(
      aligned_ptr, target_filtered, config_.max_correspondence_distance_m);
    const auto target_overlap = directedOverlap(
      target_filtered, aligned_ptr, config_.max_correspondence_distance_m);
    result.source_inliers = source_overlap.first;
    result.target_inliers = target_overlap.first;
    result.inlier_count = std::min(result.source_inliers, result.target_inliers);
    result.source_overlap = source_overlap.second;
    result.target_overlap = target_overlap.second;
    result.bidirectional_overlap = std::min(result.source_overlap, result.target_overlap);
  }

  CloudT icp_aligned;
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(std::max(1, config_.max_iterations));
  icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance_m);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-5);
  icp.setInputSource(source_filtered);
  icp.setInputTarget(target_filtered);
  icp.align(icp_aligned, initial_source_to_target);
  result.icp_converged = icp.hasConverged();
  result.icp_source_to_target = icp.getFinalTransformation();
  if (result.gicp_converged && result.icp_converged &&
      finiteTransform(result.source_to_target) && finiteTransform(result.icp_source_to_target)) {
    result.icp_translation_disagreement_m =
      (result.source_to_target.block<3, 1>(0, 3) -
       result.icp_source_to_target.block<3, 1>(0, 3)).norm();
    result.icp_rotation_disagreement_rad = rotationDistance(
      result.source_to_target.block<3, 3>(0, 0),
      result.icp_source_to_target.block<3, 3>(0, 0));
  } else {
    result.icp_translation_disagreement_m = std::numeric_limits<double>::infinity();
    result.icp_rotation_disagreement_rad = std::numeric_limits<double>::infinity();
  }

  acceptRelocalizationGeometry(config_, result);
  finish();
  return result;
}

}  // namespace localization
