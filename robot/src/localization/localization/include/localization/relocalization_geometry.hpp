#ifndef LOCALIZATION_RELOCALIZATION_GEOMETRY_HPP
#define LOCALIZATION_RELOCALIZATION_GEOMETRY_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <string>

namespace localization {

struct RelocalizationGeometryConfig {
  float voxel_size_m = 0.20f;
  float max_correspondence_distance_m = 1.0f;
  int max_iterations = 30;
  int num_threads = 2;
  int correspondence_randomness = 20;

  int min_points = 200;
  int min_inliers = 500;
  double min_bidirectional_overlap = 0.45;
  double max_rmse_m = 0.25;
  double max_seed_translation_delta_m = 3.0;
  double max_seed_rotation_delta_rad = 20.0 * 3.14159265358979323846 / 180.0;
  double max_vertical_delta_m = 0.75;
  double max_hessian_condition = 1e6;
  double max_icp_translation_disagreement_m = 0.30;
  double max_icp_rotation_disagreement_rad = 3.0 * 3.14159265358979323846 / 180.0;
};

struct RelocalizationGeometryResult {
  bool gicp_converged = false;
  bool icp_converged = false;
  Eigen::Matrix4f source_to_target = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f icp_source_to_target = Eigen::Matrix4f::Identity();
  int source_points = 0;
  int target_points = 0;
  int source_inliers = 0;
  int target_inliers = 0;
  int inlier_count = 0;
  double source_overlap = 0.0;
  double target_overlap = 0.0;
  double bidirectional_overlap = 0.0;
  double rmse_m = 0.0;
  double seed_translation_delta_m = 0.0;
  double seed_rotation_delta_rad = 0.0;
  double vertical_delta_m = 0.0;
  bool hessian_positive_definite = false;
  double hessian_condition = 0.0;
  double icp_translation_disagreement_m = 0.0;
  double icp_rotation_disagreement_rad = 0.0;
  double elapsed_ms = 0.0;
  bool accepted = false;
  std::string rejection_reason;
};

/// Apply the safety gates to already-computed metrics. Kept separate for unit tests.
bool acceptRelocalizationGeometry(
  const RelocalizationGeometryConfig& config,
  RelocalizationGeometryResult& result);

/**
 * @brief Real 3D FastGICP verification with ordinary ICP as an independent cross-check.
 *
 * The returned transform maps source/query lidar coordinates into target/candidate
 * lidar coordinates. This class is ROS-free and suitable for both offline map checks
 * and a bounded background runtime worker.
 */
class RelocalizationGeometryVerifier {
public:
  explicit RelocalizationGeometryVerifier(RelocalizationGeometryConfig config = {});

  const RelocalizationGeometryConfig& config() const { return config_; }

  RelocalizationGeometryResult verify(
    const pcl::PointCloud<pcl::PointXYZI>& source,
    const pcl::PointCloud<pcl::PointXYZI>& target,
    const Eigen::Matrix4f& initial_source_to_target) const;

private:
  RelocalizationGeometryConfig config_;
};

}  // namespace localization

#endif  // LOCALIZATION_RELOCALIZATION_GEOMETRY_HPP
