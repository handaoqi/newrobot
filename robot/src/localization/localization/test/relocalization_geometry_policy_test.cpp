#include "localization/relocalization_geometry.hpp"

#include <gtest/gtest.h>

namespace {

localization::RelocalizationGeometryResult healthyResult() {
  localization::RelocalizationGeometryResult result;
  result.gicp_converged = true;
  result.icp_converged = true;
  result.source_points = 1000;
  result.target_points = 1000;
  result.inlier_count = 700;
  result.bidirectional_overlap = 0.60;
  result.rmse_m = 0.15;
  result.seed_translation_delta_m = 0.20;
  result.seed_rotation_delta_rad = 0.02;
  result.vertical_delta_m = 0.05;
  result.hessian_positive_definite = true;
  result.hessian_condition = 1000.0;
  result.icp_translation_disagreement_m = 0.05;
  result.icp_rotation_disagreement_rad = 0.01;
  return result;
}

TEST(RelocalizationGeometryPolicy, AcceptsHealthyResult) {
  localization::RelocalizationGeometryConfig config;
  auto result = healthyResult();
  EXPECT_TRUE(localization::acceptRelocalizationGeometry(config, result));
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.rejection_reason.empty());
}

TEST(RelocalizationGeometryPolicy, RejectsInsufficientPointsFirst) {
  localization::RelocalizationGeometryConfig config;
  auto result = healthyResult();
  result.source_points = config.min_points - 1;
  EXPECT_FALSE(localization::acceptRelocalizationGeometry(config, result));
  EXPECT_EQ(result.rejection_reason, "insufficient_points");
}

TEST(RelocalizationGeometryPolicy, RejectsLowOverlap) {
  localization::RelocalizationGeometryConfig config;
  auto result = healthyResult();
  result.bidirectional_overlap = config.min_bidirectional_overlap - 0.01;
  EXPECT_FALSE(localization::acceptRelocalizationGeometry(config, result));
  EXPECT_EQ(result.rejection_reason, "low_overlap");
}

TEST(RelocalizationGeometryPolicy, RejectsDegenerateHessian) {
  localization::RelocalizationGeometryConfig config;
  auto result = healthyResult();
  result.hessian_positive_definite = false;
  EXPECT_FALSE(localization::acceptRelocalizationGeometry(config, result));
  EXPECT_EQ(result.rejection_reason, "degenerate_hessian");
}

TEST(RelocalizationGeometryPolicy, RejectsIcpDisagreement) {
  localization::RelocalizationGeometryConfig config;
  auto result = healthyResult();
  result.icp_translation_disagreement_m = config.max_icp_translation_disagreement_m + 0.01;
  EXPECT_FALSE(localization::acceptRelocalizationGeometry(config, result));
  EXPECT_EQ(result.rejection_reason, "icp_disagreement");
}

}  // namespace
