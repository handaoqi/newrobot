#include <gtest/gtest.h>

#include <localization/anchor_ukf.hpp>

namespace localization {
namespace {

TEST(AnchorUkf, FusesMeasurementsByAdaptiveVariance) {
  AnchorUkf filter;
  filter.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 1.0, 1.0));
  AnchorObservation rtk;
  rtk.value = Eigen::Vector3d(1.0, 0.0, 0.2);
  rtk.variance = Eigen::Vector3d(0.01, 0.01, 0.01);
  rtk.source = "rtk";
  ASSERT_TRUE(filter.update(rtk).accepted);
  EXPECT_GT(filter.value().x(), 0.98);

  const double after_rtk = filter.value().x();
  AnchorObservation ndt;
  ndt.value = Eigen::Vector3d(3.0, 0.0, 0.2);
  ndt.variance = Eigen::Vector3d(1.0, 1.0, 1.0);
  ndt.source = "ndt";
  ASSERT_TRUE(filter.update(ndt).accepted);
  EXPECT_LT(filter.value().x() - after_rtk, 0.05);
}

TEST(AnchorUkf, WrapsYawInnovationAcrossPi) {
  AnchorUkf filter;
  filter.reset(
    Eigen::Vector3d(0.0, 0.0, 179.0 * M_PI / 180.0),
    Eigen::Vector3d(0.1, 0.1, 0.1));
  AnchorObservation observation;
  observation.value = Eigen::Vector3d(0.0, 0.0, -179.0 * M_PI / 180.0);
  observation.variance = Eigen::Vector3d(0.1, 0.1, 0.1);
  const auto result = filter.update(observation);
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(result.innovation.z(), 2.0 * M_PI / 180.0, 1.0e-6);
  EXPECT_NEAR(std::fabs(filter.value().z()), M_PI, 1.0e-6);
}

TEST(AnchorUkf, PredictionGrowsCovarianceWithoutMovingMean) {
  AnchorUkf filter;
  filter.reset(Eigen::Vector3d(2.0, 3.0, 0.4), Eigen::Vector3d(0.01, 0.01, 0.01));
  const auto before = filter.value();
  filter.predict(2.0, 0.5, 0.0025, 0.01);
  EXPECT_TRUE(filter.value().isApprox(before));
  EXPECT_NEAR(filter.covariance()(0, 0), 0.015, 1.0e-9);
  EXPECT_NEAR(filter.covariance()(2, 2), 0.015, 1.0e-9);
}

TEST(AnchorUkf, RejectsInvalidObservation) {
  AnchorUkf filter;
  AnchorObservation observation;
  observation.value.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(filter.update(observation).accepted);
}

}  // namespace
}  // namespace localization
