#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "pointcloud_timestamp.hpp"

namespace {

sensor_msgs::msg::PointCloud2 CloudWithTimestamps(
    const std::vector<double> &timestamps) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = timestamps.size();
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(double);
  cloud.row_step = cloud.point_step * cloud.width;
  sensor_msgs::msg::PointField field;
  field.name = "timestamp";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT64;
  field.count = 1;
  cloud.fields.push_back(field);
  cloud.data.resize(cloud.row_step);
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    std::memcpy(
        cloud.data.data() + index * cloud.point_step, &timestamps[index],
        sizeof(double));
  }
  return cloud;
}

TEST(PointCloudTimestamp, UsesNewestPerPointNanosecondOffset) {
  const auto cloud = CloudWithTimestamps({0.0, 4947.0, 100309965.0});
  const auto offset = robot_navigo::PointCloudEndOffsetNs(cloud);
  ASSERT_TRUE(offset.has_value());
  EXPECT_EQ(*offset, 100309965);
}

TEST(PointCloudTimestamp, DoesNotRequirePointsToBeOrdered) {
  const auto cloud = CloudWithTimestamps({100000000.0, 0.0, 50000000.0});
  const auto offset = robot_navigo::PointCloudEndOffsetNs(cloud);
  ASSERT_TRUE(offset.has_value());
  EXPECT_EQ(*offset, 100000000);
}

TEST(PointCloudTimestamp, RejectsMissingOrAbsoluteTimestampFields) {
  sensor_msgs::msg::PointCloud2 missing;
  EXPECT_FALSE(robot_navigo::PointCloudEndOffsetNs(missing).has_value());

  const auto absolute = CloudWithTimestamps({1788085365400126464.0});
  EXPECT_FALSE(robot_navigo::PointCloudEndOffsetNs(absolute).has_value());
}

}  // namespace
