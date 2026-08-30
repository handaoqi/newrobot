#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace robot_navigo {

// The Livox bridge stamps /front_lidar at the beginning of the accumulated
// frame.  Its FLOAT64 `timestamp` field is a per-point nanosecond offset from
// that header stamp.  Health checks need the newest measurement in the frame,
// not the oldest one, otherwise a healthy 10 Hz frame appears about 100 ms
// older than it really is.
inline std::optional<int64_t> PointCloudEndOffsetNs(
    const sensor_msgs::msg::PointCloud2 &cloud) {
  const auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [](const sensor_msgs::msg::PointField &candidate) {
        return candidate.name == "timestamp" &&
               candidate.datatype == sensor_msgs::msg::PointField::FLOAT64 &&
               candidate.count == 1;
      });
  if (field == cloud.fields.end() || cloud.point_step == 0 ||
      field->offset + sizeof(double) > cloud.point_step) {
    return std::nullopt;
  }

  const std::size_t point_count = cloud.data.size() / cloud.point_step;
  if (point_count == 0) {
    return std::nullopt;
  }

  const uint16_t endian_probe = 1;
  const bool host_big_endian =
      *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  double maximum = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < point_count; ++index) {
    const auto byte_offset = index * cloud.point_step + field->offset;
    uint8_t bytes[sizeof(double)];
    std::memcpy(bytes, cloud.data.data() + byte_offset, sizeof(double));
    if (cloud.is_bigendian != host_big_endian) {
      std::reverse(std::begin(bytes), std::end(bytes));
    }
    double value = 0.0;
    std::memcpy(&value, bytes, sizeof(double));
    if (std::isfinite(value)) {
      maximum = std::max(maximum, value);
    }
  }

  // Livox offsets are nanoseconds and a navigation frame is ~100 ms. Reject
  // absolute epoch timestamps and corrupt values instead of silently adding
  // them to the header stamp.
  constexpr double kMaximumPlausibleFrameNs = 10.0 * 1e9;
  if (!std::isfinite(maximum) || maximum < 0.0 ||
      maximum > kMaximumPlausibleFrameNs) {
    return std::nullopt;
  }
  return static_cast<int64_t>(std::llround(maximum));
}

}  // namespace robot_navigo
