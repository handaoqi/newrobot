// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "navigo_collision_monitor/scan.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "navigo_util/node_utils.hpp"

namespace navigo_collision_monitor
{

Scan::Scan(
  const navigo_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure()
{
  Source::configure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  getCommonParameters(source_topic);
  navigo_util::declare_parameter_if_not_declared(
    node, source_name_ + ".cache_by_stamp", rclcpp::ParameterValue(true));
  cache_by_stamp_ = node->get_parameter(source_name_ + ".cache_by_stamp").as_bool();
  navigo_util::declare_parameter_if_not_declared(
    node, source_name_ + ".performance_log_interval_seconds", rclcpp::ParameterValue(10.0));
  performance_log_interval_seconds_ = std::max(
    1.0, node->get_parameter(source_name_ + ".performance_log_interval_seconds").as_double());

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS().keep_last(1);
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

void Scan::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    scan = data_;
  }
  if (scan == nullptr) {
    return;
  }
  if (!sourceValid(scan->header.stamp, curr_time)) {
    return;
  }

  const auto conversion_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> cache_lock(cache_mutex_);
  const bool same_scan = cache_valid_ &&
    cached_stamp_sec_ == scan->header.stamp.sec &&
    cached_stamp_nanosec_ == scan->header.stamp.nanosec &&
    cached_frame_id_ == scan->header.frame_id;
  if (cache_by_stamp_ && same_scan) {
    data.insert(data.end(), cached_points_.begin(), cached_points_.end());
    ++cache_hits_;
  } else {
    tf2::Transform tf_transform;
    const bool already_in_base = !base_shift_correction_ &&
      scan->header.frame_id == base_frame_id_;
    if (!already_in_base) {
      if (base_shift_correction_) {
        if (!navigo_util::getTransform(
            scan->header.frame_id, scan->header.stamp,
            base_frame_id_, curr_time, global_frame_id_,
            transform_tolerance_, tf_buffer_, tf_transform))
        {
          return;
        }
      } else if (!navigo_util::getTransform(
          scan->header.frame_id, base_frame_id_,
          transform_tolerance_, tf_buffer_, tf_transform))
      {
        return;
      }
    }

    std::vector<Point> converted;
    converted.reserve(scan->ranges.size());
    float angle = scan->angle_min;
    for (const float range : scan->ranges) {
      if (range >= scan->range_min && range <= scan->range_max) {
        const tf2::Vector3 point_scan(range * std::cos(angle), range * std::sin(angle), 0.0);
        const tf2::Vector3 point_base = already_in_base ? point_scan : tf_transform * point_scan;
        converted.push_back({point_base.x(), point_base.y()});
      }
      angle += scan->angle_increment;
    }
    data.insert(data.end(), converted.begin(), converted.end());
    cached_points_ = std::move(converted);
    cached_stamp_sec_ = scan->header.stamp.sec;
    cached_stamp_nanosec_ = scan->header.stamp.nanosec;
    cached_frame_id_ = scan->header.frame_id;
    cache_valid_ = true;
    ++converted_scans_;
    output_points_ += cached_points_.size();
  }

  conversion_milliseconds_ += std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - conversion_start).count();
  const auto now = std::chrono::steady_clock::now();
  if (last_performance_log_ == std::chrono::steady_clock::time_point{}) {
    last_performance_log_ = now;
  } else if (std::chrono::duration<double>(now - last_performance_log_).count() >=
      performance_log_interval_seconds_) {
    const std::uint64_t requests = converted_scans_ + cache_hits_;
    RCLCPP_INFO(
      logger_,
      "[%s] scan perf received=%llu requests=%llu converted=%llu cache_hits=%llu "
      "cache_hit_ratio=%.2f avg_points=%.1f avg_get_data_ms=%.3f age_ms=%.1f",
      source_name_.c_str(),
      static_cast<unsigned long long>(received_scans_),
      static_cast<unsigned long long>(requests),
      static_cast<unsigned long long>(converted_scans_),
      static_cast<unsigned long long>(cache_hits_),
      requests > 0 ? static_cast<double>(cache_hits_) / requests : 0.0,
      converted_scans_ > 0 ? static_cast<double>(output_points_) / converted_scans_ : 0.0,
      requests > 0 ? conversion_milliseconds_ / requests : 0.0,
      std::max(0.0, (curr_time - rclcpp::Time(scan->header.stamp)).seconds() * 1000.0));
    received_scans_ = 0;
    converted_scans_ = 0;
    cache_hits_ = 0;
    output_points_ = 0;
    conversion_milliseconds_ = 0.0;
    last_performance_log_ = now;
  }
}

bool Scan::isFresh(const rclcpp::Time & curr_time) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return data_ != nullptr && sourceValid(data_->header.stamp, curr_time);
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    data_ = msg;
  }
  std::lock_guard<std::mutex> cache_lock(cache_mutex_);
  ++received_scans_;
}

}  // namespace navigo_collision_monitor
