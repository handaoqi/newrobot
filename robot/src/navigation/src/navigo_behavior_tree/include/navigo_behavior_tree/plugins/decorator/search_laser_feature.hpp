#ifndef NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEARCH_LASER_FEATURE_HPP_
#define NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEARCH_LASER_FEATURE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "localization/msg/scan_matching_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace navigo_behavior_tree
{

class SearchLaserFeature : public BT::DecoratorNode
{
public:
  SearchLaserFeature(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void onScanMatchingStatus(const localization::msg::ScanMatchingStatus::SharedPtr msg);
  void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool ndtHealthy() const;
  bool laserFresh() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<localization::msg::ScanMatchingStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::atomic<bool> converged_{false};
  std::atomic<double> score_{0.0};
  std::atomic<int64_t> status_received_ns_{0};
  std::atomic<int64_t> scan_received_ns_{0};
  double score_threshold_{0.4};
  double max_age_seconds_{0.75};
};

}  // namespace navigo_behavior_tree

#endif  // NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__SEARCH_LASER_FEATURE_HPP_
