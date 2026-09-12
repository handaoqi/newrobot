#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/decorator/search_laser_feature.hpp"

namespace navigo_behavior_tree
{

SearchLaserFeature::SearchLaserFeature(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  status_subscription_ = node_->create_subscription<localization::msg::ScanMatchingStatus>(
    "/status", rclcpp::SensorDataQoS(),
    std::bind(&SearchLaserFeature::onScanMatchingStatus, this, std::placeholders::_1));
  scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/laser_scan", rclcpp::SensorDataQoS(),
    std::bind(&SearchLaserFeature::onLaserScan, this, std::placeholders::_1));
}

BT::PortsList SearchLaserFeature::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "Whether feature search is allowed"),
    BT::InputPort<double>("score_threshold", 0.4, "Maximum healthy NDT score"),
    BT::InputPort<double>("max_age_seconds", 0.75, "Maximum status and scan age")};
}

void SearchLaserFeature::onScanMatchingStatus(
  const localization::msg::ScanMatchingStatus::SharedPtr msg)
{
  converged_.store(msg && msg->has_converged);
  score_.store(msg ? static_cast<double>(msg->matching_error) : 0.0);
  status_received_ns_.store(node_->now().nanoseconds());
}

void SearchLaserFeature::onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (msg) {
    scan_received_ns_.store(node_->now().nanoseconds());
  }
}

bool SearchLaserFeature::ndtHealthy() const
{
  const auto received = status_received_ns_.load();
  const double age = received > 0
    ? static_cast<double>(node_->now().nanoseconds() - received) / 1.0e9
    : INFINITY;
  const double score = score_.load();
  return converged_.load() && std::isfinite(score) && score < score_threshold_ &&
    age >= 0.0 && age <= max_age_seconds_;
}

bool SearchLaserFeature::laserFresh() const
{
  const auto received = scan_received_ns_.load();
  const double age = received > 0
    ? static_cast<double>(node_->now().nanoseconds() - received) / 1.0e9
    : INFINITY;
  return age >= 0.0 && age <= max_age_seconds_;
}

BT::NodeStatus SearchLaserFeature::tick()
{
  bool enabled = true;
  getInput("enabled", enabled);
  getInput("score_threshold", score_threshold_);
  getInput("max_age_seconds", max_age_seconds_);
  if (!enabled || !laserFresh()) {
    if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING) {
      child_node_->halt();
    }
    return BT::NodeStatus::FAILURE;
  }
  setStatus(BT::NodeStatus::RUNNING);
  if (ndtHealthy()) {
    if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING) {
      child_node_->halt();
    }
    return BT::NodeStatus::SUCCESS;
  }
  const auto child_status = child_node_->executeTick();
  if (ndtHealthy()) {
    if (child_node_->status() == BT::NodeStatus::RUNNING) {
      child_node_->halt();
    }
    return BT::NodeStatus::SUCCESS;
  }
  return child_status == BT::NodeStatus::SUCCESS ? BT::NodeStatus::FAILURE : child_status;
}

void SearchLaserFeature::halt()
{
  if (child_node_) {
    child_node_->halt();
  }
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::SearchLaserFeature>("SearchLaserFeature");
}
