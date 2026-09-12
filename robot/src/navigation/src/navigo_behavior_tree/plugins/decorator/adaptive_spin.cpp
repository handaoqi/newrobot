#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/decorator/adaptive_spin.hpp"

namespace navigo_behavior_tree
{

AdaptiveSpin::AdaptiveSpin(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  subscription_ = node_->create_subscription<localization::msg::ScanMatchingStatus>(
    "/status", rclcpp::SensorDataQoS(),
    std::bind(&AdaptiveSpin::onScanMatchingStatus, this, std::placeholders::_1));
}

BT::PortsList AdaptiveSpin::providedPorts()
{
  return {
    BT::InputPort<bool>("forbid_spin", true, "Fail closed unless Edge permits rotation"),
    BT::InputPort<double>("score_threshold", 0.4, "Maximum healthy NDT score"),
    BT::InputPort<double>("max_age_seconds", 0.75, "Maximum NDT status age")};
}

void AdaptiveSpin::onScanMatchingStatus(
  const localization::msg::ScanMatchingStatus::SharedPtr msg)
{
  converged_.store(msg && msg->has_converged);
  score_.store(msg ? static_cast<double>(msg->matching_error) : 0.0);
  received_ns_.store(node_->now().nanoseconds());
}

bool AdaptiveSpin::ndtHealthy() const
{
  const auto received = received_ns_.load();
  const double age = received > 0
    ? static_cast<double>(node_->now().nanoseconds() - received) / 1.0e9
    : INFINITY;
  const double score = score_.load();
  return converged_.load() && std::isfinite(score) && score < score_threshold_ &&
    age >= 0.0 && age <= max_age_seconds_;
}

BT::NodeStatus AdaptiveSpin::tick()
{
  // Fail closed: diagnosis must explicitly permit rotation. This guarantees
  // that an unavailable Edge diagnosis service cannot fall through to spin.
  bool forbid_spin = true;
  getInput("forbid_spin", forbid_spin);
  getInput("score_threshold", score_threshold_);
  getInput("max_age_seconds", max_age_seconds_);
  if (forbid_spin) {
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
  // Completing the requested angle without a healthy match is not a recovery.
  return child_status == BT::NodeStatus::SUCCESS ? BT::NodeStatus::FAILURE : child_status;
}

void AdaptiveSpin::halt()
{
  if (child_node_) {
    child_node_->halt();
  }
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::AdaptiveSpin>("AdaptiveSpin");
}
