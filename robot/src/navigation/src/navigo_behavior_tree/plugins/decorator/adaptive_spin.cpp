#include <cmath>
#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/decorator/adaptive_spin.hpp"

namespace navigo_behavior_tree
{

using namespace std::chrono_literals;

AdaptiveSpin::AdaptiveSpin(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  subscription_ = node_->create_subscription<localization::msg::ScanMatchingStatus>(
    "/status", rclcpp::SensorDataQoS(),
    std::bind(&AdaptiveSpin::onScanMatchingStatus, this, std::placeholders::_1));
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  permission_client_ = node_->create_client<robots_dog_msgs::srv::NavigationSelfHealing>(
    "/navigation/self_healing", rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);
}

BT::PortsList AdaptiveSpin::providedPorts()
{
  return {
    BT::InputPort<bool>("forbid_spin", true, "Fail closed unless Edge permits rotation"),
    BT::InputPort<std::string>("episode_id", std::string(""), "Active Edge self-healing episode"),
    BT::InputPort<std::string>(
      "fault_label", std::string("navigation_failed"), "Current normalized fault label"),
    BT::InputPort<double>("permission_timeout_seconds", 1.0, "Fresh Edge permission timeout"),
    BT::InputPort<double>("score_threshold", 0.4, "Maximum healthy NDT score"),
    BT::InputPort<double>("max_age_seconds", 0.75, "Maximum NDT status age")};
}

void AdaptiveSpin::resetPermission()
{
  permission_sent_ = false;
  permission_confirmed_ = false;
  episode_id_.clear();
  fault_label_.clear();
}

bool AdaptiveSpin::edgePermissionReady()
{
  if (permission_confirmed_) {
    return true;
  }
  if (!permission_sent_) {
    getInput("episode_id", episode_id_);
    getInput("fault_label", fault_label_);
    getInput("permission_timeout_seconds", permission_timeout_seconds_);
    permission_timeout_seconds_ = std::max(0.1, std::min(2.0, permission_timeout_seconds_));
    if (episode_id_.empty() || !permission_client_->service_is_ready()) {
      return false;
    }
    auto request = std::make_shared<robots_dog_msgs::srv::NavigationSelfHealing::Request>();
    request->operation = robots_dog_msgs::srv::NavigationSelfHealing::Request::STATUS;
    request->episode_id = episode_id_;
    request->fault_label = fault_label_;
    request->level = 2;
    permission_future_ = permission_client_->async_send_request(request).share();
    permission_started_ = node_->now();
    permission_sent_ = true;
    return false;
  }
  callback_group_executor_.spin_some(0ms);
  if (permission_future_.wait_for(0ms) != std::future_status::ready) {
    if ((node_->now() - permission_started_).seconds() <= permission_timeout_seconds_) {
      return false;
    }
    resetPermission();
    return false;
  }
  const auto response = permission_future_.get();
  permission_sent_ = false;
  permission_confirmed_ = response && response->accepted &&
    response->episode_id == episode_id_ && !response->recovered &&
    !response->forbid_spin && response->level == 2 &&
    response->action_type == "adaptive_spin";
  return permission_confirmed_;
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
    resetPermission();
    return BT::NodeStatus::FAILURE;
  }
  setStatus(BT::NodeStatus::RUNNING);
  if (!permission_confirmed_) {
    if (!edgePermissionReady()) {
      // A pending request keeps the decorator running without ticking Spin.
      // Inability to send, rejection, and timeout all leave no pending request
      // and therefore fail closed.
      return permission_sent_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
    }
  }
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
  resetPermission();
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::AdaptiveSpin>("AdaptiveSpin");
}
