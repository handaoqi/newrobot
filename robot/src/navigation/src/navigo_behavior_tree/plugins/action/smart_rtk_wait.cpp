#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/action/smart_rtk_wait.hpp"

namespace navigo_behavior_tree
{

using namespace std::chrono_literals;

SmartRTKWait::SmartRTKWait(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  client_ = node_->create_client<robots_dog_msgs::srv::NavigationSelfHealing>(
    "/navigation/self_healing", rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);
}

BT::PortsList SmartRTKWait::providedPorts()
{
  return {
    BT::InputPort<bool>("enabled", true, "Whether smart RTK wait is selected"),
    BT::InputPort<std::string>(
      "episode_id", std::string(""), "Self-healing episode"),
    BT::InputPort<std::string>(
      "fault_label", std::string("rtk_transient_loss"), "Normalized fault label"),
    BT::InputPort<double>("wait_seconds", 5.0, "Maximum wait time"),
    BT::InputPort<double>("poll_seconds", 0.1, "Health polling interval")};
}

BT::NodeStatus SmartRTKWait::onStart()
{
  bool enabled = true;
  getInput("enabled", enabled);
  if (!enabled || !client_->service_is_ready()) {
    return BT::NodeStatus::FAILURE;
  }
  double wait_seconds = 5.0;
  getInput("wait_seconds", wait_seconds);
  getInput("poll_seconds", poll_seconds_);
  getInput("episode_id", episode_id_);
  getInput("fault_label", fault_label_);
  wait_seconds = std::max(0.0, wait_seconds);
  poll_seconds_ = std::clamp(poll_seconds_, 0.05, 1.0);
  deadline_ = node_->now() + rclcpp::Duration::from_seconds(wait_seconds);
  next_poll_ = node_->now();
  request_sent_ = false;
  return BT::NodeStatus::RUNNING;
}

void SmartRTKWait::sendStatusRequest()
{
  auto request = std::make_shared<robots_dog_msgs::srv::NavigationSelfHealing::Request>();
  request->operation = robots_dog_msgs::srv::NavigationSelfHealing::Request::STATUS;
  request->episode_id = episode_id_;
  request->fault_label = fault_label_;
  request->level = 0;
  future_ = client_->async_send_request(request).share();
  request_sent_ = true;
}

BT::NodeStatus SmartRTKWait::onRunning()
{
  const auto now = node_->now();
  if (now >= deadline_) {
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }
  if (!request_sent_ && now >= next_poll_) {
    if (!client_->service_is_ready()) {
      return BT::NodeStatus::FAILURE;
    }
    sendStatusRequest();
  }
  if (!request_sent_) {
    return BT::NodeStatus::RUNNING;
  }
  callback_group_executor_.spin_some(0ms);
  if (future_.wait_for(0ms) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }
  const auto response = future_.get();
  request_sent_ = false;
  next_poll_ = now + rclcpp::Duration::from_seconds(poll_seconds_);
  if (!response || !response->accepted) {
    return BT::NodeStatus::FAILURE;
  }
  episode_id_ = response->episode_id;
  return response->recovered ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void SmartRTKWait::onHalted()
{
  request_sent_ = false;
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::SmartRTKWait>("SmartRTKWait");
}
