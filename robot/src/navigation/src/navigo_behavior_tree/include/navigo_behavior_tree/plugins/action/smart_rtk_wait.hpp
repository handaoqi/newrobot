#ifndef NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__SMART_RTK_WAIT_HPP_
#define NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__SMART_RTK_WAIT_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robots_dog_msgs/srv/navigation_self_healing.hpp"

namespace navigo_behavior_tree
{

class SmartRTKWait : public BT::StatefulActionNode
{
public:
  SmartRTKWait(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void sendStatusRequest();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<robots_dog_msgs::srv::NavigationSelfHealing>::SharedPtr client_;
  std::shared_future<robots_dog_msgs::srv::NavigationSelfHealing::Response::SharedPtr> future_;
  rclcpp::Time deadline_;
  rclcpp::Time next_poll_;
  std::string episode_id_;
  std::string fault_label_;
  double poll_seconds_{0.1};
  bool request_sent_{false};
};

}  // namespace navigo_behavior_tree

#endif  // NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__SMART_RTK_WAIT_HPP_
