#ifndef NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__RECOVERY_LEASE_SCOPE_HPP_
#define NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__RECOVERY_LEASE_SCOPE_HPP_

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robots_dog_msgs/srv/navigation_recovery_lease.hpp"
#include "robots_dog_msgs/srv/navigation_self_healing.hpp"

namespace navigo_behavior_tree
{

/** Guard an otherwise motion-producing BT recovery node with an Edge lease. */
class RecoveryLeaseScope : public BT::DecoratorNode
{
public:
  RecoveryLeaseScope(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  bool ensureActionReported();
  void requestRelease(bool success = false, const std::string & detail = "halted");

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<robots_dog_msgs::srv::NavigationRecoveryLease>::SharedPtr client_;
  rclcpp::Client<robots_dog_msgs::srv::NavigationSelfHealing>::SharedPtr self_heal_client_;
  std::shared_future<robots_dog_msgs::srv::NavigationRecoveryLease::Response::SharedPtr> acquire_future_;
  std::shared_future<robots_dog_msgs::srv::NavigationSelfHealing::Response::SharedPtr>
    action_start_future_;
  std::string owner_;
  std::string reason_;
  std::string episode_id_;
  std::string action_id_;
  std::string action_type_;
  unsigned level_{0};
  uint64_t generation_{0};
  bool acquire_sent_{false};
  bool acquired_{false};
  bool action_start_sent_{false};
  bool action_reported_{false};
  bool action_reporting_unavailable_{false};
  rclcpp::Time acquire_started_;
  rclcpp::Time action_start_time_;
  std::chrono::milliseconds server_timeout_{1000};
};

}  // namespace navigo_behavior_tree

#endif  // NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__RECOVERY_LEASE_SCOPE_HPP_
