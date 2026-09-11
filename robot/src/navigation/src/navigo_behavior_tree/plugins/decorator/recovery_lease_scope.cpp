#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/decorator/recovery_lease_scope.hpp"

namespace navigo_behavior_tree
{

using namespace std::chrono_literals;

RecoveryLeaseScope::RecoveryLeaseScope(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<robots_dog_msgs::srv::NavigationRecoveryLease>(
    "/navigation/recovery/lease");
  getInput("owner", owner_);
  getInput("reason", reason_);
  auto timeout = config().blackboard->get<std::chrono::milliseconds>("server_timeout");
  getInput("server_timeout", timeout);
  server_timeout_ = timeout;
}

BT::PortsList RecoveryLeaseScope::providedPorts()
{
  return {
    BT::InputPort<std::string>("owner", "BT_NAVIGATOR"),
    BT::InputPort<std::string>("reason", "bt_recovery"),
    BT::InputPort<std::chrono::milliseconds>("server_timeout")};
}

BT::NodeStatus RecoveryLeaseScope::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  if (!acquired_) {
    if (!acquire_sent_) {
      if (!client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Recovery lease service is unavailable");
        return BT::NodeStatus::FAILURE;
      }
      auto request = std::make_shared<robots_dog_msgs::srv::NavigationRecoveryLease::Request>();
      request->operation = robots_dog_msgs::srv::NavigationRecoveryLease::Request::ACQUIRE;
      request->owner = owner_;
      request->reason = reason_;
      acquire_future_ = client_->async_send_request(request).share();
      acquire_started_ = node_->now();
      acquire_sent_ = true;
      return BT::NodeStatus::RUNNING;
    }
    if (acquire_future_.wait_for(0ms) != std::future_status::ready) {
      if ((node_->now() - acquire_started_).nanoseconds() < server_timeout_.count() * 1000000LL) {
        return BT::NodeStatus::RUNNING;
      }
      RCLCPP_WARN(node_->get_logger(), "Recovery lease request timed out");
      acquire_sent_ = false;
      return BT::NodeStatus::FAILURE;
    }
    const auto response = acquire_future_.get();
    acquire_sent_ = false;
    if (!response->granted) {
      RCLCPP_WARN(node_->get_logger(), "Recovery lease rejected: attempts=%u exhausted=%s",
        response->attempts, response->exhausted ? "true" : "false");
      return BT::NodeStatus::FAILURE;
    }
    generation_ = response->generation;
    acquired_ = true;
  }

  const auto child_status = child_node_->executeTick();
  if (child_status != BT::NodeStatus::RUNNING) {
    requestRelease();
  }
  return child_status;
}

void RecoveryLeaseScope::requestRelease()
{
  if (!acquired_) {
    return;
  }
  auto request = std::make_shared<robots_dog_msgs::srv::NavigationRecoveryLease::Request>();
  request->operation = robots_dog_msgs::srv::NavigationRecoveryLease::Request::RELEASE;
  request->owner = owner_;
  request->generation = generation_;
  // Releasing is best-effort: retaining a lease until the supervised task
  // reset is safer than allowing another owner after a transport failure.
  client_->async_send_request(request);
  acquired_ = false;
  generation_ = 0;
}

void RecoveryLeaseScope::halt()
{
  requestRelease();
  acquire_sent_ = false;
  if (child_node_) {
    child_node_->halt();
  }
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::RecoveryLeaseScope>("RecoveryLeaseScope");
}
