#ifndef NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__FAULT_DIAGNOSE_NODE_HPP_
#define NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__FAULT_DIAGNOSE_NODE_HPP_

#include <memory>
#include <string>

#include "navigo_behavior_tree/bt_service_node.hpp"
#include "robots_dog_msgs/srv/navigation_self_healing.hpp"

namespace navigo_behavior_tree
{

class FaultDiagnoseNode : public BtServiceNode<robots_dog_msgs::srv::NavigationSelfHealing>
{
public:
  FaultDiagnoseNode(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<robots_dog_msgs::srv::NavigationSelfHealing::Response> response) override;
};

}  // namespace navigo_behavior_tree

#endif  // NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__FAULT_DIAGNOSE_NODE_HPP_
