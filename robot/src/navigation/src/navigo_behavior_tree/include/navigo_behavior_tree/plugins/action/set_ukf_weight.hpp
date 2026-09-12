#ifndef NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__SET_UKF_WEIGHT_HPP_
#define NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__SET_UKF_WEIGHT_HPP_

#include <memory>
#include <string>

#include "navigo_behavior_tree/bt_service_node.hpp"
#include "robots_dog_msgs/srv/set_localization_fusion_profile.hpp"

namespace navigo_behavior_tree
{

class SetUkfWeight : public BtServiceNode<robots_dog_msgs::srv::SetLocalizationFusionProfile>
{
public:
  SetUkfWeight(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  void on_tick() override;
  BT::NodeStatus on_completion(
    std::shared_ptr<robots_dog_msgs::srv::SetLocalizationFusionProfile::Response> response) override;
};

}  // namespace navigo_behavior_tree

#endif  // NAVIGO_BEHAVIOR_TREE__PLUGINS__ACTION__SET_UKF_WEIGHT_HPP_
