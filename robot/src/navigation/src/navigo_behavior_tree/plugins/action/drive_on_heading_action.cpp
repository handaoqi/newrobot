// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <string>
#include <memory>

#include "navigo_behavior_tree/plugins/action/drive_on_heading_action.hpp"

namespace navigo_behavior_tree
{

DriveOnHeadingAction::DriveOnHeadingAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::DriveOnHeading>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("dist_to_travel", dist);
  double lateral_dist;
  getInput("lateral_dist", lateral_dist);
  double speed;
  getInput("speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.target.x = std::abs(lateral_dist) > 1e-6 ? 0.0 : dist;
  goal_.target.y = lateral_dist;
  goal_.target.z = 0.0;
  const double direction = std::abs(lateral_dist) > 1e-6 ? lateral_dist : dist;
  goal_.speed = std::copysign(std::abs(speed), direction);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace navigo_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<navigo_behavior_tree::DriveOnHeadingAction>(
        name, "drive_on_heading", config);
    };

  factory.registerBuilder<navigo_behavior_tree::DriveOnHeadingAction>("DriveOnHeading", builder);
}
