#!/bin/bash

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/robot_navigation/install/setup.bash

# Set ROS domain ID
export ROS_DOMAIN_ID=0

# Run navigation node
echo  Starting navigation node...
ros2 run robot_navigation_core navigation_node

