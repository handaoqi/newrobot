#!/bin/bash
# Desktop helper only. On the NX robot use start_mapping_real.sh.
source /opt/ros/humble/setup.bash
source /home/dogrobot/robot/install/setup.bash
ros2 service call /slam/start_mapping std_srvs/srv/Trigger
