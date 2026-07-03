#!/bin/bash
# 启动机器人基础服务（雷达 + zenoh + 点云转激光）
# 用法: bash /home/robot/genisom_roamerx_open/script/start_base.sh

set -e
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "=== 1/3 启动 zenoh 路由器 ==="
source /opt/ros/humble/setup.bash
if ! pgrep -f rmw_zenohd > /dev/null 2>&1; then
    ros2 run rmw_zenoh_cpp rmw_zenohd &>/tmp/zenoh.log &
    sleep 1
    echo "zenoh 路由器已启动"
else
    echo "zenoh 已在运行"
fi

echo "=== 2/3 启动雷达驱动 ==="
source /opt/robot-driver/install/setup.bash
if ! pgrep -f livox_driver_node > /dev/null 2>&1; then
    ros2 launch livox_driver lidar.launch.py &>/tmp/livox.log &
    sleep 3
    echo "雷达驱动已启动"
else
    echo "雷达驱动已在运行"
fi

echo "=== 3/3 启动点云转激光（导航避障用） ==="
sleep 2  # 等雷达稳定
if ! pgrep -f pointcloud_to_laserscan > /dev/null 2>&1; then
    ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
        --ros-args \
        -r cloud_in:=/front_lidar \
        -r scan:=/laser_scan \
        -p target_frame:=base_link \
        -p min_height:=0.1 \
        -p max_height:=0.5 &>/tmp/pcl2laser.log &
    sleep 1
    echo "点云转激光已启动"
else
    echo "点云转激光已在运行"
fi

echo ""
echo "=== 基础服务已全部启动 ==="
echo " radar: /front_lidar"
echo " scan:  /laser_scan"
echo ""
