#!/bin/bash
# 启动机器人基础服务（雷达 + zenoh + 点云转激光）
# 用法: bash /home/dogrobot/robot/script/start_base.sh

set -e
PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "=== 1/2 启动 zenoh 路由器 ==="
source /opt/ros/humble/setup.bash
if ! pgrep -f rmw_zenohd > /dev/null 2>&1; then
    ros2 run rmw_zenoh_cpp rmw_zenohd &>/tmp/zenoh.log &
    sleep 1
    echo "zenoh 路由器已启动"
else
    echo "zenoh 已在运行"
fi

echo "=== 2/2 确保雷达、静态 TF 和导航激光链路 ==="
WAIT_SECONDS="${WAIT_SECONDS:-90}" "${SCRIPT_DIR}/ensure_navigation_sensors.sh"

echo ""
echo "=== 基础服务已全部启动 ==="
echo " radar: /front_lidar"
echo " scan:  /laser_scan"
echo ""
