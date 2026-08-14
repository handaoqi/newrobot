#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/robot/genisom_roamerx_open}"
# Keep this aligned with the LiDAR/IMU cold-start readiness window.
WAIT_SECONDS="${WAIT_SECONDS:-90}"

"${PROJECT_DIR}/script/robot/ensure_mapping_sensors.sh"

set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if ! pgrep -f 'pointcloud_to_laserscan_node.*laser_scan_raw' >/dev/null 2>&1; then
  setsid ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r cloud_in:=/front_lidar -r scan:=/laser_scan_raw \
    -p target_frame:=base_link -p transform_tolerance:=0.35 \
    -p min_height:=0.05 -p max_height:=1.60 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0087 \
    -p scan_time:=0.1 -p range_min:=0.18 -p range_max:=4.0 \
    -p use_inf:=true -p inf_epsilon:=1.0 \
    >/tmp/pointcloud_to_laserscan.log 2>&1 < /dev/null &
fi

if ! pgrep -f 'self_filter_scan.py' >/dev/null 2>&1; then
  setsid ros2 run robot_navigo self_filter_scan.py --ros-args \
    -r scan_in:=/laser_scan_raw -r scan_out:=/laser_scan \
    -p self_x_min:=0.10 -p self_x_max:=0.46 \
    -p self_y_min:=-0.19 -p self_y_max:=0.10 \
    >/tmp/self_filter_scan.log 2>&1 < /dev/null &
fi

if ! pgrep -f '/sensor_health_monitor($| )' >/dev/null 2>&1; then
  setsid ros2 run robot_navigo sensor_health_monitor \
    >/tmp/sensor_health_monitor.log 2>&1 < /dev/null &
fi

scan_ready=false
scan_deadline=$((SECONDS + WAIT_SECONDS))
while (( SECONDS < scan_deadline )); do
  # Give a newly-created Zenoh subscriber enough time to discover the scan
  # publisher before deciding the conversion chain is unavailable.
  if timeout 6 ros2 topic echo /laser_scan --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
    scan_ready=true
    break
  fi
  sleep 0.5
done
if [ "${scan_ready}" != "true" ]; then
  echo "ERROR: navigation laser scan has no data on /laser_scan" >&2
  tail -n 30 /tmp/pointcloud_to_laserscan.log >&2 2>/dev/null || true
  tail -n 30 /tmp/self_filter_scan.log >&2 2>/dev/null || true
  exit 1
fi

echo "Navigation sensors OK: /front_lidar, /front_lidar/imu, /laser_scan"
