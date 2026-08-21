#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
SCRIPT_DIR="${SCRIPT_DIR:-${PROJECT_DIR}/script/robot}"
MAP_PCD="${MAP_PCD:-/home/dogrobot/runtime/nx-edge/data/jszr/map/map.pcd}"
LOG_DIR="/tmp/roamerx_official_ukf"
mkdir -p "${LOG_DIR}"
set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

stop_legacy_localization() {
  pkill -f '[r]os2 launch localization localization.launch.py' || true
  pkill -f '[l]ocalization_node' || true
  pkill -f '[s]tatic_transform_publisher.*base_link livox_frame' || true
  sleep 2
}

case "${1:-status}" in
  start)
    # Nav2 is deliberately not started here. This only changes the TF owner.
    stop_legacy_localization
    setsid ros2 run localization localization_node --ros-args -r __node:=localization \
      --params-file "${PROJECT_DIR}/install/localization/share/localization/config/config.yaml" \
      -p send_tf_transforms:=false >"${LOG_DIR}/ndt_measurement.log" 2>&1 < /dev/null &
    setsid ros2 run tf2_ros static_transform_publisher \
      0.382765605 -0.046855740 0.513445457 \
      0.007172121 -0.043589169 -0.009509268 0.998978536 \
      base_link livox_frame >"${LOG_DIR}/lidar_tf.log" 2>&1 < /dev/null &
    sleep 2
    timeout 25 ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap \
      "{pcd_path: '${MAP_PCD}'}" >"${LOG_DIR}/load_map.log"
    "${SCRIPT_DIR}/start_official_ukf_shadow.sh" stop
    PUBLISH_TF=true "${SCRIPT_DIR}/start_official_ukf_shadow.sh" start
    ;;
  rollback)
    "${SCRIPT_DIR}/start_official_ukf_shadow.sh" stop
    stop_legacy_localization
    setsid ros2 launch localization localization.launch.py >"${LOG_DIR}/legacy_localization.log" 2>&1 < /dev/null &
    sleep 2
    timeout 25 ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap \
      "{pcd_path: '${MAP_PCD}'}" >"${LOG_DIR}/load_map.log"
    ;;
  status)
    ros2 run tf2_ros tf2_echo map odom 2>/dev/null | head -30 || true
    ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -30 || true
    ;;
  *)
    echo "Usage: $0 {start|rollback|status}" >&2
    exit 2
    ;;
esac
