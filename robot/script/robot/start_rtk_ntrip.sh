#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

set +u
source /opt/ros/humble/setup.bash
PROJECT_DIR="${PROJECT_DIR:-/home/dogrobot/robot}"
source "${PROJECT_DIR}/install/setup.bash"
source /opt/robot-driver/install/setup.bash
set -u

CONFIG_PATH="${RTK_NTRIP_CONFIG:-/home/dogrobot/runtime/nx-edge/conf/rtk-ntrip.yaml}"
export RTK_CONFIG_PATH="${RTK_CONFIG_PATH:-/home/dogrobot/runtime/nx-edge/conf/sixents-no-sdk.ini}"
LOG_DIR="${RTK_NTRIP_LOG_DIR:-/home/dogrobot/runtime/nx-edge/data/logs/robot-launch}"
mkdir -p "$LOG_DIR"

kill_pattern() {
  local pattern="$1"
  local pid
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "${PPID:-}" ]] && continue
    kill "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" || true)
}

kill_pattern "sixents_gps_driver"
kill_pattern "rtk_ntrip_bridge.py"
kill_pattern "static_transform_publisher.*base_link.*gps_link"
sleep 1

setsid ros2 run tf2_ros static_transform_publisher \
  -0.05 0.0 0.15 0.0 0.0 0.0 base_link gps_link \
  >"$LOG_DIR/rtk-gps-link-tf.stdout" 2>"$LOG_DIR/rtk-gps-link-tf.stderr" </dev/null &
echo $! > "$LOG_DIR/rtk-gps-link-tf.pid"

setsid ros2 launch sixents_gps_driver sixents_gps_driver.launch.py \
  >"$LOG_DIR/rtk-sixents.stdout" 2>"$LOG_DIR/rtk-sixents.stderr" </dev/null &
echo $! > "$LOG_DIR/rtk-sixents.pid"

sleep 2

setsid python3 "${PROJECT_DIR}/script/rtk_ntrip_bridge.py" \
  --ros-args -p "config_path:=${CONFIG_PATH}" \
  >"$LOG_DIR/rtk-ntrip.stdout" 2>"$LOG_DIR/rtk-ntrip.stderr" </dev/null &
echo $! > "$LOG_DIR/rtk-ntrip.pid"

echo "RTK started. Logs: $LOG_DIR/rtk-sixents.stdout, $LOG_DIR/rtk-ntrip.stdout"
