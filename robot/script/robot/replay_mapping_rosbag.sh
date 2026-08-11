#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 BAG_DIR OUTPUT_ROOT [RATE] [BAG_SECONDS] [ACC_COV] [GYR_COV] [SAVE]" >&2
  exit 2
fi

BAG_DIR=$(realpath "$1")
OUTPUT_ROOT=$2
RATE=${3:-1.0}
BAG_SECONDS=${4:-0}
ACC_COV=${5:-1.0}
GYR_COV=${6:-0.2}
SAVE_RESULT=${7:-0}
ROS_DOMAIN_ID=${REPLAY_ROS_DOMAIN_ID:-77}
WORKSPACE=${ROAMERX_WORKSPACE:-/home/robot/genisom_roamerx_open}
PARAMS_FILE="$WORKSPACE/install/robot_slam/share/robot_slam/config/config.yaml"
MAPPING_BIN="$WORKSPACE/install/robot_slam/lib/robot_slam/mapping"

if [[ ! -d "$BAG_DIR" ]]; then
  echo "Bag directory does not exist: $BAG_DIR" >&2
  exit 2
fi
if [[ -e "$OUTPUT_ROOT" ]] && find "$OUTPUT_ROOT" -mindepth 1 -print -quit | grep -q .; then
  echo "Output directory must be empty: $OUTPUT_ROOT" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"
OUTPUT_ROOT=$(realpath "$OUTPUT_ROOT")
LOG_FILE="$OUTPUT_ROOT/replay_mapping.log"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=${REPLAY_RMW_IMPLEMENTATION:-rmw_zenoh_cpp}

MAPPING_PID=""
cleanup() {
  if [[ -n "$MAPPING_PID" ]] && kill -0 "$MAPPING_PID" 2>/dev/null; then
    kill -INT "$MAPPING_PID" 2>/dev/null || true
    wait "$MAPPING_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting isolated mapping: domain=$ROS_DOMAIN_ID rate=$RATE acc_cov=$ACC_COV gyr_cov=$GYR_COV"
stdbuf -oL -eL "$MAPPING_BIN" --ros-args \
  --params-file "$PARAMS_FILE" \
  -p storage.data_path:="$OUTPUT_ROOT" \
  -p mapping.acc_cov:="$ACC_COV" \
  -p mapping.gyr_cov:="$GYR_COV" \
  -p odom_guard.enable:=true \
  >"$LOG_FILE" 2>&1 &
MAPPING_PID=$!

for _ in $(seq 1 100); do
  if ros2 service type /slam_state_service 2>/dev/null | grep -q robots_dog_msgs; then
    break
  fi
  sleep 0.1
done
if ! ros2 service type /slam_state_service 2>/dev/null | grep -q robots_dog_msgs; then
  echo "SLAM service did not become ready; see $LOG_FILE" >&2
  exit 1
fi

ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{data: 3}' >/dev/null

PLAY_STATUS=0
if [[ "$BAG_SECONDS" != "0" ]]; then
  WALL_SECONDS=$(awk -v duration="$BAG_SECONDS" -v rate="$RATE" 'BEGIN { printf "%d", duration / rate + 8 }')
  timeout --signal=INT --kill-after=10 "${WALL_SECONDS}s" \
    ros2 bag play "$BAG_DIR" --rate "$RATE" --disable-keyboard-controls \
      --topics /front_lidar /front_lidar/imu /odom/mc_odom || PLAY_STATUS=$?
  if [[ "$PLAY_STATUS" -ne 0 && "$PLAY_STATUS" -ne 124 ]]; then
    echo "Bag playback failed with status $PLAY_STATUS" >&2
    exit "$PLAY_STATUS"
  fi
else
  ros2 bag play "$BAG_DIR" --rate "$RATE" --disable-keyboard-controls \
    --topics /front_lidar /front_lidar/imu /odom/mc_odom
fi

sleep 5
SESSION_DIR=$(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)
if [[ -z "$SESSION_DIR" ]]; then
  echo "Replay produced no mapping session; see $LOG_FILE" >&2
  exit 1
fi

if [[ "$SAVE_RESULT" == "1" ]]; then
  ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{data: 5}' >/dev/null
  for _ in $(seq 1 7200); do
    STAGE=$(sed -n 's/.*"stage": "\([^"]*\)".*/\1/p' "$SESSION_DIR/save_progress.json" 2>/dev/null | head -1)
    if [[ "$STAGE" == "completed" || "$STAGE" == "failed" ]]; then
      break
    fi
    sleep 1
  done
fi

cleanup
trap - EXIT INT TERM
echo "SESSION_DIR=$SESSION_DIR"
cat "$SESSION_DIR/save_progress.json"
