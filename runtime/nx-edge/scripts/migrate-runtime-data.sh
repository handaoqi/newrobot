#!/usr/bin/env bash
set -euo pipefail

RUNTIME_ROOT="/home/dogrobot/runtime/nx-edge"
DATA_ROOT="$RUNTIME_ROOT/data"
CONF_ROOT="$RUNTIME_ROOT/conf"
INSTALL_ROOT="$RUNTIME_ROOT/install"
LEGACY_HOME="${ROAMERX_LEGACY_HOME:-$(getent passwd robot | awk -F: 'NR == 1 { print $6 }')}"
SERVICES=(roamerx-edge-agent roamerx-bike-bot roamerx-local-asr)
ACTIVE_SERVICES=()

if [[ -z "$LEGACY_HOME" || ! -d "$LEGACY_HOME" ]]; then
  echo "Legacy home does not exist; set ROAMERX_LEGACY_HOME explicitly: ${LEGACY_HOME:-<empty>}" >&2
  exit 1
fi

if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

for service in "${SERVICES[@]}"; do
  if systemctl is-active --quiet "$service"; then
    ACTIVE_SERVICES+=("$service")
  fi
done

restore_services() {
  if ((${#ACTIVE_SERVICES[@]})); then
    systemctl start "${ACTIVE_SERVICES[@]}"
  fi
}
trap restore_services EXIT
if ((${#ACTIVE_SERVICES[@]})); then
  systemctl stop "${ACTIVE_SERVICES[@]}"
fi

install -d -o dogrobot -g robot -m 2775 \
  "$DATA_ROOT" "$DATA_ROOT/cache" "$DATA_ROOT/logs" "$CONF_ROOT" "$INSTALL_ROOT"

move_and_link() {
  local source="$1" destination="$2"
  install -d -o dogrobot -g robot -m 2775 "$(dirname "$destination")" "$(dirname "$source")"
  if [[ -L "$source" ]]; then
    [[ "$(readlink -f "$source")" == "$(readlink -f "$destination")" ]] || {
      echo "Unexpected compatibility link: $source -> $(readlink "$source")" >&2
      return 1
    }
    return
  fi
  if [[ -e "$source" ]]; then
    if [[ -d "$destination" ]] && ! find "$destination" -mindepth 1 \( -type f -o -type l \) -print -quit | grep -q .; then
      find "$destination" -depth -type d -empty -delete
    fi
    [[ ! -e "$destination" ]] || { echo "Both source and destination exist: $source, $destination" >&2; return 1; }
    mv "$source" "$destination"
  fi
  [[ -e "$destination" ]] || { echo "Missing migration source and destination: $source" >&2; return 1; }
  ln -s "$destination" "$source"
}

move_and_link "$LEGACY_HOME/.jszr" "$DATA_ROOT/jszr"
move_and_link "$LEGACY_HOME/rosbags" "$DATA_ROOT/rosbags"
move_and_link "$LEGACY_HOME/.local/share/roamerx-voice" "$DATA_ROOT/voice"
move_and_link "$LEGACY_HOME/.cache/roamerx" "$DATA_ROOT/cache/roamerx"
move_and_link "$LEGACY_HOME/.robot" "$DATA_ROOT/robot-state"
move_and_link "$LEGACY_HOME/.ros" "$DATA_ROOT/ros-home"
move_and_link "$LEGACY_HOME/data/patrol_data" "$DATA_ROOT/patrol-data"
move_and_link "$LEGACY_HOME/.robot_launch_logs" "$DATA_ROOT/logs/robot-launch"
move_and_link "$LEGACY_HOME/genisom_l1_sdk_old" "$INSTALL_ROOT/genisom_l1_sdk"

if [[ -d "$LEGACY_HOME/edge_agent" && ! -L "$LEGACY_HOME/edge_agent/config.yaml" ]]; then
  LEGACY="$DATA_ROOT/legacy-edge-agent"
  [[ ! -e "$LEGACY" ]] || { echo "$LEGACY already exists" >&2; exit 1; }
  mv "$LEGACY_HOME/edge_agent" "$LEGACY"
  install -d -o robot -g robot -m 2775 "$LEGACY_HOME/edge_agent"
  if [[ -d "$LEGACY/data" ]]; then
    mv "$LEGACY/data" "$DATA_ROOT/edge-agent"
  fi
  for pair in \
    "config.yaml:edge-agent.yaml" \
    "rtk_ntrip.yaml:rtk-ntrip.yaml" \
    "rtk_ntrip.secret.yaml:rtk-ntrip.secret.yaml" \
    "sixents_no_sdk.ini:sixents-no-sdk.ini"; do
    source_name="${pair%%:*}"
    target_name="${pair##*:}"
    [[ -f "$LEGACY/$source_name" ]] && mv "$LEGACY/$source_name" "$CONF_ROOT/$target_name"
  done
fi

install -d -o robot -g robot -m 2775 "$LEGACY_HOME/edge_agent" "$DATA_ROOT/edge-agent"
ln -sfn "$DATA_ROOT/edge-agent" "$LEGACY_HOME/edge_agent/data"
ln -sfn "$CONF_ROOT/edge-agent.yaml" "$LEGACY_HOME/edge_agent/config.yaml"
ln -sfn "$CONF_ROOT/rtk-ntrip.yaml" "$LEGACY_HOME/edge_agent/rtk_ntrip.yaml"
ln -sfn "$CONF_ROOT/rtk-ntrip.secret.yaml" "$LEGACY_HOME/edge_agent/rtk_ntrip.secret.yaml"
ln -sfn "$CONF_ROOT/sixents-no-sdk.ini" "$LEGACY_HOME/edge_agent/sixents_no_sdk.ini"

install -d -o robot -g robot -m 2775 "$LEGACY_HOME/.config/roamerx"
if [[ -f "$LEGACY_HOME/.config/roamerx/bike-bot.yaml" && ! -L "$LEGACY_HOME/.config/roamerx/bike-bot.yaml" ]]; then
  mv "$LEGACY_HOME/.config/roamerx/bike-bot.yaml" "$CONF_ROOT/bike-bot.yaml"
fi
ln -sfn "$CONF_ROOT/bike-bot.yaml" "$LEGACY_HOME/.config/roamerx/bike-bot.yaml"

chown -R robot:robot "$DATA_ROOT"
chown -R dogrobot:robot "$CONF_ROOT"
chmod 750 "$CONF_ROOT"
find "$CONF_ROOT" -type f -exec chmod 640 {} +
find "$CONF_ROOT" -type f \( -name README.md -o -name '*.example' -o -name '*.example.*' \) -exec chmod 644 {} +
echo "NX runtime data migrated to $RUNTIME_ROOT"
