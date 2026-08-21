#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="$ROOT/conf/platform.env"
[[ -f "$ENV_FILE" ]] || { echo "Run $ROOT/bin/platformctl init first" >&2; exit 1; }

set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a
: "${MQTT_USERNAME:?Set MQTT_USERNAME in platform.env}"
: "${MQTT_PASSWORD:?Set MQTT_PASSWORD in platform.env}"

docker run --rm -v "$ROOT/conf:/work" eclipse-mosquitto:2 \
  mosquitto_passwd -b -c /work/mosquitto.passwd "$MQTT_USERNAME" "$MQTT_PASSWORD"
chmod 600 "$ROOT/conf/mosquitto.passwd"

