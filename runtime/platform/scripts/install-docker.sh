#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

apt-get update
apt-get install -y ca-certificates curl docker.io docker-compose-v2
systemctl enable --now docker
usermod -aG docker "${SUDO_USER:-root}"
echo "Docker installed. Re-login once so docker group membership takes effect."

