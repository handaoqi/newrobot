#!/usr/bin/env bash
set -euo pipefail

CLOUD_HOST="${DOG_REMOTE_SERVER_HOST:?set DOG_REMOTE_SERVER_HOST first}"
CLOUD_PORT="${DOG_REMOTE_SERVER_PORT:-22}"
CLOUD_USER="${DOG_REMOTE_SERVER_USER:-mluser}"
REMOTE_ORIN_PORT="${DOG_REMOTE_ORIN_PORT:-60022}"
REMOTE_RK_PORT="${DOG_REMOTE_RK_PORT:-60021}"
RK_HOST="${DOG_RK_HOST:-192.168.234.1}"

mkdir -p /home/jszr/.ssh /home/jszr/.local/bin /home/jszr/.local/run
chmod 700 /home/jszr/.ssh

if [ ! -f /home/jszr/.ssh/id_ed25519 ]; then
  ssh-keygen -t ed25519 -N '' -f /home/jszr/.ssh/id_ed25519 >/dev/null
fi

cat > /home/jszr/.local/bin/run-reverse-tunnel.sh <<SH
#!/usr/bin/env bash
set -euo pipefail

while true; do
  ssh \
    -i /home/jszr/.ssh/id_ed25519 \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/home/jszr/.ssh/known_hosts.remote \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 \
    -N \
    -p "${CLOUD_PORT}" \
    -R "${REMOTE_ORIN_PORT}:127.0.0.1:22" \
    -R "${REMOTE_RK_PORT}:${RK_HOST}:22" \
    "${CLOUD_USER}@${CLOUD_HOST}" >>/home/jszr/.local/run/revssh.log 2>&1 || true
  sleep 5
done
SH

cat > /home/jszr/.local/bin/start-reverse-tunnel.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail

mkdir -p /home/jszr/.local/run
if [ -f /home/jszr/.local/run/revssh.pid ] && kill -0 "$(cat /home/jszr/.local/run/revssh.pid)" 2>/dev/null; then
  echo "revssh already running pid=$(cat /home/jszr/.local/run/revssh.pid)"
  exit 0
fi

nohup bash /home/jszr/.local/bin/run-reverse-tunnel.sh >/home/jszr/.local/run/revssh.stdout 2>&1 &
echo $! >/home/jszr/.local/run/revssh.pid
sleep 3
kill -0 "$(cat /home/jszr/.local/run/revssh.pid)"
echo "revssh started pid=$(cat /home/jszr/.local/run/revssh.pid)"
SH

chmod 700 /home/jszr/.local/bin/run-reverse-tunnel.sh /home/jszr/.local/bin/start-reverse-tunnel.sh

if command -v systemctl >/dev/null 2>&1; then
  mkdir -p /home/jszr/.config/systemd/user
  cat > /home/jszr/.config/systemd/user/dog-reverse-tunnel.service <<'SERVICE'
[Unit]
Description=Dog reverse SSH tunnel to cloud server
After=network-online.target

[Service]
Type=simple
ExecStart=/home/jszr/.local/bin/run-reverse-tunnel.sh
Restart=always
RestartSec=5

[Install]
WantedBy=default.target
SERVICE

  systemctl --user daemon-reload || true
  systemctl --user enable dog-reverse-tunnel.service || true
  systemctl --user restart dog-reverse-tunnel.service || bash /home/jszr/.local/bin/start-reverse-tunnel.sh
else
  bash /home/jszr/.local/bin/start-reverse-tunnel.sh
fi

echo "Orin public key:"
cat /home/jszr/.ssh/id_ed25519.pub
