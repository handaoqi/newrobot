# AGENTS.md

Project rules for RoamerX robot and cloud-platform work. Read this before making changes.

## Scope

- Canonical local development checkout: `/home/dogrobot` (preserve the repository's
  top-level layout: `robot/`, `edge-agent/`, `dev-agent/`, `platform/`, etc.).
- Deployed NX ROS workspace: `/home/robot/genisom_roamerx_open`. This remains
  the live runtime path used by current services; do not move or delete it
  while developing from `/home/dogrobot`.
- ROS: Humble
- Real robot platform: `NX_XG3588`
- Robot code: `ZSL-1A-07`
- Cloud platform: `http://39.107.250.69:8088`
- MQTT broker: `39.107.250.69:1884`
export http_proxy=http://127.0.0.1:7888
export https_proxy=http://127.0.0.1:7888
codex --dangerously-bypass-approvals-and-sandbox

## Hard Rules

- Do not use or recreate `/home/robot/genisom_roamerx_open/web_platform`; it was a stale local demo and has been removed.
- The real platform is the cloud server at `39.107.250.69`.
- Do not store passwords or secrets in repo files. SSH key login to the cloud server from this robot computer is already configured.
- Do not trigger physical motion unless explicitly requested and the path is confirmed clear.
- If the requirement says "through cloud platform", do not send local ROS navigation goals as the final test. Use the cloud UI/API -> MQTT -> edge_agent -> Nav2 path.
- Keep using the canonical real-robot scripts only:
  - `script/robot/start_navigation_real.sh`
  - `script/robot/start_mapping_real.sh`
- Avoid old/ambiguous entrypoints; they were intentionally removed:
  - `script/start_navigation.sh`
  - `script/bash/start_navigation.sh`
  - `script/bash/stop_navigation.sh`
  - `start_slam.sh`

## Navigation Stack

Canonical navigation commands:

```bash
cd /home/robot/genisom_roamerx_open
script/robot/start_navigation_real.sh start
script/robot/start_navigation_real.sh status
script/robot/start_navigation_real.sh stop
script/robot/start_navigation_real.sh restart
```

Expected healthy state:

- `/localization_info.status == 3`
- `/planner_server`: `active [3]`
- `/controller_server`: `active [3]`
- `/bt_navigator`: `active [3]`
- `/follow_waypoints` action server available
- `/cmd_vel` has the SDK bridge subscriber

Nav2/SDK control chain:

```text
Nav2 -> /cmd_vel -> mode_status_publisher -> /mode_switch_cmd=171
-> vel_cmd_udp_pub -> SDK standUp() -> SDK move()
```

When `/cmd_vel` stops for about 1 second, mode returns to `170` and `vel_cmd_udp_pub` calls SDK `passive()`.

## Localization And Map

Current local map symlinks should point to:

```text
/home/robot/.jszr/map/20260703_170635/
```

Important local files:

```text
/home/robot/.jszr/map/map.yaml
/home/robot/.jszr/map/map.pgm
/home/robot/.jszr/map/map.pcd
/home/robot/.jszr/map/map.txt
```

ROS LiDAR TF currently uses the factory calibration inverse:

```text
base_link -> livox_frame
translation: 0.382765605, -0.046855740, 0.513445457
rotation xyzw: 0.007172121, -0.043589169, -0.009509268, 0.998978536
```

Do not revert to the older `base_link -> LiDAR = [0.205, -0.02329, 0.04412]` reasoning for ROS TF.

## codex CLI / OpenAI Proxy (中转站)

When working from this machine (China network), direct OpenAI access is blocked. Use the sub2api relay:

```text
Proxy server:      49.51.35.229:8080
Local listening:   127.0.0.1:7888 (SSH RemoteForward from Windows host)
Windows VPN port:  localhost:7899
```

SSH tunnel setup (on Windows host `~/.ssh/config`):

```text
RemoteForward 7888 localhost:7899
```

Environment variables for CLI tools:

```bash
export http_proxy=http://127.0.0.1:7888
export https_proxy=http://127.0.0.1:7888
```

codex CLI config (`~/.codex/config.toml`):

```toml
model_provider = "sub2api"
model = "gpt-5.5"
model_reasoning_effort = "high"
disable_response_storage = true
[model_providers]
[model_providers.sub2api]
name = "sub2api"
base_url = "http://49.51.35.229:8080"
wire_api = "responses"
requires_openai_auth = false
```

Login (requires proxy env first):

```bash
codex login --device-auth
# or with bypass flag
codex --dangerously-bypass-approvals-and-sandbox
```

Note: If the proxy returns 503, the relay quota is likely exhausted — contact the relay provider to recharge.

## Cloud Platform

Cloud runtime:

```text
/opt/roamerx/current -> /opt/roamerx/releases/20260703_1140_task_calendar_mvp
/opt/roamerx/shared/center.env
/opt/roamerx/shared/db.sqlite3
/opt/roamerx/shared/media
/opt/roamerx/shared/logs
```

Cloud services:

```text
roamerx-center-api.service
roamerx-device-worker.service
roamerx-patrol-scheduler.service
nginx.service
mosquitto
```

Local copy of current platform code:

```text
/home/robot/yw/roamerx_analysis/center_platform/platform_server_code
```

Important platform files:

```text
backend/monitoring/urls.py
backend/monitoring/views.py
backend/monitoring/mqtt_client.py
backend/monitoring/message_handlers.py
backend/monitoring/protocol.py
backend/monitoring/services/command_service.py
backend/monitoring/services/task_service.py
frontend/src/views/MapsPage.vue
frontend/src/App.vue
```

Important frontend routes:

```text
/dashboard/tasks/maps
/dashboard/tasks/routes
/dashboard/tasks/tracks
```

## Cloud-To-Dog Command Path

Use this as source of truth:

```text
Browser/Django API -> RemoteCommand -> roamerx-device-worker
-> MQTT robots/ZSL-1A-07/commands
-> dog edge_agent
-> /follow_waypoints
-> Nav2
-> /cmd_vel
-> UDP SDK bridge
```

Dog-side edge agent:

```text
systemd service: roamerx-edge-agent.service
working dir: /home/robot/edge_agent
runtime config: /home/robot/edge_agent/config.yaml
repo source: /home/robot/genisom_roamerx_open/edge_agent
```

MQTT topics:

```text
cloud -> dog: robots/ZSL-1A-07/commands
dog -> cloud: robots/ZSL-1A-07/commands/<command_id>/ack
dog -> cloud: robots/ZSL-1A-07/commands/<command_id>/result
dog -> cloud: robots/ZSL-1A-07/presence
dog -> cloud: robots/ZSL-1A-07/telemetry/status
dog -> cloud: robots/ZSL-1A-07/events/task
dog -> cloud: robots/ZSL-1A-07/events/alert
dog -> cloud: robots/ZSL-1A-07/sync/state
```

## Safety Stop

Emergency local stop command:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/genisom_roamerx_open/install/setup.bash
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

## Python Tests

The user Python environment provides `pytest>=8,<9` so it is compatible with
the installed AnyIO pytest plugin. Run Python tests normally; do not disable
plugin autoload:

```bash
cd /home/robot/genisom_roamerx_open/dev_agent
python3 -m pytest -q

cd /home/robot/genisom_roamerx_open/edge_agent
python3 -m pytest -q
```
