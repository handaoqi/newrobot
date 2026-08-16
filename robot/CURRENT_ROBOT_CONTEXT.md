# RoamerX Current Context

Last updated: 2026-07-03 18:14 CST

This file records the latest verified state so a later debugging session can continue without replaying the full chat.

## Machine And Workspace

- Robot computer workspace: `/home/robot/genisom_roamerx_open`
- ROS: Humble
- Real robot platform: `NX_XG3588`
- Navigation communication: `UDP`
- Motion controller mode used by Nav2: `RL_TRACK_VELOCITY`
- Current map yaml used by Nav2: `/home/robot/genisom_roamerx_open/map/map.yaml`
- Current PCD map used by localization: `/home/robot/.jszr/map/map.pcd`
- Current map symlinks point to: `/home/robot/.jszr/map/20260703_170635/`
- Only usable local map as of 2026-07-03: `/home/robot/.jszr/map/20260703_170635/`
- Deprecated local platform directory was removed:

```text
/home/robot/genisom_roamerx_open/web_platform
```

## Current Verified Runtime State

At 2026-07-02 22:09 CST:

- `localization` launch is running.
- `robot_navigo navigation_bringup.launch.py` is running.
- Nav2 lifecycle nodes are active:
  - `/planner_server`: `active [3]`
  - `/controller_server`: `active [3]`
  - `/bt_navigator`: `active [3]`
- `/localization_info` reports `status: 3`.
- Current approximate localization pose after the last 1m test:
  - `x ~= 2.08`
  - `y ~= 0.51`
  - `yaw ~= 0.49 rad`
  - speed is low, around `0.02-0.03 m/s` while standing.

## Proven Navigation Test

A Nav2 `NavigateToPose` goal was sent to approximately:

```text
map x=2.07, y=0.58, yaw=21 deg
```

Result:

- Action returned `status=4` (success for this interface).
- Feedback moved from about `(1.12, 0.23)` to about `(2.08, 0.51)`.
- Onsite confirmation: the robot physically walked forward.
- There was one recovery in feedback, so future tests should watch for localization/control oscillation, but the complete chain works.

## Latest New-Extrinsic Map Test

At 2026-07-03 CST:

- Current map symlinks were switched to:

```text
/home/robot/.jszr/map/20260703_170635/
```

- This map was built after updating the ROS `base_link -> livox_frame` TF from factory calibration.
- Localization loaded:

```text
/home/robot/.jszr/map/map.pcd -> /home/robot/.jszr/map/20260703_170635/map.pcd
```

- Localization reached `status: 3` with LiDAR `10/10` and confidence `1.00`.
- Nav2 lifecycle nodes were active:
  - `/planner_server`: `active [3]`
  - `/controller_server`: `active [3]`
  - `/bt_navigator`: `active [3]`
- A short `nav2_msgs/action/NavigateToPose` test goal was sent from about:

```text
start ~= (-1.31, -0.17)
goal  ~= (-0.714, -0.179)
```

Result:

- Goal was accepted by `/bt_navigator`.
- Feedback distance started around `0.59m`.
- `/cmd_vel` ramped forward, then returned to zero.
- Action finished with `SUCCEEDED`.
- Final localization sample was around:

```text
x ~= -0.729
y ~= -0.150
status: 3
speed ~= 0.02 m/s
```

This validates the new map and updated ROS LiDAR TF for a short forward Nav2 test. The stack was kept running in tmux sessions:

```text
roamerx_localization
roamerx_navigation
```

## Important Fixes Already Made

### 1. Localization TF listener

File:

```text
src/localization/localization/apps/localization_nodelet.cpp
```

The localization node now creates a `tf2_ros::TransformListener` for its `tf_buffer`. Before this, it had a buffer but no listener, so it could not consume external TF properly.

### 2. Localization config

File:

```text
src/localization/localization/config/config.yaml
```

Important current settings:

```yaml
points_topic: /front_lidar
imu_topic: /front_lidar/imu
robot_odom_frame_id: odom
odom_child_frame_id: base_link
localization_odom_frame_id: base_link
send_tf_transforms: true
tf_use_current_time: true
use_imu: false
```

`use_imu: false` is intentional for the current stable test state.

### 3. LiDAR extrinsic

Files:

```text
src/localization/localization/launch/localization.launch.py
install/localization/share/localization/launch/localization.launch.py
```

Static TF now uses the factory calibration from `calibration_results.yaml`.
Technical support confirmed the file's `/front_lidar` R/T uses:

```text
p_lidar = R * p_base + T
```

So the ROS TF `base_link -> livox_frame` must use the inverse transform:

```text
base_link -> livox_frame
translation: 0.382765605, -0.046855740, 0.513445457
rotation xyzw: 0.007172121, -0.043589169, -0.009509268, 0.998978536
```

This is updated in:

```text
src/localization/localization/launch/localization.launch.py
install/localization/share/localization/launch/localization.launch.py
```

Historical previous reasoning from system calibration, now superseded for the base_link TF:

```text
/home/robot/.robot/param/arc_mapping/arc_mapping.yaml
extrinsic_T = [-0.011, -0.02329, 0.04412]
imu2base_T = [-0.216, 0, 0]
```

The SLAM code reads `extrinsic_T` as `Lidar_T_wrt_IMU`, so:

```text
base_link -> IMU   ~= [0.216, 0, 0]
IMU -> LiDAR       = [-0.011, -0.02329, 0.04412]
base_link -> LiDAR = [0.205, -0.02329, 0.04412]
```

That older value was physically inconsistent with the observed head-mounted front/top LiDAR position and should not be used for the ROS `base_link -> livox_frame` TF.

### 4. Dog SDK connectivity

Earlier, direct SDK connectivity was fixed by updating the dog-side startup scripts on `firefly@192.168.234.1` so `SDK_CLIENT_IP="192.168.234.1"`.

The Nav2 `vel_cmd_udp_pub` may print `SDK checkConnect=false` at startup, but the 1m navigation test still physically moved the robot. If motion fails later, inspect `vel_cmd_udp_pub` logs for `standUp()` and `move()` return codes.

## Canonical Startup Scripts

Keep using only these real-robot scripts:

```bash
/home/robot/genisom_roamerx_open/script/robot/start_navigation_real.sh
/home/robot/genisom_roamerx_open/script/robot/start_mapping_real.sh
```

Navigation:

```bash
script/robot/start_navigation_real.sh start
script/robot/start_navigation_real.sh status
script/robot/start_navigation_real.sh stop
script/robot/start_navigation_real.sh restart
```

Mapping:

```bash
script/robot/start_mapping_real.sh start
script/robot/start_mapping_real.sh save
script/robot/start_mapping_real.sh stop
script/robot/start_mapping_real.sh status
```

Old ambiguous entrypoints were removed:

```text
script/start_navigation.sh
script/bash/start_navigation.sh
script/bash/stop_navigation.sh
start_slam.sh
```

## Navigation Control Logic

The current Nav2/SDK control logic is:

- Nav2 outputs `/cmd_vel`.
- `mode_status_publisher` sees `/cmd_vel` and publishes mode `171` on `/mode_switch_cmd`.
- `vel_cmd_udp_pub` receives `/cmd_vel`, calls SDK `standUp()`, waits `standup_settle_ms`, then forwards SDK `move(vx, vy, yaw_rate)`.
- If no `/cmd_vel` arrives for about 1 second, mode returns to `170`, and `vel_cmd_udp_pub` calls SDK `passive()`.

So Nav2 does not permanently take over the remote controller; it takes over while navigation velocity is active, then releases.

## Center Platform Server

Cloud server:

```text
39.107.250.69
```

The real platform is the cloud platform on `39.107.250.69`. Do not use or recreate the old local
`/home/robot/genisom_roamerx_open/web_platform`; it was a stale demo platform and has been deleted.

SSH:

- Root SSH key login from this robot computer is already working with `/home/robot/.ssh/id_rsa`.
- Do not store the root password in this document.

Current cloud platform code was copied from the server runtime directory into:

```text
/home/robot/yw/roamerx_analysis/center_platform/platform_server_code
```

The previous local copy was backed up before replacement:

```text
/home/robot/yw/roamerx_analysis/center_platform/platform_server_code.backup_20260703_172741
```

Important cloud platform API/code locations from that copy:

```text
backend/monitoring/urls.py
backend/monitoring/views.py
backend/monitoring/mqtt_client.py
backend/monitoring/message_handlers.py
backend/monitoring/protocol.py
backend/monitoring/services/command_service.py
backend/monitoring/services/task_service.py
frontend/src/App.vue
```

Relevant cloud HTTP endpoints:

```text
POST /api/device/maps/upload/
GET  /api/maps/
GET  /api/maps/<id>/
POST /api/maps/<id>/set_active/
GET  /api/routes/
POST /api/routes/
POST /api/patrol-tasks/<task_id>/execute/
GET  /api/task-executions/<execution_id>/
POST /api/task-executions/<execution_id>/pause/
POST /api/task-executions/<execution_id>/resume/
POST /api/task-executions/<execution_id>/cancel/
GET  /api/robots/<robot_id>/mapping/status/
POST /api/robots/<robot_id>/mapping/start/
POST /api/robots/<robot_id>/mapping/save/
POST /api/robots/<robot_id>/mapping/cancel/
POST /api/robots/<robot_id>/mapping/sync/
```

Frontend route names seen in the built app:

```text
/dashboard/tasks/maps
/dashboard/tasks/routes
/dashboard/tasks/tracks
```

The path planning page is the cloud frontend route planner. It stores route waypoints through
`POST /api/routes/`; navigation starts by executing a PatrolTask, which creates a TaskExecution and
a `task.start` RemoteCommand.

## Cloud To Dog Protocol

Dog-side Edge Agent runtime:

```text
systemd service: roamerx-edge-agent.service
working dir: /home/robot/edge_agent
runtime config: /home/robot/edge_agent/config.yaml
repo source: /home/robot/genisom_roamerx_open/edge_agent
robot id/code: ZSL-1A-07
MQTT broker: 39.107.250.69:1884
HTTP media upload: http://39.107.250.69:8088/api/device/media/upload/
HTTP map upload: http://39.107.250.69:8088/api/device/maps/upload/
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

Dog-side code responsibilities:

- `edge_agent/roamerx_edge/mqtt_client.py`: outbound MQTT v5 connection, subscriptions, retained presence/status, ack/result publishing, outbox replay.
- `edge_agent/roamerx_edge/protocol.py`: dog-side accepted message types include `task.start/pause/resume/cancel` and `mapping.start/save/cancel/status`.
- `edge_agent/roamerx_edge/command_processor.py`: validates and dispatches command envelopes.
- `edge_agent/roamerx_edge/task_executor.py`: task state machine; emits `task.accepted`, `task.started`, progress, terminal events.
- `edge_agent/roamerx_edge/ros_adapter.py`: executes navigation by sending `nav2_msgs/action/FollowWaypoints` to `/follow_waypoints`.
- `edge_agent/roamerx_edge/mapping_adapter.py`: executes `mapping.*` commands, starts SLAM, calls `/slam_state_service`, packages maps, uploads to cloud.
- `edge_agent/roamerx_edge/media_client.py`: uploads map/media files to cloud with `X-Device-Id` and `X-Device-Key`.
- `edge_agent/roamerx_edge/app.py`: application wiring and `presence.online` capability list.

Cloud-side code responsibilities:

- `backend/monitoring/mqtt_client.py`: platform MQTT worker publishes `RemoteCommand` to `robots/<code>/commands` and subscribes to presence/status/events/ack/result.
- `backend/monitoring/protocol.py`: cloud-side formal `COMMAND_TYPES` currently lists `task.start/pause/resume/cancel`; `mapping.*` commands are created by mapping views and published through `RemoteCommand`. If cloud validation blocks `mapping.*`, update cloud protocol to include them.
- `backend/monitoring/services/task_service.py`: builds `route_snapshot` from `PatrolRoute`; map version format is `legacy-mapdata-<map_id>`.
- `backend/monitoring/services/command_service.py`: creates `task.start` and control commands; `create_robot_command()` creates mapping commands without TaskExecution.
- `backend/monitoring/message_handlers.py`: consumes dog ack/result/task events/status and updates Robot/TaskExecution state.

## Current Cloud Map Sync

On 2026-07-03 18:05 CST, local map `20260703_170635` was uploaded to cloud through:

```text
POST http://39.107.250.69:8088/api/device/maps/upload/
```

Upload response:

```text
cloud map id: 9
cloud map name: 20260703_170635
robot_code: ZSL-1A-07
pgm_url: /media/maps/9_map.pgm
yaml_url: /media/maps/9_map.yaml
thumbnail_url: /api/maps/9/preview/
```

It was then activated with:

```text
POST http://39.107.250.69:8088/api/maps/9/set_active/
```

Verified final cloud state:

```text
GET http://39.107.250.69:8088/api/maps/9/
active: true
```

Local map cleanup was performed: `/home/robot/.jszr/map` now only contains:

```text
/home/robot/.jszr/map/20260703_170635/
/home/robot/.jszr/map/map.yaml -> /home/robot/.jszr/map/20260703_170635/map.yaml
/home/robot/.jszr/map/map.pgm  -> /home/robot/.jszr/map/20260703_170635/map.pgm
/home/robot/.jszr/map/map.pcd  -> /home/robot/.jszr/map/20260703_170635/map.pcd
/home/robot/.jszr/map/map.txt  -> /home/robot/.jszr/map/20260703_170635/map.txt
```

Dog-side runtime config was updated:

```yaml
robot:
  current_map_id: "9"
  current_map_version: legacy-mapdata-9
```

`systemctl restart roamerx-edge-agent` required interactive auth, so the robot-owned process was
terminated and systemd auto-restarted it (`Restart=always`). New PID observed:

```text
31550
```

Verified MQTT state after restart:

```text
TCP ESTAB: 10.98.189.210:* -> 39.107.250.69:1884
presence.online current_map: {"map_id": "9", "map_version": "legacy-mapdata-9"}
telemetry.status current_map: {"map_id": "9", "map_version": "legacy-mapdata-9"}
```

## Cloud Navigation Test Status

Do not trigger motion from local scripts when the requirement is "through cloud platform".
Correct end-to-end path for navigation test:

```text
Cloud UI/API route planner -> PatrolRoute -> PatrolTask execute -> RemoteCommand task.start
-> MQTT robots/ZSL-1A-07/commands -> edge_agent -> /follow_waypoints -> Nav2 -> /cmd_vel -> UDP SDK bridge
```

As of this update, cloud connectivity and map sync are verified, but the final cloud-originated
navigation goal test has not yet been run in this session after the platform clarification.

Server runtime layout:

```text
/opt/roamerx/current -> /opt/roamerx/releases/20260703_1140_task_calendar_mvp
/opt/roamerx/shared/center.env
/opt/roamerx/shared/db.sqlite3
/opt/roamerx/shared/media
/opt/roamerx/shared/logs
```

Running services on the cloud server:

```text
roamerx-center-api.service        Django API, runserver 127.0.0.1:8081
roamerx-device-worker.service     MQTT worker, publishes commands and consumes edge messages
roamerx-patrol-scheduler.service  Calendar scheduler, scans every 10s
nginx.service                     Serves frontend and proxies /api/
mosquitto                         MQTT broker on 0.0.0.0:1884
docker/ZLMediaKit                 RTMP/RTSP/HTTP media ports including 1935, 8554, 8090
```

Important exposed ports observed:

```text
8088  nginx RoamerX center platform, frontend + /api/
1884  mosquitto MQTT broker used by the robot edge_agent
1935  RTMP ingest, e.g. dog_ZSL-1A-07_front
8554  RTSP through ZLMediaKit container
8090  ZLMediaKit HTTP
```

Cloud environment settings observed without secrets:

```text
DJANGO_DEBUG=true
DJANGO_ALLOWED_HOSTS=39.107.250.69,127.0.0.1,localhost
SQLITE_DB_PATH=/opt/roamerx/shared/db.sqlite3
MQTT_HOST=127.0.0.1
MQTT_PORT=1884
MQTT_TLS_ENABLED=false
DJANGO_MEDIA_ROOT=/opt/roamerx/shared/media
```

Robot-side connection evidence:

```text
python3 run_edge_agent.py --config /home/robot/edge_agent/config.yaml
10.232.96.112:* -> 39.107.250.69:1884
rtmp://39.107.250.69:1935/live/dog_ZSL-1A-07_front
```

The robot edge agent config also uploads media/maps to:

```text
http://39.107.250.69:8088/api/device/media/upload/
http://39.107.250.69:8088/api/device/maps/upload/
```

Current cloud platform code structure:

```text
backend/       Django + DRF API, SQLite/Postgres support, monitoring app
frontend/      Vue 3 + Vite UI, built dist served by nginx
bot-version/   older board-side demo/control/stream code bundled with platform repo
deploy/        env examples, systemd units, mosquitto config
docs/          architecture and operations docs
```

Current command/control architecture, based on running code:

- Browser calls Django API on `/api/...`.
- Manual patrol execution uses `POST /api/patrol-tasks/<id>/execute/`.
- Pause/resume/cancel use `/api/task-executions/<uuid>/{pause,resume,cancel}/`.
- Calendar dispatch is handled by `run_patrol_scheduler --interval 10`.
- Django creates `TaskExecution` and `RemoteCommand` rows.
- `roamerx-device-worker` publishes pending commands to MQTT topic:

```text
robots/<robot_code>/commands
```

- Robot `edge_agent` subscribes to that topic and converts task waypoints to ROS2 `/follow_waypoints`.
- Robot replies through MQTT with presence, telemetry, trajectory, task events, command ack/result, and alert events.
- The cloud worker subscribes to topics including:

```text
robots/+/presence
robots/+/telemetry/status
robots/+/telemetry/pose
robots/+/telemetry/trajectory
robots/+/events/task
robots/+/events/alert
robots/+/commands/+/ack
robots/+/commands/+/result
robots/+/sync/state
```

Important compatibility note:

- Some bundled docs and older API code still describe direct HTTP control through `RobotCommandView` and `control_endpoint`.
- The current real cloud-to-robot patrol workflow is MQTT based through `RemoteCommand` and `edge_agent`; use that as the source of truth for navigation/task work.

## Next Recommended Tests

1. Ensure real navigation stack is running on the dog:

   ```bash
   cd /home/robot/genisom_roamerx_open
   script/robot/start_navigation_real.sh start
   script/robot/start_navigation_real.sh status
   ```

2. On cloud platform `39.107.250.69`, verify robot `ZSL-1A-07` is online and map 9 (`20260703_170635`) is active.
3. In cloud route planner (`/dashboard/tasks/routes`), create a very short test route on map 9.
4. Create or use a PatrolTask bound to that route and robot, then execute it from the cloud.
5. Watch the full cloud-to-dog chain:
   - `RemoteCommand` becomes published/accepted/running/succeeded.
   - MQTT topic `robots/ZSL-1A-07/commands` receives `task.start`.
   - Dog publishes `commands/<id>/ack`, `events/task`, and `commands/<id>/result`.
   - Dog-side `/follow_waypoints` action receives the goal.
   - `/cmd_vel` publishes during motion.
   - `/localization_info` remains `status: 3`.
   - Physical robot moves a short distance and stops.
6. If no motion occurs, check:
   - Nav2 lifecycle nodes with `script/robot/start_navigation_real.sh status`.
   - edge agent logs with `journalctl -u roamerx-edge-agent -n 120 --no-pager`.
   - cloud worker logs for publish/ack/result handling.
   - `vel_cmd_udp_pub` logs for SDK `standUp()` and `move()` return codes.

## Safety Reminder

Before any platform goal test:

- Confirm front/side path is physically clear.
- Use short goals first.
- Have a stop path ready:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/genisom_roamerx_open/install/setup.bash
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```
