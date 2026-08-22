# HANDOFF.md

Current handoff state as of 2026-07-03 19:55 CST.

## Latest Session Summary

- Created this handoff structure: `AGENTS.md`, `HANDOFF.md`, `TODO.md`.
- Verified cloud platform information from `CURRENT_ROBOT_CONTEXT.md`.
- Refactored cloud route planning coordinate conversion so route waypoints use ROS `map` coordinates as navigation truth and current image pixels only for rendering.
- Added patrol task template delete support in the cloud UI/API.
- Investigated robot navigation startup. `script/robot/start_navigation_real.sh start` reported localization OK and started Nav2, but the localization/Nav2 launch processes did not remain running in that attempt. `navigation.log` was empty. This still needs follow-up.
- Confirmed `roamerx_edge_agent` was running as a robot-owned process:

```text
python3 run_edge_agent.py --config /home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml
```

- Discussed the right architecture for Nav2 persistence:
  - platform displays detailed Nav2/localization state
  - platform sends controlled `nav.*` RemoteCommands
  - dog-side edge_agent or a systemd/supervisor component owns actual process start/restart/status

## Cloud Platform Map Delete Fix

User reported that deleting maps in the map management UI failed.

Implemented and deployed:

- Backend file:

```text
/home/dogrobot/platform/backend/monitoring/views.py
/opt/roamerx/current/backend/monitoring/views.py
```

- Frontend file:

```text
/home/dogrobot/platform/frontend/src/views/MapsPage.vue
/opt/roamerx/current/frontend/src/views/MapsPage.vue
```

Behavior after fix:

- `DELETE /api/maps/<id>/` rejects active maps with HTTP `409` and clear `detail`.
- `DELETE /api/maps/<id>/` rejects maps referenced by routes, zones, tracks, task executions, or inspection events, with counts in `references`.
- Non-active, unreferenced maps still delete normally.
- Frontend now shows `error.message` instead of only `删除失败`.

Deployment:

```text
roamerx-center-api.service restarted and active
frontend dist rebuilt locally and uploaded to /opt/roamerx/current/frontend/dist
nginx reloaded
```

Verification:

- Deleting active map `id=1` returned:

```text
409 {"detail":"当前活动地图不能删除，请先切换活动地图后再删除。"}
```

- Temporary map `id=12` was created and deleted successfully with HTTP `204`.

Current cloud route/map state:

```text
map id=9  20260703_170635  active=true  width=151 height=131 resolution=0.05 origin=[-5.5576,-5.17251,0]
route id=4  测试  task "11" binds here
```

## Route Coordinate Refactor

User reported that reopening route 4 showed points in different positions and the physical target was offset.

Implemented and deployed:

- Frontend files:

```text
/home/dogrobot/platform/frontend/src/views/RoutePlannerPage.vue
/opt/roamerx/current/frontend/src/views/RoutePlannerPage.vue
```

- New behavior:
  - Click conversion uses the loaded image bounding rect plus current map metadata.
  - Saved waypoint objects use ROS `map` coordinates as navigation truth:

```json
{"frame_id":"map","x":-1.23,"y":0.45,"yaw":0.0,"image_x":80.0,"image_y":23.0,"u":0.5298,"v":0.1769}
```

  - `image_x/image_y` and `u/v` are kept only to make marker rendering stable on reopen and browser resize.
  - Old legacy array points such as `[632,184]` are detected as image pixels when possible and converted through the selected map.
  - Saving an opened route updates that route instead of creating a duplicate route.

Deployment:

```text
frontend dist rebuilt and deployed
current served asset: /assets/index-4a5a8d52.js
nginx reloaded
```

Route 4 now contains the converted user points in the new stable format:

```json
[
  {"frame_id": "map", "x": -1.5811, "y": 0.2185, "yaw": 0.1132, "image_x": 79.53, "image_y": 23.18, "u": 0.52668874, "v": 0.17694656},
  {"frame_id": "map", "x": -0.1971, "y": 0.3760, "yaw": 0.1132, "image_x": 107.21, "image_y": 20.03, "u": 0.71, "v": 0.15290076}
]
```

After the user hard-refreshes the browser, route 4 should reopen in the same relative map position. If the intended physical target still needs correction, edit route 4 on the page and save; do not create a duplicate route.

## Patrol Task Delete

Implemented and deployed:

- Backend:

```text
/home/dogrobot/platform/backend/monitoring/views.py
/opt/roamerx/current/backend/monitoring/views.py
```

- Frontend:

```text
/home/dogrobot/platform/frontend/src/views/TasksPage.vue
/home/dogrobot/platform/frontend/src/services/api.js
/opt/roamerx/current/frontend/src/views/TasksPage.vue
/opt/roamerx/current/frontend/src/services/api.js
```

Behavior:

- `DELETE /api/patrol-tasks/<id>/` deletes unreferenced task templates.
- If a task already has execution history or calendar schedules, backend returns `409` with a clear reason.
- UI now has a delete button in the patrol task template list.

Verification:

- Frontend build passed.
- Cloud `manage.py check` passed.
- Temporary task template `id=5` was created and deleted through HTTP `DELETE`, returning `204`; DB confirmed it no longer exists.

## Route Planner Navigation Test Panel

Implemented and deployed on 2026-07-03 around 20:06 CST.

Purpose:

- In `/dashboard/tasks/routes`, show the active/current map, route waypoints, Nav2/localization state, and the robot's current pose on the same map.
- Allow operator to start/query/restart/stop the navigation stack from the route planning page without starting a patrol route.

Frontend:

```text
/home/dogrobot/platform/frontend/src/views/RoutePlannerPage.vue
/home/dogrobot/platform/frontend/src/services/api.js
/opt/roamerx/current/frontend/src/views/RoutePlannerPage.vue
/opt/roamerx/current/frontend/src/services/api.js
```

Current served frontend asset:

```text
/assets/index-e35d9911.js
/assets/index-97567c80.css
```

Backend:

```text
/home/dogrobot/platform/backend/monitoring/models.py
/home/dogrobot/platform/backend/monitoring/protocol.py
/home/dogrobot/platform/backend/monitoring/urls.py
/home/dogrobot/platform/backend/monitoring/views.py
/home/dogrobot/platform/backend/monitoring/services/telemetry_service.py
```

New cloud API:

```text
GET  /api/robots/<id>/navigation/status/
POST /api/robots/<id>/navigation/probe/    -> nav.status
POST /api/robots/<id>/navigation/start/    -> nav.start
POST /api/robots/<id>/navigation/restart/  -> nav.restart
POST /api/robots/<id>/navigation/stop/     -> nav.stop
```

Dog-side edge_agent:

```text
/home/dogrobot/edge-agent/roamerx_edge/navigation_stack_adapter.py
/home/dogrobot/edge-agent/roamerx_edge/command_processor.py
/home/dogrobot/edge-agent/roamerx_edge/protocol.py
/home/dogrobot/edge-agent/roamerx_edge/app.py
/home/dogrobot/edge-agent/roamerx_edge/config.py
/home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml
```

Runtime behavior:

- `nav.status/start/restart/stop` are real RemoteCommands over MQTT.
- Edge executes `/home/dogrobot/robot/script/robot/start_navigation_real.sh <action>`.
- `nav.start` only starts localization/Nav2; it does not send route goals or move the dog.
- Route execution still uses the existing patrol task path: cloud task -> MQTT `task.start` -> edge_agent -> `/follow_waypoints`.

Verification:

- Frontend build passed.
- Cloud `manage.py check` passed.
- Services active: `roamerx-center-api`, `roamerx-device-worker`, `nginx`, `roamerx-edge-agent`.
- Non-motion `POST /api/robots/1/navigation/probe/` succeeded:
  - command type `nav.status`
  - command status `succeeded`
  - edge result showed `/planner_server`, `/controller_server`, `/bt_navigator` active `[3]`
  - `/cmd_vel` bridge subscribers present.
- `GET /api/robots/1/navigation/status/` currently returns:
  - `connection_status=online`
  - `current_map_id=9`
  - `localization_status=normal`
  - `nav_ready=true`
  - pose around `x=-0.2207`, `y=0.0880`, `yaw=0.00267`.

Telemetry fix:

- `TelemetryService.apply_status` now accepts newer `sampled_at` even if `state_version` decreases after edge_agent restart. This fixes stale map position display after restarting edge_agent.

## Known Robot Navigation State

Previously verified good state:

- Short local Nav2 `NavigateToPose` tests succeeded on the real robot.
- Latest local map `20260703_170635` localized with `/localization_info.status == 3`.
- A short test from roughly `(-1.31, -0.17)` to `(-0.714, -0.179)` succeeded and physically moved the robot.

Current unresolved issue:

- On the latest attempted startup, no tmux sessions existed.
- `ros2 daemon` had an `rclpy.ok()` issue and was restarted.
- `start_navigation_real.sh start` loaded the map successfully and saw localization status 3, but later `ros2 node list` did not show localization/Nav2 nodes.
- `script/robot/start_navigation_real.sh status` did not show lifecycle nodes.
- Need to make Nav2/localization persistent before cloud-originated motion testing.

## Cloud And Edge Connectivity

Cloud platform:

```text
http://39.107.250.69:8088
```

Robot:

```text
ZSL-1A-07
```

MQTT:

```text
39.107.250.69:1884
```

Dog edge agent runtime:

```text
/home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml
/home/dogrobot/edge-agent
```

Expected map config in dog edge agent from prior work:

```yaml
robot:
  current_map_id: "9"
  current_map_version: legacy-mapdata-9
```

Because cloud active map is currently `id=1`, verify whether dog edge agent and cloud active map need to be reconciled.

## Useful Commands

Robot status:

```bash
cd /home/dogrobot/robot
script/robot/start_navigation_real.sh status
ps -ef | rg 'ros2 launch localization|robot_navigo|localization_node|planner_server|controller_server|bt_navigator|vel_cmd|mode_status'
```

ROS status without daemon:

```bash
source /opt/ros/humble/setup.bash
source /home/dogrobot/robot/install/setup.bash
ROS_DISABLE_DAEMON=1 ros2 node list
ROS_DISABLE_DAEMON=1 ros2 lifecycle get /planner_server
ROS_DISABLE_DAEMON=1 ros2 lifecycle get /controller_server
ROS_DISABLE_DAEMON=1 ros2 lifecycle get /bt_navigator
```

Cloud API status:

```bash
ssh root@39.107.250.69 'systemctl status roamerx-center-api.service roamerx-device-worker.service --no-pager'
curl -sS http://39.107.250.69:8088/api/maps/
```

Cloud Django shell with the correct shared DB:

```bash
ssh root@39.107.250.69 'cd /opt/roamerx/current && env $(grep -v "^#" /opt/roamerx/shared/center.env | xargs) /root/miniconda/envs/py310/bin/python backend/manage.py shell'
```
