# TODO.md

Next tasks, in recommended order.

## 1. Verify Route 4 With New Coordinate Editor

- Ask the user to hard-refresh the cloud page so the browser loads `/assets/index-e35d9911.js`, which includes the route planner navigation test panel.
- Open `/dashboard/tasks/routes`, select route `id=4` (`测试`) on map `id=9`.
- If markers still do not match the intended physical targets, delete the two route points on the page, click them again, and save. The new save format includes `frame_id`, `x/y/yaw`, `image_x/image_y`, and `u/v`.
- Verify with:

```bash
curl -sS http://39.107.250.69:8088/api/routes/4/ | python3 -m json.tool
```

Expected map state:

```text
map id=9 active=true width=151 height=131 resolution=0.05 origin=[-5.5576,-5.17251,0]
edge_agent current_map_id="9"
task "11" / PatrolTask id=4 -> route id=4
```

## 2. Stabilize Navigation Stack On The Dog

- Reproduce why `start_navigation_real.sh start` reports success but localization/Nav2 processes do not remain running.
- Check:
  - `/tmp/roamerx_nav_logs/localization.log`
  - `/tmp/roamerx_nav_logs/navigation.log`
  - `ros2 launch localization localization.launch.py` in foreground
  - `ros2 launch robot_navigo navigation_bringup.launch.py ...` in foreground
  - lifecycle nodes and `/follow_waypoints`
- Target healthy state:
  - localization process persists
  - `/localization_info.status == 3`
  - `/planner_server`, `/controller_server`, `/bt_navigator` active
  - `/follow_waypoints` action server available
  - `/cmd_vel` bridge subscriber present

## 3. Add Platform-Visible Nav2 State

Extend dog telemetry and platform UI beyond a single `nav_ready` flag.

Current partial implementation:

- `/dashboard/tasks/routes` has a navigation test panel.
- `GET /api/robots/<id>/navigation/status/` returns current map, localization, `nav_ready`, latest pose, and latest `nav.*` command summary.
- The map displays the robot pose marker when the robot-reported map matches the selected map.

Remaining:

- Add the same richer Nav2 state to the main robot dashboard, not only route planning.
- Split `nav_ready` into process/lifecycle/action/cmd_vel bridge details in structured telemetry instead of only parsing `nav.status` stdout.

Suggested runtime payload:

```json
{
  "runtime": {
    "ros_ready": true,
    "localization_ready": true,
    "nav2_process_running": true,
    "nav2_lifecycle_active": true,
    "follow_waypoints_ready": true,
    "cmd_vel_bridge_ready": true,
    "nav_state": "nav2_ready",
    "last_nav_error": null
  }
}
```

Suggested platform state labels:

```text
offline
edge_online
ros_online
localization_starting
localization_ready
nav2_starting
nav2_ready
nav2_degraded
nav2_failed
task_running
manual_takeover
emergency_stop
```

## 4. Add Controlled Nav2 Commands

Add cloud-to-dog `nav.*` commands:

```text
nav.status
nav.start
nav.restart
nav.stop
nav.recover
```

Current implemented subset:

```text
nav.status
nav.start
nav.restart
nav.stop
```

Remaining:

- Add `nav.recover`.
- Add stronger guardrails: block `nav.restart` and `nav.stop` while a task is actively running unless a forced operator action is added.

Design constraints:

- Platform sends RemoteCommand only.
- Dog-side edge_agent or dog-side supervisor executes `start_navigation_real.sh`.
- Commands must be idempotent.
- `nav.restart` and `nav.stop` must be blocked while a task is running unless explicitly forced.
- `nav.start` when already ready should return `already_ready`.

## 5. Cloud-To-Dog Non-Motion Test

Before motion:

- Verify robot online in platform.
- Verify MQTT presence/status from `ZSL-1A-07`.
- Run a non-motion command such as `mapping.status` or future `nav.status`.
- Confirm:
  - RemoteCommand created
  - worker publishes
  - dog receives
  - dog publishes ack/result
  - platform updates status

## 6. Cloud-Originated Short Navigation Test

Only after map state and Nav2 state are clean:

1. Confirm physical path is clear.
2. Ensure stop command is ready.
3. Use route `id=4` after confirming its markers are correct on the map.
4. Use PatrolTask `id=4` / name `11`, bound to robot `ZSL-1A-07`.
5. Execute through the cloud platform, not local ROS.
6. Watch:
   - RemoteCommand status
   - MQTT command/ack/result
   - edge_agent logs
   - `/follow_waypoints`
   - `/cmd_vel`
   - `/localization_info.status == 3`
   - physical robot movement and stop

## 7. Clean Up Map Management UX

Map delete now returns useful errors. Follow-ups:

- Surface reference counts in the map card/detail panel before delete.
- Add "set another active map first" guidance for active-map deletion.
- Consider a "force cleanup unused routes/zones" admin workflow, but do not cascade-delete important patrol history silently.

## 8. Patrol Task Template Cleanup UX

- Delete button is deployed for patrol task templates.
- Follow-up: add a compact reference hint before delete for templates that already have execution history or calendar schedules, since backend currently rejects those with HTTP `409`.
