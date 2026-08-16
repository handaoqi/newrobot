# MCP Remote Control And Skills

## Purpose

This module exposes the existing cloud remote-control path as MCP tools. It does not bypass the platform or publish ROS commands directly:

```text
MCP client -> platform API -> RemoteCommand -> MQTT -> Edge Agent -> ROS teleop -> vendor remote protocol
```

The platform records the command lifecycle, so the web page, API, and MCP client see the same command ID and terminal result.

## Interfaces

The token-protected platform API is under `/api/mcp/robots/<robot_id>/`.

| MCP tool | Function |
| --- | --- |
| `robot_direction` | `forward`, `backward`, `left`, `right`, turns, stop, or velocity vector |
| `robot_speed` | `micro`, `low`, `medium`, `high` |
| `robot_action` | stand up, prone, passive, start/stop motion control |
| `robot_skill_list` | read the Edge-backed catalog of executable presets and their requirements |
| `robot_skill_run` | start a preset or structured high-level skill |
| `robot_skill_status` | return the source command lifecycle |
| `robot_skill_cancel` | cancel a running local skill |
| `robot_person_detection` / `robot_person_detection_status` | enable or inspect person detection |
| `robot_person_follow` / `robot_person_follow_status` / `robot_person_follow_stop` | follow an explicit `track_id` or the latest image-center person, inspect, or stop persistent local following |
| `robot_remote_control_capabilities` | return the released, user-facing capability list |

The remote page calls the same lower-level command types. Its `lie_down` command is now labeled **匍匐**; the compatible robot-side command name remains `lie_down`.

Persistent person following is local to the dog. The detection process writes the latest frame atomically to `/run/roamerx/person_detections.json`; the Edge Agent uses that local data at a fixed control rate. Target loss, a stale frame, a front obstacle, an explicit stop, task takeover, passive mode, prone mode, and Edge shutdown all send a zero-velocity command and end the session.

For `robot_person_follow`, provide an explicit `track_id`, or use `target="center"` only when the user explicitly asks to follow the person in the middle of the image. The platform resolves that request from a fresh, enabled detection frame by considering only `person` boxes and choosing the box whose center is nearest the camera-frame center. It then submits the resolved stable `track_id` through the normal RemoteCommand path.

## Skills

`robot_skill_list` is the source of truth for released presets. The current catalog includes timed prone/micro forward and backward movement, four 5-meter localization-closed movement variants, left/right full turns, left/right forward-avoid maneuvers, and four directional detours.

AI clients may submit explicit `steps` for arbitrary sequences and retain the natural-language intent in `description`. The Edge Agent validates and executes structured steps only; it does not call an LLM. A velocity step is re-published every 150 ms. Every success, failure, or cancellation ends with a zero-velocity command. Turns and distance moves require live pose data; they fail rather than estimate by time when pose data is unavailable. Avoid-forward checks the front laser scan. Backward presets are localization-closed but do not claim rear obstacle protection.

## Runtime

`mcp_server/roamerx_robot_mcp.py` supports stdio and Streamable HTTP. The installed service binds only `127.0.0.1:8095/mcp` and uses `ROAMERX_PLATFORM_TOKEN` from systemd's runtime environment. Tokens must not be committed. A reverse proxy with its own authentication is required before exposing the HTTP transport outside the robot.

For a single-robot installation, set the positive numeric `ROAMERX_DEFAULT_ROBOT_ID` in that same service environment. All robot-specific MCP tools then accept an omitted `robot_id`; an explicit ID overrides the default. Missing or invalid IDs are rejected locally before any platform request.

The installable `roamerx-basic-teleop` plugin points to this loopback endpoint and bundles the corresponding Codex Skill. Every released control action is direct once the user clearly requests it; no separate “现场通道和行进路线已确认安全” confirmation is required. Presets and person following still require an explicit preset, structured steps, or `track_id`; the client must not guess an ambiguous target.

## Verification

- Edge unit tests cover skill dispatch, cancellation, stop-on-exit, and missing-pose failure.
- Platform tests cover authenticated MCP command creation and invalid tool values.
- The deployment uses the existing cloud platform deployment script, then restarts only Edge Agent; no physical movement is triggered during verification.
