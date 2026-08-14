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
| `robot_skill_run` | start a preset or structured high-level skill |
| `robot_skill_status` | return the source command lifecycle |
| `robot_skill_cancel` | cancel a running local skill |

The remote page calls the same lower-level command types. Its `lie_down` command is now labeled **匍匐**; the compatible robot-side command name remains `lie_down`.

## Skills

Presets: `prone_forward_5s`, `micro_reverse_5s`, `turn_left_full_circle`, `left_two_steps_then_avoid_forward`, and `turn_left_and_forward_detour`.

AI clients may submit explicit `steps` for arbitrary sequences and retain the natural-language intent in `description`. The Edge Agent validates and executes structured steps only; it does not call an LLM. A velocity step is re-published every 150 ms. Every success, failure, or cancellation ends with a zero-velocity command. Turns and distance moves require live pose data; they fail rather than estimate by time when pose data is unavailable. Avoid-forward checks the front laser scan, stops at an obstacle, then performs the defined left-turn bypass.

## Runtime

`mcp_server/roamerx_robot_mcp.py` supports stdio and Streamable HTTP. The installed service binds only `127.0.0.1:8095/mcp` and uses `ROAMERX_PLATFORM_TOKEN` from systemd's runtime environment. Tokens must not be committed. A reverse proxy with its own authentication is required before exposing the HTTP transport outside the robot.

## Verification

- Edge unit tests cover skill dispatch, cancellation, stop-on-exit, and missing-pose failure.
- Platform tests cover authenticated MCP command creation and invalid tool values.
- The deployment uses the existing cloud platform deployment script, then restarts only Edge Agent; no physical movement is triggered during verification.
