# RoamerX Robot MCP

The server exposes the platform remote-control command path as MCP tools. It does not publish ROS commands itself.

```bash
python3 -m pip install -r mcp_server/requirements.txt
export ROAMERX_PLATFORM_URL=http://39.107.250.69:8088
export ROAMERX_PLATFORM_TOKEN='<platform-token>'
# Optional for a single-robot installation; explicit robot_id still overrides it.
export ROAMERX_DEFAULT_ROBOT_ID='<positive-platform-robot-id>'
python3 mcp_server/roamerx_robot_mcp.py --transport stdio
```

For cloud Streamable HTTP, run the same program with `--transport streamable-http` in a service whose token is stored only in its environment. The underlying API requires `Authorization: Token <token>` and routes through MQTT and Edge Agent.

Available tools: `robot_list_platform_devices`, `robot_direction`, `robot_speed`, `robot_action`, `robot_skill_run`, `robot_skill_status`, `robot_skill_cancel`, `robot_person_detection`, `robot_person_detection_status`, `robot_person_follow`, `robot_person_follow_status`, `robot_person_follow_stop`, and `robot_remote_control_capabilities`.

`robot_list_platform_devices` calls the platform's read-only device inventory endpoint with the MCP service token. It returns only fields needed for selection: platform ID, code, name, location, connection/status, and battery level. It never sends a robot command; `default_robot_id` is included when the service has a valid `ROAMERX_DEFAULT_ROBOT_ID` binding.

`robot_skill_run` supports presets and a structured `steps` array. An AI client may store the original natural-language request in `description`, but must turn it into explicit steps before it reaches the robot. It waits up to 10 seconds, then returns the asynchronous command ID.

`robot_person_follow` starts a persistent follow session on the Edge Agent. The MCP server never performs the vision/velocity loop itself: the dog consumes a local atomic detection snapshot and fails closed if it becomes stale, the target disappears, or an obstacle is too close. Use `robot_person_follow_status` and `robot_person_follow_stop` to inspect or end the session.

For user-facing capability questions, call `robot_remote_control_capabilities`. It returns only released skills; planned functions are documented separately in `docs/roamerx_mcp_skills_plan.md`.

## Default robot binding

Every robot-specific tool accepts an optional `robot_id`.  When it is omitted, the server reads the positive numeric platform ID in `ROAMERX_DEFAULT_ROBOT_ID`; an explicit tool argument takes precedence.  Set that variable in the MCP service environment (alongside `ROAMERX_PLATFORM_TOKEN`) rather than in source control.  If neither is present, the command is rejected before it reaches the platform.

The companion `roamerx-basic-teleop` Codex plugin uses the local Streamable HTTP endpoint at `http://127.0.0.1:8095/mcp`, so a deployment can bind one robot without including its identifier in the plugin.
