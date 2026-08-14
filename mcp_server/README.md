# RoamerX Robot MCP

The server exposes the platform remote-control command path as MCP tools. It does not publish ROS commands itself.

```bash
python3 -m pip install -r mcp_server/requirements.txt
export ROAMERX_PLATFORM_URL=http://39.107.250.69:8088
export ROAMERX_PLATFORM_TOKEN='<platform-token>'
python3 mcp_server/roamerx_robot_mcp.py --transport stdio
```

For cloud Streamable HTTP, run the same program with `--transport streamable-http` in a service whose token is stored only in its environment. The underlying API requires `Authorization: Token <token>` and routes through MQTT and Edge Agent.

Available tools: `robot_direction`, `robot_speed`, `robot_action`, `robot_skill_run`, `robot_skill_status`, and `robot_skill_cancel`.

`robot_skill_run` supports presets and a structured `steps` array. An AI client may store the original natural-language request in `description`, but must turn it into explicit steps before it reaches the robot. It waits up to 10 seconds, then returns the asynchronous command ID.
