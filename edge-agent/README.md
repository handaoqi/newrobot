# RoamerX Edge Agent

Edge Agent runs on the robot, establishes an outbound MQTT 5/TLS connection to the center,
translates P0 patrol commands to ROS2 `FollowWaypoints`, and reports status, task events,
trajectory batches and alerts.

It is not the center platform and does not expose a public business HTTP server.

## Runtime dependencies

- ROS2 Humble and this workspace sourced so `robots_dog_msgs` and `nav2_msgs` are importable.
- Python packages from `requirements.txt`.
- A per-device MQTT credential or client certificate.

## Start

```bash
source /opt/ros/humble/setup.bash
source ~/genisom_roamerx_open/install/setup.bash
python3 run_edge_agent.py --config config.yaml
```

Recommended production deployment uses `systemd/roamerx-edge-agent.service`.

## P0 ROS interfaces

- subscribes `/localization_info` (`robots_dog_msgs/msg/Localization`);
- uses `/follow_waypoints` (`nav2_msgs/action/FollowWaypoints`);
- performs real Action cancellation for pause/cancel;
- reports `power.available=false` until a verified battery topic is configured.

On restart an unfinished local task becomes `interrupted`; the Agent publishes `sync.request`
and never resumes motion automatically.

## AI detector integration

The existing detector process must not publish directly to center MQTT. Integrate it locally by
calling `AlertBridge.emit_detection_alert()` with `event_type`, `severity`, `model_version` and
detection attributes. The bridge injects the current `task_execution_id`, map identity and latest
ROS map pose before publishing the protocol-v1 `alert.event`. Upload media through `MediaClient`
first and pass the returned `media_id` to the bridge. This keeps bot-version detection capability
while making Edge Agent the single business-protocol owner.

## On-robot verification

1. Confirm `ros2 action info /follow_waypoints` shows a Nav2 server.
2. Confirm `ros2 topic echo /localization_info --once` returns `robots_dog_msgs/Localization`.
3. Start a three-waypoint task and verify feedback indices 0, 1, 2.
4. Pause while moving; verify the Action becomes canceled and speed remains below the configured
   threshold for the full stop-confirmation interval.
5. Resume and confirm the new goal starts at the persisted current waypoint.
6. Disconnect cellular networking for at least 20 seconds, then restore it and verify trajectory
   outbox rows are deleted only after `trajectory.ack`.
