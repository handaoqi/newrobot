# DogRobot

DogRobot is the unified repository for the RoamerX robot runtime, cloud
platform, edge control, and deployment tooling.

## Repository Layout

- `robot/`: ROS 2 Humble SLAM, localization, navigation, messages, and
  real-robot scripts for the NX computer.
- `edge-agent/`: MQTT command, telemetry, map, charging, audio, and power-mode
  agent deployed on each robot.
- `dev-agent/`: optional remote development agent.
- `platform/backend/`: Django API, device worker, scheduler, and persistence.
- `platform/frontend/`: Vue operator interface.
- `platform/bot-version/`: video inference, streaming, and alert reporting.
- `deploy/`: stable entry points for robot, cloud, and RK3588 deployment.

Runtime data and credentials are intentionally excluded. Maps live under
Runtime data is stored under `/home/dogrobot/runtime/nx-edge`, with maps in
`/home/dogrobot/runtime/nx-edge/data/jszr/map` and rosbags in
`/home/dogrobot/runtime/nx-edge/data/rosbags`. Cloud state remains on the
cloud host under `/opt/roamerx/shared`.

## Common Commands

Build the robot workspace:

```bash
cd robot
./build.sh all
```

Test the cloud platform:

```bash
cd platform/backend && python3 manage.py test monitoring
cd ../frontend && npm test && npm run build
```

Test both agent packages:

```bash
scripts/test_agents.sh
```

Deploy using the stable wrappers:

```bash
deploy/nx-edge/deploy.sh --host robot@ROBOT_IP --build
deploy/platform/deploy.sh
deploy/3588/deploy.sh
deploy/backup/backup-all.sh
```

The canonical branch is `main`. Pre-consolidation snapshots are retained as
the annotated tags `archive/legacy-main-20260812`,
`archive/robot-20260812`, and `archive/platform-20260812`.
