# NX Edge Deployment

This is the canonical NX deployment entrypoint.  It synchronizes the tracked
`robot/` and `edge-agent/` source while preserving
`runtime/nx-edge/{data,conf}`.

```bash
cd /home/dogrobot
deploy/nx-edge/deploy.sh --dry-run
deploy/nx-edge/deploy.sh --host robot@ROBOT_IP --build
```

Use `--restart-edge` only when there is no active task; the script checks the
Edge database before restarting the service.  `--install-service` installs the
managed units and required system dependencies.  This entrypoint replaces the
removed historical NX deployment wrapper.

For ROS build, service lifecycle, maps, and physical-robot safety rules, see
`robot/AGENTS.md` and `PROJECT_DEPLOYMENT_MANUAL.md`.
