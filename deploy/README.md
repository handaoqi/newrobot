# Deployment and Recovery Entrypoints

`deploy/` contains the stable repository-level entrypoints.  It does not hold
runtime state, credentials, maps, or build output.

| Target | Entrypoint | Scope |
| --- | --- | --- |
| Cloud platform | `deploy/platform/deploy.sh` | Synchronizes the cloud platform source and runtime templates. |
| NX Edge | `deploy/nx-edge/deploy.sh` | Publishes the ROS workspace and Edge Agent without copying runtime data. |
| RK3588 motion overlay | `deploy/3588-motion/deploy.sh` | Publishes only the managed charging and leg-power overlay. |
| Recovery | `deploy/backup/backup-all.sh` | Creates a recoverable backup; it is an operations tool, not a deployment target. |

Run these commands from the repository root.  Start with `--dry-run` where it
is available.  Robot navigation, mapping, charging, and any physical motion
remain subject to the safety rules in `AGENT.md` and `robot/AGENTS.md`.
