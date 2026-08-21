# Runtime Deployment Result

Updated: 2026-08-21

## Layout

All three deployment targets use the same six directories:

```text
runtime/{platform,nx-edge,3588-motion}/
  bin/  data/  conf/  scripts/  docs/  install/
```

## NX Edge

The active NX runtime is `/home/dogrobot/runtime/nx-edge`. The migration moved:

| Asset | Runtime destination | Approximate size |
| --- | --- | ---: |
| Maps and map replay | `data/jszr` | 15 GB |
| Mapping/navigation bags | `data/rosbags` | 18 GB |
| ASR environment and models | `data/voice` | 4.9 GB |
| Patrol recordings | `data/patrol-data` | 1.8 GB |
| ROS logs/state | `data/ros-home` | 3.0 GB |
| Vendor parameters/logs | `data/robot-state` | 301 MB |
| Edge database/state | `data/edge-agent` | 22 MB |
| Codex main sessions | `data/codex/sessions` | Runtime-dependent |
| Genisom L1 SDK | `install/genisom_l1_sdk` | 17 MB |

Legacy paths under `/home/robot` are compatibility symlinks. Edge Agent,
Dev Agent, vision, local ASR, MCP and teleop services remain enabled and active.
The current Codex main session is also retained under runtime data with a
compatibility link from the original Codex session path. See
`nx-edge/docs/CODEX_SESSION_BACKUP.md` for restore instructions.

## 3588 Motion

The managed overlay is deployed to `/home/firefly/dogrobot-runtime`. It contains
the charging vendor bundle and leg-power build assets while retaining the
vendor image's `robot-launch`, eggs, calibration and MCU configuration. The
deployment performed status-only verification and did not start motion.

## Cloud Platform

The Docker runtime is staged at `/opt/roamerx/runtime/platform`; source build
contexts are staged at `/opt/roamerx/source/platform`. Docker Compose resolves
the six services successfully. The current systemd production stack remains
active until a separately approved cutover migrates persistent data and stops
the old writers.
