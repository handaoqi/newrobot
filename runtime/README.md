# RoamerX Runtime Layout

`/home/dogrobot` is both the canonical Git checkout and the runtime root. Source
code remains in the top-level project directories; deployable state is isolated
under `runtime/`.

| Runtime | Purpose | Canonical host path |
| --- | --- | --- |
| `platform/` | Cloud API, worker, scheduler, frontend, MQTT and media services | `/opt/roamerx/runtime/platform` after deployment |
| `nx-edge/` | NX ROS, Edge Agent, models, maps, bags and device configuration | `/home/dogrobot/runtime/nx-edge` |
| `3588-motion/` | Managed 3588 overlays for motion, charging and leg-power tools | `/home/firefly/dogrobot-runtime` after deployment |

Every runtime has the same contract:

- `bin/`: stable operator entrypoints.
- `data/`: mutable state and large data; ignored by Git.
- `conf/`: secrets and host-specific configuration; only examples are tracked.
- `scripts/`: installation, migration, deployment and verification tools.
- `docs/`: runtime-specific operations documentation.
- `install/`: dependency manifests, models and external vendor installation packages.

Never commit real credentials, databases, maps, rosbag files or generated logs.
