# Platform Docker Runtime

The production Compose stack includes Django API, MQTT worker, patrol scheduler,
Vue/nginx frontend, Mosquitto and ZLMediaKit. Persistent state is bind-mounted
under `runtime/platform/data`; host-specific credentials remain in `conf`.

```bash
cd /home/dogrobot/runtime/platform
sudo scripts/install-docker.sh
bin/platformctl init
# Edit conf/platform.env, then:
scripts/prepare-mqtt-password.sh
bin/platformctl init-db
bin/platformctl preflight
bin/platformctl up
bin/platformctl verify
```

Database ownership and table responsibilities are documented in
[`docs/architecture/THREE_ENDPOINT_DATA_ARCHITECTURE.md`](../../../docs/architecture/THREE_ENDPOINT_DATA_ARCHITECTURE.md).
Initialization and recovery steps are in
[`docs/operation-manual/DATABASE_INITIALIZATION_MANUAL.md`](../../../docs/operation-manual/DATABASE_INITIALIZATION_MANUAL.md).

Cloud deployment is driven from the repository-level `deploy/platform/` entry
point. It keeps source in `/opt/roamerx/source` and persistent runtime
state in `/opt/roamerx/runtime/platform`:

```bash
deploy/platform/deploy.sh
# Edit cloud conf/platform.env and generate mosquitto.passwd once.
deploy/platform/deploy.sh --start
```

`preflight` checks credentials, source files, free disk, Compose syntax and the
legacy systemd API before any container is started. Do not run the legacy
systemd and Compose platform stacks at the same time against the same database
and MQTT identity.
