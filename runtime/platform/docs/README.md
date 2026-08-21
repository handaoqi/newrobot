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
bin/platformctl preflight
bin/platformctl up
bin/platformctl verify
```

Cloud deployment keeps source in `/opt/roamerx/source` and persistent runtime
state in `/opt/roamerx/runtime/platform`:

```bash
runtime/platform/scripts/deploy-cloud.sh
# Edit cloud conf/platform.env and generate mosquitto.passwd once.
runtime/platform/scripts/deploy-cloud.sh --start
```

`preflight` checks credentials, source files, free disk, Compose syntax and the
legacy systemd API before any container is started. Do not run the legacy
systemd and Compose platform stacks at the same time against the same database
and MQTT identity.
