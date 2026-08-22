# 3588 Motion Runtime

This directory is a managed overlay on the vendor 3588 image. It contains the
charging-station package, leg-power helper source, operator wrappers and
dependency manifests. It deliberately does not replace `/opt/robot`, egg
definitions, power MCU configuration, calibration or `robot-launch`.

Deploy from NX without starting motion:

```bash
cd /home/dogrobot
deploy/3588/deploy.sh
ssh 3588 /home/firefly/dogrobot-runtime/bin/motionctl init-state
ssh 3588 /home/firefly/dogrobot-runtime/scripts/verify.sh
```

`motionctl start` changes physical robot state. Only invoke it after checking
the robot and surrounding area.

The 3588 runtime has no project database. `init-state` only creates
`/var/lib/roamerx-charge-pile/state` with `unknown` when the file is missing;
it preserves an existing charging state and does not start any motion process.
