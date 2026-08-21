# 3588 Motion Runtime

This directory is a managed overlay on the vendor 3588 image. It contains the
charging-station package, leg-power helper source, operator wrappers and
dependency manifests. It deliberately does not replace `/opt/robot`, egg
definitions, power MCU configuration, calibration or `robot-launch`.

Deploy from NX without starting motion:

```bash
cd /home/dogrobot
runtime/3588-motion/scripts/deploy.sh
ssh 3588 /home/firefly/dogrobot-runtime/scripts/verify.sh
```

`motionctl start` changes physical robot state. Only invoke it after checking
the robot and surrounding area.

