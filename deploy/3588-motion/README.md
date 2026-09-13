# RK3588 Motion Overlay Deployment

This entrypoint deploys only repository-managed assets for the RK3588 motion
overlay.  It does not copy the NX ROS workspace, Edge Agent, vendor firmware,
SDK credentials, or the full controller image.

```bash
cd /home/dogrobot
deploy/3588-motion/deploy.sh --dry-run
deploy/3588-motion/deploy.sh
```

The target is `firefly@192.168.234.1` by default and its runtime root is
`/home/firefly/dogrobot-runtime`.  The release includes the managed motion
commands, verification scripts, charge-pile assets, and leg-power tool only.

Deployment does not authorize charging or robot motion.  Follow the safety
constraints in `AGENT.md` and the 3588 section of
`PROJECT_DEPLOYMENT_MANUAL.md` before starting any controller-side service.
