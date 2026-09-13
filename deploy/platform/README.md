# Cloud Platform Deployment

This is the canonical cloud-platform deployment entrypoint.

```bash
cd /home/dogrobot
deploy/platform/deploy.sh --dry-run
deploy/platform/deploy.sh --start
```

It synchronizes platform source and runtime templates to the configured cloud
host while preserving remote runtime data and credentials.  `--start` runs the
managed platform controller after synchronization.  The legacy systemd release
script at `platform/scripts/deploy_cloud_platform.sh` remains available for the
existing cloud layout; do not run it concurrently with the Docker deployment.
