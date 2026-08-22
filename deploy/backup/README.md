# Three-Endpoint Backup

`backup-all.sh` creates a timestamped backup under `/home/dogrobot/backup` for
the platform, NX Edge and managed RK3588 overlay.

The standard backup contains recoverable configuration, databases, maps and
device state. It intentionally excludes build outputs, caches, historical
logs, rosbag/MCAP, media and model archives. Excluded files remain listed in
the manifest when they are present on the host.

```bash
deploy/backup/backup-all.sh --dry-run
deploy/backup/backup-all.sh
deploy/backup/verify-backup.sh /home/dogrobot/backup/YYYYMMDD_HHMMSS
```

The script does not restore directly into a live endpoint. The copied
`restore-check.sh` validates the backup in a temporary directory.
