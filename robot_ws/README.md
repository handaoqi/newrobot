# Robot ROS2 Workspaces

This folder contains source snapshots pulled from the robot over the local network.

## Imported Workspaces

- `orin_zsibot_roamerx_lite/`: Orin workspace from `/home/jszr/zsibot_roamerx_lite`.
- `firefly_robot_navigation/`: Firefly/RK workspace from `/home/firefly/robot_navigation`.

## Sync Rules

Only source, configuration, launch files, maps metadata, and scripts should be committed here.

Do not commit:

- `build/`, `install/`, or `log/`
- ROS bag files
- point clouds or large generated maps
- local SSH keys, passwords, tokens, or access notes

If code is changed directly on the robot during field testing, pull it back into this folder, commit it, and push it so GitHub remains the source of truth.
