# Project Directory Map

This repo is split into three working layers:

- `dog_mvp_platform/`: web control panel and backend bridge.
- `robot_ws/orin_zsibot_roamerx_lite/`: Orin-side ROS2 stack for SLAM, localization, navigation, and patrol.
- `robot_ws/firefly_robot_navigation/`: Firefly-side experimental navigation and patrol code.
- `robot_ws/vendor/`: third-party dependencies and vendor snapshots, not for day-to-day edits.

```mermaid
mindmap
  root((New project))
    dog_mvp_platform
      server.py
      scripts
        dog_sdk_bridge.py
        nav_cmdvel_sdk_bridge.py
      static
        index.html
        app.js
        app.css
    robot_ws
      orin_zsibot_roamerx_lite
        map
        script
        src
          interface
          localization
          navigation
          slam
      firefly_robot_navigation
        launch
        scripts
        src
          robot_navigation_core
          robot_patrol
      vendor
    docs
      COLLABORATION.md
      PATROL_TASK_PLAN.md
      PROJECT_DIRECTORY_MAP.md
```

## Where To Work

- SLAM and map saving: `robot_ws/orin_zsibot_roamerx_lite/src/slam/`
- Localization: `robot_ws/orin_zsibot_roamerx_lite/src/localization/`
- Navigation core: `robot_ws/orin_zsibot_roamerx_lite/src/navigation/`
- Patrol task logic: `robot_ws/orin_zsibot_roamerx_lite/src/navigation/` or a new `park_patrol` package under that tree
- Web UI and API: `dog_mvp_platform/`
- Early testing and lower-level experiments: `robot_ws/firefly_robot_navigation/`

## Recommended Editing Rule

Keep vendor code, generated build folders, logs, and map artifacts out of normal edits. Main feature work should stay in the Orin workspace and the web control layer.
