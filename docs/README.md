# RoamerX 项目文档

`/home/dogrobot/docs` 是项目架构分析和人工操作手册的统一入口。源码目录内的 README 只保留组件级说明，并链接到这里；不要在多个目录维护内容不同的同名手册。

## 架构分析

| 文档 | 内容 |
| --- | --- |
| [导航传感器说明](NAVIGATION_SENSOR_DESCRIPTION.md) | LiDAR、IMU、RTK、里程计、TF 与 Nav2 数据链 |
| [三端数据架构](architecture/THREE_ENDPOINT_DATA_ARCHITECTURE.md) | 云平台、NX Edge、3588 的存储边界、表职责和初始记录 |
| [SLAM 采集与世界位姿计划](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md) | 建图话题、完整世界位姿、IMU 预积分、Scan-Context 与无 RTK 场景限制 |
| [SLAM 采集与世界位姿执行进度](SLAM_DATA_CAPTURE_AND_WORLD_POSE_EXECUTION.md) | 上述计划的任务拆分、实现决策和验收记录 |

## 操作手册

| 文档 | 内容 |
| --- | --- |
| [云平台操作手册](operation-manual/ROAMERX_CLOUD_PLATFORM_OPERATION_MANUAL.md) | 值守、监测、机器人、任务、地图、路线和远程控制 |
| [数据库初始化手册](operation-manual/DATABASE_INITIALIZATION_MANUAL.md) | 三端建库、初始化、验收、备份和恢复 |

## 文档规则

1. 架构原理、跨端数据流和设计决策放在 `docs/architecture/`。
2. 可执行的安装、初始化、操作和故障恢复步骤放在 `docs/operation-manual/`。
3. 截图只放在对应手册的 `screenshots/` 子目录。
4. 密码、Token、私钥、NTRIP 凭据和真实配置文件不得写入文档或提交 Git。
5. 运行逻辑变更时，同一提交必须更新相关文档和验收步骤。
