# RoamerX 项目文档

`/home/dogrobot/docs` 是项目架构分析和人工操作手册的统一入口。源码目录内的 README 只保留组件级说明，并链接到这里；不要在多个目录维护内容不同的同名手册。

## 架构分析

| 文档 | 内容 |
| --- | --- |
| [导航传感器说明](NAVIGATION_SENSOR_DESCRIPTION.md) | LiDAR、IMU、RTK、里程计、TF 与 Nav2 数据链 |
| [三端数据架构](architecture/THREE_ENDPOINT_DATA_ARCHITECTURE.md) | 云平台、NX Edge、3588 的存储边界、表职责和初始记录 |
| [系统总体架构](ROAMERX_SYSTEM_ARCHITECTURE.md) | 云平台、前端、Edge 和运动控制的组件边界与业务链路 |
| [前端总体架构](前端总体架构.md) | 前端技术栈、模块/路由/API/状态、异步视频、播放器生命周期、响应式、测试、部署与安全约束 |
| [巡检任务与路径规划架构](INSPECTION_TASK_ROUTE_ARCHITECTURE.md) | 地图、路线、任务、执行和统一机器狗图示 |
| [Navigo 与标准 Nav2 双栈回退方案](NAVIGO_TO_STANDARD_NAV2_DUAL_STACK_FALLBACK_PLAN.md) | 自研与官方组件差异、双栈切换、自动回退、安全门槛和分阶段验收 |
| [统一定位、FAST-LIO2 收敛与 Nav2 Lifecycle 合并计划](NAVIGATION_LIFECYCLE_LOCALIZATION_STATE_MACHINE_PLAN.md) | NDT 优先定位、FAST-LIO2 收敛门、连续 status=3 最终验收、Nav2 分组激活和路径规划页双时间线 |
| [路径规划 RTK 与地图管理执行计划](ROUTE_PLANNER_RTK_AND_MAPPING_EXECUTION_PLAN.md) | RTK 状态、时间基准、定位初始化、原点锁定和实机验收 |
| [地图管理 RTK、里程计与诊断录制计划](MAP_ORIGIN_RTK_ODOM_DIAGNOSTIC_PLAN.md) | 室外 RTK 质量、建图全程里程计源切换和同步诊断 rosbag |
| [SLAM 采集与世界位姿计划](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md) | 建图话题、完整世界位姿、IMU 预积分、Scan-Context 与无 RTK 场景限制 |
| [SLAM 采集与世界位姿执行进度](SLAM_DATA_CAPTURE_AND_WORLD_POSE_EXECUTION.md) | 上述计划的任务拆分、实现决策和验收记录 |

## 诊断与优化方案（2026-08-25）

基于实机在线采集数据的诊断结论与分阶段整改方案。各方案文档互相引用，建议按 IMU → 自愈 → 快速重定位 → 地图管线的顺序阅读和实施。

| 文档 | 内容 |
| --- | --- |
| [IMU 漂移诊断与整改方案](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) | 实测陀螺零偏 0.740 °/s；定位侧 UKF 零偏估而不用、aarch64 并行未启用、LiDAR 时延未补偿的根因链与分级整改。**P0-1 / P0-2 已落码编译，未重启生效** |
| [定位丢失恢复与自愈方案](LOCALIZATION_SELF_HEALING_PLAN.md) | 现有三层自愈梳理、六个确认缺口、恢复策略 L0–L6 分级升级建议。**三个 P0 已落码编译，未重启生效** |
| [地图管线 mcap、Foxglove 与路线预演方案](MAP_PIPELINE_MCAP_FOXGLOVE_AND_ROUTE_PREVIEW_PLAN.md) | rosbag 生命周期管理、mcap 切换、Foxglove 离线回环优化、平台端路线几何预演。**生命周期管理已实现（§1.3.1），仅跑过 dry run** |
| [快速重定位方案](FAST_RELOCALIZATION_PLAN.md) | 现有"全局重定位"实为单假设 ICP、无偏航搜索；接入建图已产出的 Scan-Context 底库做位置识别播种。离线留一法实测 103 张地图 top-5 命中 97.9 %。**已落码编译，默认关闭，未重启生效** |
| [工作区改动梳理](WORKING_TREE_TRIAGE_20260825.md) | 未提交的 117 项改动分类、四段提交拆分建议、家目录污染与 `.gitignore` 缺口 |

## 操作手册

| 文档 | 内容 |
| --- | --- |
| [云平台操作手册](operation-manual/ROAMERX_CLOUD_PLATFORM_OPERATION_MANUAL.md) | 值守、监测、机器人、任务、地图、路线和远程控制 |
| [数据库初始化手册](operation-manual/DATABASE_INITIALIZATION_MANUAL.md) | 三端建库、初始化、验收、备份和恢复 |

## Word 架构导出

| 文档 | Word 文件 |
| --- | --- |
| 数据库管理手册 V2.0 | [ROAMERX_数据库管理手册_V2.0.docx](ROAMERX_数据库管理手册_V2.0.docx) |
| 安装配置手册 V2.0 | [ROAMERX_安装配置手册_V2.0.docx](ROAMERX_安装配置手册_V2.0.docx) |
| 云平台操作手册 V2.0 | [ROAMERX_云平台操作手册_V2.0.docx](ROAMERX_云平台操作手册_V2.0.docx) |
| 系统总体架构 | [ROAMERX_SYSTEM_ARCHITECTURE.docx](ROAMERX_SYSTEM_ARCHITECTURE.docx) |
| 前端总体架构 | [前端总体架构.docx](前端总体架构.docx) |
| 巡检任务与路径规划架构 | [INSPECTION_TASK_ROUTE_ARCHITECTURE.docx](INSPECTION_TASK_ROUTE_ARCHITECTURE.docx) |

## 文档规则

1. 架构原理、跨端数据流和设计决策放在 `docs/architecture/`。
2. 可执行的安装、初始化、操作和故障恢复步骤放在 `docs/operation-manual/`。
3. 截图只放在对应手册的 `screenshots/` 子目录。
4. 密码、Token、私钥、NTRIP 凭据和真实配置文件不得写入文档或提交 Git。
5. 运行逻辑变更时，同一提交必须更新相关文档和验收步骤。
