# 路径规划 RTK 状态、时间基准与地图管理执行计划

## 1. 文档目的

本计划合并路径规划页面 RTK 状态分析、传感器与避障链路分析、原始状态详情分析，以及地图管理中的定位初始化和锁定原点建议，形成一套可落地、可验收的执行方案。

目标是让系统明确回答以下问题：

1. 当前收到的 RTK 数据是什么质量、是否新鲜、是否可以参与融合。
2. 为什么页面可能显示“RTK 可用待命”，但原始解状态仍为“无效”。
3. 坐标、航向和里程计时间源为什么为空或 unavailable。
4. 地图管理如何启动传感器检查、获取 RTK 数据、锁定室外原点，并把结果交给后续定位。

本计划不默认启用控制器里程计预测，也不在锁定地图原点期间启动定位节点，避免引入未经验证的时间基准和 TF 冲突。

## 2. 当前实现与问题结论

### 2.1 RTK 数据存在四层含义

当前系统实际上有四层状态，前端需要分层显示，不能用一个“RTK 状态”覆盖全部含义：

| 层级 | 含义 | 当前来源 |
| --- | --- | --- |
| 原始观测 | GNSS 接收机实际输出的定位解、经纬度、协方差、航向和接收时间 | `/fix`、`/rtk_pvh`、`/rtk/ntrip_status` |
| 传感器健康 | 话题是否在线、频率、接收年龄、测量时间偏差 | `/sensor_health`，由 `sensor_health_monitor` 生成 |
| 定位决策 | 当前解是否满足融合门限、是否已融合、地图坐标和航向是否有效 | `/localization/decision` |
| 地图/导航初始化 | 是否加载有效 GNSS 原点、是否完成 RTK 初始位姿、当前导航是否已建立地图坐标 | 定位节点、地图管理流程和 `seed_from_rtk` |

因此，“可用待命”通常表示原始 RTK 或传感器健康层已经满足候选条件，但定位决策层尚未完成地图原点加载、初始位姿注入或融合；“解状态无效”则可能是前端直接读取了空值、枚举未统一，或定位节点在未加载 GNSS 原点时按设计返回 `invalid`，不一定说明 `/fix` 没有数据。

### 2.2 路径规划页的数据链路

`RoutePlannerPage.vue` 每约 2 秒读取机器人状态和导航状态。后端 `RobotStatusSerializer` 将 Edge 上报的 `sensors` 和 `localization.decision` 合并后返回前端。

传感器与避障链路中的 RTK 文案“`/fix` 为浮点解 · 水平误差 · 频率”主要来自：

```text
/rtk_pvh
  └─ rtk_ntrip_bridge
       ├─ /fix (sensor_msgs/NavSatFix)
       ├─ /gps/rtk
       └─ /rtk/ntrip_status
            └─ sensor_health_monitor → /sensor_health
                 └─ Edge telemetry_collector → sensors.rtk
                      └─ RobotStatusSerializer → RoutePlannerPage.vue
```

`sensor_health_monitor` 从 `/fix` 的 `status.status` 和 `position_covariance` 计算解状态、水平误差、频率和时间年龄；Edge 再根据接收时间计算 `sample_age_seconds`。当前前端 RTK 特殊展示分支使用了质量、水平误差和频率，但没有把 `sample_age_seconds` 渲染为“x.x 秒前”，这是需要补齐的显示缺口。

原始状态详情中的 RTK 状态、解状态、融合可用、地图坐标、航向、水平误差和时间偏差来自 `sensors.rtk` 与 `localization.decision` 的组合。坐标和航向为空的主要原因是：

- 未加载带 `alignment_locked` 的 `gnss_origin.yaml`，定位节点会直接返回无效 RTK 观测；
- RTK 虽有浮点解，但尚未满足融合门限或尚未注入地图初始位姿；
- `rtk_x/y` 是地图坐标，不是原始经纬度，只有原点和坐标变换有效后才有意义；
- `rtk_yaw` 还要求双天线航向解有效，单天线位置固定不等于航向有效；
- 前端对 `fixed/float` 与 `rtk_fixed/rtk_float` 两套枚举没有完全统一，可能导致原始解已有效而文案显示“无效”。

### 2.3 里程计时间源 unavailable 的含义

当前定位配置使用 `/odom/localization_odom` 作为定位输出和 Nav2 链路，控制器 `/odom/mc_odom` 的预测功能 `enable_robot_odometry_prediction` 默认关闭。因而 `odom_time_source = unavailable` 在当前配置下更多表示“控制器里程计预测未启用”，不是单独证明激光、RTK 和里程计时钟已经失步。

定位节点仍然会用激光帧时间戳与 `/fix`、RTK 航向的测量时间戳计算年龄。激光、IMU、RTK 的时间基准必须一致，否则会出现 RTK 在线但不能融合、位置跳变或航向被拒绝的问题。当前主链路应保持：

```text
/front_lidar + /front_lidar/imu
        └─ localization
             └─ /odom/localization_odom → /odom/nav2
```

控制器里程计不应在没有回放和实机验证前加入主融合链路。

## 3. 目标数据契约

后续统一使用以下字段，并明确字段的时间语义：

### 3.1 原始 RTK 快照 `raw_rtk`

- `topic`：`/fix` 或 `/rtk_pvh`；
- `quality`：统一为 `invalid`、`standalone`、`float`、`fixed`；
- `fix_status`、`solution_status`、`position_type`；
- `latitude`、`longitude`、`altitude`；
- `horizontal_std_m`、`baseline_m`、`satellites`；
- `measurement_stamp`、`received_at`、`sample_age_seconds`；
- `measurement_time_offset_ms`、`measurement_time_valid`；
- `online`、`stale`、`reason`。

### 3.2 定位决策 `localization_decision`

- `active_source`、`preferred_source`；
- `rtk_quality`、`rtk_usable`、`rtk_heading_usable`；
- `rtk_position_fused`、`rtk_heading_fused`；
- `rtk_x`、`rtk_y`、`rtk_yaw`，无效时必须返回 `null`，不能用 0 冒充有效坐标；
- `odom_time_source`、`odom_time_valid`；
- `reason` 或 `blocked_reason`，说明是原点未加载、数据过期、误差超限、航向无效还是尚未初始化。

### 3.3 时间诊断 `time_diagnostics`

至少记录激光、IMU、RTK 定位、RTK 航向和定位里程计的：

- 消息时间戳；
- Edge/ROS 接收时间；
- 当前年龄；
- 时间偏差和时间戳有效性；
- 时间戳是否单调递增；
- 以激光帧为基准的 `lidar_to_rtk_delta_ms`、`lidar_to_heading_delta_ms`。

## 4. 分阶段执行计划

### 阶段 A：统一前端状态显示

目标：让路径规划页同时显示“原始 RTK 状态”和“定位融合状态”，避免两处文案相互矛盾。

任务：

1. 在前端增加质量枚举归一化，将 `rtk_fixed/fixed`、`rtk_float/float`、`standalone`、`invalid` 映射到同一套显示值。
2. RTK 传感器行增加数据年龄：在线时显示“x.x 秒前”，超过阈值显示“数据过期 x.x 秒”；同时显示频率和水平误差。
3. 原始状态详情增加原始 `/fix` 解状态、经纬度、测量时间、接收时间和数据年龄。
4. 定位详情继续单独显示 `rtk_usable`、`rtk_position_fused`、`rtk_heading_usable` 和阻塞原因。
5. `rtk_x/y/yaw` 为空时显示“未形成地图坐标/航向”，禁止把空值或 0 显示成有效位置。
6. `odom_time_source=unavailable` 改成可理解的“控制器里程计预测未启用”；若存在实际时间异常，再显示“时间戳无效/未同步”。
7. 导航状态机的“可用待命”详情改为“原始 RTK 已满足候选条件，等待地图原点或定位初始化”，并保留真正的“无效”状态用于无数据、过期或质量不合格。

涉及文件：

- `platform/frontend/src/views/RoutePlannerPage.vue`；
- 如地图管理需要展示同一套字段，补充 `platform/frontend/src/views/MapsPage.vue`。

验收：同一份状态数据下，传感器卡片、导航状态机和原始详情的质量、年龄和原因一致；能区分“RTK 有数据但未融合”和“RTK 数据无效”。

### 阶段 B：补齐 Edge/API 状态契约

目标：在 Edge 到云平台的状态快照中保留足够的原始信息和诊断信息，前端不再依赖推测。

任务：

1. 保持现有 `sensors.rtk` 和 `localization.decision` 兼容，同时增加规范化的 `raw_rtk` 或等价字段。
2. 透传 `sample_age_seconds`、`measurement_stamp`、`received_at`、`measurement_time_offset_ms`、`measurement_time_valid`。
3. 透传 `/fix` 的质量状态、协方差水平误差、经纬度，以及 `/rtk_pvh` 的航向质量、基线和解状态；敏感的 NTRIP 凭据不得进入状态快照。
4. 在 Edge 生成 `reason/blocked_reason`，至少覆盖：未收到、数据过期、质量不满足、GNSS 原点未加载、误差超限、航向不可用、等待初始化。
5. 后端序列化时保持字段缺失与字段为 0 的区别，不能用默认空对象覆盖有效的诊断数据。

涉及文件：

- `edge-agent/roamerx_edge/ros_adapter.py`；
- `edge-agent/roamerx_edge/telemetry_collector.py`；
- `platform/backend/monitoring/serializers.py` 及相关 API 测试；
- 必要时调整 `sensor_health_monitor.cpp` 的 JSON 字段。

验收：断开 `/fix`、延迟 `/fix`、切换浮点/固定解时，API 能分别表达在线、质量、年龄和阻塞原因；前端刷新期间不丢失上一次有效的测量时间和当前接收年龄。

### 阶段 C：完成激光、IMU、RTK、里程计时间审计

目标：用可量化的时间诊断确认“不能融合”是质量问题还是时间问题。

任务：

1. 采集并检查 `/front_lidar`、`/front_lidar/imu`、`/fix`、`/rtk_pvh`、`/odom/localization_odom` 和 `/odom/mc_odom` 的消息时间戳与接收时间。
2. 以激光帧时间为定位参考，检查 `/fix` 和双天线航向是否在定位节点允许的最大年龄内。
3. 增加时间戳有效性、单调性、ROS epoch/设备 uptime 混用、接收延迟和跨话题时间差告警。
4. 对 `/odom/mc_odom` 单独标记其控制器 uptime/non-monotonic 特性；在 `enable_robot_odometry_prediction=false` 时不把它作为主定位时间源。
5. 如果未来必须启用控制器里程计预测，使用 ROS 接收时间的单调间隔，并在回放和实机测试通过后再开放配置。

验收建议：

- 正常链路下 `lidar_to_rtk_delta_ms`、`lidar_to_heading_delta_ms` 在设定阈值内且连续稳定；
- 旧数据或时间戳异常时，UI 明确显示过期/时间无效，定位决策拒绝融合；
- `odom_time_source` 的文案能区分“预测未启用”和“时间戳实际异常”。

### 阶段 D：规范定位初始化和 RTK 融合

目标：明确“获得 RTK 话题数据”和“建立地图坐标”是两个不同动作。

任务：

1. 定位节点启动后先检查 `gnss_fusion.enable`、`/fix`、`/rtk_pvh` 和 `gnss_origin.yaml`。
2. 只有原点文件存在且 `alignment_locked >= 0.5` 时，才允许把 RTK 观测转换为地图坐标并参与融合。
3. 原始 RTK 有效但原点未加载时，继续展示原始经纬度和质量，但定位决策返回“原点未加载”，`rtk_x/y/yaw` 保持空值。
4. 地图加载完成后，导航初始化调用 `seed_source: rtk` 或 `/localization/seed_from_rtk`，等待定位节点报告初始位姿有效，再进入可导航状态。
5. `rtk_yaw` 只有双天线航向状态、基线和航向精度同时满足门限时才显示有效；位置固定不代表航向已经有效。
6. 把 `fixed/float` 的规范化放在数据边界或定位决策层，避免前端直接猜测枚举。

验收：在无原点、原点已加载但未注入、注入成功三种状态下，页面分别显示不同的状态和原因；不会因为 `/fix` 有数据就错误地显示已完成导航初始化。

### 阶段 E：地图管理 RTK 预检查、锁定原点和启动定位

目标：让地图管理能独立完成室外原点准备，同时不与 SLAM/定位 TF 链路冲突。

流程：

```text
地图管理进入室外建图
  → 启动并检查（prepare_only）
  → 确认 LiDAR/IMU/RTK 传感器在线
  → 读取 /fix、/rtk_pvh、/rtk/ntrip_status
  → 固定解、时间新鲜、航向有效
  → 锁定原点（连续质量窗口）
  → 写入 gnss_origin.yaml
  → 启动 SLAM 建图/完成地图包
  → 下次启动加载地图和原点
  → 启动 localization
  → seed_from_rtk 建立地图初始位姿
  → 导航状态进入可用
```

执行要求：

1. “启动并检查”继续使用 `startRobotMappingOrigin(..., prepare_only: true)`，负责启动/确认建图传感器，不立即锁定原点。
2. 检查页直接展示 `/fix`、`/rtk_pvh`、`/rtk/ntrip_status` 的质量、水平误差、航向、基线、频率、测量时间和“x 秒前”。
3. 锁定原点前必须阻止 SLAM 或导航栈并发占用冲突 TF；原点采样由 `mapping_adapter` 的 origin monitor 完成。
4. 锁定窗口使用现有连续采样和稳定性门限：固定解、有效航向、时间不过期、水平误差合格、位置散布小于 2 cm，并持续约 60 秒。
5. 锁定成功后生成带 `alignment_locked` 的 `gnss_origin.yaml`，并在地图详情中显示原点状态、经纬度、航向、样本数和锁定时间。
6. 原点锁定不等于定位已经启动。地图包激活后，按阶段 D 启动定位并执行 RTK 初始位姿注入。
7. 室内地图不强制依赖 RTK 原点，使用本地定位/NDT；室外或室内外切换地图才强制要求有效 GNSS 原点。

涉及文件：

- `platform/frontend/src/views/MapsPage.vue`；
- `edge-agent/roamerx_edge/mapping_adapter.py` 及 origin monitor；
- `robot/script/rtk_ntrip_bridge.py`；
- `robot/src/localization/localization/apps/localization_nodelet.cpp`；
- `robot/src/localization/localization/config/config.yaml`。

既有原点锁定操作细节见：[MAP_ORIGIN_LOCK_SOP_PLAN.md](MAP_ORIGIN_LOCK_SOP_PLAN.md)。本计划负责把它与路径规划状态、时间诊断和定位初始化串起来，不重复维护两套门限。

### 阶段 F：测试、部署与实机验收

测试场景至少包括：

1. 无 `/fix`；
2. 单点解 `standalone`；
3. 浮点解 `float`；
4. 固定解 `fixed`；
5. `/fix` 过期；
6. header 时间无效、设备 uptime 与 ROS 时间混用；
7. 原始 RTK 有效但未加载原点；
8. 已加载原点但尚未 `seed_from_rtk`；
9. 原点锁定并成功启动定位；
10. 双天线航向无效或基线不合格；
11. 控制器里程计预测关闭；
12. 室内地图不依赖 RTK，室外地图依赖 RTK 原点。

验收分为三层：

- 单元/API：枚举归一化、字段缺失、时间年龄、过期判断、序列化结果；
- ROS/回放：话题时间戳、融合门限、原点加载、`seed_from_rtk` 和定位决策；
- 实机：断链、恢复、浮点转固定、原点锁定、地图重启和导航启动全流程。

部署时必须同时更新 Edge、定位节点、前端和相关配置，重启后检查实际运行版本；如果修改了 systemd 服务或安装包，记录服务状态、日志摘要和回滚版本。

## 5. 建议实施顺序

1. 先冻结并补齐 Edge/API 数据契约和质量枚举；
2. 再修改路径规划页、原始详情和地图管理页的显示；
3. 加入时间诊断并用 rosbag/现场数据验证阈值；
4. 接通地图管理的 RTK 预检查与原点锁定结果；
5. 接通地图加载后的定位启动和 `seed_from_rtk`；
6. 完成自动化测试、实机验收、部署记录和回滚验证；
7. 最后再评估是否有必要开启控制器里程计预测。

依赖关系是：原始数据可观测 → 时间有效 → RTK 原点锁定 → 地图坐标可建立 → 定位初始化 → 导航可用。任何上游条件未满足时，下游页面必须显示具体阻塞原因，而不是统一显示“无效”。

## 6. 完成检查清单

- [ ] `/fix`、`/rtk_pvh`、`/rtk/ntrip_status`、`/sensor_health` 和 `/localization/decision` 的字段含义已统一。
- [ ] 路径规划页显示 RTK 质量、水平误差、频率、数据年龄和测量时间偏差。
- [ ] 原始 RTK 状态与定位融合状态分开显示。
- [ ] `fixed/float` 与 `rtk_fixed/rtk_float` 不再造成错误的“无效”文案。
- [ ] 坐标/航向无效时显示原因，且不以 0 冒充有效值。
- [ ] `odom_time_source` 能区分预测未启用、时间有效和时间异常。
- [ ] 激光、IMU、RTK 和定位里程计时间差有可观测字段和告警。
- [ ] 地图管理可执行 RTK 预检查，并显示“x 秒前”。
- [ ] 室外原点锁定成功后生成并校验 `gnss_origin.yaml`。
- [ ] 地图重启后能加载原点、启动定位并执行 `seed_from_rtk`。
- [ ] 室内流程不被不必要的 RTK 原点条件阻塞。
- [ ] 自动化、回放和实机验收记录齐全，部署版本可回滚。

## 7. 相关文件与文档

- [导航传感器说明](NAVIGATION_SENSOR_DESCRIPTION.md)
- [地图原点锁定操作计划](MAP_ORIGIN_LOCK_SOP_PLAN.md)
- [巡检任务与路径规划架构](INSPECTION_TASK_ROUTE_ARCHITECTURE.md)
- [SLAM 采集与世界位姿计划](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md)
- [SLAM 采集与世界位姿执行进度](SLAM_DATA_CAPTURE_AND_WORLD_POSE_EXECUTION.md)
