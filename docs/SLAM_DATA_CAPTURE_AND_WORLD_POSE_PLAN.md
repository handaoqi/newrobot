# SLAM 完整数据采集与世界位姿闭环计划

更新时间：2026-08-21

## Summary

补齐建图所需采集数据、完整世界位姿、IMU 预积分和 Scan-Context 回环优化，并增加无 RTK 原点地图的场景限制：

- 有 RTK 原点：允许室内、室外和室内外过渡区使用。
- 无 RTK 原点：只能标记为室内 `local_only` 地图，只允许室内 NDT 匹配定位。
- 无 RTK 原点地图不得选择室外或过渡区，也不得进入依赖绝对坐标的定位模式。

## 建图采集话题

### 必须采集

| 话题 | 作用 |
|---|---|
| `/front_lidar` | 原始三维 LiDAR 点云，生成关键帧点云和地图 |
| `/front_lidar/imu` | LiDAR 内置 IMU，进行状态估计和预积分 |
| `/tf_static` | LiDAR、IMU、GPS 外参和固定坐标关系 |
| `/fix` | GNSS/RTK 经纬高、状态和精度 |
| `/rtk_pvh` | 双天线 RTK 航向、基线、解状态和航向精度 |

当前采用轻量建图采集模式，标准 rosbag 只录制：

```text
/front_lidar
/front_lidar/imu
/fix
/rtk_pvh
/tf_static
```

建图状态估计优先使用 LiDAR 内置 IMU；`/odom/mc_odom` 不作为建图采集依赖，也不进入标准建图 rosbag。动态 `/tf`、SLAM 输出和日志仅在故障诊断时通过额外话题开启。

### 诊断模式可选采集

| 话题 | 作用 |
|---|---|
| `/slam_odom` | 记录 SLAM 输出位姿，便于离线复盘 |
| `/odom/localization_odom` | 记录定位输出和 Nav2 使用的里程计 |
| `/odom/mc_odom` | 控制器里程计，仅作为诊断和运动约束，不作为地图真值 |
| `/tf` | 动态坐标变换，便于诊断坐标链和外部节点关系 |
| `/world_points` | 记录 SLAM 生成的世界坐标点云，便于调试 |
| `/rosout` | 记录建图、RTK、回环和异常日志 |

其中 `/world_points` 是由原始 LiDAR 和 SLAM 位姿派生的点云，默认不录制；`/rosout`、`/tf`、`/slam_odom` 和 `/odom/mc_odom` 只在诊断模式启用。

`/laser_scan`、`/map` 等派生话题不作为原始建图数据的替代品；地图应以三维 LiDAR、IMU、`/tf_static` 和结构化关键帧数据为准。

录制目录必须生成 `recording_manifest.yaml`，记录：

- 话题清单和消息类型
- QoS 配置
- 起止时间
- 消息数量和丢包统计
- ROS 时间与系统时间偏移
- RTK 有效时间区间

## 地图坐标模式与场景约束

新增地图元数据：

```yaml
schema_version: 2
coordinate_mode: rtk_fixed    # rtk_fixed 或 local_only
scene_scope: indoor            # indoor、transition、outdoor
localization_mode: ndt         # ndt 或 rtk_ndt
origin_status: fixed           # fixed 或 local_only
rtk_origin_required: true
```

### RTK 原点有效地图

只有满足以下条件时才能标记为 `rtk_fixed`：

- RTK 状态达到有效固定解。
- 双天线航向有效，基线和航向精度合格。
- RTK 与 LiDAR/IMU 轨迹完成对齐。
- 对齐样本数和 RMS 误差达到阈值。
- `gnss_origin.yaml` 完整生成。

允许选择：

```text
indoor
transition
outdoor
```

可使用：

```text
NDT
RTK + NDT
```

### 无 RTK 原点地图

RTK 不可用时仍允许完成 LiDAR+IMU 建图，但必须标记：

```yaml
coordinate_mode: local_only
origin_status: local_only
scene_scope: indoor
localization_mode: ndt
rtk_origin_required: false
```

强制限制：

- `scene_scope` 只能是 `indoor`。
- 禁止选择 `transition`。
- 禁止选择 `outdoor`。
- 禁止选择 `rtk_ndt`。
- 禁止在平台地图编辑器中将其绑定到室外或过渡区任务。
- Edge Agent 激活地图时再次校验，不能只依赖前端限制。
- 若任务、地图或区域配置要求 RTK 原点，直接拒绝执行并返回明确错误码。

建议错误码：

```text
MAP_RTK_ORIGIN_REQUIRED
MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN
MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN
MAP_LOCAL_ONLY_NDT_ONLY
```

平台和 Edge Agent 的场景选择必须使用同一套枚举和校验逻辑，避免前端允许选择但机器人侧拒绝不一致。

## 关键帧和世界位姿

每个关键帧保存：

```text
index
timestamp
world_x/y/z
world_qx/qy/qz/qw
lidar_x/y/z
lidar_qx/qy/qz/qw
point_count
point_cloud_file
rtk_valid
rtk_status
rtk_latitude/longitude/altitude
rtk_horizontal_std
rtk_heading_valid
preintegration_file
scan_context_index
```

同时保存：

```text
keyframes/scan_XXXXX.pcd
imu_preintegration/preint_XXXXX.json
```

世界位姿使用 `map` 坐标系和 ROS 四元数顺序：

```text
qx, qy, qz, qw
```

保留 `trajectory_raw.csv` 和 `trajectory_optimized.csv`，明确地图最终使用的轨迹来源。

### 完整四元数世界位姿的作用

完整世界位姿由位置和姿态组成：

```text
位置：x, y, z
姿态：qx, qy, qz, qw
```

姿态使用 ROS 顺序 `qx, qy, qz, qw`，四元数必须归一化；`q` 与 `-q` 表示同一个旋转。

相比只保存 `yaw`，完整四元数可以：

- 保存机器人在坡面、台阶和不平地面上的滚转、俯仰和航向。
- 避免欧拉角万向节锁问题。
- 将关键帧 LiDAR 点云从传感器坐标系准确变换到 `map` 世界坐标系。
- 与 IMU 预积分旋转增量 `ΔR` 直接关联。
- 为 Scan-Context 回环约束和位姿图优化提供完整姿态变量。
- 支持三维轨迹回放、姿态插值和地图重新拼接。
- 对接 ROS `geometry_msgs/Pose` 和 `nav_msgs/Odometry` 的 orientation 字段。

世界位姿必须明确 `frame_id=map`，并同时保存 LiDAR 传感器位姿，避免外参和坐标系信息丢失。

## IMU 预积分和 Scan-Context

IMU 预积分每个关键帧保存：

- 起止时间和 `delta_t`
- `ΔR`
- `Δv`
- `Δp`
- 线性化加速度/陀螺仪偏置
- 15×15 协方差
- IMU 样本数

预积分文件采用逐关键帧 JSON 格式：

```json
{
  "schema_version": 1,
  "keyframe_index": 0,
  "start_timestamp": 0.0,
  "end_timestamp": 0.0,
  "delta_t": 0.0,
  "delta_rotation": [1.0, 0.0, 0.0, 0.0],
  "delta_velocity": [0.0, 0.0, 0.0],
  "delta_position": [0.0, 0.0, 0.0],
  "linearized_accel_bias": [0.0, 0.0, 0.0],
  "linearized_gyro_bias": [0.0, 0.0, 0.0],
  "covariance_15x15": [],
  "imu_sample_count": 0
}
```

预积分区间定义为前一关键帧结束时间到当前关键帧时间。时间回退、IMU 缺帧或空积分区间必须记录错误状态，不得生成伪造数据。

Scan-Context 保存：

```text
scan_context/descriptors.bin
scan_context/ring_keys.bin
scan_context/sector_keys.bin
scan_context/index.json
scan_context/loop_candidates.csv
```

首期固定参数：

```text
rings = 20
sectors = 60
max_radius_m = 80.0
descriptor = max-height
candidate_top_k = 5
min_keyframe_gap = 30
yaw_search_steps = 60
```

每个关键帧保存描述子、Ring Key、Sector Key 和索引；检索结果记录候选帧、相似度、估计 yaw、几何验证结果和最终接受状态。

回环流程：

```text
关键帧点云
→ Scan-Context 检索
→ 点云几何验证
→ 回环约束
→ 位姿图优化
→ 更新完整世界位姿
→ 重建最终地图
```

无有效回环或优化失败时，保留原始轨迹和地图，不影响地图可用性。

回环优化保留以下产物：

```text
trajectory_raw.csv
trajectory_optimized.csv
map_raw.pcd
map.pcd
```

只有同时通过 Scan-Context 相似度和点云几何验证的候选才能进入位姿图。优化失败、回环残差过大或轨迹发生异常跳变时，丢弃该回环并继续使用 raw 结果。

## 地图包结构

新地图包采用以下结构：

```text
map_package/
├── map.yaml
├── map.pgm
├── map.pcd
├── gnss_origin.yaml
├── keyframes/
├── imu_preintegration/
├── scan_context/
├── recording_manifest.yaml
├── trajectory_raw.csv
├── trajectory_optimized.csv
└── map_manifest.json
```

`map_manifest.json` 至少记录：

```json
{
  "schema_version": 2,
  "coordinate_mode": "rtk_fixed",
  "origin_status": "fixed",
  "keyframe_count": 0,
  "point_cloud_count": 0,
  "preintegration_count": 0,
  "scan_context_count": 0,
  "loop_closure_count": 0,
  "trajectory_source": "optimized",
  "raw_recording": "mapping.mcap"
}
```

旧地图缺少新文件时标记为 `legacy_incomplete`，保持兼容读取但不静默伪造新数据。

## 接口和验证

地图 manifest 增加：

```json
{
  "coordinate_mode": "local_only",
  "scene_scope": "indoor",
  "localization_mode": "ndt",
  "origin_status": "local_only",
  "keyframe_count": 0,
  "preintegration_count": 0,
  "scan_context_count": 0,
  "loop_closure_count": 0
}
```

测试覆盖：

- 必需话题和录制 manifest 完整性。
- RTK fixed、RTK invalid、RTK 缺失三种建图场景。
- 无 RTK 原点地图只能选择室内。
- 前端、后端、Edge Agent 三层均拒绝室外/过渡区绑定。
- `local_only` 地图只能使用 NDT 定位。
- RTK 原点地图可使用 RTK+NDT。
- 关键帧点云、完整四元数、时间戳、预积分和 Scan-Context 索引一一对应。
- 回环优化失败时不破坏 raw 地图。
- 旧地图保持兼容读取，并标记为 `legacy_incomplete`。
- 新建地图完成后重新加载，验证场景限制和定位模式仍然有效。

### 现场验收顺序

1. 启动 RTK、LiDAR、IMU 和建图录制。
2. 检查 `/fix` 和 `/rtk_pvh` 的实际状态、时间戳和有效性。
3. 启动建图并确认关键帧持续生成。
4. 检查每个关键帧的 PCD、完整四元数位姿、时间戳、RTK 快照和 IMU 预积分。
5. 检查 Scan-Context 候选、几何验证和回环接受结果。
6. 执行位姿图优化，确认 raw 与 optimized 轨迹均保留。
7. 保存完整地图包并重新加载。
8. 验证 RTK 原点、坐标模式和定位模式。
9. 使用无 RTK 模式重复测试，确认地图只能选择室内 NDT 定位。

## 交付验收标准

新建地图只有在以下条件满足时才可标记为完整：

- `recording_manifest.yaml` 存在且话题清单完整。
- 关键帧数量、点云数量、预积分数量和 Scan-Context 数量一致。
- 所有关键帧时间戳严格递增。
- 所有四元数已归一化，且明确属于 `map` 坐标系。
- 每个关键帧都能关联点云、预积分和 Scan-Context 索引。
- RTK 原点状态与 `coordinate_mode` 一致。
- `local_only` 地图的场景范围只能为 `indoor`。
- 回环优化失败不会覆盖 raw 轨迹和 raw 地图。
- 地图重新加载后，平台和 Edge Agent 的场景限制保持一致。

## 假设与默认决策

- 只保证新建地图生成完整采集数据，历史地图不强行补造。
- 无 RTK 原点时允许建图，但只能作为室内 `local_only` 地图。
- 室外和过渡区必须具备有效 RTK 原点。
- `NDT` 是室内地图默认定位模式，`RTK+NDT` 是具备 RTK 原点地图的增强模式。
- 原始 rosbag/MCAP 保留在 NX，地图包保存结构化关键帧、预积分和 Scan-Context 数据。
