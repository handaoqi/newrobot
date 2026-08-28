# 地图管理：RTK、全程里程计与同步诊断录制执行计划

## 1. 目标

本计划合并室外 ENU 原点锁定、RTK 实时质量展示、建图全程里程计输出和同步诊断 rosbag 录制要求。

执行目标：

1. 室外建图页面能实时说明 RTK 当前是什么解、误差多大、坐标与航向是多少、数据是否新鲜。
2. 从室外原点检查、SLAM 预热到正式建图，`/odom/localization_odom` 始终有明确且唯一的数据源。
3. 勾选“同步录制诊断数据”后，rosbag 能恢复 LiDAR、IMU、RTK、里程计和完整 TF 链。
4. RTK、里程计或 TF 缺失时，页面、状态接口和 `recording_manifest.yaml` 都能给出明确证据。

路线规划页面的定位初始化流程不在本计划修改范围内。

## 2. 室外 RTK 数据契约

原点检查继续读取：

```text
/fix
/rtk_pvh
/rtk/ntrip_status
```

Edge 原点状态必须返回：

| 字段 | 含义 |
| --- | --- |
| `ntrip_quality` | `rtk_fixed`、`rtk_float`、`standalone` 或 `invalid` |
| `fix_status` | `/fix.status.status` 原始值 |
| `solution_status` | 接收机位置解状态 |
| `position_type` | 接收机原始定位类型编号 |
| `solution_satellites` | 参与解算的卫星数 |
| `latitude/longitude/altitude` | 实时经纬高 |
| `horizontal_std_m/vertical_std_m` | 水平和垂直标准差 |
| `heading_deg/heading_std_deg` | 双天线航向及标准差 |
| `baseline_m` | 双天线基线长度 |
| `differential_age_seconds` | 差分龄期 |
| `message_time_offset_seconds` | 当前系统时间减 RTK 消息 Header 时间 |
| `data_age_seconds` | RTK/NTRIP 数据综合年龄 |
| `measurement_time_source` | RTK 消息使用的测量时间来源 |
| `last_sample_at` | Edge 最近成功采样时间 |

页面每秒随建图状态刷新，空值显示“—”，过期值显示“已过期”。60 秒锁原点门控继续要求位置 FIX、双天线航向 FIX、位置波动小于 2 cm、航向质量合格和数据未过期。

## 3. 建图全程里程计

对外统一话题为：

```text
/odom/localization_odom
```

数据源按阶段切换：

| 阶段 | 数据源 | 约束 |
| --- | --- | --- |
| 原点准备和锁定 | `localization` | 停止 Nav2，保留定位节点 |
| SLAM 预热 | FAST-LIO-SAM | 启动 SLAM 前停止定位节点 |
| 正式建图和保存 | FAST-LIO-SAM | 同时保留原始 `/slam_odom` |
| 取消/退出后 | `localization` | 后续导航启动流程恢复定位节点 |

FAST-LIO-SAM 将同一份建图里程计同时发布到 `/slam_odom` 和 `/odom/localization_odom`。任何时刻只能有一个节点发布兼容话题；定位节点与 SLAM 不得同时拥有建图 TF 链。

Edge 直接订阅 `/odom/localization_odom`，上报在线状态、频率、消息年龄、时间偏差、`frame_id`、`child_frame_id` 和数据源。

## 4. 同步诊断录制

勾选“同步录制诊断数据”后固定录制：

```text
/front_lidar
/front_lidar/imu
/fix
/rtk_pvh
/rtk/ntrip_status
/odom/localization_odom
/slam_odom
/tf
/tf_static
```

`ROSBAG_EXTRA_TOPICS` 继续用于现场临时追加话题。录制停止后生成的 manifest 必须列出全部话题、类型、消息数、缺失必需话题、录制起止时间、ROS 与系统时间偏差及 RTK 有效时间区间。

## 5. 实施清单

- [x] 扩展 `OriginSample` 和原点状态快照字段。
- [x] 地图管理室外质量卡展示解类型、误差、坐标、航向和时间诊断。
- [x] 导航脚本支持只停止 Nav2、只停止定位和完整停止三种动作。
- [x] 原点阶段保留定位节点，SLAM 启动前显式释放定位节点。
- [x] FAST-LIO-SAM 增加 `/odom/localization_odom` 兼容发布。
- [x] Edge 增加里程计直接订阅和健康明细。
- [x] 扩展 rosbag 默认话题和 recording manifest 必需话题。
- [x] 增加单元测试并完成前端生产构建、FAST-LIO-SAM 编译检查。
- [ ] 在 NX 实机完成 RTK、单发布者、九话题 rosbag 和 TF 回放验收。

## 6. 验收标准

1. 室外原点锁定页面持续刷新三个 RTK 话题的完整质量信息。
2. RTK 无信号超过 3 秒时，状态机停在原点步骤并显示明确错误。
3. 原点阶段和 SLAM 阶段执行 `ros2 topic hz /odom/localization_odom` 都有持续输出。
4. SLAM 阶段 `/slam_odom` 与 `/odom/localization_odom` 位姿和时间戳一致。
5. `ros2 topic info -v /odom/localization_odom` 任一阶段只有一个发布者。
6. rosbag metadata 包含九个固定话题，缺失话题会写入 manifest。
7. 回放 rosbag 时可重建 `map → odom → base_link → livox_frame` 关系并核对 RTK/SLAM 时间线。
8. 室内建图不因 RTK 离线而失败，室外建图不能绕过 ENU 原点门控。
