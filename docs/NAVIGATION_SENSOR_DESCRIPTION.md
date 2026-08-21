# 导航传感器说明

更新时间：2026-08-21  
适用平台：NX_XG3588（NX 为 ROS 定位、导航和传感器计算板；3588 运行厂商运动控制镜像）

## 1. 结论与传感器清单

定位和导航使用的物理定位传感器限定为部署在 NX 上的以下三类：

| 传感器 | NX 驱动与 ROS 话题 | 定位/导航用途 |
| --- | --- | --- |
| Livox Mid-360 激光雷达 | `livox_lidar_publisher` → `/front_lidar` | NDT 地图匹配、短时 LiDAR 里程计、障碍物感知 |
| Mid-360 雷达内置 IMU | 同一 Livox 驱动 → `/front_lidar/imu` | 姿态与运动预测融合 |
| Sixents RTK（含双天线航向） | `sixents_gps_driver` → `/rtk_pvh` → `rtk_ntrip_bridge` → `/fix` | 有效 RTK 解时的位置与航向约束 |

`/laser_scan` 不是另一台二维雷达，而是 `/front_lidar` 点云转换并剔除机身点后的导航扫描。`/odom/lidar_odom` 也不是额外传感器，而是由 Mid-360 点云和内置 IMU 推导出的短时 LiDAR 里程计。

核验时，Mid-360 点云和内置 IMU 均持续发布。RTK 驱动和桥接节点已运行，但读取到 `/fix.status=-1`、卫星数为 3、双天线基线为 0，尚无有效 RTK 解；定位会按质量门限拒绝该次 RTK 修正并使用 LiDAR+IMU 工作。

## 2. 调整前的链路与问题

调整前的导航里程计链如下：

```text
3588 运动控制器
  └─ /odom/mc_odom ──> odom_to_tf_broadcaster ──> /odom/nav2、odom→base_link

Mid-360 点云 + 内置 IMU
  └─ /front_lidar、/front_lidar/imu ──> localization ──> /odom/localization_odom（map→base_link）
```

`/odom/mc_odom` 的时间戳来自控制器启动后的单调时钟，且其航向会相对 LiDAR 地图漂移。它不应作为 Nav2 的主定位里程计。此前定位模块已经关闭了将 `mc_odom` 用作 NDT 初值预测，但仍用它维持 `map→odom` TF；这会让导航执行层仍依赖低精度控制器里程计。

## 3. 本次调整设计

目标是使 Nav2 的 `/odom/nav2` 与 `odom→base_link` 都来自高精度的 Mid-360 点云匹配和内置 IMU 融合结果，而不是来自 3588 控制器里程计。

```text
Mid-360 点云 + 内置 IMU
  └─ localization
       └─ /odom/localization_odom（map→base_link）
            └─ odom_to_tf_broadcaster
                 ├─ /odom/nav2（odom→base_link，ROS 当前时间戳）
                 ├─ odom→base_link
                 └─ map→odom（恒等变换）
```

实现原则：

1. `odom_to_tf_broadcaster` 的输入切换为 `/odom/localization_odom`，输出保持 `/odom/nav2`。
2. 输入消息的 `map` 坐标被规范为 Nav2 使用的 `odom` 坐标；同时发布恒等 `map→odom`，因此位姿数值保持不变且坐标树连续。
3. 定位模块停止发布 TF，避免它与转换节点同时发布 `map→odom` 或 `odom→base_link`。
4. 定位模块在本配置下不再订阅 `mc_odom`；控制器里程计不参与定位、TF 或 Nav2 里程计链。
5. 传感器健康状态中的 `odometry` 监测目标改为 `/odom/localization_odom`，使平台展示的里程计与实际导航来源一致。

## 4. 修改范围

| 文件 | 修改内容 |
| --- | --- |
| `robot/src/localization/localization/config/config.yaml` | 关闭定位节点 TF 发布，说明 `mc_odom` 不属于定位和 Nav2 位姿链。 |
| `robot/src/localization/localization/apps/localization_nodelet.cpp` | 仅在需要 NDT 控制器预测或定位节点自有 TF 时订阅控制器里程计。 |
| `robot/src/navigation/src/robot_navigo/launch/navigation_bringup.launch.py` | 将 `/odom/nav2` 的输入改为 `/odom/localization_odom`，启用恒等 `map→odom`。 |
| `robot/src/navigation/src/robot_navigo/src/tf_publisher.cpp` | 规范 `/odom/nav2` 的 `odom`/`base_link` 帧并统一发布对应 TF。 |
| `robot/src/navigation/src/robot_navigo/src/sensor_health_monitor.cpp` | 监测定位里程计而不是控制器里程计。 |

## 5. 验收标准与回退

修改后应验证：

1. `/odom/nav2` 的发布者是 `odom_to_tf_broadcaster`，其 `header.frame_id` 为 `odom`、`child_frame_id` 为 `base_link`，且时间戳为当前 ROS 系统时间。
2. `/odom/nav2` 的位姿与 `/odom/localization_odom` 一致（在 `map→odom` 恒等条件下数值相同）。
3. TF 树中仅由 `odom_to_tf_broadcaster` 发布 `map→odom` 和 `odom→base_link`，不存在多发布者冲突。
4. `/localization_info.status == 3`，且 Nav2 的 planner、controller、BT navigator 处于 `active [3]`。
5. 全程不发送导航目标或速度命令；验证仅检查话题、TF 和节点状态。

如需回退，可恢复导航启动参数为 `input_odom_topic: /odom/mc_odom`、`publish_map_to_odom: false`，并将定位配置 `send_tf_transforms` 恢复为 `true`，然后重建并重启导航。仅在确认 Mid-360+IMU 定位不可用时使用该回退方案。

## 6. 本次实施与验收记录

本次已在 NX 现役工作区完成与开发仓库一致的部署，并执行：

```bash
colcon build --packages-select localization robot_navigo --parallel-workers 4
```

构建和安装均成功。随后使用 `script/robot/start_navigation_real.sh` 依次停止 Nav2、重启定位、恢复地图并重新启动 Nav2；未下发导航目标或速度命令。

实际 ROS 核验结果：

| 核验项 | 结果 |
| --- | --- |
| 转换节点输入 | `/odom/localization_odom` |
| `/odom/localization_odom` | `localization` 唯一发布；由转换节点和健康监测订阅 |
| `/odom/nav2` | `odom_to_tf_broadcaster` 唯一发布；`controller_server` 与 `bt_navigator` 订阅 |
| `/odom/mc_odom` | `dog_task` 仍发布，但订阅数为 0，不参与定位和导航 |
| `/odom/nav2` 帧 | `header.frame_id=odom`、`child_frame_id=base_link` |
| `map→odom` | 恒等变换（零平移、单位四元数） |
| 定位状态 | `/localization_info.status = 3` |
| Nav2 生命周期 | `planner_server`、`controller_server`、`bt_navigator` 均为 `active [3]` |
| 传感器健康 | LiDAR、内置 IMU、`/laser_scan` 与 `/odom/localization_odom` 均在线 |

验收时 RTK 仍无有效解，因此运行中的定位源为 `ndt_imu`；这不影响本次将 Nav2 里程计切换为 Mid-360+IMU 融合结果。
