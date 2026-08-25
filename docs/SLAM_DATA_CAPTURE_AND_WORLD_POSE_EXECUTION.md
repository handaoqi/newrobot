# SLAM 完整数据采集与世界位姿 — 执行进度

更新时间：2026-08-21

本文件跟踪 [SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md) 的落地，不修改原计划。四元数统一为 ROS `qx, qy, qz, qw`（JSON 字段带 `_xyzw` 后缀）。航点定位方式继续使用现有 `ndt` / `rtk`；地图级能力使用 `ndt` / `rtk_ndt`。

## 任务顺序

| ID | 任务 | 状态 |
|---|---|---|
| P0 | 核对现有建图闭环（start / 首关键帧 / save / 打包 / 上传） | 已完成（代码路径与单测；现场实机建图未跑） |
| P1 | 完整四元数、世界/LiDAR 双位姿、recording_manifest、map_manifest schema 2 | 已完成 |
| P2 | `rtk_fixed` / `local_only` 与前端、后端、Edge 三层场景限制 | 已完成 |
| P3 | IMU 预积分导出、Scan-Context 检索与安全回环（失败保留 raw） | 已完成 |

## 已拍板的实现决策

- 新建地图才生成完整产物；旧地图标 `legacy_incomplete`，不伪造预积分或回环数据，也不套用新的室外禁止规则，除非 manifest 已明确 `local_only`。
- “同步录制诊断数据”固定录制 `/front_lidar`、`/front_lidar/imu`、`/fix`、`/rtk_pvh`、`/rtk/ntrip_status`、`/odom/localization_odom`、`/slam_odom`、`/tf`、`/tf_static`。
- NX 会话目录保存完整 `keyframes/`、`imu_preintegration/`、`scan_context/`。上传 zip 只带轻量元数据（manifest、轨迹 CSV、Scan-Context 索引/候选、地图预览），不把全量关键帧 PCD 传到云端。
- `rtk_fixed` 复用现有 `alignment_locked`、样本数和 RMS 阈值。
- 回环优化失败或残差过大时，保留 `map_raw.pcd` 和 `trajectory_raw.csv`，不覆盖可用导航地图。
- 无 RTK 原点但操作员选了室外/过渡区时，保存时自动降为室内 `local_only`，不让整张地图保存失败。

## 执行记录

### P0 现有建图闭环

- 核对 `mapping_rosbag.sh`、`MappingAdapter.start/save`、`/slam/start_mapping`、`/slam/save_map` 与上传 zip 路径完整。
- 测试：`cd /home/dogrobot/edge-agent && python3 -m pytest -q tests/test_mapping_adapter.py tests/test_map_activation_adapter.py` → 24 passed。
- 未启动实机建图或运动。

### P1 完整世界位姿与 manifest

- `MappingKeyframe` 同时保存 `map` 系 IMU 世界位姿和 LiDAR 位姿，四元数为 `qx,qy,qz,qw`，并保留旧 `x,y,z,yaw` 列。
- 保存时写出 `trajectory_raw.csv`、`trajectory_optimized.csv`、`map_raw.pcd`、`map_manifest.json`。
- 停录后根据 rosbag2 `metadata.yaml` 生成 `recording_manifest.yaml`。
- `robot_slam` 已编译链接：`[100%] Built target mapping`。`colcon install` 因现有 `install/` 权限失败，但 `install/robot_slam/lib/robot_slam/mapping` 已指向新的 `build/robot_slam/mapping`。

### P2 场景限制

- 共享校验：`edge-agent/roamerx_edge/map_coordinate.py` 与 `platform/backend/monitoring/services/map_coordinate.py`。
- Edge 激活地图、任务启动、平台路线序列化和任务创建都会拒绝 `local_only` 的室外/过渡区/RTK 航点。
- 前端建图可选场景；路线编辑器在 `local_only` 地图上禁用 RTK 和室外/过渡区。
- 迁移：`0055_map_coordinate_mode`。

### P3 预积分与 Scan-Context

- 建图节点在关键帧之间做 IMU 预积分，写出 `imu_preintegration/preint_XXXXX.json`，旋转字段为 `delta_rotation_xyzw`；缺帧/回退写入 `error_status`，不伪造数据。
- 保存后 Python 生成 Scan-Context 描述子、候选和几何验证；位姿图失败则继续使用 raw 地图。

### 测试

```bash
cd /home/dogrobot/edge-agent && python3 -m pytest -q tests/test_map_coordinate.py tests/test_map_package_finalize.py tests/test_map_activation_adapter.py tests/test_mapping_adapter.py
# 46 passed

cd /home/dogrobot/platform/backend && python3 manage.py test monitoring.test_map_coordinate monitoring.test_p0_tasks monitoring.test_force_delete_and_maps
# 28 passed
```

现场验收仍按原计划第 8–9 步：有 RTK / 无 RTK 各建一张图，确认场景限制在平台和 Edge 重新加载后仍然有效。
