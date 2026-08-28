# 完整回环约束与在线 FAST-LIO2-SLAM 下一阶段计划

> 记录日期：2026-08-28
> 状态：待实施
> 适用范围：室内、室外建图链路

## 1. 总结

当前系统是 FAST-LIO2 前端加保存后 GTSAM/LM 批量优化；回环由 Python Scan Context 离线生成，几何验证只是二维占据重叠与质心差，不是真实点云配准。历史地图曾出现回环后最大 3.37 m、23.67° 修正，说明现有回环不能直接在线使用。

下一阶段采用以下方案：

- 第一阶段实现 Pose3 iSAM2 在线位姿图，不重复融合雷达内置 IMU。
- 采用 4DoF 回环约束：x、y、z、yaw，roll/pitch 保持 FAST-LIO2 重力估计。
- FAST-LIO2 ESKF 不回灌、不重置。
- 使用平滑 `map→lio_odom` 校正。
- 先室内影子验证，再启用室内在线校正，最后接入室外 RTK/ENU。

完整 SLAM 的完成标准是：真实回环配准、在线增量图优化、错误回环拒绝与回滚、实时全局位姿输出、保存后按优化轨迹重建地图全部打通。

## 2. 核心实现

### 2.1 在线前后端架构

在建图进程增加独立后台线程，FAST-LIO2 前端仍以约 10 Hz 输出连续里程计，在线后端只在关键帧到达时运行：

1. 记录关键帧原始 LIO 位姿、点云、6×6 协方差和可用 RTK。
2. 向 iSAM2 添加首帧先验和 FAST-LIO2 相邻位姿因子。
3. 异步计算 Scan Context，检索历史候选。
4. 使用局部子地图 FastGICP 生成真实相对位姿。
5. 候选通过连续证据、几何质量和试优化保护后才加入图。
6. 根据优化位姿计算目标 `map→lio_odom`，限速平滑发布。
7. 保存时停止接收新关键帧、排空后端队列，再执行最终批量精化和地图重建。

后端队列最大保留两个待处理关键帧；积压时跳过回环检测，但不能丢失关键帧和相邻位姿因子，也不能阻塞 FAST-LIO2。

### 2.2 回环检测与约束

替换 `edge-agent/roamerx_edge/map_loop_closure.py` 的质心验证，在线主路径使用 C++ FastGICP：

- Scan Context：20 rings、60 sectors、最大半径室内 12 m/室外 30 m、Top-K=3。
- 候选条件：关键帧间隔至少 30，沿轨迹距离至少 15 m，描述子距离不超过 0.25。
- 配准子地图：候选点前后各 3 个关键帧，室内体素 0.20 m、室外 0.30 m，每个子地图最多 4 万点。
- FastGICP：2 线程、最多 30 次迭代；室内最大对应距离 1.0 m、室外 1.5 m。
- 几何门：至少 500 个内点、重叠率不低于 45%，室内 RMSE 不超过 0.25 m、室外不超过 0.40 m。
- 一致性门：相对当前图预测偏差不超过 3 m、20°、垂直 0.75 m。
- 连续证据：连续 3 个查询关键帧命中同一候选簇，变换差不超过 0.30 m 和 3°，仅保留质量最好的一条约束。
- 从 GICP 对应点 Hessian 计算 4DoF 协方差；矩阵非正定、退化条件数超过 `1e6` 时拒绝。
- 回环加入前执行临时 LM 试优化；最大轨迹修正超过 1.5 m/10°、相邻修正突变超过 0.15 m/1°或图误差增大时，标记 `requires_review`，不提交 iSAM2。
- iSAM2 提交失败时从已确认因子记录重建图，确保错误回环可回滚。

### 2.3 在线因子图与地图输出

重构 `robot/src/slam/src/src/global_factor_graph.cpp`，批量优化和在线图共用因子构造：

- 在线状态只维护关键帧 `Pose3`。
- LIO 相邻因子使用前端协方差近似求和并做特征值裁剪；平移标准差限制在 0.03～0.50 m，旋转限制在 0.5～10°。
- 回环因子约束 x/y/z/yaw，roll/pitch 噪声设为无约束量级。
- 室内不允许任何 RTK 因子。
- 室外仅在原点锁定、时间新鲜和质量门通过时添加 RTK 水平位置及双天线航向因子；不使用 GNSS 高度约束。
- 所有 LIO、回环和 RTK 因子使用鲁棒核并记录拒绝原因。
- 当前 CombinedImuFactor 保留在保存后最终批量优化；第一阶段在线图不再次融合雷达 IMU，避免和 FAST-LIO2 紧耦合结果重复计权。
- iKD-Tree 和 FAST-LIO2 内部地图始终位于连续的 `lio_odom` 坐标系；全局校正只用于发布、关键帧重投影和最终地图。

最终地图按每个关键帧的优化 Pose3 重新投影，保留 `map_raw.pcd`，生成优化后的 `map.pcd`、栅格和轨迹；无有效回环/RTK 时退化为现有 LIO/IMU 保存后平滑。

## 3. 接口与状态

坐标链统一为：

```text
map → lio_odom → base_link
```

- `/odom/lio_odom`：FAST-LIO2 原始连续位姿。
- `/slam_odom`、`/odom/localization_odom`：组合后的平滑全局位姿。
- `/slam/global_optimized_odom`：iSAM2 最新关键帧优化位姿。
- `/slam/global_optimized_path`：在线优化轨迹。
- `/slam/loop_closure_markers`：候选、接受和拒绝回环可视化。
- TF 只发布 `map→lio_odom` 和 `lio_odom→base_link`，移除当前直接 `map→body`，避免重复父节点。
- 平滑器限制校正速度为 XY 0.20 m/s、Z 0.10 m/s、yaw 2°/s；影子模式固定发布单位 `map→lio_odom`。
- 新增 `/slam/online_backend/set_active`（`std_srvs/srv/SetBool`）：`false` 为影子模式，`true` 允许发布校正 TF。
- `/slam/global_optimize` 继续承担保存后的最终批量精化。

状态 JSON 增加：

- `backend_mode`
- `isam_keyframe_count`
- `pending_queue_size`
- `candidate_loop_count`
- `verified_loop_count`
- `accepted_loop_count`
- `rejected_loop_count`
- `last_loop_rmse`
- `last_loop_overlap`
- `last_loop_inlier_count`
- `map_to_lio_translation`
- `map_to_lio_yaw`
- `optimization_latency_ms`
- `rollback_count`
- `frontend_rate_hz`

影子数据写入 `online_backend/`；启用模式下确认的回环同步写入标准 `loop_closures.csv`。Edge 打包和云上传清单加入在线后端状态、候选和事件日志。

## 4. 实施与验收顺序

### 阶段一：离线验证器升级

- 实现 C++ Scan Context、FastGICP、4DoF 协方差和回环门。
- 使用历史地图回放，不接入 TF，不改变正式地图。
- 对已有接受回环的地图人工标注真回环和假回环，建立固定回归集。

### 阶段二：室内影子模式

- 接入 iSAM2 和后台线程，只记录在线图和目标校正，不发布有效校正 TF。
- 连续完成 10 次闭环地图。
- 人工复核的错误回环接受数必须为 0。
- FAST-LIO2 前端频率不低于 9 Hz。
- 在线后端额外内存不超过 500 MB。

### 阶段三：室内在线启用

- 启用平滑 `map→lio_odom`。
- 完成 5 次闭环建图。
- 已知闭环终点误差不超过 0.25 m/3°。
- TF 单周期不得出现超过 0.03 m/0.3°跳变。
- 导航地图不得出现新增重影、墙体分叉或异常扭曲。

### 阶段四：室外扩展

- 接入 ENU 水平位置和双天线航向因子。
- 完成 5 条闭环路线。
- 原点偏移不超过 0.20 m/2°。
- RTK 失效时图继续使用 LIO 和回环，恢复后不能产生位姿跳变。

### 阶段五：默认启用

- 只有上述验收全部通过后，才将室内、室外配置从 `shadow` 改为 `active`。
- 任何异常均可通过服务切回影子模式，不影响 FAST-LIO2 建图和原始地图保存。

## 5. 测试计划

自动化测试覆盖：

- Scan Context 描述子、候选排序和时间间隔排除。
- FastGICP 真回环、无重叠、退化结构和重复走廊。
- 4DoF 投影及 roll/pitch 不受回环干扰。
- 回环协方差正定性和退化条件数门。
- iSAM2 增量更新、错误回环试优化及回滚。
- LIO、回环和 RTK 因子计数及协方差。
- 后端队列积压、跳过检测和保存时排空。
- RTK 中断、恢复、时间戳过期及室内 GPS 禁用。
- TF 唯一父节点和 `map→lio_odom→base_link` 连续性。
- `map_raw.pcd` 保留及优化地图重建。
- 历史 rosbag 的旧算法/新算法 A/B 地图对比。

无回环、无 RTK 的地图必须保持安全退化：在线校正接近单位变换，不能因为启动在线后端而人为改变 FAST-LIO2 原始轨迹。

## 6. 假设与边界

- 以当前工作树为基线，保留已完成的 `lio_between_factor_count`、兼容字段和自动激活保护。
- 设备现有 GTSAM 4.2、iSAM2、PCL 和 FastGICP 依赖可直接使用。
- 正式建图期间继续停止定位和导航服务。
- 第一阶段不实现全状态在线 IMU 速度/偏置图；该内容在在线 Pose 图稳定后单独评估。
- 不直接修改或重置 FAST-LIO2 ESKF 状态。
- 保存后批量优化继续作为最终精化和兜底链路。
