# 真实 GICP/ICP 几何验证与保存后自动定位自检计划

## 一、背景、现状与任务解释

### 1.1 Fast-LIO 和地图定位不是同一件事

FAST-LIO2 负责连续相对里程计，即：

> 从本次开机位置开始，机器人移动了多少。

它本身不知道机器人位于历史地图的哪个位置，也不能独立提供历史地图中的绝对 XY 和 yaw。

FAST-LIO2 启动时，室内外共用同一套初始化：

1. 机器人保持静止。
2. 采集约 600 个 Mid-360 IMU 样本。
3. 检查加速度和角速度方差，运动过大则重新初始化。
4. 用平均加速度估计重力方向和初始 roll/pitch。
5. 用平均角速度初始化陀螺零偏。
6. 从当前启动位置建立局部 `lio_odom` 原点。

所以 FAST-LIO2 初始化完成后提供的是：

```text
lio_odom → base_link
```

而不是历史地图中的：

```text
map → base_link
```

“初始化定位”真正要解决的是如何建立 `map→lio_odom` 或等效地图绝对位姿。

### 1.2 当前室内初始化定位

室内没有稳定 RTK 绝对位置，当前流程是：

1. 加载 `map.pcd`。
2. 从以下来源获得一个地图位姿种子：

   - 配置中的默认 `(0,0,0)`；
   - 平台或 RViz 发布的 `/initialpose`；
   - 定位丢失前最后一个可信位姿；
   - Scan Context 候选，但目前默认关闭。

3. 从种子位姿执行两遍 PCL ICP。
4. 使用 ICP 结果重新创建定位估计器。
5. 正常定位使用全图 NDT 粗配准和局部 FastVGICP 精配准。
6. 连续两帧收敛且 fitness 低于 `0.50` 后判定初始化成功。
7. 初始化后由 FAST-LIO 持续提供相对运动，NDT/VGICP 按门限提供地图绝对校正。

当前所谓“全局 ICP”并不是真正全局搜索，它没有多位置、多 yaw 搜索，成功高度依赖初始种子。机器人若不在建图原点附近，或者 yaw 偏差较大，可能无法初始化。

项目已经具备 Scan Context 位置识别代码，但正式配置仍为关闭状态，因此室内冷启动仍主要依赖人工初始位姿。

### 1.3 当前室外初始化定位

室外在 FAST-LIO 相对里程计基础上增加 RTK 绝对坐标。

建图前必须：

- 锁定 RTK ENU 原点；
- 保存 `gnss_origin.yaml`；
- 确认 ENU 到地图坐标的旋转；
- 使用双天线航向建立绝对 yaw。

RTK 不会直接重置 FAST-LIO 前端 ESKF。FAST-LIO 仍维持连续局部轨迹，RTK 位置和航向作为全局约束进入地图后端或定位校正。

导航定位时：

1. 加载地图及 `gnss_origin.yaml`。
2. 将当前经纬度转换为地图 ENU XY。
3. 根据天线杆臂换算 `base_link` 位置。
4. 用双天线航向计算地图 yaw。
5. 通过 `/localization/seed_from_rtk` 注入初始位姿。
6. 使用 NDT/FastVGICP 验证 RTK 种子与点云地图是否一致。
7. 连续正常样本通过后进入定位正常状态。

室外初始化优先级为：

```text
固定 RTK + 有效双天线航向
→ 地图 XY/yaw 种子
→ NDT/FastVGICP 验证
→ 初始化成功
```

RTK 不可用时才回退到人工种子、最后可信位姿、ICP 或后续 Scan Context 方案。

初始化完成后，FAST-LIO 仍是唯一连续位姿源；RTK 和 NDT/VGICP 只执行去重、门控和平滑后的绝对校正。

### 1.4 室内外差异

| 项目 | 室内 | 室外 |
|---|---|---|
| FAST-LIO IMU 初始化 | 静止采样、重力和零偏 | 相同 |
| FAST-LIO 初始 XY/yaw | 本次启动的局部原点 | 相同 |
| 历史地图种子 | 人工位姿、最后可信位姿、可选 Scan Context | 优先固定 RTK XY 和双天线 yaw |
| 初值精化 | ICP → NDT/FastVGICP | RTK 种子 → NDT/FastVGICP |
| 地图原点 | 本地坐标 | 强制锁定 ENU 原点 |
| 点云范围 | 约 10 m | 约 25 m |
| 主要风险 | 未知位置、重复走廊、yaw 误差 | RTK 遮挡、原点错误、航向失效 |

因此：

> FAST-LIO 负责“从这里开始走了多少”，初始化定位负责“这里在历史地图中的什么位置”。

### 1.5 当前回环几何验证的缺口

现有 Scan Context 回环流程已经能够找出外观相似的历史关键帧，但当前所谓几何验证只做：

- 按 Scan Context yaw 旋转点云；
- 投影为二维占据格；
- 计算占据重叠；
- 比较点云质心差。

它没有真正执行点云迭代配准，因此无法提供可靠的：

- GICP/ICP 收敛状态；
- 三维相对位姿；
- RMSE；
- 内点数量与比例；
- 双向重叠率；
- Hessian 退化程度；
- 可信回环协方差。

长走廊、对称房间和重复厂房结构可能同时满足“描述子相似”和“二维重叠较高”，但实际上是假回环。假回环进入位姿图后可能拉歪整张地图。

项目虽然已经集成 FastGICP，并用于帧间激光里程计和局部 NDT 后的 VGICP 精配准，但保存后的回环验证尚未调用真实 GICP。

### 1.6 当前保存后自检的缺口

保存流程已经产生 `localization_validation.json`，但当前只运行 Scan Context 留一法检索，明确记录：

```text
test_type = offline_scan_context_leave_one_out
live_ndt_initialization_tested = false
threshold_policy = manual_review
```

它只能说明关键帧描述子能否检索到相近关键帧，不能证明：

- 新 `map.pcd` 能被正式定位节点加载；
- 当前真实点云能与新地图配准；
- NDT/FastVGICP 能连续收敛；
- 室外 RTK 原点和双天线航向与地图一致；
- `/localization_info.status` 能进入正常状态；
- 新地图激活后不会产生位姿跳变。

而且当前验证结果没有直接参与地图指针和云端自动激活门禁。

本任务要把地图质量判断从：

> 数据看起来合理。

升级为：

> 真实点云配准成立，而且正式定位栈确实能够使用这张地图完成初始化。

## 二、目标与总体流程

新的地图保存闭环为：

```text
FAST-LIO 保存原始地图
→ Scan Context 检索候选
→ FastGICP 真实三维几何验证
→ 安全位姿图优化
→ 按优化轨迹重建地图
→ 离线地图完整性与定位能力检查
→ 停止建图进程
→ 加载候选地图做实机静止定位自检
→ 通过后原子激活
→ 打包上传验证报告
```

最终原则：

- FastGICP 是回环几何验证的权威结果。
- 普通 ICP 仅作为兼容回退和交叉诊断。
- 室内必须通过优化终点种子的真实定位初始化。
- 室内 Scan Context 未知位姿恢复先影子运行，达标后再正式启用。
- 室外必须通过固定 RTK、双天线航向和点云匹配联合验证。
- 自检失败或待验证时禁止本地、云端自动激活，旧地图保持不变。
- 保存失败不删除 raw 地图、关键帧或诊断数据。
- 自检期间不启动 Nav2、不发布 `/cmd_vel`、不触发运动。
- 本期不修改 FAST-LIO2 ESKF，不实施在线 iSAM2。

## 三、真实 GICP/ICP 几何验证设计

### 3.1 C++ 几何验证器

在 `robot_slam` 增加可独立运行的 C++ 验证器，链接现有 `fast_gicp`，同时服务于新地图保存和历史地图离线复核。

处理步骤：

1. Python 只生成 Scan Context 描述子和 Top-K 候选，不再计算二维质心回环位姿。
2. 对每个候选，以查询帧和匹配帧为中心各取前后 3 个关键帧。
3. 使用关键帧位姿把相邻点云变换到各自中心帧的 LiDAR 坐标系。
4. 室内按 0.20 m、室外按 0.30 m 降采样，每侧最多 4 万点。
5. 使用 Scan Context yaw 作为旋转初值，稳健质心差作为平移初值。
6. FastGICP 使用 2 个线程、最多 30 次迭代：

   - 室内最大对应距离 1.0 m；
   - 室外最大对应距离 1.5 m。

7. FastGICP 执行异常或不收敛时，允许普通 ICP 运行一次诊断回退。
8. 输出统一为从查询关键帧到匹配关键帧的相对位姿：

```text
T_query_match = T_world_query⁻¹ × T_world_match
```

### 3.2 几何质量门

| 指标 | 室内 | 室外 |
|---|---:|---:|
| 最少内点 | 500 | 500 |
| 双向重叠率 | ≥45% | ≥45% |
| RMSE | ≤0.25 m | ≤0.40 m |
| 与当前图预测最大差异 | 3.0 m / 20° | 3.0 m / 20° |
| 最大垂直差异 | 0.75 m | 0.75 m |
| Hessian 条件数 | ≤`1e6` | ≤`1e6` |

双向重叠率取源到目标、目标到源两个内点比例中的较小值。

以下情况直接拒绝：

- 输入点数不足；
- 结果未收敛；
- 变换或评分非有限；
- RMSE 超限；
- 内点或重叠率不足；
- 与 FAST-LIO 当前图预测严重不一致；
- 垂直方向异常；
- Hessian 非正定或明显退化；
- ICP 和 GICP 结果差异超过 0.30 m/3°；
- 临时位姿图优化不能安全收敛。

标准拒绝码：

- `no_convergence`
- `insufficient_points`
- `insufficient_inliers`
- `low_overlap`
- `rmse_too_high`
- `pose_inconsistent`
- `vertical_inconsistent`
- `degenerate_hessian`
- `icp_disagreement`
- `trial_optimization_rejected`

没有候选或候选被正常拒绝不代表地图失败；验证器异常、结果损坏或错误回环已影响地图且无法回滚才算失败。

### 3.3 回环约束格式

将 `loop_closures.csv` 升级为 v2，保存：

- 查询和匹配索引；
- 完整平移和四元数；
- GICP/ICP 算法标识；
- 收敛状态；
- RMSE；
- 内点数和双向重叠率；
- Hessian 条件数；
- Scan Context 距离；
- `verified`；
- 拒绝原因；
- `[x,y,z,yaw]` 4DoF 协方差的十个上三角元素。

GTSAM 装载规则：

- 只读取 `verified=true` 的 v2 约束；
- 移除从原始 `loop_candidates.csv` 自动生成回环因子的兼容路径；
- 将 4DoF 协方差映射到 Pose3 的 x/y/z/yaw；
- roll/pitch 使用无约束量级方差，保持 FAST-LIO 重力姿态；
- 对回环因子使用鲁棒核；
- 加入正式图前执行临时 LM 试优化。

试优化满足以下任一条件时拒绝回环并保留 raw 地图：

- 图误差增大；
- 最大轨迹修正超过 1.5 m 或 10°；
- 相邻关键帧修正突变超过 0.15 m 或 1°；
- 输出出现非有限值；
- 最终地图重建不完整。

`auto_loop_optimization_enabled` 初期保持关闭，真实几何结果先用于人工复核。历史回归达到上线门槛后，才允许已验证回环自动进入保存后位姿图。

## 四、保存后自动定位自检设计

### 4.1 保存流程重排

保存协调器按以下顺序执行：

1. 停止新增关键帧。
2. 保存 `map_raw.*`、原始轨迹、关键帧和 IMU 预积分。
3. 生成 Scan Context 候选。
4. 执行真实 GICP 几何验证。
5. 调用 GTSAM 保存后优化。
6. 质量门失败时恢复完整 raw 地图产品集。
7. 写入最终轨迹、地图和外参元数据。
8. 运行离线检查。
9. 停止建图进程，确认 `/mapping` 和 `/lio_odometry` 退出。
10. 启动仅定位验证栈，不启动 Nav2。
11. 通过显式路径加载候选地图，不修改当前地图指针。
12. 执行室内或室外实机静止自检。
13. 通过后原子切换地图指针。
14. 打包上传；失败或待验证地图强制 `auto_activate=false`。

地图激活操作从打包函数中移出，成为验证通过后的独立步骤，避免“打包即切图”。

`stop_process=false` 时只完成保存、几何验证和离线检查，状态进入 `pending_live_validation`；停止建图进程后才能继续实机检查和激活。

### 4.2 离线必检

离线检查必须验证：

- `map.pcd/map.yaml/map.pgm` 可读且对应同一地图版本；
- 关键帧数量、点云数量、轨迹数量一致；
- 位姿、四元数和协方差均为有限值；
- GICP 验证器正常完成；
- 已应用回环全部来自 `verified=true` 结果；
- 优化质量门通过，或者 raw 回滚完整；
- 最终地图可被定位节点地图载入器读取；
- 最终轨迹与地图使用同一坐标系和同一外参版本。

继续保留 Scan Context 留一法评估，但在影子阶段不作为激活门：

- 排除查询帧前后至少 2 帧；
- 只统计底库中确有空间近邻的可答查询；
- 输出 Top-1、Top-5、位置误差和 yaw 误差；
- 没有可答查询时记录 `insufficient_revisit`，不判地图失败。

### 4.3 室内实机静止自检

室内采用“终点必过 + Scan Context 影子”策略。

强制检查：

1. 从最终优化轨迹末帧取得地图位姿。
2. 使用地图记录的 LiDAR/IMU/base 外参转换为 `map→base_link` 种子。
3. 启动定位节点和 FAST-LIO odometry-only 前端，不启动 Nav2。
4. 显式加载候选 `map.pcd`。
5. 发布 `/initialpose`。
6. 只接受地图加载后产生的新点云、新 ScanMatchingStatus 和新定位状态。
7. 30 秒内必须满足：

   - 连续至少 3 个 `/localization_info.status == 3`；
   - 至少产生一次新 NDT/FastVGICP 正常匹配；
   - fitness `<0.50`；
   - 内点率 `≥0.05`；
   - 定位结果距离优化终点不超过 0.50 m；
   - yaw 差不超过 10°；
   - 静止期间相邻输出跳变不超过 0.15 m/3°。

强制检查通过后，使用同一批新点云运行 Scan Context Top-5 和 ICP/GICP 验证影子检查：

- 不改变正式定位估计器；
- 不影响本次地图激活；
- 记录候选位置、yaw、描述子距离和几何验证结果；
- 输出“如果正式启用，能否从未知位姿完成初始化”。

### 4.4 室外实机静止自检

1. 加载候选地图和对应 `gnss_origin.yaml`。
2. 检查原点锁定状态、会话 ID 和外参版本。
3. 要求新鲜固定 RTK 和有效双天线航向。
4. 调用 `/localization/seed_from_rtk`。
5. 由定位节点完成经纬度到 ENU、ENU 到 map 和杆臂转换。
6. 使用 NDT/FastVGICP 验证 RTK 种子。
7. 30 秒内要求：

   - 连续至少 3 个正常定位样本；
   - 新鲜 NDT/FastVGICP 匹配；
   - fitness `<0.50`；
   - 内点率 `≥0.05`；
   - 定位结果与 RTK 地图坐标偏差不超过 0.75 m；
   - yaw 差不超过 10°；
   - 相邻输出跳变不超过 0.15 m/3°。

固定 RTK 或双天线航向暂时不可用时：

- 状态保持 `pending_rtk`；
- 不判地图损坏；
- 不激活地图；
- 每 30 秒限频重试；
- Edge Agent 重启后恢复待验证任务；
- RTK 恢复后自动继续；
- 不回退为室内终点种子通过，避免绕过室外绝对坐标链路。

### 4.5 失败和恢复

- 自检通过：原子切换 `current` 地图指针，定位节点保持加载候选地图且状态正常，Nav2 仍由后续导航命令启动。
- 自检失败：停止候选定位栈，保持旧地图指针，Nav2 保持停止。
- 自检待定：保持旧地图，不启动导航，等待自动重试。
- 失败或待定地图允许保存、打包和上传诊断材料，但不得成为活动地图。
- 所有失败保留输入地图、关键帧、日志、指标和失败原因。
- 不自动删除或覆盖当前有效地图。

## 五、接口与状态设计

### 5.1 配置

新增配置：

- `loop_geometry.mode: shadow|enforce`
- `loop_geometry.indoor.*`
- `loop_geometry.outdoor.*`
- `post_save_validation.mode: shadow|enforce`
- `post_save_validation.timeout_seconds: 30`
- `post_save_validation.required_normal_samples: 3`
- `post_save_validation.outdoor_retry_seconds: 30`
- `post_save_validation.preserve_previous_map: true`
- `relocalization.scan_context_runtime_mode: disabled|shadow|active`

初始部署：

- 真实几何验证为 `shadow`；
- 保存后自检为 `shadow`；
- 正式 Scan Context 初始化保持 `disabled`；
- 影子验收通过后，将几何验证和保存自检切换为 `enforce`；
- 最后单独将室内 Scan Context 切换为 `active`。

最终生产状态下，任一必检失败都阻止激活。

### 5.2 验证报告

将 `localization_validation.json` 升级为 v2，至少包含：

- `overall_status`: `pending|passed|failed|legacy_unverified`
- `activation_allowed`
- `scene_scope`
- `selected_map_product`: `raw|optimized`
- `offline_validation`
- `loop_geometry`
- `live_localization`
- `scan_context_shadow`
- `rtk_evidence`
- `attempt_count`
- `failure_reasons`
- `started_at`
- `completed_at`
- `previous_active_map`
- `candidate_map`

`map_manifest.json` 增加：

- `validation_required`
- `validation_status`
- `activation_allowed`
- `validation_schema`
- `loop_geometry_schema`
- `sensor_extrinsics`
- `pose_frame`

### 5.3 重试和激活接口

新增 `mapping.validate` 命令：

- 对指定本地地图版本重新执行未完成或失败的实机自检；
- 自动 RTK 重试与人工重新验证共用同一实现；
- 不接受运动参数；
- 不提供跳过质量门或强制通过能力；
- 成功后执行原子地图激活；
- 失败后保持旧地图。

### 5.4 Edge 和平台双重门禁

Edge 端：

- `validation_required=true` 且状态不是 `passed` 时拒绝激活；
- `current` 地图指针只在通过后切换；
- 上传元数据必须携带验证摘要；
- 失败或待定地图上传时强制 `auto_activate=false`。

平台端：

- 地图模型增加 `validation_status` 和 `localization_validation`；
- 自动激活和手动 `map.activate` 都拒绝未通过的新地图；
- 返回统一错误码 `MAP_LOCALIZATION_VALIDATION_REQUIRED`；
- 地图页面展示几何验证、离线检查、实机定位、RTK 等待和 Scan Context 影子结果。

兼容策略：

- 已有地图缺少新字段时标记 `legacy_unverified`；
- 不追溯取消现有活动地图；
- 新 schema 地图必须通过新验证门禁。

## 六、测试与验收

### 6.1 C++ 测试

覆盖：

- 已知三维变换的真回环；
- 无重叠点云；
- 重复走廊和对称结构；
- 平面、直线等退化结构；
- GICP 不收敛；
- ICP 回退及 GICP/ICP 结果不一致；
- 内点、重叠率和 RMSE 门限；
- Hessian 正定性和条件数；
- 4DoF 协方差映射；
- `T_query_match` 方向约定；
- 错误回环试优化；
- raw 地图完整回滚。

### 6.2 Edge 测试

覆盖：

- 保存阶段执行顺序；
- `stop_process=false` 进入待实机验证；
- 通过后才切换地图指针；
- 失败、超时或进程崩溃时旧指针保持不变；
- 室外无 RTK 进入 pending 并自动重试；
- Edge Agent 重启后恢复待验证任务；
- 失败地图上传时 `auto_activate=false`；
- 验证期间 Nav2 未启动；
- 验证期间没有 `/cmd_vel` 运动输出。

### 6.3 定位回放与实机测试

覆盖：

- 室内优化终点种子初始化；
- 错误初值被拒绝；
- raw 地图与优化地图 A/B 定位；
- 室外固定 RTK 初始化；
- RTK 浮点解、过期、航向无效及恢复；
- Scan Context Top-K 影子检索；
- GICP 候选验证；
- 地图切换后的 TF 连续性；
- 定位输出无明显瞬时跳变。

### 6.4 平台测试

覆盖：

- 上传包解析验证摘要；
- 未通过地图不能自动激活；
- 未通过地图不能手动激活；
- 通过地图正常激活；
- legacy 地图兼容；
- 前端正确显示 `pending/failed/passed`；
- 室外待 RTK 状态不会误显示为地图损坏。

### 6.5 影子上线门槛

进入强制门禁前必须满足：

- 历史人工标注回环集中假回环接受数为 0；
- 真回环几何验证召回率达到 95% 以上；
- 至少连续完成 10 次室内保存；
- 至少连续完成 5 次室外保存；
- 实机必检没有误拒绝确认可用的地图；
- GICP 验证不阻塞 FAST-LIO 前端；
- 保存阶段额外内存不超过 500 MB；
- 所有失败场景均确认旧地图指针未改变。

室内 Scan Context 正式启用还需满足：

- 可答查询 Top-5 命中率不低于 95%；
- 影子 GICP/ICP 验证没有错误位置接受；
- 每次只验证一个候选；
- 不增加单帧定位回调的 ICP 次数；
- 失败后按现有 5 秒节流尝试下一个候选；
- 地图没有 Scan Context 资产时可靠回退人工种子或最后可信位姿。

## 七、假设与边界

- 新地图必须记录最终轨迹、姿态坐标系及 LiDAR/IMU/base 外参，不得默认这些坐标系完全重合。
- 无回环不是失败，假回环被正确拒绝也不是失败。
- 地图能够从优化终点完成局部初始化是强制激活门。
- 室内未知位姿 Scan Context 能力在本阶段只做影子评价。
- 室外必须验证 RTK 绝对坐标链，RTK 暂时不可用时保持待验证。
- 失败地图允许保存和上传用于诊断，但不能成为活动地图。
- 当前活动地图、导航任务和 FAST-LIO ESKF 状态不由本计划自动迁移或重写。
- 普通 ICP 不能仅凭 `hasConverged()` 和单一 fitness 获得回环接受资格，必须经过相同的内点、重叠率、一致性和退化门。
