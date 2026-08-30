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

### 2.1 与 Tier 2 快速重定位问题的关系

当前快速重定位分为三层：

| 层级 | 定位种子来源 | 当前状态 |
|---|---|---|
| Tier 0 | 固定 RTK XY + 双天线航向 | 室外已具备 |
| Tier 1 | Scan Context 检索关键帧位置和 yaw，再做 ICP/GICP 验证 | 已落码但默认关闭 |
| Tier 2 | 最后可信位姿，再做有初值依赖的 ICP | 现网兜底 |

Tier 2 的根本问题是：它只会从最后可信位姿重新开始，没有全地图位置搜索和 360° yaw 搜索。位姿或航向漂出 ICP 收敛域后，重复重试仍可能使用同一个错误种子。

本计划不是把 Tier 2 本身改成全局定位，而是逐步验证并正式启用更强的 Tier 1，使大部分深度失位在进入 Tier 2 前被自动找回。正式启用后的自动流程为：

```text
当前实时点云
→ 生成 Scan Context 描述子
→ 在地图关键帧数据库中检索 Top-K
→ 得到候选关键帧的地图位置和 yaw
→ FastGICP/ICP 真实几何验证与精化
→ NDT/FastVGICP 连续初始化验证
→ 重建定位估计器
→ /localization_info.status 恢复为 3
```

里程碑必须明确区分：

- 只完成真实回环验证和保存后自检时，解决的是地图质量和激活安全，现网 Tier 2 行为不变。
- 完成 Scan Context 影子验证但仍为 `shadow` 时，只能知道“它本可以找到哪里”，不会接管正式定位。
- 只有将 `relocalization.scan_context_runtime_mode` 切换为 `active` 后，系统才具备室内自动寻找关键帧对应位置并绕过错误 Tier 2 种子的正式能力。
- 即使 Tier 1 已正式启用，Tier 2 仍保留为无关键帧资产、无候选或所有候选均被拒绝时的安全兜底。

自动找回不承诺覆盖所有场景。以下情况必须安全回退 Tier 2 或人工初值，不能为了提高成功率放宽错误位置接受门：

- 长走廊、对称房间、重复厂房等感知混淆场景；
- 当前区域未被建图覆盖；
- 环境变化、遮挡或动态物体导致描述子和几何严重失配；
- 当前点云点数不足或姿态倾斜超出描述子稳定范围；
- 地图缺少描述子、关键帧位姿或几何验证点云；
- Top-K 候选全部未通过 GICP、ICP 或 NDT/FastVGICP 质量门。

### 2.2 本机和跨机器人地图资产要求

Tier 1 不能只依赖本机建图目录中的原始关键帧。为了让平台下发到另一台机器人的地图也能自动寻找关键帧，地图包必须新增一套明确的重定位资产：

```text
relocalization/
├── index.json
├── descriptors.bin
├── ring_keys.bin
├── sector_keys.bin
├── keyframe_poses.csv
└── keyframes/
    ├── scan_00000.pcd.zst
    └── ...
```

约束如下：

- 描述子包含版本、rings、sectors、最大半径、坐标系和生成算法。
- `keyframe_poses.csv` 保存与最终选用轨迹一致的地图位姿，优化地图不得继续使用 raw 种子。
- 重定位关键帧使用降采样后的本地 LiDAR 坐标点云，室内 0.20 m、室外 0.30 m，并压缩存储；不直接上传全分辨率建图关键帧。
- 每个资产记录大小和 SHA-256，激活前校验完整性。
- 运行时优先加载预计算描述子，避免要求跨机器人端重新生成。
- GICP 验证按候选按需解压对应关键帧或局部子图，设置缓存上限，不能一次性把全部点云常驻内存。
- 旧地图缺少这些资产时标记 `relocalization_assets=missing`，保持 Tier 2 行为，不伪造 Tier 1 可用状态。

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
8. 采用列向量约定，输出把查询/源点变换到匹配/目标坐标系的相对位姿：

```text
T_target_source = T_world_target⁻¹ × T_world_source
T_world_source = T_world_target × T_target_source
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

## 七、分阶段实施与目标推进

### 7.1 阶段 P0：冻结基线和统一接口

- 固定当前地图集、历史 rosbag、人工确认的真回环和假回环作为回归集。
- 固定 `T_query_match`、LiDAR/IMU/base 外参和 4DoF 协方差顺序。
- 定义 `loop_closures.csv` v2、`localization_validation.json` v2 和重定位资产格式。
- 为当前 Tier 0、Tier 1、Tier 2 增加明确来源、尝试次数、候选和拒绝原因日志。
- 本阶段不改变定位、回环应用或地图激活行为。

完成标准：接口样例、回归数据和旧行为基线全部可重复生成。

### 7.2 阶段 P1：真实 GICP/ICP 几何验证影子运行

- 实现 C++ FastGICP 验证器、ICP 诊断回退、质量门和退化检测。
- 保存后同时运行旧二维验证和新三维验证，只比较结果，不自动应用新回环。
- 在历史地图上标记旧算法接受、新算法拒绝及两者变换差异。
- 验证器异常不覆盖 raw 地图，也不写入正式回环因子。

完成标准：人工标注假回环接受数为 0，真回环召回率达到 95% 以上，输出变换方向和协方差通过单元测试。

### 7.3 阶段 P2：保存后离线和实机定位自检影子运行

- 重排保存流程，但在影子模式下只记录“如果强制门禁是否允许激活”。
- 室内执行终点种子真实定位自检，并记录 Scan Context 自动找回结果。
- 室外执行 RTK + 双天线航向 + 点云匹配联合自检。
- 记录耗时、误拒绝、定位误差、跳变和失败原因，不改变现有地图指针策略。

完成标准：至少 10 次室内、5 次室外保存完成；没有误拒绝人工确认可用的地图；失败和 RTK 待定状态能够稳定复现。

### 7.4 阶段 P3：启用地图激活强制门禁

- 将 GICP 验证和保存后必检切换为 `enforce`。
- 自检通过后才允许切换本地 `current` 指针和云端活动地图。
- 失败地图保留旧图并上传诊断；室外无 RTK 保持待验证并自动重试。
- Edge 和平台同时校验，避免绕过任一侧门禁。

完成标准：通过、失败、超时、进程崩溃和 Edge 重启场景下地图指针均符合预期，验证期间无运动命令。

> 到 P3 为止，地图生产和激活已经安全，但正式运行时 Scan Context 仍未接管定位，Tier 2 问题尚未被解决。

### 7.5 阶段 P4：运行时 Scan Context 影子重定位

- 定位丢失或冷启动时，使用当前真实点云检索关键帧 Top-K。
- 对候选执行 FastGICP/ICP 几何验证，但不修改正式定位估计器。
- 同时记录现有 Tier 2 最后可信位姿 ICP 的真实结果，形成同场景 A/B。
- 每次最多验证一个候选，候选失败后按现有 5 秒节流继续，避免阻塞单线程点云回调。
- 记录“候选是否正确、若接管是否能恢复、耗时、内存和错误接受”。

完成标准：可答场景 Top-5 命中率不低于 95%，GICP/ICP 错误位置接受数为 0，运行时开销不破坏定位点云处理频率。

### 7.6 阶段 P5：本机地图正式启用 Tier 1

- 将本机完整资产地图的 `scan_context_runtime_mode` 切换为 `active`。
- 重定位优先级正式变为 Tier 0 → Tier 1 → Tier 2。
- Tier 1 候选通过 GICP/ICP 后仍必须经过 NDT/FastVGICP 连续初始化门，不能直接宣布定位成功。
- 候选全部失败、数据库缺失或场景不可答时安全回退 Tier 2，不发布错误地图位姿。
- 平台显示实际使用层级、命中关键帧、验证指标和回退原因。

完成标准：室内绑架、冷启动非原点和大 yaw 偏差测试能够自动恢复；错误候选不进入正常定位状态；Tier 2 兜底仍可用。

> 到 P5 才算解决本机地图中大部分“Tier 2 种子漂出 ICP 收敛域后无法恢复”的问题。

### 7.7 阶段 P6：跨机器人地图正式启用 Tier 1

- 保存和上传重定位描述子、最终关键帧位姿及压缩降采样点云。
- 下载、校验、解压和激活流程验证重定位资产版本与哈希。
- 运行时从预计算描述子加载数据库，并按需读取候选关键帧点云。
- 在非建图机器人上重复室内冷启动、绑架和错误候选测试。
- 缺少或损坏资产时自动标记降级并回退 Tier 2，不阻止 legacy 地图按原能力使用。

完成标准：平台下发地图在另一台兼容机器人上达到与建图机器人相同的 Tier 1 验收门槛。

### 7.8 最终完成定义

本计划的目标按三个层次验收：

| 目标 | 对应阶段 | 能力结果 |
|---|---|---|
| M1 地图安全生产 | P0～P3 | 真回环验证、保存后自检、失败不激活 |
| M2 本机自动关键帧找回 | P4～P5 | 室内正式使用 Tier 1，解决大部分 Tier 2 深度失位 |
| M3 跨机器人自动关键帧找回 | P6 | 平台下发地图同样具备 Tier 1 |

只有 M2 完成后，才能对外宣称“系统能够自动找到当前点云对应的历史关键帧位置”；只有 M3 完成后，该能力才适用于跨机器人地图复用。

## 八、假设与边界

- 新地图必须记录最终轨迹、姿态坐标系及 LiDAR/IMU/base 外参，不得默认这些坐标系完全重合。
- 无回环不是失败，假回环被正确拒绝也不是失败。
- 地图能够从优化终点完成局部初始化是强制激活门。
- 室内未知位姿 Scan Context 能力在本阶段只做影子评价。
- 室外必须验证 RTK 绝对坐标链，RTK 暂时不可用时保持待验证。
- 失败地图允许保存和上传用于诊断，但不能成为活动地图。
- 当前活动地图、导航任务和 FAST-LIO ESKF 状态不由本计划自动迁移或重写。
- 普通 ICP 不能仅凭 `hasConverged()` 和单一 fitness 获得回环接受资格，必须经过相同的内点、重叠率、一致性和退化门。
- 正式启用 Tier 1 只提高可恢复范围，不保证所有环境下 100% 自动重定位；拒绝不确定候选优先于错误定位。

## 九、2026-08-30 执行记录与下一步

### 9.1 已落码能力

本轮完成了 P1/P4 的第一段可运行实现，但没有开启生产位姿接管：

- 新增 ROS 无关的 C++ `RelocalizationGeometryVerifier`，真实调用现有 FastGICP，并使用普通 ICP 做独立结果交叉检查。
- 几何结果包含：GICP/ICP 收敛、RMSE、源/目标内点、双向重叠率、相对初值的平移/旋转/垂直变化、Hessian 正定性与条件数、GICP/ICP 位姿差和耗时。
- 按统一拒绝码执行安全门：`insufficient_points`、`no_convergence`、`rmse_too_high`、`insufficient_inliers`、`low_overlap`、`pose_inconsistent`、`vertical_inconsistent`、`degenerate_hessian`、`icp_disagreement`。
- `ScanContextDatabase` 新增关键帧索引解析和按需 PCD 加载。运行时只保存路径与原始 LiDAR 位姿，不把全部关键帧点云常驻内存。
- 新增 `localization_relocalization_geometry_check` 离线工具，把 Scan Context Top-K 召回、真实几何接受、正确接受和错误位置接受分别计数。
- 定位后台重定位线程新增 `relocalization.scan_context_runtime_mode`：
  - `disabled`：不加载、不执行；
  - `shadow`：检索和真实 GICP/ICP 只记日志，不写正式位姿；
  - `active`：只有几何门通过后，才把精化种子交给原有全图 ICP。
- 旧参数 `relocalization.use_scan_context=true` 仅作为兼容别名映射到 `active`；默认配置明确保持 `disabled`。
- 新增 5 个几何质量门和 2 个 Scan Context 余弦距离单元测试；定位包最新完整测试结果为 `129 tests, 0 errors, 0 failures, 8 skipped`。

主要实现文件：

- `robot/src/localization/localization/include/localization/relocalization_geometry.hpp`
- `robot/src/localization/localization/src/localization/relocalization_geometry.cpp`
- `robot/src/localization/localization/tools/relocalization_geometry_check.cpp`
- `robot/src/localization/localization/test/relocalization_geometry_policy_test.cpp`
- `robot/src/localization/localization/include/localization/scan_context_db.hpp`
- `robot/src/localization/localization/src/localization/scan_context_db.cpp`
- `robot/src/localization/localization/apps/localization_nodelet.cpp`

### 9.2 当前活动地图离线结果

验证地图：`20260828_205712_005`，共 76 个关键帧。为避免时间相邻帧造成虚高，评估排除索引前后 20 帧；39/76 个查询在剩余底库中确有 2 m 内空间近邻，属于可答查询。

默认室内门限（0.20 m voxel、1.0 m 对应距离、30 次迭代、RMSE 0.25 m、双向重叠率 45%、最少 500 内点）的结果：

| 指标 | 结果 |
|---|---:|
| Scan Context Top-5 召回 | 92.3%（36/39） |
| 首个几何通过候选覆盖 | 69.2%（27/39） |
| 正确位置接受 | 69.2%（27/39） |
| 错误位置接受 | 0 |
| 无候选通过 | 12/39 |
| 候选验证耗时中位数 / p90 | 131.88 / 419.84 ms |
| 接受后位置误差中位数 / p90 | 0.01 / 0.01 m |
| 接受后 yaw 误差中位数 / p90 | 0.02° / 0.04° |

正确候选与错误候选的指标已经呈现明显分离：

| 候选类型 | RMSE 中位数 / p90 | 双向重叠率 p10 / 中位数 | ICP 平移差中位数 / p90 |
|---|---:|---:|---:|
| 正确 | 0.22 / 0.30 m | 0.83 / 0.99 | 0.06 / 0.17 m |
| 错误 | 0.51 / 0.57 m | 0.00 / 0.00 | 1.11 / 1.61 m |

门限曲线结果：

- RMSE 放宽到 0.30 m：正确查询覆盖 74.4%，错误位置接受仍为 0。
- RMSE 放宽到 0.40 m：正确查询覆盖仍为 74.4%，错误位置接受仍为 0；新增候选主要被 ICP 不一致门拒绝。
- 最大迭代从 30 增到 60：覆盖没有提高，耗时 p90 增至约 653 ms，因此不采用。
- 正确候选的拒绝组成（RMSE 0.40 m、30 次）：`no_convergence=5`、`rmse_too_high=6`、`icp_disagreement=5`。
- 把 Top-K 直接增至 10 时召回反而降到 87.2%。原因是当前 ring-key 预筛候选数随 Top-K 改变，候选集合不具备单调性；不能用盲目增大 K 解决覆盖。

候选检索与子图追加实验：

- 已将 ring-key 预筛候选池与返回 Top-K 解耦；固定候选池为 30 后，Top-1/Top-5/Top-10 召回依次为 82.1%/97.4%/100%，恢复单调性。
- 新增标准 Scan Context 列余弦距离。固定候选池为 15/30/全部时 Top-5 均为 97.4%，优于原绝对高度距离的 92.3%/82.1%/82.1%。余弦排序已达到当前单图 95% 召回门槛，但仍需多地图复核。
- 余弦距离的描述子值仍不能作为硬阈值：正确候选距离 p90 约 0.07，错误 Top-1 距离中位数约 0.06，分布继续重叠。
- 单纯用绝对高度距离为余弦候选估 yaw 会出现对称混淆，yaw p90 达到 57.2°，因此该混合单 yaw 方案不采用。
- 目标侧前后 3 帧子图把默认几何覆盖从 69.2% 提到 74.4%，错误位置接受仍为 0；候选耗时中位数/p90 约 108.66/329.40 ms。
- 查询和目标两侧都使用前后 3 帧子图时覆盖仍为 74.4%，但耗时 p90 增至约 1.08 s，不适合运行时路径。
- 对余弦候选在首次失败后追加 ±6° 假设没有提高 74.4% 查询覆盖，反而出现 1 个错误候选通过几何门，耗时 p90 约 915 ms。该实验不满足“错误候选接受数为 0”，运行时必须保持 `yaw_neighbors=0`。

### 9.3 当前上线判断

当前实现证明了“错误候选可被真实三维几何拒绝”，但尚未达到正式接管条件：

- Top-5 召回 92.3%，低于计划要求的 95%；
- 默认几何覆盖 69.2%，低于真回环召回 95% 门槛；
- 当前评估仍是单关键帧对单关键帧，尚未实现计划中的前后各 3 帧局部子图和实时多帧积累；
- 只验证了当前一张地图，未完成跨地图、独立 rosbag 和实机绑架/冷启动回归；
- 保存协调器尚未接入 v2 验证摘要、raw 回滚、静止定位必检和验证后原子激活。

因此 `scan_context_runtime_mode` 必须继续保持 `disabled`。现在不能宣称 Tier 2 问题已经解决，也不能宣称系统已能在所有可答场景自动找到关键帧对应位置。已经具备安全影子运行和后续接管所需的核心几何门，但正式能力仍以 P5 验收为准。

### 9.4 后续推进顺序

1. 用历史地图与独立 rosbag 复核“固定 30 个预筛候选 + 列余弦距离”，确认 Top-5 提升不是当前地图过拟合，再决定运行时排序默认值。
2. 目标侧保留前后各 3 帧子图作为 P1/P4 候选；运行时增加有上限的多帧点云积累前，先解决子图重复加载和 p90 计算开销。
3. 用相同工具回归历史地图与独立 rosbag，门限只从正确/错误标注集校准，要求错误位置接受为 0。
4. 在 `shadow` 模式下做静止实机 A/B；确认不增加点云回调重任务次数，不产生 `/cmd_vel`，不修改正式定位估计器。
5. 将验证器接入地图保存流程，生成 `loop_closures.csv` v2 和 `localization_validation.json` v2，再实现 raw 回滚与验证后原子激活。
6. 只有 P4 门槛全部满足后才切本机地图为 `active`，并继续经过现有全图 ICP、NDT/FastVGICP 连续初始化门；失败时保留 Tier 2 安全兜底。

### 9.5 本轮运行安全记录

- 所有几何回归均为离线只读地图检查，没有发布 `/initialpose`、导航目标或 `/cmd_vel`。
- 现场 ROS launch 组在 11:50:10 收到统一 `SIGINT/SIGTERM` 后按节点顺序退出；定位日志没有崩溃栈、GICP 异常或 OOM，属于受控整组终止，不是本轮代码失败。
- 本轮没有自动重启导航栈。后续静止实机影子测试需先由任务上下文明确允许启动传感器/定位，并继续禁止运动输出。
- 12:00 后导航/定位栈由本轮命令之外的流程重新启动；只读核验显示节点实际参数为 `scan_context_runtime_mode=disabled`、`use_scan_context=false`，定位状态为 3、报告速度约 0.0018 m/s，6 秒观察窗口内 `/cmd_vel` 没有消息。本轮未发送服务请求、初始位姿或控制命令。
- 新启动实例仍可见稳定阶段重帧 p90 约 165～188 ms，主要耗时为现有 VGICP 约 142～164 ms；Scan Context 几何路径处于 disabled 且 `global=0.0 ms`，因此该残余回调耗时不是本轮几何验证引入，需继续按点云回调性能计划单独处理。

### 9.6 2026-08-31 当前地图复核

使用已安装的 `localization_relocalization_geometry_check` 对地图
`20260828_205712_005` 做只读复核（`top-k=5`、空间半径 2 m、最小间隔 1）：

- 76/76 查询可检索，Top-5 检索覆盖率 100%。
- 几何接受率 75.0%，正确位置接受率 75.0%。
- 错误位置接受 0，未通过候选 19 个。
- 正确/错误候选的 RMSE 中位数分别为 0.24/0.44 m；错误候选仍被 `no_convergence`、`rmse_too_high` 等门拒绝。
- 几何验证耗时中位数/p90 为 68.17/151.15 ms。

该结果进一步确认“错误候选接受数为 0”的安全性，但正确候选召回仍低于 P1/P4 的 95% 门槛；尚不能切换运行时 `active`，也不能进入 P3 强制激活门禁验收。
## 10. 地图管理页与保存后静止自检实现（2026-08-30）

- 保存成功并完成会话清理后，`MappingAdapter` 自动创建异步静止自检任务，不发布 `/cmd_vel`。
- 自检结果持久化到 `post_save_validation.json`，按 `indoor`/`outdoor` 分桶累计成功次数，目标分别为 10 次和 5 次，并保留最近 20 次历史。
- 默认 ROS 验证器通过显式 `PCD_MAP/MAP_YAML` 加载本次保存目录；室内发布保存终点 `/initialpose`，室外保留地图加载时的 RTK/ENU 初始化，不启动 Nav2、不发布速度命令。
- 只统计初始化边界之后的新定位、ScanMatchingStatus 和姿态帧；要求连续 3 帧正常、NDT 收敛、fitness `<0.50`、内点率 `>=0.05`、终点位置误差 `<=0.50 m`、航向误差 `<=10°`，相邻姿态跳变不超过 `0.15 m/3°`。
- 验证器输出 `roamerx.post-save-localization-check.v1` 结构化记录，包括候选地图目录、定位坐标、保存终点、位置/航向偏差、NDT 指标、帧数、最大跳变、判定代码和时间；非结构化或地图身份不一致的输出强制判失败。
- 失败后停止候选定位栈；结果以 `validation_id` 隔离，旧线程不能覆盖后一次保存状态。
- 云端成功保存快照只合并同一 `map_dir` 的后续实时自检结果，地图管理页显示本次定位位置、参考终点、质量指标、样本数、判定代码、累计次数和进度。
- `ROAMERX_POST_SAVE_VALIDATION_CMD` 仍可覆盖默认验证器，命令可使用 `{map_dir}` 与 `{mapping_type}` 占位符，但必须输出上述结构化 schema 才能累计成功。
