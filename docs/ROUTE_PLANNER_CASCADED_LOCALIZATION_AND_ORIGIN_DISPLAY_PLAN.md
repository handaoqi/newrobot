# 路径规划分级定位搜索与原点展示执行计划

## UKF 航点校正门控（2026-09-13 修订）

UKF 航点校正按以下顺序处理：固定 RTK 合格时优先 RTK；无固定解且 NDT score `< 0.10` 时立即选择 NDT 单源校正并记录 `ukf_high_quality_ndt`；只有 NDT score `0.10～<0.40` 且浮点 RTK 与当前校正前 UKF/LIO 位姿的 XY 偏差 `≤0.40 m` 时，才允许按观测协方差加权融合。浮点 RTK 偏差 `>0.40 m` 时，无论 NDT 分数、锚点偏好和事务类型如何，都必须剔除该 RTK 观测；若 NDT 仍满足 `<0.40` 的完整质量门，则只用 NDT，否则标记 `ukf_no_correction_sources_meet_gate`，保持当前 UKF/LIO 结果并继续后续处理，不等待 RTK 固定解。

`0.40 m` 是浮点 RTK 能否参与 UKF 航点校正的硬边界，不是自动漂移校正的触发阈值，也不放宽 fixed RTK 作为连续导航主源时已有的质量、年龄和航向要求。偏差必须在 map 坐标系下使用同一时刻或通过时间同步后的 RTK XY 与校正前 UKF/LIO XY 计算，禁止混用原始经纬度、不同时间戳或校正后的位姿。

## 阶段：行进中受限锚点校正与 FAST-LIO 健康分级（2026-09-13）

### 目标与边界

保留停车航点的一次性 `ndt` / `rtk` / `ukf` 校正事务和 float RTK `≤0.40 m` 硬门；新增仅在低速正常巡航时生效的在线 `map→lio` 锚点平滑校正。FAST-LIO 原始 `/odom/lio_odom` 不被改写，绝对观测只更新锚点。

在线校正不是“边走边追大偏差”：小中度、持续一致的绝对观测允许缓慢收敛；大偏差、LIO 数据故障或观测冲突必须停止导航并走重定位恢复。

### 在线门与来源规则

- `/localization/policy` 扩展为兼容的第五字段 `online_anchor_correction_allowed`；缺失字段按 `false` 处理。Edge 仅在普通巡航 `moving` 阶段置为 `true`；停车、最终到点、微调、转向、避障恢复、重定位、人工控制、暂停和停靠均为 `false`。
- 在线校正必须同时满足：LIO 新鲜、无 LIO 异常、无进行中的锚点平滑、融合 profile 为 nominal、LIO 实测线速度 `≤0.15 m/s`、角速度 `≤0.10 rad/s`。
- 在线 NDT 仅接受 score `<0.10`、内点率 `≥0.50`、新鲜样本和连续三帧稳定；Fixed RTK 必须通过现有位置/航向/年龄/精度门和 `1 s / ≤0.35 m` 自稳定窗口。Float RTK 永不参与在线校正。
- 仅对 XY `0.30～<1.00 m` 或可信 yaw `5～<10°` 的稳定残差启动在线校正；平滑速率限制为 `0.05 m/s`、`3°/s`，沿用单活动任务与冷却门，避免 Nav2 位姿跳变。

### FAST-LIO 健康与安全恢复

`lio_health` 分为 `healthy`、`degraded`、`fault`：

- `fault`：LIO 超过 `0.30 s` 未更新、位移突跳 `>1.50 m`、yaw 非有限、时间戳异常、单帧 yaw `>30°` 或 yaw 速率 `>60°/s`。任务立即取消导航、零速度并重定位。
- `degraded`：连续高质量 NDT 或自稳定 Fixed RTK 显示当前 LIO 与绝对地图观测的 XY `≥1.00 m` 或可信 yaw `≥10°`。不得在线追赶；发布明确诊断，Edge 安全停车后重定位。
- NDT 或 RTK 自身质量不合格只表示绝对观测不可用，不能单独判定 LIO 失真；LIO 协方差继续用于权重和诊断，而不是单独的硬故障门。

### 可观测性、测试与验收

- `/localization/decision` 和 Edge 事件记录：健康级别、故障/降级原因、NDT/RTK 原子质量、残差、在线门拒绝原因、LIO 线速度/角速度、选源、平滑进度和大偏差停车原因。
- 单元测试覆盖在线门边界、三帧稳定、Float RTK 禁止在线、单活动平滑、NDT/RTK 大偏差分流与 policy 五字段兼容。
- rosbag 回放覆盖低速直线、低速转弯、高速巡航、弱结构、Fixed RTK 跳变、Float RTK、大偏差和 LIO yaw/时间戳异常；验收要求 `/odom/nav2` 不跳变、无新增 wz 振荡、严重偏差在继续逼近前停车。

## UKF、RTK、NDT 三模式最终校正算法（2026-09-13 修订）

三种航点校正模式统一采用“先判定质量，再选择校正源，最后决定是否阻塞”的状态机。所有阈值均针对当前 LIO 锚点，RTK 偏差为 XY 距离，NDT 分数为当前新鲜匹配结果：

| 配置模式 | 条件 | 动作 | 事务结果 |
|---|---|---|---|
| `UKF` | RTK fixed 且质量/年龄/航向合格 | RTK 优先作为绝对观测；NDT 仅做一致性辅助 | `corrected_rtk` |
| `UKF` | 无 fixed；NDT score `<0.10` 且完整质量门通过 | 直接采用高质量 NDT 单源校正，不引入 float RTK | `ukf_high_quality_ndt` |
| `UKF` | 无 fixed；NDT score `0.10～<0.40`，且 RTK float 偏差 `≤0.40 m` | RTK 与 NDT 按各自观测协方差加权融合进 UKF | `corrected_ukf_fused` |
| `UKF` | RTK float 偏差 `>0.40 m`，但 NDT score `<0.40` 且完整质量门通过 | 硬拒绝 float RTK，只用 NDT 校正 | `ukf_float_outside_gate_ndt_only` |
| `UKF` | RTK float 偏差 `>0.40 m` 且 NDT score `≥0.40` 或无效 | 不做本轮校正，保持当前 UKF/LIO | `ukf_no_correction_sources_meet_gate` |
| `RTK` | RTK fixed 且质量合格 | 直接 RTK 绝对校正 | `corrected_rtk` |
| `RTK` | 非 fixed（包括任意偏差的 float）、无效或过期 | 不使用 NDT 或 float 融合，不等待 RTK 变好，保持当前 LIO/UKF 并继续后续航点处理 | `rtk_no_correction_continue` |
| `NDT` | NDT 收敛且 score `<0.40`、内点率和样本新鲜 | 直接 NDT 校正 | `corrected_ndt` |
| `NDT` | NDT score `≥0.40` 或样本无效 | 不校正；保持当前 LIO/UKF，不进入 RTK 等待 | `ndt_no_correction_continue` |

补充约束：`rtk_primary_allowed` 只控制连续导航是否允许 fixed RTK 接管，不影响航点阶段的 UKF 浮点融合；`localization_anchor_preference=rtk` 也不得绕过 float `0.40 m` 硬门。任何“不校正继续”都必须记录原因和当时的 RTK/NDT 质量快照，不得伪报为 `corrected`。到点 XY/yaw 验收继续使用校正后最新位姿；未校正放行只表示当前位姿质量未触发安全阻塞。

## RTK float 0.40 m 门控实施计划（2026-09-13）

### 当前问题

- 当前代码将 `NDT <0.08` 硬编码为 UKF 高质量 NDT 单源分支，而配置中的 `lio_primary.dynamic_covariance.ndt_good_score=0.10` 只参与观测噪声计算，未参与来源选择。
- `rtkCorrectionCandidate()` 虽会拒绝偏差 `>0.20 m` 的 float 候选，但 `ukf_float_fusion` 快捷分支只检查 `rtk_float_ready`，没有复用候选的偏差门禁，可能重新把已拒绝的 float 观测加入融合。
- `maybeCorrectUkfFused()` 未独立复核 float 偏差，也未像 NDT/RTK 单源校正一样在函数入口强制停车，调用关系变化时存在绕过安全门的风险。
- float 数据因标准差或年龄不合格、同时 NDT 无效时，事务可能停留在 `waiting_source`；该行为不符合“不因 RTK 暂时不佳无限阻塞”的共同约束。

### 参数与唯一判定口径

新增并统一使用以下参数，禁止在选择器、执行器和诊断层分别硬编码：

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `lio_primary.ukf_fusion.high_quality_ndt_score` | `0.10` | 低于该值时选择 NDT 单源校正 |
| `lio_primary.ukf_fusion.ndt_max_fitness_score` | `0.40` | NDT 可参与任何校正的严格上限；可直接复用全局 `ndt_max_fitness_score` |
| `lio_primary.ukf_fusion.float_max_residual_m` | `0.40` | float RTK 参与 UKF 融合的最大 XY 偏差，边界值允许 |

统一派生一次 `rtk_float_within_gate`：float 状态、位置有限、质量/年龄门通过、时间同步有效，并且 XY 偏差 `≤0.40 m`。来源选择、融合执行、`policy_source_ready` 和事务结果都必须使用该布尔值，不得只使用不含偏差判断的 `rtk_float_ready`。

### 开发项

- [x] 将 UKF 高质量 NDT 单源门限从硬编码 `<0.08` 改为可配置的 `<0.10`。
- [x] 将 float 候选偏差硬门统一为 `≤0.40 m`，并让 `ukf_float_fusion` 必须检查 `rtk_float_within_gate`。
- [x] 在 `maybeCorrectUkfFused()` 内再次检查 float `≤0.40 m`、NDT `0.10～<0.40` 及 `motion_phase=stationary`，形成执行层防御；RTK 与 NDT 新鲜度继续由其既有质量门验证。
- [x] float `>0.40 m` 时清除 RTK 候选并禁止其更新锚点 UKF；NDT 合格则退化为 NDT 单源，否则本轮不校正。
- [x] float 超门、过期、标准差超限或无效时，不得仅凭 float 将 `policy_source_ready` 标记为 true。
- [x] 航点一次性校正没有任何合格来源时结束事务并继续任务，禁止长期停留在 `waiting_source`；定位丢失或 LIO 异常仍走独立安全保持机制。
- [x] 保留 fixed RTK 的现有优先级、质量/年龄/航向门和连续导航主源限制，不让本次 float 门限调整影响 fixed 路径。
- [x] 诊断增加实际 float XY 偏差、配置门限、`within_gate`、拒绝原因及最终来源，确保未校正、NDT 单源和 UKF 融合可明确区分。

### 测试与验收

- [x] NDT `0.0999` 选择 NDT 单源，`0.1000` 不进入高质量 NDT 分支。
- [x] NDT `0.1000～0.3999` 且 float 偏差 `0.4000 m` 时允许 `corrected_ukf_fused`。
- [x] float 偏差 `0.4001 m` 时禁止融合；NDT 合格则得到 `ukf_float_outside_gate_ndt_only`，NDT 不合格则得到 `ukf_no_correction_sources_meet_gate`。
- [x] NDT `0.4000` 及以上不得参与 NDT 单源或 UKF 融合。
- [ ] `anchor_preference=rtk`、balanced、自动漂移校正和航点一次性事务均不能绕过 float `0.40 m` 门。
- [ ] 机器人运动中不得启动 NDT、RTK 或 UKF 融合锚点校正；停车后才允许选择和执行。
- [ ] float 过期、水平标准差超限、坐标或时间戳无效时不得参与融合，也不得造成航点无限等待。
- [ ] 回放记录必须能从诊断中还原每次来源选择：输入质量、NDT 分数、float 偏差、门限、选择结果和事务终态。

## 目标

统一“下发地图、初始化定位、主动重定位”三个入口：先执行不超过 30 秒的快速初始化搜索，只有没有任何合格候选时才进入 Scan Context / FastGICP / 全图 ICP 全局搜索。定位成功后展示最终位姿、质量依据、定位尝试点，以及地图原点和 RTK/ENU 原点。

## 定位状态机

1. 人工指定位置时，围绕人工点尝试 8 个航向和前后左右 0.3 m 局部环；失败后进入全局搜索。
2. 未指定位置时，室外先等待最多 8 秒 RTK 固定解，然后尝试当前地图最近可信位姿、建图起点 8 个航向和 0.3 m 局部环。
3. NDT 候选必须满足收敛、内点率、种子位置/航向门和连续 3 帧稳定；`matching_error < 0.01` 时提前停止。没有达到 0.01 但存在普通健康候选时，提交快速阶段排名最优者。
4. RTK 必须为新鲜固定解且完成 ENU→map 对齐；锚定 FAST-LIO 后连续 3 帧 LIO↔RTK XY 漂移严格小于 0.30 m，才确认初始化成功。
5. 快速阶段没有合格候选时，才执行 Scan Context Top-K、FastGICP 几何验证、全图 ICP 和连续 3 帧 NDT 复核。

## 接口与页面

- 定位命令携带 `scene_scope`、`coordinate_mode`；外部 `global` 兼容映射到 `quick_then_global`。
- 定位进度返回阶段、候选数量、提前停止原因、最优来源、NDT 误差和 RTK 漂移。
- 页面解释三个按钮的定位算法；紫色图标只表示已经通过质量门并完成接管的最终定位结果。
- 增加“隐藏/显示定位尝试点”按钮，同时控制临时候选点和紫色结果点。
- 地图标记并显示 `map(0,0)`、RTK/ENU 原点在 map 中的偏移、经纬高、ENU→map 旋转；`map.yaml origin` 单独标为栅格左下角。

## 验收

- NDT `0.0099`提前停止，`0.0100`继续搜索；RTK漂移`0.299 m`连续 3 帧成功，`0.300 m`不成功。
- 快速搜索存在合格候选时不调用全局服务；全部失败后最多进入一次全局阶段。
- RTK固定但精度、航向、时间或漂移不合格时不得完成初始化。
- FAST-LIO 原始里程计连续，绝对修正只通过平滑调整 `map←lio` 锚点完成。
- 定位失败或 FAST-LIO 接管失败不生成紫色结果点。

## 执行结果（2026-09-06）

- [x] Edge Agent 实现 `quick_then_global`：快速阶段限制 30 秒，人工点/可信位姿/建图原点候选失败后只调用一次全局服务。
- [x] NDT 健康候选 `matching_error < 0.01` 且连续稳定 3 帧时提前停止，其余候选标记为跳过；普通合格候选仍按质量排序提交。
- [x] RTK 初始化增加新鲜固定解、有效航向及对齐后 FAST-LIO XY 漂移 `< 0.30 m` 连续 3 帧验证。
- [x] 定位节点诊断增加 `rtk_drift`，发布 FAST-LIO 对齐位姿与 RTK 在 map 坐标中的 XY 偏差。
- [x] 下发地图、初始化定位、主动重定位统一接入快速搜索与全局回退；旧 `global` 命令保持兼容。
- [x] 路径规划页面增加算法说明、候选计数、提前停止/全局阶段、RTK 漂移、定位尝试点显隐。
- [x] 紫色结果点仅在严格 NDT 最优阈值或 RTK 漂移验证通过时生成，移除仅凭 `relocalized` 状态生成结果点的逻辑。
- [x] 地图详情接口与页面增加 map 原点、栅格原点、锁定 RTK/ENU 原点及 ENU→map 对齐信息。
- [x] 验证完成：Edge Agent 373 项、前端 94 项、后端相关 13 项测试通过；前端生产构建和 ROS `localization` 包编译通过。

## 统一三场景定位初始化与重定位流程（2026-09-15 合并方案）

### 当前检查结论

现有实现的初始化、地图下发、主动重定位和定位丢失恢复尚未完全统一：

- 巡检任务启动的室外/过渡分支仍可能先验证 fixed RTK，失败后才进入建图原点和航点搜索。
- 主动重定位的室内路径可能优先使用最近航点、可信历史位姿或人工种子，不固定执行 NDT 原点优先。
- 地图下发前端的 `progressiveLocalization` 仍可能先调用 RTK 初始化。
- `progressive_relocalize()` 已具备“建图原点及周边 → 航点 → 全局搜索”，但不是所有入口的唯一编排器。
- RTK 初始化当前可能先以 RTK 设置初始位姿，再做 NDT 交叉验证，顺序与统一目标相反。
- 定位丢失后的自动恢复可能先等待 RTK，fixed 后直接调用 `set_initial_pose_from_rtk()`；第二次及后续恢复没有稳定读取当前航点 `localization_mode`。
- 自动恢复的 UKF 路径没有完整复用正常航点的 NDT/RTK/float 融合门控；自愈诊断仍有 float `0.20 m` 旧门限，必须统一为 `0.40 m`。

底层已经具备 FAST-LIO + IMU 连续主定位、NDT/RTK/UKF 绝对校正、候选排名和接管验证能力，缺口主要在入口编排、恢复顺序和结果状态统一。

### 统一目标流程

所有入口（任务启动、地图下发、主动重定位、定位丢失、漂移超阈值）统一执行：

1. 停车并确认 FAST-LIO + IMU 正常输出。
2. 读取地图约束：`indoor + local_only`、`outdoor + rtk_fixed` 或 `transition + rtk_fixed`。
3. 优先进行 NDT 搜索：建图原点、原点 8 个航向、0.3/0.6/1.0 m 周边候选，随后才搜索路线航点和全局候选。
4. 按 NDT 收敛、score、内点率、新鲜度和连续稳定帧筛选候选。
5. 选出当前阶段最优 NDT 候选并提交 `/initialpose`。
6. 等待连续新鲜 FAST-LIO 正常帧，验证 `map→lio` 锚点接管，定位状态更新为 `normal`，连续源标记为 `lio_imu`。
7. 根据地图场景和当前/首个航点的 `localization_mode` 执行 RTK、UKF 或 NDT 二次定位校正。
8. 重新读取校正后的 FAST-LIO + IMU 位姿，作为导航和航点验收位姿。

统一编排器阶段固定为：

`precheck_stop → ndt_searching → ndt_best_selected → ndt_best_committing → fast_lio_imu_handoff → secondary_correction → final_pose_verification → localization_ready/localization_failed`

所有阶段必须记录时间、候选编号、NDT 质量、RTK/UKF 质量、接管结论和最终来源；任何阶段不得发送运动指令。

### 场景和航点模式

首航点的 `localization_mode` 只决定 NDT 提交后的二次定位校正，不再决定初始候选搜索顺序：

- 室内 `local_only`：NDT 提交后再次执行一次新鲜静止 NDT 二次定位校正并完成；不得读取或等待 RTK。首航点配置为 `rtk` 时后端拒绝或归一化为 NDT；`ukf` 只允许室内 UKF 状态校正，不引入 RTK。
- 室外/过渡 `rtk_fixed`：`ndt` 模式完成 NDT 提交后结束；`rtk` 模式随后验证 fixed RTK 并执行 RTK 锚点校正；`ukf` 模式按 fixed RTK、NDT 和 float RTK 门控执行 UKF 策略。

UKF 唯一口径：

- fixed RTK 合格：RTK 优先；
- NDT `<0.10`：NDT 单源；
- `0.10≤NDT<0.40` 且 float RTK 偏差 `≤0.40 m`：UKF 加权融合；
- float RTK 偏差 `>0.40 m`：无论 NDT、锚点偏好或事务类型都禁止引入；
- 无合格来源：保持 FAST-LIO + IMU，记录未校正放行，不无限等待 RTK。

这里的“二次 NDT 定位校正”不是重复提交同一个初始候选，而是完成首次 NDT 最优位姿提交并验证 FAST-LIO 接管后，使用新的静止点云重新执行一次局部 NDT 精配准/验收。二次 NDT 只能在停车、样本新鲜且质量门通过时更新 `map→lio` 锚点；若二次 NDT 不合格，保持首次提交后的 FAST-LIO + IMU，不得回退到旧候选或等待 RTK。

### 定位丢失与第二次恢复策略

定位丢失时必须在事件发生瞬间保存当前航点快照：航点索引、轮次、`localization_mode`、`localization_anchor_preference`、场景、地图版本和任务 ID。第二次及后续恢复每次重新读取当前航点配置，不能固定切换成 RTK，也不能沿用第一次恢复的旧模式。

恢复阶段和质量门与初始化完全相同，但候选顺序按当前位置优先：

1. 当前航点及其有限周边；
2. 最近可信 FAST-LIO 位姿；
3. 当前航点前后相邻点；
4. 建图原点及附近；
5. Scan Context / FastGICP / 全图搜索。

每次恢复均执行“NDT 最优提交 → FAST-LIO + IMU 接管 → 当前航点 RTK/UKF/NDT 二次定位校正”。

- 当前航点为 `ndt`：只做 NDT 单源恢复，不等待 RTK。
- 当前航点为 `rtk`：NDT 成功后才验证 fixed RTK；非 fixed 时结束本次 RTK 事务并以 NDT/FAST-LIO 结果继续，不等待 RTK 变好。
- 当前航点为 `ukf`：复用正常 UKF 的 fixed RTK、NDT `<0.10`、float `≤0.40 m` 规则。

恢复等级为：当前航点局部候选 → 扩大当前航点/相邻点候选 → 建图原点及附近 → 全局搜索。全局搜索成功后仍必须经过 NDT 提交、FAST-LIO 接管和当前航点 RTK/UKF/NDT 二次定位校正。

仅当室外/过渡场景的局部和全局 NDT 均无候选、且 fixed RTK 全部质量门通过时，才允许使用明确标记的 `rtk_fallback_without_ndt` 安全兜底；室内禁止该路径。

### 入口改造要求

- `TaskExecutor.initialize_before_navigation()`：删除室外先 RTK 旁路，统一执行 NDT 搜索、FAST-LIO 接管和首航点 RTK/UKF/NDT 二次定位校正后再发送第一个 Nav2 目标。
- `nav.relocalize` 和 `nav.initial_pose`：所有 `progressive`、`quick_then_global`、`global`、人工点和历史兼容种子进入统一编排器；保留协议兼容字段，但不得绕过 NDT 提交。
- `map.activate`：切图成功后由 Edge 自动执行统一定位；前端只展示和等待 Edge 阶段，不再重复调用 RTK/NDT。
- 自动恢复、漂移恢复和主动重定位共用同一结果结构，不能再单独实现直接 RTK 或旧 UKF 门控。

结果至少包含：`initial_ndt_commit`、`secondary_correction`、`continuous_source=lio_imu`、`map_lio_anchor_generation`、当前航点策略和完整质量证据。

### 验收要求

- 三场景 × 三模式 × 四入口（任务启动、地图下发、主动重定位、自动恢复）阶段顺序一致。
- 第二次及后续恢复仍使用当时当前航点的 `ndt/rtk/ukf` 配置。
- NDT 最优提交始终发生在 RTK/UKF/NDT 二次定位校正之前。
- 室内不读取、不等待 RTK；室外/过渡无 fixed RTK 不阻塞 NDT 初始化。
- NDT `<0.10` 只用 NDT；`0.10≤NDT<0.40` 且 float `≤0.40 m` 才允许融合；`0.4001 m` 必须拒绝。
- FAST-LIO 接管失败不得伪报定位成功，所有失败都要返回 NDT/RTK/UKF 具体数据和结论。
- 恢复前无非零速度命令，恢复成功后的连续定位源为 `lio_imu`。

### 前端路径规划页显示与编排（2026-09-15 补充）

- 路径规划页、值守页和其他地图入口统一展示固定阶段链：`NDT 原点/附近搜索 → 提交最优 NDT → FAST-LIO + IMU 接管 → RTK/UKF/NDT 二次校正 → 最终位姿验收`。
- 前端不得再根据室外 `rtk_fixed` 元数据先下发 `seed_source=rtk`；初始化请求统一进入渐进 NDT 编排，航点 `localization_mode` 只作为二次校正策略透传给 Edge。
- 阶段卡片必须异步显示候选编号、NDT score/内点率、最优点提交结果、FAST-LIO 接管结论和二次校正来源；未收到阶段时间时明确显示“阶段时间未上报”。
- 地图下发、初始化定位、主动重定位和任务启动页面共用同一进度文案，避免出现“RTK 初始位姿”或“手选位姿绕过 NDT”的误导性提示；人工初始位姿仅保留为显式操作入口。
- 前端测试契约同步验证 NDT 优先顺序、室内跳过 RTK、模式参数透传及二次校正阶段展示。

当前执行检查点：前端已切换 NDT 优先并透传航点模式；Edge 已将旧 `nav.initial_pose(seed_source=rtk)` 兼容请求改写为渐进 NDT 搜索。剩余验收为更新旧 RTK 优先测试、补齐 `map.activate` 的 Edge 内部自动编排及二次校正完整结果字段。
