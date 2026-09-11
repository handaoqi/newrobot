# 路径规划分级定位搜索与原点展示执行计划

## UKF 航点校正门控（2026-09-11）

UKF 航点校正按以下顺序处理：固定 RTK 合格时优先 RTK；无固定解且 NDT score `< 0.08` 时立即选择 NDT 校正并记录 `ukf_high_quality_ndt`；浮点 RTK 与当前位置偏差不超过 `0.20 m` 且 NDT score `< 0.40` 时按观测噪声权重融合；其他 NDT score `< 0.40` 的候选可单独校正。当浮点 RTK 偏差 `> 0.20 m` 且 NDT score `>= 0.40` 时，标记 `ukf_no_correction_sources_meet_gate`，保持当前 UKF/LIO 结果并继续后续处理，不等待 RTK 固定解。该放行仅适用于 UKF 航点校正，不改变 RTK 固定解作为连续导航主源的安全门槛。

## UKF、RTK、NDT 三模式最终校正算法（2026-09-11）

三种航点校正模式统一采用“先判定质量，再选择校正源，最后决定是否阻塞”的状态机。所有阈值均针对当前 LIO 锚点，RTK 偏差为 XY 距离，NDT 分数为当前新鲜匹配结果：

| 配置模式 | 条件 | 动作 | 事务结果 |
|---|---|---|---|
| `UKF` | RTK fixed 且质量/年龄/航向合格 | RTK 优先作为绝对观测；NDT 仅做一致性辅助 | `corrected_rtk` |
| `UKF` | 无 fixed；RTK float 与 LIO 偏差 `≤0.20m`，且 NDT score `<0.40` | RTK 与 NDT 按各自观测协方差加权融合进 UKF | `corrected_ukf_fused` |
| `UKF` | 无 fixed；NDT score `<0.08` | 直接采用高质量 NDT 校正，不等待 RTK | `ukf_high_quality_ndt` |
| `UKF` | RTK float 偏差 `>0.20m` 且 NDT score `≥0.40` | 不做本轮校正，保持当前 UKF/LIO | `ukf_no_correction_sources_meet_gate` |
| `RTK` | RTK fixed 且质量合格 | 直接 RTK 绝对校正 | `corrected_rtk` |
| `RTK` | RTK float 与 LIO 偏差 `≤0.20m` | 临时按 UKF 观测融合方式处理浮点 RTK，禁止将其作为 fixed 主源 | `corrected_ukf_fused` |
| `RTK` | 单点解、无效/过期解，或 float 偏差 `>0.20m` | 不等待 RTK 变好，保持当前 LIO/UKF 并继续后续航点处理 | `rtk_no_correction_continue` |
| `NDT` | NDT 收敛且 score `<0.40`、内点率和样本新鲜 | 直接 NDT 校正 | `corrected_ndt` |
| `NDT` | NDT score `≥0.40` 或样本无效 | 不校正；保持当前 LIO/UKF，不进入 RTK 等待 | `ndt_no_correction_continue` |

补充约束：`rtk_primary_allowed` 只控制连续导航是否允许 fixed RTK 接管，不影响航点阶段的 UKF 浮点融合；任何“不校正继续”都必须记录原因和当时的 RTK/NDT 质量快照，不得伪报为 `corrected`。到点 XY/yaw 验收继续使用校正后最新位姿；未校正放行只表示当前位姿质量未触发安全阻塞。

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
