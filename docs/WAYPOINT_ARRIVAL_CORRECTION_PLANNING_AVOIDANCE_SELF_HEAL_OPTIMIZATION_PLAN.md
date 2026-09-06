# 航点到达、定位校正、路径规划、动态避障与自愈优化计划

## 1. 文档状态

- 状态：分阶段实施中（本轮已完成 P0 单航点边界、P1 算法插件注册；到点事务/恢复仲裁仍待后续阶段）。
- 适用范围：平台路径配置、边缘任务执行器、定位节点、Nav2 导航栈、状态展示与语音播报。
- 目标：把“到点、校正、转向、下一段规划、避障、自愈”收敛成可验证的单一状态机，消除航点漏报、重复播报、错误跳点、配置名实不符以及多套恢复流程互相抢占等问题。
- 本文同时记录实施状态；未标记为“已完成”的项目仍不可视为生产闭环。

### 1.1 本轮执行步骤（2026-09-06）

| 步骤 | 状态 | 实施内容 |
| --- | --- | --- |
| 1 | 已完成 | Edge `_batch_end_index()` 固定按单航点 dispatch，不再默认合并为 `NavigateThroughPoses`。 |
| 2 | 已完成 | Nav2 planner 注册 `ThetaStar` 与 `NavFn`；`NavFn.use_astar=true` 对应 A*，ThetaStar 对 NavFn 路径做栅格视线松弛。 |
| 3 | 已完成 | Nav2 controller 注册 `FollowPath`（MPPI）与 `RPP`（Regulated Pure Pursuit）两个独立插件。 |
| 4 | 已完成 | Edge selector 映射 `theta_star→ThetaStar`、`navfn→NavFn`、`mppi→FollowPath`、`rpp→RPP`，并下发对应参数。 |
| 5 | 已完成（静态/构建） | Python selector 单测、XML/YAML 解析与两个 ROS 包编译安装已通过；真实运行时 selector 读回需在设备上执行。 |
| 6 | 待执行 | 到点确认新鲜位姿/连续多帧、完整 `LegProfile`、恢复仲裁器、dwell 和统一幂等键。 |

本轮明确不合并航点：每个航点完成校正和业务确认后，才以校正后的新鲜位姿生成下一段；`NavigateThroughPoses` 保留为底层能力但不作为默认业务路径。

## 2. 结论摘要

当前系统的主体链路已经形成：平台保存路线快照，Edge 按路线分批，下发 Nav2 action；Nav2 以 1 Hz 生成全局路径、MPPI 以 20 Hz 跟踪，局部代价地图和 Collision Monitor 负责近场障碍；每个需要停留的批次结束后，Edge 停车、切换静止定位策略、等待校正、播报，再朝下一点转向并重新下发目标。

需要优先处理的不是继续放宽到点阈值，而是统一以下语义和所有权：

1. “经过航点”与“确认到达航点”必须分开。Nav2 在半径 1.0 m 内移除中间点，只能代表通过，不应触发要求停车、校正、动作或播报的到点事件。
2. 到点确认必须使用定位校正完成后的新鲜位姿，并要求连续稳定样本，不能使用校正前或上一航点遗留的位姿。
3. 每一段路线必须绑定不可变的“段配置”，配置变化即切批；目前只按部分属性切批，会使批次中间的规划器、控制器或避障设置被忽略。
4. 页面中的 `theta_star/navfn` 与实际 Nav2 行为不一致；当前只有 NavFn 插件，所谓切换实质是同一插件的 Dijkstra/A* 参数切换。MPPI/RPP 也不存在真实的双插件切换。
5. `avoidance_to_next=false` 不能同时关闭最后一道碰撞急停。动态绕行、减速和硬急停应拆成三个独立安全层。
6. Nav2 行为树恢复与 Edge 自愈需要一个恢复仲裁器，确保任一时刻只有一个恢复动作拥有控制权。
7. 配置下发必须有能力协商、应用确认和读回校验，失败时不能带着旧配置继续导航。

建议按“可观测性与名实修正 → 到点/校正事务 → 段配置与转向 → 自愈仲裁 → 灰度验证”实施，避免一次性改动整个导航链路。

## 3. 当前实现基线

### 3.1 航点数据

平台在 `platform/backend/monitoring/services/task_service.py` 中将路线航点归一化为以下主要字段：

| 字段 | 当前含义 | 当前问题 |
| --- | --- | --- |
| `sequence` | 航点顺序 | 协议要求连续，但运行事件仍需携带轮次和生成号避免重复 |
| `map_point_number` / `waypoint_id` | 页面点号和持久标识 | 语音、执行索引和显示点号容易混用 |
| `x/y/yaw` | 地图坐标和方向 | 中间点可能使用路径方向覆盖点击 yaw |
| `require_yaw` | 是否严格要求目标方向 | Nav2 默认 yaw 容差实际为 3.14 rad，仅要求时才使用 0.25 rad |
| `dwell_seconds` | 到点停留时间 | 当前只影响切批，Nav2 实际仍使用全局固定 200 ms 等待 |
| `actions` | 如 `snapshot` | 已进入快照，但未找到完整的动作执行闭环 |
| `localization_mode` | `ndt`、`rtk`、`ukf` | 能影响静止/运动定位策略，应纳入段配置边界 |
| `local_controller` | `mppi` 或 `rpp` | 分别映射 `FollowPath`（MPPI）和 `RPP` 插件 |
| `global_controller` | `theta_star` 或 `navfn` | 分别映射 `ThetaStar` 和 `NavFn(use_astar=true)` |
| `avoidance_to_next` | 下一段是否避障 | 当前会连硬急停一起关闭，风险过大 |
| 语音模板 | 到点播报内容 | 阻塞策略是全局配置，不是航点显式语义 |

协议层已经校验顺序、坐标数值、布尔字段、控制器取值和 0～3600 秒停留时间，但没有表达“航点类型”“到达策略”“播报是否阻塞”和分层避障开关。

### 3.2 当前执行链路

```text
平台路线快照
  → Edge 选择任务起点和执行方向
  → Edge 计算本批次终点
  → 设置定位/规划/控制/避障参数
  → 单点：FollowWaypoints（当前默认每个航点独立发送）
  → Nav2 全局规划与 MPPI 局部跟踪
  → 批次终点停车并等待定位校正
  → Edge 检查到点距离
  → 生成 waypoint_reached、动作和语音事件
  → 面向下一航点转向
  → 下发下一批次并重新生成全局路径
```

`edge-agent/roamerx_edge/ros_adapter.py` 在目标数大于 1 时发送 `NavigateThroughPoses`，单点时发送 `FollowWaypoints`。这意味着两条路径使用的行为树和 Waypoint Follower 行为并不完全相同，应当由 Edge 统一航点业务语义，而不能依赖 action 名称推断“已经完成停留”。

### 3.3 当前切批规则

`edge-agent/roamerx_edge/task_executor.py::_batch_end_index()` 当前会在以下场景切批：

- 室外或充电/停靠任务，通常按单点执行；
- `require_yaw=true`；
- `dwell_seconds>0`；
- 存在到点语音；
- 定位模式变化；
- 地图分段边界。

当前实现已取消普通室内连续点的默认合并；每个航点独立 action，避免定位校正、恢复和控制器配置跨点失效。`NavigateThroughPoses` 仍可由底层接口支持，但不参与本计划的默认业务执行。

当前缺口：批次不会因为 `global_controller`、`local_controller` 或 `avoidance_to_next` 变化而必然切分。配置只在批次发送前应用，所以批次中间航点的配置可能从未生效。

### 3.4 当前到点判定是四层叠加

#### 第一层：Nav2 Goal Checker

当前 `SimpleGoalChecker` 主要配置为：

- XY 容差：0.35 m；
- 默认 yaw 容差：3.14 rad，相当于不要求方向；
- `require_goal_yaw` 时 yaw 容差：0.25 rad；
- `stateful=true`。

它判断的是 Nav2 action 可结束，不等价于业务层“校正后确认到点”。

#### 第二层：多航点通过判定

`navigate_through_poses_w_replanning_and_recovery.xml` 使用 `RemovePassedGoals radius="1.0"`。中间点进入 1.0 m 半径即可被移除，因此反馈中的进度只表示“通过该点”，不能表示“精确停车到达”。

#### 第三层：批次终点确认

Edge 在批次结束后：

1. 下发零速并确认停车；
2. 切换静止定位策略；
3. 等待定位校正结束；
4. 检查点击点误差；
5. 必要时对同一点重新接近，最多约两次。

室外还会检查 LIO 与固定解 RTK；但当前室内 `_arrival_within_tolerance()` 基本直接通过，导致室内批次终点缺少校正后的点击点复核。

#### 第四层：整条路线终点

- 普通终点运行时容差约 0.45 m；
- 停靠终点约 0.08 m，并要求约 5° yaw；
- 室外还要通过 RTK 一致性检查。

因此曾经出现的“最终误差 0.42 m，大于 0.35 m”对普通终点应由业务层 0.45 m 容差接纳，但停靠和室外精确终点仍不应放宽。继续统一调大 Nav2 容差会掩盖校正跳变或真实偏航，不是正确修复方法。

### 3.5 当前定位校正

Edge 通过 `/localization/policy` 发布 `moving/stationary` 和 `ndt/rtk/ukf` 策略。定位节点根据策略选择可用绝对源，更新的是 `map→lio_odom` 关系，不是驱动机器人实际移动。

校正目标可表达为：

```text
target(map→lio_odom) = target(map→base_link) × inverse(current(lio_odom→base_link))
```

当前平滑校正速率约为 0.25 m/s、8°/s，平移剩余约 0.01 m、旋转剩余约 0.25°时完成。任务执行器会在 `correction_smoothing_active=true` 时保持停车。室内等待窗口约 12 秒；室外干净样本可较快通过，发生校正时同样需要更长稳定窗口。

主要风险是状态没有按“任务 + 轮次 + 航点 + 校正代次”形成完整事务，迟到的定位状态可能误解锁下一航点；到点检查也未强制要求位姿时间戳晚于本次校正完成时间。

### 3.6 当前转向和下一段规划

到点后，Edge 优先使用校正后的当前位置指向下一点击点计算期望 yaw，缺少当前位姿时才退回点击点到点击点的方向：

```text
desired_yaw = atan2(next_y - corrected_y, next_x - corrected_x)
```

当距离过近或方向误差小于约 0.35 rad（20°）时跳过转向；否则优先使用遥控速度原地转向，典型角速度约 0.40 rad/s、周期 0.1 s、超时 20 s，另有 Nav2 转向后备路径。转向完成后才持久化下一索引并发送新 action，因此新一段会从校正后位姿重新规划。

风险点：转向结束可能只凭单次角度样本；遥控转向与 Nav2/Cancellation/Collision Monitor 的控制权边界不够明确；所有大于 20°的方向差采用同一种动作，容易在狭窄环境中产生突兀切换。

### 3.7 当前全局与局部算法

#### 全局规划

Nav2 当前仅注册：

```yaml
planner_plugins: ["GridBased"]
GridBased:
  plugin: "navigo_navfn_planner/NavfnPlanner"
```

行为树也固定使用 `planner_id="GridBased"`，没有真正使用 Planner Selector。

Edge 所谓全局规划切换实际为：

| 页面值 | 当前实际配置 | 真实算法 |
| --- | --- | --- |
| `theta_star` | `GridBased.use_astar=false` | NavFn/Dijkstra |
| `navfn` | `GridBased.use_astar=true` | NavFn/A* |

系统并未加载真正的 Theta* 插件。因此这是高优先级的名实错误，页面、任务快照和运行日志会给出错误认知。

全局规划频率为 1 Hz。室外配置可优先直线路径并允许失败回退，室内以栅格规划为主。全局代价地图以静态地图和膨胀为主，实时激光障碍主要在局部层处理。

#### 局部控制

当前只注册 MPPI `FollowPath`，控制频率 20 Hz，预测域为 `56 × 0.05 = 2.8 s`，采样批量 1000，常规上限约 `vx=0.30 m/s`、`wz=0.35 rad/s`。接近终点还有较慢的速度配置。

`rpp` 只是兼容别名，仍会归一化为 MPPI，不是实际插件切换。因此前端不应继续展示无法兑现的本地控制器选择，除非真正注册第二插件并接入 Controller Selector。

### 3.8 当前避障

局部滚动代价地图约 8 m × 8 m、0.05 m 分辨率、5 Hz 更新，使用 `/laser_scan`，障碍标记约 3 m、清除射线约 4 m。MPPI 的 CostCritic 根据局部代价图绕行；Collision Monitor 位于速度输出链路末端，负责近场减速和硬停。

当前 `avoidance_to_next=false` 会同时关闭：

- 局部障碍层；
- MPPI CostCritic；
- Collision Monitor 减速区；
- Collision Monitor 急停区。

这把“允许不绕行”和“允许撞上障碍”混成了一个开关，必须拆分。

### 3.9 当前自愈

Nav2 行为树已经具备清除全局/局部代价图、旋转约 1.57 rad、后退 0.30 m、等待 2 秒等恢复动作，外层最多重试 6 次。多点行为树还会调用 `/reinitialize_global_localization`，单点行为树没有完全一致的重定位步骤。

Edge 另有障碍和无进展监测：

- 周期约 0.5 秒检查；
- 近距离物理障碍或请求速度与实际速度长期不一致时触发；
- 约 5 秒无明显进展时升级；
- 前两次清局部代价图并保持目标；
- 后续可取消目标、短距离低速后退，再尝试侧向绕行点并重发原目标；
- 定位丢失时保存当前航点，取消并停车，依次尝试 RTK、可信历史位姿和 NDT 重定位，恢复后重发当前航点。

现有保护包括 action 生成号，旧 action 的迟到成功回调不能推进路线，这是应保留的设计。

当前最大问题是 Nav2 和 Edge 都可能同时执行恢复；临时侧移点未必经过完整全局路径、地图边界和机器人足迹验证；定时后退也必须先验证后方净空并始终经过碰撞安全链路。

## 4. 目标架构与核心不变量

### 4.1 目标流程

```text
读取不可变路线快照
  → 生成带能力校验的 LegProfile
  → 按 LegProfile 和业务停点切批
  → 原子应用配置并读回确认
  → 可选的安全预转向
  → 下发带 generation 的 Nav2 action
  → 中间点只记录 waypoint_passed
  → 终点进入停车/稳定/校正事务
  → 使用校正后新鲜位姿确认到达
  → 执行动作、停留和语音
  → 计算下一段方向并重新规划
```

### 4.2 必须保持的不变量

1. 同一时刻最多存在一个有效导航 action、一个运动控制所有者和一个恢复所有者。
2. 航点索引只能由匹配当前 `leg_generation` 的成功事件推进。
3. 要求停车的航点只可由“校正完成后的新鲜稳定位姿”确认。
4. `waypoint_passed` 不得触发要求停车的动作、停留和到点语音。
5. 每一段的配置必须在 action 被接受前完成应用和读回；任何失败必须阻止发车。
6. 碰撞硬急停默认始终开启。只有具备单独安全许可的停靠末段才允许受限降级，并必须限制速度和距离。
7. 规划器/控制器的对外名称必须等于真正加载的插件或算法，不允许静默回退。
8. 所有事件、语音、动作和状态持久化都使用相同幂等键。

建议幂等键：

```text
(task_execution_id, round_index, waypoint_id, leg_generation, event_type)
```

## 5. 航点与路段数据模型优化

### 5.1 明确航点类型

新增 `arrival_policy`：

- `pass_through`：允许连续通过，不停车、不做静止校正；
- `stop_and_confirm`：停车、稳定、校正并确认点击点；
- `precision`：严格 XY/yaw，用于窄门、设备操作等；
- `dock`：由充电桩接触/充电状态和精确位姿共同确认。

历史数据迁移建议：

- 有 `dwell_seconds`、语音、动作或 `require_yaw` 的点自动迁移为 `stop_and_confirm`；
- 充电桩终点迁移为 `dock`；
- 其余中间点迁移为 `pass_through`；
- 路线最后一点至少为 `stop_and_confirm`。

### 5.2 引入不可变 LegProfile

每个“从当前点到下一点”的路段生成：

```text
LegProfile = {
  localization_mode,
  global_planner_id,
  local_controller_id,
  detour_enabled,
  collision_slowdown_enabled,
  collision_stop_enabled,
  speed_profile,
  goal_checker_id,
  arrival_policy
}
```

切批条件改为：

```text
当前点要求业务停留
OR 地图/室内外/停靠边界变化
OR LegProfile(current) != LegProfile(next)
```

这样可以保证一个 Nav2 action 生命周期内配置不变，不需要在 action 中途热切换关键插件或安全参数。

### 5.3 修复停留与动作语义

- `dwell_seconds` 由 Edge 在 `arrival_confirmed` 后执行，使用单调时钟，误差目标不超过 ±0.2 秒；不要依赖全局固定 200 ms 的 Waypoint Follower 等待。
- `actions` 必须有执行器注册表、超时、幂等键、成功/失败策略和状态事件；无法识别的动作在任务开始前拒绝，不要运行中忽略。
- 语音增加 `speech_mode=blocking|non_blocking|disabled`，优先按航点配置，路线级配置只提供默认值。

## 6. 到达状态机

### 6.1 状态定义

```text
TARGET_DISPATCHING
  → TARGET_ACCEPTED
  → NAVIGATING
  → WAYPOINT_PASSED             # 仅连续中间点
  → ARRIVAL_PENDING_SETTLE      # 要求停车的批次终点
  → ARRIVAL_CORRECTING
  → ARRIVAL_VERIFYING
  → ARRIVAL_CONFIRMED
  → DWELLING / RUNNING_ACTIONS / SPEAKING
  → DEPARTURE_ALIGNING
  → NEXT_LEG_DISPATCHING
```

失败分支统一进入：

```text
RECOVERY_REQUESTED → RECOVERING → RETRY_SAME_LEG
                               ↘ SAFE_HOLD / MANUAL_REQUIRED
```

### 6.2 分层到达条件

| 类型 | 判定建议 |
| --- | --- |
| `pass_through` | 使用路径投影、沿轨进度和横向误差；可保留 Nav2 的移除半径，但事件名只能是 `waypoint_passed` |
| `stop_and_confirm` | Nav2 完成 + 速度稳定 + 静止校正完成 + 点击点 XY 连续稳定通过 |
| `precision` | 上述条件 + 严格 yaw + 更小 XY 阈值，失败时有限次数物理重新接近 |
| `dock` | 上述条件 + 停靠/接触/充电反馈；不能只靠地图坐标判断 |

建议初始参数保持现有尺度并按场测调整：

- 普通业务终点 XY：0.45 m；
- Nav2 控制结束 XY：0.35 m；
- 要求方向 yaw：0.25 rad；
- 停靠：XY 0.08 m、yaw 约 5°；
- 到点位姿连续 3 个样本通过；
- 线速度和角速度连续保持在停车阈值内 0.5～1.0 秒；
- 校正后位姿时间戳必须晚于 `correction_completed_at`。

这些阈值要按点型配置，不能用一个全局值覆盖室内、室外和停靠。

### 6.3 校正后物理复核

定位校正只改变坐标估计。若校正后机器人距离点击点仍超限，系统必须区分：

- `FRAME_CORRECTION_REQUIRED`：坐标系需要平滑修正，机器人不动；
- `PHYSICAL_REAPPROACH_REQUIRED`：机器人真实未到点击点，需要重新规划接近；
- `ARRIVAL_UNCERTAIN`：定位质量不足，保持停车而不是盲目移动。

室内也要执行校正后复核。建议普通点最多物理重试 2 次；精确点或终点重试失败后进入 `SAFE_HOLD`；是否允许非终点失败继续必须成为路线策略，禁止隐藏的 fail-open。

## 7. 定位校正事务

### 7.1 事务状态

每次业务停点创建独立事务：

```text
STATIONARY_REQUESTED
  → SOURCE_READY
  → CORRECTION_STARTED（没有必要校正时可标记 NOOP）
  → CORRECTION_COMPLETED
  → FRESH_POSE_ACQUIRED
  → POSE_CONFIRMED
```

所有定位状态必须携带或由 Edge 关联：

- `task_execution_id`；
- `waypoint_id`；
- `leg_generation`；
- `correction_generation`；
- 源数据时间戳和接收时间；
- 定位源、样本年龄、NDT fitness/inlier、RTK fix/std、UKF 决策；
- 校正前后增量和剩余误差。

无法匹配当前事务或时间早于事务开始的消息只记录诊断，不允许推进状态。

### 7.2 时间迟滞而非单纯放宽阈值

LIO 主定位模式对短时 NDT/VGICP 延迟应使用健康状态迟滞：

- 短时超时：继续使用 LIO 推算，状态为 `DEGRADED_HOLDOVER`；
- 连续多个绝对校正周期异常或创新量越界：进入 `UNHEALTHY`；
- 恢复时要求连续健康样本，避免健康/故障抖动；
- 大位姿创新仍必须触发拒绝或安全保持，不能靠扩大超时阈值掩盖跳变。

到点确认期间若定位质量降级，保持停车并等待有限时间；超时转入恢复仲裁器。

## 8. 转向与下一段全局路径

### 8.1 转向策略分级

建议根据方向误差和场地空间使用三级策略：

- 小于 10°：不单独转向，由局部控制器自然进入下一段；
- 10°～60°：使用控制器预对齐或低速弧线起步；
- 大于 60°：在确认足迹旋转净空、定位健康、唯一控制权后执行受控原地转向。

现有约 20°单阈值可作为过渡值，但最终应消除阈值附近频繁切换。完成判定要求连续 2～3 个 yaw 样本在容差内，且角速度已经下降。

### 8.2 重新规划顺序

必须严格执行：

1. 当前航点 `ARRIVAL_CONFIRMED`；
2. 动作、停留和阻塞语音结束；
3. 使用校正后新鲜位姿计算下一段朝向；
4. 应用下一段 LegProfile 并读回；
5. 可选预转向；
6. 记录新的 `leg_generation`；
7. 下发 action；
8. action 接受后才进入 `NAVIGATING`。

验收时应证明新全局路径的时间戳晚于本次校正完成时间，且路径起点与校正后位姿的偏差不超过约 0.25 m。

## 9. 规划器和控制器真实切换

### 9.1 P0 推荐方案：先修正名称

在未引入新插件前，把全局算法公开值改为：

- `dijkstra` → `GridBased.use_astar=false`；
- `astar` → `GridBased.use_astar=true`。

旧值迁移：

- 旧 `theta_star` 按当前真实行为迁移为 `dijkstra`；
- 旧 `navfn` 按当前真实行为迁移为 `astar`；
- 页面明确显示“NavFn Dijkstra / NavFn A*”。

本地控制器只显示 `mppi`，保留 `rpp` 仅作输入兼容并在日志中警告迁移，不继续让用户误以为已切换插件。

### 9.2 后续真正多插件方案

如果确实需要 Theta* 和 RPP：

1. 在 Nav2 参数中注册独立插件 ID；
2. 行为树接入 Planner Selector 和 Controller Selector；
3. Edge 从 Nav2 获取 capability 列表；
4. 平台只显示机器人当前版本支持的能力；
5. action 下发前选择插件并获得确认；
6. 不可用时任务预检失败，禁止静默退回默认算法。

配置应用建议封装为单个 `ApplyNavigationProfile(profile_id, generation)` 服务，内部原子校验并返回实际值，代替多次独立参数调用。失败时回滚上一已知安全配置。

## 10. 动态避障安全分层

将现有单一开关拆成：

| 开关 | 职责 | 默认值 |
| --- | --- | --- |
| `detour_enabled` | 局部代价层和 MPPI CostCritic 是否主动绕行 | 开 |
| `collision_slowdown_enabled` | 接近障碍是否按距离减速 | 开 |
| `collision_stop_enabled` | 最后一道防撞硬停 | 强制开 |

`collision_stop_enabled` 只能由受控的停靠末段安全模式临时调整，且同时满足：

- 明确的停靠任务类型和目标 ID；
- 极低速度上限；
- 最大允许行程/超时；
- 充电桩或接触传感器闭环；
- 随时可恢复硬停；
- 审计事件完整。

预转向、后退和侧移自愈都必须经过同一安全速度输出链路。侧移绕行点在下发前至少检查：

- 全局地图边界和禁行区；
- 局部代价地图机器人足迹碰撞；
- `ComputePathToPose` 可达性；
- 侧向和前后净空；
- 定位健康和传感器新鲜度。

## 11. 自愈仲裁器

### 11.1 单一所有权

新增 `RecoveryArbiter`，至少维护：

```text
owner = NONE | NAV2_BT | EDGE_OBSTACLE | EDGE_LOCALIZATION | OPERATOR
recovery_generation
reason
started_at
budget
```

任一恢复流程开始前必须获取所有权；获取失败只能观察，不能再次下发运动。切换恢复所有者时先取消并确认旧 action 已终止、速度为零，再开始新动作。

### 11.2 分级恢复

建议使用有限预算的三级恢复：

1. `QUICK_LOCAL`：短暂等待、重新规划、清局部代价图；
2. `STRUCTURED`：清全局图、受控旋转、验证净空后后退、重定位、重新发送同一段；
3. `SAFE_HOLD`：超过时间/距离/次数/电量预算，停车、告警、请求人工。

恢复预算应同时考虑：

- 尝试次数；
- 已耗时间；
- 自愈移动距离；
- 剩余电量；
- 定位质量趋势；
- 是否已接近地图边界或充电桩。

不建议默认无限静默重定位。若业务确需长期等待，应进入可见的 `SAFE_HOLD_RETRYING` 状态，周期播报告警并允许人工取消，而不是表现为任务一直运行。

### 11.3 单点与多点行为一致

统一 `NavigateToPose` 与 `NavigateThroughPoses` 的恢复能力，特别是重定位步骤。Edge 不应依赖两棵行为树的偶然差异决定业务结果。所有恢复结果必须回到“重试当前段”，不能跳到下一点。

## 12. 事件、页面和语音

### 12.1 事件模型

页面至少区分：

- `target_dispatched`：目标已发送；
- `target_accepted`：Nav2 已接受；
- `waypoint_passed`：连续路线中经过；
- `arrival_pending_settle`：目标点黄色，等待停车；
- `arrival_correcting`：正在校正；
- `arrival_confirmed`：到点确认，可转为完成色；
- `departure_aligning`：面向下一段；
- `recovery_active`：显示恢复层级和所有者；
- `safe_hold`：需要人工或等待条件恢复。

页面不能在刚下发第一个目标时把后续点都标成“当前”；也不能用 Nav2 多点反馈索引直接触发“确认到点”。

### 12.2 语音

到点语音只消费 `arrival_confirmed`，通过点如需播报应配置独立模板。语音消息携带稳定的 `waypoint_id` 而不是闭包中的可变循环索引，并使用幂等键防止重试后重复播报。

阻塞语音完成前保持当前点状态，不提前下发下一段；非阻塞语音允许并发，但不得因语音回调迟到改变任务索引。

## 13. 实施阶段

### P0：基线冻结与可观测性

- 固化一组室内、室外、往返、精确终点和停靠 rosbag/任务快照。
- 增加 action generation、航点 ID、配置读回、定位样本时间和恢复所有者日志。
- 页面先区分 `waypoint_passed` 与 `arrival_confirmed`。
- 建立现有耗时、到点误差、重复事件和恢复成功率基线。

### P1：正确性收口

- 修正 `theta_star/navfn` 的名实问题，先公开真实的 Dijkstra/A*。
- 前端暂时只公开 MPPI。
- 按完整 LegProfile 切批。
- 实现真实 `dwell_seconds`。
- Collision Monitor 硬急停从普通避障开关中剥离并保持默认开启。
- 配置应用失败时禁止发送 action。

### P2：到点与校正事务

- 引入航点 `arrival_policy`。
- 落地到点状态机和幂等键。
- 室内增加校正后点击点复核。
- 使用校正完成后的新鲜位姿和连续样本确认。
- 明确普通、精确、室外和停靠失败策略。

### P3：下一段与配置事务

- 创建 LegProfile capability 协商、应用、读回和回滚接口。
- 分级预转向并统一 Collision Monitor 安全链路。
- 确保下一全局路径基于校正后位姿重新生成。
- 增加 action 接受前后状态和超时诊断。

### P4：自愈统一

- 引入 RecoveryArbiter 和 generation。
- 统一单点、多点 Nav2 恢复能力。
- 对后退和侧移增加后方/足迹/边界/路径验证。
- 用有限恢复预算替换无界静默重试。

### P5：多插件能力（可选）

- 经实测确有收益后再引入真正 Theta*、RPP 或其他插件。
- 行为树接入 selector，平台依据 capability 动态显示。
- 对插件切换做独立回归，禁止在同一 action 中途切换。

### P6：灰度发布

- 仿真 → rosbag 回放 → 空载室内 → 有障碍室内 → 室外 → 精确停靠。
- 先 shadow 计算新到点结论但不控制任务，比较新旧差异。
- 单机器人灰度后扩展，保留旧执行器版本和配置开关用于快速回滚。

## 14. 测试与验收

### 14.1 单元测试

- 所有 LegProfile 字段变化都会正确切批。
- 历史规划器值迁移后与真实算法一致。
- `pass_through` 不产生 `arrival_confirmed`。
- 校正前或旧 generation 位姿不能确认到点。
- dwell、动作和语音幂等。
- 旧 action 回调不能推进新任务。
- 配置读回失败阻止 action 下发并恢复安全配置。

### 14.2 集成测试

- 两点、五点、往返、多轮路线不漏点、不跳点、不重复播报。
- 第 1、2、3 点语音分别绑定正确点号和模板。
- 中间连续点平滑通过；业务停点停车、校正、播报后再发下一段。
- 校正产生明显位姿变化时，下一全局路径从新位姿开始。
- NDT 短暂超时只进入迟滞降级，真实位姿跳变仍会被拒绝并安全停车。
- Nav2 goal rejected、定位丢失、障碍无进展时只存在一个恢复所有者。
- 取消任务后所有 action、计时器、语音等待和恢复代次均失效。

### 14.3 安全测试

- `detour_enabled=false` 时硬急停仍生效。
- 预转向、后退和侧移遇障碍均能停止。
- 侧移点越界、无全局路径或足迹碰撞时禁止执行。
- 停靠特例超时、超距、失去接触反馈时立即恢复硬停并退出特例。
- 低电量期间自愈预算受限，不因重复恢复耗尽电量。

### 14.4 性能与业务验收指标

- MPPI 20 Hz 周期 p99 不超过 50 ms，连续超时有明确告警。
- 全局路径通常在 1 秒内产生，漏掉规划周期必须可观测。
- 要求停车的航点 100% 使用校正后新鲜位姿确认。
- 正常任务中航点漏报、重复完成、重复语音均为 0。
- dwell 实际误差不超过 ±0.2 秒。
- 页面显示的规划器/控制器与运行中实际插件及参数 100% 一致。
- 恢复期间并发运动所有者数量始终不超过 1。

## 15. 建议代码改动位置

| 模块 | 主要改动 |
| --- | --- |
| `platform/backend/monitoring/services/task_service.py` | 航点类型、语音模式、分层避障字段、旧字段迁移和路线快照 |
| `platform/backend/monitoring/protocol.py` | 新字段校验、capability/version 校验、真实规划器名称 |
| `platform/frontend/src/views/RoutePlannerPage.vue` | 航点到达策略、真实算法选项、能力驱动显示 |
| `platform/frontend/src/views/TaskExecutionPage.vue` | 通过/校正/确认/恢复状态和真实运行配置展示 |
| `edge-agent/roamerx_edge/task_executor.py` | LegProfile 切批、到点状态机、校正事务、dwell、语音幂等、恢复仲裁 |
| `edge-agent/roamerx_edge/ros_adapter.py` | 原子配置应用与读回、action generation、capability、统一安全控制权 |
| `robot/src/navigation/src/robot_navigo/params/navigo_params.yaml` | 真实插件 ID、goal checker、局部控制和 Collision Monitor 分层参数 |
| `robot/src/navigation/src/navigo_bt_navigator/behavior_trees/*.xml` | 单点/多点恢复对齐、selector 接入、恢复所有权边界 |
| `robot/src/localization/localization/apps/localization_nodelet.cpp` | 校正 generation、新鲜度与质量证据、健康迟滞状态 |

## 16. 风险与回滚

- 切批更细会增加 action 数量和停车机会。必须以 `arrival_policy=pass_through` 保持普通中间点连续，不可机械地每点停车。
- 室内校正后复核可能暴露更多历史地图/点位误差。先 shadow 记录，不应直接用大规模自动重试掩盖地图问题。
- 更严格的配置确认可能让以前“带旧参数勉强运行”的任务提前失败，这是正确的 fail-closed 行为，但页面必须清楚显示缺失能力和实际配置。
- 自愈统一后，短期内成功率可能看似下降，因为无限重试会转成显式安全保持；应以安全、可诊断和可恢复为优先指标。
- 所有阶段使用独立功能开关；数据库迁移必须向后兼容；回滚应用版本时仍能读取旧路线快照。

## 17. 不建议的优化方式

- 不要只把 Nav2 XY、yaw 或超时阈值整体调大。
- 不要把 Nav2 中间点 1.0 m 移除事件当成精确到点。
- 不要在一个 action 运行中热切换规划器、控制器和安全层。
- 不要在配置调用异常后记录日志并继续发车。
- 不要让 Edge 与 Nav2 同时执行旋转、后退或重定位。
- 不要为了停靠方便而关闭全程碰撞硬急停。
- 不要继续向用户暴露实际没有加载的 Theta* 或 RPP。
- 不要依靠可变循环索引生成航点播报和完成事件。

## 18. 完成定义

本计划只有在以下条件全部满足时才算完成：

1. 路线配置、运行参数、实际插件和页面展示一致；
2. 航点通过、停车、校正、确认、播报和离开均有明确且不可逆的状态；
3. 所有停点基于校正后的新鲜稳定位姿确认；
4. 每段配置在发车前完成能力校验和读回；
5. 碰撞硬停保持独立且默认开启；
6. Nav2 与 Edge 自愈由同一仲裁器串行控制；
7. 室内、室外、往返、低定位质量、动态障碍和停靠测试均通过；
8. 任务全过程没有漏点、跳点、重复语音、迟到回调推进或重复运动指令。
