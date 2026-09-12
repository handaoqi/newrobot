# 导航与定位四层自愈架构实施方案

状态：开发完成，待部署后的仿真/现场验收
日期：2026-09-12

## 1. 复核结论

现有系统已经具备定位状态/NDT/RTK/Nav2 障碍与无进展检测、Edge 定位恢复、
Nav2 BT 恢复动作以及单所有者 `RecoveryArbiter`，但仍是多个局部状态机的组合：

- 检测入口分散，故障原因主要用字符串传递，没有统一诊断结果和证据快照；
- Edge 与 Nav2 BT 都能选择恢复动作，尚未形成唯一的场景化决策权威；
- BT 恢复树包含无条件 `Spin`，不能保证室外 RTK 短暂丢失绝不旋转；
- 恢复动作没有统一的 0→1→2→3 等级和成功短路语义；
- 恢复进度主要存在于内存遥测，没有可统计的本地自愈事件记录。

采用混合架构：Edge 是故障诊断、场景决策、等级推进和记录的唯一权威；
Nav2 BT 节点只执行 Edge 允许的动作并实时监控退出条件。继续复用现有
`RecoveryArbiter`，不再建立第二套动作控制/租约服务。

UKF 复核修正：当前 `AnchorUkf` 估计的是 `map→lio` 锚点。增大锚点过程噪声会
提高后续绝对观测的 Kalman 增益，并不等价于更加信任 `odom_lidar`。当 RTK 与
NDT 都差时，应拒绝或降低两者观测权重、保持 LIO 连续传播，并提高对外发布的
定位不确定度；外部观测恢复后再渐进恢复其权重。

## 2. 四层职责

### 2.1 故障检测层

统一接收现有定位 10 Hz 回调、NDT 状态、RTK 质量、Nav2 action 结果、导航无进展
和 Collision Monitor 状态，产生带 `episode_id` 的故障证据：

- `localization_lost`
- `ndt_degraded`
- `rtk_transient_loss`
- `nav_stuck`
- `nav_action_failed`
- `sensor_stale`
- `collision_stop`

保留现有连续样本、防抖和消息新鲜度门槛；检测层只报告事实，不选择动作。

### 2.2 故障诊断层

`FaultDiagnoseNode` 请求 Edge 的统一诊断接口。Edge 根据场景模式、当前/期望定位
源、RTK/NDT/LIO 状态、导航状态和障碍快照输出：规范故障标签、恢复等级、推荐
动作、禁旋转标志及理由。

### 2.3 自愈决策层

每次只允许一个 `episode_id` 和一个恢复租约推进；严格按照 0→1→2→3 级执行，
每一级恢复成功立即结束本次 episode，不再执行更高代价动作。

| 等级 | 定位类故障 | 导航/障碍类故障 |
| --- | --- | --- |
| 0 | 零速保持；室外/过渡场景执行 `SmartRTKWait` | 短时观察、清理代价地图 |
| 1 | `SetUkfWeight` 切换降级配置；可信位姿/NDT 局部校正 | 局部重规划 |
| 2 | `SearchLaserFeature`；仅场景许可时选择 `AdaptiveSpin` | 安全检查后的倒车或侧向绕行 |
| 3 | 全局重定位 | 安全保持并请求人工处理 |

室外或过渡场景中，只要故障为 RTK 短暂丢失或 RTK 是期望绝对源，均设置
`forbid_spin=true`。该约束同时由 Edge 决策和 `AdaptiveSpin` 执行节点检查，不能
由行为树配置绕过。

### 2.4 自愈执行层

新增五个薄执行节点：

1. `FaultDiagnoseNode`：读取 Edge 诊断结果并写入 BT blackboard；
2. `SmartRTKWait`：实时轮询恢复状态，RTK 连续 3 次稳定后立即成功退出；
3. `AdaptiveSpin`：仅在 `forbid_spin=false` 时包装现有 Spin，NDT 达标立即 halt；
4. `SetUkfWeight`：调用定位节点运行时融合配置服务；
5. `SearchLaserFeature`：包装受 Collision Monitor 保护的小范围移动，NDT 达标立即
   halt，任何扫描陈旧或碰撞条件不满足立即失败并零速。

动作成功必须由实时健康条件确认，而不是只看动作本身返回值。所有动作退出时
零速、恢复临时融合配置并释放准确代次的租约。

## 3. 接口与数据

- 新增一个自愈诊断/状态/动作反馈 ROS 服务，供 BT 调用；Edge 内部仍使用直接
  Python 调用，避免重复序列化。响应包含 `episode_id`、故障标签、场景、等级、
  推荐动作、`forbid_spin`、`recovered` 和原因。
- 新增一个定位融合配置 ROS 服务：支持 nominal、LIO hold、balanced 三种受限
  profile；不允许传入任意未校验协方差。LIO hold 降权/拒绝低质量 RTK/NDT，保持
  LIO 传播并增加输出不确定度；退出自愈时恢复 nominal。
- 保持现有 `NavigationRecoveryLease` wire contract，不扩展字段；episode、等级和
  动作信息由 Edge 自愈协调器保存，避免破坏已部署客户端。
- Edge SQLite 新增 `self_heal_episodes` 和 `self_heal_actions`。记录触发时间、故障、
  场景、动作、等级、成功状态、耗时、任务/地图、证据与失败原因，并通过现有
  `task` 事件/outbox 上报。

## 4. 实施与验收

1. 实现纯逻辑诊断矩阵和 episode 状态机，接入现有定位与导航故障入口；
2. 改造 Edge 定位恢复为显式分级，并在每个动作后做统一健康检查；
3. 增加 ROS 接口和五个 BT 插件，替换两个默认导航树中的无条件 Spin；
4. 增加定位融合 profile，并保证进程异常/episode 结束后回到 nominal；
5. 增加本地持久化、MQTT 事件和统计查询数据；
6. 完成 Python 单测、C++ 单测和 BT XML/插件检查；部署后使用仿真、回放或受控
   场地完成运行时验收。

验收条件：

- 室外 RTK 短暂丢失期间 `/cmd_vel` 不出现自愈角速度；
- RTK 或 NDT 恢复时当前动作在一个检测周期内停止；
- 任一级成功后不再出现后续等级动作；
- Edge 与 BT 同时触发时只有一个运动恢复所有者；
- 双绝对源质量差但 LIO 健康时保持 LIO 连续传播，不接纳坏观测；
- 每次自愈都有完整 episode 和 action 记录，可统计故障频率、动作成功率和耗时；
- 3 级仍失败时进入安全保持，不无限重试。

## 5. 实施结果（2026-09-12）

已完成代码实施：

- Edge 新增统一故障诊断矩阵、单 episode 的 0→1→2→3 等级状态机，以及动作
  成功短路、等级耗尽后安全保持；
- 室外/过渡场景的 RTK 短暂丢失只执行零速实时等待，连续 3 个 fixed 样本即退出；
  默认导航树中的旋转全部由 `AdaptiveSpin` 和恢复租约共同保护；
- 新增 `FaultDiagnoseNode`、`SmartRTKWait`、`AdaptiveSpin`、`SetUkfWeight`、
  `SearchLaserFeature` 五个 BT 节点，并接入单点和多点默认恢复树；
- 新增受限的 nominal、LIO hold、balanced 定位融合接口。LIO hold 拒绝自动坏锚点
  更新、保持 FAST-LIO 连续传播、放大输出协方差，并通过 10 Hz 回调的稳态时钟
  到期保护和 Edge 健康观察器恢复 nominal；
- 新增 `self_heal_episodes`、`self_heal_actions` SQLite 表、统计查询和 MQTT 任务
  事件；导航恢复动作通过原有 `RecoveryArbiter` 保证单所有者；
- 修正 BT 中止顺序：先停止运动子节点，再上报动作失败并释放恢复租约，避免
  所有权已释放但旧动作仍可能短暂发布速度的窗口。

离线验证结果：

- `robots_dog_msgs`、`localization`、`navigo_behavior_tree`、
  `navigo_bt_navigator`、`robot_navigo` 编译成功；
- Edge 自愈相关测试 93 项通过；Edge 可运行全量测试 478 项通过、2 项既有的
  非本次用例被明确排除；
- localization C++ 测试 9 项通过，robot_navigo C++ 测试 2 项通过；
- 六个新增/改造 BT 动态库依赖检查通过，两个已安装默认 BT XML 和节点模型
  XML 校验通过；ROS 服务生成接口导入与 `duration_seconds` 字段检查通过。

当前工作区未找到可用 rosbag，因此本轮没有启动 ROS 导航进程、仿真运动或实机
动作。上面的前三项运行时验收条件仍须在部署后的仿真/回放或受控场地验证；尤其
需要记录 `/cmd_vel`、恢复 episode/action 事件与定位健康状态，确认室外禁旋转、
健康恢复一个检测周期内停动作，以及单恢复所有者三个条件。
