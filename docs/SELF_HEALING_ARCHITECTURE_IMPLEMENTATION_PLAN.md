# 导航与定位四层自愈架构实施方案

状态：第一、二阶段开发与离线验证完成，待提交、部署及现场验收
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

## 6. 第二阶段加固方案（2026-09-12）

### 6.1 “临时 UKF 自动恢复”的需求含义

`LIO hold` 不是新的长期定位模式，而是一份有期限、有代次、可条件释放的降级
租约。它只用于定位源故障，且仅在 RTK fixed 不可用、浮点 RTK 不满足 `≤0.20 m`
UKF 门槛、NDT 不健康、FAST-LIO 健康时启用。导航控制器失败不能仅因当时绝对源
较弱而修改 UKF。目标是在绝对观测异常期间拒绝错误锚点更新，同时继续输出连续
但不确定度更高的 LIO 位姿。

必须同时具备以下退出保护：

1. fixed RTK 或健康 NDT 连续 3 个观察样本稳定后，先进入短时 `balanced` 过渡，
   降低刚恢复的绝对观测权重，再自动回到 `nominal`；单个抖动样本不能提前释放；
2. 普通恢复 episode 结束、任务完成/取消/失败/安全保持、人工接管或 Edge 正常退出
   时主动恢复 `nominal`。唯一例外是以 `lio_hold_degraded_continuation` 成功放行任务：
   故障动作闭环可以结束，但融合租约继续由健康观察器、任务终态和 TTL 管理；
3. 每次非 nominal 配置都必须有 1～180 秒 TTL，定位节点以稳态时钟在独立 10 Hz
   回调中到期恢复，不能依赖 Edge 线程仍然存活；
4. 定位进程重启后默认进入 `nominal`；Edge 重启但定位进程未重启时，根据
   `/localization/decision` 上报的当前 generation 对遗留临时配置做条件恢复；
5. 每次配置变更生成单调递增 generation。Edge 只允许用匹配 generation 释放
   自己创建的租约，旧观察线程不得把后来 episode 的新配置误恢复为 nominal。

`balanced` 明确定义为短时恢复过渡：继续使用原有质量门槛接纳 fixed RTK/NDT，
但放大绝对观测方差以降低锚点更新增益；它不是 `nominal` 的空别名。`LIO hold`
仍采用拒绝坏绝对观测、保持 LIO 传播和放大输出协方差的方式，不通过增大 Anchor
UKF 过程噪声实现。当前时限为 Edge `lio_hold=180 s`、BT `lio_hold=10 s`、
`balanced=5 s`；所有非 nominal 时限最终都由定位节点自身执行。

### 6.2 等级、动作和旋转授权加固

- 新 episode 只能从 0 级开始；同一 episode 每次最多提升一级，拒绝跨 episode、
  跳级和过期请求；
- Edge 为每个故障、场景和等级生成允许动作集合。BT 的 `ACTION_STARTED` 必须先
  通过集合校验，恢复租约只解决单所有者问题，不能代替动作策略授权；
- `AdaptiveSpin` 除读取 `forbid_spin` 外，还必须携带 episode，在启动子 Spin 前
  向 Edge 获取一次新鲜的 2 级授权。服务不可用、episode 不匹配、诊断已恢复、
  室外/过渡或动作不是 `adaptive_spin` 时一律失败；
- BT 动作授权服务失败时 fail closed，不允许以“仅缺少记录”为理由继续运动。

### 6.3 记录闭环和容量控制

- action 增加精确 `duration_seconds`，统计同时提供动作成功率和平均耗时；
- Edge 启动时把上次异常退出遗留的未完成 action/episode 标记为
  `interrupted_by_restart`，避免永久 `NULL`；
- 增加每日汇总表。详细记录默认保留 180 天，清理前按故障和动作聚合长期统计；
  清理顺序为 action→episode，并显式执行一致性检查，不依赖当前未启用的 SQLite
  外键约束；
- `self_heal_episodes` 每个故障一行，保存故障/场景/任务/地图、压缩后的首次证据、
  最终等级/动作、成功状态、原因和总耗时；
- `self_heal_actions` 每个实际尝试一行，保存 episode 外键、等级、动作、成功状态、
  原因和精确耗时；
- `self_heal_daily_summary` 只保存日期×类别×标签的总次数、成功次数和总耗时，供
  详细记录清理后继续计算频率、成功率和平均耗时；
- 首次证据只保留标量诊断字段；全局路径只记录是否更新和点数，不保存点数组，
  更不保存 10 Hz 轮询样本，避免定位频率或长路径直接转化为磁盘增长。

容量按保守单 episode 1～4 KB、每个 action 0.3～0.8 KB、每 episode 1～4 个 action
估算：20 次故障/天保留 180 天约 4～20 MB；100 次/天约 20～100 MB（均含索引与
SQLite 页开销的量级估算）。每日汇总通常每天不足 20～40 行，长期增长远低于明细。
实际运行一个月后应以 `page_count×page_size` 和每表行数复核，不因删除明细自动
执行 `VACUUM`，避免启动时长时间锁库和额外闪存写放大。

### 6.4 第二阶段验收

- 直接从新 episode 请求 2/3 级、使用错误 episode 或请求非推荐运动动作均被拒绝；
- 自定义 BT 即使写死 `forbid_spin=false`，没有 Edge 同 episode 新鲜许可也不能旋转；
- 连续 3 个绝对源健康样本触发 `lio_hold→balanced→nominal`，旧 generation 的恢复
  请求不能覆盖新租约；TTL 和定位重启仍可独立恢复 nominal；
- Edge 异常重启后没有长期未完成记录；每日汇总、180 天清理和动作平均耗时测试
  通过；
- 重新编译接口、定位、BT 和导航配置包，完成 Python/C++/XML/动态库检查。

### 6.5 第二阶段实施结果

- [x] 服务增加 `expected_generation/generation`；定位节点的配置变更、稳态时钟 TTL
  到期均递增 generation，并用同一互斥锁消除“旧 TTL 覆盖新配置”的竞态；
- [x] Edge 连续 3 个健康样本后执行 `lio_hold→balanced`，由 5 秒 TTL 回
  `nominal`；旧观察线程、任务终态、人工接管、Edge 启停均使用 generation 条件
  恢复，不覆盖后来配置；
- [x] `balanced` 对 NDT/RTK 观测方差应用 4 倍缩放；`lio_hold` 继续抑制自动绝对
  锚点更新并放大输出协方差；
- [x] 新 episode 只允许 0 级开始，禁止跳级、跨 episode、无 episode 动作和策略外
  动作；`RecoveryLeaseScope` 在 Edge 授权失败时不执行运动子节点；
- [x] `AdaptiveSpin` 在启动 Spin 前再次向 Edge 获取同 episode 的 2 级许可，服务
  不可用、超时、室外/过渡、故障已恢复或动作不匹配时 fail closed；
- [x] SQLite 已增加 action 耗时、重启中断对账、180 天明细聚合清理、每日汇总、
  外键检查和证据压缩。

离线验证：第 6 节相关 Edge 测试 143 项通过（包含定位初始化字符串状态回归用例）；5 个 ROS 包重新编译成功；
localization/robot_navigo 共 74 个 C++ 测试用例通过；安装后的服务字段、两个默认
BT XML 和动态库依赖检查通过。全部 40 个已跟踪 Edge 测试文件为 483 通过、1 个
既有失败：旧用例要求在 1.2 m 总恢复距离预算内连续批准 `6×0.3 m`，生产实现按
预算在第 5 次拒绝；该矛盾与本阶段修改无关，未在本次扩大范围修改。

本阶段未启动 Nav2、未发送速度或导航目标。提交/部署后仍需用回放、仿真或受控
现场验证 `/cmd_vel` 室外无自愈旋转、健康恢复一个检测周期内停止动作，以及进程
异常时 TTL 独立恢复三项运行时条件。

## 7. 第三阶段：恢复闭环与部署持久性补齐（2026-09-14）

本阶段解决“机器人已经停车但循环一直等待安全条件”复盘后确认的三个剩余边界：

1. Edge 必须从版本化 release 和永久 `current-edge` 原子链接启动，禁止服务重启或
   NX 重启后回退到带未提交改动的开发目录；部署前拒绝脏的 `edge-agent` 源树，
   新版本健康检查失败时恢复上一链接并重启旧版本；
2. `task.resume` 接受到点前转向动作后，必须在释放任务锁前把本地状态从
   `resuming` 持久化为 `running`。迟到的中心恢复命令只能返回
   `already_running`，不得取消刚接受的转向或巡航目标；
3. Edge 状态明确携带 `actual_velocity_observed`。从未收到实际 `/cmd_vel`、速度
   字段缺失或样本时间无效时，中心返回 `EDGE_ROS_DATA_UNAVAILABLE`；实际速度与
   定位缓存同时过期时返回 `EDGE_ROS_DATA_STALE`。两种情况都不能开始五秒恢复
   观察，也不能把默认零值当成停车证据。

实施验收：

- [x] systemd 服务重启后仍从同一 `current-edge` release 启动；永久 unit 已启用且
  不再依赖 `/run` 临时覆盖；
- [x] release 在全部准备步骤完成后才原子切换，健康检查失败分支会切回上一版本；
  开发工作区未提交文件不进入 release；
- [x] 恢复后先执行原地转向时，迟到恢复命令不增加取消次数或停车命令次数；
- [x] 实际速度样本为空、缺字段、非法或与定位同时过期时全部保持安全阻塞；
- [x] 新鲜零速连续观察五秒可恢复，真实非零速度仍返回 `ROBOT_NOT_STOPPED`；
- [x] `task.recover.v1` 失败不终结任务，`task.force_exit` 的入口和终态语义不变；
- [x] Edge 相关 208 项、平台循环与消息处理 45 项通过；部署后 Edge 状态、永久
  systemd 配置、进程工作目录和云端安全判定均已核验。
- [ ] NX 整机重启验证留待受控维护窗口执行；当前已通过永久 unit 静态校验和服务
  级重启验证，不为单项验证中断正在进行的充电。

实施提交为 `8633f58`，部署事务补丁为 `0490bdc`、`6a60ee7` 和 `3fac58a`。
现场 Edge 已通过 `current-edge` 运行版本化 release；在充电待机、ROS 尚无实际
速度样本时，云端实测返回 `EDGE_ROS_DATA_UNAVAILABLE`，没有把默认零值当作停车。
