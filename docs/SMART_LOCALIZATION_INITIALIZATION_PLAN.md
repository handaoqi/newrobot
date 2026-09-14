# 智能化初始化定位与人工兜底方案

## 目标

将当前仅依赖 `last_trusted_pose` 的初始化/恢复方式升级为分级定位：

```text
当前航点候选 → 相邻航点候选 → 原地旋转/短移搜索
→ 全局关键帧搜索 → NDT/GICP 几何验证
→ 成功后从当前航点继续
```

室内优先使用 NDT/GICP，室外优先使用质量合格的 RTK；Fast-LIO2 继续作为连续运动观察源。

## 定位候选与安全约束

1. 优先使用任务当前未完成航点，而不是历史可信位姿。
2. 对航点尝试 `yaw ±45°、±90°、180°`，必要时进行不超过 0.3 m 的短距离搜索。
3. 原地旋转速度和短移速度受限；每次动作前检查急停、电量、避障、Nav2 目标取消和机器人零速。
4. 候选必须同时满足 NDT/GICP 收敛、score、内点率、位姿跳变和连续稳定帧要求。
5. 室外 RTK 不是 fixed、精度或数据年龄不合格时，不得用于校正，转入 NDT/GICP。
6. 局部候选失败后进入全局关键帧/Scan Context 搜索，默认预算 90 秒；全局候选仍必须经过几何验证。

## 成功后的恢复

定位验证成功后恢复 Fast-LIO2 跟踪，保留当前轮次和当前未完成航点索引，从该航点继续导航，不重新跳选最近点。

## 失败后的人工兜底

航点候选、局部动作和全局搜索全部失败后：

```text
停止机器人 → 取消 Nav2 目标 → 任务 paused → 上报定位恢复失败业务告警
```

失败状态必须允许人工再次操作，不能无限自动重试：

### 重新发起智能初始化

在路径规划/导航测试页面提供“重新初始化定位”按钮。点击后：

- 仅允许机器人在线、地图一致、速度为零时执行；
- 复用当前任务航点和地图上下文；
- 重新从航点候选开始，失败后再次进入全局搜索；
- 新一轮尝试覆盖旧的恢复状态，但不清除任务执行记录。

### 人工下发初始点

在地图上点击机器狗实际位置，再设置实际朝向，执行“下发初始定位”：

- 必须同时提供 `frame_id=map`、`x`、`y`、`yaw`、`map_id` 和 `map_version`；
- 页面显示当前机器人位姿、地图版本和定位状态，地图不一致时禁止下发；
- 下发后等待 NDT/GICP 或 RTK 达到稳定条件；
- 稳定后允许人工点击“继续任务”，从暂停时保存的航点继续；
- 初始点下发只改变定位，不直接让机器人移动。

人工操作期间仍保留急停、低电量和 Nav2 状态检查。定位未验证成功前不得恢复导航。

## 任务开始前初始化定位（新增）

任务开始前也必须经过同一套定位门禁，不能只依赖导航过程中定位丢失后的恢复：

```text
task.start → 检查地图和机器人状态 → 检查定位是否稳定
→ 使用第一个待执行航点生成候选 → 失败后全局搜索（最多 90 秒）
→ 验证成功后才发送 Nav2 导航目标
```

- 初始化期间机器人保持停止，禁止先发导航目标再补定位；
- 首个航点作为第一优先级种子，室外先检查 RTK，RTK 不合格时使用 NDT/GICP；
- 初始化失败时拒绝 `task.start`，任务不进入 `running`；
- 页面可重新发起智能初始化或人工下发初始点，成功后由人工重新执行任务；
- 启动前失败返回 `INITIALIZATION_FAILED`，导航中失败则保存当前航点并进入 `paused`。

### 固定 RTK 与 FAST-LIO 接管（2026-09-13 更新）

室外或过渡场景、且地图坐标模式为 `rtk_fixed` 时，任务启动不复用上一次
任务的 `last_trusted_pose`，按以下状态机完成：

```text
task.start
→ 清除本地图内存/SQLite last_trusted
→ fixed RTK + 双天线航向连续 3 帧、RTK 自身跨度 ≤0.30m
→ /localization/seed_from_rtk 写入绝对位姿、创建 map←lio 锚点代次
→ 等待置姿后的新鲜 FAST-LIO 帧
→ lio_healthy + lio_anchored + absolute_stable + active_source=lio_imu
→ 启动 Nav2
```

- RTK 与 FAST-LIO 的 XY 差仅用于诊断，不能阻止 fixed RTK 初始化；
  原有“差值 <0.30m 连续三帧”门禁取消。
- NDT 候选提交后走相同的 `lio_handoff_pending` 状态；NDT 负责绝对校正，
  RTK/NDT 都不得替代 FAST-LIO 成为连续输出源。
- 接管仅接受绝对置姿/候选提交之后的新 LIO 帧。若 8 秒内未完成，返回
  `LIO_HANDOFF_TIMEOUT`，上报 LIO 流、锚点或运动异常诊断，保持安全停止，
  不重复候选搜索。
- 定位决策需发布 `handoff_state`、锚点代次、接管来源和失败原因；导航 MCAP
  必须录制 `/localization/decision`、`/localization/policy`。
- 新任务完成启动验证前冻结 trusted 写入；只有本任务已完成 LIO 接管后，才以
  当前新鲜稳定位姿重建 `last_trusted`。启动失败则保持失效。

## 室外初始化、接管与进度链路修复阶段（2026-09-14，代码已实施，待部署实机验收）

### 已确认问题

- 地图 171 的权威属性为 `outdoor + rtk_fixed`，但部分渐进定位命令被下发为
  `indoor + local_only`。页面还会依据地图切换后的单次状态快照决定是否尝试
  RTK，状态同步稍慢便可能跳过 RTK 优先流程。
- Edge 创建 `CommandProcessor` 时未注入 `mqtt.publish_progress`；实际定位命令
  只有 created/ack/result，没有 `command.progress`，因此页面无法异步刷新候选点。
- 2026-09-14 16:00:04，`/planner_server/get_parameters` 超时后销毁仍有请求在途的
  ROS Client，触发 `rclpy InvalidHandle` 并终止 `ros-executor`。定位节点仍在发布，
  但 Edge 不再接收新定位帧，继而误报 RTK 新鲜样本不足和 FAST-LIO 接管失败。
- 定位异常终态会把阶段结构展平或覆盖，RTK 阶段也没有完整起止时间，导致页面
  显示“阶段时间未上报”，最终结果下还可能丢失阶段归属。

### 目标流程

```text
室外/过渡 + rtk_fixed 地图
→ RTK 新鲜度、质量、双天线航向和三帧自身稳定性验证
→ fixed 通过后取得 RTK 在地图坐标系中的已定位位姿
→ 以该位姿建立唯一候选 #1，并在该位置执行一次定点 NDT 匹配
→ 上报位置号 #1、位置标签、NDT score、内点率、收敛状态和匹配位姿
→ 提交 RTK 固定解验证结果并调用 /localization/seed_from_rtk
→ 将 map←lio 锚点更新到当前 RTK XY/yaw
→ 等待置姿后的新鲜 FAST-LIO 帧
→ active_source=lio_imu + lio_anchored + absolute_stable + status=normal
→ 启动 Nav2，不进入原点/周边/航点/全局 NDT 候选搜索
```

- 定位编排以地图的 `scene_scope`、`coordinate_mode` 为权威，不允许路线表单默认值
  覆盖地图属性；渐进定位请求也必须携带这两个字段。
- 符合 RTK 地图条件时始终先进入 RTK 验证，不因云端状态快照缺失或稍旧而跳过。
  新鲜样本明确为非 fixed、位置/航向质量不合格时立即结束 RTK 事务并转 NDT；
  仅在等待首个新鲜样本时保留短暂启动窗口。
- fixed 验证通过后，只针对 RTK 已定位位姿执行一次有超时边界的本地 NDT
  交叉验证，不再搜索其他位置。该记录固定使用 `candidate_number=1`、
  `candidate_label=RTK固定解定位点`；若当前位姿可唯一关联路线航点，可在标签中
  附带航点号，但不得用航点号替代候选序号。
- NDT 交叉验证用于取得位置号和可审计的 NDT 质量信息，不改变 RTK fixed 的绝对
  定位权威，也不把 NDT 切换为连续主定位源。只要提交前 RTK 仍为新鲜合格 fixed，
  NDT 分数不佳、未收敛或超时必须明确上报，但不得因此启动多点搜索或无限等待；
  若 RTK 在提交前失效，才结束本次 fixed 事务并转入渐进 NDT 流程。
- 完成单点 NDT 记录后立即提交 RTK fixed 结果，完成绝对置姿和 LIO+IMU 接管。
  `LIO_HANDOFF_TIMEOUT` 表示连续主定位源确实不可用，应保持停车并上报诊断，
  不得作为 RTK 质量失败继续盲目搜索 NDT。

### 实施项

1. 修复共享地图初始化编排：统一路径规划、地图下发、主动重定位和任务启动的
   地图场景判定，移除依赖单次云端 RTK 快照才能进入 RTK 验证的条件。
2. 在 `_set_initial_pose_from_rtk_once` 的 fixed 稳定性验证通过后、最终 RTK 提交前，
   以 RTK 地图位姿调用一次无永久提交副作用的 NDT probe。probe 只创建候选 #1，
   等待一组新的 NDT 结果并记录 `matching_error`、`inlier_fraction`、
   `has_converged`、匹配位姿和耗时；完成或超时后均退出该单点 probe。
3. 复用参数读写、生命周期查询等 ROS Service Client；超时后不销毁仍在途的 Client。
   `RosRuntime` 监测执行器线程，异常退出时终止 Edge 主进程，由现有 systemd
   `Restart=always` 在 5 秒后恢复，避免 MQTT 在线但 ROS 回调永久停止。
4. 将 `mqtt.publish_progress` 注入 `CommandProcessor`，在 RTK 样本验证、RTK 定点
   NDT 候选 #1 开始/完成、RTK 结果提交和导航启动时发布进度。
5. 后端统一合并成功与失败终态的 `localization_attempts`，保留最近一次完整候选、
   阶段、最优位姿和接管诊断，禁止终态错误覆盖执行中进度。
6. 每个阶段上报 `started_at`、`updated_at`，终态补充 `finished_at`；每个候选保留
   编号、标签、阶段、起止时间、NDT score、内点率、收敛状态和拒绝原因。
7. 路径规划页按一秒周期刷新；在“RTK固定解验证”节点下显示唯一位置 #1 及其实时
   NDT score、内点率和收敛结论，在“建图原点及周边候选”“路线航点候选”等节点下
   逐项显示渐进候选。执行中显示已用时间，完成后显示耗时，待执行节点不再显示
   “阶段时间未上报”；旧命令使用命令时间作兼容回退。
8. 接管失败结果补充 `ros_executor_alive`、定位帧年龄、LIO 健康、锚点代次、
   `handoff_state` 和 `handoff_failure_reason`。页面文案改为“RTK固定解质量验证与
   定点NDT交叉验证、FAST-LIO+IMU 接管”。

### 2026-09-14 实施结果

- 路径规划、地图激活、主动重定位和任务启动均以地图内的 `scene_scope`、
  `coordinate_mode` 为权威。室外/过渡 `rtk_fixed` 地图即使云端状态快照为空，
  也先由 Edge 检查实时 RTK；明确收到非 fixed 或质量不合格的新样本后立即转入
  渐进 NDT，不再等待完整超时窗口。
- fixed RTK 连续样本通过后，以其地图 XY/yaw 建立唯一候选 #1
  `RTK固定解定位点`，执行一次最长 5 秒的 NDT 交叉验证。NDT 优、差、未收敛、
  无新结果四种情况均保留 score、内点率、收敛与拒绝原因；只要提交前 RTK 仍为
  合格 fixed，就提交 `/localization/seed_from_rtk` 并验证 FAST-LIO+IMU 接管，
  不展开第二个候选。probe 期间冻结 trusted pose，最终由 RTK 提交覆盖临时锚点。
- Edge 生产入口已注入 `mqtt.publish_progress`，任务启动也临时挂接候选进度回调；
  后端按阶段和候选身份递归合并 progress/result，页面按 1 秒刷新阶段时间、候选 #1、
  NDT 质量与接管诊断。
- ROS 参数、生命周期、代价地图等临时 Service Client 改为节点生命周期内复用，
  超时不再销毁仍可能留在 executor wait set 中的句柄。`ros-executor` 对瞬时异常继续
  运行；连续异常或线程退出时，Edge 以非零状态退出，由部署单元现有的
  `Restart=always`、`RestartSec=5s` 拉起。
- 自动测试结果：Edge 本阶段定向测试 56 项通过；当前提交 Edge 全量 568 项通过；
  后端相关链路 75 项通过；前端 179 项通过并完成 Vite 生产构建。

ROS 退出问题需按边界记录：本阶段解决的是 Edge 内部 `ros-executor` 因 Client
生命周期竞态停止、以及 executor 真正失效后 Edge 仍假在线的问题。定位、LIO、Nav2、
雷达和 GPS 是独立长运行 ROS 子进程；当前部署使用 `KillMode=process`，重启 Edge
不会重启这些进程。独立 ROS 子进程自身退出后的统一存活探测、按组件重启和冷却限流
不在本阶段实现范围内，不能将其表述为“所有 ROS 进程退出问题已解决”。

### 接口和验收

- 沿用 `nav.initial_pose`、`nav.relocalize` 和 `command.progress`，不增加数据库迁移；
  `localization_attempts.stages[]` 统一包含阶段时间，接管结果增加执行器与 LIO 诊断。
- 室外 RTK 地图且状态快照暂时为空时，第一条定位命令仍须为
  `nav.initial_pose(seed_source=rtk)`；明确非 fixed 时应立即转 NDT。
- fixed 合格后必须恰好产生一个以 RTK 已定位位姿为种子的 NDT 验证位置 #1；页面
  必须显示该位置号、位置标签、具体 NDT score、内点率和收敛结论，然后提交 RTK
  fixed 结果，更新当前位置、LIO 锚点和定位状态并启动 Nav2。不得产生第二个候选，
  也不得进入原点、周边、航点或全局搜索；真实 LIO 故障须保持停车并返回完整接管原因。
- 覆盖 NDT 优、差、未收敛和超时四种结果：RTK 在提交前仍合格时都只记录结论并
  继续 RTK 提交；RTK 同时失效时才允许转入渐进定位。
- 模拟参数服务超时不得再终止 ROS 执行器；模拟执行器异常退出时 Edge 应自动重启。
- MQTT、后端和前端联调必须观察到每个候选开始/完成事件，页面对应阶段逐项异步刷新，
  新命令所有已开始阶段均有时间。
- 完成 Edge 定位/命令处理测试、后端 MQTT 进度合并测试、前端定位流程与时间线测试
  及前端构建后，再以室外 RTK 地图进行一次停车实机验收。

本阶段不调整 NDT、RTK、UKF 质量阈值，也不改变 FAST-LIO+IMU 作为连续主定位源的
约束。

## 接口与状态

复用现有 `nav.initial_pose`、`nav.relocalize`、`task.resume` 接口，智能初始化增加：

```json
{
  "mode": "smart",
  "seed_waypoint_index": 0,
  "allow_local_motion": true,
  "global_timeout_seconds": 90
}
```

恢复状态保存：

```json
{
  "stage": "waypoint_search",
  "attempt": 3,
  "current_waypoint_index": 0,
  "round_number": 2,
  "last_candidate": {}
}
```

定位尝试过程只写日志和遥测；只有最终恢复失败进入业务事件中心。

## 验收标准

- 机器人在 1 号航点附近、朝向偏差较大时可通过航点候选收敛；
- 局部失败后自动进入全局搜索，最长不超过 90 秒；
- 全部失败后机器人保持停止并进入 paused；
- 页面可重新发起智能初始化；
- 页面可人工选择初始位置和方向并下发；
- 人工初始点验证成功后可从原航点继续；
- 定位失败过程中不会跳过航点、重复运动或产生普通过程告警。
