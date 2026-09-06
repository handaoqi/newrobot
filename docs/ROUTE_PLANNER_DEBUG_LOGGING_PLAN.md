# 路径规划调试全链路日志执行计划（恢复版）

## 目标与现状

复用现有 `SystemLog`、`DebugLogSession`、Edge `StructuredLogEmitter`、MQTT `system.log.batch`、日志保留策略和路径规划页日志面板，补齐定位、导航、避障、主动重定位、航点调整及规划算法的可追踪诊断链路。

当前主要缺口：命令之间没有共享顶层 `trace_id`，任务异步回调未继承 trace；日志列表只有时间游标；平台没有显式 `dedupe_key`；日志面板只有原始轮询；规划器、MPPI、Collision Monitor 和真实 action 终态覆盖不足。

## 实施内容

### 日志契约

- 路径规划页每次顶层操作生成 UUID `trace_id`，通过 `X-Trace-Id` 传递到所有嵌套请求；非法 UUID 返回 400，缺失时后端生成。
- `TaskContext`、命令、航点回调、避障线程和定位 generation 保存并继承 `trace_id`。
- `SystemLog` 增加 `dedupe_key`、route、waypoint_id、round_number、nav_goal_generation、localization_generation；`data` 保留阶段、实际值、门限、错误码和处理建议。
- 事件码固定为 `模块.对象.阶段`，阶段使用 requested/started/progress/applied/retrying/fallback/succeeded/cancelled/failed/sample。
- 仅显式 `dedupe_key` 的周期异常允许在同一 robot/trace/module/event 范围内合并，禁止跨任务、航点和候选合并。

### 后端与传输

- `system-logs` REST 接口升级为 `(occurred_at,id)` 复合游标，兼容旧时间游标，支持 trace、任务、命令、地图、路线和航点过滤。
- 新增 `/robots/{id}/system-logs/stream/`，以数据库游标提供多 worker 可用的 SSE；前端使用带 Authorization 的 fetch 流，不在 URL 中放长期 token；断线以2秒 REST轮询补偿。
- 非管理员从后端彻底过滤 DEBUG；DEBUG 只允许管理员限时开启，时长5/15/30分钟，采样率0.5/1/2/5Hz。
- INFO 最迟1秒刷新，WARNING/ERROR立即刷新；INFO/WARNING/ERROR使用QoS1和离线outbox，DEBUG使用QoS0且离线丢弃；DEBUG全局限流30条/秒。
- 新增受限 `waypoint.edit_confirmed` 诊断事件接口，路线保存日志只记录变化摘要，完整前后变量仅在DEBUG会话记录。

### Edge、ROS与流程事件

- 定位记录种子、NDT/VGICP指标、RTK漂移、位姿校正、样本年龄、协方差、接管和失败原因。
- 导航记录栈启动、目标接受、目标 generation、反馈、航点通过、停车校正、到达确认和真实 action 终态；中间点1米通过不记为到达。
- 避障记录障碍出现、CLEAR/SLOW/STOP状态变化、减速、停车、清图、后退、绕行和恢复耗尽。
- 主动重定位记录快速搜索、候选、门限拒绝、全局回退、最优候选提交和FAST-LIO接管。
- 航点记录稳定ID、坐标/yaw、定位方式、局部/全局规划器、避障、停留和语音配置变化。
- 规划器记录真实插件ID、Dijkstra/A*模式、路径耗时/长度/点数；MPPI记录P50/P90/P99；不上传完整点云、扫描帧或完整代价图，路径最多50个预览点。
- Edge订阅 `/planner/performance`、`/mppi/performance`、`/collision_monitor/state`、`/localization/decision` 和 `ScanMatchingStatus`，不转发整条 `/rosout`。

### 页面显示

- 路径规划页增加“流程”和“原始日志”两个视图。
- 流程视图按 trace 聚合阶段、耗时、结果、异常和折叠的DEBUG样本；支持复制trace和导出JSON。
- 原始视图支持级别、模块、时间、trace、任务、命令、路线、航点、关键词、历史加载、暂停恢复和地图定位。
- ERROR自动展开并滚动定位；WARNING只更新计数；SSE断开自动切换轮询。

## 实施顺序

1. 日志字段迁移、复合游标、权限、去重和协议校验。
2. HTTP/命令/任务/异步回调的 trace 传播。
3. Edge可靠批次、限流、刷新和离线策略。
4. 定位、导航、避障、规划器、MPPI、Collision Monitor 事件采集。
5. 稳定航点ID、保存差异日志和诊断确认接口。
6. 路径规划页流程/原始双视图、SSE和轮询兜底。
7. 后端、Edge、前端和导航ROS包测试后统一部署。

## 验收

- 地图下发→定位→重定位→导航启动→航点执行可由一个trace完整还原。
- 同一时间戳日志不漏、不重；不同任务不串日志、不错误去重。
- WARNING/ERROR页面延迟不超过2秒，DEBUG默认1Hz不影响MPPI 20Hz周期。
- 所有ERROR包含错误码、实际值、门限和处理建议。
- 日志、MQTT或数据库异常不影响导航、速度控制和安全停车。

## 默认与边界

- 保留当前工作树未提交的路径规划页和导航部署文件修改，不覆盖、不回退。
- 管理员沿用 Django `is_staff`；DEBUG默认15分钟、1Hz。
- 本计划增加诊断能力，不改变现有定位、导航、避障和安全判定。

## 执行状态（2026-09-06）

- [x] 后端字段、复合游标、SSE、权限、去重和诊断事件接口。
- [x] Edge/ROS trace 传播、可靠日志批次、限流和规划/避障性能事件。
- [x] 路径规划页流程/原始日志、筛选、错误定位和断线轮询补偿。
- [x] 前端构建、后端日志测试、ROS 规划器和 Collision Monitor 编译验证。
- [x] `waypoint.edit_confirmed` 白名单接口及覆盖测试。

说明：Edge 全量测试中仍有 12 项历史航点批处理断言与当前工作树既有的“每航点独立下发”行为不一致；不属于本计划日志功能回归。
