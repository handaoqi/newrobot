# 统一定位、FAST-LIO2收敛、Nav2 Lifecycle与全入口状态机完整计划

> 状态：代码收尾完成；定向验证完成，现场联合验收待按安全场地执行
> 保存日期：2026-09-16  
> 适用范围：NX机器狗导航、定位、地图切换、任务启停、自动恢复与路径规划页状态展示

## 阶段归并规则

保留七阶段结构：FAST-LIO2状态、最终定位门、Lifecycle分组、全入口接入、RTK可信种子、页面与平台、联合验收。

## 总体目标

```text
prepare节点与地图安全组
→ FAST-LIO2收敛
→ NDT优先定位
→ 最优NDT提交
→ FAST-LIO + IMU接管
→ RTK/UKF/NDT二次校正
→ 连续3帧status=3最终验收
→ 激活导航执行组
→ navigation_allowed / nav_ready=true
```

`nav_ready` 只表示可安全下发导航目标，不表示进程存在、节点已configure或单帧 `status=3`。

## 当前实施进度（2026-09-16）

- 已完成：FAST-LIO状态发布与Edge订阅、连续本地里程计证据、NDT后二次校正的最终放行门。
- 已完成（代码与无运动验收）：Nav2地图/安全组与执行组分离；`prepare` 显式配置执行节点为 inactive，任务启动采用 `prepare → 定位 → activate_execution → Action验收`。现场已完成10轮无目标 activate/deactivate，7个执行节点最终均为 inactive，安全组保持 active；激活失败回滚由定向测试覆盖。
- 已完成（代码）：人工初始位姿、主动重定位、自动定位丢失恢复、`nav.start/restart/recover`、地图激活及分地图切换进入 NDT 优先事务；RTK fixed仅作为室外/过渡搜索候选。
- 已完成（代码）：路径规划定位时间线补入 prepare、FAST-LIO 收敛、RTK可信种子、最终放行和执行组激活；新增独立 Lifecycle、执行组与导航放行状态卡；地图激活前端不再重复触发定位。
- 已验证：`mapping` 目标构建成功；本轮Edge定向测试及全量测试 634 项、地图激活闭环定向测试 52 项、前端 183 项测试和生产构建通过。现场联合验收仍待执行。

## 阶段一：FAST-LIO2状态与Edge订阅

目的：把“等待收敛”变成可判断、可展示的真实状态；本阶段只增加观测，不改变运动行为。

- 在 `/lio_odometry/status` 发布 `initializing_imu`、`building_local_map`、`stabilizing_odometry`、`ready`、`degraded`、`failed`。
- 上报IMU初始化样本数/600、加速度/角速度方差和阈值、重置原因与次数、iKD-tree、有效SLAM位姿、里程计序号及健康错误。
- Edge订阅并确认：状态 `ready`、机器人停车、连续3帧时间戳递增且年龄不超过0.50秒的 `/odom/lio_odom`。
- 此阶段不要求地图定位 `status=3`，因为尚未做NDT全局定位。
- 不重置、不改写FAST-LIO2 ESKF、iKD-tree和原始轨迹。
- 验收：静止初始化、移动导致重新计数、点云不足、LIO断流、健康恢复均有明确状态与原因。

## 阶段二：NDT门限与最终放行统一

目的：消除“定位节点显示正常、Edge却不允许导航”的口径冲突。

- C++定位配置和Edge候选筛选统一NDT score `<0.40`、内点率 `≥0.50`、连续稳定3帧。
- 保留稳定帧XY差 `≤0.10m`、yaw差 `≤2°`、种子修正 `≤1.50m/30°`；score `<0.01` 可提前结束搜索。
- 最优NDT提交后，验证FAST-LIO接管、锚点有效、锚点代数与本次事务一致、二次校正终态。
- 最终放行要求连续3帧新鲜 `/localization_info.status=3`，单帧年龄 `≤0.50秒`，活动主源和连续主源均为 `lio_imu`。
- 另要求：`lio_handoff_pending=false`、无LIO运动异常、`absolute_stable=true`、导航执行组active、FollowWaypoints可用。
- Collision Monitor和速度桥继续独立使用新鲜 `status=3` 作为硬停止条件。
- 验收：单帧status=3、非LIO主源、锚点不一致、接管等待中或二次校正未结束时均不得放行。

## 阶段三：Nav2 Lifecycle分组控制

目的：定位未完成时，Nav2不能产生导航控制。

- 全部Nav2节点先configure，两个Lifecycle Manager均设置 `autostart=false`。
- 地图安全组保持active：`map_server`、`filter_mask_server`、`costmap_filter_info_server`、`collision_monitor`。
- 执行组保持inactive：规划、控制、平滑、行为树、速度优化、航点跟随。
- 实现 `prepare()`、`activate_execution()`、`deactivate_execution()`、`reconcile()`、`shutdown()`。
- 激活失败反序回滚执行组，保持 `nav_ready=false`；Edge重启后根据ROS实际状态对账。
- 新增 `stack_prepared` 与严格的 `wait_until_prepared()`；保留 `wait_until_ready()` 仅表示可以发目标。
- 验收：无目标时连续10次configure/activate/deactivate无非零速度、无节点残留，部分节点激活失败可安全回滚。

## 阶段四：统一接入全部入口与安全暂停恢复事务

目的：所有入口走同一条状态机，不再有RTK旁路、旧回调残留或切图重启分叉。

- 冷启动：prepare → LIO收敛 → NDT定位 → 接管/二次校正 → 最终门 → 激活执行组。
- `nav.start/restart/recover`：成功标准为 `navigation_allowed`；LIO健康时优先deactivate→重定位→activate，只有LIO或进程故障才完整重启。
- `nav.stop`：取消目标、确认零速、deactivate后关闭Nav2；保留定位链。
- `task.start`：拆分为任务冲突、急停、电量、地图/路线、人工接管等预校验；prepare和定位后，再做定位稳定、`nav_ready`、Action最终校验并下发首航点。
- `task.pause`：普通人工暂停只取消目标和停车，保持热Nav2；`task.resume` 仅在普通暂停时直接恢复。
- `task.cancel/force_exit`：取消目标、零速、取消定位事务和旧回调代数，保留热Nav2。
- 人工 `nav.initial_pose/nav.relocalize`：运行任务时自动取消当前目标、确认停车、停用执行组；定位完成后重新激活并从当前航段自动恢复。
- 自动定位丢失：保存当前航点模式、任务与地图快照，停车、停用执行组；通过统一定位和最终门后重发当前航段。
- `nav.single_goal`：仅在 `navigation_allowed` 且没有活动巡检任务时允许。
- `map.activate`：停用执行组→重载PCD/栅格→统一定位→激活执行组；命令完成即地图可导航。
- 分地图任务和停靠切图：替换 `switch_map()` 的full-stop重启，统一使用内部地图过渡：停用→换图→定位→激活。
- 建图启动可显式全停导航；建图后再次导航从完整冷启动定位链开始。
- 所有入口使用 `lifecycle_operation_id`、定位事务ID和 `leg_generation`；取消、新任务、切图、强退时统一使旧操作失效。
- 验收：人工定位、自动恢复、切图和任务取消不能并发推进旧航段或旧Action。

## 阶段五：RTK fixed可信搜索种子

目的：室外/过渡场景利用RTK缩小NDT搜索范围，但不让RTK绕过地图匹配。

- fixed RTK须通过位置、航向、新鲜度、连续3帧有效样本和位置跨度 `≤0.30m` 验证。
- 合格后作为 `trusted_seed` 传给NDT；只影响候选排序，不能直接完成初始定位。
- 应用于冷启动、地图激活、任务启动、主动重定位、自动恢复、分地图和停靠切图。
- NDT仍须产生最优候选、提交并完成FAST-LIO接管。
- RTK float、过期、航向无效或不稳定时立即跳过，不等待。
- 室内不读取、不等待RTK。
- UKF保持：NDT `<0.10` 只用NDT；`0.10≤NDT<0.40` 且float偏差 `≤0.40m` 才融合；float偏差 `>0.40m` 禁止引入。
- 验收：RTK仅辅助候选，不得绕过NDT提交或使室内等待RTK。

## 阶段六：路径规划页与平台状态

目的：让现场能看清卡在哪一环，而不是只看到“等待”或“status=3”。

- 上半时间线：进程、Lifecycle配置、地图安全组激活、LIO收敛、RTK种子、NDT候选、最优提交、接管、二次校正、连续status=3、执行组激活、Action验证。
- 下半时间线：目标下发、规划/平滑、路径跟踪、到点停车、校正、微靠近、独立转向、验收、动作、下一航点。
- 退化、障碍恢复、缺少证据、跳过RTK、复用已验证定位作为分支显示。
- Edge写入每个阶段的开始时间、结束时间、质量数据、采用/拒绝结论和失败原因；保留最近一次启动或重定位会话。
- 新增 `navigation.lifecycle`、`navigation.runtime`、`stack_prepared`、`navigation_allowed` 展示。
- `nav_ready=false` 时，页面按“准备中/定位中/退化中”展示，不得笼统显示为导航故障。
- 地图激活、初始化定位、主动重定位、任务启动共用相同阶段文案；前端不再二次触发定位。
- 验收：页面可完整重建启动、定位、航段、到点、障碍与定位恢复过程，消除“待执行/阶段时间未上报”假状态。

## 阶段七：联合验收

目的：确认不是只在单一理想场景可用。

- 覆盖室内、室外、过渡三场景。
- 覆盖NDT、RTK、UKF三种航点校正模式。
- 覆盖冷启动、`nav.start/restart/recover/stop`、切图、分地图、停靠切图、任务启动、普通暂停/取消、人工定位、主动重定位、定位丢失恢复。
- 注入IMU移动、LIO断流、NDT失败、RTK失效、节点激活失败、Action不可用、Edge重启、取消恢复事务。
- 先完成无运动10次Lifecycle切换，再在安全场地验证自动恢复后只重发当前航段。
- 验收所有失败路径均保持 `nav_ready=false`、无非零导航速度、无旧任务回调或旧地图锚点残留。

## 最终验收标准

只有FAST‑LIO稳定、NDT定位已提交、LIO已接管、二次校正结束、连续3帧 `status=3`、主源为 `lio_imu`、Nav2执行组active时，机器人才能开始导航。

重定位过程中不得重启FAST‑LIO或清空iKD-tree；显式 `nav.stop`、完整进程故障恢复和建图启动除外。

## 现场联合验收边界

代码侧故障分支和无运动 Lifecycle/地图切换路径已经完成定向验证。由于本次部署不下发运动目标，室内、室外、过渡三场景的真实点云、RTK 与故障注入联合验收需在安全场地按阶段七执行；这不影响本次代码门禁，现场未验证时导航仍由最终 LIO/NDT 门严格阻止放行。
