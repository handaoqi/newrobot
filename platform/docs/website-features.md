# 网站功能说明

[返回文档中心](./project-docs-index.md)

## 1. 页面路由

路由文件：`frontend/src/router/index.js`。

| 路径 | 页面 | 说明 |
| --- | --- | --- |
| `/` | 重定向 | 跳转到 `/dashboard/overview` |
| `/login` | 登录页 | 用户登录 |
| `/dashboard/overview` | 实时监测中心 | 视频、概览、远程控制台、历史事件 |
| `/dashboard/guard-duty` | 保安值守 | 直播值守、地图、任务和收音 |
| `/dashboard/remote-control` | 远程控制 | 机器人远程接管与动作控制 |
| `/dashboard/remote-development` | 远程 AI 开发 | 远程开发会话与任务 |
| `/dashboard/analytics` | 统计分析中心 | 指标卡片和趋势图 |
| `/dashboard/events` | 事件中心 | 事件筛选、搜索、排序、复核归档 |
| `/dashboard/robots` | 机器人管理 | 机器人列表与详情 |
| `/dashboard/tasks` | 巡检任务 | 巡检任务列表 |
| `/dashboard/tasks/calendar` | 巡检日历 | 计划执行与日历查看 |
| `/dashboard/task-executions/:executionId` | 任务执行详情 | 执行状态、轨迹和事件 |
| `/dashboard/tasks/maps` | 地图管理 | 地图上传、建图、清理与活动地图 |
| `/dashboard/tasks/routes` | 路径规划 | 地图选点、路线保存与导航测试 |
| `/dashboard/tasks/zones` | 禁区管理 | 地图关联的禁区配置 |
| `/dashboard/tasks/scene-visualizer` | 场景视角调试 | 只读查看地图场景、点云、路线和定位证据 |
| `/dashboard/tasks/tracks` | 轨迹回放 | 历史轨迹列表与回放 |

路由守卫规则：

- 带 `meta.requiresAuth` 的页面必须存在 `localStorage.inspection_token`。
- 未登录访问工作台会跳转到 `/login`。
- 已登录访问 `/login` 会跳转到 `/dashboard/overview`。

## 2. 登录页

文件：`frontend/src/views/LoginPage.vue`。

主要功能：

- 默认填入演示账号 `operator / admin123456`。
- 调用 `POST /api/auth/login/` 验证账号。
- 登录成功后写入：
  - `localStorage.inspection_token`
  - `localStorage.inspection_user`
- 跳转到实时监测中心。
- 登录失败时展示后端返回的错误信息。

页面内容：

- 平台介绍文案。
- 登录表单。
- 演示账号提示。

## 3. 工作台布局

文件：`frontend/src/views/DashboardLayout.vue`。

主要区域：

| 区域 | 功能 |
| --- | --- |
| 左侧导航 | 监测中心、统计分析、事件中心、机器人管理、巡检任务 |
| 品牌区 | 平台名称和英文标识 |
| 值守提醒 | 固定提示建议 |
| 顶部栏 | 当前页面标题、主题切换、当前用户、退出登录 |
| 内容区 | 当前子页面 |

退出登录行为：

- 删除 `localStorage.inspection_token`。
- 删除 `localStorage.inspection_user`。
- 跳转到 `/login`。

主题切换：

- 使用 `useTheme()` composable 管理。

## 4. 实时监测中心

文件：`frontend/src/views/DashboardOverview.vue`。

接口：

- `GET /api/dashboard/overview/`
- `GET /api/robots/`
- `POST /api/robots/<robot_id>/commands/`

主要功能：

### 4.1 顶部状态摘要

展示：

- 当前设备编号。
- 今日告警数。
- 当前区域。

数据来源：`overview.header`。

### 4.2 实时视频流监控

展示机器人前端画面。

播放策略：

1. 若 `latest_robot.play_urls.flv` 存在，并且浏览器支持 MSE FLV，则使用 `mpegts.js` 播放 HTTP-FLV。
2. 若 FLV 不可用但 `play_urls.hls` 存在，则使用 `hls.js` 播放 HLS。
3. 若浏览器原生支持 HLS，则直接设置 video `src`。
4. 若没有实时流，则显示事件抓拍图或默认图片。

叠加信息：

- 巡检位置。
- 视频流 ID。

底部信息：

- 今日巡检时长。
- 当前巡检区域。
- 设备电量。
- 紧急停止按钮。

当前远程控制采用“手柄模式 / 远程接管模式”互斥设计。默认处于手柄模式，板端不初始化 SDK；点击“接管”后下发 `takeover_enter`，板端才初始化 SDK 并进入远程接管。接管状态下紧急停止下发 `passive`，退出接管下发 `takeover_exit`，板端先停止或进入被动状态，再尽量释放 SDK 控制权。

### 4.3 运行概况

展示：

- 在线机器人数量。
- 待处理事件。
- 已处理事件。
- 今日告警。

数据来源：`overview.summary`。

### 4.4 设备列表

展示所有机器人：

- 名称。
- 位置。
- 状态。
- 电量。

数据来源：`GET /api/robots/`。

### 4.5 远程控制台

功能：

- 编辑喊话文本。
- 快速切换预置喊话模板。
- 选择预置音频或填写机器狗可访问的自定义音频 URL。
- 在浏览器中预览音频。
- 使用浏览器麦克风录音、上传并下发到机器狗播放。
- 展示当前音量和链路状态。

当前喊话链路会创建 `play_audio` 命令，由机器狗携带设备凭证轮询领取，下载音频并通过 USB 音箱播放；执行状态会回报为运行中、完成或失败。

### 4.6 历史事件识别

展示当前机器人最近事件：

- 抓拍图。
- 事件标题。
- 位置。
- 时间。
- 待处理状态或复核结论。

数据来源：`latest_robot.recent_events`。

## 5. 统计分析中心

文件：

- `frontend/src/views/AnalyticsPage.vue`
- `frontend/src/components/TrendLineChart.vue`

接口：

- `GET /api/dashboard/analytics/`

页面组成：

| 区域 | 内容 |
| --- | --- |
| 指标卡片 | 累计预警、检测识别、平均完成度、值守响应 |
| 趋势图 | 预警次数、检测次数、巡检时长、执行里程 |

趋势图能力：

- 使用 ECharts 折线图。
- 展示近 7 个周期数据。
- 展示最新值、较上一周期变化、7 期累计、平均值。
- 图表带平均值参考线。

说明：当前部分统计值是演示计算逻辑，趋势序列在后端 `build_analytics_payload()` 中生成。

## 6. 事件中心

文件：`frontend/src/views/EventsPage.vue`。

接口：

- `GET /api/events/`
- `POST /api/events/<event_id>/handle/`

### 6.1 事件筛选

状态筛选：

- 全部记录。
- 待处理。
- 已处理。

默认筛选：`pending`。

### 6.2 搜索

搜索输入支持：

- 标题。
- 地点。
- 机器人名称。
- 机器人编号。
- 事件类型。
- 描述。
- 处置备注。

按回车或点击查询触发。

### 6.3 排序

排序选项：

- 最新优先：`detected_desc`。
- 最早优先：`detected_asc`。
- 高风险优先：`risk_desc`。
- 置信度优先：`confidence_desc`。

### 6.4 分页加载

前端默认 `pageSize=8`。

列表滚动到底部附近时会自动加载下一页；也提供“加载更多事件”按钮。

### 6.5 事件详情

选中事件后展示：

- 事件标题。
- 机器人名称。
- 位置。
- 风险等级。
- 抓拍图或后端生成的标注图。
- 识别置信度。
- 关联视频流。
- 检测框坐标。
- 处置备注。
- 复核结论。

图片选择顺序：

1. `annotated_snapshot_url`。
2. `snapshot_url`。
3. 前端默认演示图片。

### 6.6 复核归档

仅 `pending` 事件可归档。

可选复核结论：

- 确认违规：`confirmed`。
- 怀疑：`suspected`。
- 误报：`false_alarm`。

点击“完成复核并归档”后：

- 调用 `POST /api/events/<event_id>/handle/`。
- 提交 `status=resolved`。
- 提交处置备注。
- 提交复核结论。
- 成功后刷新事件列表。

## 7. 机器人管理

文件：`frontend/src/views/RobotsPage.vue`。

接口：

- `GET /api/robots/`
- `GET /api/robots/<robot_id>/`

主要功能：

- 左侧展示机器人列表。
- 点击机器人后加载详情。
- 详情展示：
  - 名称、编号、区域。
  - 当前模式。
  - 电量。
  - 网络强度。
  - 扬声器音量。
  - 固件版本。
  - 当前执行任务。
  - 近期识别事件。

## 8. 巡检任务

文件：`frontend/src/views/TasksPage.vue`。

接口：

- `GET /api/tasks/`

展示字段：

- 任务名称。
- 机器人名称。
- 路线名称。
- 任务状态。
- 完成度。

当前页面以列表展示为主，未提供新增、编辑、暂停、完成等控制接口。

## 9. 地图、路线与执行页面

地图管理、路径规划、禁区、日历、执行详情和轨迹回放分别对应：

- `MapsPage.vue`：上传/同步地图，室内或室外现场建图，显示地图包文件、建图轨迹、关键帧和全局 ENU；最终建图指标只在完整地图包上传后展示。
- `RoutePlannerPage.vue`：选择地图并编辑途经点、方向、定位方式、避障和播报；地图区支持轨迹/关键帧查看，导航测试区显示地图一致性、定位质量和导航栈状态。
- `ZoneManagerPage.vue`：按地图筛选、添加、编辑、启用或删除禁区。
- `PatrolCalendarPage.vue`、`TaskExecutionPage.vue` 和 `TrackPlaybackPage.vue`：分别查看计划、执行详情和历史轨迹。

地图与导航页面都有轮询或窗口/动画监听，离开页面时必须清理；路线执行、导航启动/停止、传感器重启、地图清理/删除和活动地图切换均属于副作用操作，按钮禁用、确认提示和错误反馈不能省略。

## 10. 前端状态与容错

| 场景 | 当前处理 |
| --- | --- |
| 未登录访问工作台 | 跳转登录页 |
| 登录失败 | 展示错误信息 |
| 页面加载中 | 部分页面展示加载文案 |
| 事件列表为空 | 展示空状态 |
| FLV 不可播放 | 尝试 HLS |
| 没有实时流 | 显示静态图片 |
| 控制类按钮 | 远程控制按当前控制通道下发；喊话通过独立音频命令队列下发并接收板端执行结果 |

## 11. 可扩展功能建议

后续若从演示升级到真实业务，可补充：

- 远程控制扩展：返航、更多动作和细粒度权限控制。
- WebSocket 或 SSE：实时刷新事件、机器人状态和视频流状态。
- 任务管理：创建任务、分配机器人、暂停/恢复/完成任务。
- 事件批量操作：批量归档、导出、复核流转。
- 多机器人地图：轨迹、位置、巡检路线。
- 权限分级：管理员、值班员、只读访客。
