# RoamerX 三端数据架构

更新时间：2026-08-21
适用范围：云平台、NX Edge Agent、3588 厂商运动控制系统

## 1. 架构结论

```text
云平台 Django 业务库
  ├─ 设备、地图、路线、任务、事件、媒体元数据
  ├─ 命令和任务状态机审计
  └─ 遥测、轨迹、AI 开发与语音识别历史
             ↑ MQTT/HTTP
NX Edge SQLite
  ├─ 命令幂等缓存
  ├─ 活动任务恢复上下文
  ├─ 断线消息 outbox
  └─ 最后可信定位等本机元数据
             ↓ SSH/厂商接口
3588 运控
  └─ 无项目数据库；仅有充电模式状态文件
```

| 端 | 当前持久化位置 | 数据所有权 |
| --- | --- | --- |
| 云平台 | 生产 SQLite `/opt/roamerx/shared/db.sqlite3` | 全局业务事实和审计历史 |
| NX Edge | `/home/dogrobot/runtime/nx-edge/data/edge-agent/edge.db` | 断网可恢复的本机执行状态 |
| 3588 | `/var/lib/roamerx-charge-pile/state` | 厂商充电程序当前模式，不是数据库 |

地图、PCD、PGM、模型、媒体、rosbag 和音频是文件资产。平台数据库保存文件元数据或 URL；Edge SQLite 不保存大文件。

## 2. 初始化记录

### 2.1 平台基础记录

正式空库初始化创建：

- Django 内容类型、权限和迁移版本。
- 1 个由 `PLATFORM_OPERATOR_*` 指定的操作员。
- 1 台由 `PLATFORM_ROBOT_*` 指定的主机器人，初始为离线、待命。
- 11 个监测中心播报模板。
- 4 个告警技能绑定：自行车、发现障碍、避障、劝阻。

地图、路线、任务、排班、设备凭据和历史事件不是基础记录。只有开发演示环境可以显式使用 `--with-demo-data`。

### 2.2 监测中心 11 个播报模板

| 分类 | 模板 | 初始化正文 |
| --- | --- | --- |
| 监测告警 | 重点路段 | 您好，当前区域为巡检重点路段，请勿长时间占道停留。 |
| 监测告警 | 注意避让 | 您好，系统检测到现场存在安全风险，请注意避让并配合引导。 |
| 监测告警 | 驶离提醒 | 您好，这里禁止自行车长时间停放，请尽快驶离指定区域，感谢配合。 |
| 巡检智能播报 | 森林火灾 | 共享森林美景，严防森林火灾 |
| 巡检智能播报 | 公园南门 | 太阳宫南门入口设置蓝色健身步道起点，常举办春日牡丹主题亲子游园活动；临近便民休息座椅、饮水点，适配日常散步、亲子休闲 |
| 巡检智能播报 | 赏花 | 赏花不采花，文明你我他 |
| 巡检智能播报 | 无界公园介绍 | 太阳宫公园是朝阳区首批试点改造的无界公园之一。无界公园的核心理念：拆除公园围墙、围栏，打破城市道路与绿地的物理边界，公园绿化景观与城市街区无缝融合，市民可随时随地就近进入绿地，不用专门寻找出入口 |
| 巡检智能播报 | 发现障碍物 | 前方巡检线路有障碍物,正在避让 |
| 巡检智能播报 | 后退尝试避障 | 请注意避障离开巡检路线，并配合公园管理引导 |
| 巡检智能播报 | 劝阻离开线路 | 任务受阻无法绕行，请您配合离开巡检线路 |
| 设备管控 | 自动充电失败 | 自动充电失败，请在机器人管理页查看问题提示，或手工更换电池 |

迁移 `0054_seed_eleven_speech_templates` 固化首次建库内容。后续运行 `initialize_platform` 只补缺失记录，不覆盖页面已经修改的正文或技能绑定。

### 2.3 Edge 与 3588 初始状态

Edge 五张表初始均为空，不创建默认任务、假定位或默认地图。机器人编号、MQTT、RTK、话题和安全阈值来自 YAML 配置。3588 不建库，只在状态文件缺失时初始化为 `unknown`；已有 `lying` 状态必须保留。

## 3. 云平台业务表

| 表 | 责任 |
| --- | --- |
| `monitoring_robot` | 机器人主数据、在线状态、当前地图、视频流与回充配置 |
| `monitoring_robotpersondetectionstate` | 最新视频帧检测框快照，不作为告警历史 |
| `monitoring_patroltask` | 可复用巡检任务模板 |
| `monitoring_calendarday` | 节假日、调休日和特殊活动日 |
| `monitoring_patrolschedule` | 日常、节假日或单次巡检排班 |
| `monitoring_schedulerun` | 排班触发、跳过和失败审计 |
| `monitoring_inspectionevent` | 识别告警及人工处置结果 |
| `monitoring_robottelemetry` | Edge 历史遥测和原始报文 |
| `monitoring_mediaasset` | 抓拍图和视频片段元数据 |
| `monitoring_robotcommand` | 旧版页面控制、音频与充电命令 |
| `monitoring_speechcategory` | 播报模板分类 |
| `monitoring_speechtemplate` | TTS/录音播报模板 |
| `monitoring_alertskillbinding` | 告警技能与播报模板绑定 |
| `monitoring_recordedaudio` | 上传录音、ASR 文本和识别状态 |
| `monitoring_mapdata` | 地图文件、轨迹、原点和活动状态 |
| `monitoring_mapset` | 大场景多子图集合 |
| `monitoring_mapsetmember` | 子图顺序和成员元数据 |
| `monitoring_patrolroute` | 地图途径点、方向和路线参数 |
| `monitoring_zone` | 禁入、警告和限行区域 |
| `monitoring_track` | 旧版整段轨迹汇总 |
| `monitoring_robotcredential` | 设备凭据哈希、证书指纹和吊销状态 |
| `monitoring_robotsession` | Edge 连接会话、boot ID 和能力 |
| `monitoring_taskexecution` | 单次任务执行状态、轮次与进度 |
| `monitoring_taskexecutionevent` | 任务事件、定位丢失点和恢复结果 |
| `monitoring_remotecommand` | 新命令协议生命周期 |
| `monitoring_commandevent` | 命令发布、接受、执行和结束事件 |
| `monitoring_inboundmessage` | MQTT 入站消息去重与处理状态 |
| `monitoring_robotstatuslatest` | 每台机器人最新状态快照 |
| `monitoring_trajectorypoint` | 任务逐点定位轨迹 |
| `monitoring_trajectorybatchreceipt` | 轨迹批次幂等回执 |
| `monitoring_developmentagentstate` | 远程开发 Agent 状态 |
| `monitoring_developmentconversationstate` | AI 会话模式、模型和工作区 |
| `monitoring_developmenttask` | AI 开发或语音命令任务 |
| `monitoring_developmenttaskevent` | AI 任务实时输出和步骤 |
| `monitoring_voicerecognitionevent` | ASR、唤醒和关联任务审计 |

## 4. Django 框架表

| 表组 | 责任 |
| --- | --- |
| `auth_user`、`auth_group`、`auth_permission` | 用户、角色和权限 |
| `auth_user_groups`、`auth_user_user_permissions`、`auth_group_permissions` | 权限关联关系 |
| `authtoken_token` | REST API Token |
| `django_content_type` | 模型类型注册 |
| `django_migrations` | 已应用迁移版本，禁止手工修改 |
| `django_session` | Web 会话 |
| `django_admin_log` | Django Admin 操作审计 |

## 5. NX Edge 表

| 表 | 责任 |
| --- | --- |
| `processed_commands` | 缓存 ACK/结果，避免重发命令重复执行 |
| `task_context` | 任务状态、路线快照和当前途径点 |
| `outbox` | MQTT 断线期间待补发消息 |
| `trajectory_sequence` | 每个任务下一轨迹序号 |
| `agent_metadata` | 最后可信定位等键值元数据 |

## 6. 一致性原则

1. 云平台是任务和命令审计的最终事实源，Edge 只保留恢复执行所需的最小副本。
2. `message_id`、命令 ID、任务 ID 和轨迹序号负责跨端幂等，禁止用时间戳代替主键。
3. 地图 ID 与版本必须同时上报；定位丢失事件必须保存对应地图和最后可信位姿。
4. 云平台数据库不能复制到 Edge，Edge 数据库也不能作为云端业务恢复源。
5. 3588 厂商配置、egg 和 MCU 参数属于设备镜像边界，不写入 Django 或 Edge SQLite。
