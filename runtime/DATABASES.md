# 三端数据库与初始化

统一维护的架构分析和操作手册位于：

- [`docs/architecture/THREE_ENDPOINT_DATA_ARCHITECTURE.md`](../docs/architecture/THREE_ENDPOINT_DATA_ARCHITECTURE.md)
- [`docs/operation-manual/DATABASE_INITIALIZATION_MANUAL.md`](../docs/operation-manual/DATABASE_INITIALIZATION_MANUAL.md)

本文保留运行时速查内容；设计依据和完整操作步骤以上述 `docs/` 文档为准。

## 存储边界

| 端 | 持久化存储 | 建库方式 | 初始记录 |
| --- | --- | --- | --- |
| 云平台 | Django 业务库；当前生产为 `/opt/roamerx/shared/db.sqlite3` | Django migrations | 系统权限、播报模板、告警技能、部署指定的操作员与机器人 |
| NX Edge | `runtime/nx-edge/data/edge-agent/edge.db` | `LocalStore` SQLite schema | 无业务种子；所有表初始为空 |
| 3588 运控 | 无项目数据库 | 不建库 | 仅 `/var/lib/roamerx-charge-pile/state=unknown` |

地图、模型、媒体、rosbag 和音频是文件资产，不写入 Edge SQLite。平台数据库只保存这些文件的元数据或 URL。

## 云平台初始化

Docker 新部署：

```bash
runtime/platform/bin/platformctl init
# 修改 runtime/platform/conf/platform.env，至少设置各类密钥和 PLATFORM_OPERATOR_PASSWORD
runtime/platform/scripts/prepare-mqtt-password.sh
runtime/platform/bin/platformctl init-db
runtime/platform/bin/platformctl up
```

`init-db` 依次执行 `migrate --noinput` 和 `initialize_platform`。容器每次启动也会幂等执行这两步。旧 systemd 部署使用：

```bash
set -a; source /opt/roamerx/shared/center.env; set +a
cd /opt/roamerx/current
/root/miniconda/envs/py310/bin/python backend/manage.py migrate --noinput
PLATFORM_OPERATOR_PASSWORD='<strong-password>' \
  /root/miniconda/envs/py310/bin/python backend/manage.py initialize_platform
```

基础初始化只补缺失项，不覆盖页面上已经修改的播报文字或技能绑定：

- 11 个监测中心模板：`重点路段`、`注意避让`、`驶离提醒`、`森林火灾`、`公园南门`、`赏花`、`无界公园介绍`、`发现障碍物`、`后退尝试避障`、`劝阻离开线路`、`自动充电失败`。
- 这些模板使用 `监测告警`、`巡检智能播报`、`设备管控` 三个分类；历史迁移留下的空分类允许保留。
- 4 个告警技能绑定：自行车、发现障碍、避障、劝阻。
- 1 个操作员，由 `PLATFORM_OPERATOR_*` 配置；首次创建必须显式提供密码。
- 1 台主机器人，由 `PLATFORM_ROBOT_*` 配置；初始为离线、待命状态。

历史迁移 `0011` 曾无条件创建硬编码机器人。迁移 `0053` 只清理没有任何关联记录的旧占位项；生产中已有地图、任务或遥测关联的机器人绝不会被删除。

地图、路线、任务、排班、设备凭据和历史事件不是基础配置，不自动伪造。仅开发演示可追加 `--with-demo-data`。`ENABLE_DEMO_SEED=false` 时页面访问和登录不会偷偷创建演示数据。

## 云平台业务表

| 表 | 作用 |
| --- | --- |
| `monitoring_robot` | 机器人主数据、在线状态、当前地图、视频流和回充路线 |
| `monitoring_robotpersondetectionstate` | 当前视频帧的人员检测框快照，不作为告警历史 |
| `monitoring_patroltask` | 可复用的巡检任务模板及关联路线 |
| `monitoring_calendarday` | 节假日、调休日和特殊活动日配置 |
| `monitoring_patrolschedule` | 日常、节假日或单次巡检排班规则 |
| `monitoring_schedulerun` | 每次排班触发、跳过或失败的审计记录 |
| `monitoring_inspectionevent` | 自行车、障碍等识别告警及人工处置结果 |
| `monitoring_robottelemetry` | Edge 上报的历史遥测和原始报文 |
| `monitoring_mediaasset` | 抓拍图、视频片段的文件元数据 |
| `monitoring_robotcommand` | 旧版页面控制、音频和充电命令队列 |
| `monitoring_speechcategory` | 播报模板分类 |
| `monitoring_speechtemplate` | TTS/录音播报模板 |
| `monitoring_alertskillbinding` | 告警技能到播报模板的绑定和启停状态 |
| `monitoring_recordedaudio` | 上传录音、ASR 文本和识别状态 |
| `monitoring_mapdata` | 地图文件、缩略图、轨迹、原点和活动状态 |
| `monitoring_mapset` | 大场景多子图集合及全局清单 |
| `monitoring_mapsetmember` | 子图在地图集合中的顺序和元数据 |
| `monitoring_patrolroute` | 地图上的途径点、方向和路线配置 |
| `monitoring_zone` | 地图禁入区、警告区和限行区 |
| `monitoring_track` | 旧版整段轨迹汇总；新任务轨迹主要使用 `trajectorypoint` |
| `monitoring_robotcredential` | 设备凭据哈希、证书指纹和吊销状态 |
| `monitoring_robotsession` | 每次 Edge MQTT/WSS 连接会话和 boot ID |
| `monitoring_taskexecution` | 一次任务执行的当前状态、轮次和进度 |
| `monitoring_taskexecutionevent` | 任务状态机事件、定位丢失点和恢复结果 |
| `monitoring_remotecommand` | 新命令协议的完整命令生命周期 |
| `monitoring_commandevent` | 命令发布、接受、执行和结束事件 |
| `monitoring_inboundmessage` | MQTT 入站消息去重、处理状态和原始载荷 |
| `monitoring_robotstatuslatest` | 每台机器人最新状态快照，供页面快速刷新 |
| `monitoring_trajectorypoint` | 任务期间逐点定位轨迹 |
| `monitoring_trajectorybatchreceipt` | 轨迹批次幂等回执和序号范围 |
| `monitoring_developmentagentstate` | 远程开发 Agent 在线状态和工作区能力 |
| `monitoring_developmentconversationstate` | 小太阳主会话的模式、模型和工作区 |
| `monitoring_developmenttask` | 云端 AI 开发/语音命令任务 |
| `monitoring_developmenttaskevent` | AI 任务的实时输出、步骤和结果 |
| `monitoring_voicerecognitionevent` | ASR 文本、唤醒结果及关联开发任务审计 |

## Django 框架表

| 表组 | 作用 |
| --- | --- |
| `auth_user`、`auth_group`、`auth_permission` | 用户、角色和权限 |
| `auth_user_groups`、`auth_user_user_permissions`、`auth_group_permissions` | 用户、角色和权限多对多关系 |
| `authtoken_token` | REST API 登录 Token |
| `django_content_type` | Django 模型类型注册表 |
| `django_migrations` | 已应用迁移版本，禁止手工删除 |
| `django_session` | Web 会话；当前 Token 登录时通常为空 |
| `django_admin_log` | Django Admin 操作审计 |

权限、内容类型和迁移版本由 `migrate` 生成，不是人工业务配置。

## NX Edge SQLite

初始化或校验现有库：

```bash
runtime/nx-edge/bin/nxctl init-db
```

命令会执行 SQLite `integrity_check`，保留现有数据，不会生成任务或假定位：

| 表 | 作用 |
| --- | --- |
| `processed_commands` | 命令 ACK/结果幂等缓存，防止断线重发后重复执行 |
| `task_context` | 活动与历史任务状态、路线快照、当前途径点 |
| `outbox` | MQTT 断线期间待补发的可靠消息 |
| `trajectory_sequence` | 每个任务下一条轨迹序号 |
| `agent_metadata` | 键值元数据，目前保存各地图最后可信定位 |

新库五张表均为空。机器人 ID、MQTT、RTK、传感器主题和安全阈值属于 `runtime/nx-edge/conf/edge-agent.yaml`，不是数据库种子。

## 3588 初始化

3588 的厂商 `robot-launch`、egg 定义和充电程序不使用本项目数据库。部署后只需：

```bash
runtime/3588-motion/scripts/deploy.sh
ssh 3588 /home/firefly/dogrobot-runtime/bin/motionctl init-state
```

`init-state` 仅在状态文件不存在时写入 `unknown`，已有 `lying` 状态会被保留；它不启动腿部运控，也不触发充电动作。

## 备份原则

- 云平台同时备份数据库与 `media`，否则事件图片元数据和文件会断开。
- NX 同时备份 `edge.db`、地图和配置；不要把云平台数据库复制到 NX。
- SQLite 备份前使用项目备份脚本或先停止写入服务，禁止直接复制正在高频写入的 WAL 临时状态。
- 3588 只备份仓库管理的配置和状态文件，不复制厂商运行镜像作为数据库备份。
