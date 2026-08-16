# 功能、界面与代码边界矩阵

本矩阵用于组织材料，不等同于最终源程序页。正式抽取前须逐文件核对著作权归属、第三方许可证和代码连续性。

| 编号 | 核心功能 | 主要界面证据 | 中心平台自研代码候选 | 机器人端自研代码候选 | 状态 |
| --- | --- | --- | --- | --- | --- |
| ZC-01 | 视频、在线状态、设备遥测、设备切换 | `DashboardOverview.vue`、机器人页面 | 状态模型、序列化、遥测服务、视频/状态API | `telemetry_collector.py`、`mqtt_client.py`、`app.py` | 已有基础实现；BMS、网络和温度真值需确认 |
| ZC-02 | AI事件、电子围栏、告警处置、统计 | 事件中心、统计页、`ZoneManagerPage.vue` | 事件模型、事件上传、告警服务、围栏API | AI事件上报、`alert_bridge.py`、`media_client.py` | 自行车事件链路已有实现；围栏联动需真机证据 |
| ZC-03 | 建图、地图版本、点位和路线 | `MapsPage.vue`、`RoutePlannerPage.vue` | 地图/路线模型、地图API、建图命令 | `mapping_adapter.py`、`map_set_coordinator.py`、地图激活适配 | 已有实现；现场建图及地图切换需验收截图 |
| ZC-04 | 任务模板、日历、下发、状态机 | `TasksPage.vue`、`PatrolCalendarPage.vue`、`TaskExecutionPage.vue` | 任务模型、任务服务、命令服务、执行API | `task_executor.py`、`command_processor.py`、`navigation_stack_adapter.py` | 主流程已有实现；自动日历触发需确认 |
| ZC-05 | 接管、运动控制、语音、定点干预 | `RemoteControlPage.vue`、`DashboardOverview.vue` | 远程命令API、音频/TTS/录音服务 | `teleop_control_adapter.py`、媒体播放/命令适配 | 接管、运动和喊话已有代码；返航不列为必备功能 |
| ZC-06 | 轨迹、任务记录、事件时间线、回放 | `TrackPlaybackPage.vue`、`TaskExecutionPage.vue` | 轨迹模型、轨迹API、告警时间线服务 | `trajectory_buffer.py`、`local_store.py` | 已有基础实现；自动报告不列为必备功能 |

## 代码抽取规则

1. 每项先建立文件白名单，再从核心业务入口向服务、模型和适配器连续抽取。
2. 避免将迁移文件、测试夹具、生成文件、压缩库、模型权重和第三方源码作为主体。
3. 六项可以出现少量公共协议与鉴权代码，但主要算法、业务流程和前后各30页应体现各自差异。
4. 代码页眉必须使用对应软件全称及V1.0，不能保留RoamerX作为登记主名称。
5. 不修改代码中的历史项目代号来制造名称差异；“无界公园”定位统一通过登记材料、产品界面和正式版本发布完成。

## 截图命名规则

使用 `ZC编号-序号-功能名称.png`，例如 `ZC01-03-机器人状态卡.png`。截图应隐藏密码、令牌、公网地址、个人信息和不宜公开的地图细节。
