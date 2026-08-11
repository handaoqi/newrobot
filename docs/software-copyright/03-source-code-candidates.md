# 源程序候选文件清单

路径以当前工作区为准。正式交存前需复制到独立材料工作区、去除非自研部分并按连续代码页排版；本清单不会改动现有源代码。

## 公共中心平台根目录

`/home/robot/yw/roamerx_analysis/center_platform/platform_server_code`

## 公共机器人端根目录

`/home/robot/genisom_roamerx_open/edge_agent`

## ZC-01 实时监测软件

优先候选：

- `frontend/src/views/DashboardOverview.vue`
- `frontend/src/views/RobotsPage.vue`
- `frontend/src/services/api.js`中的状态与机器人接口部分
- `backend/monitoring/services/telemetry_service.py`
- `backend/monitoring/message_handlers.py`中的presence和status处理部分
- `backend/monitoring/models.py`中的Robot、RobotSession、RobotStatusLatest和RobotTelemetry
- 机器人端`roamerx_edge/telemetry_collector.py`
- 机器人端`roamerx_edge/app.py`中的状态采集与上报流程

## ZC-02 事件预警软件

优先候选：

- 事件中心和统计分析对应的前端视图
- `frontend/src/views/ZoneManagerPage.vue`
- `backend/monitoring/services/alert_service.py`
- `backend/monitoring/views.py`中的事件上报、查询、复核和围栏接口
- `backend/monitoring/models.py`中的InspectionEvent、MediaAsset和Zone
- `bot-version/src/`中由北京知川科技有限公司自研且权属明确的识别编排、事件构造和上报代码
- 机器人端`roamerx_edge/alert_bridge.py`
- 机器人端`roamerx_edge/media_client.py`

## ZC-03 地图路线软件

优先候选：

- `frontend/src/views/MapsPage.vue`
- `frontend/src/views/RoutePlannerPage.vue`
- `backend/monitoring/views.py`中的MapData、建图和导航栈接口
- `backend/monitoring/models.py`中的MapData、MapSet、PatrolRoute和地图关联模型
- 机器人端`roamerx_edge/mapping_adapter.py`
- 机器人端`roamerx_edge/map_set_coordinator.py`
- 机器人端`roamerx_edge/map_activation_adapter.py`
- 机器人端`roamerx_edge/map_cleaner.py`和`manual_map_cleanup.py`

## ZC-04 任务调度软件

优先候选：

- `frontend/src/views/TasksPage.vue`
- `frontend/src/views/PatrolCalendarPage.vue`
- `frontend/src/views/TaskExecutionPage.vue`中的任务状态与控制部分
- `backend/monitoring/services/task_service.py`
- `backend/monitoring/services/command_service.py`
- `backend/monitoring/models.py`中的PatrolTask、TaskExecution、TaskExecutionEvent和RemoteCommand
- `backend/monitoring/views.py`中的任务创建、执行和暂停继续终止接口
- 机器人端`roamerx_edge/task_executor.py`
- 机器人端`roamerx_edge/command_processor.py`
- 机器人端`roamerx_edge/navigation_stack_adapter.py`

## ZC-05 远程指挥软件

优先候选：

- `frontend/src/views/RemoteControlPage.vue`
- `frontend/src/views/DashboardOverview.vue`中的接管、运动和喊话部分
- `backend/monitoring/views.py`中的RobotCommand、录音、TTS和播放接口
- `backend/monitoring/services/tts_service.py`
- `backend/monitoring/services/asr_service.py`仅在该功能进入申报版本且权属明确时使用
- `backend/monitoring/models.py`中的RobotCommand、SpeechTemplate和RecordedAudio
- 机器人端`roamerx_edge/teleop_control_adapter.py`
- 机器人端命令接收与媒体播放的自研适配代码

## ZC-06 巡检回溯软件

优先候选：

- `frontend/src/views/TrackPlaybackPage.vue`
- `frontend/src/views/TaskExecutionPage.vue`中的地图轨迹和时间线部分
- `backend/monitoring/services/alert_service.py`中的时间线查询部分
- `backend/monitoring/message_handlers.py`中的轨迹批次处理部分
- `backend/monitoring/models.py`中的Track、TrajectoryPoint和TrajectoryBatchReceipt
- `backend/monitoring/views.py`中的轨迹查询与执行详情接口
- 机器人端`roamerx_edge/trajectory_buffer.py`
- 机器人端`roamerx_edge/local_store.py`中的轨迹序号、outbox和确认删除部分

## 默认排除

- `node_modules/`、虚拟环境、`dist/`、缓存、日志和数据库文件。
- Django迁移文件，除非其中包含不可替代的自研数据处理逻辑。
- ROS2、Nav2、SLAM、定位库、厂商SDK及其他第三方源码。
- 模型权重、训练数据、开源检测器主体和自动生成代码。
- 测试夹具、演示固定数据、密钥、证书、密码和生产地址。
- 仅改变格式、注释或名称而没有独立功能表达的重复代码。
