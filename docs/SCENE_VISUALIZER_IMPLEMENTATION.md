# 场景视角调试页实施说明

## 目标

- 在服务器和容器生产构建的“巡检任务”下提供独立、只读的“场景视角调试”页。
- 将世界/场景地图、二维占据图、三维伪彩点云、局部点云、代价地图障碍物、路线航点、导航边界、定位轨迹和融合校正统一显示。
- 同时支持机器狗实时 Foxglove WebSocket 与浏览器本地 MCAP。MCAP 不上传服务器。
- 以人的俯视、跟随和机器狗第一视角观察场景；按钮可固定 2D/3D，滚轮或双指缩放跨过迟滞阈值时自动切换。
- YOLO 二维框只有经过相机/LiDAR 标定、100 ms 同步门、点云前景聚类、20 m 距离门、0.5 m 位置标准差门和 cloud-time TF 后，才发布为地图三维对象。

## 接口与页面

### 页面入口

- 模块 ID：`scene-visualizer`
- 地址：`/dashboard/tasks/scene-visualizer`
- 菜单：`巡检任务 → 场景视角调试`
- 页面不会调用机器人命令、边界保存或边界发布接口。

### 平台接口

- `GET /api/maps/{id}/scene/`：鉴权的只读场景清单，包含坐标模式、地图边界、点云能力、静态资产及当前导航边界版本。
- `GET /api/maps/{id}/scene-cloud/`：鉴权的 PCD 预览；服务器按确定性步长限制为 60 万点，按地图包 SHA-256 缓存，支持 ETag 和单段 Range。
- 现有机器人状态、导航状态、YOLO 框与系统日志接口继续复用。

### ROS / MCAP 主题

- `/tf`、`/tf_static`：建立传感器坐标系到 `map` 的变换链；链不完整时拒绝绘制对应数据层。
- `/front_lidar`：实时局部点云，经 cloud-frame → `map` 变换后显示，页面限 8 万点、最高 5 Hz 更新。
- `/local_costmap/costmap_raw`：局部障碍栅格，页面限 3 万个占用点。
- `/odom/localization_odom`：机器狗位姿与最近 500 个轨迹点。
- `/status`、`/localization/decision`、`/localization_info`：NDT、IMU、RTK、校正、初始化与主动重定位证据。
- `/perception/semantic_objects`：`robots_dog_msgs/SemanticObjectArray` 三维语义对象。
- `/perception/projection_status`：投影成功数、同步差、支持点数、标定 ID 或拒绝原因。

### YOLO 到三维资产

`roamerx-bike-bot` 每次推理把 `captured_at_unix`、`source_frame_id`、图像尺寸、track ID、类别、置信度和框原子写入 `/run/roamerx/person_detections.json`。独立命令 `roamerx-semantic-projection` 读取它，与 `/front_lidar` 和 TF 对齐后发布三维对象。

部署前必须把真实标定写入 `/home/dogrobot/runtime/nx-edge/conf/semantic_projection.yaml`，格式参考 `edge-agent/semantic_projection.example.yaml`。不得使用示例单位矩阵上车验收。

安装 Edge 包并加载机器人 ROS 环境后，用下面的只读节点接线：

```bash
roamerx-semantic-projection --ros-args \
  -p calibration_file:=/home/dogrobot/runtime/nx-edge/conf/semantic_projection.yaml \
  -p detection_path:=/run/roamerx/person_detections.json \
  -p cloud_topic:=/front_lidar \
  -p map_frame:=map
```

## 性能与用时标准

以下是开发原型的验收预算，不把网络延迟算成渲染耗时：

| 项目 | 标准 |
| --- | --- |
| 页面可交互 | 热缓存 ≤ 2 s；冷缓存地图点云 ≤ 8 s |
| 首次点云预览生成 | 600 万点以内 ≤ 15 s，且内存不读取整个 ZIP/PCD |
| 2D/3D 语义切换 | ≤ 300 ms，不重新请求地图 |
| 实时点云端到端显示 | P95 ≤ 500 ms |
| 定位/融合状态显示 | P95 ≤ 1 s |
| YOLO 三维资产显示 | P95 ≤ 700 ms，图像—点云同步差必须 ≤ 100 ms |
| 桌面渲染 | 目标 ≥ 30 FPS；全局 60 万点、局部 8 万点、障碍 3 万点上限 |
| 动态对象清理 | 最后一次观测后 1.5 s |
| MCAP 跳转后画面更新 | ≤ 300 ms（文件初次索引完成后） |

测试时需记录机器型号、浏览器、地图点数、MCAP 大小、网络 RTT 和 P50/P95；任一门限失败都不能只用平均值判定通过。

## 验收标准

1. 服务器和容器构建都能看到菜单和页面；其他仿真与回放临时页仍不进入容器模块清单。
2. 页面三种视角、2D/3D 按钮及缩放迟滞切换可用；切换不丢路线、边界、对象或当前时间轴。
3. 地图包含 `scene_preview.pcd` 时优先使用它，否则采样 `map.pcd`；路径穿越、损坏 ZIP、截断 PCD 返回明确 422。
4. 实时与 MCAP 使用相同主题和解码逻辑；本地文件不上传。
5. IMU、RTK、NDT、当前融合源、漂移门、平滑进度、重定位阶段、时间差和拒绝原因可核对。
6. 航点显示名称、坐标、Yaw、定位模式、全局/局部控制器、避障与到点朝向；页面不允许修改或下发。
7. 边界显示草稿/生效版本；编辑和发布只在路径规划页进行。
8. 无标定、TF 缺失、时间基不一致、同步超 100 ms、框内少于 8 点、距离超 20 m或位置标准差超 0.5 m时不生成三维资产，并发布拒绝码。
9. `INFO` 记录阶段，`DEBUG` 记录时间戳/点数等变量，`WARNING` 记录单帧拒绝，`ERROR` 记录标定或运行级错误；页面系统日志可按级别定位。

## 执行结果

- 已实现页面、菜单、Three.js 原生渲染、PCD/MCAP/Foxglove 数据源、诊断面板和系统日志联动。
- 已实现地图场景清单、PCD 流式抽样缓存、Range/ETag、静态资产与生效边界读取。
- 已实现 PointCloud2/OccupancyGrid 解码、伪彩、轨迹、校正向量、障碍物和基础资产拼接。
- 已实现语义消息定义、纯算法投影/拒绝门/跟踪滤波、YOLO 原子快照以及可独立运行的 ROS2 发布器。
- 自动化结果：前端单元测试 98 项通过；地图场景 API 4 项通过；语义投影算法及适配器 5 项通过；Edge 全套 380 项和检测端 12 项通过；Django system check、服务器构建与容器构建通过。
- 尚需现场完成相机—LiDAR 标定并采集真实 MCAP，按上表测量 P50/P95；这是进入机器狗实测前的硬性条件。
