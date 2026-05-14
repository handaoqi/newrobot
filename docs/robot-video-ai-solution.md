# 机器狗视频上云与自行车视觉识别整体方案

## 1. 项目背景

当前项目需要将机器狗摄像头视频流呈现到远程云服务器的网站上，同时在机器狗端进行视觉识别，检测自行车或自行车违停事件，并将识别结果回传服务器。

项目约束如下：

- 机器狗可以主动联网。
- 机器狗没有公网 IP。
- 云服务器有公网 IP。
- 浏览器用户通过云服务器网站查看实时视频、机器人状态和识别事件。
- 本地文档 `docs/robot-payload-standard.md` 已定义了机器人遥测与识别事件上报数据结构，但其中“图片/视频片段通过对象存储上传后回传 URL”的传输方式不完全符合实际部署条件，需要调整。

因此，本方案采用“机器狗主动推流 + 机器狗主动上报识别结果”的架构。

## 2. 总体架构

```text
机器狗端
  摄像头采集
  视频编码
  AI 自行车检测
  抓拍/短视频切片
  状态心跳/检测事件上报
      |
      | 主动连接公网云服务器
      |
公网云服务器
  ZLMediaKit / PyMKUI 视频服务
  Django 业务后端
  文件存储 / 对象存储
  数据库
      |
      |
Web 前端
  实时视频播放
  检测框/事件列表
  机器人状态
  事件处置
  轨迹与告警统计
```

整体上拆成两条链路：

```text
视频链路：机器狗 -> ZLMediaKit -> 浏览器播放
数据链路：机器狗 -> Django API -> 数据库/前端事件展示
```

两条链路通过 `robot_code`、`camera_id`、`stream_id`、`event_time`、`sequence_id` 等字段进行关联。

设计原则：

- 云端不要主动拉取机器狗视频，因为机器狗没有公网 IP。
- 机器狗必须主动推流到云服务器。
- 机器狗必须主动上报识别结果、心跳状态和抓拍文件。
- 视频流和识别 JSON 不建议混成一条主通道，避免耦合过重。
- 视觉识别优先放在机器狗端执行，减少云端算力和网络压力。

## 3. 云端视频服务方案

推荐基于 PyMKUI 与 ZLMediaKit 建设云端视频服务。

职责划分：

- ZLMediaKit：负责流媒体接入、转协议、分发、录像、流状态回调。
- PyMKUI：负责 ZLMediaKit 的 Web 管理、流查看、基础运维能力。
- 业务后端：负责机器人、用户、权限、播放地址分发、事件和遥测数据。
- 业务前端：负责最终操作界面，不建议直接把 PyMKUI 当作业务系统主界面。

### 3.1 视频推流方向

由于机器狗没有公网 IP，必须采用主动推流：

```text
机器狗摄像头 -> 视频编码 -> 主动推流到云服务器 -> ZLMediaKit -> 浏览器播放
```

第一阶段建议使用 RTMP 推流，便于快速验证：

```bash
ffmpeg -f v4l2 -i /dev/video0 \
  -vcodec libx264 -preset veryfast -tune zerolatency \
  -f flv rtmp://<server-ip>/live/dog_ZSL-1A-07_front
```

正式低延迟场景可以逐步升级为：

- RTMP 推流 + WebRTC 播放
- SRT 推流 + WebRTC 播放
- WHIP/WebRTC 推流 + WHEP/WebRTC 播放

### 3.2 播放协议选择

| 方案 | 优点 | 缺点 | 适用场景 |
| --- | --- | --- | --- |
| HLS | 浏览器兼容性好，稳定 | 延迟高 | 普通监控、回看 |
| HTTP-FLV | 延迟较低，实现简单 | 需要 JS 播放器 | MVP、实时监测 |
| WebRTC/WHEP | 延迟最低 | 部署和网络配置更复杂 | 远程驾驶、强实时巡检 |

推荐路线：

```text
MVP：RTMP 推流 + HTTP-FLV 播放
正式版：SRT/WHIP 推流 + WebRTC/WHEP 播放
兼容兜底：HLS 播放
```

### 3.3 流命名规范

建议统一使用：

```text
app: live
stream: dog_{robot_code}_{camera_id}
```

示例：

```text
/live/dog_ZSL-1A-07_front
/live/dog_ZSL-1A-07_rear
```

业务后端可以返回如下播放地址：

```json
{
  "robot_code": "ZSL-1A-07",
  "camera_id": "front",
  "stream_id": "dog_ZSL-1A-07_front",
  "play_urls": {
    "webrtc": "https://server/index/api/webrtc?app=live&stream=dog_ZSL-1A-07_front&type=play",
    "flv": "https://server/live/dog_ZSL-1A-07_front.live.flv",
    "hls": "https://server/live/dog_ZSL-1A-07_front/hls.m3u8"
  }
}
```

## 4. 机器狗端视觉识别方案

视觉识别建议在机器狗端完成，而不是把所有视频传到服务器后再统一检测。

原因：

- 减少服务器 GPU/CPU 压力。
- 弱网时仍可本地识别。
- 事件数据量远小于视频数据量。
- 机器狗可以更快触发本地声光提醒或行为决策。

### 4.1 板端处理流水线

```text
摄像头采集
  |
  | 同一路视频帧
  |
  +--> 编码推流到 ZLMediaKit
  |
  +--> AI 推理：自行车检测
          |
          +--> 输出 bbox、confidence、label
          +--> 判断是否违规/是否进入告警区域
          +--> 抓拍 JPEG
          +--> 调用云端上传接口
          +--> 调用 telemetry/ingest 上报事件 JSON
```

### 4.2 模型选择

可以根据机器狗板端硬件选择模型部署方式：

| 硬件/平台 | 推荐方案 |
| --- | --- |
| NVIDIA Jetson | YOLO + TensorRT |
| Intel 边缘设备 | OpenVINO |
| 瑞芯微 RK 平台 | RKNN |
| 普通 Linux/ARM | ONNX Runtime 或 ncnn |
| 前期验证 | YOLOv8/YOLOv5 + Python |

建议检测类别：

- `bicycle`
- `electric_bicycle`
- `motorcycle`
- `person`，可选，用于判断是否有人骑行或辅助过滤误报。

### 4.3 自行车违停规则

如果业务目标是“自行车违停”，不能只依赖模型检测到自行车。建议增加规则层：

```text
检测到 bicycle/electric_bicycle
  +
bbox 中心点位于禁停区域 ROI 内
  +
连续 N 帧或持续 M 秒存在
  +
置信度超过阈值
  =
生成 vehicle_illegal_parking 事件
```

建议初始阈值：

- `confidence >= 0.65`
- 连续 10 帧内出现不少于 6 次
- 同一区域同一目标 60 秒内只上报一次
- bbox 面积过小的目标忽略
- 夜间或逆光时可降低置信度阈值，但增加连续帧要求

禁停区域建议配置为多边形 ROI：

```json
{
  "camera_id": "front",
  "forbidden_parking_roi": [
    [120, 280],
    [1080, 260],
    [1240, 700],
    [80, 710]
  ]
}
```

## 5. 识别结果与遥测上报

本地文档 `docs/robot-payload-standard.md` 中定义的主结构可以继续沿用：

- `sequence_id`
- `robot_code`
- `robot_name`
- `reported_at`
- `position`
- `motion`
- `power`
- `network`
- `runtime`
- `detections`

现有 Django 后端已经有 `POST /api/telemetry/ingest/` 接口，当前逻辑会：

- 根据 `robot_code` 创建或更新机器人。
- 更新机器人位置、电量、网络强度、运行模式和状态。
- 写入 `RobotTelemetry`。
- 将 `detections` 转换成 `InspectionEvent`。

### 5.1 推荐上报频率

| 数据类型 | 频率 |
| --- | --- |
| 心跳与状态 | 每 5 秒 |
| 位置与运动信息 | 每 1-2 秒，或与心跳合并 |
| 检测事件 | 触发后立即上报 |
| 抓拍图片 | 事件触发后上传 |
| 短视频片段 | 可选，事件前后 5-10 秒 |

### 5.2 建议扩展字段

为了让视频和识别事件能准确关联，建议在原文档基础上增加：

- `video.stream_id`
- `video.camera_id`
- `video.frame_width`
- `video.frame_height`
- `video.frame_timestamp`
- `detections[].object_class`
- `detections[].track_id`
- `detections[].bbox`

示例：

```json
{
  "sequence_id": "ZSL-1A-07-20260514-223501-0001",
  "robot_code": "ZSL-1A-07",
  "robot_name": "南入口巡检机器人",
  "reported_at": "2026-05-14T22:35:01+08:00",
  "position": {
    "name": "太阳宫公园南入口",
    "latitude": 39.983521,
    "longitude": 116.447153
  },
  "motion": {
    "speed": 0.8,
    "heading": 83.5
  },
  "power": {
    "battery_level": 78,
    "charging": false
  },
  "network": {
    "signal_strength": 92,
    "network_type": "5G"
  },
  "runtime": {
    "mode": "auto",
    "status": "warning"
  },
  "video": {
    "stream_id": "dog_ZSL-1A-07_front",
    "camera_id": "front",
    "frame_width": 1280,
    "frame_height": 720,
    "frame_timestamp": "2026-05-14T22:35:00.820+08:00"
  },
  "detections": [
    {
      "type": "vehicle_illegal_parking",
      "label": "自行车违停",
      "object_class": "bicycle",
      "confidence": 0.925,
      "risk_level": "medium",
      "track_id": "trk-1842",
      "bbox": {
        "x": 124,
        "y": 88,
        "width": 162,
        "height": 236
      },
      "snapshot_url": "https://server/media/snapshots/ZSL-1A-07/20260514/event-0001.jpg",
      "event_time": "2026-05-14T22:35:00.820+08:00"
    }
  ]
}
```

## 6. 抓拍图片与短视频上传

原文档中写的是“图片/视频片段通过对象存储上传后回传 URL”。该方式对机器狗端不够友好，原因是：

- 机器狗端不应该持有对象存储长期密钥。
- 弱网环境下直传对象存储失败后不好统一补偿。
- 正式环境需要服务端统一鉴权、审计和文件命名。

建议改为：

```text
机器狗 -> Django 文件上传接口 -> 云端保存本地文件或转存对象存储 -> 返回 snapshot_url
```

推荐接口：

```http
POST /api/device/media/upload/
Content-Type: multipart/form-data
X-Device-Code: ZSL-1A-07
X-Timestamp: 2026-05-14T22:35:01+08:00
X-Nonce: random-string
X-Signature: hmac-signature
```

表单字段：

| 字段 | 说明 |
| --- | --- |
| `robot_code` | 机器人编号 |
| `camera_id` | 摄像头编号 |
| `media_type` | `snapshot` 或 `clip` |
| `event_time` | 事件发生时间 |
| `sequence_id` | 上报流水号 |
| `sha256` | 文件校验值 |
| `file` | 图片或视频文件 |

返回：

```json
{
  "url": "https://server/media/snapshots/ZSL-1A-07/20260514/event-0001.jpg"
}
```

弱网场景下，机器狗端应支持本地缓存：

```text
事件先落本地 SQLite/文件队列
图片先存在本地磁盘
上传失败定时重试
上报成功后标记 ack
超过保留期再清理
```

不要将图片 base64 直接塞进遥测 JSON。

## 7. 前端展示方案

前端建议由你们自己的 Vue 平台承载，不直接使用 PyMKUI 作为业务页面。

### 7.1 第一阶段展示

先实现：

- 机器人状态卡片。
- 实时视频窗口。
- 最新识别事件。
- 抓拍图片。
- 事件列表与处理状态。

这种方式实现简单，适合演示和 MVP。

### 7.2 第二阶段实时叠框

如果需要在实时视频上显示检测框，可以增加实时检测结果通道：

```text
机器狗检测结果 -> Django WebSocket/SSE -> 前端 Canvas/SVG overlay
视频流 -> ZLMediaKit -> 前端播放器
```

前端根据 `frame_width`、`frame_height` 与播放器实际尺寸计算缩放比例：

```text
display_x = bbox.x * player_width / frame_width
display_y = bbox.y * player_height / frame_height
display_width = bbox.width * player_width / frame_width
display_height = bbox.height * player_height / frame_height
```

注意事项：

- 机器狗和服务器必须 NTP 对时。
- bbox 坐标必须明确对应原始推理帧分辨率。
- 如果视频经过裁剪、旋转或缩放，必须同步上报变换关系。
- 实时叠框可以只保留最近 1-3 秒结果，避免旧框残留。

## 8. 云端服务模块划分

建议云服务器包含以下模块：

```text
Nginx
  负责 HTTPS、反向代理、静态文件、媒体文件访问

ZLMediaKit + PyMKUI
  负责推流接入、转协议、流状态、播放、录像

Django 后端
  负责机器人管理、遥测接收、事件入库、文件上传、鉴权、业务 API

数据库
  MVP 可使用 SQLite
  正式环境建议 PostgreSQL/MySQL

Redis
  可选，用于设备在线状态、事件推送、任务队列

文件存储/对象存储
  保存抓拍图、短视频片段、录像索引
```

## 9. 数据模型建议

当前项目已有：

- `Robot`
- `InspectionEvent`
- `RobotTelemetry`
- `PatrolTask`

建议后续增加：

| 模型 | 说明 |
| --- | --- |
| `RobotDeviceCredential` | 设备密钥、启停状态、最后认证时间 |
| `RobotCamera` | 摄像头编号、stream_id、播放地址、在线状态 |
| `MediaAsset` | 抓拍图、短视频、文件大小、sha256 |
| `DetectionRecord` | 原始检测结果，包含 bbox、track_id、模型版本 |
| `StreamSession` | 推流上线/下线记录 |

`InspectionEvent` 建议补充：

- `camera_id`
- `stream_id`
- `bbox_x`
- `bbox_y`
- `bbox_width`
- `bbox_height`
- `frame_width`
- `frame_height`
- `track_id`
- `model_name`
- `model_version`
- `raw_detection`

这样可以支持后续事件复核、视频叠框和模型效果追踪。

## 10. 鉴权与安全

机器狗端不应使用普通用户登录 token，应使用设备级凭证。

推荐请求头：

```http
X-Device-Code: ZSL-1A-07
X-Timestamp: 2026-05-14T22:35:01+08:00
X-Nonce: random-string
X-Signature: hmac-signature
```

签名内容建议：

```text
HMAC-SHA256(
  device_secret,
  method + "\n" +
  path + "\n" +
  body_sha256 + "\n" +
  timestamp + "\n" +
  nonce
)
```

服务端校验：

- 设备是否存在。
- 设备是否启用。
- 时间戳是否在允许窗口内。
- `nonce` 是否重复。
- 签名是否正确。
- `sequence_id` 是否已处理。

推流地址也应鉴权：

```text
rtmp://server/live/dog_ZSL-1A-07_front?token=short-lived-token
```

云端可以通过 ZLMediaKit hook 在 `on_publish` 时校验 token，防止伪造推流。

## 11. 弱网与异常处理

机器狗是移动端设备，必须假设网络会不稳定。

建议策略：

- 视频推流断开后自动重连。
- 重连采用指数退避，避免频繁打满网络。
- 检测事件上报失败时写入本地队列。
- 抓拍图上传失败时保留本地文件并重试。
- 可以先上报事件，再补传 `snapshot_url`。
- `sequence_id` 必须支持幂等，避免重复入库。
- 服务器不可用时机器狗端保留最近 N 小时事件。
- 网络变差时自动降低码率、分辨率或帧率。

建议视频参数：

| 场景 | 参数 |
| --- | --- |
| 普通 4G/5G 演示 | 720p，15fps，1-2 Mbps |
| 稳定 5G/WiFi | 1080p，25fps，3-5 Mbps |
| 低延迟 | GOP 1-2 秒，开启 zerolatency |
| 弱网兜底 | 480p，10-15fps，500 Kbps-1 Mbps |

## 12. 基于 PyMKUI 的二次开发建议

不建议把 PyMKUI 深改成完整业务系统主体。更稳妥的方式是：

```text
PyMKUI/ZLMediaKit：视频服务基座
你们 Django/Vue：业务平台主体
两者通过 API、hook 和 stream_id 集成
```

你们自己的平台负责：

- 机器人档案。
- 设备鉴权。
- 事件管理。
- 自行车识别结果。
- 抓拍图和短视频。
- 巡检任务。
- 用户权限。
- 事件处置流程。

PyMKUI/ZLMediaKit 负责：

- 接收机器狗推流。
- 转换播放协议。
- 生成播放地址。
- 维护流状态。
- 录像。
- 流媒体服务运维。

业务前端不需要直接嵌入 PyMKUI 页面。推荐由 Django 后端根据机器人编号返回播放 URL，Vue 前端使用播放器组件展示。

## 13. 推荐落地阶段

### 第一阶段：跑通闭环

- 机器狗或模拟端 RTMP 推流到 ZLMediaKit。
- Django 接收 `POST /api/telemetry/ingest/`。
- 检测事件进入 `InspectionEvent`。
- 前端展示实时视频、事件列表和抓拍图。

### 第二阶段：补齐生产基础能力

- 增加设备密钥鉴权。
- 增加文件上传接口。
- 增加 `sequence_id` 幂等。
- 接入 ZLMediaKit 流状态 hook。
- 完善机器人在线/离线判断。
- 完善事件处置流程。

### 第三阶段：优化实时体验

- WebRTC/WHEP 低延迟播放。
- 视频画面实时叠加 bbox。
- 事件时间轴联动视频。
- 录像回放。
- 按机器人、区域、风险等级筛选。

### 第四阶段：增强可靠性和可运维性

- 弱网缓存补传。
- 多摄像头管理。
- 远程配置检测阈值。
- 模型版本管理。
- 推流质量监控。
- 告警通知。
- 日志审计。

## 14. 最终方案总结

机器狗端主动将摄像头视频推送到公网云服务器上的 ZLMediaKit，由 PyMKUI/ZLMediaKit 负责视频接入、转协议、播放和录像；机器狗端同时在本地进行自行车视觉识别，将检测结果、bbox、抓拍图 URL、机器人状态和位置信息通过 Django API 主动上报。

云端使用你们现有的 Django/Vue 平台承载业务能力，包括机器人管理、事件中心、抓拍图、遥测数据、处置流程和权限控制。PyMKUI 不作为业务主体，而作为视频服务管理底座。

该方案符合“机器狗无公网 IP、服务器有公网 IP”的现实网络条件，也能从演示版逐步演进到低延迟、可鉴权、可追溯、可运维的正式系统。
