# 前后端接口与数据传输格式

[返回文档中心](./project-docs-index.md)

## 1. 接口总览

后端根路径：

```text
http://127.0.0.1:8000/api/
```

前端 API 封装文件：

```text
frontend/src/services/api.js
```

默认请求头：

```http
Content-Type: application/json
Authorization: Token <inspection_token>
```

登录后 Token 保存在：

```text
localStorage.inspection_token
localStorage.inspection_user
```

## 2. 鉴权规则

默认接口要求 DRF Token 鉴权。

允许匿名访问的接口：

| 接口 | 原因 |
| --- | --- |
| `GET /api/health/` | 健康检查 |
| `POST /api/auth/login/` | 获取 Token |
| `POST /api/telemetry/ingest/` | 板端遥测上报 |
| `POST /api/device/media/upload/` | 板端抓拍上传 |

## 3. API 列表

| 方法 | 路径 | 说明 | 前端调用 |
| --- | --- | --- | --- |
| `GET` | `/api/health/` | 健康检查 | 无 |
| `POST` | `/api/auth/login/` | 登录，返回 Token | `login()` |
| `POST` | `/api/auth/logout/` | 删除当前 Token | 当前前端只做本地退出 |
| `GET` | `/api/auth/profile/` | 当前用户信息 | 预留 |
| `GET` | `/api/dashboard/overview/` | 监测中心汇总 | `fetchOverview()` |
| `GET` | `/api/dashboard/analytics/` | 统计分析数据 | `fetchAnalytics()` |
| `GET` | `/api/robots/` | 机器人列表 | `fetchRobots()` |
| `GET` | `/api/robots/<robot_id>/` | 机器人详情 | `fetchRobotDetail()` |
| `GET` | `/api/events/` | 事件列表，支持筛选/搜索/排序/分页 | `fetchEvents()` |
| `GET` | `/api/events/<event_id>/` | 事件详情 | 预留 |
| `POST` | `/api/events/<event_id>/handle/` | 事件处置/归档 | `handleEvent()` |
| `GET` | `/api/tasks/` | 巡检任务列表 | `fetchTasks()` |
| `POST` | `/api/telemetry/ingest/` | 板端遥测与检测事件上报 | `bot-version` |
| `POST` | `/api/device/media/upload/` | 板端抓拍/视频片段上传 | `bot-version` |

## 4. 登录接口

请求：

```http
POST /api/auth/login/
Content-Type: application/json
```

```json
{
  "username": "operator",
  "password": "admin123456"
}
```

成功响应：

```json
{
  "token": "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
  "user": {
    "username": "operator",
    "display_name": "值班员A"
  }
}
```

失败响应：

```json
{
  "detail": "用户名或密码错误"
}
```

说明：登录时后端会调用 `ensure_demo_seed()`，自动创建演示用户和基础机器人/任务数据。

## 5. 监测中心接口

请求：

```http
GET /api/dashboard/overview/
Authorization: Token <token>
```

响应结构：

```json
{
  "summary": {
    "online_robot_count": 1,
    "today_alert_count": 12,
    "pending_event_count": 3,
    "resolved_event_count": 5,
    "completed_task_count": 0
  },
  "header": {
    "device_code": "ZSL-1A-07",
    "current_mode": "AI自主巡检",
    "current_location": "太阳宫公园南入口",
    "today_alerts": 12
  },
  "live_event": {},
  "latest_robot": {},
  "event_distribution": [
    { "status": "pending", "total": 3 }
  ]
}
```

`latest_robot` 使用机器人详情结构，包含近期事件和任务。

## 6. 统计分析接口

请求：

```http
GET /api/dashboard/analytics/
```

响应结构：

```json
{
  "updated_at": "2026-05-30T10:00:00+08:00",
  "cards": [
    {
      "title": "累计预警",
      "value": "13 次",
      "note": "近 7 个统计周期内的异常提醒总量"
    }
  ],
  "trends": [
    {
      "title": "预警次数趋势",
      "subtitle": "观察异常波动，辅助值班优先级调整",
      "unit": "次",
      "accent": "#fb7b4d",
      "series": [
        { "label": "05-24", "value": 1 }
      ],
      "summary": {
        "latest": 4,
        "delta": 3,
        "direction": "较上一周期上升",
        "total": 13,
        "average": 1.9
      }
    }
  ],
  "risk_weights": [
    { "label": "05-24", "value": 0 }
  ]
}
```

前端 `TrendLineChart.vue` 使用 `trends[].series` 绘制 ECharts 折线图。

## 7. 机器人接口

### 7.1 机器人列表

请求：

```http
GET /api/robots/
```

响应项字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `id` | number | 机器人数据库 ID |
| `code` | string | 机器人唯一编号 |
| `name` | string | 显示名称 |
| `location` | string | 当前位置 |
| `area` | string | 当前区域 |
| `status` | string | `online`、`offline`、`warning`、`charging` |
| `status_label` | string | 状态中文名 |
| `mode` | string | `auto`、`manual`、`standby`、`returning` |
| `mode_label` | string | 模式中文名 |
| `battery_level` | number | 电量百分比 |
| `network_strength` | number | 网络强度 |
| `speaker_volume` | number | 扬声器音量 |
| `today_alerts` | number | 今日告警数 |
| `current_task_name` | string | 当前任务名称 |
| `last_heartbeat_at` | datetime | 最近心跳时间 |
| `camera_id` | string | 摄像头 ID |
| `stream_id` | string | 视频流 ID |
| `play_urls` | object | 播放地址，通常包含 `flv`、`hls` |

### 7.2 机器人详情

请求：

```http
GET /api/robots/<robot_id>/
```

在列表字段基础上增加：

| 字段 | 说明 |
| --- | --- |
| `patrol_duration_minutes` | 今日巡检时长 |
| `firmware_version` | 固件版本 |
| `recent_events` | 最近 5 条事件 |
| `tasks` | 最近 5 条任务 |

## 8. 事件接口

### 8.1 事件列表

请求：

```http
GET /api/events/?status=pending&page=1&page_size=8&search=南入口&ordering=detected_desc
```

查询参数：

| 参数 | 说明 |
| --- | --- |
| `status` | 可选，`pending` 或 `resolved` |
| `search` | 可选，搜索标题、地点、事件类型、描述、处置备注、机器人名称、机器人编号 |
| `ordering` | `detected_desc`、`detected_asc`、`risk_desc`、`confidence_desc` |
| `page` | 页码，最小为 1 |
| `page_size` | 每页数量，范围 1 到 50 |

响应结构：

```json
{
  "count": 23,
  "page": 1,
  "page_size": 8,
  "has_next": true,
  "results": [
    {
      "id": 1,
      "title": "自行车违停",
      "event_type": "vehicle_illegal_parking",
      "location": "太阳宫公园南入口",
      "detected_at": "2026-05-30T10:00:00+08:00",
      "confidence": "92.50",
      "risk_level": "medium",
      "risk_label": "中",
      "status": "pending",
      "status_label": "待处理",
      "review_result": "confirmed",
      "review_result_label": "确认违规",
      "snapshot_url": "http://127.0.0.1:8000/media/device-media/...",
      "annotated_snapshot_url": "http://127.0.0.1:8000/media/annotated-events/event-1.jpg",
      "description": "板端识别上报: 自行车违停",
      "handling_notes": "",
      "robot_code": "ZSL-1A-07",
      "robot_name": "南入口巡检机器人",
      "camera_id": "front",
      "stream_id": "dog_ZSL-1A-07_front",
      "object_class": "bicycle",
      "track_id": "track-1",
      "bbox_x": 124,
      "bbox_y": 88,
      "bbox_width": 162,
      "bbox_height": 236,
      "frame_width": 1280,
      "frame_height": 720,
      "raw_detection": {}
    }
  ]
}
```

### 8.2 事件处置

请求：

```http
POST /api/events/<event_id>/handle/
Content-Type: application/json
```

```json
{
  "status": "resolved",
  "handling_notes": "值班员已通过平台完成复核与处置。",
  "review_result": "confirmed"
}
```

字段约束：

| 字段 | 可选值 |
| --- | --- |
| `status` | `pending`、`resolved` |
| `review_result` | `confirmed`、`suspected`、`false_alarm` |

成功响应：返回更新后的事件对象。

## 9. 巡检任务接口

请求：

```http
GET /api/tasks/
```

响应项字段：

| 字段 | 说明 |
| --- | --- |
| `id` | 任务 ID |
| `name` | 任务名称 |
| `robot_name` | 机器人名称 |
| `route_name` | 路线名称 |
| `scheduled_start` | 计划开始时间 |
| `scheduled_end` | 计划结束时间 |
| `status` | `pending`、`running`、`completed`、`paused` |
| `completion_rate` | 完成度百分比 |

## 10. 板端遥测上报

请求：

```http
POST /api/telemetry/ingest/
Content-Type: application/json
X-Device-Code: ZSL-1A-07
X-Timestamp: 2026-05-30T10:00:00+08:00
X-Device-Key: <可选>
```

请求体：

```json
{
  "sequence_id": "ZSL-1A-07-20260530100000000000-000001-abcdef12",
  "robot_code": "ZSL-1A-07",
  "robot_name": "南入口巡检机器人",
  "reported_at": "2026-05-30T10:00:00+08:00",
  "position": {
    "name": "太阳宫公园南入口",
    "latitude": 39.983521,
    "longitude": 116.447153
  },
  "motion": {
    "speed": 1.26,
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
    "status": "online"
  },
  "video": {
    "stream_id": "dog_ZSL-1A-07_front",
    "camera_id": "front",
    "frame_width": 1280,
    "frame_height": 720,
    "frame_timestamp": "2026-05-30T10:00:00+08:00",
    "play_urls": {
      "flv": "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv",
      "hls": "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front/hls.m3u8"
    }
  },
  "detections": [
    {
      "type": "vehicle_illegal_parking",
      "label": "自行车违停",
      "confidence": 0.925,
      "risk_level": "medium",
      "object_class": "bicycle",
      "track_id": "1",
      "bbox": {
        "x": 124,
        "y": 88,
        "width": 162,
        "height": 236
      },
      "snapshot_url": "http://127.0.0.1:8000/media/device-media/2026/05/30/event.jpg",
      "event_time": "2026-05-30T10:00:00+08:00"
    }
  ]
}
```

成功响应：

```json
{
  "detail": "上报成功",
  "robot_id": 1,
  "telemetry_id": 10,
  "duplicate": false
}
```

重复 `sequence_id` 响应：

```json
{
  "detail": "重复上报已忽略",
  "robot_id": 1,
  "telemetry_id": 10,
  "duplicate": true
}
```

处理规则：

- `sequence_id` 全局唯一，用于幂等去重。
- 若机器人不存在，按 `robot_code` 自动创建。
- 每次上报会更新机器人位置、电量、网络、模式、状态、心跳、视频流信息。
- `detections` 中每个识别项会生成一条 `InspectionEvent`。
- 置信度若小于等于 `1`，后端会乘以 `100` 后保存；大于 `1` 时按百分比值保存。

## 11. 媒体上传

请求：

```http
POST /api/device/media/upload/
Content-Type: multipart/form-data
```

表单字段：

| 字段 | 类型 | 必填 | 说明 |
| --- | --- | --- | --- |
| `robot_code` | string | 是 | 机器人编号 |
| `camera_id` | string | 否 | 摄像头 ID |
| `media_type` | string | 是 | `snapshot` 或 `clip` |
| `event_time` | datetime | 否 | 事件时间 |
| `sequence_id` | string | 否 | 对应遥测序列号 |
| `sha256` | string | 否 | 文件 SHA-256，提供后后端会校验 |
| `file` | file | 是 | 上传文件 |

成功响应：

```json
{
  "url": "http://127.0.0.1:8000/media/device-media/2026/05/30/file.jpg",
  "asset": {
    "id": 1,
    "robot_code": "ZSL-1A-07",
    "media_type": "snapshot",
    "camera_id": "front",
    "sequence_id": "ZSL-1A-07-...",
    "event_time": "2026-05-30T10:00:00+08:00",
    "url": "http://127.0.0.1:8000/media/device-media/2026/05/30/file.jpg",
    "sha256": "....",
    "file_size": 123456,
    "created_at": "2026-05-30T10:00:01+08:00"
  }
}
```

SHA-256 校验失败：

```json
{
  "detail": "文件 sha256 校验失败"
}
```

## 12. 错误格式

业务错误通常返回：

```json
{
  "detail": "错误说明"
}
```

序列化校验错误会按 DRF 默认结构返回字段错误，例如：

```json
{
  "power": {
    "battery_level": [
      "Ensure this value is less than or equal to 100."
    ]
  }
}
```
