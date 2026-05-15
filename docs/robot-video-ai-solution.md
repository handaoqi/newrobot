# 机器狗视频上云与自行车视觉识别落地方案

## 1. 当前目标

本项目已经落地一个 MVP 闭环：

```text
机器狗 RTSP 视频源
  -> bot-version 通过 ffmpeg 主动推 RTMP 到 ZLMediaKit
  -> ZLMediaKit 转出 HTTP-FLV/HLS
  -> Vue 前端播放实时视频

bot-version 本地识别自行车/违停事件
  -> Django 媒体上传接口保存抓拍图
  -> Django telemetry 接口保存遥测与事件
  -> Vue 前端展示事件、抓拍图与机器人状态
```

这个方案符合现实约束：

- 机器狗可以主动联网。
- 机器狗通常没有公网 IP。
- 云服务器或本地测试机运行 ZLMediaKit、Django、Vue。
- 浏览器不能直接播放 RTSP，因此 RTSP 必须先转成 HTTP-FLV/HLS/WebRTC 等浏览器可播放协议。

## 2. 已实现架构

```text
机器狗 / 边缘端 bot-version
  RTSP 视频读取
  ffmpeg 推流线程
  YOLO 自行车识别
  抓拍 JPEG
  telemetry JSON 上报
      |
      | RTMP: rtmp://<server>:1935/live/dog_ZSL-1A-07_front
      | HTTP API: /api/device/media/upload/ 与 /api/telemetry/ingest/
      |
云端 / 本地服务器
  ZLMediaKit
  Django REST API
  SQLite / 后续可替换 PostgreSQL
  media 文件目录
      |
      | HTTP-FLV/HLS + REST API
      |
Vue 前端
  实时视频播放
  机器人状态
  事件列表
  抓拍图展示
  事件处置
```

两条链路通过以下字段关联：

- `robot_code`
- `camera_id`
- `stream_id`
- `sequence_id`
- `event_time`

当前默认机器人：

```text
robot_code: ZSL-1A-07
camera_id: front
stream_id: dog_ZSL-1A-07_front
```

## 3. ZLMediaKit 部署

项目已提供跨 Windows/Linux 的 Docker Compose：

```bash
docker compose -f docker-compose.zlmediakit.yml up -d
```

本地 Windows MVP 默认端口：

| 本机端口 | 容器端口 | 用途 |
| --- | --- | --- |
| `1935` | `1935` | RTMP 推流入口 |
| `8080` | `80` | HTTP-FLV/HLS 播放 |
| `8554` | `554` | RTSP 输出/兼容 |
| `10000` | `10000` | WebRTC 预留 |

本地播放地址：

```text
http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv
http://127.0.0.1:8080/live/dog_ZSL-1A-07_front/hls.m3u8
```

云服务器部署时只需要把 `127.0.0.1` 改成云服务器 IP 或域名，并开放对应端口。

## 4. bot-version 推流方案

端侧新增了：

- `bot-version/src/bike_bot/stream.py`
- `bot-version/run_stream.py`
- `bot-version/config.yaml` 中的 `stream` 配置段

核心配置：

```yaml
video:
  source: "rtsp://192.168.234.1:8554/test"
  width: 1280
  height: 720
  camera_id: "front"
  stream_id: "dog_ZSL-1A-07_front"
  play_urls:
    flv: "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv"
    hls: "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front/hls.m3u8"

stream:
  enable: true
  ffmpeg_path: "ffmpeg"
  rtmp_url: "rtmp://127.0.0.1/live/dog_ZSL-1A-07_front"
  reconnect_interval_seconds: 5
  video_codec: "copy"
  audio_enabled: false
  extra_args: []
```

生成的推流命令形态：

```bash
ffmpeg -hide_banner -loglevel warning \
  -rtsp_transport tcp \
  -i rtsp://192.168.234.1:8554/test \
  -an \
  -c:v copy \
  -f flv \
  rtmp://127.0.0.1/live/dog_ZSL-1A-07_front
```

单独测试视频推流：

```bash
python run_stream.py --config config.yaml
```

完整识别与推流同时运行：

```bash
python run_edge.py --config config.yaml
```

`run_stream.py` 已加入单实例锁，避免同一台机器重复启动同一个 `stream_id` 导致 ZLMediaKit 返回 `Already publishing`。

## 5. 前端播放方案

Vue 首页“实时视频流监控”已改为读取后端返回的：

```json
{
  "stream_id": "dog_ZSL-1A-07_front",
  "play_urls": {
    "flv": "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv",
    "hls": "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front/hls.m3u8"
  }
}
```

播放策略：

1. 优先使用 `mpegts.js` 播放 HTTP-FLV。
2. 如果 HTTP-FLV 不可用，使用 `hls.js` 播放 HLS。
3. 如果没有视频流地址，则回退显示最新事件抓拍图或静态演示图。

当前首页视频区域已经移除检测框、标签和事件时间轴，只保留实时视频画面、巡检位置和视频流编号。

## 6. Django 数据接口

### 6.1 媒体上传

端侧抓拍图不再伪造 `/static/snapshots/...` URL，而是先上传到 Django：

```http
POST /api/device/media/upload/
Content-Type: multipart/form-data
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
  "url": "http://127.0.0.1:8000/media/device-media/2026/05/15/event.jpg"
}
```

### 6.2 遥测与事件上报

```http
POST /api/telemetry/ingest/
Content-Type: application/json
```

当前后端已支持：

- `sequence_id` 幂等处理，重复上报不会重复入库。
- `video.stream_id`、`video.camera_id`、`video.frame_width`、`video.frame_height` 保存到遥测数据。
- detection 中的 `object_class`、`track_id`、`bbox`、`snapshot_url` 保存到事件数据。
- `Robot.play_urls` 保存前端播放地址。

示例：

```json
{
  "sequence_id": "ZSL-1A-07-20260515-001",
  "robot_code": "ZSL-1A-07",
  "robot_name": "南入口巡检机器人",
  "reported_at": "2026-05-15T20:12:00+08:00",
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
    "frame_height": 720
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
      "snapshot_url": "http://127.0.0.1:8000/media/device-media/2026/05/15/event.jpg",
      "event_time": "2026-05-15T20:12:00+08:00"
    }
  ]
}
```

## 7. 数据模型现状

已落地的模型能力：

| 模型 | 当前作用 |
| --- | --- |
| `Robot` | 机器人档案、状态、电量、网络强度、`stream_id`、`play_urls` |
| `RobotTelemetry` | 每次 telemetry 原始数据、位置、运动、电量、网络、video 元数据 |
| `InspectionEvent` | 识别事件、抓拍 URL、bbox、track_id、raw_detection |
| `MediaAsset` | 抓拍图/视频片段文件、sha256、文件大小、访问 URL |
| `PatrolTask` | 演示巡检任务 |

后续可扩展但暂未落地：

- `RobotDeviceCredential`
- `RobotCamera`
- `StreamSession`
- `DetectionRecord`

## 8. 本地 MVP 验证流程

1. 启动 Django：

```bash
cd backend
python manage.py runserver 127.0.0.1:8000
```

2. 启动 Vue：

```bash
cd frontend
npm run dev
```

3. 启动 ZLMediaKit：

```bash
docker compose -f docker-compose.zlmediakit.yml up -d
```

4. 启动推流：

```bash
cd bot-version
python run_stream.py --config config.yaml
```

5. 验证播放：

```text
http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv
```

6. 登录前端：

```text
http://127.0.0.1:5173/
```

首页应显示实时视频流。事件页面应显示后端保存的识别事件与抓拍图。

## 9. 已知注意事项

### 9.1 RTSP 源必须可被 ffmpeg 正常读取

如果运行 `run_stream.py` 后看到：

```text
Invalid data found when processing input
```

说明当前机器访问到的 RTSP 地址不是有效媒体流。需要先单独验证：

```bash
ffmpeg -rtsp_transport tcp -i rtsp://192.168.234.1:8554/test -t 5 -f null -
```

只有这个命令成功，后续推流到 ZLMediaKit 才会成功。

### 9.2 不要重复推同一个 stream_id

ZLMediaKit 同一时刻不允许两个推流器发布同一个：

```text
app=live
stream=dog_ZSL-1A-07_front
```

否则会返回：

```text
Already publishing
```

当前 `run_stream.py` 已加入本机单实例锁，但如果另一台机器正在推同名流，也需要先停掉旧推流或换一个 `stream_id`。

### 9.3 本地与云端地址不同

本地 Docker 使用：

```text
rtmp://127.0.0.1/live/dog_ZSL-1A-07_front
http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv
```

云服务器应改为：

```text
rtmp://<cloud-server-ip>/live/dog_ZSL-1A-07_front
http://<cloud-server-ip>/live/dog_ZSL-1A-07_front.live.flv
```

如果前面有 Nginx/HTTPS，也可以把播放 URL 换成公网 HTTPS 域名。

## 10. 后续演进

MVP 已完成视频推流、前端播放、抓拍上传、遥测入库和事件展示。正式部署建议继续补齐：

- 设备级鉴权：`X-Device-Code`、`X-Timestamp`、`X-Nonce`、`X-Signature`
- ZLMediaKit `on_publish` hook，校验推流 token
- HTTPS 与 Nginx 反向代理
- 弱网缓存与补传队列
- 视频流在线/离线状态回调
- WebRTC/WHEP 低延迟播放
- 多摄像头与多机器人管理
- PostgreSQL/MySQL 替代 SQLite
