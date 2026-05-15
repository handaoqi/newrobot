# ZLMediaKit RTSP 推流落地方案

## 目标链路

```text
机器狗 RTSP 摄像头
  -> bot-version 内置 ffmpeg 推流线程
  -> 云服务器 ZLMediaKit RTMP 接入
  -> ZLMediaKit 输出 HTTP-FLV/HLS
  -> Vue 前端播放
```

识别数据仍走原来的 Django API：

```text
bot-version 识别事件/抓拍图
  -> Django /api/device/media/upload/
  -> Django /api/telemetry/ingest/
  -> 前端事件与抓拍展示
```

## 云服务器 ZLMediaKit

本项目已提供跨平台 Docker Compose 文件：

```bash
docker compose -f docker-compose.zlmediakit.yml up -d
```

云服务器需要开放至少这些端口：

| 端口 | 用途 |
| --- | --- |
| `1935` | RTMP 推流入口 |
| `80` 或 `8080` | HTTP-FLV/HLS 播放 |

ZLMediaKit 接收的推流地址：

```text
rtmp://<cloud-server-ip>/live/dog_ZSL-1A-07_front
```

ZLMediaKit 对应播放地址：

```text
http://<cloud-server-ip>/live/dog_ZSL-1A-07_front.live.flv
http://<cloud-server-ip>/live/dog_ZSL-1A-07_front/hls.m3u8
```

## bot-version 配置

`bot-version/config.yaml` 的关键字段：

```yaml
video:
  source: "rtsp://192.168.133.1:8554/test"
  camera_id: "front"
  stream_id: "dog_ZSL-1A-07_front"
  play_urls:
    flv: "http://<cloud-server-ip>/live/dog_ZSL-1A-07_front.live.flv"
    hls: "http://<cloud-server-ip>/live/dog_ZSL-1A-07_front/hls.m3u8"

stream:
  enable: true
  ffmpeg_path: "ffmpeg"
  rtmp_url: "rtmp://<cloud-server-ip>/live/dog_ZSL-1A-07_front"
  video_codec: "copy"
```

`video.source` 是机器狗本地可访问的 RTSP 地址；`stream.rtmp_url` 是云服务器 ZLMediaKit 地址。

## 启动方式

机器狗端安装 `ffmpeg` 后运行：

```bash
python run_edge.py --config config.yaml
```

运行后会同时启动：

- 摄像头读取与自行车识别
- 抓拍上传与 telemetry 上报
- RTSP 到 ZLMediaKit 的 RTMP 推流

## 验证

1. 在云服务器确认 ZLMediaKit 有在线流：

```text
app=live
stream=dog_ZSL-1A-07_front
```

2. 浏览器直接访问 HTTP-FLV 或 HLS 地址，确认能播放。
3. 登录 Vue 前端，首页“实时视频流监控”会优先播放 `Robot.play_urls.flv`。
4. 触发识别事件后，前端事件列表应显示抓拍图与 bbox。

## 注意

- 浏览器不能直接播放 RTSP，所以前端不使用 `rtsp://...`。
- 机器狗没有公网 IP 时，必须由机器狗主动推流到云服务器。
- 正式环境建议给 RTMP 推流 URL 增加短期 token，并在 ZLMediaKit hook 中校验。
