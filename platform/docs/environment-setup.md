# 环境配置与启动

[返回文档中心](./project-docs-index.md)

## 1. 环境要求

| 组件 | 建议版本/说明 |
| --- | --- |
| Python | 项目根目录已有 `.venv`，后端依赖见 `backend/requirements.txt` |
| Node.js | 用于运行 Vue + Vite 前端 |
| npm | 用于安装前端依赖 |
| Docker | 用于启动 ZLMediaKit 视频服务 |
| ffmpeg | 板端推流需要，可使用系统 ffmpeg 或 `imageio-ffmpeg` |
| 摄像头/RTSP 源 | 板端识别和推流需要 |

## 2. 后端配置

后端配置文件：`backend/config/settings.py`。

关键配置：

| 配置项 | 当前值/说明 |
| --- | --- |
| `DEBUG` | `True`，演示环境开启 |
| `ALLOWED_HOSTS` | `127.0.0.1`、`localhost`、`testserver`、`192.168.234.8` |
| 数据库 | SQLite，文件为 `backend/db.sqlite3` |
| 语言时区 | `zh-hans`、`Asia/Shanghai` |
| `MEDIA_ROOT` | `backend/media` |
| `MEDIA_URL` | `/media/` |
| CORS | `CORS_ALLOW_ALL_ORIGINS=True` |
| REST 鉴权 | 默认 Token 鉴权 |

安装依赖：

```powershell
cd backend
..\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

如果使用项目根目录的 `.venv`，PowerShell 命令建议写成：

```powershell
cd C:\Users\lzc\Documents\Playground\backend
..\.venv\Scripts\python.exe manage.py migrate
..\.venv\Scripts\python.exe manage.py runserver 127.0.0.1:8000
```

健康检查：

```powershell
curl http://127.0.0.1:8000/api/health/
```

预期返回：

```json
{
  "status": "ok",
  "timestamp": "2026-05-30T..."
}
```

## 3. 前端配置

前端 API 地址在 `frontend/src/services/api.js`：

```js
const API_BASE = 'http://127.0.0.1:8000/api'
```

安装与启动：

```powershell
cd C:\Users\lzc\Documents\Playground\frontend
npm install
npm run dev
```

默认访问：

```text
http://127.0.0.1:5173/
```

登录账号：

```text
operator
admin123456
```

## 4. ZLMediaKit 启动

项目提供 Docker Compose 文件：`docker-compose.zlmediakit.yml`。

启动：

```powershell
cd C:\Users\lzc\Documents\Playground
docker compose -f docker-compose.zlmediakit.yml up -d
```

检查：

```powershell
docker ps --filter name=playground-zlmediakit
```

端口：

| 端口 | 用途 |
| --- | --- |
| `1935` | RTMP 推流入口 |
| `8080` | HTTP-FLV/HLS 播放入口，映射容器 `80` |
| `8443` | HTTPS，映射容器 `443` |
| `8554` | RTSP |
| `10000` | RTP/RTC 相关端口 |
| `8000/udp`、`9000/udp` | ZLMediaKit 相关 UDP 端口 |

## 5. 板端程序配置

配置文件：`bot-version/config.yaml`，模板为 `bot-version/config.example.yaml`。

复制模板：

```powershell
cd C:\Users\lzc\Documents\Playground\bot-version
copy config.example.yaml config.yaml
```

关键字段：

| 字段 | 说明 |
| --- | --- |
| `robot.code` | 机器人唯一编号，例如 `ZSL-1A-07` |
| `location.name` | 当前区域名称，会写入机器人位置和事件位置 |
| `video.source` | 摄像头编号或 RTSP 地址 |
| `video.stream_id` | 视频流 ID，会写入后端 `Robot.stream_id` |
| `video.play_urls.flv` | 前端优先播放的 HTTP-FLV 地址 |
| `video.play_urls.hls` | HLS 备用播放地址 |
| `stream.enable` | 是否启用 ffmpeg 推流线程 |
| `stream.rtmp_url` | 推给 ZLMediaKit 的 RTMP 地址 |
| `model.path` | ONNX 或其他模型路径 |
| `model.backend` | 板端推荐 `opencv_dnn` |
| `telemetry.endpoint` | 遥测上报接口 |
| `telemetry.media_upload_endpoint` | 抓拍媒体上传接口 |
| `snapshot.directory` | 本地抓拍保存目录 |
| `storage.telemetry_log_path` | 本地 JSONL 上报日志 |

安装板端依赖：

```powershell
cd C:\Users\lzc\Documents\Playground\bot-version
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

完整识别闭环启动：

```powershell
python run_edge.py --config config.yaml
```

仅视频推流：

```powershell
python run_stream.py --config config.yaml
```

## 6. 推荐本地启动顺序

### 6.1 只看前后端页面

```text
1. Django 后端
2. Vue 前端
3. 浏览器访问 http://127.0.0.1:5173/
```

### 6.2 视频 MVP

```text
1. ZLMediaKit
2. Django 后端
3. Vue 前端
4. bot-version/run_stream.py
```

### 6.3 完整识别闭环

```text
1. ZLMediaKit
2. Django 后端
3. Vue 前端
4. bot-version/run_edge.py
```

`run_edge.py` 已经可以启动推流线程。如果已经单独运行 `run_stream.py`，不要再让 `run_edge.py` 推同一个流，否则 ZLMediaKit 可能返回 `Already publishing`。

## 7. 常用检查命令

后端健康检查：

```powershell
curl http://127.0.0.1:8000/api/health/
```

RTSP 源检查：

```powershell
ffmpeg -rtsp_transport tcp -i rtsp://192.168.234.1:8554/test -t 5 -f null -
```

前端构建：

```powershell
cd frontend
npm run build
```

后端迁移状态：

```powershell
cd backend
..\.venv\Scripts\python.exe manage.py showmigrations
```
