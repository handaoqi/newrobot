# Bike Bot

面向机器人板端的最小可用项目，完成以下链路：

- 读取摄像头或 RTSP 视频流
- 调用 YOLO 模型实时识别自行车
- 实时显示识别框、置信度和 FPS 预览
- 按约定 JSON 结构上报遥测、状态和告警事件
- 接收后端下发的远程控制指令，并映射到机器狗 SDK 动作
- 将每次发往服务器的 JSON 保存到本地日志文件
- 保存抓拍图，并可回传可访问 URL

## 1. 安装

板端使用 ONNX Runtime GPU 推理，不需要安装 PyTorch。先安装通用依赖：

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

Jetson JetPack 6 / CUDA 12.6 需要安装 NVIDIA Jetson AI Lab 提供的适配包，不能使用通用 PyPI wheel：

```bash
chmod +x ../deploy/install_jetson_onnxruntime.sh
../deploy/install_jetson_onnxruntime.sh
```

脚本固定安装 `onnxruntime-gpu 1.23.0`，并检查 `CUDAExecutionProvider` 是否可用。

如果你想直接使用 `python -m bike_bot.main` 这类模块启动方式，再额外执行一次：

```bash
pip install -e .
```

## 2. 配置

复制并修改配置文件：

```bash
copy config.example.yaml config.yaml
```

关键字段：

- `video.source`: 摄像头编号或 RTSP 地址
- `video.rtsp_transport`: `tcp` 或 `udp`
- `video.open_timeout_seconds`: RTSP 建连超时时间
- `video.read_timeout_seconds`: RTSP 读帧超时时间
- `model.path`: 自行车告警专用 ONNX 模型，例如 `models/bike.onnx`
- `person_model.path`: 人员跟随使用的通用 YOLO 模型；平台开启跟踪识别后才执行推理
- `model.backend` / `person_model.backend`: 板端优先使用 `onnxruntime` CUDA Provider，也可使用 `opencv_dnn`
- `detection.tracking_enabled`: 是否启用同车跟踪去重
- `detection.track_ttl_seconds`: 目标离开画面多久后释放跟踪 ID
- `detection.duplicate_alert_seconds`: 同一跟踪 ID 两次告警的最小间隔
- `telemetry.endpoint`: 远程服务地址，例如 `http://10.0.0.8:8000/api/telemetry/ingest/`
- `telemetry.device_key`: 如服务端启用 `X-Device-Key`，这里填写
- `control.port`: 板端控制服务端口，默认 `9100`
- `control.dry_run`: 开发联调时保持 `true`；上板真实执行 SDK 动作时改为 `false`
- `control.sdk_lib_path`: GENISOM L1 SDK Python `.so` 所在目录，例如 `.../lib/zsl-1/aarch64`
- `control.local_ip` / `control.local_port` / `control.robot_ip`: 对应官方 SDK `initRobot()` 参数
- `snapshot.public_base_url`: 若抓拍图片经 nginx/对象存储暴露，这里填写访问前缀
- `display.enable`: 是否弹出实时识别预览窗口
- `storage.telemetry_log_path`: 本地保存上报 JSON 的日志文件

## 3. 启动板端程序

```bash
python run_edge.py --config config.yaml
```

如果你已经执行过 `pip install -e .`，也可以使用：

```bash
python -m bike_bot.main --config config.yaml
```

如果只验证“后端下发控制指令 -> 板端接收 -> SDK 动作映射”，可不启动检测和推流，单独运行：

```bash
python run_control_server.py --config config.yaml
```

当前 demo 中，前端“紧急停止”按钮会通过后端下发 `shake_hand`，板端收到后调用 GENISOM L1 SDK 的 `shakeHand()`；生产环境应把真正急停映射到 `passive` 或 `move_stop`。

## 4. 免安装部署包

如果机器狗运行时不方便安装 Python、pip 或 PyTorch，可以在同架构 Linux 环境构建免安装包。

重要限制：

- 构建环境必须和机器狗系统架构一致，例如 `aarch64` 板端要在 `aarch64 Linux` 上构建
- 免安装包仍依赖机器狗系统的基础运行环境，例如 glibc、网络和摄像头驱动
- 如果 `stream.enable: true`，需要把对应架构的 `ffmpeg` 放到 `packaging/bin/ffmpeg`，并在配置里使用 `stream.ffmpeg_path: "bin/ffmpeg"`；否则关闭推流

构建：

```bash
chmod +x scripts/build_edge_bundle.sh
./scripts/build_edge_bundle.sh
```

产物：

```text
dist/bike-bot-edge-<arch>.tar.gz
```

机器狗上只需解压并运行：

```bash
tar -xzf bike-bot-edge-aarch64.tar.gz
cd bike-bot-edge-bundle
./start.sh
```

## 5. 运行效果

- 程序启动后会弹出实时预览窗口，显示检测框、置信度、目标数量和 FPS
- 按 `q` 或 `Esc` 可关闭预览并停止程序
- 每次向服务器发送的数据都会追加写入 `data/telemetry/telemetry.jsonl`

## 6. 上报策略

- 状态遥测：默认每 `2s` 上报一次
- 心跳：默认每 `5s` 上报一次
- 告警事件：检测命中后按目标跟踪 ID 去重，同一辆车默认 300 秒内只上报一次
- 抓拍：默认保存至 `snapshots/`，并拼接 `snapshot_url`

## 7. 部署建议

- 开发机如需从 `.pt` 导出 ONNX，安装 `pip install -r requirements-dev.txt`
- 板端使用 `models/bike.onnx` + `model.backend: opencv_dnn`，避免安装 PyTorch
- 如果板端有专用 NPU/GPU，再考虑继续转换为 RKNN / TensorRT
- 使用板端硬件解码摄像头流，避免 CPU 成为瓶颈
- 告警尽量做冷却时间控制，避免同一目标连续刷屏
- 生产环境建议补充 `X-Device-Key`、请求签名或双向证书

## 8. 目录

```text
src/bike_bot/
  main.py
  config.py
  detector.py
  runtime.py
  telemetry.py
  control.py
  models.py
config.example.yaml
```
