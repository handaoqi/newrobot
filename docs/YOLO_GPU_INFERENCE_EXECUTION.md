# 板端 YOLO GPU/TensorRT 优化 — 执行进度

更新时间：2026-08-22

本文件跟踪 [YOLO_GPU_INFERENCE_PLAN.md](YOLO_GPU_INFERENCE_PLAN.md) 的落地，不修改原计划。本轮按方案 B 执行。

## 拍板结论

| ID | 结论 |
|---|---|
| D1 代码树 | A，只改 `platform/bot-version-test` |
| D2 推理后端 | B，TensorRT FP16 + CUDA 兜底 + NumPy 后处理 |
| D3 目标帧率 | A，8–12fps 即可 |
| D4 输入分辨率 | A，本轮不改，仍 1080p |
| D5 engine 缓存目录 | `platform/bot-version-test/data/trt-cache/yolo11n`（见 P2 说明） |
| D6 与 SenseVoice 共用 GPU | A，共存 |
| D7 CPUQuota | A，先不改 |
| D8 检测框是否进直播 | A，不画，直播继续 copy |
| D9 允许重启 bike-bot | 已重启 `roamerx-bike-bot` |

## 任务顺序

| ID | 任务 | 状态 |
|---|---|---|
| P0 | 固化优化前基线 | 已完成 |
| P1 | NumPy 向量化后处理 + 单测 | 已完成 |
| P2 | TensorRT FP16 provider 与可回退配置 | 已完成 |
| P3 | 检测输入缩小（仅 D4≠A） | 跳过（D4=A） |
| P4 | 拆分 preprocess / session_run / parse 耗时日志 | 已完成 |
| P5 | 现场重启与验收 | 已完成（D3=A 通过；RSS 高于预期） |
| P6 | 同步板端视频链路文档 | 已完成 |

## 优化前基线

P0 于 2026-08-22 21:12 重测（进程 PID 41842，已运行约 29 分钟）。

| 项 | 值 | 来源 |
|---|---|---|
| 时间 | 2026-08-22 21:12 | 现场 |
| 进程 | `run_edge.py --config .../bike-bot.yaml` PID 41842 | `ps` |
| `%CPU` / RSS / VSZ | 138% / 648MB / ~20GB | `ps -o` |
| `avg_detect_ms` | 418–482 | `bike-bot.log` |
| `effective_fps` | 2.06–2.37 | 同上 |
| `source_fps` | 30 | 同上 |
| `dropped_frames` / 10s | 283–302 | 同上 |
| 实际采集 | 1920×1080 | `video_source_opened` |
| providers | CUDA + CPU | 启动日志 20:43 |
| `GR3D_FREQ` | 0% | `tegrastats` |
| ORT | 1.23.0，可用 TensorRT / CUDA / CPU | `python3 -c` |
| systemd | `CPUQuota=200%` | `roamerx-bike-bot.service` |

## 执行记录

### P0 固化基线

已完成。上表即为优化前数字。ORT 已具备 `TensorrtExecutionProvider`。`libnvinfer.so.10.3.0` 存在。

### P1 向量化后处理

已完成。

- `platform/bot-version-test/src/bike_bot/detector.py`：新增 `parse_yolo_predictions()`，用 NumPy `argmax`/掩码，禁止逐 anchor `.tolist()`。
- `platform/bot-version-test/tests/test_detector_parse.py`：6 项（多类、阈值、单类、channel-first、NMS、未知类）。

```bash
cd /home/dogrobot/platform/bot-version-test
python3 -m pytest -q tests/test_detector_parse.py
# 6 passed
```

### P2 TensorRT provider

已完成。

- 默认 provider：TensorRT FP16 → CUDA → CPU。
- 配置开关：`model.tensorrt_enabled`（默认 true）、`tensorrt_fp16`、`tensorrt_engine_cache_path`、`allow_cpu_fallback`。
- 编 engine 失败会打 error 并回退 CUDA/CPU。
- 现场 `runtime/nx-edge/conf/bike-bot.yaml` 已显式写入 TensorRT、FP16、固定缓存路径和 CPU fallback 策略。
- 缓存已迁移到运行时数据盘 `/home/dogrobot/runtime/nx-edge/data/vision/trt-cache/yolo11n`，并由安装脚本在服务启动前预热。

启动日志：

```text
2026-09-10 23:38:35 loading TensorRT engine cache=/home/dogrobot/runtime/nx-edge/data/vision/trt-cache/yolo11n fp16=True
2026-08-22 21:21:21 loaded ONNX ... providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'] tensorrt_enabled=True
```

首次 `session.run` 编 engine 约 349s，写出：

`/home/dogrobot/runtime/nx-edge/data/vision/trt-cache/yolo11n/TensorrtExecutionProvider_TRTKernel_graph_main_graph_*_fp16_sm87.engine`（约 8MB）

### P3 检测输入缩小

跳过。D4=A。

### P4 性能日志

已完成。`edge_perf detection_window` 增加 `avg_preprocess_ms`、`avg_session_run_ms`、`avg_parse_ms`、`max_session_run_ms`、`providers=`。

当前在跑的进程把 `providers` 打成 `-`：`_onnx_providers_label` 曾在 `_load_model()` 之后被清掉。代码已修，下次重启后会显示 `TensorrtExecutionProvider,...`。session 日志仍证明 TRT 已激活。

### P5 现场验收

2026-08-22 21:21 `sudo -n systemctl restart roamerx-bike-bot`。engine 编完后稳态（21:29–21:30）：

| 项 | 优化前 | 优化后 | 是否通过 |
|---|---|---|---|
| `avg_detect_ms` | 418–482 | 50–63（`avg_session_run_ms` 11.4–11.5；`avg_parse_ms` 2.6–2.8） | 通过 D3=A（&lt;80ms）。个别窗口到 100ms+，`max_detect_ms` 仍有 ~1s，主要是检测循环里轮询人员开关的 HTTP，不是 GPU |
| `effective_fps` | ~2.3 | 13.5–16.5 | 通过（≥8） |
| `dropped_frames` / 10s | 283–302 | 138–167 | 通过 |
| `GR3D_FREQ` | 0% | 55% | 通过 |
| RSS 是否爬升 | 648MB | 约 2120MB，稳定不爬 | 未泄漏；比计划「一两百 MB」高，Jetson 统一内存上 TRT context 会映到 RSS |
| 直播仍为 copy | 是 | 是，`ffmpeg ... -c:v copy ... dog_ZSL-1A-07_front` | 通过 |
| 告警类别/跟踪 | 自行车跟踪 | 仍有 `target_frames`，`max_target_count=2` | 通过 |

CPU 仍约 109%，因为检测从 2.5fps 提到 ~15fps，1080p letterbox/画框和读流还在 CPU 上。配额 `CPUQuota=200%` 未改。

### P6 文档同步

已完成。`platform/docs/edge-and-video-pipeline.md` 的 model 节改为 onnxruntime + TensorRT 开关，并写明检测与直播分离。

## 回滚

在 `bike-bot.yaml` 的 `model` 下增加：

```yaml
tensorrt_enabled: false
```

然后 `sudo systemctl restart roamerx-bike-bot`。向量化后处理可保留。需要重建 engine 时，先备份旧缓存，再运行预热脚本：

```bash
mv /home/dogrobot/runtime/nx-edge/data/vision/trt-cache/yolo11n \
  /home/dogrobot/runtime/nx-edge/data/vision/trt-cache/yolo11n.backup
PYTHONPATH=/home/dogrobot/platform/bot-version-test/src \
  /home/dogrobot/runtime/nx-edge/data/vision/venv/bin/python \
  /home/dogrobot/platform/bot-version-test/tools/prewarm_vision_runtime.py \
  --config /home/dogrobot/runtime/nx-edge/conf/bike-bot.yaml --runs 2
```
