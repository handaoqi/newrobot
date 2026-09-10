#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import time

import numpy as np

from bike_bot.config import AppConfig
from bike_bot.detector import YoloDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build and validate the configured TensorRT engine cache."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--runs", type=int, default=2)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    config = AppConfig.from_file(args.config)
    config.ensure_directories()
    detector = YoloDetector(config, emit_events=False)
    if not detector.gpu_inference_active:
        raise SystemExit(
            f"GPU prewarm failed: active providers={detector.inference_providers}"
        )

    frame = np.zeros((config.video.height, config.video.width, 3), dtype=np.uint8)
    timings = []
    for _ in range(max(1, args.runs)):
        started = time.perf_counter()
        detector._predict(frame)
        timings.append(round((time.perf_counter() - started) * 1000.0, 2))

    print(json.dumps({
        "status": "ready",
        "providers": detector.inference_providers,
        "onnxruntime_version": detector.onnxruntime_version,
        "cache_path": config.model.tensorrt_engine_cache_path,
        "run_ms": timings,
    }, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
