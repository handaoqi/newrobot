from __future__ import annotations

import argparse
import logging
import threading
import time
from queue import Empty, Queue

import cv2

from .config import AppConfig
from .detector import YoloDetector
from .runtime import RuntimeState
from .stream import StreamPusher
from .telemetry import TelemetryClient

LOGGER = logging.getLogger(__name__)


def configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )


def heartbeat_worker(stop_event: threading.Event, client: TelemetryClient, interval_seconds: int) -> None:
    while not stop_event.is_set():
        client.send()
        stop_event.wait(interval_seconds)


def status_worker(stop_event: threading.Event, client: TelemetryClient, interval_seconds: int) -> None:
    while not stop_event.is_set():
        client.send()
        stop_event.wait(interval_seconds)


def detection_worker(
    stop_event: threading.Event,
    detector: YoloDetector,
    client: TelemetryClient,
    runtime_state: RuntimeState,
    error_queue: Queue[BaseException],
) -> None:
    capture = None
    try:
        while not stop_event.is_set():
            if capture is None or not capture.isOpened():
                LOGGER.info("opening video source: %s", detector.config.video.source)
                capture = detector.open_capture()
                if not capture.isOpened():
                    LOGGER.warning(
                        "cannot open video source, retrying in %ss: %s",
                        detector.config.video.reconnect_interval_seconds,
                        detector.config.video.source,
                    )
                    try:
                        capture.release()
                    except Exception:
                        pass
                    capture = None
                    stop_event.wait(detector.config.video.reconnect_interval_seconds)
                    continue

                LOGGER.info("video source opened: %s", detector.config.video.source)
                if detector.config.display.enable:
                    LOGGER.info("preview enabled, creating window: %s", detector.config.display.window_name)
                    cv2.namedWindow(detector.config.display.window_name, cv2.WINDOW_NORMAL)
                last_frame_at = time.perf_counter()

            ok, frame = capture.read()
            if not ok:
                LOGGER.warning(
                    "failed to read frame from source, reconnecting in %ss",
                    detector.config.video.reconnect_interval_seconds,
                )
                capture.release()
                capture = None
                stop_event.wait(detector.config.video.reconnect_interval_seconds)
                continue

            now = time.perf_counter()
            fps = 1.0 / max(now - last_frame_at, 1e-6)
            last_frame_at = now

            result = detector.detect(frame)
            target_count = len(result.events)
            preview_frame = detector.annotate_status(result.preview_frame, fps=fps, target_count=target_count)
            should_continue = detector.show_preview(preview_frame)
            if not should_continue:
                LOGGER.info("preview window requested shutdown")
                stop_event.set()
                break

            if not result.events:
                continue

            runtime_state.update_status(runtime_status="warning")
            for event in result.events:
                if not detector.should_emit_event():
                    continue
                event = detector.enrich_with_snapshot(event)
                client.send(detections=[event.detection])
            runtime_state.update_status(runtime_status="online")
    except BaseException as exc:
        LOGGER.exception("detection worker crashed")
        error_queue.put(exc)
    finally:
        if capture is not None:
            capture.release()
        if detector.config.display.enable:
            cv2.destroyAllWindows()


def stream_worker(stop_event: threading.Event, pusher: StreamPusher, error_queue: Queue[BaseException]) -> None:
    try:
        pusher.run_forever(stop_event)
    except BaseException as exc:
        LOGGER.exception("stream worker crashed")
        error_queue.put(exc)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Realtime bicycle detection edge client.")
    parser.add_argument("--config", default="config.yaml", help="path to yaml config")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_logging()

    config = AppConfig.from_file(args.config)
    config.ensure_directories()
    runtime_state = RuntimeState(config)
    detector = YoloDetector(config)
    client = TelemetryClient(config, runtime_state)
    pusher = StreamPusher(config)
    stop_event = threading.Event()
    error_queue: Queue[BaseException] = Queue()

    threads = [
        threading.Thread(
            target=heartbeat_worker,
            args=(stop_event, client, config.telemetry.heartbeat_interval_seconds),
            daemon=False,
            name="heartbeat-worker",
        ),
        threading.Thread(
            target=status_worker,
            args=(stop_event, client, config.telemetry.status_interval_seconds),
            daemon=False,
            name="status-worker",
        ),
        threading.Thread(
            target=detection_worker,
            args=(stop_event, detector, client, runtime_state, error_queue),
            daemon=False,
            name="detection-worker",
        ),
    ]
    if config.stream.enable:
        threads.append(
            threading.Thread(
                target=stream_worker,
                args=(stop_event, pusher, error_queue),
                daemon=False,
                name="zlm-stream-worker",
            )
        )

    for thread in threads:
        thread.start()

    LOGGER.info("bike bot started")
    try:
        while True:
            try:
                worker_error = error_queue.get_nowait()
                stop_event.set()
                raise RuntimeError("detection worker failed") from worker_error
            except Empty:
                pass
            time.sleep(1)
    except KeyboardInterrupt:
        LOGGER.info("shutdown requested")
        stop_event.set()
    finally:
        for thread in threads:
            thread.join(timeout=2)


if __name__ == "__main__":
    main()
