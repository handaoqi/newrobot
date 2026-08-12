from __future__ import annotations

import argparse
import logging
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Any

import cv2

from .audio_commands import AudioCommandClient
from .config import AppConfig
from .control import CommandServer
from .detector import YoloDetector
from .logging_utils import rotating_file_handler
from .runtime import RuntimeState
from .sdk import RobotSdkClient
from .stream import StreamPusher
from .telemetry import TelemetryClient

LOGGER = logging.getLogger(__name__)
PERF_LOG_INTERVAL_SECONDS = 10.0


def _milliseconds(seconds: float) -> float:
    return round(seconds * 1000, 1)


class DetectionPerfWindow:
    def __init__(self, source_fps: float) -> None:
        self.source_fps = source_fps if source_fps > 0 else 0.0
        self.reset(time.perf_counter())

    def reset(self, now: float) -> None:
        self.started_at = now
        self.frames = 0
        self.events = 0
        self.target_frames = 0
        self.max_target_count = 0
        self.dropped_frames = 0
        self.input_age_seconds = 0.0
        self.read_seconds = 0.0
        self.detect_seconds = 0.0
        self.preview_seconds = 0.0
        self.snapshot_seconds = 0.0
        self.telemetry_seconds = 0.0
        self.loop_seconds = 0.0
        self.max_read_seconds = 0.0
        self.max_detect_seconds = 0.0
        self.max_telemetry_seconds = 0.0
        self.max_loop_seconds = 0.0

    def record(
        self,
        *,
        read_seconds: float,
        detect_seconds: float,
        preview_seconds: float,
        snapshot_seconds: float,
        telemetry_seconds: float,
        loop_seconds: float,
        event_count: int,
        target_count: int,
        dropped_frames: int,
        input_age_seconds: float,
    ) -> None:
        self.frames += 1
        self.events += event_count
        self.dropped_frames += dropped_frames
        self.input_age_seconds += input_age_seconds
        if target_count > 0:
            self.target_frames += 1
            self.max_target_count = max(self.max_target_count, target_count)
        self.read_seconds += read_seconds
        self.detect_seconds += detect_seconds
        self.preview_seconds += preview_seconds
        self.snapshot_seconds += snapshot_seconds
        self.telemetry_seconds += telemetry_seconds
        self.loop_seconds += loop_seconds
        self.max_read_seconds = max(self.max_read_seconds, read_seconds)
        self.max_detect_seconds = max(self.max_detect_seconds, detect_seconds)
        self.max_telemetry_seconds = max(self.max_telemetry_seconds, telemetry_seconds)
        self.max_loop_seconds = max(self.max_loop_seconds, loop_seconds)

    def should_log(self, now: float) -> bool:
        return now - self.started_at >= PERF_LOG_INTERVAL_SECONDS

    def log_and_reset(self, now: float) -> None:
        elapsed = max(now - self.started_at, 1e-6)
        frames = max(self.frames, 1)
        effective_fps = self.frames / elapsed
        avg_loop_seconds = self.loop_seconds / frames
        lag_risk = self._lag_risk(effective_fps, avg_loop_seconds)
        LOGGER.info(
            "edge_perf detection_window frames=%d target_frames=%d max_target_count=%d events=%d "
            "dropped_frames=%d elapsed_s=%.1f effective_fps=%.2f "
            "source_fps=%.2f lag_risk=%s avg_wait_latest_ms=%.1f avg_input_age_ms=%.1f "
            "avg_detect_ms=%.1f avg_preview_ms=%.1f "
            "avg_snapshot_ms=%.1f avg_telemetry_ms=%.1f avg_loop_ms=%.1f max_read_ms=%.1f "
            "max_detect_ms=%.1f max_telemetry_ms=%.1f max_loop_ms=%.1f",
            self.frames,
            self.target_frames,
            self.max_target_count,
            self.events,
            self.dropped_frames,
            elapsed,
            effective_fps,
            self.source_fps,
            lag_risk,
            _milliseconds(self.read_seconds / frames),
            _milliseconds(self.input_age_seconds / frames),
            _milliseconds(self.detect_seconds / frames),
            _milliseconds(self.preview_seconds / frames),
            _milliseconds(self.snapshot_seconds / frames),
            _milliseconds(self.telemetry_seconds / frames),
            _milliseconds(avg_loop_seconds),
            _milliseconds(self.max_read_seconds),
            _milliseconds(self.max_detect_seconds),
            _milliseconds(self.max_telemetry_seconds),
            _milliseconds(self.max_loop_seconds),
        )
        self.reset(now)

    def _lag_risk(self, effective_fps: float, avg_loop_seconds: float) -> str:
        if self.dropped_frames > 0:
            return "dropping_old_frames_to_keep_latest"
        if self.source_fps > 0:
            source_frame_seconds = 1.0 / self.source_fps
            if avg_loop_seconds >= source_frame_seconds * 2 or effective_fps < self.source_fps * 0.5:
                return "high_processing_slower_than_source"
            if avg_loop_seconds >= source_frame_seconds * 1.2 or effective_fps < self.source_fps * 0.8:
                return "medium_processing_near_source_rate"
            return "low_processing_keeps_up"
        if avg_loop_seconds >= 1.0:
            return "unknown_source_fps_but_processing_under_1fps"
        if avg_loop_seconds >= 0.2:
            return "unknown_source_fps_check_against_camera_fps"
        return "unknown_source_fps_processing_fast"


@dataclass
class LatestFrameSample:
    frame_id: int
    frame: Any
    read_at: float
    source_fps: float
    wait_seconds: float
    dropped_frames: int


class LatestFrameCapture:
    def __init__(self, stop_event: threading.Event, detector: YoloDetector) -> None:
        self.stop_event = stop_event
        self.detector = detector
        self._reader_stop_event = threading.Event()
        self._condition = threading.Condition()
        self._thread = threading.Thread(target=self._run, daemon=False, name="latest-frame-reader")
        self._frame: Any | None = None
        self._frame_id = 0
        self._frame_read_at = 0.0
        self._source_fps = 0.0
        self._exception: BaseException | None = None

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._reader_stop_event.set()
        with self._condition:
            self._condition.notify_all()
        self._thread.join(timeout=2)

    def wait_for_latest(self, last_frame_id: int, timeout_seconds: float = 1.0) -> LatestFrameSample | None:
        started_at = time.perf_counter()
        deadline = started_at + timeout_seconds
        with self._condition:
            while (
                self._exception is None
                and not self.stop_event.is_set()
                and not self._reader_stop_event.is_set()
                and self._frame_id <= last_frame_id
            ):
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)

            if self._exception is not None:
                raise self._exception
            if self._frame is None or self._frame_id <= last_frame_id:
                return None

            frame_id = self._frame_id
            dropped_frames = max(0, frame_id - last_frame_id - 1) if last_frame_id else 0
            return LatestFrameSample(
                frame_id=frame_id,
                frame=self._frame,
                read_at=self._frame_read_at,
                source_fps=self._source_fps,
                wait_seconds=time.perf_counter() - started_at,
                dropped_frames=dropped_frames,
            )

    def _run(self) -> None:
        capture = None
        reader_started_at = time.perf_counter()
        reader_frames = 0
        reader_failures = 0
        reader_read_seconds = 0.0
        reader_max_read_seconds = 0.0

        try:
            while not self.stop_event.is_set() and not self._reader_stop_event.is_set():
                if capture is None or not capture.isOpened():
                    LOGGER.info("opening video source: %s", self.detector.config.video.source)
                    capture = self.detector.open_capture()
                    if not capture.isOpened():
                        LOGGER.warning(
                            "cannot open video source, retrying in %ss: %s",
                            self.detector.config.video.reconnect_interval_seconds,
                            self.detector.config.video.source,
                        )
                        try:
                            capture.release()
                        except Exception:
                            pass
                        capture = None
                        self.stop_event.wait(self.detector.config.video.reconnect_interval_seconds)
                        continue

                    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    source_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
                    capture_width = float(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0.0)
                    capture_height = float(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0.0)
                    with self._condition:
                        self._source_fps = source_fps
                    LOGGER.info("video source opened: %s", self.detector.config.video.source)
                    LOGGER.info(
                        "edge_perf video_source_opened source=%s capture_fps=%.2f capture_width=%.0f "
                        "capture_height=%.0f configured_width=%d configured_height=%d rtsp_transport=%s "
                        "latest_frame_mode=true",
                        self.detector.config.video.source,
                        source_fps,
                        capture_width,
                        capture_height,
                        self.detector.config.video.width,
                        self.detector.config.video.height,
                        self.detector.config.video.rtsp_transport,
                    )
                    reader_started_at = time.perf_counter()
                    reader_frames = 0
                    reader_failures = 0
                    reader_read_seconds = 0.0
                    reader_max_read_seconds = 0.0

                read_started_at = time.perf_counter()
                ok, frame = capture.read()
                read_seconds = time.perf_counter() - read_started_at
                if not ok:
                    reader_failures += 1
                    LOGGER.warning(
                        "edge_perf read_failed read_ms=%.1f reconnect_in_s=%s source=%s",
                        _milliseconds(read_seconds),
                        self.detector.config.video.reconnect_interval_seconds,
                        self.detector.config.video.source,
                    )
                    capture.release()
                    capture = None
                    self.stop_event.wait(self.detector.config.video.reconnect_interval_seconds)
                    continue

                read_at = time.perf_counter()
                reader_frames += 1
                reader_read_seconds += read_seconds
                reader_max_read_seconds = max(reader_max_read_seconds, read_seconds)
                with self._condition:
                    self._frame_id += 1
                    self._frame = frame
                    self._frame_read_at = read_at
                    self._condition.notify_all()

                now = time.perf_counter()
                if now - reader_started_at >= PERF_LOG_INTERVAL_SECONDS:
                    elapsed = max(now - reader_started_at, 1e-6)
                    LOGGER.info(
                        "edge_perf latest_frame_reader_window captured_frames=%d elapsed_s=%.1f "
                        "capture_effective_fps=%.2f avg_capture_read_ms=%.1f max_capture_read_ms=%.1f "
                        "read_failures=%d latest_frame_id=%d",
                        reader_frames,
                        elapsed,
                        reader_frames / elapsed,
                        _milliseconds(reader_read_seconds / max(reader_frames, 1)),
                        _milliseconds(reader_max_read_seconds),
                        reader_failures,
                        self._frame_id,
                    )
                    reader_started_at = now
                    reader_frames = 0
                    reader_failures = 0
                    reader_read_seconds = 0.0
                    reader_max_read_seconds = 0.0
        except BaseException as exc:
            LOGGER.exception("latest frame reader crashed")
            with self._condition:
                self._exception = exc
                self._condition.notify_all()
        finally:
            if capture is not None:
                capture.release()


def configure_logging(
    log_path: str = "data/logs/bike_bot.log",
    max_bytes: int = 10 * 1024 * 1024,
    backup_count: int = 5,
) -> None:
    file_handler = rotating_file_handler(log_path, max_bytes, backup_count)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        handlers=[file_handler],
        force=True,
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
    person_detector: YoloDetector | None,
    client: TelemetryClient,
    runtime_state: RuntimeState,
    error_queue: Queue[BaseException],
) -> None:
    latest_capture = LatestFrameCapture(stop_event, detector)
    latest_capture.start()
    last_frame_at = time.perf_counter()
    last_frame_id = 0
    frame_number = 0
    perf_window = DetectionPerfWindow(0.0)
    person_detection_enabled = False
    next_person_mode_poll_at = 0.0
    try:
        while not stop_event.is_set():
            sample = latest_capture.wait_for_latest(last_frame_id)
            if sample is None:
                continue

            if perf_window.source_fps <= 0 and sample.source_fps > 0:
                perf_window.source_fps = sample.source_fps
            if detector.config.display.enable and frame_number == 0:
                LOGGER.info("preview enabled, creating window: %s", detector.config.display.window_name)
                cv2.namedWindow(detector.config.display.window_name, cv2.WINDOW_NORMAL)

            loop_started_at = time.perf_counter()
            frame = sample.frame
            client.update_frame_size(frame.shape[1], frame.shape[0])
            frame_number += 1
            input_age_seconds = max(0.0, loop_started_at - sample.read_at)
            fps = 1.0 / max(loop_started_at - last_frame_at, 1e-6)
            last_frame_at = loop_started_at
            last_frame_id = sample.frame_id

            detect_started_at = time.perf_counter()
            result = detector.detect(frame)
            now = time.monotonic()
            if now >= next_person_mode_poll_at:
                requested = client.fetch_person_detection_enabled()
                if requested is not None and requested != person_detection_enabled:
                    person_detection_enabled = requested
                    LOGGER.info("person detection inference %s", "enabled" if requested else "disabled")
                next_person_mode_poll_at = now + 1.0
            live_tracks = [
                track
                for track in (result.tracked_objects or [])
                if str(track.label).lower() in {"bicycle", "bike", "自行车"}
            ]
            if person_detection_enabled:
                if person_detector is not None:
                    person_result = person_detector.detect(result.preview_frame)
                    result.preview_frame = person_result.preview_frame
                    result.target_count += person_result.target_count
                    live_tracks.extend(person_result.tracked_objects or [])
                else:
                    live_tracks.extend(
                        track
                        for track in (result.tracked_objects or [])
                        if str(track.label).lower() == "person"
                    )
            client.send_person_detections(live_tracks)
            detect_seconds = time.perf_counter() - detect_started_at
            target_count = result.target_count
            preview_started_at = time.perf_counter()
            preview_frame = detector.annotate_status(result.preview_frame, fps=fps, target_count=target_count)
            should_continue = detector.show_preview(preview_frame)
            preview_seconds = time.perf_counter() - preview_started_at
            if not should_continue:
                LOGGER.info("preview window requested shutdown")
                stop_event.set()
                break

            snapshot_seconds = 0.0
            telemetry_seconds = 0.0
            event_count = len(result.events)
            if not result.events:
                loop_seconds = time.perf_counter() - loop_started_at
                perf_window.record(
                    read_seconds=sample.wait_seconds,
                    detect_seconds=detect_seconds,
                    preview_seconds=preview_seconds,
                    snapshot_seconds=snapshot_seconds,
                    telemetry_seconds=telemetry_seconds,
                    loop_seconds=loop_seconds,
                    event_count=event_count,
                    target_count=target_count,
                    dropped_frames=sample.dropped_frames,
                    input_age_seconds=input_age_seconds,
                )
                if perf_window.should_log(time.perf_counter()):
                    perf_window.log_and_reset(time.perf_counter())
                continue

            LOGGER.info(
                "edge_perf detection_event_ready frame=%d source_frame_id=%d dropped_before_frame=%d "
                "input_age_ms=%.1f event_count=%d target_count=%d wait_latest_ms=%.1f detect_ms=%.1f "
                "preview_ms=%.1f",
                frame_number,
                sample.frame_id,
                sample.dropped_frames,
                _milliseconds(input_age_seconds),
                event_count,
                target_count,
                _milliseconds(sample.wait_seconds),
                _milliseconds(detect_seconds),
                _milliseconds(preview_seconds),
            )
            runtime_state.update_status(runtime_status="warning")
            for event in result.events:
                snapshot_started_at = time.perf_counter()
                event = detector.enrich_with_snapshot(event)
                snapshot_elapsed = time.perf_counter() - snapshot_started_at
                snapshot_seconds += snapshot_elapsed
                send_started_at = time.perf_counter()
                sent = client.send(detections=[event.detection])
                send_elapsed = time.perf_counter() - send_started_at
                telemetry_seconds += send_elapsed
                LOGGER.info(
                    "edge_perf detection_event_sent frame=%d source_frame_id=%d track_id=%s sent=%s "
                    "snapshot_ms=%.1f telemetry_total_ms=%.1f event_time=%s",
                    frame_number,
                    sample.frame_id,
                    event.detection.track_id or "",
                    sent,
                    _milliseconds(snapshot_elapsed),
                    _milliseconds(send_elapsed),
                    event.detection.event_time,
                )
            runtime_state.update_status(runtime_status="online")
            loop_seconds = time.perf_counter() - loop_started_at
            perf_window.record(
                read_seconds=sample.wait_seconds,
                detect_seconds=detect_seconds,
                preview_seconds=preview_seconds,
                snapshot_seconds=snapshot_seconds,
                telemetry_seconds=telemetry_seconds,
                loop_seconds=loop_seconds,
                event_count=event_count,
                target_count=target_count,
                dropped_frames=sample.dropped_frames,
                input_age_seconds=input_age_seconds,
            )
            if perf_window.should_log(time.perf_counter()):
                perf_window.log_and_reset(time.perf_counter())
    except BaseException as exc:
        LOGGER.exception("detection worker crashed")
        error_queue.put(exc)
    finally:
        latest_capture.close()
        if detector.config.display.enable:
            cv2.destroyAllWindows()


def stream_worker(stop_event: threading.Event, pusher: StreamPusher, error_queue: Queue[BaseException]) -> None:
    try:
        pusher.run_forever(stop_event)
    except BaseException as exc:
        LOGGER.exception("stream worker crashed")
        error_queue.put(exc)


def command_worker(stop_event: threading.Event, server: CommandServer, error_queue: Queue[BaseException]) -> None:
    server_thread = threading.Thread(target=server.serve_forever, daemon=True, name="command-http-server")
    try:
        server_thread.start()
        while not stop_event.is_set():
            stop_event.wait(0.5)
    except BaseException as exc:
        LOGGER.exception("command worker crashed")
        error_queue.put(exc)
    finally:
        server.shutdown()
        server_thread.join(timeout=2)


def cloud_audio_command_worker(stop_event: threading.Event, client: AudioCommandClient) -> None:
    try:
        while not stop_event.is_set():
            try:
                command = client.poll_once()
                if command:
                    LOGGER.info("received cloud audio command id=%s; replacing current playback", command.get("id"))
                    client.replace_command(command)
            except Exception:
                LOGGER.exception("cloud audio command poll failed")
                stop_event.wait(2)
                continue
            stop_event.wait(0.2)
    finally:
        client.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Realtime bicycle detection edge client.")
    parser.add_argument("--config", default="config.yaml", help="path to yaml config")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = AppConfig.from_file(args.config)
    config.ensure_directories()
    configure_logging(
        config.storage.app_log_path,
        config.storage.log_max_bytes,
        config.storage.log_backup_count,
    )
    LOGGER.info("logging to data file: %s", config.storage.app_log_path)
    runtime_state = RuntimeState(config)
    sdk_client = RobotSdkClient(config)
    try:
        detector = YoloDetector(config)
    except Exception:
        LOGGER.exception("model load failed, detection disabled")
        detector = None
    person_detector = None
    if config.person_model is not None:
        try:
            person_detector = YoloDetector(
                config,
                model_config=config.person_model,
                target_labels=["person"],
                event_labels=[],
                emit_events=False,
            )
        except Exception:
            LOGGER.exception("person model load failed, person following disabled")
    client = TelemetryClient(config, runtime_state, sdk_client)
    audio_command_client = AudioCommandClient(config)
    pusher = StreamPusher(config)
    command_server = CommandServer(config, sdk_client) if config.control.enable else None
    stop_event = threading.Event()
    error_queue: Queue[BaseException] = Queue()

    threads = []
    if config.telemetry.heartbeat_enabled:
        threads.append(
            threading.Thread(
                target=heartbeat_worker,
                args=(stop_event, client, config.telemetry.heartbeat_interval_seconds),
                daemon=False,
                name="heartbeat-worker",
            )
        )
    if config.telemetry.status_enabled:
        threads.append(
            threading.Thread(
                target=status_worker,
                args=(stop_event, client, config.telemetry.status_interval_seconds),
                daemon=False,
                name="status-worker",
            )
        )
    if detector is not None:
        threads.append(
            threading.Thread(
                target=detection_worker,
                args=(stop_event, detector, person_detector, client, runtime_state, error_queue),
                daemon=False,
                name="detection-worker",
            )
        )
    threads.append(
        threading.Thread(
            target=cloud_audio_command_worker,
            args=(stop_event, audio_command_client),
            daemon=False,
            name="cloud-audio-command-worker",
        )
    )
    if config.stream.enable:
        threads.append(
            threading.Thread(
                target=stream_worker,
                args=(stop_event, pusher, error_queue),
                daemon=False,
                name="zlm-stream-worker",
            )
        )
    if command_server:
        threads.append(
            threading.Thread(
                target=command_worker,
                args=(stop_event, command_server, error_queue),
                daemon=False,
                name="command-worker",
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
