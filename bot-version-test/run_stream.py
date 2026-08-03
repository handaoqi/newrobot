from __future__ import annotations

import argparse
import contextlib
import logging
import os
import signal
import sys
import threading
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from bike_bot.config import AppConfig
from bike_bot.logging_utils import rotating_file_handler
from bike_bot.stream import StreamPusher

LOGGER = logging.getLogger(__name__)


def configure_logging(
    log_path: str = "data/logs/run_stream.log",
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Push robot RTSP stream to ZLMediaKit.")
    parser.add_argument("--config", default="config.yaml", help="path to yaml config")
    return parser.parse_args()


@contextlib.contextmanager
def single_instance_lock(config: AppConfig):
    stream_id = config.video.stream_id or f"dog_{config.robot.code}_{config.video.camera_id}"
    safe_name = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in stream_id)
    lock_dir = Path("data") / "stream-locks"
    lock_dir.mkdir(parents=True, exist_ok=True)
    lock_path = lock_dir / f"{safe_name}.lock"
    lock_file = lock_path.open("a+")
    lock_file.seek(0)
    acquired = False
    try:
        if os.name == "nt":
            import msvcrt

            try:
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
                acquired = True
            except OSError as exc:
                raise RuntimeError(f"stream {stream_id} is already pushed by another local process") from exc
        else:
            import fcntl

            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                acquired = True
            except OSError as exc:
                raise RuntimeError(f"stream {stream_id} is already pushed by another local process") from exc

        lock_file.seek(0)
        lock_file.truncate()
        lock_file.write(str(os.getpid()))
        lock_file.flush()
        yield
    finally:
        if acquired:
            if os.name == "nt":
                import msvcrt

                lock_file.seek(0)
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        lock_file.close()


def main() -> None:
    args = parse_args()
    config = AppConfig.from_file(args.config)
    config.ensure_directories()
    configure_logging(
        config.storage.run_stream_log_path,
        config.storage.log_max_bytes,
        config.storage.log_backup_count,
    )
    LOGGER.info("logging to data file: %s", config.storage.run_stream_log_path)
    stop_event = threading.Event()

    def request_stop(_signum=None, _frame=None) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)

    try:
        with single_instance_lock(config):
            StreamPusher(config).run_forever(stop_event)
    except RuntimeError as exc:
        LOGGER.error("%s", exc)
        sys.exit(2)


if __name__ == "__main__":
    main()
