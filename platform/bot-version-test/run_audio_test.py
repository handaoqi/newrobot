from __future__ import annotations
import argparse, logging, shutil, signal, sys, threading
from pathlib import Path
PROJECT_ROOT = Path(__file__).resolve().parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path: sys.path.insert(0, str(SRC_DIR))
from bike_bot.audio_commands import AudioCommandClient
from bike_bot.config import AppConfig
LOGGER = logging.getLogger("bike_bot.audio_test")
def parse_args():
    p=argparse.ArgumentParser(description="Isolated cloud audio command test worker")
    p.add_argument("--config",default="config.audio-test.yaml")
    p.add_argument("--check",action="store_true")
    p.add_argument("--once",action="store_true")
    return p.parse_args()
def main():
    args=parse_args(); Path("data/logs").mkdir(parents=True,exist_ok=True)
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(levelname)s [%(name)s] %(message)s",handlers=[logging.StreamHandler(),logging.FileHandler("data/logs/audio-test.log",encoding="utf-8")])
    config=AppConfig.from_file(args.config); client=AudioCommandClient(config)
    if args.check:
        print(f"robot_code={config.robot.code}"); print(f"api_base={client.api_base}")
        for name in ("ffmpeg","ffplay","aplay","mpg123"): print(f"{name}={shutil.which(name) or ''}")
        print(f"usb_audio_device={client._detect_usb_audio_device()}"); return
    if args.once:
        command=client.poll_once()
        if command: LOGGER.info("received cloud audio command id=%s",command.get("id")); client.handle_command(command)
        else: LOGGER.info("no pending cloud audio command")
        return
    stop_event=threading.Event()
    def stop(*_args): stop_event.set()
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    LOGGER.info("audio test worker started api_base=%s robot_code=%s",client.api_base,config.robot.code)
    while not stop_event.is_set():
        try:
            command=client.poll_once()
            if command: LOGGER.info("received cloud audio command id=%s",command.get("id")); client.handle_command(command)
        except Exception: LOGGER.exception("cloud audio command poll failed"); stop_event.wait(5); continue
        stop_event.wait(1)
if __name__ == "__main__": main()
