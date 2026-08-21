#!/usr/bin/env python3
import base64
import math
import os
import select
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime, time as datetime_time, timezone

import rclpy
import serial
import yaml
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from robots_dog_msgs.msg import Nmea, UniRtkPvh


def nmea_checksum(sentence_body: str) -> str:
    value = 0
    for char in sentence_body:
        value ^= ord(char)
    return f"{value:02X}"


def decimal_to_nmea(value: float, is_latitude: bool) -> tuple[str, str]:
    hemi = "N" if is_latitude else "E"
    if value < 0:
        hemi = "S" if is_latitude else "W"
    value = abs(value)
    degrees = int(value)
    minutes = (value - degrees) * 60.0
    if is_latitude:
        return f"{degrees:02d}{minutes:09.6f}", hemi
    return f"{degrees:03d}{minutes:09.6f}", hemi


def build_gga(latitude: float, longitude: float, altitude: float, satellites: int, hdop: float) -> str:
    lat, ns = decimal_to_nmea(latitude, True)
    lon, ew = decimal_to_nmea(longitude, False)
    utc = time.strftime("%H%M%S.00", time.gmtime())
    sats = max(0, min(99, int(satellites or 0)))
    hdop = hdop if hdop > 0 else 1.0
    body = (
        f"GPGGA,{utc},{lat},{ns},{lon},{ew},1,{sats:02d},{hdop:.1f},"
        f"{altitude:.3f},M,0.000,M,,"
    )
    return f"${body}*{nmea_checksum(body)}\r\n"


@dataclass
class RtkPosition:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    satellites: int = 0
    hdop: float = 1.0
    stamp: float = 0.0
    quality: str = "invalid"
    horizontal_std_m: float = math.inf
    vertical_std_m: float = math.inf
    solution_status: int = -1
    position_type: int = -1
    differential_age_s: float = math.inf
    solution_satellites: int = 0

    def valid(self) -> bool:
        return abs(self.latitude) > 1e-7 and abs(self.longitude) > 1e-7


class NtripBridge(Node):
    def __init__(self):
        super().__init__("rtk_ntrip_bridge")
        config_path = self.declare_parameter(
            "config_path", "/home/dogrobot/runtime/nx-edge/conf/rtk-ntrip.yaml"
        ).value
        self.config = self.load_config(config_path)
        self.position = RtkPosition()
        self.position_lock = threading.Lock()
        self.running = True
        self.last_measurement_time_source = "none"
        self.status_pub = self.create_publisher(String, self.config["status_topic"], 10)
        self.gps_pub = self.create_publisher(Nmea, self.config["gps_topic"], 10)
        self.fix_pub = self.create_publisher(NavSatFix, self.config["fix_topic"], 10)
        self.sub = self.create_subscription(
            UniRtkPvh,
            self.config["rtk_topic"],
            self.on_rtk,
            10,
        )
        self.worker = threading.Thread(target=self.run_bridge, daemon=True)
        self.worker.start()

    def measurement_stamp(self, msg: UniRtkPvh):
        """Use the receiver measurement epoch; reception time is only a last-resort fallback."""
        header_seconds = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if header_seconds >= 946684800.0:
            return msg.header.stamp, "receiver_header"

        utc_value = float(msg.bestnav.utc_time_s)
        if utc_value >= 946684800.0:
            epoch_seconds = utc_value
            source = "receiver_utc_epoch"
        else:
            now_utc = datetime.now(timezone.utc)
            if 0.0 <= utc_value < 86400.0:
                seconds_of_day = utc_value
            elif 0.0 <= utc_value < 240000.0:
                hours = int(utc_value // 10000)
                minutes = int((utc_value % 10000) // 100)
                seconds_of_day = hours * 3600 + minutes * 60 + (utc_value % 100)
            else:
                return self.get_clock().now().to_msg(), "ros_reception_fallback"
            midnight = datetime.combine(now_utc.date(), datetime_time(), tzinfo=timezone.utc)
            epoch_seconds = midnight.timestamp() + seconds_of_day
            if epoch_seconds - now_utc.timestamp() > 43200.0:
                epoch_seconds -= 86400.0
            elif now_utc.timestamp() - epoch_seconds > 43200.0:
                epoch_seconds += 86400.0
            source = "receiver_utc_day"
        stamp = self.get_clock().now().to_msg()
        stamp.sec = int(epoch_seconds)
        stamp.nanosec = int(round((epoch_seconds - stamp.sec) * 1e9))
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        return stamp, source
        self.timer = self.create_timer(2.0, self.publish_snapshot)

    def load_config(self, path: str) -> dict:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        secret_path = data.get("secret_path", "")
        if secret_path and os.path.exists(secret_path):
            with open(secret_path, "r", encoding="utf-8") as handle:
                secret = yaml.safe_load(handle) or {}
            data.update({k: v for k, v in secret.items() if v is not None})
        env_map = {
            "host": "RTK_NTRIP_HOST",
            "port": "RTK_NTRIP_PORT",
            "mountpoint": "RTK_NTRIP_MOUNTPOINT",
            "username": "RTK_NTRIP_USERNAME",
            "password": "RTK_NTRIP_PASSWORD",
        }
        for key, env_name in env_map.items():
            if os.getenv(env_name):
                data[key] = os.getenv(env_name)
        defaults = {
            "host": "vrs.sixents.com",
            "port": 8003,
            "mountpoint": "RTCM32_GNSS",
            "username": "",
            "password": "",
            "serial_port": "/dev/ttyTHS3",
            "baudrate": 460800,
            "rtk_topic": "/rtk_pvh",
            "gps_topic": "/gps/rtk",
            "fix_topic": "/fix",
            "status_topic": "/rtk/ntrip_status",
            "gga_interval_sec": 5.0,
            "connect_timeout_sec": 8.0,
            "position_timeout_sec": 30.0,
            "default_horizontal_std_m": 0.8,
            "default_vertical_std_m": 1.5,
            "fixed_max_horizontal_std_m": 0.20,
            "float_max_horizontal_std_m": 1.50,
            "max_differential_age_sec": 5.0,
            "min_solution_satellites": 5,
            # NovAtel-compatible BESTPOS position types used by the controller.
            "fixed_position_types": [48, 49, 50],
            "float_position_types": [17, 18, 32, 33, 34],
        }
        defaults.update(data)
        defaults["port"] = int(defaults["port"])
        defaults["baudrate"] = int(defaults["baudrate"])
        defaults["gga_interval_sec"] = float(defaults["gga_interval_sec"])
        defaults["connect_timeout_sec"] = float(defaults["connect_timeout_sec"])
        defaults["position_timeout_sec"] = float(defaults["position_timeout_sec"])
        defaults["default_horizontal_std_m"] = float(defaults["default_horizontal_std_m"])
        defaults["default_vertical_std_m"] = float(defaults["default_vertical_std_m"])
        defaults["fixed_max_horizontal_std_m"] = float(defaults["fixed_max_horizontal_std_m"])
        defaults["float_max_horizontal_std_m"] = float(defaults["float_max_horizontal_std_m"])
        defaults["max_differential_age_sec"] = float(defaults["max_differential_age_sec"])
        defaults["min_solution_satellites"] = int(defaults["min_solution_satellites"])
        defaults["fixed_position_types"] = {int(value) for value in defaults["fixed_position_types"]}
        defaults["float_position_types"] = {int(value) for value in defaults["float_position_types"]}
        if not defaults["username"] or not defaults["password"]:
            raise RuntimeError("NTRIP username/password is not configured")
        return defaults

    def classify_quality(self, pos, valid: bool, horizontal_std_m: float) -> str:
        solution_ok = int(pos.p_sol_status) == 0
        position_type = int(pos.pos_type)
        differential_age = float(pos.diff_age_s)
        satellites = int(pos.soln_svs_num)
        common_ok = (
            valid
            and solution_ok
            and math.isfinite(horizontal_std_m)
            and satellites >= self.config["min_solution_satellites"]
            and 0.0 <= differential_age <= self.config["max_differential_age_sec"]
        )
        if (
            common_ok
            and position_type in self.config["fixed_position_types"]
            and horizontal_std_m <= self.config["fixed_max_horizontal_std_m"]
        ):
            return "rtk_fixed"
        if (
            common_ok
            and position_type in self.config["float_position_types"]
            and horizontal_std_m <= self.config["float_max_horizontal_std_m"]
        ):
            return "rtk_float"
        if valid and solution_ok:
            return "standalone"
        return "invalid"

    def on_rtk(self, msg: UniRtkPvh):
        pos = msg.bestnav
        valid = abs(float(pos.latitude_deg)) > 1e-7 and abs(float(pos.longitude_deg)) > 1e-7
        lat_std = float(pos.lat_std) if float(pos.lat_std) > 0.0 else self.config["default_horizontal_std_m"]
        lon_std = float(pos.lon_std) if float(pos.lon_std) > 0.0 else self.config["default_horizontal_std_m"]
        hgt_std = float(pos.hgt_std) if float(pos.hgt_std) > 0.0 else self.config["default_vertical_std_m"]
        horizontal_std = max(lat_std, lon_std)
        quality = self.classify_quality(pos, valid, horizontal_std)
        with self.position_lock:
            self.position = RtkPosition(
                latitude=float(pos.latitude_deg),
                longitude=float(pos.longitude_deg),
                altitude=float(pos.altitude_m),
                satellites=int(pos.svs_num),
                hdop=1.0,
                stamp=time.time(),
                quality=quality,
                horizontal_std_m=horizontal_std,
                vertical_std_m=hgt_std,
                solution_status=int(pos.p_sol_status),
                position_type=int(pos.pos_type),
                differential_age_s=float(pos.diff_age_s),
                solution_satellites=int(pos.soln_svs_num),
            )
        fix = NavSatFix()
        fix.header = msg.header
        fix.header.stamp, measurement_time_source = self.measurement_stamp(msg)
        fix.header.frame_id = self.config["gps_frame_id"]
        fix.status.status = {
            "rtk_fixed": NavSatStatus.STATUS_GBAS_FIX,
            "rtk_float": NavSatStatus.STATUS_SBAS_FIX,
            "standalone": NavSatStatus.STATUS_FIX,
        }.get(quality, NavSatStatus.STATUS_NO_FIX)
        fix.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        fix.latitude = float(pos.latitude_deg)
        fix.longitude = float(pos.longitude_deg)
        fix.altitude = float(pos.altitude_m)
        lat_var = lat_std * lat_std
        lon_var = lon_std * lon_std
        hgt_var = hgt_std * hgt_std
        fix.position_covariance = [lat_var, 0.0, 0.0, 0.0, lon_var, 0.0, 0.0, 0.0, hgt_var]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        nmea = Nmea()
        nmea.header = msg.header
        nmea.header.frame_id = self.config["gps_frame_id"]
        nmea.nav_sat_fix = fix
        nmea.utc_time = f"{float(pos.utc_time_s):.3f}"
        nmea.qual = {"rtk_fixed": 4, "rtk_float": 5, "standalone": 1}.get(quality, 0)
        nmea.satellites_used = int(pos.soln_svs_num)
        nmea.hdop = 1.0
        nmea.undulation = float(pos.undulation)
        nmea.undulation_units = "M"
        nmea.heading = float(msg.heading.heading_deg)
        nmea.pitch = float(msg.heading.pitch_deg)
        nmea.roll = 0.0
        if valid:
            nmea.raw_sentence_gpgga = build_gga(
                float(pos.latitude_deg),
                float(pos.longitude_deg),
                float(pos.altitude_m),
                int(pos.svs_num),
                1.0,
            ).strip()
        self.fix_pub.publish(fix)
        self.gps_pub.publish(nmea)
        self.last_measurement_time_source = measurement_time_source

    def publish_status(self, state: str, **extra):
        payload = {"state": state, "time": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        payload.update(extra)
        msg = String()
        msg.data = yaml.safe_dump(payload, allow_unicode=False, sort_keys=True).strip()
        self.status_pub.publish(msg)

    def publish_snapshot(self):
        with self.position_lock:
            pos = self.position
        self.publish_status(
            "waiting_position" if not pos.valid() else "position_ok",
            lat=round(pos.latitude, 8),
            lon=round(pos.longitude, 8),
            satellites=pos.satellites,
            solution_satellites=pos.solution_satellites,
            quality=pos.quality,
            fusion_usable=pos.quality in {"rtk_fixed", "rtk_float"},
            horizontal_std_m=round(pos.horizontal_std_m, 3) if math.isfinite(pos.horizontal_std_m) else None,
            vertical_std_m=round(pos.vertical_std_m, 3) if math.isfinite(pos.vertical_std_m) else None,
            solution_status=pos.solution_status,
            position_type=pos.position_type,
            differential_age_s=round(pos.differential_age_s, 2) if math.isfinite(pos.differential_age_s) else None,
            age_sec=round(time.time() - pos.stamp, 1) if pos.stamp else None,
            measurement_time_source=self.last_measurement_time_source,
        )

    def current_gga(self) -> str | None:
        with self.position_lock:
            pos = self.position
        if not pos.valid():
            return None
        if time.time() - pos.stamp > self.config["position_timeout_sec"]:
            return None
        return build_gga(pos.latitude, pos.longitude, pos.altitude, pos.satellites, pos.hdop)

    def open_serial(self):
        return serial.Serial(
            port=self.config["serial_port"],
            baudrate=self.config["baudrate"],
            timeout=0,
            write_timeout=1,
            exclusive=False,
        )

    def connect_ntrip(self):
        sock = socket.create_connection(
            (self.config["host"], self.config["port"]),
            timeout=self.config["connect_timeout_sec"],
        )
        sock.setblocking(False)
        auth = base64.b64encode(
            f"{self.config['username']}:{self.config['password']}".encode("utf-8")
        ).decode("ascii")
        request = (
            f"GET /{self.config['mountpoint']} HTTP/1.0\r\n"
            "User-Agent: NTRIP roamerx-rtk/1.0\r\n"
            "Ntrip-Version: Ntrip/1.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            "Connection: close\r\n\r\n"
        )
        sock.sendall(request.encode("ascii"))
        return sock

    def run_bridge(self):
        while self.running and rclpy.ok():
            ser = None
            sock = None
            try:
                self.publish_status("opening_serial", serial_port=self.config["serial_port"])
                ser = self.open_serial()

                while self.running and rclpy.ok() and self.current_gga() is None:
                    self.publish_status("waiting_valid_rtk_position")
                    time.sleep(1.0)

                self.publish_status(
                    "connecting_ntrip",
                    host=self.config["host"],
                    port=self.config["port"],
                    mountpoint=self.config["mountpoint"],
                )
                sock = self.connect_ntrip()
                last_gga = 0.0
                header_checked = False
                rtcm_bytes = 0

                while self.running and rclpy.ok():
                    now = time.time()
                    if now - last_gga >= self.config["gga_interval_sec"]:
                        gga = self.current_gga()
                        if gga:
                            sock.sendall(gga.encode("ascii"))
                            last_gga = now
                    readable, _, _ = select.select([sock], [], [], 1.0)
                    if not readable:
                        continue
                    data = sock.recv(4096)
                    if not data:
                        raise RuntimeError("NTRIP caster closed connection")
                    if not header_checked:
                        header_checked = True
                        if b"ICY 200" not in data[:120] and b"200 OK" not in data[:120]:
                            raise RuntimeError(f"NTRIP rejected request: {data[:80]!r}")
                        header_end = data.find(b"\r\n\r\n")
                        if header_end >= 0:
                            data = data[header_end + 4 :]
                        else:
                            data = data.split(b"\n", 1)[-1]
                    if data:
                        written = ser.write(data)
                        rtcm_bytes += written
                        self.publish_status("rtcm_streaming", rtcm_bytes=rtcm_bytes)
            except Exception as exc:
                self.publish_status("error", error=type(exc).__name__, detail=str(exc)[:160])
                time.sleep(3.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = NtripBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
