#!/usr/bin/env python3
"""
机器狗地图管理 HTTP API 服务
提供给前端网页的地图上传/下载/建图控制接口

启动方式:
    python3 map_api_server.py --port 8088
"""

import os
import json
import shutil
import zipfile
import tempfile
import argparse
import subprocess
import threading
import time
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ──────────────────── 配置 ────────────────────
MAP_ROOT = os.path.expanduser("~/genisom_roamerx_open/map")
SLAM_MAP_DIR = "/home/user_name/.jszr/map"    # SLAM 代码硬编码了这个路径
WORKSPACE_SETUP = os.path.expanduser("~/genisom_roamerx_open/install/setup.bash")
ROS2_SETUP = "/opt/ros/humble/setup.bash"

_slam_process = None       # 当前建图进程
_slam_lock = threading.Lock()


# ──────────────────── 工具函数 ────────────────────
def ensure_dir(path):
    Path(path).mkdir(parents=True, exist_ok=True)


def ros2_source_cmd():
    """生成 source ROS2 环境的前缀命令"""
    return (
        f"export ROS_DOMAIN_ID=24 && "
        f"export RMW_IMPLEMENTATION=rmw_zenoh_cpp && "
        f"source {ROS2_SETUP} && source {WORKSPACE_SETUP} && "
    )


def get_slam_state():
    """查询 SLAM 当前状态，返回 dict"""
    with _slam_lock:
        running = _slam_process is not None and _slam_process.poll() is None
    return {
        "mapping_active": running,
        "pid": _slam_process.pid if running else None,
    }


def list_maps():
    """列出 MAP_ROOT 下所有地图"""
    maps = []
    if not os.path.isdir(MAP_ROOT):
        return maps
    for f in os.listdir(MAP_ROOT):
        if f.endswith(".yaml"):
            name = f[:-5]  # 去掉 .yaml
            pgm_path = os.path.join(MAP_ROOT, f"{name}.pgm")
            pcd_path = os.path.join(MAP_ROOT, f"{name}.pcd")
            yaml_path = os.path.join(MAP_ROOT, f)
            maps.append({
                "name": name,
                "has_pgm": os.path.isfile(pgm_path),
                "has_pcd": os.path.isfile(pcd_path),
                "yaml_size": os.path.getsize(yaml_path),
            })
    return maps


# ──────────────────── HTTP Handler ────────────────────
class MapAPIHandler(SimpleHTTPRequestHandler):
    """处理地图管理相关的 HTTP 请求"""

    def log_message(self, format, *args):
        print(f"[API] {self.client_address[0]} - {format % args}")

    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

    def _send_error_json(self, msg, status=400):
        self._send_json({"ok": False, "error": msg}, status)

    def _send_file_download(self, filepath, filename):
        """发送文件下载"""
        if not os.path.isfile(filepath):
            self._send_error_json("文件不存在", 404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(os.path.getsize(filepath)))
        self.end_headers()
        with open(filepath, "rb") as f:
            self.wfile.write(f.read())

    def do_OPTIONS(self):
        """处理 CORS 预检请求"""
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")

        # ---- 列出所有地图 ----
        if path == "/api/map/list":
            self._send_json({"ok": True, "maps": list_maps()})

        # ---- 下载单个地图 (yaml + pgm 打包成 zip) ----
        elif path.startswith("/api/map/download/"):
            map_name = path.split("/")[-1]
            yaml_path = os.path.join(MAP_ROOT, f"{map_name}.yaml")
            pgm_path = os.path.join(MAP_ROOT, f"{map_name}.pgm")

            if not os.path.isfile(yaml_path):
                self._send_error_json(f"地图 {map_name} 不存在", 404)
                return

            # 打包 zip
            tmp = tempfile.NamedTemporaryFile(suffix=".zip", delete=False)
            tmp.close()
            with zipfile.ZipFile(tmp.name, "w") as zf:
                zf.write(yaml_path, f"{map_name}.yaml")
                if os.path.isfile(pgm_path):
                    zf.write(pgm_path, f"{map_name}.pgm")

            self._send_file_download(tmp.name, f"{map_name}.zip")
            os.unlink(tmp.name)

        # ---- 下载 SLAM 最近建好的地图 ----
        elif path == "/api/slam/map":
            yaml_path = os.path.join(SLAM_MAP_DIR, "map.yaml")
            pgm_path = os.path.join(SLAM_MAP_DIR, "map.pgm")

            if not os.path.isfile(yaml_path):
                self._send_error_json("SLAM 尚未保存地图", 404)
                return

            tmp = tempfile.NamedTemporaryFile(suffix=".zip", delete=False)
            tmp.close()
            with zipfile.ZipFile(tmp.name, "w") as zf:
                zf.write(yaml_path, "map.yaml")
                if os.path.isfile(pgm_path):
                    zf.write(pgm_path, "map.pgm")

            self._send_file_download(tmp.name, "slam_map.zip")
            os.unlink(tmp.name)

        # ---- 查询 SLAM 状态 ----
        elif path == "/api/slam/status":
            self._send_json({"ok": True, **get_slam_state()})

        # ---- 静态文件 / 健康检查 ----
        elif path == "/" or path == "" or path == "/api/health":
            self._send_json({
                "ok": True,
                "service": "机器狗地图管理 API",
                "endpoints": [
                    "GET  /api/map/list          - 列出所有地图",
                    "GET  /api/map/download/<name>  - 下载地图 zip",
                    "POST /api/map/upload        - 上传地图 zip (multipart, field: file)",
                    "GET  /api/slam/status       - 建图状态",
                    "POST /api/slam/start         - 开始建图",
                    "POST /api/slam/stop          - 停止并保存地图",
                    "GET  /api/slam/map          - 下载建好的地图",
                ],
            })

        else:
            self._send_error_json("接口不存在", 404)

    def do_POST(self):
        global _slam_process
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/")
        content_length = int(self.headers.get("Content-Length", 0))

        # ---- 上传地图 ----
        if path == "/api/map/upload":
            content_type = self.headers.get("Content-Type", "")
            if "multipart/form-data" not in content_type:
                self._send_error_json("请使用 multipart/form-data 上传文件")
                return

            # 简易 multipart 解析（只取 file 字段）
            body = self.rfile.read(content_length)
            boundary = content_type.split("boundary=")[-1].encode()
            if not boundary:
                self._send_error_json("无法解析 boundary")
                return

            parts = body.split(b"--" + boundary)
            for part in parts:
                if b"Content-Disposition" not in part:
                    continue
                if b'name="file"' not in part and b'name="map"' not in part:
                    continue

                header_end = part.find(b"\r\n\r\n")
                if header_end == -1:
                    continue
                file_data = part[header_end + 4:]
                if file_data.endswith(b"\r\n"):
                    file_data = file_data[:-2]

                # 写入临时文件解压
                tmp = tempfile.NamedTemporaryFile(suffix=".zip", delete=False)
                tmp.write(file_data)
                tmp.close()

                try:
                    with zipfile.ZipFile(tmp.name, "r") as zf:
                        map_name = None
                        yamls = [n for n in zf.namelist() if n.endswith(".yaml")]
                        if yamls:
                            map_name = yamls[0][:-5]

                        ensure_dir(MAP_ROOT)
                        zf.extractall(MAP_ROOT)

                    os.unlink(tmp.name)
                    self._send_json({"ok": True, "map_name": map_name, "message": "地图上传成功"})
                    return
                except zipfile.BadZipFile:
                    os.unlink(tmp.name)
                    self._send_error_json("文件不是有效的 zip 压缩包")
                    return

            self._send_error_json("未找到上传文件，请使用字段名 file")

        # ---- 开始建图 ----
        elif path == "/api/slam/start":
            with _slam_lock:
                if _slam_process is not None and _slam_process.poll() is None:
                    self._send_json({"ok": False, "error": "建图已在运行中"})
                    return

                # 1. 启动 SLAM 节点
                cmd = f"bash -c '{ros2_source_cmd()} ros2 launch robot_slam slam.launch.py'"
                _slam_process = subprocess.Popen(
                    cmd, shell=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
                time.sleep(3)  # 等待节点启动

                if _slam_process.poll() is not None:
                    self._send_error_json("SLAM 启动失败")
                    return

                # 2. 发送开始建图指令 (data: 3 = ACTIVE)
                subprocess.run(
                    f"bash -c '{ros2_source_cmd()} "
                    f"ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{{data: 3}}''",
                    shell=True, timeout=10,
                )

            self._send_json({
                "ok": True,
                "message": "建图已开始，请遥控机器狗遍历环境",
                "pid": _slam_process.pid,
            })

        # ---- 停止并保存地图 ----
        elif path == "/api/slam/stop":
            with _slam_lock:
                if _slam_process is None or _slam_process.poll() is not None:
                    self._send_error_json("建图未在运行")
                    return

                # 1. 发送保存指令 (data: 5 = SAVE)
                print("[API] 正在发送保存指令...")
                result = subprocess.run(
                    f"bash -c '{ros2_source_cmd()} "
                    f"ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{{data: 5}}''",
                    shell=True, capture_output=True, timeout=10,
                )
                print(f"[API] 保存指令响应: {result.stdout.decode()}")

                # 2. 等待 SLAM 节点写文件 (点云大的话需要较长时间)
                print("[API] 等待地图文件写入 (最多60秒)...")
                for i in range(12):
                    time.sleep(5)
                    yaml_p = os.path.join(SLAM_MAP_DIR, "map.yaml")
                    if os.path.isfile(yaml_p) and os.path.getsize(yaml_p) > 0:
                        print(f"[API] 地图文件已生成 (第{i+1}次检查)")
                        break

                # 3. 终止 SLAM 进程
                _slam_process.terminate()
                try:
                    _slam_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    _slam_process.kill()
                _slam_process = None

            # 检查输出文件
            yaml_path = os.path.join(SLAM_MAP_DIR, "map.yaml")
            pgm_path = os.path.join(SLAM_MAP_DIR, "map.pgm")

            self._send_json({
                "ok": True,
                "message": "建图已停止，地图已保存",
                "map_files": {
                    "yaml": yaml_path,
                    "pgm": pgm_path,
                    "yaml_exists": os.path.isfile(yaml_path),
                    "pgm_exists": os.path.isfile(pgm_path),
                },
            })

        else:
            self._send_error_json("接口不存在", 404)


def main():
    parser = argparse.ArgumentParser(description="机器狗地图管理 API")
    parser.add_argument("--port", type=int, default=8088, help="监听端口 (默认 8088)")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址 (默认 0.0.0.0)")
    args = parser.parse_args()

    ensure_dir(MAP_ROOT)
    print(f"[API] 地图存储目录: {MAP_ROOT}")
    print(f"[API] SLAM 地图目录: {SLAM_MAP_DIR}")
    print(f"[API] 服务启动: http://{args.host}:{args.port}")
    print(f"[API] 接口列表见 http://{args.host}:{args.port}/api/health")

    server = HTTPServer((args.host, args.port), MapAPIHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[API] 服务已停止")
        server.server_close()


if __name__ == "__main__":
    main()
