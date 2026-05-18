import json
import os
import base64
import re
import signal
import socket
import subprocess
import sys
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parent
ASKPASS = os.environ.get("DOG_SSH_ASKPASS")
TUNNEL_PID = ROOT.parent / ".dog_remote_tunnel.pid"
TUNNEL_LOG = ROOT.parent / ".dog_remote_tunnel.log"
REMOTE_SERVER_HOST = os.environ.get("DOG_REMOTE_SERVER_HOST", "")
REMOTE_SERVER_PORT = os.environ.get("DOG_REMOTE_SERVER_PORT", "22")
REMOTE_SERVER_USER = os.environ.get("DOG_REMOTE_SERVER_USER", "mluser")
REMOTE_SERVER_KEY = Path(
    os.environ.get(
        "DOG_REMOTE_SERVER_KEY",
        str(Path.home() / ".ssh" / "dog_remote_mluser_ed25519"),
    )
)
LOCAL_RK_PORT = int(os.environ.get("DOG_REMOTE_LOCAL_RK_PORT", "2201"))
LOCAL_ORIN_PORT = int(os.environ.get("DOG_REMOTE_LOCAL_ORIN_PORT", "2202"))
TIMEOUTS = {
    "status": 10,
    "slam_start": 30,
    "slam_save": 90,
    "slam_reset": 25,
    "stop_cmd": 8,
    "map_read": 30,
    "video_info": 30,
    "video_snapshot": 45,
    "load_map": 30,
    "localization": 25,
    "nav_safety": 16,
    "navigate": 20,
    "cancel_nav": 12,
    "motion": 16,
    "motion_stream": 16,
    "motion_mode": 8,
}

def _ssh_target(user_env, default_user, host_env, default_host, port_env, default_port):
    user = os.environ.get(user_env, default_user)
    host = os.environ.get(host_env, default_host)
    port = os.environ.get(port_env, str(default_port))
    return user, host, str(port)


def _jump_spec(user, host, port):
    if str(port) == "22":
        return f"{user}@{host}"
    return f"{user}@{host}:{port}"


def build_ssh_bases():
    jump_user, jump_host, jump_port = _ssh_target(
        "DOG_JUMP_USER", "firefly", "DOG_JUMP_HOST", "192.168.234.1", "DOG_JUMP_PORT", 22
    )
    orin_user, orin_host, orin_port = _ssh_target(
        "DOG_ORIN_USER", "jszr", "DOG_ORIN_HOST", "192.168.234.234", "DOG_ORIN_PORT", 22
    )
    common = [
        "ssh",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=NUL",
        "-o",
        "ConnectTimeout=8",
    ]
    server_loopback = (
        jump_host == "127.0.0.1"
        and orin_host == "127.0.0.1"
        and jump_port == "60021"
        and orin_port == "60022"
    )
    remote_loopback = (
        jump_host == "127.0.0.1"
        and orin_host == "127.0.0.1"
        and jump_port == str(LOCAL_RK_PORT)
        and orin_port == str(LOCAL_ORIN_PORT)
    )
    if remote_loopback or server_loopback:
        ssh_base = common.copy()
        if orin_port != "22":
            ssh_base += ["-p", orin_port]
        ssh_base += [f"{orin_user}@{orin_host}"]
        rk_base = common.copy()
        if jump_port != "22":
            rk_base += ["-p", jump_port]
        rk_base += [f"{jump_user}@{jump_host}"]
        return ssh_base, rk_base

    ssh_base = common + ["-J", _jump_spec(jump_user, jump_host, jump_port)]
    if orin_port != "22":
        ssh_base += ["-p", orin_port]
    ssh_base += [f"{orin_user}@{orin_host}"]
    rk_base = common.copy()
    if jump_port != "22":
        rk_base += ["-p", jump_port]
    rk_base += [f"{jump_user}@{jump_host}"]
    return ssh_base, rk_base


def refresh_ssh_bases():
    global SSH_BASE, RK_SSH_BASE
    SSH_BASE, RK_SSH_BASE = build_ssh_bases()


refresh_ssh_bases()

ROS_ENV = (
    "export RMW_IMPLEMENTATION=rmw_zenoh_cpp; "
    "export ROS_DOMAIN_ID=24; "
    "export ROS_LOG_DIR=/tmp/roslog-jszr; "
    "source /opt/ros/humble/setup.bash; "
    "source /opt/robot/robot_nav/install/setup.bash 2>/dev/null || true; "
    "source /opt/robot/robot_slam/install/setup.bash 2>/dev/null || true; "
    "source /opt/robot/robot-driver/install/setup.bash 2>/dev/null || true; "
)


def ssh_env():
    env = os.environ.copy()
    if ASKPASS:
        env["SSH_ASKPASS"] = str(Path(ASKPASS).expanduser())
        env["SSH_ASKPASS_REQUIRE"] = "force"
        env.setdefault("DISPLAY", ":0")
    return env


def _as_text(value):
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def parse_last_json_line(result, label):
    stdout = result.get("stdout", "")
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    for line in reversed(lines):
        if line.startswith("{") or line.startswith("["):
            try:
                return json.loads(line), None
            except json.JSONDecodeError:
                continue
    detail = result.get("stderr", "") or stdout or "远端没有返回内容。"
    return None, {
        "ok": False,
        "stdout": stdout,
        "stderr": f"{label}失败：没有读到有效 JSON。{detail}",
    }


def read_json_body(handler):
    length = int(handler.headers.get("Content-Length", "0") or "0")
    if length <= 0:
        return {}
    raw = handler.rfile.read(length).decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}


def parse_map_yaml(yaml_text):
    metadata = {}
    for line in yaml_text.splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if key == "resolution":
            try:
                metadata["resolution"] = float(value)
            except ValueError:
                pass
        elif key == "origin":
            try:
                metadata["origin"] = [float(item) for item in re.findall(r"-?\d+(?:\.\d+)?", value)[:3]]
            except ValueError:
                pass
    return metadata


def _tcp_open(host, port, timeout=0.6):
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except OSError:
        return False


def _ssh_banner(host, port, timeout=1.5):
    try:
        with socket.create_connection((host, int(port)), timeout=timeout) as conn:
            conn.settimeout(timeout)
            data = conn.recv(128)
        text = data.decode("utf-8", errors="replace").strip()
        return text if text.startswith("SSH-") else ""
    except OSError:
        return ""


def _read_pid(path):
    try:
        return int(path.read_text(encoding="ascii").strip())
    except (OSError, ValueError):
        return None


def _pid_alive(pid):
    if not pid:
        return False
    try:
        if os.name == "nt":
            completed = subprocess.run(
                ["tasklist", "/FI", f"PID eq {pid}"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            return str(pid) in completed.stdout
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def set_remote_route(enabled):
    keys = {
        "DOG_JUMP_HOST": "127.0.0.1",
        "DOG_JUMP_PORT": str(LOCAL_RK_PORT),
        "DOG_ORIN_HOST": "127.0.0.1",
        "DOG_ORIN_PORT": str(LOCAL_ORIN_PORT),
    }
    if enabled:
        os.environ.update(keys)
    else:
        for key in keys:
            os.environ.pop(key, None)
    refresh_ssh_bases()


def remote_tunnel_status():
    pid = _read_pid(TUNNEL_PID)
    jump_banner = _ssh_banner("127.0.0.1", LOCAL_RK_PORT)
    orin_banner = _ssh_banner("127.0.0.1", LOCAL_ORIN_PORT)
    local_jump_ok = bool(jump_banner)
    local_orin_ok = bool(orin_banner)
    running = _pid_alive(pid)
    using_remote = os.environ.get("DOG_JUMP_HOST") == "127.0.0.1"
    if using_remote and not (local_jump_ok and local_orin_ok):
        set_remote_route(False)
        using_remote = False
    return {
        "ok": local_jump_ok and local_orin_ok,
        "running": running,
        "pid": pid,
        "mode": "remote" if using_remote else "lan",
        "server": f"{REMOTE_SERVER_USER}@{REMOTE_SERVER_HOST}:{REMOTE_SERVER_PORT}",
        "local_ports": {
            "jump": LOCAL_RK_PORT,
            "orin": LOCAL_ORIN_PORT,
        },
        "ports": {
            "jump": local_jump_ok,
            "orin": local_orin_ok,
        },
        "banners": {
            "jump": jump_banner,
            "orin": orin_banner,
        },
        "key_ready": REMOTE_SERVER_KEY.exists(),
        "log": str(TUNNEL_LOG),
    }


def start_remote_tunnel():
    status = remote_tunnel_status()
    if status["ok"]:
        set_remote_route(True)
        status["stdout"] = "remote tunnel already connected"
        return status
    if status.get("running"):
        stop_remote_tunnel()
    if not REMOTE_SERVER_HOST:
        return {
            "ok": False,
            "stderr": "DOG_REMOTE_SERVER_HOST is not configured",
        }
    if not REMOTE_SERVER_KEY.exists():
        return {
            "ok": False,
            "stderr": f"missing ssh key: {REMOTE_SERVER_KEY}",
        }

    ssh_cmd = [
        "ssh",
        "-i",
        str(REMOTE_SERVER_KEY),
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=NUL" if os.name == "nt" else "/dev/null",
        "-o",
        "ExitOnForwardFailure=yes",
        "-o",
        "ServerAliveInterval=30",
        "-o",
        "ServerAliveCountMax=3",
        "-N",
        "-p",
        str(REMOTE_SERVER_PORT),
        "-L",
        f"{LOCAL_RK_PORT}:127.0.0.1:60021",
        "-L",
        f"{LOCAL_ORIN_PORT}:127.0.0.1:60022",
        f"{REMOTE_SERVER_USER}@{REMOTE_SERVER_HOST}",
    ]
    TUNNEL_LOG.parent.mkdir(parents=True, exist_ok=True)
    with open(TUNNEL_LOG, "ab") as handle:
        creationflags = 0
        if os.name == "nt":
            creationflags = (
                getattr(subprocess, "DETACHED_PROCESS", 0)
                | getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
            )
        proc = subprocess.Popen(
            ssh_cmd,
            stdin=subprocess.DEVNULL,
            stdout=handle,
            stderr=subprocess.STDOUT,
            creationflags=creationflags,
            close_fds=True,
        )
    TUNNEL_PID.write_text(str(proc.pid), encoding="ascii")
    for _ in range(20):
        if _ssh_banner("127.0.0.1", LOCAL_RK_PORT) and _ssh_banner("127.0.0.1", LOCAL_ORIN_PORT):
            set_remote_route(True)
            status = remote_tunnel_status()
            status["stdout"] = "remote tunnel connected"
            return status
        if proc.poll() is not None:
            break
        import time
        time.sleep(0.4)
    stderr = ""
    try:
        stderr = TUNNEL_LOG.read_text(encoding="utf-8", errors="replace")[-2000:]
    except OSError:
        pass
    return {
        "ok": False,
        "stderr": stderr or "failed to establish remote tunnel",
    }


def stop_remote_tunnel():
    pid = _read_pid(TUNNEL_PID)
    if pid and _pid_alive(pid):
        if os.name == "nt":
            subprocess.run(
                ["taskkill", "/PID", str(pid), "/T", "/F"],
                capture_output=True,
                text=True,
                timeout=10,
            )
        else:
            os.kill(pid, signal.SIGTERM)
    try:
        TUNNEL_PID.unlink()
    except OSError:
        pass
    set_remote_route(False)
    status = remote_tunnel_status()
    status["ok"] = not status["ports"]["jump"] and not status["ports"]["orin"]
    status["stdout"] = "remote tunnel stopped"
    return status


def run_remote(command, timeout=30):
    env = ssh_env()
    full_command = SSH_BASE + [f"{ROS_ENV}{command}"]
    try:
        completed = subprocess.run(
            full_command,
            text=True,
            capture_output=True,
            timeout=timeout,
            env=env,
        )
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "stdout": _as_text(completed.stdout).strip(),
            "stderr": _as_text(completed.stderr).strip(),
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "returncode": -1,
            "stdout": _as_text(exc.stdout).strip(),
            "stderr": f"remote command timed out after {timeout}s",
        }
    except Exception as exc:
        return {
            "ok": False,
            "returncode": -2,
            "stdout": "",
            "stderr": str(exc),
        }


def run_remote_script(script, timeout=30):
    env = ssh_env()
    remote_script = f"{ROS_ENV}\n{script}".replace("\r\n", "\n").replace("\r", "\n")
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n", suffix=".sh", delete=False) as handle:
        handle.write(remote_script)
        script_path = handle.name
    try:
        with open(script_path, "rb") as handle:
            completed = subprocess.run(
                SSH_BASE + ["bash -s"],
                stdin=handle,
                text=False,
                capture_output=True,
                timeout=timeout,
                env=env,
            )
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "stdout": _as_text(completed.stdout).strip(),
            "stderr": _as_text(completed.stderr).strip(),
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "returncode": -1,
            "stdout": _as_text(exc.stdout).strip(),
            "stderr": f"remote script timed out after {timeout}s",
        }
    except Exception as exc:
        return {
            "ok": False,
            "returncode": -2,
            "stdout": "",
            "stderr": str(exc),
        }
    finally:
        try:
            os.unlink(script_path)
        except OSError:
            pass


def status_payload():
    command = r"""
echo '== nodes =='
timeout 5 ros2 node list | sort | grep -E 'bt_navigator|controller_server|planner_server|map_server|waypoint_follower|robot_slam|localization|livox_lidar_publisher' || true
echo '== topics =='
timeout 3 ros2 topic list | grep -E '/front_camera/image_compressed|/image_raw/compressed_h264|/navigation_state|/localization_state|/arc/slam_state|/arc/mc_state' || true
"""
    result = run_remote(command, timeout=TIMEOUTS["status"])
    result["timeout_info"] = TIMEOUTS
    result["summary"] = parse_status_summary(result.get("stdout", ""))
    return result


def slam_call(data):
    command = (
        "ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "
        f"'{{mapping_type: 0, data: {data}}}'"
    )
    result = run_remote(
        command,
        timeout=TIMEOUTS["slam_save"] if data == 5 else TIMEOUTS["slam_start"],
    )
    stdout = result.get("stdout", "")
    if data == 3 and "Current state is SUCCESS" in stdout:
        result["stdout"] = (
            stdout
            + "\n\n人话解释：上一轮地图已经保存完成，建图模块还停在“完成”状态，"
            "所以这次没有真正开始新的建图。需要先重置建图模块，再点“开始建图”。"
        ).strip()
    if data == 3 and "Current state is ACTIVE" in stdout:
        result["ok"] = True
        result["stderr"] = ""
        result["stdout"] = (
            stdout
            + "\n\n人话解释：建图已经在进行中，刚才这次点击只是重复点击，"
            "不影响当前建图。走完路线后点“保存地图”。"
        ).strip()
    if data == 5 and "Current state is SUCCESS" in stdout:
        result["ok"] = True
        result["stderr"] = ""
        result["stdout"] = (
            stdout
            + "\n\n地图已经处于保存完成状态。这通常表示这次没有新的建图会话可保存，"
            "或者上一轮保存已经完成。"
        ).strip()
    return result


def zero_velocity():
    command = """
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist
from robots_dog_msgs.msg import HighLevelCmd

rclpy.init()
node = rclpy.create_node('mvp_stop_motion')
cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
high_pub = node.create_publisher(HighLevelCmd, '/highlevel_cmd', 1)
twist = Twist()
high = HighLevelCmd()
high.control_mode = 1
high.motion_mode = 1
end = time.time() + 0.8
while time.time() < end:
    cmd_vel_pub.publish(twist)
    high_pub.publish(high)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.06)
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["stop_cmd"])
    if result.get("stdout"):
        result["stdout"] = (result["stdout"] + "\n\n人话解释：已发送停止/站立保持命令。").strip()
    else:
        result["stdout"] = "人话解释：已发送停止/站立保持命令。"
    return result


def motion_pulse(linear=0.0, angular=0.0, duration=0.6):
    try:
        linear = max(-0.25, min(float(linear), 0.25))
        angular = max(-0.8, min(float(angular), 0.8))
        duration = max(0.1, min(float(duration), 2.0))
    except (TypeError, ValueError):
        return {"ok": False, "stdout": "", "stderr": "运动参数格式不对。"}

    command = f"""
python3 - <<'PY'
import time
import rclpy
from robots_dog_msgs.msg import NavigationCmd

rclpy.init()
node = rclpy.create_node('mvp_motion_pulse')
pub = node.create_publisher(NavigationCmd, '/navigo/cs/cmn/intf/cmd_vel_raw', 10)

def make_cmd(linear, angular):
    msg = NavigationCmd()
    msg.source = NavigationCmd.SOURCE_NAV
    msg.mc_mode_cmd = NavigationCmd.MC_MODE_NAV_VEL_CTRL
    msg.vel_cmd.linear.x = linear
    msg.vel_cmd.angular.z = angular
    return msg

move = make_cmd({linear:.6f}, {angular:.6f})
stop = make_cmd(0.0, 0.0)
end = time.time() + {duration:.6f}
while time.time() < end:
    pub.publish(move)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.05)
for _ in range(12):
    pub.publish(stop)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["motion"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + f"\n\n人话解释：已发送点动指令 linear={linear:.2f}, angular={angular:.2f}, "
            f"duration={duration:.1f}s，并补发停止。"
        ).strip()
    return result


def motion_stream(linear=0.0, angular=0.0, duration=0.35):
    try:
        linear = max(-0.25, min(float(linear), 0.25))
        angular = max(-0.8, min(float(angular), 0.8))
        duration = max(0.1, min(float(duration), 0.8))
    except (TypeError, ValueError):
        return {"ok": False, "stdout": "", "stderr": "运动参数格式不对。"}

    command = f"""
python3 - <<'PY'
import time
import rclpy
from robots_dog_msgs.msg import NavigationCmd

rclpy.init()
node = rclpy.create_node('mvp_motion_stream')
pub = node.create_publisher(NavigationCmd, '/navigo/cs/cmn/intf/cmd_vel_raw', 10)
msg = NavigationCmd()
msg.source = NavigationCmd.SOURCE_NAV
msg.mc_mode_cmd = NavigationCmd.MC_MODE_NAV_VEL_CTRL
msg.vel_cmd.linear.x = {linear:.6f}
msg.vel_cmd.angular.z = {angular:.6f}
end = time.time() + {duration:.6f}
while time.time() < end:
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
"""
    return run_remote_script(command, timeout=TIMEOUTS["motion_stream"])


def stand_up():
    command = """
python3 - <<'PY'
import time
import rclpy
from robots_dog_msgs.msg import HighLevelCmd

rclpy.init()
node = rclpy.create_node('mvp_stand_up')
pub = node.create_publisher(HighLevelCmd, '/highlevel_cmd', 1)
msg = HighLevelCmd()
msg.control_mode = 1
msg.motion_mode = 6
end = time.time() + 1.5
while time.time() < end:
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.08)
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["motion"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + "\n\n人话解释：已发送起立命令。请观察机器狗是否站稳，站稳后再按住移动按钮。"
        ).strip()
    else:
        result["stdout"] = "人话解释：已发送起立命令。请观察机器狗是否站稳，站稳后再按住移动按钮。"
    return result


def set_motion_mode(mode=1):
    try:
        mode = int(mode)
    except (TypeError, ValueError):
        return {"ok": False, "stdout": "", "stderr": "运动模式参数格式不对。"}
    if mode not in {0, 1, 2}:
        return {"ok": False, "stdout": "", "stderr": "只允许切换待机、导航速度控制或 ARC 速度控制。"}
    command = (
        "ros2 topic pub --once /arc/mc_mode_cmd robots_dog_msgs/msg/McModeCmd "
        f"'{{mode: {mode}}}'"
    )
    result = run_remote(command, timeout=TIMEOUTS["motion_mode"])
    labels = {0: "待机", 1: "导航速度控制", 2: "ARC 速度控制"}
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + f"\n\n人话解释：已请求切换到“{labels[mode]}”模式。"
        ).strip()
    return result


def reset_slam_manager():
    command = (
        "robot-launch restart robot-alg-manager; "
        "sleep 3; "
        "ros2 service call /get_slam_state_service robots_dog_msgs/srv/GetMapState '{}' 2>/dev/null || true"
    )
    result = run_remote(command, timeout=TIMEOUTS["slam_reset"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + "\n\n人话解释：已重启算法管理器。请等 5 到 10 秒后刷新状态，"
            "如果建图状态不再是“已保存”，就可以重新开始建图。"
        ).strip()
    return result


def map_payload(map_dir="/home/jszr/.jszr/map"):
    safe_map_dir = json.dumps(map_dir)
    command = r"""
python3 - <<'PY'
import base64
import json
from pathlib import Path

root = Path('/home/jszr/.jszr/map')
requested = Path(__MAP_DIR__).expanduser()
try:
    requested = requested.resolve()
    root_resolved = root.resolve()
except FileNotFoundError:
    requested = requested.absolute()
    root_resolved = root.absolute()

if root_resolved != requested and root_resolved not in requested.parents:
    raise SystemExit('map path is outside allowed map directory')

yaml_path = requested / 'map.yaml'
pgm_path = requested / 'map.pgm'
pcd_path = requested / 'map.pcd'
bin_path = requested / 'map.bin'
db_path = requested / 'map_db.bin'

payload = {
    'exists': yaml_path.exists() and pgm_path.exists(),
    'name': requested.name if requested != root_resolved else '当前地图',
    'directory': str(requested),
    'yaml': yaml_path.read_text(encoding='utf-8', errors='replace') if yaml_path.exists() else '',
    'pgm_b64': base64.b64encode(pgm_path.read_bytes()).decode() if pgm_path.exists() else '',
    'files': {
        'yaml': str(yaml_path),
        'pgm': str(pgm_path),
        'pcd': str(pcd_path),
        'bin': str(bin_path),
        'db': str(db_path),
    },
    'sizes': {
        'yaml': yaml_path.stat().st_size if yaml_path.exists() else 0,
        'pgm': pgm_path.stat().st_size if pgm_path.exists() else 0,
        'pcd': pcd_path.stat().st_size if pcd_path.exists() else 0,
        'bin': bin_path.stat().st_size if bin_path.exists() else 0,
        'db': db_path.stat().st_size if db_path.exists() else 0,
    }
}
print(json.dumps(payload, ensure_ascii=False))
PY
""".replace("__MAP_DIR__", safe_map_dir)
    result = run_remote_script(command, timeout=TIMEOUTS["map_read"])
    if not result["ok"]:
        return result
    payload, error = parse_last_json_line(result, "地图读取")
    if error:
        return error
    payload["ok"] = True
    payload["metadata"] = parse_map_yaml(payload.get("yaml", ""))
    payload["stderr"] = result["stderr"]
    return payload


def maps_payload():
    command = r"""
python3 - <<'PY'
import json
from datetime import datetime
from pathlib import Path

root = Path('/home/jszr/.jszr/map')
items = []

def map_item(folder, label):
    yaml_path = folder / 'map.yaml'
    pgm_path = folder / 'map.pgm'
    pcd_path = folder / 'map.pcd'
    bin_path = folder / 'map.bin'
    db_path = folder / 'map_db.bin'
    if not yaml_path.exists() or not pgm_path.exists():
        return None
    stat = yaml_path.stat()
    sizes = {
        'yaml': yaml_path.stat().st_size if yaml_path.exists() else 0,
        'pgm': pgm_path.stat().st_size if pgm_path.exists() else 0,
        'pcd': pcd_path.stat().st_size if pcd_path.exists() else 0,
        'bin': bin_path.stat().st_size if bin_path.exists() else 0,
        'db': db_path.stat().st_size if db_path.exists() else 0,
    }
    return {
        'label': label,
        'directory': str(folder),
        'updated_at': datetime.fromtimestamp(stat.st_mtime).isoformat(timespec='seconds'),
        'total_size': sum(sizes.values()),
        'sizes': sizes,
        'has_pcd': pcd_path.exists(),
        'has_bin': bin_path.exists(),
    }

current = map_item(root, '当前地图')
if current:
    items.append(current)

history = root / 'history_map'
if history.exists():
    for yaml_path in history.rglob('map.yaml'):
        item = map_item(yaml_path.parent, yaml_path.parent.name)
        if item:
            items.append(item)

items.sort(key=lambda item: item['updated_at'], reverse=True)
print(json.dumps({'ok': True, 'root': str(root), 'maps': items[:80]}, ensure_ascii=False))
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["map_read"])
    if not result["ok"]:
        return result
    payload, error = parse_last_json_line(result, "地图列表读取")
    if error:
        return error
    payload["stderr"] = result["stderr"]
    return payload


def video_info_payload():
    command = r"""
echo 'RTSP: rtsp://192.168.234.1:8554'
python3 - <<'PY'
import socket
for host, port in [('192.168.234.1', 8554)]:
    try:
        sock = socket.create_connection((host, port), timeout=3)
        sock.close()
        print(f'rtsp_port: {host}:{port} open')
    except Exception as exc:
        print(f'rtsp_port: {host}:{port} failed {exc}')
PY
ros2 topic list | grep -E '/front_camera/image_compressed|/image_raw/compressed_h264|/seg_vis/compressed' || true
timeout 4 ros2 topic echo /front_camera/image_compressed --once 2>/dev/null | head -12 || true
"""
    result = run_remote(command, timeout=TIMEOUTS["video_info"])
    result["timeout_info"] = TIMEOUTS
    return result


def video_snapshot():
    command = r"""
python3 - <<'PY'
import base64
import rclpy
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = rclpy.create_node('mvp_camera_snapshot')
box = {}

def cb(msg):
    box['data'] = bytes(msg.data)
    box['format'] = msg.format

node.create_subscription(CompressedImage, '/front_camera/image_compressed', cb, 10)
deadline = node.get_clock().now().nanoseconds + 5_000_000_000
while rclpy.ok() and 'data' not in box and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()

if 'data' not in box:
    raise SystemExit('no camera frame received')
print(base64.b64encode(box['data']).decode())
PY
"""
    result = run_remote(command, timeout=TIMEOUTS["video_snapshot"])
    if not result["ok"]:
        return None, result
    lines = [line.strip() for line in result["stdout"].splitlines() if line.strip()]
    if not lines:
        return None, {"ok": False, "stderr": "empty camera frame response", "stdout": result["stdout"]}
    try:
        return base64.b64decode(lines[-1]), None
    except Exception as exc:
        return None, {"ok": False, "stderr": f"snapshot decode failed: {exc}", "stdout": result["stdout"]}


def load_navigation_map(map_dir="/home/jszr/.jszr/map"):
    safe_map_dir = json.dumps(map_dir)
    command = r"""
python3 - <<'PY'
import json
from pathlib import Path

root = Path('/home/jszr/.jszr/map').resolve()
requested = Path(__MAP_DIR__).expanduser().resolve()
if root != requested and root not in requested.parents:
    raise SystemExit('map path is outside allowed map directory')

pcd = requested / 'map.pcd'
yaml = requested / 'map.yaml'
pgm = requested / 'map.pgm'
print(json.dumps({
    'pcd': str(pcd),
    'yaml_exists': yaml.exists(),
    'pgm_exists': pgm.exists(),
    'pcd_exists': pcd.exists(),
}, ensure_ascii=False))
PY
""".replace("__MAP_DIR__", safe_map_dir)
    info = run_remote_script(command, timeout=TIMEOUTS["map_read"])
    if not info["ok"]:
        return info
    payload, error = parse_last_json_line(info, "地图路径校验")
    if error:
        return error
    if not payload.get("pcd_exists"):
        return {
            "ok": False,
            "stdout": json.dumps(payload, ensure_ascii=False),
            "stderr": "这张地图没有 map.pcd，不能加载给定位模块。",
        }

    pcd_path = payload["pcd"].replace("'", "'\"'\"'")
    ros = (
        "ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap "
        f"\"{{pcd_path: '{pcd_path}'}}\""
    )
    result = run_remote(ros, timeout=TIMEOUTS["load_map"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + "\n\n人话解释：已请求定位/导航模块加载这张地图。请等几秒后刷新状态，"
            "定位不再显示“未加载地图”时，再发送目标点。"
        ).strip()
    return result


def localization_status():
    command = r"""
python3 - <<'PY'
import json
import subprocess
import time

import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

topic = '/localization_state'
try:
    topic_type = subprocess.check_output(['ros2', 'topic', 'type', topic], text=True, timeout=4).strip()
    if not topic_type:
        raise RuntimeError('empty topic type')
    msg_type = get_message(topic_type)
    box = {}

    rclpy.init()
    node = rclpy.create_node('mvp_localization_probe')

    def cb(msg):
        box['message'] = message_to_ordereddict(msg)

    sub = node.create_subscription(msg_type, topic, cb, 10)
    deadline = time.time() + 5.0
    while time.time() < deadline and 'message' not in box:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()

    if 'message' not in box:
        print(json.dumps({'ok': False, 'error': 'no localization_state message'}, ensure_ascii=False))
    else:
        payload = {'ok': True, 'topic_type': topic_type, 'message': box['message']}
        print(json.dumps(payload, ensure_ascii=False))
except Exception as exc:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=False))
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["localization"])
    if not result["ok"]:
        return result
    payload, error = parse_last_json_line(result, "定位状态读取")
    if error:
        return error

    message = payload.get("message") or {}
    description = str(message.get("description") or message.get("desc") or "")
    status = message.get("status")
    status_text = f"status={status}" if status is not None else "status=unknown"
    check_text = f"{description} {status_text}".lower()
    bad_words = [
        "failed",
        "failure",
        "error",
        "map not loaded",
        "not loaded",
        "standby",
        "waiting",
        "reached max",
        "init failed",
        "timeout",
    ]
    ready = bool(payload.get("ok")) and bool(message) and not any(word in check_text for word in bad_words)
    return {
        "ok": True,
        "ready": ready,
        "status": status,
        "description": description,
        "message": message,
        "stdout": result.get("stdout", ""),
        "stderr": result.get("stderr", ""),
    }


def navigation_safety_status():
    command = r"""
python3 - <<'PY'
import json
import subprocess
import time

import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

topic = '/navigo/ea/cmn/intf/nav_error_primary'
try:
    topic_type = subprocess.check_output(['ros2', 'topic', 'type', topic], text=True, timeout=4).strip()
    if not topic_type:
        raise RuntimeError('empty topic type')
    msg_type = get_message(topic_type)
    box = {}

    rclpy.init()
    node = rclpy.create_node('mvp_nav_safety_probe')

    def cb(msg):
        box['message'] = message_to_ordereddict(msg)

    sub = node.create_subscription(msg_type, topic, cb, 10)
    deadline = time.time() + 5.0
    while time.time() < deadline and 'message' not in box:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()

    if 'message' not in box:
        print(json.dumps({'ok': True, 'active': False, 'message': None}, ensure_ascii=False))
    else:
        payload = {'ok': True, 'active': True, 'topic_type': topic_type, 'message': box['message']}
        print(json.dumps(payload, ensure_ascii=False))
except Exception as exc:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=False))
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["nav_safety"])
    if not result["ok"]:
        return result
    payload, error = parse_last_json_line(result, "导航安全状态读取")
    if error:
        return error

    message = payload.get("message") or {}
    text = str(message.get("message") or "")
    code = message.get("code", 0)
    details = message.get("details") or []
    active = bool(payload.get("active")) and bool(code)
    return {
        "ok": True,
        "active": active,
        "code": code,
        "message": text,
        "details": details,
        "stdout": result.get("stdout", ""),
        "stderr": result.get("stderr", ""),
    }


def set_initial_pose(x, y, yaw=0.0):
    try:
        x = float(x)
        y = float(y)
        yaw = float(yaw)
    except (TypeError, ValueError):
        return {"ok": False, "stdout": "", "stderr": "定位初始位姿坐标格式不对。"}

    import math

    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    command = f"""
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
node = rclpy.create_node('mvp_initial_pose')
pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.header.stamp = node.get_clock().now().to_msg()
msg.pose.pose.position.x = {x:.6f}
msg.pose.pose.position.y = {y:.6f}
msg.pose.pose.position.z = 0.0
msg.pose.pose.orientation.x = 0.0
msg.pose.pose.orientation.y = 0.0
msg.pose.pose.orientation.z = {qz:.9f}
msg.pose.pose.orientation.w = {qw:.9f}
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.25
for _ in range(12):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.02)
    time.sleep(0.08)
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = run_remote_script(command, timeout=TIMEOUTS["load_map"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + f"\n\n人话解释：已把初始位姿发到 x={x:.3f}, y={y:.3f}, yaw={yaw:.2f}。"
            "请等几秒后刷新状态，定位正常后再发送导航目标。"
        ).strip()
    else:
        result["stdout"] = (
            f"人话解释：已把初始位姿发到 x={x:.3f}, y={y:.3f}, yaw={yaw:.2f}。"
            "请等几秒后刷新状态，定位正常后再发送导航目标。"
        )
    return result


def navigate_to_pose(x, y, yaw=0.0, speed=0.25, tolerance=0.35):
    try:
        x = float(x)
        y = float(y)
        yaw = float(yaw)
        speed = max(0.05, min(float(speed), 0.5))
        tolerance = max(0.1, min(float(tolerance), 1.0))
    except (TypeError, ValueError):
        return {"ok": False, "stdout": "", "stderr": "导航目标坐标格式不对。"}

    loc = localization_status()
    if not loc.get("ok"):
        return {
            "ok": False,
            "stdout": loc.get("stdout", ""),
            "stderr": "导航前检查定位状态失败，先不要让机器狗自动走。",
            "localization": loc,
        }
    if not loc.get("ready"):
        desc = loc.get("description") or "没有读到可用定位描述"
        status = loc.get("status")
        return {
            "ok": False,
            "stdout": loc.get("stdout", ""),
            "stderr": (
                f"定位还没准备好，已拦截导航目标。当前定位 status={status}, description={desc}。"
                "请先加载地图，并在地图上点机器狗当前大概位置发送初始位姿，等定位正常后再发目标点。"
            ),
            "localization": loc,
        }

    safety = navigation_safety_status()
    if not safety.get("ok"):
        return {
            "ok": False,
            "stdout": safety.get("stdout", ""),
            "stderr": "导航前检查安全状态失败，先不要让机器狗自动走。",
            "safety": safety,
        }
    if safety.get("active"):
        message = safety.get("message") or "未知导航安全错误"
        code = safety.get("code")
        return {
            "ok": False,
            "stdout": safety.get("stdout", ""),
            "stderr": (
                f"导航安全层还有未清除错误，已拦截目标点。当前错误 code={code}, message={message}。"
                "请先确认雷达/避障数据正常、机器狗周围安全区没有障碍物，再重新发送目标。"
            ),
            "safety": safety,
        }

    import math

    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    goal = {
        "pose": {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
        },
        "desired_velocity": {"x": speed, "y": 0.0, "z": 0.0},
        "goal_tolerance": {"x": tolerance, "y": tolerance, "theta": 0.5},
        "behavior_tree": "",
    }
    goal_text = json.dumps(goal)
    command = (
        "ros2 action send_goal /navigate_to_pose robots_dog_msgs/action/NavigateToPose "
        + json.dumps(goal_text)
    )
    result = run_remote(command, timeout=TIMEOUTS["navigate"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + f"\n\n人话解释：已发送目标点 x={x:.3f}, y={y:.3f}。"
            "机器狗会尝试规划路径并自动避障。请现场看护，必要时点“取消导航/停止”。"
        ).strip()
    return result


def cancel_navigation():
    command = """
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
python3 - <<'PY'
import time
import rclpy
from robots_dog_msgs.msg import NavigationCmd

rclpy.init()
node = rclpy.create_node('mvp_cancel_nav_stop')
pub = node.create_publisher(NavigationCmd, '/navigo/cs/cmn/intf/cmd_vel_raw', 10)
msg = NavigationCmd()
msg.source = NavigationCmd.SOURCE_NAV
msg.mc_mode_cmd = NavigationCmd.MC_MODE_NAV_VEL_CTRL
for _ in range(16):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.01)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
"""
    result = run_remote(command, timeout=TIMEOUTS["cancel_nav"])
    if result.get("stdout"):
        result["stdout"] = (
            result["stdout"]
            + "\n\n人话解释：已请求取消当前导航目标，并补发停止速度指令。"
        ).strip()
    return result


def _find_line(text, prefix):
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            return stripped.split(":", 1)[1].strip()
    return ""


def _find_all(text, prefix):
    values = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            if ":" in stripped:
                values.append(stripped.split(":", 1)[1].strip())
            else:
                values.append(stripped)
    return values


def parse_status_summary(stdout):
    if not stdout:
        return {
            "overall": "未读到状态",
            "robot": "状态读取失败",
            "navigation": "未知",
            "localization": "未知",
            "mapping": "未知",
            "pose": "",
            "recommendation": "请先刷新状态，确认 Orin 和 ROS 服务在线。",
        }

    nav_state = _find_line(stdout, "state:")
    x = _find_line(stdout, "x:")
    y = ""
    x_seen = False
    for line in stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("x:") and not x_seen:
            x_seen = True
            x = stripped.split(":", 1)[1].strip()
            continue
        if x_seen and stripped.startswith("y:"):
            y = stripped.split(":", 1)[1].strip()
            break

    localization_desc = _find_line(stdout, "description:")
    slam_error = _find_line(stdout, "error_msg:")
    motion_match = re.search(r"== motion_state ==.*?state:\s*(\d+).*?error_code:\s*(\d+).*?error_msg:\s*'?(.*?)'?\s*(?:---|==)", stdout, re.S)
    motion_code = motion_match.group(1) if motion_match else ""
    slam_service_state = ""
    service_match = re.search(r"GetMapState_Response\(state=(\d+),\s*error_code=(\d+),\s*error_msg='([^']*)'", stdout)
    if service_match:
        slam_service_state = service_match.group(1)
    lifecycle_states = _find_all(stdout, "active")
    nodes = [line.strip() for line in stdout.splitlines() if line.strip().startswith("/")]
    core_nodes = [n for n in nodes if n in {
        "/bt_navigator",
        "/controller_server",
        "/planner_server",
        "/map_server",
        "/waypoint_follower",
        "/robot_slam",
        "/localization",
        "/livox_lidar_publisher",
    }]

    mapping_state = "待机"
    mapping_code = slam_service_state or ""
    mapping_hint = "建图服务空闲。"
    if slam_service_state == "3":
        mapping_state = "建图中"
        mapping_hint = "正在记录地图。请让机器狗缓慢走完整个区域，完成后保存地图。"
    elif slam_service_state == "6":
        mapping_state = "已保存"
        mapping_hint = "上一轮地图已保存完成。要重新建图，需要先重置建图模块。"
    elif slam_error == "standby":
        mapping_state = "待机"
        mapping_hint = "当前没有正在进行的建图任务。"
    elif slam_error:
        mapping_state = slam_error
        mapping_hint = slam_error

    localization_state = "未定位"
    localization_hint = "定位模块没有拿到可用地图。"
    if "map not loaded yet" in localization_desc:
        localization_state = "未加载地图"
        localization_hint = "先加载地图，定位模块才能进入正常工作状态。"
    elif "standby" in localization_desc.lower():
        localization_state = "待机"
        localization_hint = localization_desc
    elif localization_desc:
        localization_state = localization_desc
        localization_hint = localization_desc

    navigation_state = "空闲"
    navigation_hint = "当前没有执行导航任务。"
    if nav_state and nav_state != "0":
        navigation_state = f"任务中({nav_state})"
        navigation_hint = "导航栈正在执行或处理中间状态。"

    motion_names = {
        "0": "待机",
        "1": "导航速度控制",
        "2": "ARC速度控制",
        "3": "位置控制",
        "4": "SU控制",
        "5": "充电",
        "6": "被动",
        "7": "故障",
    }
    motion_state = motion_names.get(motion_code, "未知")
    motion_hint = "趴着或待机时，速度指令通常不会让机器狗移动。"
    if motion_code in {"1", "2"}:
        motion_hint = "底盘处于速度控制模式，可以测试按住移动。"

    overall = "在线"
    robot = "核心节点在线，传感器与导航栈已启动。"
    recommendation = "下一步可以先加载地图，再做定位和固定路线巡检。"

    if not core_nodes:
        overall = "异常"
        robot = "没有读到核心节点，可能是 ROS 域或网络链路出了问题。"
        recommendation = "先检查热点连接、Orin 在线状态和 robot-launch 服务。"
    elif "未加载地图" in localization_state:
        overall = "待准备"
        recommendation = "地图已经生成的话，下一步应该把地图加载进定位模块。"

    pose = f"x={x}, y={y}" if x and y else ""

    can_start_mapping = mapping_state in {"待机", "未知"}
    can_save_map = mapping_state == "建图中"
    can_reset_mapping = mapping_state in {"已保存", "未知"}

    return {
        "overall": overall,
        "robot": robot,
        "navigation": navigation_state,
        "navigation_hint": navigation_hint,
        "motion": motion_state,
        "motion_code": motion_code,
        "motion_hint": motion_hint,
        "localization": localization_state,
        "localization_hint": localization_hint,
        "mapping": mapping_state,
        "mapping_code": mapping_code,
        "mapping_hint": mapping_hint,
        "can_start_mapping": can_start_mapping,
        "can_save_map": can_save_map,
        "can_reset_mapping": can_reset_mapping,
        "pose": pose,
        "node_count": len(core_nodes),
        "recommendation": recommendation,
        "timeouts": TIMEOUTS,
    }


class Handler(BaseHTTPRequestHandler):
    def send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_bytes(self, content, content_type, status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/":
                content = (ROOT / "static" / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)
                return
            if path == "/static/app.css":
                content = (ROOT / "static" / "app.css").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/css; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)
                return
            if path == "/static/app.js":
                content = (ROOT / "static" / "app.js").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/javascript; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)
                return
            if path == "/api/status":
                self.send_json(status_payload())
                return
            if path == "/api/remote/status":
                self.send_json(remote_tunnel_status())
                return
            if path == "/api/map":
                query = parse_qs(parsed.query)
                map_dir = query.get("dir", ["/home/jszr/.jszr/map"])[0]
                self.send_json(map_payload(map_dir))
                return
            if path == "/api/maps":
                self.send_json(maps_payload())
                return
            if path == "/api/video/info":
                self.send_json(video_info_payload())
                return
            if path == "/api/video/snapshot":
                frame, error = video_snapshot()
                if error:
                    self.send_json(error, status=502)
                    return
                self.send_bytes(frame, "image/jpeg")
                return
            self.send_json({"ok": False, "error": "not found"}, status=404)
        except Exception as exc:
            self.send_json({"ok": False, "stderr": str(exc)}, status=500)

    def do_POST(self):
        try:
            path = urlparse(self.path).path
            if path == "/api/remote/connect":
                self.send_json(start_remote_tunnel())
                return
            if path == "/api/remote/disconnect":
                self.send_json(stop_remote_tunnel())
                return
            if path == "/api/slam/start":
                self.send_json(slam_call(3))
                return
            if path == "/api/slam/save":
                self.send_json(slam_call(5))
                return
            if path == "/api/slam/reset":
                self.send_json(reset_slam_manager())
                return
            if path == "/api/nav/load_map":
                body = read_json_body(self)
                self.send_json(load_navigation_map(body.get("dir", "/home/jszr/.jszr/map")))
                return
            if path == "/api/localization/initial_pose":
                body = read_json_body(self)
                self.send_json(set_initial_pose(
                    body.get("x"),
                    body.get("y"),
                    body.get("yaw", 0.0),
                ))
                return
            if path == "/api/nav/goal":
                body = read_json_body(self)
                self.send_json(navigate_to_pose(
                    body.get("x"),
                    body.get("y"),
                    body.get("yaw", 0.0),
                    body.get("speed", 0.25),
                    body.get("tolerance", 0.35),
                ))
                return
            if path == "/api/nav/cancel":
                self.send_json(cancel_navigation())
                return
            if path == "/api/motion/pulse":
                body = read_json_body(self)
                self.send_json(motion_pulse(
                    body.get("linear", 0.0),
                    body.get("angular", 0.0),
                    body.get("duration", 0.6),
                ))
                return
            if path == "/api/motion/stream":
                body = read_json_body(self)
                self.send_json(motion_stream(
                    body.get("linear", 0.0),
                    body.get("angular", 0.0),
                    body.get("duration", 0.35),
                ))
                return
            if path == "/api/motion/stand":
                self.send_json(stand_up())
                return
            if path == "/api/motion/mode":
                body = read_json_body(self)
                self.send_json(set_motion_mode(body.get("mode", 1)))
                return
            if path == "/api/stop":
                self.send_json(zero_velocity())
                return
            self.send_json({"ok": False, "error": "not found"}, status=404)
        except Exception as exc:
            self.send_json({"ok": False, "stderr": str(exc)}, status=500)

    def log_message(self, fmt, *args):
        sys.stdout.write("%s - %s\n" % (self.address_string(), fmt % args))


def main():
    port = int(os.environ.get("DOG_MVP_PORT", "8765"))
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"Dog MVP platform running at http://127.0.0.1:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
