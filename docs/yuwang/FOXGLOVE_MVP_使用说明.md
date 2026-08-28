# Foxglove 实时观测与 Bag 回放 MVP 使用说明

> 日期：2026-08-28
> 对应需求：`基于Foxglove Web插件实现ROS2 Bag回放调试方案（替代桌面版）.docx`、`Foxglove Studio ROS2 Bag回放调试界面设计.docx`
> 需求文档与真实环境的差异见同目录 [需求文档勘误.md](需求文档勘误.md)

## 1. 这个 MVP 解决什么

三件事，其中只有第三件需要写代码：

| 目标 | 实现方式 |
|---|---|
| 实时同步机器狗话题 | `foxglove_bridge`（已随 ROS 2 Humble 安装，3.4.3），只读配置 |
| Bag 回放查看 | Foxglove/Lichtblick 原生 MCAP 播放器；旧 `.db3` 用 `ros2 bag convert` 转换 |
| 不用真狗也能跑 | `roamerx_patrol_demo` 假机器狗，发布一整套标准话题 |
| 操作员不装任何客户端 | NX 自托管 Lichtblick 静态网页（需求文档的"方案 2"），浏览器直接打开 |

一条命令串起来的入口是 `robot/script/robot/foxglove_demo.sh`。

## 2. 安全边界（先读这一段）

- 假机器狗和所有回放都跑在 **`ROS_DOMAIN_ID=77` + `rmw_fastrtps_cpp`**。
  生产导航跑在 **domain 24 + `rmw_zenoh_cpp`**。两个不同的 RMW 实现互相**根本发现不了**，
  所以即使 domain 号写错，demo 数据也到不了真狗身上。
- bridge 的 `capabilities` 只保留 `connectionGraph`，`clientPublish`、`services`、
  `parameters`、`parametersSubscribe` 全部移除。**浏览器无法向机器狗发布 `/cmd_vel` 或调用服务。**
  这一点由 `foxglove_ws_probe.py` 在每次连接时断言，出现可写能力直接判失败。
- bridge 默认只绑 `127.0.0.1`。要从别的电脑连，显式传 `BRIDGE_ADDRESS=<本机内网IP>`，
  并且清楚它自身没有任何身份认证。
- **用网页模式时不要动 `BRIDGE_ADDRESS`**：网页宿主在 `/ws` 上把 WebSocket 反代到本地
  bridge，所以 bridge 可以一直只绑 `127.0.0.1`，对外只暴露一个 HTTP 端口。
  放开局域网访问改 `WEB_ADDRESS=0.0.0.0` 即可（同样没有身份认证，只在可信内网用）。
- `convert` 以只读方式打开源 Bag，不改写、不 reindex、不删除原件。

## 3. 一次性准备

```bash
cd /home/dogrobot/robot
source /opt/ros/humble/setup.bash
colcon build --packages-select roamerx_patrol_demo --symlink-install
```

再取一次网页宿主（**已在本机装好**，只有换机器或升级版本时才要重跑）：

```bash
cd /home/dogrobot
robot/script/robot/fetch_lichtblick_web.sh
```

拉的是 Lichtblick v1.28.1 的官方静态包（MPL-2.0，Foxglove Studio 的开源延续，离线、不需要账户），
解压到 `runtime/nx-edge/install/lichtblick-web/dist/`。脚本**先校验 sha256 再解压**，
不匹配直接失败。40 MB 的包和解压后的目录都不进 Git，只提交 lock 文件；
细节见 `runtime/nx-edge/install/lichtblick-web/README.md`。

装完之后**操作员的电脑上什么都不用装**，浏览器打开 NX 上的一个地址就行（见 4.2）。

需要桌面版的话，也可以自行下载 Lichtblick Desktop 或已购买授权的 Foxglove Desktop，
连 `ws://<NX_IP>:8765`；但这条路要另外放开 `BRIDGE_ADDRESS`，网页模式不需要。

## 4. 常用流程

### 4.1 环境自检（不改任何东西）

```bash
cd /home/dogrobot
robot/script/robot/foxglove_demo.sh preflight
```

检查 overlay、`foxglove_bridge`、`rosbag2_storage_mcap`、8765 端口占用和磁盘余量。

### 4.2 网页模式看假机器狗（推荐，不用真狗、不用装客户端）

```bash
robot/script/robot/foxglove_demo.sh web 600
```

一条命令同时起三样东西：假机器狗、只读 bridge、Lichtblick 网页宿主。
终端会打印访问地址，**浏览器打开就是已经加载好的 demo 布局，不需要手动导入**。

```
open http://127.0.0.1:8080/ -- the layout is already loaded
the page connects to the bridge through ws://127.0.0.1:8080/ws
```

给局域网里的操作员看：

```bash
WEB_ADDRESS=0.0.0.0 robot/script/robot/foxglove_demo.sh web 600
```

此时打印的是本机内网 IP。**注意改的是 `WEB_ADDRESS` 不是 `BRIDGE_ADDRESS`** ——
bridge 继续只绑 `127.0.0.1`，浏览器的数据走 `/ws` 反向代理，对外只开一个端口。

可调环境变量：`WEB_PORT`（默认 8080）、`WEB_ADDRESS`（默认 `127.0.0.1`）、
`WEB_LAYOUT`（默认 `docs/yuwang/demo_patrol_layout.json`，换成 `nav_debug_layout.json`
就是诊断布局）、`WEB_DIST`。

### 4.3 桌面客户端看假机器狗

```bash
robot/script/robot/foxglove_demo.sh live 120
```

然后在 Foxglove/Lichtblick 里 `Open connection` → `Foxglove WebSocket` → `ws://127.0.0.1:8765`，
导入布局 `docs/yuwang/demo_patrol_layout.json`。

跨机器访问（这条路才需要放开 bridge 本身）：

```bash
BRIDGE_ADDRESS=192.168.x.x robot/script/robot/foxglove_demo.sh live 120
```

验证 bridge 是否正常且只读：

```bash
python3 robot/script/robot/foxglove_ws_probe.py --expect /tf /odom /map /clock
```

网页模式下同一个探针可以走反向代理验证整条链路：

```bash
python3 robot/script/robot/foxglove_ws_probe.py --port 8080 --path /ws --expect /tf /odom
```

### 4.4 录一个 demo Bag

```bash
robot/script/robot/foxglove_demo.sh record --duration 120 --name indoor_patrol
```

产物在 `/home/dogrobot/runtime/nx-edge/data/rosbags/foxglove-demo/<时间戳>_<名字>/`，
MCAP + zstd。录制在临时目录完成、校验通过后才原子改名，中途失败会留下 `.partial-*` 供诊断。

校验包括：`metadata.yaml` 存在、消息数 > 0、15 个必需话题全部在场。
**录到空 Bag 会直接失败**，不会假装成功。

### 4.5 回放已有 Bag

MCAP 直接拖进 Foxglove（`File → Open local file`），用原生时间轴暂停/倍速/跳转。
故障分析建议 0.25×～0.5×。

旧的 SQLite3 Bag 先转换：

```bash
robot/script/robot/foxglove_demo.sh convert \
  runtime/nx-edge/data/rosbags/navigation/20260808_002131_smoke_test \
  /tmp/smoke_test-mcap
```

**必须转换，不能直接拖 `.db3`**：`.db3` 不携带消息定义，
`robots_dog_msgs/UniRtkPvh`、`localization/ScanMatchingStatus` 这类自定义类型在宿主里解不出来。
MCAP 会把 schema 内嵌进文件。

### 4.6 节点在环回放（改完参数重跑一遍）

```bash
robot/script/robot/foxglove_demo.sh play <bag_dir> 0.5
```

在 domain 77 上 `ros2 bag play --clock 100 --rate 0.5 --start-paused`，同时起一个
`use_sim_time:=true` 的 bridge。播放器默认暂停，在终端按空格开始。
需要一起跑的 UKF/Nav2 节点必须同样设 `use_sim_time:=true` 并加入 domain 77。

### 4.7 用公开数据集演示（不用真狗、不用假狗、本机不落盘）

Lichtblick 自带一个公开的 nuScenes 样例 Bag，宿主可以直接从 URL 流式读取，
本机不存任何文件：

```bash
robot/script/robot/foxglove_demo.sh web 600 &   # 只要网页宿主，不需要 bridge
# 浏览器打开（WEB_LAYOUT=docs/yuwang/nuscenes_demo_layout.json 起的服务）：
#   http://127.0.0.1:8080/?ds=remote-file&ds.url=<URL>
```

URL：`https://mcap-proxy.lichtblick.workers.dev/NuScenes-v1.0-mini-scene-sample.mcap`

三点必须先知道，否则会以为是布局坏了：

1. **它不是 rosbag2，`ros2 bag play` 播不了。** 该文件 `profile` 为空、`library` 是
   `nuscenes2mcap`，schema 全是 protobuf。只能由宿主自己作为 `remote-file` 数据源读。
2. **在 NX 上要先等大约 6 分钟才会出画，不是坏了。** 实测到该 URL 约 **0.86 MB/s**，
   而这个 Bag 是 488 MB / 19.2 秒仿真时间（约 25 MB 对应 1 秒 Bag 时间），
   实时播需要约 30 倍于此的带宽，所以宿主是边缓冲边等。点了播放之后面板会长时间停在
   `Waiting for messages…`，**这段时间不要以为是布局配错了** —— 判断方法很简单：
   panel 标题这时已经正确显示各自配置的话题了。缓冲够了以后 3D、两路相机、`/gps`、
   `/diagnostics` 会一起出来。带宽够的机器（操作员自己的电脑）不用等这么久。
3. 因此这个布局**不作为**数据链路的自动化验证手段 —— 六分钟的网络等待不该进测试。
   `foxglove_demo.sh verify` 只对它做离线的布局接受性检查，不联网、不下载。

本机实测到的效果（供对照）：3D 面板出 `/LIDAR_TOP` 点云（turbo 伪彩）、`/RADAR_FRONT`、
`/drivable_area` 可行驶区域和 3D 标注框；两路相机出真实街景**并叠加了 `annotations` 标注框**
（说明 `calibrationTopic` 和 `annotations` 都生效了）；`/gps` 出真实的
`foxglove.LocationFix`（新加坡经纬度）；诊断面板出 3 条 OK。

### 4.8 收尾

```bash
robot/script/robot/foxglove_demo.sh stop
```

只终止本脚本 PID 文件里记录的进程，**不按进程名匹配**，因此不会误杀生产节点。

## 5. 布局文件

| 文件 | 用途 | 主要话题 |
|---|---|---|
| `demo_patrol_layout.json` | 假机器狗 demo | `/map` `/scan` `/odom` `/patrol/trajectory` `/plan` `/goal_pose` `/camera/front/image/compressed` `/diagnostics` |
| `nav_debug_layout.json` | 真机蛇形/漂移诊断 | `/laser_scan` `/front_lidar` `/odom/localization_odom` `/odom/nav2` `/plan` `/transformed_global_plan` `/cmd_vel` `/cmd_vel_raw` `/status` `/localization/decision` |
| `nuscenes_demo_layout.json` | 公开数据集演示（见 4.7） | `/LIDAR_TOP` `/RADAR_FRONT` `/CAM_FRONT` `/CAM_BACK` `/pose` `/gps` `/drivable_area` `/semantic_map` |

导入方式：

- **网页模式不用导入**。`foxglove_web_serve.py` 在启动时把布局注入 `index.html` 的
  `LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER`（上游 Docker 镜像自己就是这么做的），
  打开页面就是加载好的状态。换布局用 `WEB_LAYOUT=...`，重启服务生效。
- 桌面客户端：左上角菜单 → `Import layout from file`。

注入发生在内存里，硬盘上的 `index.html` 不动 —— 否则会破坏包的校验和，而且第二次启动会重复注入。

### 改布局时最容易踩的坑：宿主不认的键会被静默丢弃

**Lichtblick 不会因为一个配置键不认识就报错。** 它照样把面板挂上，只是忽略那个键，
然后面板自己去自动选一个话题。结果就是「面板渲染正常、内容全是错的」，肉眼很难发现。

本项目实际踩过一次：Image 面板原本写的是 `"cameraTopic": "..."`（很多旧版 Foxglove 文档
是这么写的），1.28.1 里根本没有这个键 —— 整个 Image 配置在 `imageMode` 下面：

```json
"Image!democam": {
  "imageMode": {
    "imageTopic": "/camera/front/image/compressed",
    "calibrationTopic": "/camera/front/camera_info",
    "synchronize": false
  }
}
```

当时两个 Image 面板都悄悄回退去显示了 `/CAM_BACK_LEFT`，而"面板全部挂载"的检查照样通过。
所以 `verify` 现在**除了挂载还断言配置生效**：Image 面板必须真的显示它自己配的那个话题
（该面板即使没接数据源也会把话题名打在标题上，所以这条检查离线可跑）。
改任何布局的面板配置后都要重跑 `verify`，升级宿主版本后同样。

### 蛇形故障的判读（对应需求文档第三节）

- `nav_debug_layout.json` 右上 Plot 同时画 `/cmd_vel.angular.z` 和 `/cmd_vel_raw.angular.z`。
  两条线的差就是 `collision_monitor` 的干预量 —— 这是原需求文档没有的信息，
  但在本项目里是区分"控制器自己在震荡"和"被安全层反复截断"的关键。
- 3D 里同时画 `/odom/localization_odom`（红，定位输出）和 `/odom/nav2`（蓝，Nav2 实际消费的位姿）。
  两者不是同一条流：`tf_publisher` 会把 roll/pitch 压平后再转发，所以只看
  `/odom/localization_odom` 解释不了 Nav2 的行为。
- 轨迹本身抖 → 定位层；轨迹平滑但 `angular.z` 正负翻转 → 控制器参数。

## 6. 已验证 / 未验证

在本机（NX，Ubuntu 22.04 aarch64，ROS 2 Humble）实测通过：

| 项 | 结果 |
|---|---|
| `preflight` | 通过 |
| `record --duration 12` | 1712 条消息，15/15 话题齐全，MCAP+zstd，127 KiB |
| `live` + WS 探针 | 16 个 channel，capabilities 仅 `connectionGraph`（只读确认） |
| `play` + WS 探针 | 16 个 channel，capabilities `connectionGraph` + `time` |
| `convert` 真实 db3 | 2153 条消息进出一致，源库以 READ_ONLY 打开 |
| 进程清理 | `stop` 后无残留 bridge/player，8765 端口释放 |
| 三个布局 JSON | 结构与 panel id 引用自洽，无孤儿/悬空引用 |
| 网页宿主取包 | sha256 校验通过，解压 403 个文件、5 个 wasm 模块 |
| 静态托管 | `.wasm` → `application/wasm`；带哈希的资源 `immutable`；`index.html` `no-store` |
| 目录穿越 | 请求 `../../../etc/passwd` 返回 `index.html`，不是 `passwd` |
| `/ws` 反向代理 | 无 upgrade → 400；bridge 未起 → 502；探针经 `--port 8080 --path /ws` 结果与直连一致 |
| **三个布局的真实渲染** | **Lichtblick 1.28.1 + 真实 Chromium，三个布局各自声明的面板全部挂载，无错误卡片、无控制台错误** |
| **Image 面板配置生效** | **面板显示的是布局里配的话题本身，不是自动选的**（见 5 节的坑；此前 `cameraTopic` 被静默丢弃就是这条检查抓出来的） |
| 真实 WebGL | 3D 面板拿到真的 WebGL2 上下文（SwiftShader），不是降级的错误卡片 |
| 端到端数据链路 | 浏览器经 `ws://127.0.0.1:8091/ws` 连上后，3D 出 TF/激光/路径/里程计轨迹，Plot 出 5 条曲线，`/patrol/status` 出真实 JSON |

布局验证是**可重复的自动化检查**，不是肉眼看一眼：

```bash
robot/script/robot/foxglove_demo.sh verify     # 5 个用例
```

它在真实 Chromium 里分别加载三个布局，逐条断言：

1. JSON 里声明的每个 panel id 都真的挂载了（宿主静默丢弃面板是这里最可能出的问题）；
2. **每个 Image 面板显示的话题就是布局里配的那个** —— 「挂载了」不等于「配对了」，
   这条是补上第 5 节那个坑之后加的；
3. 没有错误卡片、没有相关的控制台报错；
4. WebGL2 上下文真的存在 —— 否则 3D 面板会以"渲染出一张错误卡片"的方式假装通过，
   整套检查就没意义了；
5. 另有一条纯静态用例，直接读 JSON 断言 Image 配置在 `imageMode` 下面、顶层没有
   `cameraTopic` 这类会被丢弃的键，把错误定位在文件里而不是浏览器里。

第 2、5 条都实测过"把 bug 放回去会失败"，不是只在正确输入上通过。

**仍未验证 / 已知限制**：

- `pose_error_user_script.ts` 未在宿主中执行过。
- `nav_debug_layout.json` 只验证了"面板全部渲染 + 配置生效"，没有用真机 MCAP 灌数据验证
  `/cmd_vel` vs `/cmd_vel_raw` 双线实际出图。要验的话：
  `WEB_LAYOUT=docs/yuwang/nav_debug_layout.json` 起网页模式，另开一个终端 `play` 一个真机 Bag。
- `nuscenes_demo_layout.json` 的数据链路**手工验过一次**（见 4.7 的实测效果），
  但**没有进 `verify`**：在 NX 上要等约 6 分钟缓冲，这种时长的网络等待不该放进自动化测试。
  自动化里它只做离线的布局接受性检查（面板挂载 + Image 话题正确）。
- Image 面板在假机器狗下是**纯黑**，这是对的：`/camera/front/image/compressed`
  发的是 1×1 的黑色 PNG 占位符（`_PNG_1X1`），面板正确解码并渲染了它，只是里面没内容。
  要让 demo 看起来更像回事，得让假机器狗发一张合成图，属于假机器狗的事，不是布局的事。
- 浏览器打不开 NX 上的 Bag 文件（浏览器只能读用户自己选的本地文件）。
  不需要 —— `foxglove_demo.sh play` 已经把 Bag 经 bridge 播成实时流，回放工作流是闭环的。

## 7. 相关文件

```
robot/src/tools/roamerx_patrol_demo/
├── launch/indoor_patrol_demo.launch.py    假机器狗
├── launch/foxglove_readonly.launch.py     只读 bridge
├── config/indoor_patrol.yaml              场景参数
├── config/bridge.yaml                     bridge 只读白名单
└── roamerx_patrol_demo/                   场景模型与发布器

robot/script/robot/
├── foxglove_demo.sh                       一键入口
├── foxglove_ws_probe.py                   bridge 连通性与只读性探针
├── fetch_lichtblick_web.sh                取网页宿主并校验 sha256
└── foxglove_web_serve.py                  静态托管 + 布局注入 + /ws 反向代理

platform/frontend/
├── playwright.foxglove.config.js          布局验证专用配置（独立于既有测试套件）
└── tests/foxglove/layout.spec.js          在真实浏览器里断言三个布局的挂载与配置生效

runtime/nx-edge/install/lichtblick-web/
├── lichtblick-web.lock                    version / url / sha256
└── README.md                              来源、离线重装、升级步骤

docs/yuwang/
├── FOXGLOVE_MVP_使用说明.md               本文
├── 需求文档勘误.md                        与两份 docx 的差异
├── nav_debug_layout.json                  真机诊断布局
├── demo_patrol_layout.json                demo 布局
├── nuscenes_demo_layout.json              公开数据集演示布局
└── pose_error_user_script.ts              位姿偏差脚本（已按真实 API 重写）
```

布局验证的测试放在 `platform/frontend` 下，是因为那是仓库里唯一已装好浏览器测试基础设施的
地方；用独立 config + 独立 `test:foxglove` 脚本把耦合限制到最小，既有 `npm test` 行为不变。

完整分阶段方案（含自定义 `.foxe` 面板、平台同源入口、性能基线）见
`docs/superpowers/plans/2026-08-28-foxglove-web-ros2-bag-debugging.md`。
