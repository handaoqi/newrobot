# RoamerX Lite 导航栈启动命令文档

## 系统架构

```
NX板 (192.168.234.234)                    Firefly RK3588 (192.168.234.1)
┌──────────────────────────┐              ┌──────────────────────────┐
│ 导航栈 (robot_navigo)     │              │ 运动控制 (mc_ctrl)        │
│ ┌──────────────────────┐ │              │                          │
│ │ bt_navigator         │ │   LCM/UDP   │                          │
│ │ planner_server       │ │ ◄────────── │  spline_daemon           │
│ │ controller_server    │ │  /cmd_vel   │                          │
│ │ vel_cmd_lcm_pub      │ │             │                          │
│ └──────────────────────┘ │             │                          │
│                          │             │                          │
│ 定位 (localization)      │             │                          │
│ ┌──────────────────────┐ │             │                          │
│ │ NDT 点云匹配          │ │             │                          │
│ │ TF: map→odom         │ │             │                          │
│ └──────────────────────┘ │             │                          │
│                          │             │                          │
│ 驱动                     │             │                          │
│ ┌──────────────────────┐ │   /odom/    │                          │
│ │ livox_driver         │ │ ◄────────── │  ecal2ros2 (里程计桥接)    │
│ │ pointcloud_to_       │ │   mc_odom   │                          │
│ │   laserscan          │ │             │                          │
│ └──────────────────────┘ │             │                          │
└──────────────────────────┘              └──────────────────────────┘
```

**通信协议：**
- NX → Firefly 速度指令：LCM 组播 `239.255.76.67:7667`
- Firefly → NX 里程计：ROS2 topic `/odom/mc_odom`（通过 ecal2ros2 桥接）

**TF 树：**
```
map → odom → base_link → livox_frame
      ↑ 定位发布       ↑ 静态TF(雷达在狗中心上方10cm、前方20cm)
```

---

## 零、前置条件

### 1. SSH 登录 NX 板
```bash
ssh robot@192.168.234.234
# 密码: robot
```

### 2. 设置 ROS2 环境变量（每次新开终端都要执行）
```bash
# 加载 ROS2 Humble 基础环境
source /opt/ros/humble/setup.bash

# 加载 RoamerX Lite 编译产物
source ~/genisom_roamerx_open/install/setup.bash
```

---

## 一、启动底层驱动（已配置自启动，通常无需手动操作）

### 1.1 Livox 激光雷达驱动
```bash
# 激光雷达驱动 - 发布 /front_lidar 点云话题
# 驱动目录: /opt/robot-driver/install/livox_driver
source /opt/robot-driver/install/setup.bash
ros2 launch livox_driver lidar.launch.py
```
> **说明：** 激光雷达为 Livox Mid-360，发布原始点云到 `/front_lidar` 话题。

### 1.2 点云转激光扫描
```bash
# 将 3D 点云转换为 2D 激光扫描 - 发布 /scan 话题
# 导航避障使用 /scan 话题数据
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r cloud_in:=/front_lidar -r scan:=/scan
```

### 1.3 ecal2ros2 桥接（Firefly 端）
```bash
# 此节点运行在 Firefly RK3588 板 (192.168.234.1) 上
# 作用：将 mc_ctrl 的里程计数据通过 ROS2 话题发布出来
# 发布 /odom/mc_odom 话题（包含 odom→base_link 变换）
ssh firefly@192.168.234.1
# 然后在 Firefly 上启动 ecal2ros2 桥接
```

---

## 二、启动物流节点

```bash
# 加载 RoamerX Lite 环境
source /opt/ros/humble/setup.bash
source ~/genisom_roamerx_open/install/setup.bash

# 启动定位节点
# 功能：
#   - NDT (Normal Distributions Transform) 点云匹配定位
#   - 发布 TF: map → odom 变换
#   - 发布里程计话题 /odom/localization_odom
#   - 发布静态 TF: base_link → livox_frame（雷达安装位置）
ros2 launch localization localization.launch.py platform:=NX_XG3588
```
> **定位原理：** 使用预先构建的全局地图点云（`/home/robot/.jszr/map/map.pcd`）与实时激光点云进行 NDT 匹配，计算机器人在地图中的位姿，并通过 TF 发布 `map→odom` 变换。

### 定位配置文件
```
~/genisom_roamerx_open/src/localization/localization/config/config.yaml
```
关键参数：
- `send_tf_transforms: true` — 启用 TF 发布
- `reg_method: "NDT_OMP"` — 使用 NDT 点云匹配算法
- `points_topic: "/front_lidar"` — 激光雷达点云话题

---

## 三、启动导航栈

```bash
# 加载 RoamerX Lite 环境
source /opt/ros/humble/setup.bash
source ~/genisom_roamerx_open/install/setup.bash

# 启动导航
# 参数说明：
#   platform:=NX_XG3588           — 目标平台标识
#   mc_controller_type:=RL_TRACK_VELOCITY  — 控制器类型（速度跟踪）
#   communication_type:=LCM       — 通信方式（LCM 组播）
#   map:=/home/robot/.jszr/map/map.yaml  — 地图文件路径
ros2 launch robot_navigo navigation_bringup.launch.py \
  platform:=NX_XG3588 \
  mc_controller_type:=RL_TRACK_VELOCITY \
  communication_type:=LCM \
  map:=/home/robot/.jszr/map/map.yaml
```
> **导航栈包含的节点：**
> - `map_server` — 加载并发布栅格地图 `/map`
> - `planner_server` — 全局路径规划（A* / NavFn）
> - `controller_server` — 局部路径跟踪（MPPI 控制器）
> - `behavior_server` — 行为树（恢复行为：旋转、后退等）
> - `bt_navigator` — 行为树导航器
> - `velocity_optimizer` — 速度优化
> - `waypoint_follower` — 航点跟随
> - `vel_cmd_lcm_pub` — 将 `/cmd_vel` 通过 LCM 发送给 Firefly
> - `lifecycle_manager_navigation` — 生命周期管理器

### 导航配置文件
```
~/genisom_roamerx_open/install/robot_navigo/share/robot_navigo/params/navigo_params.yaml
```
关键参数：
- `footprint: "[[0.35,0.25],[0.35,-0.25],[-0.35,-0.25],[-0.35,0.25]]"` — 机器人足迹 70cm×50cm
- `obstacle_min_range: 0.15` — 最近障碍物检测距离（米）

---

## 四、可视化工具

### 4.1 启动 RViz（需要 Windows 端运行 VcXsrv）
```bash
# 在 Windows 上先启动 VcXsrv（XLaunch），配置：
#   - Multiple windows
#   - Display number: 0
#   - Start no client
#   - 勾选 Disable access control

# SSH 登录时开启 X11 转发
ssh -X robot@192.168.234.234

# 启动 RViz
source /opt/ros/humble/setup.bash
source ~/genisom_roamerx_open/install/setup.bash
ros2 run rviz2 rviz2 -d ~/genisom_roamerx_open/install/robot_navigo/share/robot_navigo/rviz/rviz2_config.rviz
```
> **RViz 操作：**
> - 点击工具栏 **"2D Goal Pose"**（绿色箭头）设置导航目标
> - 鼠标左键拖动平移视角，滚轮缩放
> - 点击 **"Fit to View"** 按钮居中显示地图

### 4.2 命令行发送导航目标
```bash
# 发送导航目标（坐标 x: 1.0m, y: 0.0m, 朝向不变）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: { frame_id: 'map' },
    pose: {
      position: { x: 1.0, y: 0.0, z: 0.0 },
      orientation: { z: 0.0, w: 1.0 }
    }
  }
}"
```

---

## 五、建图流程

### 5.1 启动建图
```bash
source /opt/ros/humble/setup.bash
source ~/genisom_roamerx_open/install/setup.bash

# 启动 SLAM 建图
ros2 launch robot_slam slam.launch.py platform:=NX_XG3588
```

### 5.2 开始记录数据
```bash
# 通过服务调用开始建图（data: 3 = 开始建图）
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 3}"
```

### 5.3 保存地图
```bash
# 地图自动保存到 /home/robot/.jszr/map/
# 通过服务调用保存地图（data: 5 = 保存地图）
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 5}"
```

### 5.4 地图文件说明
```
/home/robot/.jszr/map/
├── map.pcd         # 全局点云地图（约 170MB，用于 NDT 定位）
├── map.pgm         # 栅格地图（用于导航路径规划）
├── map.yaml        # 栅格地图配置文件
└── map_preview.png # 地图预览图
```

---

## 六、快速启动脚本（一次启动全部）

```bash
#!/bin/bash
# 文件: ~/start_nav_all.sh
# 一键启动：定位 + 导航 + RViz

# 1. 加载环境
source /opt/ros/humble/setup.bash
source ~/genisom_roamerx_open/install/setup.bash

# 2. 启动定位（后台运行）
ros2 launch localization localization.launch.py platform:=NX_XG3588 &
LOC_PID=$!
echo "定位已启动 PID=$LOC_PID"

# 3. 等待定位就绪
sleep 5

# 4. 启动导航（后台运行）
ros2 launch robot_navigo navigation_bringup.launch.py \
  platform:=NX_XG3588 \
  mc_controller_type:=RL_TRACK_VELOCITY \
  communication_type:=LCM \
  map:=/home/robot/.jszr/map/map.yaml &
NAV_PID=$!
echo "导航已启动 PID=$NAV_PID"

# 5. 等待导航就绪
sleep 10

# 6. 启动 RViz（前台运行，便于查看日志）
ros2 run rviz2 rviz2 -d ~/genisom_roamerx_open/install/robot_navigo/share/robot_navigo/rviz/rviz2_config.rviz

# 7. RViz 关闭后清理后台进程
kill $LOC_PID $NAV_PID 2>/dev/null
echo "所有进程已停止"
```

使用方法：
```bash
chmod +x ~/start_nav_all.sh
~/start_nav_all.sh
```

---

## 七、常用调试命令

### 7.1 查看运行状态
```bash
source /opt/ros/humble/setup.bash

# 查看所有 ROS2 节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看地图话题发布情况
ros2 topic info /map

# 查看 TF 树结构
ros2 run tf2_tools view_frames
# 生成 frames.pdf，里面有 TF 树结构

# 查看特定 TF 变换
ros2 run tf2_ros tf2_echo map base_link
```

### 7.2 查看日志
```bash
# ROS2 日志目录
ls ~/.ros/log/

# 实时查看导航节点日志
ros2 topic echo /rosout | grep navigo

# 检查系统时间（TF 时间戳同步需要正确的时间）
date
```

### 7.3 进程管理
```bash
# 查看导航相关进程
ps aux | grep -E "navigo|localization|rviz" | grep -v grep

# 停止所有导航相关进程
pkill -9 -f "navigation_bringup"
pkill -9 -f "localization"
pkill -9 -f "rviz2"

# 停止全部（驱动除外）
pkill -9 -f "vel_cmd_lcm_pub"
pkill -9 -f "mode_status_pub"
pkill -9 -f "component_container_isolated"
```

### 7.4 检查地图文件
```bash
# 确认地图文件存在
ls -la /home/robot/.jszr/map/

# 查看地图配置
cat /home/robot/.jszr/map/map.yaml
# 应包含: image: /home/robot/.jszr/map/map.pgm（绝对路径）
```

---

## 八、常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| RViz 无地图显示 | map_server 未启动或地图路径错误 | 检查 `/map` 话题是否有发布者 |
| TF 变换超时 | 定位未启动或 TF 链断裂 | 运行 `view_frames` 检查 TF 树 |
| 狗不移动 | LCM 通信故障 / Firefly 未就绪 | 检查 vel_cmd_lcm_pub 是否运行 |
| 系统时间不对 | NTP 未同步 | `sudo date -s "YYYY-MM-DD HH:MM:SS"` |
| 重复节点警告 | 多次启动导致节点冲突 | 先清理所有进程再启动 |
