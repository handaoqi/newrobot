# 机器狗自主巡检导航系统

## 项目概述
本项目为ZSL-1A钢镚导航套件开发自主巡检导航功能，基于ROS2 Humble框架。

## 系统架构
- **RK3588控制器**: 基础运控系统 (192.168.234.1)
- **Orin NX算力板**: 导航算法处理 (192.168.234.234)
- **传感器**: 激光雷达、深度相机、IMU等

## 开发环境
- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS2 Humble
- **编程语言**: C++ 和 Python
- **工作空间**: ~/robot_navigation/

## 项目结构
`
robot_navigation/
├── src/                    # 源代码包
│   ├── robot_navigation_core/  # C++导航核心包
│   └── robot_patrol/          # Python巡检控制包
├── launch/                 # 启动文件
├── scripts/               # 脚本文件
├── config/                # 配置文件
├── maps/                  # 地图文件
└── install/               # 编译输出
`

## 功能模块

### 1. 导航核心 (robot_navigation_core)
- **navigation_node.cpp**: 基础导航节点
- **sensor_test_node.cpp**: 传感器测试节点
- 功能: 避障、路径规划、运动控制

### 2. 巡检控制 (robot_patrol)
- **patrol_control.py**: 巡检路径控制
- 功能: 多点巡检、路径规划、状态监控

### 3. 建图功能
- **simple_mapping.py**: 简单建图脚本
- 功能: 自动建图、路径记录

## 使用方法

### 1. 环境设置
`ash
cd ~/robot_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
`

### 2. 运行导航节点
`ash
ros2 run robot_navigation_core navigation_node
`

### 3. 运行巡检控制
`ash
ros2 run robot_patrol patrol_control
`

### 4. 运行建图功能
`ash
python3 scripts/simple_mapping.py
`

### 5. 启动完整系统
`ash
ros2 launch robot_navigation patrol_launch.py
`

## 当前状态
- ✅ SSH密钥配置完成
- ✅ 基础ROS2工作空间建立
- ✅ 导航核心模块编译成功
- ✅ 巡检控制模块开发完成
- ✅ 建图脚本开发完成
- ⏳ 传感器数据集成待完善
- ⏳ Orin NX连接待调试
- ⏳ SLAM算法待集成

## 下一步计划
1. 完善传感器数据采集
2. 集成SLAM建图算法
3. 实现精确定位功能
4. 开发Web监控界面
5. 系统集成测试

## 技术挑战
1. 双控制器通信协调
2. 传感器数据融合
3. 实时性能优化
4. 环境适应性提升

## 作者信息
- 开发者: AI Assistant
- 项目时间: 2026年5月
- 版本: v0.1.0
