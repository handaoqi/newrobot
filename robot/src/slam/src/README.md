# robot_slam

## 描述（Description）

​	A SLAM package integrating multi-sensor data for simultaneous localization and mapping



## 依赖（Depend）

​	Cmake > 3.10

​	G++/GCC > 11.4.0

​	ROS == Humble

​	PCL == 1.12.1

​	OpenCV == 4.5.4

​	Eigen == 3.4.0

​	zsibot_interface



## 构建（Build）

### 单包构建（Single Build）

```c++
cp -r robot_slam 工作空间/src

cd 工作空间

colcon build --packages-select robot_slam
```

### 工作空间构建（ros2_workspace Build）

```C++
cd ros2_worksapce

colcon build --packages-select robot_slam
```



## 运行（Run）

真机不要用 `ros2 launch robot_slam slam.launch.py` 或 `./run_slam`。NX 入口：

```
cd /home/dogrobot/robot
script/robot/start_mapping_real.sh start
```

桌面调试若必须用 launch，保持 `use_rviz:=false`，除非本机有显示器。

## 开始建图（Run Mapping）

```
ros2 service call /slam/start_mapping std_srvs/srv/Trigger
```

旧接口 `/slam_state_service` `data: 3` 仍可用。输入话题为 `/front_lidar` 和 `/front_lidar/imu`。

## 保存建图（Save Map）

```
ros2 service call /slam/save_map std_srvs/srv/Trigger
```

地图默认保存在 `/home/dogrobot/runtime/nx-edge/data/jszr/map/`（`storage.data_path`）。包含 pcd、pgm、yaml、轨迹 txt 和 keyframes。



## 参数说明（Command Line Arguments）

#### RosWrapper Paramters

| 参数名称                      |      数据类型      |     默认值         | 参数说明                       |
| :---------------------------: | :-------------: | :-------------: | :----------------------------: |
| --feature_extract_enable | bool | false | 点云处理模块是否提取特征 |
| --point_filter_num | int   | 2 | 点云处理模块的采样数目     |
| --max_iteration | int   | 3 | 匹配迭代次数 |
| --filter_size_surf |   double   |      0.2      |                原始点云降采样体素边长                |
|     --filter_size_map     |  double  |               0.2               |                 地图点云降采样体素边长                 |
| --cube_side_length |   double   | 1000.0 | 地图裁剪立方体边长 |
| --common.lid_topic |   str    | "/livox/lidar" |                订阅雷达话题（默认Qos为best_effort）                |
|   --common.imu_topic   |   str    |     "/livox/imu"     | 订阅imu话题（默认Qos为best_effort） |
|    --common.time_sync_en    |   bool   |              false              | 是否进行时间同步 |
| --common.time_offset_lidar_to_imu |   double   |     0.0     | 雷达数据落后imu时间 |
|       --preprocess.lidar_type       |   int   |      1       |  雷达类型默认为1（livox: mid360）  |
|      --preprocess.scan_line      |   int   | 4 | 雷达线束数量 |
| --preprocess.blind | double | 0.5 | 雷达盲区半径 |
| --preprocess.timestamp_unit | int | 3 | 时间单位量级 |
| --preprocess.scan_rate | int | 10 | 雷达扫描频率 |
| --mapping.acc_cov | double | 0.1 | imu加速度噪声 |
| --mapping.gyr_cov | double | 0.1 | imu陀螺仪噪声 |
| --mapping.b_acc_cov | double | 0.0001 | imu加速度偏置噪声 |
| --mapping.b_gyr_cov | double | 0.0001 | imu陀螺仪偏置噪声 |
| --mapping.fov_degree | double | 360.0 | 视角度数 |
| --mapping.det_range | float | 100.0 | 范围阈值 |
| --mapping.extrinsic_est_en | bool | false | 是否优化雷达与imu的外参 |
| --mapping.extrinsic_T | vetor | [ -0.011, -0.02329, 0.04412 ] | lidar2imu平移 |
| --mapping.extrinsic_R | vetor | [ 1., 0., 0.,0.,1.,0.,0.,0.,1.] | lidar2imu旋转 |
| --publish.path_en | bool | true | 是否发布路径 |
| --publish.map_en | bool | false | 是否发布地图点云 |
| --publish.world_points_en | bool | true | 是否发布map坐标系下的点云 |
| --publish.body_points_en | bool | true | 是否发布body坐标系下的点云 |
| --pcd2pgm.file_name | str | "map" | 2D地图前缀名字 |
| --pcd2pgm.thre_z_min | double | 0.1 | 提取有效点云的最小z值 |
| --pcd2pgm.thre_z_max | double | 2.0 | 提取有效点云的最大z值 |
| --pcd2pgm.flag_pass_through | int | 0 | 统计滤波提取内点标志位 |
| --pcd2pgm.map_resolution | double | 0.05 | 地图分辨率 |

`注：发布map坐标系点云话题为: /world_points，qos为best_effort；发布body坐标系下点云话题为：/body_points，qos为reliable；发布路径话题为: /path，qos为reliable；发布odomtery话题为: /slam_odom，qos为reliable, tf随odom信息一起发布`

## GTSAM 多帧全局优化

保存地图时会对已写入的关键帧执行一次批量因子图优化；因子包括连续 NDT/LIO 位姿、关键帧 IMU 预积分、质量门控后的 RTK XY、RTK 双天线航向，以及可选回环约束。优化结果写入：

- `trajectory_raw.csv`：原始增量轨迹
- `trajectory_optimized.csv`：GTSAM 全局优化轨迹
- `trajectory_covariance.json`：每个关键帧的全局 6x6 边缘协方差，顺序为 `rotation_xyz,translation_xyz`
- `map_manifest.json`：因子数量、回环数量和轨迹来源

可通过以下服务在当前会话中再次执行历史轨迹重优化：

```bash
ros2 service call /slam/global_optimize std_srvs/srv/Trigger
```

Edge 的 Scan-Context 会先生成 `scan_context/loop_candidates.csv`。地图包完成阶段仅保留 `accepted=true` 且 `geometric_verified=true` 的候选；适配器随后调用 `/slam/global_optimize`，C++ 自动转换并写入当前地图目录的 `loop_closures.csv`，格式为：

```text
from,to,tx,ty,tz,qx,qy,qz,qw,sigma_translation,sigma_rotation,score
```

其中相对位姿满足 `T_from_to`，四元数为 `xyzw`。节点会将这些约束与 RTK、NDT、IMU 放入同一图中，并使用 Huber 鲁棒核抑制 RTK 异常和错误回环。GTSAM 成功后会重新变换所有关键帧点云并重建 `map.pcd`/栅格；`map_raw.pcd` 始终保留为原始地图。运行期输出话题为 `/slam/global_optimized_path`、`/slam/global_optimized_odom` 和 `/slam/global_optimization_status`。
