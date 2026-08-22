- Foxglove 应连接：ws://192.168.234.234:8765

  先在机器人电脑新开一个终端，执行：

  source /opt/ros/humble/setup.bash
  source /home/dogrobot/robot/install/setup.bash

  export ROS_DOMAIN_ID=24
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp

  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    address:=192.168.234.234

  source /opt/ros/humble/setup.bash
  source /home/dogrobot/robot/install/setup.bash

  export ROS_DOMAIN_ID=24
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp

  ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    address:=192.168.234.234

---

  定位实际使用、但当前因导航未启动而未出现的关键输入/输出是：

  - /front_lidar：原始点云，定位主输入
  - /front_lidar/imu：雷达 IMU，预测输入
  - /odom/lidar_odom：定位节点内部生成的激光短时运动预测输入
  - /fix：RTK/GNSS 位置输入
  - /rtk_pvh：RTK 双天线航向输入，定位节点订阅并参与 UKF 航向融合
  - /odom/localization_odom：定位算法输出里程计
  - /tf：运行时 map/odom/base_link 变换

  /odom/mc_odom 仅为控制器原始里程计，当前不被定位节点订阅，不参与定位、TF 或 Nav2 位姿链路。

  排查“定位丢失”时，足够且推荐录这 11 个：

  source /opt/ros/humble/setup.bash
  source /home/dogrobot/robot/install/setup.bash

  ros2 bag record -s mcap -o /home/dogrobot/runtime/nx-edge/data/patrol-data \
    /front_lidar \
    /front_lidar/imu \
    /odom/mc_odom \
    /fix \
    /rtk_pvh \
    /tf \
    /tf_static \
    /odom/localization_odom \
    /status \
    /localization_info \
    /rosout

  若问题发生时有人重定位或策略切换，再追加低带宽主题：

  /initialpose  /localization/policy  /localization/decision  /sensor_health

  /aligned_points 可用于直接可视化点云对地图的配准效果，但会显著增大包体，不是最小集必需项。录
  制时建议覆盖丢失前后各至少 30 秒，并同时记录当时的地图版本（map.pcd）和定位 config.yaml；否
  则离线分析无法确认是否由地图或参数变化引起。
