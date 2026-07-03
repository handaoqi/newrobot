import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取配置文件路径
    localization_dir = get_package_share_directory('localization')
    config_file = os.path.join(localization_dir, 'config', 'config.yaml')
    
    return LaunchDescription([
        # 启动定位节点
        Node(
            package='localization',
            executable='localization_node',
            name='localization',
            output='screen',
            parameters=[config_file]
        ),

        # Node(
        #     package='localization',
        #     executable='localization_node',
        #     name='localization_map_server',
        #     output='screen',
        #     parameters=[config_file]
        # )
        
        # 静态TF: 由出厂标定 /front_lidar R/T 取逆得到，base_link -> livox_frame
        Node(
            name='lidar_tf',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.382765605', '-0.046855740', '0.513445457',
                '0.007172121', '-0.043589169', '-0.009509268', '0.998978536',
                'base_link', 'livox_frame'
            ]
        )
    ])
