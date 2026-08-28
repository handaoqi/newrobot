import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取配置文件路径
    localization_dir = get_package_share_directory('localization')
    config_file = os.path.join(localization_dir, 'config', 'config.yaml')
    slam_dir = get_package_share_directory('robot_slam')

    return LaunchDescription([
        Node(
            package='localization',
            executable='localization_node',
            name='localization',
            output='screen',
            parameters=[config_file]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_dir, 'launch', 'lio_odometry.launch.py')),
        ),

        # Node(
        #     package='localization',
        #     executable='localization_node',
        #     name='localization_map_server',
        #     output='screen',
        #     parameters=[config_file]
        # )
    ])
