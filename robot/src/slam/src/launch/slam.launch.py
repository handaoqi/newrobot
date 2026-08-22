import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameter_config = os.path.join(
        get_package_share_directory('robot_slam'), 'config', 'config.yaml')
    default_rviz_config_path = os.path.join(
        get_package_share_directory('robot_slam'), 'rviz', 'mapping.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Desktop visualization only. Leave false on the NX robot.',
        ),
        LogInfo(msg=[
            'robot_slam/slam.launch.py is a desktop helper. ',
            'On the real robot use /home/dogrobot/robot/script/robot/start_mapping_real.sh',
        ]),
        Node(
            package='robot_slam',
            executable='mapping',
            parameters=[parameter_config],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path],
            condition=IfCondition(use_rviz),
        ),
    ])
