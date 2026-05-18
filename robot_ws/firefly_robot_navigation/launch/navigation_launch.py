#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('robot_navigation_core')
    
    return LaunchDescription([
        Node(
            package='robot_navigation_core',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),
                ('/scan', '/scan_filtered'),
                ('/odom', '/odom_filtered')
            ]
        )
    ])
