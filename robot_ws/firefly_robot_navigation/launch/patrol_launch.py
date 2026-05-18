#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_control',
            name='patrol_control',
            output='screen',
            parameters=[
                {'linear_velocity': 0.2},
                {'angular_velocity': 0.5},
                {'update_rate': 10.0}
            ]
        )
    ])
