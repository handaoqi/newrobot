import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('robot_slam')
    slam_params_file = LaunchConfiguration('slam_params_file')
    scene_scope = LaunchConfiguration('scene_scope')
    max_range = ParameterValue(
        PythonExpression(["25.0 if '", scene_scope, "' in ['outdoor', 'transition'] else 10.0"]),
        value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.environ.get(
                'ROAMERX_SLAM_PARAMS',
                os.path.join(package_share, 'config', 'config.yaml'))),
        DeclareLaunchArgument(
            'scene_scope',
            default_value=os.environ.get('ROAMERX_NAVIGATION_SCENE_SCOPE', 'indoor'),
            choices=['indoor', 'transition', 'outdoor']),
        Node(
            package='robot_slam',
            executable='mapping',
            name='lio_odometry',
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'frontend.odometry_only': True,
                    'frontend.odometry_topic': '/odom/lio_odom',
                    'preprocess.fov_degree': 360.0,
                    'preprocess.max_range': max_range,
                    'keyframe_record.enable': False,
                    'global_optimization.enable': False,
                    'gnss_fusion.enable': False,
                    'publish.path_en': False,
                    'publish.map_en': False,
                    'publish.world_points_en': False,
                    'publish.body_points_en': False,
                },
            ],
        ),
    ])
