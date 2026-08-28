import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('robot_slam')
    mapping_type = LaunchConfiguration('mapping_type')
    slam_params_file = LaunchConfiguration('slam_params_file')
    origin_params_file = LaunchConfiguration('origin_params_file')
    auto_loop_optimization = LaunchConfiguration('auto_loop_optimization')
    max_range = ParameterValue(
        PythonExpression(["25.0 if '", mapping_type, "' == 'outdoor' else 10.0"]),
        value_type=float)
    use_loop = ParameterValue(
        PythonExpression(["'", auto_loop_optimization, "'.lower() == 'true'"]),
        value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument(
            'mapping_type', default_value=os.environ.get('ROAMERX_MAPPING_TYPE', 'indoor'),
            choices=['indoor', 'outdoor']),
        DeclareLaunchArgument(
            'slam_params_file', default_value=os.environ.get(
                'ROAMERX_SLAM_PARAMS', os.path.join(package_share, 'config', 'config.yaml'))),
        DeclareLaunchArgument(
            'origin_params_file', default_value=os.environ.get(
                'ROAMERX_MAPPING_ORIGIN_PARAMS', '/dev/null')),
        DeclareLaunchArgument(
            'auto_loop_optimization', default_value=os.environ.get(
                'ROAMERX_AUTO_LOOP_OPTIMIZATION', 'false'),
            choices=['true', 'false']),
        Node(
            package='robot_slam', executable='slam_enu_converter', name='slam_enu_converter',
            parameters=[origin_params_file],
            condition=IfCondition(PythonExpression(["'", mapping_type, "' == 'outdoor'"]))),
        Node(
            package='robot_slam', executable='mapping', name='mapping',
            parameters=[slam_params_file, {
                'preprocess.max_range': max_range,
                'preprocess.fov_degree': 240.0,
                'global_optimization.use_loop': use_loop,
            }]),
    ])
