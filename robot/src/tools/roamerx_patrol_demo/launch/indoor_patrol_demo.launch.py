"""Start the deterministic fake robot dog.

The node publishes a fixed 120 s indoor patrol on standard ROS 2 message types
so Foxglove, rosbag recording and the navigation debug layout can all be
exercised without the physical robot.

Isolation is deliberate and doubled: ROS_DOMAIN_ID 77 and rmw_fastrtps_cpp.
Production runs on domain 24 over rmw_zenoh_cpp, and two different RMW
implementations cannot discover each other at all, so a mistake in the domain
id alone can never leak demo topics onto the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


# Empty string means "keep whatever indoor_patrol.yaml says", so the launch
# file never silently re-specifies a default that drifts from the config file.
_OVERRIDABLE = (
    ("duration_sec", float),
    ("publish_rate_hz", float),
    ("linear_speed_mps", float),
    ("topic_namespace", str),
    ("waypoints", str),
    ("seed", int),
)


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("roamerx_patrol_demo")
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(share, "config", "indoor_patrol.yaml")
    if not os.path.isfile(params_file):
        raise RuntimeError(f"params_file does not exist: {params_file}")

    overrides = {}
    for name, caster in _OVERRIDABLE:
        raw = LaunchConfiguration(name).perform(context).strip()
        if not raw:
            continue
        try:
            overrides[name] = caster(raw)
        except ValueError as exc:
            raise RuntimeError(f"launch argument {name}:={raw!r} is not a valid {caster.__name__}") from exc

    return [
        Node(
            package="roamerx_patrol_demo",
            executable="patrol_publisher",
            name="roamerx_patrol_demo",
            output="screen",
            emulate_tty=True,
            parameters=[params_file, overrides] if overrides else [params_file],
        )
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description="Override the packaged config/indoor_patrol.yaml",
        )
    ]
    arguments += [
        DeclareLaunchArgument(name, default_value="", description=f"Override {name} from the params file")
        for name, _ in _OVERRIDABLE
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            *arguments,
            OpaqueFunction(function=_launch_setup),
        ]
    )
