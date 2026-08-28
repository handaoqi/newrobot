"""Start foxglove_bridge with every write capability removed.

Defaults match indoor_patrol_demo.launch.py (domain 77, rmw_fastrtps_cpp) so
`foxglove_demo.sh live` needs no environment juggling.  To watch the real robot
instead, pass domain_id:=24 rmw:=rmw_zenoh_cpp -- the read-only capability set
in config/bridge.yaml is what makes that safe, and it is not overridable from
here on purpose.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("roamerx_patrol_demo")
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(share, "config", "bridge.yaml")
    if not os.path.isfile(params_file):
        raise RuntimeError(f"params_file does not exist: {params_file}")

    address = LaunchConfiguration("address").perform(context).strip()
    port = LaunchConfiguration("port").perform(context).strip()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    if use_sim_time not in ("true", "false"):
        raise RuntimeError(f"use_sim_time must be true or false, got {use_sim_time!r}")
    try:
        port_value = int(port)
    except ValueError as exc:
        raise RuntimeError(f"port:={port!r} is not an integer") from exc
    if not 1 <= port_value <= 65535:
        raise RuntimeError(f"port:={port_value} is outside 1..65535")

    return [
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    "address": address,
                    "port": port_value,
                    "use_sim_time": use_sim_time == "true",
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("domain_id", default_value="77", description="ROS_DOMAIN_ID to bridge"),
            DeclareLaunchArgument("rmw", default_value="rmw_fastrtps_cpp", description="RMW_IMPLEMENTATION to bridge"),
            DeclareLaunchArgument("address", default_value="127.0.0.1", description="Bind address; use the LAN IP to reach the bridge from another machine"),
            DeclareLaunchArgument("port", default_value="8765", description="WebSocket port"),
            # True only when a bag is being replayed with --clock, otherwise the
            # bridge waits forever for a /clock that nobody publishes.
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Set true when replaying with ros2 bag play --clock"),
            DeclareLaunchArgument("params_file", default_value="", description="Override the packaged config/bridge.yaml"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id")),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", LaunchConfiguration("rmw")),
            OpaqueFunction(function=_launch_setup),
        ]
    )
