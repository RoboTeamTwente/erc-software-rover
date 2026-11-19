"""Open the ERC 2026 Mars Yard in Gazebo Harmonic.

    ros2 launch rtt_gz_worlds erc2026.launch.py               # with point markers
    ros2 launch rtt_gz_worlds erc2026.launch.py markers:=false # clean world (use for SLAM)

No rover is spawned yet; that comes with the Gazebo rover model.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = get_package_share_directory("rtt_gz_worlds")
    markers = LaunchConfiguration("markers").perform(context).lower() in ("true", "1")
    world = os.path.join(share, "worlds", "erc2026_markers.sdf" if markers else "erc2026.sdf")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )
    return [gz_sim]


def generate_launch_description():
    share = get_package_share_directory("rtt_gz_worlds")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "markers",
                default_value="true",
                description="Show the official ERC points as coloured poles "
                "(cameras see them, so turn off for SLAM).",
            ),
            # also set by the package's environment hook; repeated so the launch works on its own
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
            OpaqueFunction(function=launch_setup),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
        ]
    )
