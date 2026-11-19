"""Gazebo side only: ERC 2026 world (from rtt_gz_worlds), rover, bridge, covariance node.
No SLAM here; use sim_slam_launch.yaml to start this together with the rtt_slam stack.

    ros2 launch rtt_gazebo_sim sim_launch.py                       # rover at S1, facing +Y
    ros2 launch rtt_gazebo_sim sim_launch.py start:=S4 yaw:=0.8    # another start point / heading
    ros2 launch rtt_gazebo_sim sim_launch.py markers:=true         # show the ERC point markers

`start` is any point name from rtt_gz_worlds/config/erc2026_points.yaml (normally S1..S9).
The rover is placed at that point's x, y in ERC yard coordinates (= Gazebo world frame),
a little above the simulated ground, and drops onto the terrain.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

# base_link sits at the wheel axle (wheel radius 0.12 m in the xacro) + 8 cm drop
SPAWN_HEIGHT_ABOVE_GROUND = 0.20


def launch_setup(context):
    pkg = get_package_share_directory('rtt_gazebo_sim')
    worlds = get_package_share_directory('rtt_gz_worlds')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    start = LaunchConfiguration('start').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    markers = LaunchConfiguration('markers').perform(context).lower() in ('true', '1')
    gz_extra_args = LaunchConfiguration('gz_extra_args').perform(context)

    with open(os.path.join(worlds, 'config', 'erc2026_points.yaml')) as fh:
        points = yaml.safe_load(fh)['points']
    if start not in points:
        raise RuntimeError(f"start:={start} is not in erc2026_points.yaml "
                           f"(known: {', '.join(sorted(points))})")
    p = points[start]
    x, y = p['x'], p['y']
    z = p['ground_z'] + SPAWN_HEIGHT_ABOVE_GROUND

    world = os.path.join(worlds, 'worlds', 'erc2026_markers.sdf' if markers else 'erc2026.sdf')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 os.path.join(pkg, 'urdf', 'rtt_rover.urdf.xacro')]),
        value_type=str)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {gz_extra_args} {world}'}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'rtt_rover',
                        '-x', str(x), '-y', str(y), '-z', f'{z:.3f}', '-Y', yaw]),

        # clock, cmd_vel, wheel odom, IMU, joint states, camera (RealSense topic names)
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}]),

        # /wheel/odom_raw -> /wheel/odom and /imu/data_raw -> /imu/data
        Node(package='rtt_gazebo_sim', executable='covariance_stamper.py', output='screen'),
    ]


def generate_launch_description():
    worlds = get_package_share_directory('rtt_gz_worlds')
    return LaunchDescription([
        DeclareLaunchArgument('start', default_value='S1',
                              description='ERC point to spawn at (S1..S9)'),
        DeclareLaunchArgument('yaw', default_value='1.5708',
                              description='heading in rad, ERC frame (1.5708 = facing +Y, into the yard)'),
        DeclareLaunchArgument('markers', default_value='false',
                              description='show the ERC point markers (cameras see them, keep false for SLAM)'),
        DeclareLaunchArgument('gz_extra_args', default_value='',
                              description="extra gz sim args, e.g. '-s' for headless"),

        SetParameter(name='use_sim_time', value=True),

        # also done by rtt_gz_worlds' environment hook; repeated so this works on its own
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(worlds, 'models')),

        OpaqueFunction(function=launch_setup),
    ])
