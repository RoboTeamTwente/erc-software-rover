"""Gazebo side only: world, rover, bridge, covariance node. No SLAM here.

Use sim_slam_launch.yaml to start this together with the rtt_slam stack.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rtt_gazebo_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    gz_extra_args = LaunchConfiguration('gz_extra_args')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                 os.path.join(pkg, 'urdf', 'rtt_rover.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'mars_yard.sdf')),
        DeclareLaunchArgument('gz_extra_args', default_value='',
                              description="extra gz sim args, e.g. '-s' for headless"),
        # spawn pose. z is above the ground on purpose, the rover drops onto the terrain.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.45'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        SetParameter(name='use_sim_time', value=True),

        # lets Gazebo resolve model://mars_yard
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r ', gz_extra_args, ' ', world]}.items()),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'rtt_rover',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}]),

        Node(package='rtt_gazebo_sim', executable='covariance_stamper.py', output='screen'),
    ])
