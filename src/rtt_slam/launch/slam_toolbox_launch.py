from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    slam_toolbox = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
        ),
        launch_arguments=[
            ("base_frame", "camera_link"),
        ],
    )

    realsense_camera = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
        ),
        launch_arguments=[
            ("enable_gyro", "True"),
            ("enable_accel", "True"),
            ("unite_imu_method", "2"),
        ],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        parameters=[
            {
                "use_mag": False,
            }
        ],
        remappings=[
            ("/imu/data_raw", "/camera/camera/imu"),
        ],
    )

    imu_to_tf = Node(
        package="rtt_imu_to_tf",
        executable="imu_to_tf",
    )

    depthimage_to_laserscan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        remappings=[
            ("/depth", "/camera/camera/depth/image_rect_raw"),
            ("/depth_camera_info", "/camera/camera/depth/camera_info"),
        ],
    )

    return LaunchDescription(
        [
            depthimage_to_laserscan,
            imu_filter,
            imu_to_tf,
            realsense_camera,
            slam_toolbox,
        ]
    )
