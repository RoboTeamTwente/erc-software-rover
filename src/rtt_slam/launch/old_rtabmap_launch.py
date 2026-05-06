import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


nav2_params = PathJoinSubstitution(
    [FindPackageShare("rtt_slam"), "config", "old_rtabmap_nav2_params.yaml"]
)

new_params = PathJoinSubstitution(
    [FindPackageShare("rtt_slam"), "config", "new_params.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            # Delete database of old map before beginning
            ExecuteProcess(
                cmd=["rm", "-f", os.path.expanduser("~/.ros/rtabmap.db")],
                output="screen",
            ),
            # Static transform between base_link and camera_link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_tf",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    # Launch realsense2_camera with IMU and depth enabled
                    GroupAction(
                        actions=[
                            IncludeLaunchDescription(
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("realsense2_camera"),
                                        "launch",
                                        "rs_launch.py",
                                    ]
                                ),
                                launch_arguments={
                                    "enable_gyro": "True",
                                    "enable_accel": "True",
                                    "unite_imu_method": "2",
                                    "align_depth.enable": "True",
                                    "enable_rgbd": "True",
                                    "pointcloud.enable": "True",
                                    "enable_depth": "True",
                                    "enable_color": "True",
                                    "publish_tf": "True",
                                    "enable_sync": "True",
                                    "rgb_camera.color_profile": "424,240,30",
                                    "depth_module.depth_profile": "424,240,30",
                                }.items(),
                            )
                        ]
                    ),
                    # IMU filter - fuses gyro + accel into /imu/data
                    Node(
                        package="imu_filter_madgwick",
                        executable="imu_filter_madgwick_node",
                        name="imu_filter_madgwick",
                        output="screen",
                        parameters=[
                            {
                                "use_mag": False,
                                "publish_tf": False,
                                "world_frame": "enu",
                            }
                        ],
                        remappings=[
                            ("/imu/data_raw", "/camera/camera/imu"),
                            ("/imu/data", "/imu/data"),
                        ],
                    ),
                    # Visual-Inertial Odometry node
                    # Publishes: odom -> base_link TF and /odom topic
                    Node(
                        package="rtabmap_odom",
                        executable="rgbd_odometry",
                        name="rtabmap_odom",
                        output="screen",
                        parameters=[
                            {
                                "frame_id": "base_link",
                                "approx_sync": True,
                                "approx_sync_max_interval": 0.02,
                                "subscribe_imu": True,
                                "use_sim_time": False,
                            }
                        ],
                        remappings=[
                            ("rgb/image", "/camera/camera/color/image_raw"),
                            ("rgb/camera_info", "/camera/camera/color/camera_info"),
                            ("depth/image", "/camera/camera/depth/image_rect_raw"),
                            ("imu", "/imu/data"),
                        ],
                    ),
                    # RTAB-Map SLAM node
                    # Consumes /odom from rgbd_odometry above
                    # Publishes: map -> odom TF
                    Node(
                        package="rtabmap_slam",
                        executable="rtabmap",
                        name="rtabmap",
                        output="screen",
                        parameters=[
                            {
                                "subscribe_depth": True,
                                "subscribe_rgb": True,
                                "subscribe_imu": False,  # IMU handled by odom node
                                "subscribe_odom_info": True,
                                "frame_id": "base_link",
                                "approx_sync": True,
                                "use_sim_time": False,
                                "Grid/FromDepth": "True",
                                "Grid/RangeMax": "0.6",
                                "Grid/RangeMin": "0.02",
                                "Grid/CellSize": "0.02",
                                "Grid/UseVoxelFilter": "True",
                                "Grid/FilterSize": "0.1",
                            }
                        ],
                        remappings=[
                            ("rgb/image", "/camera/camera/color/image_raw"),
                            ("rgb/camera_info", "/camera/camera/color/camera_info"),
                            ("depth/image", "/camera/camera/depth/image_rect_raw"),
                            ("odom", "/odom"),
                        ],
                        arguments=["--delete_db_on_start"],
                    ),
                    # Stream video to basestation
                    ExecuteProcess(
                        cmd=["/bin/bash", os.path.expanduser("~/stream.sh")],
                        output="screen",
                    ),
                    # Launch Nav2 with custom params (stvl_layer)
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nav2_bringup"),
                                "launch",
                                "navigation_launch.py",
                            ]
                        ),
                        launch_arguments={
                            "params_file": nav2_params,
                            "use_sim_time": "False",
                        }.items(),
                    ),
                ],  # end TimerAction actions
            ),  # end TimerAction
            TimerAction(
                period=12.0,  # adjust delay as needed
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "lifecycle",
                            "set",
                            "/local_costmap/local_costmap",
                            "activate",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )  # end LaunchDescription
