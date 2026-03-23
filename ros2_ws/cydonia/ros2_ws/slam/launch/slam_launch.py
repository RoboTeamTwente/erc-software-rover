import os
import sys
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


nav2_params = PathJoinSubstitution([
    FindPackageShare("slam"),
    "config",
    "nav2_params.yaml"
])

stvl_params = PathJoinSubstitution([
    FindPackageShare("slam"),
    "config",
    "stvl_params.yaml"
])


def generate_launch_description():
    return LaunchDescription([
        #Delete database of old map before beginning
        ExecuteProcess(
            cmd=['rm', '-r', os.path.expanduser('~/.ros/rtabmap.db')],
            output='screen',
        ),
        # Launch realsense2_camera with IMU and depth enabled
        GroupAction(
        actions=[
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']),
            launch_arguments={
                'enable_gyro': 'True',
                'enable_accel': 'True',
                'unite_imu_method': '2',
                'align_depth.enable': 'True',
                'enable_rgbd': 'True',
                'pointcloud__neon_.enable': 'True',
                'enable_depth': 'True',
                'enable_color': 'True',
            }.items(),
        )]),

        # Wait 6 seconds before setting the pointcloud parameter
        TimerAction(
        period=6.0,  # seconds
        actions=[
            ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/camera/camera', 'pointcloud__neon_.enable', 'true'],
            output='screen'
        )
        ]
        ),
        
        # Launch imu_filter_madgwick with the specified arguments
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': True,
                'world_frame': 'enu'
            }],
            remappings=[
                ('/imu/data_raw', '/camera/camera/imu'),
                ('/imu/data', '/imu/data')
            ]
        ),
        
        # Launch rtabmap with all the required topics and parameters
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']),
            launch_arguments={
                'rgb_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/depth/image_rect_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'imu_topic': '/imu/data',
                'approx_sync': 'True',
                'frame_id': 'camera_link',
                'subscribe_imu': 'True',
                'use_sim_time': 'False',
                'Grid/FromDepth': 'True',
                'Grid/RangeMax': '2.0',
                'Grid/RangeMin': '0.05',
                'Grid/CellSize': '0.02',
                'Grid/UseVoxelFilter': 'True',
                'Grid/FilterSize': '0.1'
            }.items(),
        ),

       # Node(
       # package='gst_pipeline',
       # executable='pipeline_node',
       # name='video_stream',
       # output='screen',
       # parameters=[
       #     PathJoinSubstitution([FindPackageShare('slam'), 'config', 'new_params.yaml']),
       #     ]
       # ),

        # Stream video to basestation
        ExecuteProcess(
           cmd=['/bin/bash', os.path.expanduser('~/stream.sh')],
           output='screen',
        ),

        # Try with streaming node 
       # Node(
       #     package='streaming',  
       #     executable='stream_video',
       #     name='video_stream_node',
       #     output='screen',
       # ),

       #Launch Nav2
       #IncludeLaunchDescription(
       #PathJoinSubstitution([
       # FindPackageShare('nav2_bringup'),
       # 'launch',
       # 'navigation_launch.py'
       # ]),
       # launch_arguments={
       # 'params_file': nav2_params,
       # 'use_sim_time': 'False'
       # }.items(),
       # )
        
       # Launch Nav2
       Node(
       package='nav2_costmap_2d',
       executable='nav2_costmap_2d',
       name='local_costmap',
       output='screen',
       parameters=[stvl_params],
       ),

       Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),


    ])

