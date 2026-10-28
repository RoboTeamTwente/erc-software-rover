# rtt_gazebo_sim

Gazebo (Harmonic, the Jazzy default) test bench for rtt_slam. Status: written but NOT yet run.
First run is expected to need small fixes. See "First run checklist".

## Run

    colcon build --packages-select rtt_slam rtt_gazebo_sim
    source install/setup.bash
    ros2 launch rtt_gazebo_sim sim_slam_launch.yaml            # everything
    ros2 launch rtt_gazebo_sim sim_slam_launch.yaml rtabmap_viz:=true
    ros2 launch rtt_gazebo_sim sim_launch.py                   # Gazebo only

Drive by hand:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

## What is in here

- models/mars_yard: terrain from Model3D_mesh.fbx. A 117k triangle visual mesh with a texture
  baked from the vertex colors, and a 25k triangle collision mesh. Meters, Z up, xy centered.
  Small rocks exist only in the visual mesh, not in the physics.
- urdf/rtt_rover.urdf.xacro: placeholder rover. Root is base_link (no base_footprint), 4 wheel
  skid steer, RealSense style camera frames, IMU. All sizes are guesses.
- config/bridge.yaml: Gazebo to ROS topics, named exactly like the RealSense topics rtt_slam uses.
- scripts/covariance_stamper.py: adds covariance to wheel odometry and IMU (Gazebo gives zeros).
- config/ekf_sim.yaml: EKF with visual odometry, wheel odometry and IMU all enabled.

## Data flow

    Gazebo camera -> /camera/camera/... -> rgbd_odometry -> /rtabmap/odom -> EKF
    Gazebo diff drive -> /wheel/odom_raw -> covariance_stamper -> /wheel/odom -> EKF
    Gazebo IMU -> /imu/data_raw -> covariance_stamper -> /imu/data -> EKF
    EKF -> TF odom->base_link -> rtabmap -> /rtabmap/map + TF map->odom -> Nav2
    Nav2 -> /cmd_vel -> Gazebo diff drive

## First run checklist

1. gz topic -l                 camera topics exist: /camera/camera/image, depth_image, camera_info, points
2. ros2 topic list            the RealSense named topics and /wheel/odom, /imu/data are there
3. ros2 run tf2_tools view_frames   one parent per frame, odom->base_link published by ekf_filter_node
4. ros2 topic hz /rtabmap/odom      visual odometry is running (camera is 15 Hz)
5. ros2 topic echo /odometry/filtered   EKF output
6. Drive a loop with teleop and watch the map in rtabmap_viz or RViz

## Known assumptions and limits

- Terrain edge: the yard mesh is a tilted rectangle, the rover can drive off its edge.
- The texture comes from vertex colors. If rgbd_odometry loses tracking, check the camera view in
  Gazebo first (lighting, texture contrast).
- Camera range 10 m, 15 Hz, no depth noise. Real RealSense data will be worse.
- Skid steer friction (mu1, mu2 in the xacro) and wheel_separation need tuning. Wheel odometry
  from a skid steer rover over rough ground is expected to be poor.
- rtt_slam changes: wait_imu_to_init is now false, publish_tf of rgbd_odometry is false, rtabmap
  reads odom from TF, and the launch files take new arguments (ekf_config, publish_camera_tf,
  use_sim_time).
