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

- models/erc2026_terrain: the ERC 2026 Mars Yard scan (Model3D_mesh2.obj) in the ERC yard frame
  (origin S1, +Y towards S2). 289k triangle visual mesh textured with ERC's 1 cm orthophoto, 76k
  triangle collision mesh (with vertex normals). Levelled to the report's heights by the link pose
  in its model.sdf. Horizontal position checked against the survey markers at S1-S9, W1 and W2
  in the orthophoto: 1.5 cm median.
- models/erc2026_landmark_01..15: the ERC ArUco landmark posts (rulebook fig. 9/10): 250 mm box on
  a pole, DICT_5X5 id 50+n (150 mm) on all four faces, tag centre 0.417 m above the terrain.
- models/erc2026_points: visual-only poles on the S (green), W (blue) and P (magenta) points.
- worlds/mars_yard.sdf: terrain + landmarks (default). worlds/mars_yard_markers.sdf: plus the
  point poles. Cameras see the poles, so use the default world for SLAM:
  `ros2 launch rtt_gazebo_sim sim_launch.py world:=$(ros2 pkg prefix rtt_gazebo_sim)/share/rtt_gazebo_sim/worlds/mars_yard_markers.sdf`
- tools/: build_meshes.py (ERC scan + orthophoto -> meshes) and gen_world.py (landmarks, point
  poles, worlds and the RTAB-Map Marker/Priors line, from rtt_slam/maps/erc_2026_points.yaml).
  See "Regenerating the yard".
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

- Terrain edge: the scan ends at the yard edge, the rover can drive off it.
- Landmark yaw is unknown (faces aligned with X/Y). The landmark ids 51-65 match rulebook fig. 10;
  the rulebook text says 51-64, the final list comes with Update Report #3.
- With the 424x240 sim camera a 150 mm tag is only decodable up to roughly 2-3 m.
- Camera range 10 m, 15 Hz, no depth noise. Real RealSense data will be worse.
- Skid steer friction (mu1, mu2 in the xacro) and wheel_separation need tuning. Wheel odometry
  from a skid steer rover over rough ground is expected to be poor.
- rtt_slam changes: wait_imu_to_init is now false, publish_tf of rgbd_odometry is false, rtabmap
  reads odom from TF, and the launch files take new arguments (ekf_config, publish_camera_tf,
  use_sim_time).

## Regenerating the yard

The raw ERC files (Model3D_mesh2.obj, orthophoto.tif) are not in the repo. From the repo root:

    python3 src/rtt_gazebo_sim/tools/build_meshes.py Model3D_mesh2.obj orthophoto.tif \
        src/rtt_gazebo_sim/models/erc2026_terrain --visual-cell 0.10 --collision-cell 0.20
    python3 src/rtt_gazebo_sim/tools/gen_world.py src/rtt_slam/maps/erc_2026_points.yaml src/rtt_gazebo_sim

gen_world.py prints the Marker/Priors line: paste it into rtt_slam/config/rtabmap.yaml.
Then rebuild the prior cost map (rtt_slam/maps/README.md).
If the levelling changes (LEVEL in gen_world.py), change PLANE in rtt_slam/tools/build_prior_map.py too.
