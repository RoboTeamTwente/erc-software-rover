# rtt_gazebo_sim

Gazebo Harmonic test bench for `rtt_slam`. It puts the placeholder rover into the ERC 2026 yard
from `rtt_gz_worlds`. Status: assembled, NOT yet run.

## Run

    colcon build --packages-select rtt_gz_worlds rtt_slam rtt_gazebo_sim
    source install/setup.bash
    ros2 launch rtt_gazebo_sim sim_slam_launch.yaml                     # everything, rover at S1
    ros2 launch rtt_gazebo_sim sim_slam_launch.yaml start:=S4 yaw:=0.8  # other start / heading
    ros2 launch rtt_gazebo_sim sim_launch.py                            # Gazebo only

Drive by hand:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

## What is in here

- `urdf/rtt_rover.urdf.xacro`: **placeholder rover**, taken from `dev-Amir` (commit 2089045, chassis
  raised to 0.20 m). It's a 4-wheel skid steer with RealSense-style camera frames and an IMU, and
  all sizes are guesses. `base_link` is the root, at wheel axle height. This will be replaced by the
  mechanical team's URDF export.
- `config/bridge.yaml`: maps Gazebo topics to ROS, named exactly like the RealSense topics `rtt_slam` uses.
- `scripts/covariance_stamper.py`: Gazebo publishes zero covariance on wheel odometry and IMU, and
  the EKF needs real values. This script adds them.
- `launch/sim_launch.py`: starts the world, the rover, the bridge and the covariance node.
- `launch/sim_slam_launch.yaml`: the above plus `rtt_slam/launch/slam_launch.yaml` with
  `realsense:=false publish_camera_tf:=false use_sim_time:=true`.

The world, terrain, landmarks and ERC points live in `rtt_gz_worlds`. This package contains no
SLAM, EKF or Nav2 config on purpose: the sim runs the rover's own config files.

## Spawning

`start` is a point name from `rtt_gz_worlds/config/erc2026_points.yaml`. The rover is placed at that
point's x, y in ERC coordinates (the Gazebo world frame), 0.20 m above the simulated ground
(`ground_z`), and drops onto the terrain. `yaw` is the heading in the ERC frame, in radians:
0 = +X, 1.5708 = +Y (into the yard).

The rulebook draws the start point and a limited heading at each trial, so test with several.

## Data flow

    Gazebo camera -> /camera/camera/... -> rgbd_odometry -> /rtabmap/odom -> EKF
    Gazebo diff drive -> /wheel/odom_raw -> covariance_stamper -> /wheel/odom -> EKF
    Gazebo IMU -> /imu/data_raw -> covariance_stamper -> /imu/data -> EKF + rtabmap
    EKF -> /odometry/filtered (+ TF odom->base_link) -> rtabmap -> /rtabmap/map + TF map->odom -> Nav2
    Nav2 -> /cmd_vel -> Gazebo diff drive

## First run checklist

1. `gz topic -l`: the camera topics exist (`/camera/camera/image`, `depth_image`, `camera_info`, `points`).
2. `ros2 topic list`: the RealSense-named topics, `/wheel/odom` and `/imu/data` are there.
3. `ros2 run tf2_tools view_frames`: one parent per frame, and `odom->base_link` comes from `ekf_filter_node`.
4. `ros2 topic hz /rtabmap/odom`: visual odometry is running (the camera runs at 15 Hz).
5. `ros2 topic echo /odometry/filtered`: EKF output.
6. Drive a loop with teleop and watch the map in rtabmap_viz or RViz.

## Known limits

- **Map frame is not the ERC frame.** With `rtt_slam`'s EKF config, `odom` (and so rtabmap's `map`)
  starts at the rover's spawn pose. ERC coordinates have to be converted using the start point and
  heading until that's solved. dev-Amir does it with a sim-only EKF `initial_state`; landmark
  priors are the other option.
- **Placeholder rover:**
  - It skid steers; the real rover steers each wheel with a second motor.
  - The friction (`mu1`, `mu2`) and `wheel_separation` need tuning.
  - Skid-steer wheel odometry on rough ground will be poor.
- **Point cloud frame:** `/camera/camera/depth/color/points` feeds the Nav2 voxel layer. Check in RViz
  that it lines up with the terrain. Gazebo publishes it with x forward, while the image frame has
  z forward.
- **Ideal camera:** 10 m range, 15 Hz, no depth noise. Real RealSense data will be worse.
- **Physics step:** the `rtt_gz_worlds` world uses a 1 ms step. If the real-time factor drops well
  below 1 on your laptop, try 2–4 ms in `tools/gen_world.py` (the old world used 4 ms).
