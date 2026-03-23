# [Oros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02     

# ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    imu_topic:=/imu/data \
    approx_sync:=true \
    frame_id:=camera_link \
    subscribe_imu:=true \
    use_sim_time:=false \
    Grid/FromDepth:=true \
    Grid/RangeMax:=2.0 \
    Grid/RangeMin:=0.05 \
    Grid/CellSize:=0.02 \
    Grid/UseVoxelFilter:=true \
    Grid/FilterSize:=0.1

ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f YOUR_MAP_NAME

#gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000

