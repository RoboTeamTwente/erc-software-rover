ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rbc_camera.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true enable_gyro:=true enable_accel:=true
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rbc_camera.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method=2
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rbc_camera.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
sudo apt install ros-humble-rtabmap ros-humble-rtabmap-launch
ls /opt/ros/humble/share/rtabmap_launch/launch
hostname
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rbc_camera.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-inspect-1.0 videoscale
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoscale ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoscale ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.109 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoscale ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.109 port=4500
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 rbc_camera.profile:=1280x720x30 pointcloud.enable:=true align_depth.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2
source install/setup.bash
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! audiovideosink
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! autovideosink
st-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 caps=width=640,height=320 ! videoconvert ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! video/x-raw,width=640,height=320 ! videoconvert ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvertscale ! video/x-raw,width=640,height=320 ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! video/x-raw,width=640,height=320 ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! video/x-raw,width=640,height=320 ! videoconvert ! x264enc tune=zerolatency bitrate=512 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
ls /dev
ls /dev/video*
dmesg
sudo dmesg
sudo dmesg -HW
ls /dev/video*
sudo udevadm 
sudo udevadm trigger
sudo udevadm --help
sudo udevadm info
sudo udevadm monitor
ls /dev/video*
ls /sys/class/video4linux/
realsense-viewer 
ros2 topic list
. ros2_ws/install/setup.bash
ls /opt/ros/humble/share/rtabmap_launch/launch
ros2 launch rtabmap_ros rtabmap.launch.py rgb_topic:/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/camera/camera/imu approx_sync:=true
ls /opt/ros/humble/share/rtabmap_launch/launch
ros2 launch rtabmap_ros rtabmap.launch.py rgb_topic:/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/camera/camera/imu approx_sync:=true
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/camera/camera/imu approx_sync:=true
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/camera/camera/imu approx_sync:=true
ros2 launch rtabmap_launch rtabmap.launch.py   frame_id:=camera_link   rgb_topic:=/camera/camera/color/image_raw   depth_topic:=/camera/camera/depth/image_rect_raw   camera_info_topic:=/camera/camera/color/camera_info   imu_topic:=/camera/camera/imu   approx_sync:=true   wait_imu_to_init:=true   imu_guess_frame_id:=camera_link
ros2 launch rtabmap_launch rtabmap.launch.py   frame_id:=camera_link   rgb_topic:=/camera/camera/color/image_raw   depth_topic:=/camera/camera/depth/image_rect_raw   camera_info_topic:=/camera/camera/color/camera_info   imu_topic:=/camera/camera/imu   approx_sync:=true   wait_imu_to_init:=true   imu_guess_frame_id:=camera_lin
vim .basher
vim .bashrc 
realsense-viewer 
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
ls /dev/video*
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
ls /dev/v4l/
ls /dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i_134523065326-video-index
ls /sys/class/video4linux/
ls /sys/class/video4linux/video0/
cat /sys/class/video4linux/video0/name 
head /sys/class/video4linux/video*/name 
v4l2-ctl --device=/dev/video0 --list-formats-ext
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
head /sys/class/video4linux/video*/dev
head /sys/class/video4linux/video*/id
tree -a /sys/class/video4linux/video0
sudo apt install tree
tree -a /sys/class/video4linux/video0
cat /sys/class/video4linux/video0/uevent 
cat /sys/class/video4linux/video0/index
cat /sys/class/video4linux/video1/index
cat /sys/class/video4linux/video4/index
cat /sys/class/video4linux/video0/dev
cat /sys/class/video4linux/video0/dev_debug 
tree -a /sys/class/video4linux/video0/subsystem/
lsusb
ls /etc/udev/
ls /etc/udev/rules.d/
cat /etc/udev/rules.d/10-nv-jetson-dm.rules 
cat /etc/udev/rules.d/99-tegra-
cat /etc/udev/rules.d/99-tegra-devices.rules 
lspci
lsusb
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video4 --list-formats-ext
udevadm info -a -p /dev/video0
udevadm info -a -n /dev/video0
udevadm info -a -n /dev/video1
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video0
udevadm info -n /dev/video1
udevadm info -n /dev/video2
udevadm info -n /dev/video3
udevadm info -n /dev/video2
udevadm info -n /dev/video0
udevadm info -n /dev/video4
udevadm info -n /dev/video5
udevadm info -n /dev/video6
udevadm info -n /dev/video*
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video1 --list-formats-ext
v4l2-ctl --device=/dev/video2 --list-formats-ext
v4l2-ctl --device=/dev/video3 --list-formats-ext
v4l2-ctl --device=/dev/video4 --list-formats-ext
v4l2-ctl --device=/dev/video5 --list-formats-ext
v4l2-ctl --device=/dev/video6 --list-formats-ext
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video1 --list-formats-ext
v4l2-ctl --device=/dev/video2 --list-formats-ext
v4l2-ctl --device=/dev/video4 --list-formats-ext
cheese
v4l2-ctl --device=/dev/video4 --list-formats-ext
cheese
gst-inspect-1.0 rtpsink
gst-inspect-1.0 udpsink
gst-launch-1.0 v4l2src device=/dev/video6 ! videoconvert ! autovideosink
gst-launch-1.0 v4l2src device=/dev/video1 ! videoconvert ! autovideosink
gst-launch-1.0 v4l2src device=/dev/video3 ! videoconvert ! autovideosink
gst-launch-1.0 v4l2src device=/dev/video5 ! videoconvert ! autovideosink
gst-launch-1.0 -v v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency crt=13 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=4500
gst-inspect-1.0 x264enc
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency pass=cbr ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:4500 latency=100
ip a
gst-launch-1.0 -v v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency pass=cbr ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:4500 latency=100
/usr/bin/time gst-launch-1.0 v4l2src device=4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:8888/ latency=100
gst-launch-1.0 v4l2src device=4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:8888/ latency=100
gst-launch-1.0 v4l2src device=video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:8888/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:8888/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:4500/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://192.168.0.109:4500/ latency=100
sudo dmesg
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://192.168.0.109:4500/ latency=100
ip a
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0:4500/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:4500/ latency=100
sudo nft list ruleset
sudo iptables -L
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://0.0.0.0:4500/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc bitrate=8000 tune=zerolatency speed-preset=superfast byte-stream=true threads=1 key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://:4500/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtserversink uri=srt://:4500/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtsink uri=srt://:4500/ latency=100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtsink uri=srt://:4500/ latency=50
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! rtpx264depau ! rtpsrc uri=rtp://192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! rtph264depau ! rtpsrc uri=rtp://192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! rtph264pau ! rtpsrc uri=rtp://192.168.0.104:500
gst-inspect-1.0 
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! rtph264pay ! rtpsrc uri=rtp://192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! rtph264depay ! rtpsrc uri=rtp://192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264depay ! rtpsrc uri=rtp://192.168.0.104:500
gst-inspect-1.0 
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! rtpsrc uri=rtp://192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! rtpsink uri=rtp://192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink uri=192.168.0.104:500
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=192.168.0.104
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! video/x-h264, profile=baseline ! mpegtsmux ! srtsink uri=srt://:4500/ latency=50
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=192.168.0.104
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=192.168.0.100
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
sudo apt install ros-humble-{navigation2,nav2-{bringup,minimal-tb-*},turtlebot3-gazebo}
sudo apt install ros-humble-{navigation2,nav2-bringup,turtlebot3-gazebo}
sudo apt install ros-humble-{navigation2,nav2-bringup}
ls /opt/ros/
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
sudo apt install ros-humble-nav2-minimal-tb
sudo apt install ros-humble-nav2-minimal-tb-*
apt search nav2-minimal
apt search turtlebot
apt search gazebo
gz
gzserver
apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-ros-gz-sim
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
gzclient
apt search
apt-file
sudo apt install apt-file
sudo apt-file update
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo
cat /etc/apt/sources.list.d/ros2.sources 
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! autovideosink
. ros2_ws/install/setup.bash
ros2 topic echo /camera/camera/gyro/metadata
nano new_file.py
vim new_file.py
. ros2_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 align_depth:=true enable_rgbd:=true
. ros2_ws/install/setup.bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false -p world_frame:=enu -r /imu/data_raw:=/camera/camera/imu -r /imu/data:=/imu/data
ros2 run imu_filter_madgwick imu_filter_madgwick_node   --ros-args   -p use_mag:=false   -p publish_tf:=false   -p world_frame:=enu   -p remove_gravity_vector:=true   -r /imu/data_raw:=/camera/camera/imu   -r /imu/data:=/imu/data
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false -p world_frame:=enu -r /imu/data_raw:=/camera/camera/imu -r /imu/data:=/imu/data
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=true -p world_frame:=enu -r /imu/data_raw:=/camera/camera/imu -r /imu/data:=/imu/data
. ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /rtabmap/odom --once
ros2 topic list
. ros2_ws/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/depth/image_rect_raw imu_topic:=/imu/data approx_sync:=true frame_id:=camera_link subscribe_imu:=true use_sim_true:=false
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/imu/data approx_sync:=true frame_id:=camera_link subscribe_imu:=true use_sim_true:=false
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/camera/camera/imu approx_sync:=false frame_id:=camera_link subscribe_imu:=true use_sim_true:=false
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/camera/camera/imu approx_sync:=false frame_id:=camera_link subscribe_imu:=true use_sim_time:=false
ros2 launch rtabmap_launch rtabmap.launch.py   rgb_topic:=/camera/camera/color/image_raw   depth_topic:=/camera/camera/depth/image_rect_raw   camera_info_topic:=/camera/camera/color/camera_info   imu_topic:=/camera/camera/imu   approx_sync:=false   frame_id:=camera_link   odom_frame_id:=odom   subscribe_imu:=true   publish_tf:=true   use_sim_time:=false
ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/depth/image_rect_raw camera_info_topic:=/camera/camera/color/camera_info imu_topic:=/imu/data approx_sync:=true frame_id:=camera_link subscribe_imu:=true use_sim_time:=false
. ros2_ws/install/setup.bash
rviz2
vim ncommands.py
ls
cat ncommands.py
ls
. ros2_ws/install/setup.bash
cat ncommands.py
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 align_depth:=true enable_rgbd:=true
. ros2_ws/install/setup.bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=true -p world_frame:=enu -r /imu/data_raw:=/camera/camera/imu -r /imu/data:=/imu/data
. ros2_ws/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=base_link     subscribe_imu:=true     use_sim_time:=false     rtabmap_args:="--delete_db_on_start"     Grid/FromDepth:=true     Grid/RangeMax:=5.0
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     rtabmap_args:="--delete_db_on_start"     Grid/FromDepth:=true     Grid/RangeMax:=5.0
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     --ros-args     -p Grid/FromDepth:=true     -p Grid/RangeMax:=2.0     -p Grid/RangeMin:=0.05     -p Grid/CellSize:=0.02     -r /rtabmap/map:=/map
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02     -r /rtabmap/map:=/map
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02     remap:=[/rtabmap/map:=/map
vim ncommand.py
ls
cat ncommands.py
vim ncommand.py
. ros2_ws/install/setup.bash
ros2 topic list | grep map
ros2 topic info /rtabmap/map
ros2 topic echo /rtabmap/map --once
ros2 topic echo /rtabmap/odom --once
ros2 topic echo /rtabmap/map --once
ros2 topic echo /map --once
ros2 topic echo /rtabmap/map --once
ros2 topic echo /camera/camera/depth/image_rect_raw --once
ros2 topic echo /rtabmap/odom --once
ros2 run tf2_tools view_frames
ros2 topic echo /rtabmap/odom --once
ros2 topic echo /camera/camera/depth/image_rect_raw --once
ros2 topic echo /rtabmap/map --once
. ros2_ws/install/setup.bash
ros2 topic echo /rtabmap/info --once
ros2 topic echo /camera/camera/depth/image_rect_raw --once
rviz2
. ros2_ws/install/setup.bash
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_tools view_frames
ros2 topic echo /camera/camera/depth/image_rect_raw --once
ros2 topic echo /rtabmap/map --once
. ros2_ws/install/setup.bash
ros2 param get /rtabmap Grid/FromDepth
ros2 node list
ros2 run tf2_tools view_frames
ros2 topic echo /camera/camera/color/camera_info
vim ncommand.py
ls
cat ncommand.py
cat ncommands.py
. ros2_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 align_depth:=true enable_rgbd:=true
. ros2_ws/install/setup.bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=true -p world_frame:=enu -r /imu/data_raw:=/camera/camera/imu -r /imu/data:=/imu/data
. ros2_ws/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02     remap:=[/rtabmap/map:=/map]
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02
. ros2_ws/install/setup.bash
ros2 topic list
. ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli --ros-args -p map_topic:=/rtabmap/map -p filename:=/tmp/my_map
ros2 topic echo /rtabmap/map
ros2 topic info /rtabmap/map
ros2 topic echo /rtabmap/map
ros2 topic echo /imu/data
ros2 topic echo /rtabmap/map
ros2 topic echo map
ros2 topic echo /map
ros2 topic echo /rtabmap/map
ros2 run nav2_map_server map_saver_cli --ros-args -p map_topic:=/rtabmap/map -p filename:=/tmp/my_map
ros2 topic echo /rtabmap/map
ros2 topic echo /rtabmap/map_updates
ros2 topic echo /rtabmap/map
. ros2_ws/install/setup.bash
ros2 lifecycle set /map_saver activate
ros2 node list
ros2 run nav2_map_server map_saver_cli --ros-args -p map_topic:=/rtabmap/map -p filename:=/tmp/my_map
ros2 lifecycle set /map_saver activate
ros2 lifecycle get /map_saver
ros2 topic list
ros2 node list
ros2 topic echo /camera/camera/color/image_raw
ros2 topic echo /imu/data_raw
ros2 topic list
ros2 topic echo /imu/data
ros2 run rqt_tf_tree rqt_tf_tree
ros2 pkg list | grep rqt_tf_tree
ros2 run tf2_tools view_frames
sudo apt install ros-humble-rqt-tf-tree
ros2 run rqt_tf_tree rqt_tf_tree
. ros2_ws/install/setup.bash
rviz2
. ros2_ws/install/setup.bash
ls
cd ros2_ws
ls
cat /install/setup.bash
cd install
ls
cd slam
ls
cd share
ls
cd slam
ls
cd ..
ls install/
ls build/
colcon build --executor sequential --cmake-args -DFORCE_LIBUVC=true
cd ..
.ros2_ws/install/setup.bash
. ros2_ws/install/setup.bash
cd slam
cd ros2_ws/
ls
cd slam
ls
cd ..
cd slam
cd launch
ls
cd ..
ros2 launch slam slam_launch.py
. ros2_ws/install/setup.bash
cd ..
. ros2_ws/install/setup.bash
cat ros2_ws/slam/launch/slam_launch.py 
rm -r .ros/rtabmap.db
ls
ros2 topic echo /imu/data | grep frame_id
ros2 topic echo /imu/data 
cat ros2_ws/slam/launch/slam_launch.py 
ls
cat launch_file.py
rviz2
rm -r .ros/rtabmap.db
rviz2
rm -r .ros/rtabmap.db
rviz2
ls
cat ncommand.py
rm -r .ros/rtabmap.db
rviz2
. ros2_ws/install/setup.bash
ls
cat ncommand.py
cat ncommands.py
cd ros2_ws
find -f rtabmap.db
sudo find / -type f -name "rtabmap.db"
cd ..
ls
rviz2
vim launch_file.py
ls
vim ros2_ws/slam/launch/slam_launch.py
cd ros2_ws
colcon build --packages-select slam
cat ros2_ws/slam/launch/slam_launch.py
cat slam/launch/slam_launch.py
vim slam/launch/slam_launch.py
cat slam/launch/slam_launch.py
cd ..
ls
vim launch_file1.py
cd ros2_ws
vim slam/launch/slam_launch.py
cat slam/launch/slam_launch.py
. ros2_ws/install/setup.bash
cd ..
. ros2_ws/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02    
. ros2_ws/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02     Grid/UseVoxelFilter:=true     Grid/FilterSize:=0.1 \
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     imu_topic:=/imu/data     approx_sync:=true     frame_id:=camera_link     subscribe_imu:=true     use_sim_time:=false     Grid/FromDepth:=true     Grid/RangeMax:=2.0     Grid/RangeMin:=0.05     Grid/CellSize:=0.02     Grid/UseVoxelFilter:=true     Grid/FilterSize:=0.1    Grid/OccupiedThresh:=1.00 
. ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -t /rtabmap/map -f new_map
ros2 param list /rtabmap
ros2 param list /rtabmap/map
ros2 node list
ros2 param list /rtabmap/rtabmap
. ros2_ws/install/setup.bash
cd ros2_ws
ros2 launch slam slam_launch.py
. ros2_ws/install/setup.bash
rvoz2
rviz2
ls
cat ncommand.py
cat ncommands.py
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 align_depth:=true enable_rgbd:=true
. ros2_ws/install/setup.bash
gst-launch-1.0 v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
cat ncommand.py
cat ncommand1.py
ls
cat ncommands.py
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 align_depth:=true enable_rgbd:=true
cat ros2_ws/slam/launch/slam_launch.py 
ls
. ros2_ws/install/setup.bash
cat launch_file1.py
vim ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws
colcon build --packages-select slam
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
cd ..
ls
cat ncommand.py
vim ros2_ws/slam/launch/slam_launch.py 
cat ros2_ws/slam/launch/slam_launch.py 
sudo apt-get install ros-humble-navigation
sudo apt-get install ros-humble-navigation2
cd ros2_ws/slam
ls
cd src
ls
cd ..
gst-launch-1.0 -v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
sudo apt install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav
gst-launch-1.0 -v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-tools
gst-launch-1.0 v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
sudo apt-get install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
. ros2_ws/install/setup.bash
gst-launch-1.0 -v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
sudo apt-get install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
gst-launch-1.0 v4l2src device=/video/video0 | videoconvert | x264enc tune=zerolatency bitrate=256 | rtph264pay config-interval=1 pt=96 | udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/video/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/video/video4 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/video/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
ls /dev/video*
sudo ls /dev/video
sudo ls /dev/video*
ls -l /dev/video0
gst-launch-1.0 v4l2src device=/video/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
sudo ls /dev/video*
dmesg | grep video
sudo dmesg | grep video
[200~lsmod | grep v4l2
~lsmod | grep v4l2
lsmod | grep v4l2
v4l2-ctl --list-devices
sudo ln -s /dev/media0 /dev/video0
gst-launch-1.0 v4l2src device=/video/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
sudo ln -s /dev/video0 /dev/media0
gst-launch-1.0 v4l2src device=/dev/media0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
v4l2-ctl --list-devices
ls /dev/video*
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
dmesg | grep video
vim ncommand.py
v4l2-ctl --list-devices
ls /dev/video*
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
dmesg | grep video
vim ncommand.py
cd ros2_ws
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws
ros2 launch slam slam_launch.py
. ros2_ws/install/setup.bash
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
ls /dev/video*
lsusb
lsmod | grep v4l2
ls /dev/video*
v4l2-ctl --list-devices
gst-launch-1.0 rsvideoconvert device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/media0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
ros2 topic list
sudo apt-get install gstreamer1.0-realsense
gst-inspect-1.0 | grep realsense
realsense-viewer
gst-launch-1.0 v4l2src device=/dev/media0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
v4l2-ctl --list-formats-ext
realsense-viewer
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! autovideosink
v4l2-ctl --list-formats-ext
v4l2-ctl --list-formats
realsense-viewer
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! autovideosink
v4l2-ctl --list-formats-ext
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.21 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-reset=ultrafast bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
v4l2-ctl --list-formats-ext
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.42.27 port=5000
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=256 ! rtph264pay config-interval=1 pt=96 ! udpsink host=145.126.109.171 port=4500
history | grep gst-launch
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
vim ncommands.py
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
v4l2-ctl --list-formats-ext
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video4 --list-formats-ext
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
v4l2-ctl --device=/dev/video4 --list-formats-ext
for device in /dev/video*; do v4l2-ctl --device=$device --list-formats-ext; done
gst-launch-1.0 v4l2src device=/dev/video4 ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.42.27
v4l2-ctl --device=/dev/video4 --list-formats-ext
vim new_script.sh
chmod +x new_script.sh
./new_script.sh
cat new_script.sh
./new_script.sh
sudo apt install ros-humble-gscam
ros2 run gscam gscam_node --ros-args -p gstreamer_pipeline:="appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink host=145.126.42.27 port=4500" -r /camera/color/image_raw:=/camera/camera/color/image_raw -r /camera/depth/image_rect_raw:=/camera/camera/depth/image_rect_raw
GSCAM_CONFIG="appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink host=145.126.42.27 port=4500" ros2 run gscam gscam_node --ros-args -r /camera/color/image_raw:=/camera/camera/color/image_raw -r /camera/depth/image_rect_raw:=/camera/camera/depth/image_rect_raw

fg
jobs
bg
cd ..
cat ros2_ws/slam/launch/slam_launch.py 
ls
cat new_script.sh
. ros2_ws/install/setup.bash
cat new_script.sh 
sudo apt-get install ros-humble-gscam
vim new_script.sh 
./new_script.sh
cat new_script.sh 
cat ros2_ws/slam/launch/slam_launch.py 
vim new_script.sh 
cat new_script.sh 
cd
cd ros2_ws/
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
colcon build --packages-select streaming
tree
tree install/streaming/
vim streaming/CMakeLists.txt 
cat streaming/streaming/stream_video.py 
ls
vim streaming/streaming/stream_video.py 
. ros2_ws/install/setup.bash
cat ros2_ws/slam/launch/slam_launch.py 
rviz2
cd ros2_ws
ls
cat slam/CMakeLists.txt 
cat slam/package.xml 
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
. ros2_ws/install/setup.bash
gst-launch-1.0 rtspsrc location=rtsp://localhost/camera/color/image_raw ! decodebin ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay ! udpsink port=4500 host=145.126.47.213
sudo apt-get install ffmpeg
ffmpeg -i rtsp://localhost/camera/color/image_raw
telnet localhost 554
sudo apt-get install telnet
telnet localhost 554
ipconfig
curl rtsp://localhost/camera/color/image_raw
gst-rtsp-server
sudo apt-get install gstreamer1.0-rtsp
gst-rtsp-server
gst-rtsp-server --version
sudo apt-get install gstreamer-rtsp-server
ffmpeg -i rtsp://localhost/camera/color/image_raw
gst-launch-1.0 videotestsrc ! x264enc ! rtph264pay ! udpsink host=127.0.0.1 port=8554
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/test ! decodebin ! autovideosink
. ros2_ws/install/setup.bash
ls
cd ros2_ws
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
history | grep colcon 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
sudo apt-get install ros-humble-gstreamer
history | grep gscam
sudo apt install ros-humble-gscam
ros2 pkg create --build-type ament_cmake --license Apache-2.0 streaming
ls
cd streaming
ls
mkdir streaming
ls
cd streaming
touch stream_video.py
vim stream_video.py 
cd ~/
cd ros2_ws/
colcon build --packages-select streaming
cd streaming
cat CMakeLists.txt
ls
cat package.xml 
ros2 launch slam slam_launch.py
colcon build --packages-select slam
cd ..
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim streaming/streaming/stream_video.py 
cd ..
. ros2_ws/install/setup.py
history | grep install
. ros2_ws/install/setup.bash
cd ros2_ws
ros2 launch slam slam_launch.py
cd streaming
ls
cd ..
colcon build --package-select streaming
ros2 launch slam slam_launch.py
ls
cat launch_file.py
cat new_script.sh 
realsense-viewer
. ros2_ws/install/setup.bash
./new_script
./new_script.sh
fg
bg
ps
kill 14776
vim new_script.sh
./new_script.sh
vim new_script.sh
./new_script.sh
cd ros2_ws
ros2 launch slam slam_launch.py
ls
cd streaming
ls
cd src
ls
cd ..
cd streaming
ls
cat stream_video.py
cd ./
cd ./~
cd ..
cat slam/launch/slam_launch.py 
cd ..
ls
vim launch_file2.py
vim ros2_ws/slam/launch/slam_launch.py 
cat launch_file1.py
cd ros2_ws
ros2 launch slam slam_launch.py
cd ..
cat ros2_ws/slam/launch/slam_launch.py 
vim ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws
history | grep colcon
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat ros2_ws/slam/launch/slam_launch.py 
cat slam/launch/slam_launch.py 
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws
rviz2
cd ..
cat new_script.sh 
sudo apt install ros-humble-gscam
ros2 run gscam gscam_node   --ros-args   -p camera_name:=ros_cam   -p image_topic:=/camera/camera/color/image_raw   -p gscam_config:="appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast bitrate=2048 key-int-max=15 ! rtph264pay ! udpsink host=145.126.42.235 port=4500"
ros2 run gscam gscam_node   --ros-args   -p camera_name:=ros_cam   -p gscam_config:="appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast bitrate=2048 key-int-max=15 ! rtph264pay ! udpsink host=145.126.42.235 port=4500"   -p image_encoding:="rgb8"   -p frame_id:="camera_link"
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws
cd streaming
ls
cd streaming
ls
cat stream_video.py
ros2 param list /gscam_node
cd ..
. ros2_ws/install/setup.bash
. ros2_ws/install/setup.bash
history | grep scam
ros2 run gscam gscam_node   --ros-args   -p camera_name:=ros_cam   -p image_topic:=/camera/camera/color/image_raw   -p gscam_config:="appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast bitrate=2048 key-int-max=15 ! rtph264pay ! udpsink host=145.126.42.235 port=4500

ros2 run gscam gscam_node   --ros-args   -p camera_name:=ros_cam   -p image_topic:=/camera/camera/color/image_raw   -p gscam_config:="appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=superfast bitrate=2048 key-int-max=15 ! rtph264pay ! udpsink host=145.126.42.235 port=4500
ls
cat ros2_ws/slam/launch/slam_launch.py 
cat new_script.sh 
. ros2_ws/install/setup.bash
cd ros2_ws
ls
cd slam
ls
cd ..
mkdir ros_gstreamer_bridge/src
mkdir ros_gstreamer_bridge
cd ros_gstreamer_bridge
mkdir src
ls
cd src
git clone https://github.com/BrettRD/ros-gst-bridge.git
ls
cd ros-gst-bridge/
ls
cd ~/
cd ros2_ws
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws
colcon build --packages-select ros_gstreamer_bridge
ls
colcon build --packages-select ros_gstreamer_bridge
history | grep colcon
cd ~/ros_gstreamer_bridge
cd ros_gstreamer_bridge
cd src
cd ~/ros_gstreamer_bridge
cd ..
rm -r ros_gstreamer_bridge
sudo rm -r ros_gstreamer_bridge
ls
ros2 pkg create --build-type ament_cmake --license Apache-2.0 bridge
cd bridge
ls
cd src
git clone https://github.com/BrettRD/ros-gst-bridge.git
cd ..
colcon build --packages-select bridge
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
ls
cd bridge/src
ls
cd ros-gst-bridge/
ls
cat README.md 
ls
cd gst_bridge/
ls
cat CMakeLists.txt 
cat package.xml 
ls
cd src
ls
cd ..
cd ..
cd ~/
cd ros2_ws/
cd slam
ls
cd src
ls
cd ..
cd librealsense/
ls
cd src
ls
ros2 pkg list | grep ros-gst-bridge
cd ~/
cd ros2_ws/
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
ls
. ros2_ws/install/setup.bash
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws
ls
git clone https://github.com/BrettRD/ros-gst-bridge.git
ls
cd ros-get-bridge
ls
cd ros-gst-bridge
ls
cd ..
colcon build --packages-select ros-gst-bridge
cd gst-bridge
cd ros-gst-bridge/gst-bridge
cd ros-gst-bridge
ls
cd gst_bridge/
ls
cd bridge/
cd src
ls
cd ros-gst-bridge/
ls
cd gst-bridge
cd gst_bridge
ls
cd src
ls
cat gst_bridge.cpp
ls
cd ..
ls
cat CMakeLists.txt 
cd ~/bridge
cd ./bridge
cd ..
ls
cat CMakeLists.txt 
cd ~/
cd ros2_ws/
ls
cd bridge/
ls
cd src
ls
cd ros-gst-bridge/
ls
cd gst_bridge/
ls
cd src
ls
cat rosimagesrc.cpp
cd ..
cd slam
ls
cat CMakeLists.txt 
cat package.xml 
cd ..
ls
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
. ros2_ws/install/setup.bash
cd ..
. ros2_ws/install/setup.bash
cd ros2_ws/
ros2 launch slam slam_launch.py
colcon build --packages-select slam bridge
ros2 launch slam slam_launch.py
colcon build --packages-select slam bridge
. install/setup.bash
ros2 launch slam slam_launch.py
cd bridge
ls
cd src
ls
cd ros-gst-bridge/
ls
cd gst_bridge/
ls
cd src
ls
cd ..
. install/setup.bash
cd bridge/
ls
cat CMakeLists.txt 
cd ..
cat slam/CMakeLists.txt 
cd bridge/
cd src
cd ros-gst-bridge/
ls
cd gst_bridge/
ls
cd src
ls
cd ~/ros2_ws/
colcon build --packages-select ros-gst-bridge
ls
colcon build --packages-select gst_bridge
rosdep update
rosdep install -iy --from-paths .
colcon build --packages-select gst_bridge
grep -R audio_msgs
mv bridge/ ..
colcon build --packages-select gst_bridge
grep -R package.sh
grep -RF package.sh
cd ros-gst-bridge/
rosdep install -iy --from-paths .
colcon build --packages-select gst_bridge
cd src
ls
git checkout ros2
cd ..
colcon build --packages-select gst_bridge
history | grep install
. ros2_ws/install/setup.bash
cd ros2_ws
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
. install/setup.bash
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
pip install asyncio-glib
cd ..
vim new_script.sh 
./new_script.sh
cat ros2_ws/slam/launch/slam_launch.py 
. ros2_ws/install/setup.bash
ros2 launch slam slam_launch.py
sudo apt install python3-pip
pip install asyncio-glib
ros2 launch slam slam_launch.py
colcon build --packages-up-to slam
. ros2_ws/install/setup.bash
ros2 launch slam slam_launch.py
. ros2_ws/install/setup.bash
cd ros2_ws
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
. install/setup.bash
cd ..
cat new_script.sh 
vim new_script.sh 
./new_script.sh 
cd ros2_ws/
ls
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/image_raw --once
cat new_script.sh 
cd ..
cat new_script.sh 
./new_script.sh 
cat new_script.sh 
cd ros2_ws
ls
cd streaming
ls
cd ..
cd ros-gst-bridge/
ls
grep -R "declare_parameter" -n .
cd gst_bridge
grep -R "declare_parameter" -n .
ls
cd src
grep -R "declare_parameter" -n .
ls
cd ..
grep -R "declare_parameter" -n .
cd gst_pipeline/
grep -R "declare_parameter" -n .
ls
cd gst_pipeline/
grep -R "declare_parameter" -n .
ls
cat pipeline_node.py 
ls
cat pipeline.py
cd slam/
ls
mkdir config
cd config
ls
vim pipeline_params.yaml
cd ..
vim launch/slam_launch.py 
cd ..
. install/setup.bash
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim launch/slam_launch.py 
cat slam/launch/slam_launch.py 
ros2 param list
ros2 param list | grep poipeline
ros2 param list | grep pipeline
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
vim slam/config/pipeline_params.yaml 
colcon build --packages-select slam
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py
vim slam/config/pipeline_params.yaml 
. install/setup.bash
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw ! videoconvert ! autovideosink"
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw ! videoconvert ! autovideosink"
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink
cd ..
./new_script.sh 
. ros2_ws/install/setup.bash
./new_script.sh 
cat n
cat new_script.sh 
./new_script.sh 
history | grep gst-launch-1.0 rosimagesrc
history | grep gst-launch-1.0 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink
. ros2_ws/install/setup.bash
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
vim commands.txt
cat commands.txt 
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink
ac commands.txt 
cat commands.txt 
vim commands.txt 
. ros2_ws/install/setup.bash
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink
. ros2_ws/install/setup.bash
cat ros2_ws/slam/launch/slam_launch.py 
vim ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws
colcon build --select-packages slam
history | grep colcon
colcon build --packages-select slam
cat ros2_ws/slam/launch/slam_launch.py 
cd ..
cd ros2_ws
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
history | grep launch
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
rqt
cat slam/launch/slam_launch.py 
. install/setup.bash
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
vim slam/config/pipeline_params.yaml 
cat slam/config/pipeline_params.yaml 
cat slam/launch/slam_launch.py 
cat slam/config/pipeline_params.yaml 
vim slam/config/pipeline_params.yaml 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
cat slam/config/pipeline_params.yaml 
vim slam/config/pipeline_params.yaml 
colcon build --packages-select slam
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py
cat slam/config/pipeline_params.yaml 
ros2 launch slam slam_launch.py
cat slam/config/pipeline_params.yaml 
cat slam/launch/slam_launch.py 
cat slam/config/pipeline_params.yaml 
cat install/slam/
cd install/slam/
ls
cd share
ls
cd slam
ls
cd config
ls
vim pipeline_params.yaml 
vim new_params.yaml
cd ~/
cd ros2_ws/
colcon build --packages-up-to slam
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
cd ..
. ros2_ws/instsall/setup.bash
vim new_script.sh 
./new_script.sh 
vim new_script.sh 
ros2 topic echo /camera/camera/color/image_raw
cat ros2_ws/slam/launch/slam_launch.py 
vim ros2_ws/slam/launch/slam_launch.py 
. ros2_ws/instsall/setup.bash
. ros2_ws/install/setup.bash
cd ros2_ws/
colcon build --packages-select slam
ros2 launch slam slam_launch.py
echo $GST_PLUGIN_PATH
gst-inspect-1.0 rosimagesrc
cd slam
ls
mkdir share/slam
mkdir share
cd share
mkdir slam
cd slam 
mkdir config
cd config
vim pipeline_params.yaml
cd ~/
cd ros2_ws/
colcon build --packages-up-to slam
cd install
ls
cd slam/share/slam
ls
mkdir config
cd config
ls
vim pipeline_params.yaml
cd ~/
cd ros2_ws/
colcon build --packages-up-to slam
vim test.yaml
ros2 node list
ros2 param list /video_stream
ros2 topic list | grep image
gst-inspect-1.0 rosimagesrc
cd ros2_ws/
ls
cd ros-gst-bridge/
ls
cd ..
source  ros2_ws/install/setup.bash 
gst-inspect-1.0 rosimagesrc
gst-inspect-1.0 
gst-inspect-1.0 | grep -i ros
gst-inspect-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib | grep -i ros
gst-inspect-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge | grep -i ros
ls ~/ros2_ws/install/gst_
ls ~/ros2_ws/install/gst_bridge/
ls ~/ros2_ws/install/gst_bridge/lib/
ls ~/ros2_ws/install/gst_bridge/lib/gst_bridge/
gst-inspect-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge | grep -i ros
gst-launch-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib rosimagesrc
gst-launch-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge rosimagesrc
gst-launch-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge rosimagesink
gst-launch-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge rosgstbridge
gst-inspect-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge 
gst-inspect-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge | grep -i ros
echo $GST_PLUGIN_PATH
source ~/ros2_ws/install/setup.bash 
echo $GST_PLUGIN_PATH
cd ros2_ws/
colcon build --packages-up-to gst_bridge
source ~/ros2_ws/install/setup.bash 
echo $GST_PLUGIN_PATH
gst-launch-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge rosgstbridge
gst-launch-1.0 --gst-plugin-path=~/ros2_ws/install/gst_bridge/lib/gst_bridge rosimagesource

colcon build --packages-up-to gst_bridge
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py
echo $GST_PLUGIN_PATH
gst-inspect-1.0 rosimagesrc
ros2 launch slam slam_launch.py
vim slam/config/pipeline_params.yaml 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py
. install/setup.bash
cd ..
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay pt=96 ! udpsink host=145.126.41.151 port=4500
cd ros2_ws/install/slam/share/slam/
ls
cd config/
ls
cat new_params.yaml 
vim new_params.yaml 
cd ~/ros2_ws/
. install/setup.bash
colcol build --packages-up-to slam
colcon build --packages-up-to slam
cat install/slam/share/slam/config/new_params.yaml 
. ros2_ws/install/setup.bash
cd ros2_ws/
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
ros2 launch slam slam_launch.py
cat commands.txt 
vim commands.txt 
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink
rosimagesrc ros-topic="/camera/camera/color/image_raw" !
gst-launch-1.0 rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! autovideosink rosimagesrc ros-topic="/camera/camera/color/image_raw" ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=superfast byte-stream=true key-int-max=15 intra-refresh=true ! rtph264pay pt=96 ! udpsink host=145.126.41.151 port=4500
ls
cat ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws/
ls
cd install/
ls
cd slam
ls
cd share
ls
cd slam
ls
cd config
ls
cat new_params.yaml 
vim new_params.yaml 
cat new_params.yaml 
vim new_params.yaml 
cat new_params.yaml 
vim new_params.yaml 
cat new_params.yaml 
vim new_params.yaml 
cat new_params.yaml 
vim new_params.yaml 
. ros2_ws/install/setup.bash
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cd ros2_ws/
colcon build packages-up-to slam
history | grep colcon
colcon build --packages-up-to slam
ls
cd ros-gst-bridge/
ls
cd ~/ros2_ws/
grep -rn "simple_bins" ~/ros2_ws
cat ros-gst-bridge/gst_pipeline/gst_pipeline/pipeline_node.py
ip a
nmcli
networkctl 
nmcli
nmcli d sh enP8p1s0 
nmcli d st 
sudo nmtui
ping 192.168.0.1
ping 192.168.1.1
i pr
ip r
nmcli 
nmcli d
nmcli d down enP8p1s0 
nmcli d up enP8p1s0 
sudo nmtu
sudo nmtui
ip a
ping 192.168.1.1
ping 192.168.1.2
sudo nmtu
sudo nmtui
ip a
ping 192.168.1.1
cd ros2_ws/slam/
ls
cd config
ls
cat pipeline_params.yaml 
vim nav2_params.yaml
cat nav2_params.yaml 
cd ..
cd sr
ls
cd install/
ls
cd slam
ls
cd share/
ls
cd slam/
cd config/
ls
vim nav2_params.yaml
cat nav2_params.yaml
. ros2_ws/install/setup.bash
rqt
ros2 param list
rqt
cat ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws/
ls
cd realsense-ros/
ls
cd realsense-camera
cd realsense2-camera
cd realsense2_camera
ls
cd launch
ls
cat rs_launch.py
ros2 topic list
. ros2_ws/install/setup.bash
rviz2
ros2 param get /camera/camera pointcloud__neon_.enable
ros2 topic list | grep depth
ros2 param get /camera/camera filters
cat ros2_ws/slam/launch/slam_launch.py 
ros2 param get /camera/camera align_depth.enable
ros2 param get /camera/camera pointcloud__neon_.enable
cat ros2_ws/slam/launch/slam_launch.py 
ros2 param get /camera/camera align_depth.enable
ros2 param get /camera/camera pointcloud__neon_.enable
ros2 param get /camera/camera align_depth.enable
ros2 param get /camera/camera pointcloud__neon_.enable
ros2 param get /camera/camera align_depth.enable
ros2 param get /camera/camera pointcloud__neon_.enable
ros2 param Set /camera/camera pointcloud__neon_.enable true
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 param get /camera/camera pointcloud__neon_.enable
ros2 run tf2_tools view_frames
rviz2
. ros2_ws/install/setup.bash
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cat commands.txt 
ls
cat new_script.sh 
vim stream.sh
ls
chmod +x stream.sh 
./stream.sh
cat stream.sh 
vim stream.sh
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-spatio-temporal-voxel-layer
ros2 topic list
ros2 topic list | grep points
rviz2
ros2 topic list
rviz2
ros2 topic list
ros2 run tf2_tools view_frames
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
cat commands.txt 
vim commands.txt 
. ros2_ws/install/setup.bash
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cat ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws/
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
cat slam/launch/slam_launch.py 
:q
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
cat slam/launch/slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
ros2 launch realsense2_camera rs_launch.py   align_depth.enable:=True   pointcloud__neon_.enable:=True   enable_gyro:=True   enable_accel:=True   unite_imu_method:=2   enable_rgbd:=True   enable_depth:=True   enable_color:=True
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
cd config
ls
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
cat slam/launch/slam_launch.py 
. ros2_ws/install/setup.bash
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cd ros2_ws/
ros2 launch slam slam_launch.py 
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py 
. ros2_ws/install/setup.bash
cat ros2_ws/slam/launch/slam_launch.py 
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
. ros2_ws/install/setup.bash
history | hrep github
history | grep github
ros2 topic list | grep points
ros2 topic list 
ros2 topic list | grep points
ros2 topic list 
ros2 param get /camera/camera pointcloud__neon_.enable
history | grep pointcloud__neon_
ros2 param Set /camera/camera pointcloud__neon_.enable true
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 topic list | grep points
vim commands.txt 
ros2 topic list | grep costmap
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 topic echo /camera/camera/depth/color/points
rviz2
cat commands.txt 
ros2 param set /camera/camera pointcloud__neon_.enable true
. ros2_ws/install/setup.bash
cd ros2_ws/
ls
cd install/
ls
cd slam/
ls
cd share
ls
cd slam
ls
cd launch
ls
cd ..
cd config/
ls
cat nav2_params.yaml 
vim stvl_params.yaml
ros2 topic list | grep costmap
ros2 topic list 
ros2 topic list | grep costmap
ros2 node list
cat nav2_params.yaml 
vim nav2_params.yaml 
cat nav2_params.yaml 
vim nav2_params.yaml 
cat nav2_params.yaml 
VIM nav2_params.yaml 
vim nav2_params.yaml 
cat nav2_params.yaml 
vim nav2_params.yaml 
cat nav2_params.yaml 
vim nav2_params.yaml 
cat nav2_params.yaml 
vim nav2_params.yaml 
cat ros2_ws/slam/launch/slam_launch.py 
vim the_yaml.yaml
rviz2
ros2 run tf2_tools view_frames
ros2 topic list | grep costmap
. ros2_ws/install/setup.bash
ros2 run nav2_controller controller_server --ros-args --params-file /ros2_ws/install/slam/share/slam/config/nav2_params.yaml
ros2 pkg executables nav2_costmap_2d
ccd ~/
cd ~/
. ros2_ws/install/setup.bash
ros2 lifecycle get /local_costmap
rviz2
ros2 topic list
sudo apt install ros-humble-nav2-dwb-controller
cat ros2_ws/slam/launch/slam_launch.py 
vim ros2_ws/slam/launch/slam_launch.py 
cat ros2_ws/slam/launch/slam_launch.py 
ros2 topic list
. ros2_ws/install/setup.bash
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cd ros2_ws/
ros2 launch slam slam_launch.py
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cat commands.txt 
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
cat ros2_ws/slam/launch/slam_launch.py 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 topic list
ros2 lifecycle set /local_setup configure
ros2 node list
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 node list
ros2 pkg list | grep spatio
ros2 lifecycle nodes
ros2 param set /camera/camera pointcloud__neon_.enable true
. ros2_ws/install/setup.bash
cd ros2_ws/
ls
cd install/
ls
cd slam
lsa
ls
cd share
ls
cd slam
ls
cd config/
ls
cat stvl_params.yaml 
cat nav2_params.yaml 
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 topic list | grep costmap
ros2 run tf2_tools view_frames
ros2 topic echo /tf | grep camera_link
ros2 run tf2_tools view_frames
ros2 topic list | grep costmap
cd ~/
. ros2_ws/install/setup.bash
rviz2
ros2 topic list
cat commands.txt 
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
rviz2
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
vim commands.txt 
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros static_transform_publisher -0.068 0.025 0.014 0.126 0.110 0.417 base_link camera_link
ros2 run tf2_ros static_transform_publisher -0.068 0.025 0.014 0.050 0.071 0.202 0.975 base_link camera_link
cat commands.txt 
vim commands.txt 
ros2 run tf2_ros static_transform_publisher -0.068 0.025 0.014 0.126 0.110 0.417 base_link camera_link
. ros2_ws/install/setup.bash
cd ros2_ws/
ls
cat slam/launch/slam_launch.py 
cd slm
ls
cd slam
ls
cd src
ls
cd ..
ls
cd realsense-ros/
ls
ros2 run tf2_ros tf2_echo base_link camera_link
rviz2
cd ~/
cat ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws/install/slam/
ls
cat share
ls
cd share
ls
cd slam
ls
cat config/stvl_params.yaml 
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo odom base_link
rviz2
rviz2 
. ros2_ws/install/setup.bash
cat ros2_ws/slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
gst-launch-1.0 v4l2-source device=/dev/video0
vim ros2_ws/slam/launch/slam_launch.py 
ros2 launch realsense2_camera rs_launch.py
realsense-viewer 
. ros2_ws/install/setup.bash
ros2 topic list
ros2 run tf2_tools view_frames
cat ros2_ws/install/slam/share/slam/config/stvl_params.yaml 
ros2 topic list --verbose
ros2 param dump /costmap/costmap
. ros2_ws/install/setup.bash
rviz2
cat commands.txt 
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 run tf2_ros static_transform_publisher -0.068 0.025 0.014 0.126 0.110 0.417 base_link camera_link
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 lifecycle set /costmap/costmap configure
ros2 lifecycle set /costmap/costmap activate
ros2 run tf2_tools view_frames
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 lifecycle set /costmap/costmap configure
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 param set /camera/camera pointcloud__neon_.enable true
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames 
ros2 lifecycle set /costmap/costmap activate
. ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
ros2 topic list
cat ros2_ws/slam/launch/slam_launch.py 
cd ros2_ws/install/
ls
cd slam/
ls
cd share
ls
cd slam
ls
cd config/
ls
cat stvl_params.yaml 
cd ~/
. ros2_ws/install/setup.bash
cat commands.txt 
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/gst_bridge/lib/gst_bridge
cd ros2_ws/
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
cat install/slam/share/slam/config/stvl_params.yaml 
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
cat slam/launch/slam_launch.py 
ros2 launch slam slam_launch.py
vim slam/launch/slam_launch.py 
colcon build --packages-up-to slam
ros2 launch slam slam_launch.py
pkill firefox
ps aux | grep -i firefox
pkill 8652
ps aux | grep -i firefox
pkill 40965
ps aux | grep -i firefo
sudo reboot
journalctl -ke
top
snap
snap remove firefox
sudo firefox
firefox
sudo pat remove firefox
sudo apt remove firefox
snap remove firefox
reboot
sudo snap remove firefox
sudo apt install firefox
sudo apt update
firefox
sudo apt upgrade
firefox
sudo apt purge firefox
sudo apt install firefox
firefox
which firefox
sudo snap remove firefox
sudo apt remove firefox
sudo apt purge firefox
wget https://download.mozilla.org/?product=firefox-latest&os=linux64&lang=en-US -O firefox.tar.bz2
ls
cd dow
cd Downloads/
l
ls
ls -all
cd ..
ls
cd Downloads/
ls
wget -O firefox-latest.tar.bz2 "htpps://download.mozilla.org/?product=firefox-latest&os=linux64&lang=en-US"
wget -O firefox-latest.tar.bz2 "https://download.mozilla.org/?product=firefox-latest&os=linux64&lang=en-US"
tar xjf firefox-latest.tar.bz2 
nemo
sudo mv firefox /opt/firefox
sudo ln -s /opt/firefox/firefox/ /usr/local/bin/firefox
firefox
ls
which firefox
ls -l /usr/local/bin/firefox
exit
sudo mv firefox/ /opt/firefox
echo $PATH
sudo chmod +x /opt/firefox/firefox/
firefox
/opt/firefox/firefox/
ls/opt/firefox/firefox/
ls /opt/firefox/firefox/
ls -all
file /opt/firefox/firefox/
sudo rm usr/local/bin/firefox
sudo ln -s /opt/firefox/firefox/firefox /usr/local/bin/firefox
ls -l /usr/local/bin | grep firefox
;s
ls
sudo rm -rf /usr/local/bin/fiefox
sudo ln -s /opt/firefox/firefox/firefox /usr/local/bin/firefox
sudo rm -rf /opt/firefox/
sudo rm -rf /usr/local/bin/fiefox
sudo rm -rf /usr/local/bin/fifefox
sudo rm -rf /usr/local/bin/firefox
sudo ln -s /opt/firefox/firefox /usr/local/bin/firefox
firefox 
sudo chmod +x /opt/firefox/firefox
firefox 
uname -m
file /opt/firefox/firefox
sudo rm -rf /opt/firefox/
sudo apt install firefox-esr
sudo apt install chromium-browser
chromium
sudo which chromium-browser 
sudo apt install policycoreutils
chromium
chromium-browser 
chromium-browser --no-sandbox
sudo apt install nvidia-jetson-chromium
sudo apt remove chromium-browser
w3m flathub.org
flatpak
ls
firefox
sudo apt install firefox
sudo apt autoremove
sudo apt purge firefox
sudo apt install firefox
journalctl -xe
curl https://google.com/search?q=firefox%20apt%20snap%20issue
curl https://google.com/search?client=firefox-b-lm&q=firefox%20apt%20snap%20issue
curl https://google.com/search?client=firefox-b-lm&q=firefox
ls
snap
sudo snap remove firefox
snap help
snap help --all
snap remove -h
snap remove --purge firefox
snap install firefox
umount
umount ls
mount ls
umount -a
umount -a | grep firefox
snap install firefox
journalctl -xe
ls
umount /snap/firefox/7965
snap version
snap update
snap upgrade
snap help
cd /etc/systemd/system
ls
ls | grep fire
cat snapd.mounts.target.wants/
systemctl status snad
systemctl status snapd
systemctl status snapd.socket
sudo snap debug sandbox-features
snap install firefox
sudo apt purge snap
snap
sudo apt purge snapd
snapd
ls
cd ~
ls
sudo apt install firefox
sudo rm -rf ~/snap/
sudo apt install snapd
sudo apt-get update
sudo apt update
sudo apt install firefox
sudo apt install snapd
sudo apt install snapd --fix-missing
sudo apt update
sudo apt install snapd --fix-missing
sudo apt install firefox
apt-get update
sudo apt-get update
apt-get update
sudo apt install firefox
sudo apt install snapd --fix-missing
sudo add-apt-repository ppa:mozillateam/ppa
sudo apt install firefox
sudo rm -rf /var/lib/apt/lists/*
sudo apt update
sudo apt install firefox
sudo add-apt-repository ppa:mozillateam/ppa
sudo apt-get udpate
sudo apt-get udpdte
sudo apt-get update
sudo apt search firefox
sudo apt search firefox | grep mozilla
sudo apt search firefox | grep ^firefox
w43m
w3m
links
elinks
lynhx
lynx
sudo apt install w3m
w3m duck.com
echo '
Package: *
Pin: origin packages.mozilla.org
Pin-Priority: 1000

Package: firefox*
Pin: release o=Ubuntu
Pin-Priority: -1' | sudo tee /etc/apt/preferences.d/mozilla
sudo apt install firefox
ls
sudo apt install gh
type -p wget >/dev/null || sudo apt install wget -y
sudo mkdir -p -m 755 /etc/apt/keyrings
out=$(mktemp) && wget -nv -O$out https://cli.github.com/packages/githubcli-archive-keyring.gpg
cat $out | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg 
sudo mkdir -p -m 755 /etc/apt/sources.list.d
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
sudo apt update
sudo apt install gh -y
cat /etc/apt/preferences.d/mozilla 
top
git add .
ls -l
git config user.name
git config user.email
git config --list
git config --global user.email
git config --global user.name
git config --global user.name cydobia
git config --global user.name cydonia
git config --global user.email cydonia
git config --global user.email test@gmail.com
git add .
ls -l ~
git config --global --add safe.directory /home/cydonia/ros2_ws/
git add .
git config --global --add safe.directory /home/cydonia/ros2_ws
git add .
sudo git add .
git status
poweroff 
sudo poweroff 
