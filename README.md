# ERC, software part

## Getting started

First, you need to clone this repo. The instructions are a bit convoluted, because we use git submodules:

```bash
git clone https://github.com/RoboTeamTwente/erc-software-rover
git submodule update --init --recursive
```

Now, pick your poison to install ROS2:

### Option 1: Docker

```bash
# TODO: install buildx
sudo docker build . -t erc-software-rover

# Run this in a separate terminal. If all is correct, it will stay silent forever.
sudo docker run --rm --name erc-software-rover --net=host -v $(pwd):/ws --privileged -e DISPLAY -v /tmp:/tmp -v /run:/run -v /dev:/dev erc-software-rover

# In a new terminal, to launch a new shell:
sudo docker exec -it erc-software-rover bash
source /opt/ros/humble/setup.bash
```

### Option 2: WSL

```bash
# Install Ubuntu
wsl --install -d Ubuntu-22.04
wsl

# add ROS2 packages source to APT
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# install ROS2
sudo apt update
sudo apt install ros-dev-tools ros-humble-desktop

# activate ROS2
source /opt/ros/humble/setup.bash

# install colcon mixins
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# initialize rosdep
sudo rosdep init
```

### Working with the project

#### Compile everything

```bash
# activate ROS2 (do this every time you open a terminal)
source /opt/ros/humble/setup.bash

# install dependencies
rosdep update
rosdep install --from-paths --ignore-src --default-yes src

# librealsense2 must be built with different CMake options (only needs to be done once)
colcon build --packages-select librealsense2 --cmake-args \
  -DCHECK_FOR_UPDATES=OFF                                 \
  -DFORCE_RSUSB_BACKEND=ON

# actually build, if your computer is low on RAM then use --executor sequential
colcon build

# activate the project
source install/local_setup.bash
```

#### Run the simulation

```bash
# activate ROS2
source install/setup.bash

# Enable GPU (only if you run WSL2)
export GALLIUM_DRIVER=d3d12

# You might or might not need to set this variable,
# it switches between integrated (Intel) and discrete (NVIDIA) GPU.
# Pick whatever makes it run faster.
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# launch the simulation
ros2 launch rtt_simulation simulation.launch.py
```
