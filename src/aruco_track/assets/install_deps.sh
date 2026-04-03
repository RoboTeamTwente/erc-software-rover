#!/bin/bash
set -e

echo "=== Installing system dependencies ==="
sudo apt-get update && sudo apt-get install -y \
    ros-humble-desktop \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    build-essential cmake ninja-build git \
    protobuf-compiler libprotobuf-dev \
    libcurl4-openssl-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libeigen3-dev \
    libgtk-3-dev libpng-dev libjpeg-dev libtiff-dev

echo "=== Checking OpenCV version ==="
OPENCV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "none")

if [[ "$OPENCV_VERSION" == 4.8* ]]; then
    echo "OpenCV 4.8 already installed, skipping build."
else
    echo "=== Building OpenCV 4.8 from source (this takes ~20-30 minutes) ==="
    git clone --depth 1 --branch 4.8.0 https://github.com/opencv/opencv.git /tmp/opencv
    git clone --depth 1 --branch 4.8.0 https://github.com/opencv/opencv_contrib.git /tmp/opencv_contrib

    cmake -S /tmp/opencv -B /tmp/opencv/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF

    cmake --build /tmp/opencv/build -j$(nproc)
    sudo cmake --install /tmp/opencv/build
    rm -rf /tmp/opencv /tmp/opencv_contrib
    echo "=== OpenCV 4.8 installed ==="
fi

echo "=== Updating rosdep ==="
rosdep update || true

echo "=== Done! Now run: ==="
echo "  source /opt/ros/humble/setup.bash"
echo "  cd <workspace_root>"
echo "  git submodule update --init --recursive"
echo "  colcon build"