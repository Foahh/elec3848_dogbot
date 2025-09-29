#!/bin/bash

# serial
git clone https://github.com/RoverRobotics-forks/serial-ros2.git /build_ws/serial_ros2
cd /build_ws/serial_ros2 || exit 1
mkdir build
cd build || exit 1
cmake .. -DBUILD_SHARED_LIBS=ON && make
sudo make install

# YDLiDar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git /build_ws/YDLidar-SDK
cd /build_ws/YDLidar-SDK || exit 1
mkdir build
cd build || exit 1
cmake ..
make -j4
sudo make install

# YDLiDar ROS2 Driver
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /build_ws/ydlidar_ws/src/ydlidar_ros2_driver
cd /build_ws/ydlidar_ws || exit 1
colcon build --symlink-install
echo "source /build_ws/ydlidar_ws/install/setup.bash --extend" >> ~/.bashrc