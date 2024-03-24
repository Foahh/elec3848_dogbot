#!/bin/bash

# YDLiDar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git /home/"$USERNAME"/YDLidar-SDK
cd /home/"$USERNAME"/YDLidar-SDK || exit 1
mkdir build
cd build || exit
cmake ..
make -j4
sudo make install

# YDLiDar ROS2 Driver
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/"$USERNAME"/ydlidar_ws/src/ydlidar_ros2_driver
cd /home/"$USERNAME"/ydlidar_ws || exit 1
colcon build --symlink-install
echo "source ~/ydlidar_ws/install/setup.bash --extend" >> ~/.bashrc