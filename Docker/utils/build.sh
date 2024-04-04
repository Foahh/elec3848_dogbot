#!/bin/bash

# serial
git clone https://github.com/RoverRobotics-forks/serial-ros2.git ~/src/serial_ros2
cd ~/src/serial_ros2 || exit 1
mkdir build
cd build || exit 1
cmake .. -DBUILD_SHARED_LIBS=ON && make
sudo make install

# YDLiDar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/src/YDLidar-SDK
cd ~/src/YDLidar-SDK || exit 1
mkdir build
cd build || exit 1
cmake ..
make -j4
sudo make install

# YDLiDar ROS2 Driver
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git ~/src/ydlidar_ws/src/ydlidar_ros2_driver

cd ~/src/ydlidar_ws || exit 1
colcon build --symlink-install
echo "source ~/src/ydlidar_ws/install/setup.bash --extend" >>~/.bashrc
