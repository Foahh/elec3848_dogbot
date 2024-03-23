#!/bin/bash

#PlatformIO
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm get-platformio.py

# YDLiDar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git /home/$USERNAME/YDLidar-SDK
cd /home/$USERNAME/YDLidar-SDK
mkdir build
cd build
cmake ..
make -j4
sudo make install

# YDLiDar ROS2 Driver
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/$USERNAME/ydlidar_ws/src/ydlidar_ros2_driver
cd /home/$USERNAME/ydlidar_ws
colcon build --symlink-install
echo "source ~/ydlidar_ws/devel/setup.bash --extend" >> /home/$USERNAME/.bashrc