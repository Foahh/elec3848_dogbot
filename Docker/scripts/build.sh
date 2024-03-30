#!/bin/bash

# serial
git clone https://github.com/RoverRobotics-forks/serial-ros2.git /home/"$USERNAME"/serial
cd /home/"$USERNAME"/serial || exit 1
mkdir build
cd build || exit 1
cmake .. -DBUILD_SHARED_LIBS=ON && make
sudo make install

# YDLiDar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git /home/"$USERNAME"/YDLidar-SDK
cd /home/"$USERNAME"/YDLidar-SDK || exit 1
mkdir build
cd build || exit 1
cmake ..
make -j4
sudo make install

# YDLiDar ROS2 Driver
if [ "$ROS_DISTRO" == "humble" ] || [ "$ROS_DISTRO" == "iron" ]; then
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/"$USERNAME"/ydlidar_ws/src/ydlidar_ros2_driver
else
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/"$USERNAME"/ydlidar_ws/src/ydlidar_ros2_driver
fi

cd /home/"$USERNAME"/ydlidar_ws || exit 1
colcon build --symlink-install
echo "source ~/ydlidar_ws/install/setup.bash --extend" >>~/.bashrc
