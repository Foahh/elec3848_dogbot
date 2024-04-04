#!/bin/bash

# serial
git clone https://github.com/RoverRobotics-forks/serial-ros2.git /home/"$USERNAME"/ros_setup/serial
cd /home/"$USERNAME"/ros_setup/serial || exit 1
mkdir build
cd build || exit 1
cmake .. -DBUILD_SHARED_LIBS=ON && make
sudo make install

# YDLiDar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git /home/"$USERNAME"/ros_setup/YDLidar-SDK
cd /home/"$USERNAME"/ros_setup/YDLidar-SDK || exit 1
mkdir build
cd build || exit 1
cmake ..
make -j4
sudo make install

# YDLiDar ROS2 Driver
if [ "$ROS_DISTRO" == "humble" ] || [ "$ROS_DISTRO" == "iron" ]; then
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/"$USERNAME"/ros_setup/ydlidar_ws/src/ydlidar_ros2_driver
else
    git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/"$USERNAME"/ros_setup/ydlidar_ws/src/ydlidar_ros2_driver
fi

cd /home/"$USERNAME"/ros_setup/ydlidar_ws || exit 1
colcon build --symlink-install
echo "source ~/ros_setup/ydlidar_ws/install/setup.bash --extend" >>~/.bashrc
