#!/bin/bash

# Base tools
apt-get update && apt-get install -y \
     vim \
     wget \
     python3-pip \
     python3-venv \
     lsb-release \
     gnupg

# PlatformIO
cd /home/"$USERNAME"/ || exit 1
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
sudo python3 get-platformio.py
rm get-platformio.py

# Gazebo
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
apt-get update
apt-get install -y ignition-fortress

# ros-gz
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt-get install -y ros-"$ROS_DISTRO"-ros-gz
rm -rf /var/lib/apt/lists/*