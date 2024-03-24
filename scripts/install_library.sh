#!/bin/bash
apt-get update 

# Build dependencies
apt-get install -y \
    libserial-dev

# ROS2 misc libraries
apt-get install -y \
    ros-dev-tools \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-ign-bridge \
    ros-humble-joint-state-publisher-gui

# ROS2 control
apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-control-msgs \
    ros-humble-ros2-controllers-test-nodes \

rm -rf /var/lib/apt/lists/*