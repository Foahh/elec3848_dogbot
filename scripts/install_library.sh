#!/bin/bash
apt-get update 
apt-get install -y \
    libserial-dev \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-control-msgs \
    ros-humble-ros2-controllers-test-nodes \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros-ign-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-dev-tools
rm -rf /var/lib/apt/lists/*