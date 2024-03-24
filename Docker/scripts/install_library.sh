#!/bin/bash

# Build dependencies
apt-get update && \
apt-get install -y \
    libserial-dev

# ros packages
apt-get install -y \
    ros-dev-tools \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs

# ros control
apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-control-msgs \
    ros-humble-ros2-controllers-test-nodes \

rm -rf /var/lib/apt/lists/*
