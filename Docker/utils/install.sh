#!/bin/bash

sudo apt-get update

# tools
sudo apt-get install -y \
     vim \
     python3-pip

pip install icm20948

# ros-related packages
sudo apt-get install -y \
    ros-dev-tools \
    ros-"$ROS_DISTRO"-robot-localization \
    ros-"$ROS_DISTRO"-imu-tools \
    ros-"$ROS_DISTRO"-urdf \
    ros-"$ROS_DISTRO"-slam-toolbox \
    ros-"$ROS_DISTRO"-ros2-control \
    ros-"$ROS_DISTRO"-ros2-controllers \
    ros-"$ROS_DISTRO"-control-toolbox \
    ros-"$ROS_DISTRO"-realtime-tools \
    ros-"$ROS_DISTRO"-control-msgs \
    ros-"$ROS_DISTRO"-ros2-controllers-test-nodes \
    ros-"$ROS_DISTRO"-joint-state-publisher-gui

sudo rm -rf /var/lib/apt/lists/*
