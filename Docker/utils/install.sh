#!/bin/bash

sudo apt-get update

# tools
sudo apt-get install -y python3-pip && \
    libi2c-dev && \
    i2c-tools && \
sudo pip install icm20948

# ros-related packages
sudo apt-get install -y \
    ros-dev-tools \
    ros-"$ROS_DISTRO"-robot-localization \
    ros-"$ROS_DISTRO"-imu-tools \
    ros-"$ROS_DISTRO"-urdf \
    ros-"$ROS_DISTRO"-slam-toolbox \
    ros-"$ROS_DISTRO"-ros2-control \
    ros-"$ROS_DISTRO"-ros2-controllers \
    ros-"$ROS_DISTRO"-realtime-tools \
    ros-"$ROS_DISTRO"-control-msgs \
    ros-"$ROS_DISTRO"-joint-state-publisher-gui \
    ros-"$ROS_DISTRO"-navigation2 \
    ros-"$ROS_DISTRO"-nav2-bringup

sudo rm -rf /var/lib/apt/lists/*
