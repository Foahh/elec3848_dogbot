#!/bin/bash
ROS_DISTRO=humble

if [ "$(uname -m)" == "x86_64" ]; then
    export FROM_IMAGE=osrf/ros:${ROS_DISTRO}-desktop-full
elif [ "$(uname -m)" == "aarch64" ]; then
    docker build -t foahh/arm64v8-ros-desktop:${ROS_DISTRO} --build-arg ROS_DISTRO=${ROS_DISTRO} -f ./Docker/arm64v8.dockerfile .
    export FROM_IMAGE=foahh/arm64v8-ros-desktop:${ROS_DISTRO}
else
    echo "Unsupported architecture: $(uname -m)"
    exit 1
fi

docker compose build || exit
docker compose up -d
