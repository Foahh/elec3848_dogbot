#!/bin/bash
apt-get update
apt-get install -y --no-install-recommends ros-"$ROS_DISTRO"-desktop-full
rm -rf /var/lib/apt/lists/*