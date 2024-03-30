ARG ROS_DISTRO
FROM arm64v8/ros:${ROS_DISTRO}-ros-base
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop-full && \
    rm -rf /var/lib/apt/lists/*