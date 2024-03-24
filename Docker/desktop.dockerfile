FROM $TARGETARCH/ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-perception && \
    rm -rf /var/lib/apt/lists/*