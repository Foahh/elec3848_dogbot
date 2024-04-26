FROM arm64v8/ros:humble-ros-base
SHELL ["/bin/bash", "-c"]


RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-robot-localization && \
    rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-joint-state-publisher && \
    rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-dev-tools \
    python3-pip \
    vim && \
    rm -rf /var/lib/apt/lists/*

RUN pip install \
    icm20948 \
    mpu6050-raspberrypi \
    transforms3d

RUN sudo apt-get update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    sudo apt-get upgrade -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source="./utils/build.sh",target="/tmp/build.sh" source /opt/ros/${ROS_DISTRO}/setup.bash && bash /tmp/build.sh
WORKDIR /colcon_ws
