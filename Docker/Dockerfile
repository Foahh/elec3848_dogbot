FROM arm64v8/ros:humble-ros-base AS humble-dev
SHELL ["/bin/bash", "-c"]

RUN sudo apt-get update && sudo apt-get install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update && sudo apt-get install -y \
    python3-pip \
    libi2c-dev \
    i2c-tools && \ 
    rm -rf /var/lib/apt/lists/* && \
    sudo pip install icm20948

FROM humble-dev

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG GPIO_ID=999
ARG I2C_ID=108
ENV USERNAME turtle
RUN groupadd -f -r -g $GPIO_ID gpio
RUN groupadd -f -r -g $I2C_ID i2c
RUN adduser --disabled-password --gecos '' $USERNAME && \
    usermod  --uid ${USER_ID} $USERNAME && \
    groupmod --gid ${GROUP_ID} $USERNAME && \
    usermod --shell /bin/bash $USERNAME && \
    adduser $USERNAME sudo && \
    adduser $USERNAME dialout && \
    adduser $USERNAME i2c && \
    adduser $USERNAME gpio && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    echo 'Defaults exempt_group+=user' >> /etc/sudoers.d/user && \
    chmod a=r,o= /etc/sudoers.d/user
USER $USERNAME

RUN sudo apt-get update && \
    rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    sudo apt-get upgrade -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,source="./utils/build.sh",target="/tmp/build.sh" source /opt/ros/${ROS_DISTRO}/setup.bash && bash /tmp/build.sh

WORKDIR /home/$USERNAME
