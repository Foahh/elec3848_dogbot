FROM foahh/ros-desktop-arm64v8:noetic

RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-moveit \
    ros-noetic-moveit-visual-tools  \
    ros-noetic-kdl-*  \
    ros-noetic-joint-state-publisher-gui  \
    ros-noetic-trac-ik  \
    liborocos-kdl-dev  \
    ros-noetic-teleop-twist-keyboard  \
    ros-noetic-moveit-resources  \
    ros-noetic-navigation  \
    ros-noetic-gmapping  \
    ros-noetic-hector-slam  \
    ros-noetic-slam-karto  \
    ros-noetic-robot-state-publisher  \
    ros-noetic-geographic-msgs  \
    ros-noetic-libuvc-*  \
    ros-noetic-rtabmap-ros \
    libavformat-dev  \
    libavcodec-dev  \
    libswresample-dev  \
    libswscale-dev  \
    libavutil-dev  \
    libsdl1.2-dev  \
    ros-noetic-pointcloud-to-laserscan  \
    ros-noetic-mbf-msgs  \
    ros-noetic-mbf-costmap-core  \
    ros-noetic-costmap-converter  \
    liborocos-bfl-dev  \
    ros-noetic-serial  \
    ros-noetic-teleop-twist-joy  \
    ros-noetic-laser-proc  \
    ros-noetic-rosserial-arduino  \
    ros-noetic-rosserial-python  \
    ros-noetic-rosserial-server  \
    ros-noetic-rosserial-client  \
    ros-noetic-rosserial-msgs  \
    ros-noetic-amcl  \
    ros-noetic-map-server  \
    ros-noetic-urdf  \
    ros-noetic-xacro  \
    ros-noetic-interactive-markers  \
    ros-noetic-octomap*  \
    ros-noetic-joy  \
    ros-noetic-dwa-local-planner  \
    ros-noetic-multirobot-map-merge  \
    python3-catkin-tools  \
    python3-dev  \
    python3-catkin-pkg-modules  \
    python3-numpy  \
    python3-yaml  \
    build-essential  \
    # ros-noetic-cartographer* \ 
    ros-noetic-imu-tools  && \ 
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y && \
    vim && \
    udev && \
    rm -rf /var/lib/apt/lists/*

ARG USER_ID=1000
ARG GROUP_ID=1000

ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

ENV USERNAME ros
RUN adduser --disabled-password --gecos '' $USERNAME && \
    usermod  --uid ${USER_ID} $USERNAME && \
    groupmod --gid ${GROUP_ID} $USERNAME && \
    usermod --shell /bin/bash $USERNAME && \
    adduser $USERNAME sudo && \
    adduser $USERNAME dialout && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $USERNAME

RUN sudo apt-get update && \
    rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    sudo rm -rf /var/lib/apt/lists/*

COPY --chown=${USER_ID}:${GROUP_ID} --chmod=760 ydlidar_build.sh /opt/src/scripts/ydlidar_build.sh
RUN /opt/src/scripts/ydlidar_build.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git /home/$USERNAME/ydlidar_ws/src/ydlidar_ros_driver && \
    cd /home/$USERNAME/ydlidar_ws/src && \
    catkin_init_workspace && \
    cd /home/$USERNAME/ydlidar_ws && \
    catkin_make && \
    echo "source ~/ydlidar_ws/devel/setup.bash --extend" >> /home/$USERNAME/.bashrc

RUN mkdir -p /home/$USERNAME/catkin_ws
WORKDIR /home/$USERNAME/catkin_ws