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