FROM foahh/ros-desktop-arm64v8:humble
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    vim \
    udev \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

ARG USER_ID=1000
ARG GROUP_ID=1000

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
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/$USERNAME/ydlidar_ws/src/ydlidar_ros2_driver && \
    cd /home/$USERNAME/ydlidar_ws && \
    colcon build --symlink-install && \
    echo "source ~/ydlidar_ws/devel/setup.bash --extend" >> /home/$USERNAME/.bashrc

RUN mkdir -p /home/$USERNAME/colcon_ws
WORKDIR /home/$USERNAME/colcon_ws