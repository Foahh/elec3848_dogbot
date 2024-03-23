FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    libserial-dev \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-control-msgs \
    ros-humble-ros2-controllers-test-nodes \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    vim \
    udev \
    wget \
    ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]
ENV TERM=xterm-256color
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc

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
    sudo apt-get upgrade -y && \
    sudo rm -rf /var/lib/apt/lists/*

COPY --chown=${USER_ID}:${GROUP_ID} --chmod=760 ./scripts/build.sh /opt/src/scripts/build.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    /opt/src/scripts/build.sh

RUN mkdir -p /home/$USERNAME/colcon_ws
WORKDIR /home/$USERNAME/colcon_ws