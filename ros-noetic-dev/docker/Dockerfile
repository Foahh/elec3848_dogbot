FROM foahh/ros-noetic-slam-dev

ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y vim udev && \
    rm -rf /var/lib/apt/lists/*

ARG USER_ID=1000
ARG GROUP_ID=1000

ENV USERNAME ros
RUN adduser --disabled-password --gecos '' $USERNAME \
    && usermod  --uid ${USER_ID} $USERNAME \
    && groupmod --gid ${GROUP_ID} $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $USERNAME

RUN sudo apt-get update \
    && rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc 

COPY --chown=${USER_ID}:${GROUP_ID} --chmod=760 ydlidar_build.sh /opt/src/scripts/ydlidar_build.sh
RUN /opt/src/scripts/ydlidar_build.sh

RUN mkdir -p /home/$USERNAME/workspace
WORKDIR /home/$USERNAME/workspace