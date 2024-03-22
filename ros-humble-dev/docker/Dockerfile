FROM infoahh/ros-desktop-armv8:humble
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

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
    && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> /home/$USERNAME/.bashrc