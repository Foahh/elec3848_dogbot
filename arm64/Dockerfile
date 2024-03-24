FROM arm64v8/ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc

ARG USER_ID=1000
ARG GROUP_ID=1000

COPY --chown=${USER_ID}:${GROUP_ID} --chmod=760 ./scripts/ /opt/src/scripts/
RUN /opt/src/scripts/install_desktop.sh && rm /opt/src/scripts/install_desktop.sh
RUN /opt/src/scripts/install_library.sh && rm /opt/src/scripts/install_library.sh
RUN /opt/src/scripts/install_tool.sh && rm /opt/src/scripts/install_tool.sh
RUN /opt/src/scripts/install_platformio.sh && rm /opt/src/scripts/install_platformio.sh

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

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    /opt/src/scripts/build.sh && \
    sudo rm -rf /opt/src/scripts

RUN mkdir -p /home/$USERNAME/colcon_ws
WORKDIR /home/$USERNAME/colcon_ws