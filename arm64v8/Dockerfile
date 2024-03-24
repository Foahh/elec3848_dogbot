FROM arm64v8/ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]
RUN echo "PS1='\e[92m\u\e[0m@\e[94m\h\e[0m:\e[35m\w\e[0m# '" >> /root/.bashrc

ARG USER_ID=1000
ARG GROUP_ID=1000

COPY --chmod=700 ./scripts/install_library.sh /tmp/install_desktop.sh
RUN /tmp/install_desktop.sh && \
    rm /tmp/install_desktop.sh

COPY --chmod=700 ./scripts/install_library.sh /tmp/install_library.sh
RUN /tmp/install_library.sh && \
    rm /tmp/install_library.sh

COPY --chmod=700 ./scripts/install_tool.sh /tmp/install_tool.sh
RUN bash /tmp/install_tool.sh && \
    rm /tmp/install_tool.sh

ENV USERNAME humble
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

COPY --chown=${USER_ID}:${GROUP_ID} --chmod=700 ./scripts/build_package.sh /tmp/build_package.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    /tmp/build_package.sh && \
    sudo rm /tmp/build_package.sh

RUN mkdir -p /home/$USERNAME/colcon_ws
WORKDIR /home/$USERNAME/colcon_ws