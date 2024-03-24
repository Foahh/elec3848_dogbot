ARG ROS_ARCH=$TARGETARCH
FROM foahh/ros-humble-desktop:${ROS_ARCH}
SHELL ["/bin/bash", "-c"]

RUN --mount=type=bind,source="./scripts/install_library.sh",target="/tmp/install_library.sh" bash /tmp/install_library.sh
RUN --mount=type=bind,source="./scripts/install_tool.sh",target="/tmp/install_tool.sh" bash /tmp/install_tool.sh

ARG USER_ID=1000
ARG GROUP_ID=1000
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
RUN --mount=type=bind,source="./scripts/build_package.sh",target="/tmp/build_package.sh" bash /tmp/build_package.sh

WORKDIR /home/$USERNAME