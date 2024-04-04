ARG FROM_IMAGE=osrf/ros:humble-desktop-full
FROM $FROM_IMAGE
SHELL ["/bin/bash", "-c"]

RUN --mount=type=bind,source="./scripts/install.sh",target="/tmp/install.sh" apt-get update && \
    bash /tmp/install.sh && \
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
    sudo rosdep init && rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    sudo apt-get upgrade -y

RUN --mount=type=bind,source="./scripts/build.sh",target="/tmp/build.sh" source /opt/ros/${ROS_DISTRO}/setup.bash && \
    bash /tmp/build.sh

WORKDIR /home/$USERNAME
